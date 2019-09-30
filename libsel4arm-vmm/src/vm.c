/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */
#include <autoconf.h>
#include <sel4arm-vmm/gen_config.h>
#include "vm.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <cpio/cpio.h>
#include <vka/object.h>
#include <vka/capops.h>
#include <string.h>
#include <sel4utils/mapping.h>
#include <utils/ansi.h>

#include <sel4/sel4.h>
#include <sel4/messages.h>

#include "arch/arm/devices/vgic.h"

#include "devices.h"
#include "sel4arm-vmm/guest_vspace.h"

#include <sel4arm-vmm/sel4_arch/vm.h>
#include <sel4arm-vmm/sel4_arch/fault.h>
#include <sel4arm-vmm/psci.h>
#include <sel4arm-vmm/smc.h>
#include <sel4vmmcore/util/io.h>

#define DEBUG_VM
//#define DEBUG_RAM_FAULTS
//#define DEBUG_DEV_FAULTS
//#define DEBUG_STRACE

#define VM_CSPACE_SIZE_BITS    4
#define VM_FAULT_EP_SLOT       1
#define VM_CSPACE_SLOT         VM_FAULT_EP_SLOT + CONFIG_MAX_NUM_NODES

#ifdef DEBUG_RAM_FAULTS
#define DRAMFAULT(...) printf(__VA_ARGS__)
#else
#define DRAMFAULT(...) do{}while(0)
#endif

#ifdef DEBUG_DEV_FAULTS
#define DDEVFAULT(...) printf(__VA_ARGS__)
#else
#define DDEVFAULT(...) do{}while(0)
#endif

#ifdef DEBUG_STRACE
#define DSTRACE(...) printf(__VA_ARGS__)
#else
#define DSTRACE(...) do{}while(0)
#endif

#ifdef DEBUG_VM
#define DVM(...) do{ printf(__VA_ARGS__);}while(0)
#else
#define DVM(...) do{}while(0)
#endif
#define HSR_EC_MASK     (0xfc000000)
#define HSR_EC_SHIFT    (26)
#define HSR_EC(x) (((x) & HSR_EC_MASK) >> HSR_EC_SHIFT)

/* HSR exception codes */
#define HSR_EC_UNKNOWN              0b000000
#define HSR_EC_WFI_WFE              0b000001
#define HSR_EC_MCR_MRC_CP15         0b000011
#define HSR_EC_MCRR_MRCC_CP15       0b000100
#define HSR_EC_MCR_MRC_CP14         0b000101
#define HSR_EC_LDC_STC              0b000110
#define HSR_EC_MRRC_CP14            0b001100
#define HSR_EC_ILLEGAL_ES           0b001110
#define HSR_EC_SVC32                0b010001
#define HSR_EC_HVC32                0b010010
#define HSR_EC_SMC32                0b010011
#define HSR_EC_SVC64                0b010101
#define HSR_EC_HVC64                0b010110
#define HSR_EC_SMC64                0b010111
#define HSR_EC_MSR_MRS_OTHER        0b011000
#define HSR_EC_INST_ABORT_LEL       0b100000
#define HSR_EC_INST_ABORT_CEL       0b100001
#define HSR_EC_PC_ALIGNMENT_FAULT   0b100010
#define HSR_EC_DATA_ABORT_LEL       0b100100
#define HSR_EC_DATA_ABORT_CEL       0b100101
#define HSR_EC_SP_ALIGNMENT_FAULT   0b100110
#define HSR_EC_SERROR_INT           0b101111
#define HSR_EC_BRK_LEL              0b110000
#define HSR_EC_BRK_CEL              0b110001
#define HSR_EC_SS_LEL               0b110010
#define HSR_EC_SS_CEL               0b110011
#define HSR_EC_WATCHPOINT_LEL       0b110100
#define HSR_EC_WATCHPIONT_CEL       0b110101
#define HSR_EC_BKPT32               0b111000
#define HSR_EC_VECTOR_CATCH32       0b111010
#define HSR_EC_BRK64                0b111100

#define HSR_EC_OP0(x) (((x) >> 20) & 0b11)
#define HSR_EC_OP1(x) (((x) >> 14) & 0b111)
#define HSR_EC_OP2(x) (((x) >> 17) & 0b111)
#define HSR_EC_CRN(x) (((x) >> 10) & 0b1111)
#define HSR_EC_CRM(x) (((x) >> 1)  & 0b1111)
#define HSR_EC_DIR(x) ((x) & 1)
#define HSR_EC_RT(x)  (((x) >> 5) & 0b11111) // 0 write; 1 read

#define MSR_MRS_OTHER(op0, op1, crn, crm, op2) \
    ((op0 << 20) | (op2 << 17) | (op1 << 14) | (crn << 10) | (crm << 1))
#define ICC_SGI1R_EL1  MSR_MRS_OTHER(3, 0, 12, 11, 5)
#define MSR_MRS_MASK   0x3FFC1E

extern char _cpio_archive[];
static virq_handle_t vtimer_irq_handle[CONFIG_MAX_NUM_NODES] = {0};
static virq_handle_t vcpu_sgi_ppi[CONFIG_MAX_NUM_NODES][32]; // TODO unmagic

extern int vmlinux_handle_monitored_address(struct device *d, vm_t *vm, fault_t *fault);

static int handle_page_fault(vm_t *vm, fault_t *fault)
{

    /* See if the device is already in our address space */
    uintptr_t addr = fault_get_address(fault);
    struct device *d = vm_find_device_by_ipa(vm, (uintptr_t) addr);
    uintptr_t pc = fault_get_ctx(fault)->pc;
    if (d != NULL) {
        if (d->devid == DEV_RAM) {
            DRAMFAULT("[%s] %s fault @ 0x%x from 0x%x\n", d->name,
                      (fault_is_read(fault)) ? "read" : "write",
                      fault_get_address(fault), fault_get_ctx(fault)->pc);
        } else {
            DDEVFAULT("[%s] %s fault @ 0x%x from 0x%x\n", d->name,
                      (fault_is_read(fault)) ? "read" : "write",
                      fault_get_address(fault), fault_get_ctx(fault)->pc);
        }
        return d->handle_page_fault(d, vm, fault);
    } else {
#ifdef CONFIG_ONDEMAND_DEVICE_INSTALL
        uintptr_t addr = fault_get_address(fault) & ~0xfff;
        if (vmlinux_handle_monitored_address(NULL, vm, fault) == 0) {
            return 0;
        }
        switch (addr) {
        case 0:
            print_fault(fault);
            return -1;
        default: {
            void *mapped = map_vm_device(vm, addr, addr, seL4_AllRights);
            if (mapped) {
              //  DVM("WARNING: Blindly mapped device @ %p for PC %p\n",
              //      (void *) fault_get_address(fault), (void *) fault_get_ctx(fault)->pc);
                restart_fault(fault);
                return 0;
            }
            mapped = map_vm_ram(vm, addr, seL4_PageBits);
            if (mapped) {
                DVM("WARNING: Mapped RAM for device @ %p for PC %p\n",
                    (void *) fault_get_address(fault), (void *) fault_get_ctx(fault)->pc);
                restart_fault(fault);
                return 0;
            }
            DVM("Unhandled fault on address 0x%x\n", (uint32_t)addr);
        }
        }
#endif
        print_fault(fault);
        abandon_fault(fault);
        return -1;
    }
}

static int handle_exception(vm_t *vm, seL4_Word ip, seL4_Word vcpu_idx)
{
    seL4_UserContext regs;
    seL4_CPtr tcb = vm_get_tcb(vm, vcpu_idx);
    printf("%sInvalid instruction from [%s] at PC: 0x"XFMT"%s\n",
           ANSI_COLOR(RED, BOLD), vm->name, seL4_GetMR(0), ANSI_COLOR(RESET));
    int err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    print_ctx_regs(&regs);
    return 1;
}

static int vm_vcpu_for_target_cpu(vm_t *vm, uintptr_t target_cpu)
{
    for (int i = 0; i < vm->n_vcpus; i++) {
        if (vm->vcpus[i].target_cpu == target_cpu && vm->vcpus[i].active) {
            return i;
        }
    }
    return -1;
}

// TODO might need to expose this for LATE_SMP? or to run qhee stuff on it?
static int vm_start_vcpu(vm_t *vm, uintptr_t target_cpu,
                       uintptr_t entry_point_address, uintptr_t context_id)
{
    vcpu_t *vcpu = &vm->vcpus[vm->n_vcpus];
    vcpu->target_cpu = target_cpu;
    vcpu->active = true;

    seL4_UserContext regs = {0};
    sel4arch_set_bootargs(&regs, entry_point_address, 0, context_id);
    seL4_Error err = seL4_TCB_WriteRegisters(vcpu->tcb.cptr, false, 0,
                                             sizeof(regs) / sizeof(regs.pc), &regs);
    if (err) {
        ZF_LOGE("Failed to write regs\n");
        return -1;
    }

    // TODO surely this needs some sanitising?
    err = seL4_ARM_VCPU_WriteRegs(vcpu->vcpu.cptr, seL4_VCPUReg_VMPIDR_EL2, target_cpu);
    if (err) {
        ZF_LOGE("Failed to write vcpu regs\n");
        return -1;
    }

    if (seL4_TCB_Resume(vcpu->tcb.cptr)) {
        ZF_LOGE("Failed to resume\n");
        return -1;
    }
    vm->n_vcpus++;
    return 0;
}

static int handle_psci(vm_t *vm, vcpu_t *vcpu, seL4_Word fn_number, bool convention)
{
    seL4_UserContext *regs = fault_get_ctx(vcpu->fault);
    switch (fn_number) {
    case PSCI_VERSION:
        regs->x0 = 0x00010000; // version 1 //(BIT(15) | BIT(1));
        return ignore_fault(vcpu->fault);
    case PSCI_CPU_ON: {
        uintptr_t target_cpu = smc_get_arg(regs, 1);
        uintptr_t entry_point_address = smc_get_arg(regs, 2);
        uintptr_t context_id = smc_get_arg(regs, 3);
        if (vm->n_vcpus == CONFIG_MAX_NUM_NODES) {
            regs->x0 = PSCI_INVALID_PARAMETERS;
            ZF_LOGE("Too many VCPUS");
        } else if (vm_vcpu_for_target_cpu(vm, target_cpu) != -1) {
            regs->x0 = PSCI_ALREADY_ON;
            ZF_LOGE("VCPU Already on\n");
        } else {
            if (vm_start_vcpu(vm, target_cpu, entry_point_address, context_id) == 0) {
                regs->x0 = PSCI_SUCCESS;
            } else {
                regs->x0 = PSCI_INTERNAL_FAILURE;
            }
        }
        return ignore_fault(vcpu->fault);
    }
    case PSCI_MIGRATE_INFO_TYPE:
        regs->x0 = 2; // trusted OS does not require migration
        return ignore_fault(vcpu->fault);
    case PSCI_FEATURES:
        //TODO yanyan implemented this, not sure if required
        regs->x0 = PSCI_NOT_SUPPORTED;
        return ignore_fault(vcpu->fault);
    case PSCI_SYSTEM_RESET:
        regs->x0 = PSCI_SUCCESS;
        return ignore_fault(vcpu->fault);
    default:
        ZF_LOGE("Unhandled PSCI function id %lu\n", fn_number);
        return -1;
    }
}

extern int vm_handle_smc(vm_t *vm, fault_t *fault, uint32_t esr);

static int handle_smc(vm_t *vm, vcpu_t *vcpu, uint32_t hsr)
{
    fault_t *fault = vcpu->fault;
    seL4_UserContext *regs = fault_get_ctx(fault);
    seL4_Word id = smc_get_function_id(regs);
    seL4_Word fn_number = smc_get_function_number(id);
    smc_call_id_t service = smc_get_call(id);

    switch (service) {
    case SMC_CALL_ARM_ARCH:
        //ZF_LOGE("Unhandled SMC: arm architecture call %lu\n", fn_number);
        //regs->x0 = -1;
        vm_handle_smc(vm, fault, hsr);
        return 0;
        //return ignore_fault(vcpu->fault);
    case SMC_CALL_CPU_SERVICE:
        ZF_LOGF("Unhandled SMC: CPU service call %lu\n", fn_number);
        break;
    case SMC_CALL_SIP_SERVICE:
        vm_handle_smc(vm, fault, hsr);
        ZF_LOGI("Got SiP service call %lu\n", fn_number);
        //restart_fault(vcpu->fault);
        return 0;
    case SMC_CALL_OEM_SERVICE:
        ZF_LOGF("Unhandled SMC: OEM service call %lu\n", fn_number);
        break;
    case SMC_CALL_STD_SERVICE:
        if (fn_number < PSCI_MAX) {
            return handle_psci(vm, vcpu, fn_number, smc_call_is_32(id));
        }
        ZF_LOGF("Unhandled SMC: standard service call %lu\n", fn_number);
        break;
    case SMC_CALL_STD_HYP_SERVICE:
        ZF_LOGF("Unhandled SMC: standard hyp service call %lu\n", fn_number);
        break;
    case SMC_CALL_VENDOR_HYP_SERVICE:
        ZF_LOGF("Unhandled SMC: vendor hyp service call %lu\n", fn_number);
        break;
    case SMC_CALL_TRUSTED_APP:
        vm_handle_smc(vm, fault, hsr);
        return 0;
        //ZF_LOGF("Unhandled SMC: trusted app call %lu\n", fn_number);
        //break;
    case SMC_CALL_TRUSTED_OS:
        vm_handle_smc(vm, fault, hsr);
        return 0;
        //ZF_LOGF("Unhandled SMC: trusted os call %lu\n", fn_number);
        break;
    default:
        ZF_LOGF("Unhandle SMC: unknown value service: %lu fn_number: %lu\n",
                (unsigned long) service, fn_number);
        break;
    }

    assert(0);
    return -1;
}

extern int handle_qhee_hvc(vm_t *vm, fault_t *fault, uint32_t hsr);

static int handle_hvc(vm_t *vm, fault_t *fault, uintptr_t hsr)
{
    handle_qhee_hvc(vm, fault, hsr);
    restart_fault(fault);
    return 0;
}

static void sgi_ack(UNUSED void *x) {
    // do nothing
}

#define MPIDR_AFF0(x) ((x) & 0xff)
#define MPIDR_AFF1(x) (((x) >> 8) & 0xff)
#define MPIDR_AFF2(x) (((x) >> 16) & 0xff)
#define MPIDR_AFF3(x) (((x) >> 32) & 0xff)

static int handle_msr_mrs_other(vm_t *vm, vcpu_t *vcpu, fault_t *fault, uintptr_t hsr)
{
    seL4_Word rt = *decode_rt(HSR_EC_RT(hsr), fault_get_ctx(fault));
    bool is_read = HSR_EC_DIR(hsr);

    extern int virq_blocked[CONFIG_MAX_NUM_NODES];
    bool masked = virq_blocked[fault->vcpu_idx];

    switch (hsr & MSR_MRS_MASK)  {
    case ICC_SGI1R_EL1: {
        uint32_t target_list = rt & 0xFFFF;
        uint32_t aff1 = (rt >> 16) & 0xFF;
        uint32_t intid = (rt >> 24) & 0xF;
        uint32_t aff2 = (rt >> 32) & 0xFF;
        uint32_t irm = (rt >> 40) & 0x1;
        uint32_t rs = (rt >> 44) & 0xF;
        uint32_t aff3 = (rt >> 48) & 0xFF;

        for (int i = 0; i < vm->n_vcpus; i++) {
            if (!vcpu_sgi_ppi[i][intid]) {
                vcpu_sgi_ppi[i][intid] = vm_virq_vcpu_new(vm, i, intid, sgi_ack, NULL);
            }
            ZF_LOGF_IF(vcpu_sgi_ppi[i][intid] == NULL, "No virq handle!")
            // Interrupts routed to all PEs in the system, excluding "self".
            if (irm) {
                if (vm->vcpus[i].target_cpu != vcpu->target_cpu) {
                    vm_inject_IRQ(vcpu_sgi_ppi[i][intid], i, masked);

                }
            // Interrupts routed to the PEs specified
            // by Aff3.Aff2.Aff1.<target list>.
            } else if (target_list & BIT(MPIDR_AFF0(vm->vcpus[i].target_cpu)) &&
                       aff1 == MPIDR_AFF1(vm->vcpus[i].target_cpu) &&
                       aff2 == MPIDR_AFF2(vm->vcpus[i].target_cpu) &&
                       aff3 == MPIDR_AFF3(vm->vcpus[i].target_cpu)) {
                //printf("%d: Got SGI %x %x %x %x\n", __LINE__, target_list, aff1, aff2, aff3);
                vm_inject_IRQ(vcpu_sgi_ppi[i][intid], i, masked);
            } else {
                printf("Unhandled SGI %x %x %x %x\n", target_list, aff1, aff2, aff3);
            }
        }
        return ignore_fault(vcpu->fault);
    }

    default:
        ZF_LOGF("Unhandled MRS_MRS_OTHER: %u op0, %u op1, %u crn, %u crm, %u op2\n",
                (unsigned int) HSR_EC_OP0(hsr),
                (unsigned int) HSR_EC_OP1(hsr),
                (unsigned int) HSR_EC_CRN(hsr),
                (unsigned int) HSR_EC_CRM(hsr),
                (unsigned int) HSR_EC_OP2(hsr));
    }
    return -1;
}

static int create_vcpu(vm_t *vm, vka_t *vka, int priority,
                       seL4_Word badge, seL4_CPtr endpoint, seL4_Word cspace_root_data,
                       seL4_CPtr auth, seL4_Word vcpu_idx)
{
    /* Badge the endpoint */
    if (badge & VCPU_BADGE_MASK) {
        ZF_LOGE("Provided badge overlaps with VCPU badge mask\n");
        return -1;
    }

    cspacepath_t src = {0};
    cspacepath_t dst = {0};
    badge = VCPU_BADGE_CREATE(badge, vcpu_idx);
    printf("Assigning badge %p to vcpu id %lu\n", (void *) badge, vcpu_idx);
    vka_cspace_make_path(vka, endpoint, &src);
    if (vka_cspace_alloc_path(vka, &dst)) {
        ZF_LOGE("CSpace alloc failed");
        goto out;
    }

    if (vka_cnode_mint(&dst, &src, seL4_AllRights, badge)) {
        goto out;
    }

    /* Copy it to the cspace of the VM for fault IPC */
    src = dst;
    dst.root = vm->cspace.cptr;
    dst.capPtr = VM_FAULT_EP_SLOT + vcpu_idx;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    if (vka_cnode_copy(&dst, &src, seL4_AllRights)) {
        goto out;
    }

    /* Create TCB */
    vcpu_t *vcpu = &vm->vcpus[vcpu_idx];
    if (vka_alloc_tcb(vka, &vcpu->tcb)) {
        goto out;
    }

    if (seL4_TCB_Configure(vcpu->tcb.cptr, dst.capPtr,
                           vm->cspace.cptr, cspace_root_data,
                           vm->pd.cptr, seL4_NilData, 0, seL4_CapNull)) {
        goto out;
    }

    if (seL4_TCB_SetSchedParams(vcpu->tcb.cptr, auth, priority, priority)) {
        goto out;
    }

    /* Create VCPU */
    if (vka_alloc_vcpu(vka, &vcpu->vcpu)) {
        goto out;
    }

    if (seL4_ARM_VCPU_SetTCB(vcpu->vcpu.cptr, vcpu->tcb.cptr)) {
        goto out;
    }

    char buf[7];
    snprintf(buf, 7, "vcpu %d", (int) vcpu_idx);
    NAME_THREAD(vcpu->tcb.cptr, buf);

#if CONFIG_MAX_NUM_NODES > 1
    if (seL4_TCB_SetAffinity(vcpu->tcb.cptr, vcpu_idx)) {
        goto out;
    }
#endif

    /* Initialise fault system */
    vcpu->fault = fault_init(vm, vcpu_idx);
    if (!vcpu->fault) {
        goto out;
    }

    return 0;

out:
    ZF_LOGE("Failed to create vcpu\n");
    if (vcpu->tcb.cptr != 0) {
        vka_free_object(vka, &vcpu->tcb);
    }
    if (vcpu->vcpu.cptr != 0) {
        vka_free_object(vka, &vcpu->vcpu);
    }
    if (dst.capPtr != 0) {
        vka_cspace_free(vka, dst.capPtr);
    }
    return -1;
}

int vm_create(const char *name, int priority,
              seL4_CPtr vmm_endpoint, seL4_Word vm_badge,
              vka_t *vka, simple_t *simple, vspace_t *vmm_vspace,
              ps_io_ops_t *io_ops,
              vm_t *vm)
{

    bzero(vm, sizeof(vm_t));
    vm->name = name;
    vm->ndevices = 0;
    vm->nhooks = 0;
    vm->vka = vka;
    vm->simple = simple;
    vm->vmm_vspace = vmm_vspace;
    vm->io_ops = io_ops;
    vm->n_vcpus = 1;
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->vchan_num_cons = 0;
    vm->vchan_cons = NULL;
#endif //CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    /* Create a cspace */
    int err = vka_alloc_cnode_object(vka, VM_CSPACE_SIZE_BITS, &vm->cspace);
    assert(!err);
    cspacepath_t src = {0};
    cspacepath_t dst = {0};
    vka_cspace_make_path(vka, vm->cspace.cptr, &src);
    seL4_Word cspace_root_data = api_make_guard_skip_word(seL4_WordBits - VM_CSPACE_SIZE_BITS);
    dst.root = vm->cspace.cptr;
    dst.capPtr = VM_CSPACE_SLOT;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, cspace_root_data);
    assert(!err);

    /* Create a vspace */
    err = vka_alloc_vspace_root(vka, &vm->pd);
    assert(!err);
    err = simple_ASIDPool_assign(simple, vm->pd.cptr);
    assert(err == seL4_NoError);
    err = vmm_get_guest_vspace(vmm_vspace, &vm->vm_vspace, vka, vm->pd.cptr);
    assert(!err);

    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        err = create_vcpu(vm, vka, priority - 1, vm_badge, vmm_endpoint, cspace_root_data,
                          simple_get_tcb(simple), i);
        ZF_LOGF_IF(err, "Failed to create vcpu %d\n", i);
    }

    if (err) {
        ZF_LOGE("Failed to write vcpu regs\n");
        return -1;
    }

    return seL4_ARM_VCPU_WriteRegs(vm->vcpus[0].vcpu.cptr, seL4_VCPUReg_VMPIDR_EL2,
            BIT(24) | BIT(31));

}


int vm_set_bootargs(vm_t *vm, seL4_Word pc, seL4_Word mach_type, seL4_Word atags)
{
    seL4_UserContext regs;
    assert(vm);
    /* Write CPU registers */
    seL4_CPtr tcb = vm_get_tcb(vm, 0);
    int err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    sel4arch_set_bootargs(&regs, pc, mach_type, atags);
    err = seL4_TCB_WriteRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    return err;
}

int vm_start(vm_t *vm)
{
    return seL4_TCB_Resume(vm_get_tcb(vm, 0));
}

int vm_stop(vm_t *vm)
{
    /* kill all the vcpus */
    for (int i = CONFIG_MAX_NUM_NODES - 1; i >= 0; i--) {
        seL4_TCB_Suspend(vm_get_tcb(vm, i));
    }
}


static void sys_pa_to_ipa(vm_t *vm, seL4_UserContext *regs)
{
    uint32_t pa;
#ifdef CONFIG_ARCH_AARCH64
#else
    pa = regs->r0;
#endif

    DSTRACE("PA translation syscall from [%s]: 0x%08x->?\n", vm->name, pa);
#ifdef CONFIG_ARCH_AARCH64
#else
    regs->r0 = pa;
#endif
}


static void sys_ipa_to_pa(vm_t *vm, seL4_UserContext *regs)
{
    long ipa;
#ifdef CONFIG_ARCH_AARCH64
#else
    ipa = regs->r0;
#endif
    seL4_CPtr cap = vspace_get_cap(vm_get_vspace(vm), (void *)ipa);
    if (cap == seL4_CapNull) {
        void *mapped_address;
        mapped_address = map_vm_ram(vm, ipa, seL4_PageBits);
        if (mapped_address == NULL) {
            printf("Could not map address for IPA translation\n");
            return;
        }
        cap = vspace_get_cap(vm_get_vspace(vm), (void *)ipa);
        assert(cap != seL4_CapNull);
    }

    seL4_ARM_Page_GetAddress_t ret = seL4_ARM_Page_GetAddress(cap);
    assert(!ret.error);
    DSTRACE("IPA translation syscall from [%s]: 0x%08x->0x%08x\n",
            vm->name, ipa, ret.paddr);
#ifdef CONFIG_ARCH_AARCH64
#else
    regs->r0 = ret.paddr;
#endif
}

static void sys_nop(vm_t *vm, seL4_UserContext *regs)
{
    DSTRACE("NOP syscall from [%s]\n", vm->name);
}

static int handle_syscall(vm_t *vm, seL4_Word length, seL4_Word vcpu_idx)
{
    seL4_Word syscall = seL4_GetMR(seL4_UnknownSyscall_Syscall);
    seL4_Word ip = seL4_GetMR(seL4_UnknownSyscall_FaultIP);

    seL4_CPtr tcb = vm_get_tcb(vm, vcpu_idx);
    seL4_UserContext regs = {0};
    seL4_Error err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    regs.pc += 4;

    DSTRACE("Syscall %d from [%s]\n", syscall, vm->name);
    switch (syscall) {
    case 65:
        sys_pa_to_ipa(vm, &regs);
        break;
    case 66:
        sys_ipa_to_pa(vm, &regs);
        break;
    case 67:
        sys_nop(vm, &regs);
        break;
    default:
        printf("%sBad syscall from [%s]: scno %zd at PC: %p%s\n",
               ANSI_COLOR(RED, BOLD), vm->name, syscall, (void *) ip, ANSI_COLOR(RESET));
        return -1;
    }
    err = seL4_TCB_WriteRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    return 0;
}

#ifdef CONFIG_ENABLE_VTIMER_FAULT
static void vtimer_irq_ack(void *token)
{
    vcpu_t *vcpu = token;
    if (!vcpu) {
        ZF_LOGE("Failed to ACK VTimer: NULL VM handle");
        return;
    }
    seL4_Error err = seL4_ARM_VCPU_AckVTimer(vcpu->vcpu.cptr);
    if (err) {
        ZF_LOGF("Failed to ACK VTimer: VCPU Ack invocation failed");
    }
}

static int virtual_timer_irq(vm_t *vm, seL4_Word vcpu_idx)
{
    if (!vtimer_irq_handle[vcpu_idx]) {
        vtimer_irq_handle[vcpu_idx] = vm_virq_vcpu_new(vm, vcpu_idx, VIRTUAL_TIMER_IRQ, &vtimer_irq_ack, &vm->vcpus[vcpu_idx]);
        if (!vtimer_irq_handle[vcpu_idx]) {
            ZF_LOGE("Failed to create virtual timer irq");
            return -1;
        }
    }
    vm_inject_IRQ(vtimer_irq_handle[vcpu_idx], vcpu_idx);
    return 0;
}
#endif

static int vmm_affinity = 0;

int vm_event(vm_t *vm, seL4_MessageInfo_t tag, seL4_Word badge)
{
    seL4_Word label = seL4_MessageInfo_get_label(tag);
    seL4_Word length = seL4_MessageInfo_get_length(tag);
    int err = 0;
    seL4_Word vcpu_idx = VCPU_BADGE_IDX(badge);
    vcpu_t *vcpu = &vm->vcpus[vcpu_idx];
    fault_t *fault = vcpu->fault;

#if CONFIG_MAX_NUM_NODES > 1
    // TODO yanyan had this to prevent races with linux
    if (vmm_affinity != vcpu_idx) {
        vmm_affinity = vcpu_idx;
        err = seL4_TCB_SetAffinity(seL4_CapInitThreadTCB, vcpu_idx);
        ZF_LOGF_IF(err != 0, "Failed to move to vcpu core");
    }
#endif
    switch (label) {
    case seL4_Fault_VMFault: {
        err = new_fault(fault);
        assert(!err);
        do {
            err = handle_page_fault(vm, fault);
            if (err) {
                return -1;
            }
        } while (!fault_handled(fault));
    }
    break;

    case seL4_Fault_UnknownSyscall: {
        assert(length == seL4_UnknownSyscall_Length);
        err = handle_syscall(vm, length, vcpu_idx);
        assert(!err);
        if (!err) {
            seL4_MessageInfo_t reply;
            reply = seL4_MessageInfo_new(0, 0, 0, 0);
            seL4_Reply(reply);
        }
    }
    break;

    case seL4_Fault_UserException: {
        assert(length == seL4_UserException_Length);
        seL4_Word ip = seL4_GetMR(0);
        err = handle_exception(vm, ip, vcpu_idx);
        assert(!err);
        if (!err) {
            seL4_MessageInfo_t reply = seL4_MessageInfo_new(0, 0, 0, 0);
            seL4_Reply(reply);
        }
    }
    break;
    case seL4_Fault_VGICMaintenance: {
        int idx = -1;
        if (length == 0) {
            /* just maintenance, no irqs */
            idx = 0;
        } else if (length == seL4_VGICMaintenance_Length) {
            idx = (seL4_Word) seL4_GetMR(seL4_VGICMaintenance_IDX);
        } else {
            ZF_LOGE("invalid maintenance message, not replying\n");
                printf("%s:%d\n", __FILE__, __LINE__);
            return -1;
        }

        if (idx != -1) {
            err = handle_vgic_maintenance(vm, idx, vcpu_idx);
        } else {
            err = 0;
        }
        if (!err && length != 0) {
            seL4_MessageInfo_t reply = seL4_MessageInfo_new(0, 0, 0, 0);
            seL4_Reply(reply);
        }
        if (err) {
            ZF_LOGE("handle vgic maintenance failed!");
                printf("%s:%d\n", __FILE__, __LINE__);
            return -1;
        } else {
            return 0;
        }
    }
    break;
    case seL4_Fault_VCPUFault: {
        seL4_MessageInfo_t reply;
        assert(length == seL4_VCPUFault_Length);
        uint32_t hsr = seL4_GetMR(seL4_UnknownSyscall_ARG0);
        /* check if the exception class (bits 26-31) of the HSR indicate WFI/WFE */
        uint32_t exception_class = HSR_EC(hsr);
        switch (exception_class) {
        case HSR_EC_WFI_WFE:
            /* generate a new WFI fault */
            new_wfi_fault(fault);
            return 0;
        case HSR_EC_SMC32:
        case HSR_EC_SMC64: {
            new_smc_fault(fault);
            return handle_smc(vm, vcpu, hsr);
        }
        case HSR_EC_MSR_MRS_OTHER: {
            new_smc_fault(fault);
            return handle_msr_mrs_other(vm, vcpu, fault, hsr);
        }
        case HSR_EC_HVC64: {
            new_smc_fault(fault);
            return handle_hvc(vm, fault, hsr);
        }
        default:
            printf("Unhandled VCPU fault from [%s]: HSR 0x%08x\n", vm->name, hsr);
            if ((hsr & 0xfc300000) == 0x60200000 || hsr == 0xf2000800) {
                new_wfi_fault(fault);
                seL4_UserContext *regs = fault_get_ctx(fault);
                regs->pc += 4;
                seL4_TCB_WriteRegisters(vm_get_tcb(vm, vcpu_idx), false, 0,
                                        sizeof(*regs) / sizeof(regs->pc), regs);
                restart_fault(fault);
                return 0;
            }
            return -1;
        }
    }
    break;
#ifdef CONFIG_ENABLE_VTIMER_FAULT
    case seL4_Fault_VTimerEvent: {
        int err = virtual_timer_irq(vm, vcpu_idx);
        assert(!err);
        seL4_MessageInfo_t reply;
        reply = seL4_MessageInfo_new(0, 0, 0, 0);
        seL4_Reply(reply);
    }
    break;
#endif
    default:
        /* What? Why are we here? What just happened? */
        printf("Unknown fault from [%s]: label=%p length=%p\n",
               vm->name, (void *) label, (void *) length);
        return -1;
    }
    return 0;
}

int vm_copyout_atags(vm_t *vm, struct atag_list *atags, uint32_t addr)
{
    reservation_t res;
    struct atag_list *atag_cur;

    vka_t *vka = vm->vka;
    void *vm_addr = (void *)(addr & ~0xffflu);
    vspace_t *vm_vspace = vm_get_vspace(vm);
    vspace_t *vmm_vspace = vm->vmm_vspace;

    /* Make sure we don't cross a page boundary
     * NOTE: the next page will usually be used by linux for PT!
     */
    size_t size = 0;
    for (atag_cur = atags; atag_cur != NULL; atag_cur = atag_cur->next) {
        size += atags_size_bytes(atag_cur);
    }
    size += 8; /* NULL tag */
    assert((addr & 0xfff) + size < 0x1000);

    /* Create a frame (and a copy for the VMM) */
    vka_object_t frame = {0};
    int err = vka_alloc_frame(vka, 12, &frame);
    assert(!err);
    if (err) {
        return -1;
    }
    /* Map the frame to the VMM */
    void *vmm_addr = vspace_map_pages(vmm_vspace, &frame.cptr, NULL, seL4_AllRights, 1, 12, 0);
    assert(vmm_addr);

    /* Copy in the atags */
    void *buf = vmm_addr + (addr & 0xfff);
    for (atag_cur = atags; atag_cur != NULL; atag_cur = atag_cur->next) {
        int tag_size = atags_size_bytes(atag_cur);
        DVM("ATAG copy %p<-%p %d\n", (void *)buf, (void *)atag_cur->hdr, tag_size);
        memcpy(buf, atag_cur->hdr, tag_size);
        buf += tag_size;
    }
    /* NULL tag terminator */
    memset(buf, 0, 8);

    /* Unmap the page and map it into the VM */
    vspace_unmap_pages(vmm_vspace, vmm_addr, 1, 12, NULL);
    res = vspace_reserve_range_at(vm_vspace, vm_addr, 0x1000, seL4_AllRights, 0);
    assert(res.res);
    if (!res.res) {
        vka_free_object(vka, &frame);
        return -1;
    }
    err = vspace_map_pages_at_vaddr(vm_vspace, &frame.cptr, NULL, vm_addr, 1, 12, res);
    vspace_free_reservation(vm_vspace, res);
    assert(!err);
    if (err) {
        printf("Failed to provide memory\n");
        vka_free_object(vka, &frame);
        return -1;
    }

    return 0;
}

int vm_add_device(vm_t *vm, const struct device *d)
{
    assert(d != NULL);
    assert(vm->ndevices < MAX_DEVICES_PER_VM);
    if (vm->ndevices < MAX_DEVICES_PER_VM) {
        vm->devices[vm->ndevices++] = *d;
        return 0;
    } else {
        return -1;
    }
}

static int cmp_id(struct device *d, void *data)
{
    return !(d->devid == *((enum devid *)data));
}

static int cmp_ipa(struct device *d, void *data)
{
    return !dev_paddr_in_range(*(uintptr_t *)data, d);
}

struct device *
vm_find_device(vm_t *vm, int (*cmp)(struct device *d, void *data), void *data)
{
    for (int i = 0; i < vm->ndevices; i++) {
        if (cmp(&vm->devices[i], data) == 0) {
            return &vm->devices[i];
        }
    }
    return NULL;
}

struct device *
vm_find_device_by_id(vm_t *vm, enum devid id)
{
    return vm_find_device(vm, &cmp_id, &id);
}

struct device *
vm_find_device_by_ipa(vm_t *vm, uintptr_t ipa)
{
    return vm_find_device(vm, &cmp_ipa, &ipa);
}

vspace_t *vm_get_vspace(vm_t *vm)
{
    return &vm->vm_vspace;
}

vspace_t *vm_get_vmm_vspace(vm_t *vm)
{
    return vm->vmm_vspace;
}

int vm_install_service(vm_t *vm, seL4_CPtr service, int index, uint32_t b)
{
    cspacepath_t src, dst;
    seL4_Word badge = b;
    vka_cspace_make_path(vm->vka, service, &src);
    dst.root = vm->cspace.cptr;
    dst.capPtr = index;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    return vka_cnode_mint(&dst, &src, seL4_AllRights, badge);
}

uintptr_t vm_ipa_to_pa(vm_t *vm, uintptr_t ipa_base, size_t size)
{
    uintptr_t pa_base = 0;
    vspace_t *vspace = vm_get_vspace(vm);
    uintptr_t ipa = ipa_base;
    do {
        /* Find the cap */
        seL4_CPtr cap = vspace_get_cap(vspace, (void *)ipa);
        if (cap == seL4_CapNull) {
            return 0;
        }
        /* Find mapping size */
        int bits = vspace_get_cookie(vspace, (void *)ipa);
        assert(bits == 12 || bits == 21);
        /* Find the physical address */
        seL4_ARM_Page_GetAddress_t ret = seL4_ARM_Page_GetAddress(cap);
        if (ret.error) {
            return 0;
        }
        if (ipa == ipa_base) {
            /* Record the result */
            pa_base = ret.paddr + (ipa & MASK(bits));
            /* From here on, ipa and ret.paddr will be aligned */
            ipa &= ~MASK(bits);
        } else {
            /* Check for a contiguous mapping */
            if (ret.paddr - pa_base != ipa - ipa_base) {
                return 0;
            }
        }
        ipa += BIT(bits);
    } while (ipa - ipa_base < size);
    return pa_base;
}

int vm_register_reboot_callback(vm_t *vm, reboot_hook_fn hook, void *token)
{
    if (hook == NULL) {
        ZF_LOGE("hook is NULL");
        return -1;
    }
    struct reboot_hooks rb = {
        .fn = hook,
        .token = token
    };
    if (vm->nhooks < MAX_REBOOT_HOOKS_PER_VM) {
        vm->rb_hooks[vm->nhooks++] = rb;
        return 0;
    } else {
        return -1;
    }

}

int vm_process_reboot_callbacks(vm_t *vm)
{
    for (int i = 0; i < vm->nhooks; i++) {
        struct reboot_hooks rb = vm->rb_hooks[i];
        if (rb.fn == NULL) {
            ZF_LOGE("NULL call back has been registered");
            return -1;
        }
        int err = rb.fn(vm, rb.token);
        if (err) {
            ZF_LOGE("Callback returned error: %d", err);
            return err;
        }
    }
    return 0;
}
