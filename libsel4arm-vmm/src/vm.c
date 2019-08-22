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

//#define DEBUG_VM
//#define DEBUG_RAM_FAULTS
//#define DEBUG_DEV_FAULTS
//#define DEBUG_STRACE

#define VM_CSPACE_SIZE_BITS    4
#define VM_FAULT_EP_SLOT       1
#define VM_CSPACE_SLOT         2

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

/* HSR exception codes */
#define HSR_EC_WFI_WFE   1
#define HSR_EC_SMC      23

extern char _cpio_archive[];

static int handle_page_fault(vm_t *vm, fault_t *fault)
{

    /* See if the device is already in our address space */
    struct device *d = vm_find_device_by_ipa(vm, fault_get_address(fault));
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
        switch (addr) {
        case 0:
            print_fault(fault);
            return -1;
        default: {
            void *mapped = map_vm_device(vm, addr, addr, seL4_AllRights);
            if (mapped) {
                DVM("WARNING: Blindly mapped device @ 0x%x for PC 0x%x\n",
                    fault_get_address(fault), fault_get_ctx(fault)->pc);
                restart_fault(fault);
                return 0;
            }
            mapped = map_vm_ram(vm, addr);
            if (mapped) {
                DVM("WARNING: Mapped RAM for device @ 0x%x for PC 0%x\n",
                    fault_get_address(fault), fault_get_ctx(fault)->pc);
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

static int handle_exception(vm_t *vm, seL4_Word ip)
{
    seL4_UserContext regs;
    seL4_CPtr tcb = vm_get_tcb(vm);
    printf("%sInvalid instruction from [%s] at PC: 0x"XFMT"%s\n",
           ANSI_COLOR(RED, BOLD), vm->name, seL4_GetMR(0), ANSI_COLOR(RESET));
    int err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    print_ctx_regs(&regs);
    return 1;
}

static int handle_psci(vm_t *vm, seL4_Word fn_number, bool convention)
{
    seL4_UserContext *regs = fault_get_ctx(vm->fault);
    switch (fn_number) {
    case PSCI_VERSION:
        regs->x0 = (BIT(15) | BIT(1));
        return ignore_fault(vm->fault);
    case PSCI_MIGRATE_INFO_TYPE:
        regs->x0 = 2; // trusted OS does not require migration
        return ignore_fault(vm->fault);
    default:
        ZF_LOGE("Unhandled PSCI function id %lu\n", fn_number);
        return -1;
    }
}

static int handle_smc(vm_t *vm)
{
    fault_t *fault = vm->fault;
    new_smc_fault(fault);
    seL4_UserContext *regs = fault_get_ctx(fault);
    seL4_Word id = smc_get_function_id(regs);
    seL4_Word fn_number = smc_get_function_number(id);
    smc_call_id_t service = smc_get_call(id);

    switch (service) {
    case SMC_CALL_ARM_ARCH:
        ZF_LOGE("Unhandled SMC: arm architecture call %lu\n", fn_number);
        break;
    case SMC_CALL_CPU_SERVICE:
        ZF_LOGE("Unhandled SMC: CPU service call %lu\n", fn_number);
        break;
    case SMC_CALL_SIP_SERVICE:
        regs->x0 = -1;
        ZF_LOGW("Ignoring SiP service call %lu\n", fn_number);
        return ignore_fault(vm->fault);
    case SMC_CALL_OEM_SERVICE:
        ZF_LOGE("Unhandled SMC: OEM service call %lu\n", fn_number);
        break;
    case SMC_CALL_STD_SERVICE:
        if (fn_number < PSCI_MAX) {
            return handle_psci(vm, fn_number, smc_call_is_32(id));
        }
        ZF_LOGE("Unhandled SMC: standard service call %lu\n", fn_number);
        break;
    case SMC_CALL_STD_HYP_SERVICE:
        ZF_LOGE("Unhandled SMC: standard hyp service call %lu\n", fn_number);
        break;
    case SMC_CALL_VENDOR_HYP_SERVICE:
        ZF_LOGE("Unhandled SMC: vendor hyp service call %lu\n", fn_number);
        break;
    case SMC_CALL_TRUSTED_APP:
        ZF_LOGE("Unhandled SMC: trusted app call %lu\n", fn_number);
        break;
    case SMC_CALL_TRUSTED_OS:
        ZF_LOGE("Unhandled SMC: trusted os call %lu\n", fn_number);
        break;
    default:
        ZF_LOGE("Unhandle SMC: unknown value service: %lu fn_number: %lu\n",
                (unsigned long) service, fn_number);
        break;
    }

    return -1;
}

int vm_create(const char *name, int priority,
              seL4_CPtr vmm_endpoint, seL4_Word vm_badge,
              vka_t *vka, simple_t *simple, vspace_t *vmm_vspace,
              ps_io_ops_t *io_ops,
              vm_t *vm)
{

    seL4_Word null_cap_data = seL4_NilData;
    seL4_Word cspace_root_data;
    cspacepath_t src, dst;

    bzero(vm, sizeof(vm_t));
    vm->name = name;
    vm->ndevices = 0;
    vm->nhooks = 0;
    vm->entry_point = NULL;
    vm->vka = vka;
    vm->simple = simple;
    vm->vmm_vspace = vmm_vspace;
    vm->io_ops = io_ops;
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->vchan_num_cons = 0;
    vm->vchan_cons = NULL;
#endif //CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    /* Create a cspace */
    int err = vka_alloc_cnode_object(vka, VM_CSPACE_SIZE_BITS, &vm->cspace);
    assert(!err);
    vka_cspace_make_path(vka, vm->cspace.cptr, &src);
    cspace_root_data = api_make_guard_skip_word(seL4_WordBits - VM_CSPACE_SIZE_BITS);
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

    /* Badge the endpoint */
    vka_cspace_make_path(vka, vmm_endpoint, &src);
    err = vka_cspace_alloc_path(vka, &dst);
    assert(!err);
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, vm_badge);
    assert(!err);
    /* Copy it to the cspace of the VM for fault IPC */
    src = dst;
    dst.root = vm->cspace.cptr;
    dst.capPtr = VM_FAULT_EP_SLOT;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_copy(&dst, &src, seL4_AllRights);
    assert(!err);

    /* Create TCB */
    err = vka_alloc_tcb(vka, &vm->tcb);
    assert(!err);
    err = seL4_TCB_Configure(vm_get_tcb(vm), VM_FAULT_EP_SLOT,
                             vm->cspace.cptr, cspace_root_data,
                             vm->pd.cptr, null_cap_data, 0, seL4_CapNull);
    assert(!err);

    err = seL4_TCB_SetSchedParams(vm_get_tcb(vm), simple_get_tcb(simple), priority - 1, priority - 1);
    assert(!err);

    /* Create VCPU */
    err = vka_alloc_vcpu(vka, &vm->vcpu);
    assert(!err);
    err = seL4_ARM_VCPU_SetTCB(vm->vcpu.cptr, vm_get_tcb(vm));
    assert(!err);

    /* Initialise fault system */
    vm->fault = fault_init(vm);
    assert(vm->fault);

    return err;
}


int vm_set_bootargs(vm_t *vm, seL4_Word pc, seL4_Word mach_type, seL4_Word atags)
{
    seL4_UserContext regs;
    assert(vm);
    /* Write CPU registers */
    seL4_CPtr tcb = vm_get_tcb(vm);
    int err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    sel4arch_set_bootargs(&regs, pc, mach_type, atags);
    err = seL4_TCB_WriteRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    return err;
}

int vm_start(vm_t *vm)
{
    return seL4_TCB_Resume(vm_get_tcb(vm));
}

int vm_stop(vm_t *vm)
{
    return seL4_TCB_Suspend(vm_get_tcb(vm));
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
        mapped_address = map_vm_ram(vm, ipa);
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

static int handle_syscall(vm_t *vm, seL4_Word length)
{
    seL4_Word syscall = seL4_GetMR(seL4_UnknownSyscall_Syscall);
    seL4_Word ip = seL4_GetMR(seL4_UnknownSyscall_FaultIP);

    seL4_CPtr tcb = vm_get_tcb(vm);
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

int vm_event(vm_t *vm, seL4_MessageInfo_t tag)
{
    seL4_Word label = seL4_MessageInfo_get_label(tag);
    seL4_Word length = seL4_MessageInfo_get_length(tag);
    int err = 0;
    fault_t *fault = vm->fault;

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
        err = handle_syscall(vm, length);
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
        err = handle_exception(vm, ip);
        assert(!err);
        if (!err) {
            seL4_MessageInfo_t reply = seL4_MessageInfo_new(0, 0, 0, 0);
            seL4_Reply(reply);
        }
    }
    break;
    case seL4_Fault_VGICMaintenance: {
        assert(length == seL4_VGICMaintenance_Length);
        int idx = seL4_GetMR(seL4_VGICMaintenance_IDX);
        /* Currently not handling spurious IRQs */
        assert(idx >= 0);

        err = handle_vgic_maintenance(vm, idx);
        assert(!err);
        if (!err) {
            seL4_MessageInfo_t reply = seL4_MessageInfo_new(0, 0, 0, 0);
            seL4_Reply(reply);
        }
    }
    break;
    case seL4_Fault_VCPUFault: {
        seL4_MessageInfo_t reply;
        assert(length == seL4_VCPUFault_Length);
        uint32_t hsr = seL4_GetMR(seL4_UnknownSyscall_ARG0);
        /* check if the exception class (bits 26-31) of the HSR indicate WFI/WFE */
        uint32_t exception_class = hsr >> 26;
        switch (exception_class) {
        case HSR_EC_WFI_WFE:
            /* generate a new WFI fault */
            new_wfi_fault(fault);
            return 0;
        case HSR_EC_SMC:
            return handle_smc(vm);
        default:
            printf("Unhandled VCPU fault from [%s]: HSR 0x%08x\n", vm->name, hsr);
            if ((hsr & 0xfc300000) == 0x60200000 || hsr == 0xf2000800) {
                new_wfi_fault(fault);
                seL4_UserContext *regs = fault_get_ctx(fault);
                regs->pc += 4;
                seL4_TCB_WriteRegisters(vm_get_tcb(vm), false, 0,
                                        sizeof(*regs) / sizeof(regs->pc), regs);
                restart_fault(fault);
                return 0;
            }
            return -1;
        }
    }
    break;
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
        DVM("ATAG copy 0x%x<-0x%x %d\n", (uint32_t)buf, (uint32_t)atag_cur->hdr, tag_size);
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
