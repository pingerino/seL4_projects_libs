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
#pragma once

#include <sel4arm-vmm/sel4_arch/vm.h>

#include <sel4utils/process.h>
#include <platsupport/io.h>

#include <elf/elf.h>
#include <simple/simple.h>
#include <vspace/vspace.h>
#include <vka/vka.h>

#include <sel4arm-vmm/atags.h>
#include <sel4arm-vmm/devices.h>
#include <sel4arm-vmm/fault.h>

#include <sel4vmmcore/util/io.h>
#include <sel4pci/pci.h>

#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
#include <sel4arm-vmm/vchan_vm_component.h>
#include <sel4vchan/vchan_component.h>
#endif //CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

#define MAX_DEVICES_PER_VM 50
#define MAX_REBOOT_HOOKS_PER_VM 10

typedef int (*reboot_hook_fn)(vm_t *vm, void *token);

struct reboot_hooks {
    reboot_hook_fn fn;
    void *token;
};

typedef struct vcpu {
    /* the identifier used by the guest to enable this vcpu
     * (the first argument passed by the PSCI_CPU_ON SMC) */
    uintptr_t target_cpu;
    /* seL4 objects for the vcpu */
    vka_object_t tcb;
    vka_object_t vcpu;
    /* fault structure for this vcpu - tracks current fault state of the guest */
    fault_t *fault;
    /* true if this vcpu is currently being used by the guest */
    bool active;
} vcpu_t;

struct vm {
    /* Identification */
    const char *name;
    int vmid;
    /* OS support */
    vka_t *vka;
    simple_t *simple;
    vspace_t *vmm_vspace;
    ps_io_ops_t *io_ops;
    /* VM objects */
    vspace_t vm_vspace;
    sel4utils_alloc_data_t data;
    vka_object_t cspace;
    vka_object_t pd;
    int n_vcpus;
    vcpu_t vcpus[CONFIG_MAX_NUM_NODES];
    /* Installed devices */
    struct device devices[MAX_DEVICES_PER_VM];
    int ndevices;
    /* Installed reboot hooks */
    struct reboot_hooks rb_hooks[MAX_REBOOT_HOOKS_PER_VM];
    int nhooks;

    /* Virtual PCI Host Bridge */
    vmm_pci_space_t *pci;
    /* IOPort Manager */
    vmm_io_port_list_t *io_port;
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    /* Installed vchan connections */
    camkes_vchan_con_t **vchan_cons;
    unsigned int vchan_num_cons;

    int (*lock)(void);
    int (*unlock)(void);
#endif //CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
};
typedef struct vm vm_t;

typedef struct virq_handle *virq_handle_t;

/**
 * Create a virtual machine
 * @param name           A logical name for the VM
 * @param priority       The priority of the new VM
 * @param vmm_endpoint   An endpoint for IPC delivery such as VM faults
 * @param vm_badge       The badge for delivered IPCs
 * @param vka            A vka for object allocation.
 *                       Must be valid for the life of the VM.
 * @param simple         A simple instance for key cap acquisition.
 *                       Must be valid for the life of the VM.
 * @param vmm_vspace     The vspace of the VMM.
 *                       Must be valid for the life of the VM.
 * @param io_ops         IO operations for the VM to use when mapping and
 *                       configuring hardware.
 *                       Must be valid for the life of the VM.
 * @param vm             A reference to a vm struct to initialise
 * @return               0 on success
 */
int vm_create(const char *name, int priority,
              seL4_CPtr vmm_endpoint, seL4_Word vm_badge,
              vka_t *vka, simple_t *simple, vspace_t *vspace,
              ps_io_ops_t *io_ops,
              vm_t *vm);

/**
 * Copy data in from the VM.
 * @param[in] vm      A handle to the VM that the data should be read from
 * @param[in] data    The address of the data to load to
 * @param[in] address The VM IPA address that the data should be read at
 * @param[in] size    The number of bytes of data to read
 * @return            0 on success
 */
int vm_copyin(vm_t *vm, void *data, uintptr_t address, size_t size);

/**
 * Copy data out to the VM.
 * @param[in] vm      A handle to the VM that the data should be loaded into
 * @param[in] data    The address of the data to load
 * @param[in] address The VM IPA address that the data should be loaded at
 * @param[in] size    The number of bytes of data to load
 * @return            0 on success
 */
int vm_copyout(vm_t *vm, void *data, uintptr_t address, size_t size);

/**
 * Copy ELF segments out to the VM/
 * The virtual addresses in the ELF file are assummed to represent IPA addresses
 * @param[in] vm       A handle to the VM that the ELF should be loaded into
 * @param[in] elf_data The address of the ELF file header
 * @return             On success, returns the IPA entry point of the ELF file
 */
void *vm_copyout_elf(vm_t *vm, elf_t *elf_data);

/**
 * Copy out an atag list to the VM.
 * @param[in] vm        A handle to the VM that the ELF should be loaded into
 * @param[in] atag_list A handle to a list of atags
 * @param[in] addr      The address that the atags should be copied to
 * @return              0 on success
 */
int vm_copyout_atags(vm_t *vm, struct atag_list *atags, uint32_t addr);

/**
 * Set the boot args and pc for the VM.
 * For linux:
 *   r0 -> 0
 *   r1 -> MACH_TYPE  (#4151 for EXYNOS5410 eval. platform smdk5410)
 *   r2 -> atags address
 * @param[in] vm        A handle to a VM
 * @param[in] pc        The initial PC for the VM
 * @param[in] mach_type Linux specific machine ID
 *                      see http://www.arm.linux.org.uk/developer/machines/
 * @param[in] atags     Linux specific IPA of atags
 * @return              0 on success
 */
int vm_set_bootargs(vm_t *vm, seL4_Word pc, seL4_Word mach_type, seL4_Word atags);


/**
 * Start up a VM
 * @param[in] vm  The virtual machine to boot
 * @return        0 on success
 */
int vm_start(vm_t *vm);

/**
 * Stop a VM. The VM can be started later with a call to vm_start
 * @param[in] vm The virtual machine to stop
 * @return       0 on success
 */
int vm_stop(vm_t *vm);

/**
 * Handle a VM event
 * @param[in] vm   A handle to the VM that triggered the event
 * @param[in] tag  The tag of the incomming message
 * @param[in] badge The badge of the incomming message
 * @return     0 on success, otherwise, the VM should be shut down
 */
int vm_event(vm_t *vm, seL4_MessageInfo_t tag, seL4_Word badge);

/**
 * Register or replace a virtual IRQ definition
 * @param[in] virq  The IRQ number to inject
 * @param[in] ack   A function to call when the VM ACKs the IRQ
 * @param[in] token A token to pass, unmodified, to the ACK callback function
 */
virq_handle_t vm_virq_new(vm_t *vm, int virq, void (*ack)(void *), void *token);

/**
 * Register or replace a virtual IRQ definition for a per-vcpu virq.
 * @param[in] virq     The IRQ number to inject
 * @param[in] vcpu_idx The index of the VCPU that the irq is for.
 * @param[in] ack      A function to call when the VM ACKs the IRQ
 * @param[in] token    A token to pass, unmodified, to the ACK callback function
 */
virq_handle_t vm_virq_vcpu_new(vm_t *vm, seL4_Word vcpu_idx, int irq, void (*ack)(void *), void *token);
/**
 * Inject an IRQ into a VM
 * @param[in] virq  A handle to the virtual IRQ
 * @param[in] vcpu_idx  The vpu identifier for which vcpu to inject the irq into.
 * @return          0 on success
 */
int vm_inject_IRQ(virq_handle_t virq, seL4_Word vcpu_idx);

/**
 * Install a service into the VM
 * Effectively a cap copy into the CNode of the VM. This allows the VM to
 * make a seL4 call
 * @param[in] vm       The VM to install the service into
 * @param[in] service  A capability for VM
 * @param[in] index    The index at which to install the cap
 * @param[in] badge    The badge to assign to the cap.
 * @return             0 on success
 */
int vm_install_service(vm_t *vm, seL4_CPtr service, int index, uint32_t badge);

/**
 * Given a guest intermiate physical address, find the actual physical address
 * of the mapping.
 * @param[in] vm       The VM in question
 * @param[in] ipa      The guest physical address in question
 * @param[in] size     The size of the request
 * @return             The physical address which corresponds to the provided
 *                     guest physical address or 0 on error. An error will occur
 *                     If a valid mapping is not present, or if the mapping does
 *                     not provide a contiguous region of the requested size.
 */
uintptr_t vm_ipa_to_pa(vm_t *vm, uintptr_t ipa, size_t size);

/**
 * Add a function to be called when the vm is reset by the vmm.
 * @param  vm    The VM
 * @param  hook  reboot_hook_fn to be called when vm resets
 * @param  token token to be passed to hook function
 * @return       0 if callback is successfully registerd.  -1 otherwise.
 */
int vm_register_reboot_callback(vm_t *vm, reboot_hook_fn hook, void *token);

/**
 * Calls all reboot callbacks.  Should be called when VM is rebooted by vmm.
 * @param  vm The VM
 * @return    0 if all callbacks successfully called. If a callback fails with
 *            an error then this returns with that error and callbacks registered
 *            after failing callback will not be called.
 */
int vm_process_reboot_callbacks(vm_t *vm);
