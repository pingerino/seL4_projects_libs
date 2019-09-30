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
#include "../../../devices.h"
#include "../../../vm.h"

static int handle_ram_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    void *addr;

    addr = map_vm_ram(vm, fault_get_address(fault), seL4_PageBits);
    if (addr == NULL) {
        print_fault(fault);
        abandon_fault(fault);
    } else {
        restart_fault(fault);
    }
    return addr == NULL;
}


const struct device dev_vram = {
    .devid = DEV_RAM,
    .name = "RAM",
    .pstart = RAM_BASE,
    .size = RAM_SIZE,
    .handle_page_fault = handle_ram_fault,
    .priv = NULL,
};


int vm_install_ram_default(vm_t *vm)
{
    struct device d;
    d = dev_vram;
    return vm_add_device(vm, &d);
}


int vm_install_ram_range(vm_t *vm, uintptr_t start, size_t size)
{
    struct device d;
    d = dev_vram;
    d.pstart = start;
    d.size = size;
    return vm_add_device(vm, &d);
}
