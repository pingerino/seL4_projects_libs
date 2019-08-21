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
#include "../../../vm.h"

#define VIRTUAL_TIMER_IRQ 27

int handle_vgic_maintenance(vm_t *vm, int idx, seL4_Word vcpu_idx);

// TODO not sure this is needed
int vm_enable_irq(vm_t *vm, int irq_idx, seL4_Word vcpu_idx);
