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

#include <sel4arm-vmm/vm.h>

struct vm_onode {
    vka_object_t object;
    struct vm_onode *next;
};

/* We use the top bits of the badge for the VCPU IDX
 */
#define VCPU_IDX_BITS 4
#define VCPU_BADGE_SHIFT  (seL4_BadgeBits - VCPU_IDX_BITS)
#define VCPU_BADGE_MASK (MASK(VCPU_IDX_BITS) << VCPU_BADGE_SHIFT)
#define VCPU_BADGE_CREATE(badge, idx)     (badge | (idx << VCPU_BADGE_SHIFT))
#define VCPU_BADGE_IDX(badge) ((badge & VCPU_BADGE_MASK) >> VCPU_BADGE_SHIFT)

_Static_assert(BIT(VCPU_IDX_BITS) >= CONFIG_MAX_NUM_NODES, "badge_bits_fit_vcpu");

/* When differentiating VM's by colour, call this function */
const char *choose_colour(vm_t *vm);

/**
 * Find a device within the VM
 */
struct device *vm_find_device_by_id(vm_t *vm, enum devid id);
struct device *vm_find_device_by_ipa(vm_t *vm, uintptr_t ipa);


vspace_t *vm_get_vspace(vm_t *vm);
vspace_t *vm_get_vmm_vspace(vm_t *vm);
