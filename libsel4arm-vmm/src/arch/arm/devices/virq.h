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
#include <stdint.h>
#include <stdbool.h>
#include <sel4arm-vmm/devices.h>
#include <sel4arm-vmm/plat/devices.h>
#include "../../../vm.h"

#define MAX_LR_OVERFLOW 64
#define LR_OF_NEXT(_i) (((_i) == MAX_LR_OVERFLOW - 1) ? 0 : ((_i) + 1))

struct virq_handle {
     int virq;
     void (*ack)(void *token);
     void *token;
     vm_t *vm;
};

typedef struct lr_of {
     struct virq_handle irqs[MAX_LR_OVERFLOW]; /* circular buffer */
     size_t head;
     size_t tail;
     bool full;
} lr_of_t;

typedef struct vgic {
/// Mirrors the vcpu list registers
     struct virq_handle *irq[CONFIG_MAX_NUM_NODES][63];
/// IRQs that would not fit in the vcpu list registers
     lr_of_t lr_overflow[CONFIG_MAX_NUM_NODES];
/// Complete set of virtual irqs
     struct virq_handle *virqs[MAX_VIRQS];
/// Virtual distributer registers
     void *registers;
} vgic_t;

static inline vgic_t *vgic_device_get_vgic(struct device *d)
{
     assert(d);
     assert(d->priv);
     return (vgic_t *)d->priv;
}

static inline vm_t *virq_get_vm(struct virq_handle *irq)
{
    return irq->vm;
}

static inline void virq_ack(struct virq_handle *irq)
{
    irq->ack(irq->token);
}

static inline struct virq_handle *virq_find_irq_data(vgic_t *vgic, int virq)
{
    int i;
    for (i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] && vgic->virqs[i]->virq == virq) {
            return vgic->virqs[i];
         }
    }
    return NULL;
}

static inline int virq_add(vgic_t *vgic, struct virq_handle *virq_data)
{
    int i;
    for (i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] == NULL) {
            vgic->virqs[i] = virq_data;
            return 0;
        }
    }
    return -1;
}

static inline int vgic_find_free_irq(vgic_t *vgic, int vcpu) {
    for (int i = 0; i < 64; i++) {
        if (vgic->irq[vcpu][i] == NULL) {
            return i;
        }
    }
    return -1;
}

static inline void vgic_shadow_irq(vgic_t *vgic, int i, struct virq_handle *irq, int vcpu)
{
    /* Shadow */
    vgic->irq[vcpu][i] = irq;
}

static inline int vgic_add_overflow_cpu(lr_of_t *lr_overflow, struct virq_handle *irq)
{
    /* Add to overflow list */
    int idx = lr_overflow->tail;
    if (unlikely(lr_overflow->full)) {
        ZF_LOGE("too many overflow irqs");
        return -1;
    }
    lr_overflow->irqs[idx] = *irq;
    lr_overflow->full = (lr_overflow->head == LR_OF_NEXT(lr_overflow->tail));
    if (!lr_overflow->full) {
        lr_overflow->tail = LR_OF_NEXT(idx);
    }
    return 0;
}

static inline int vgic_add_overflow(vgic_t *vgic, struct virq_handle *irq, int vcpu)
{
    return vgic_add_overflow_cpu(&vgic->lr_overflow[vcpu], irq);
}

int vgic_vcpu_inject_irq(vgic_t *vgic, seL4_CPtr vcpu, struct virq_handle *irq);

/* Check the overflow list for pending IRQs */
static inline int vgic_handle_overflow_cpu(vgic_t *vgic, lr_of_t *lr_overflow, seL4_CPtr vcpu)
{
    /* copy tail, as vgic_vcpu_inject_irq can mutate it, and we do
     * not want to process any new overflow irqs */
    size_t tail = lr_overflow->tail;
    for (size_t i = lr_overflow->head; i != tail; i = LR_OF_NEXT(i)) {
        if (vgic_vcpu_inject_irq(vgic, vcpu, &lr_overflow->irqs[i]) == 0) {
            lr_overflow->head = LR_OF_NEXT(i);
            lr_overflow->full = (lr_overflow->head == LR_OF_NEXT(lr_overflow->tail));
        } else {
            break;
        }
    }
}


static inline int vgic_handle_overflow(vgic_t *vgic, seL4_CPtr vcpu_cptr, int vcpu_id)
{
    return vgic_handle_overflow_cpu(vgic, &vgic->lr_overflow[vcpu_id], vcpu_cptr);
}

virq_handle_t vm_virq_new(vm_t *vm, int virq, void (*ack)(void *), void *token)
{
    struct virq_handle *virq_data;
    struct device *vgic_device;
    vgic_t *vgic;
    int err;
    vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    assert(vgic_device);
    if (!vgic_device) {
        return NULL;
    }
    vgic = vgic_device_get_vgic(vgic_device);
    assert(vgic);

    virq_data = malloc(sizeof(*virq_data));
    if (!virq_data) {
        return NULL;
    }
    virq_data->virq = virq;
    virq_data->token = token;
    virq_data->ack = ack;
    virq_data->vm = vm;
    err = virq_add(vgic, virq_data);
    if (err) {
        free(virq_data);
        return NULL;
    }
    return virq_data;
}
