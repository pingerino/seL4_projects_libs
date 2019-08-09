/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * Copyright 2018, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_DORNERWORKS_BSD)
 */

#include "vgic.h"
#include "../../../devices.h"
#include <sel4arm-vmm/plat/devices.h>

#define sgi_not_pending(...) !sgi_is_pending(__VA_ARGS__)
#define sgi_not_enabled(...) !sgi_is_enabled(__VA_ARGS__)

static inline struct gic_rdist_map *vgic_priv_get_rdist(struct device *d)
{
    assert(d);
    assert(d->priv);
    return vgic_device_get_vgic(d)->registers->rdist;
}

static inline struct gic_rdist_sgi_ppi_map *vgic_priv_get_rdist_sgi(struct device *d)
{
    assert(d);
    assert(d->priv);
    return vgic_device_get_vgic(d)->registers->sgi;
}

static inline void sgi_set_pending(struct gic_rdist_sgi_ppi_map *gic_sgi, int irq, int v)
{
    if (v) {
        gic_sgi->ispend[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_sgi->icpend[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_sgi->ispend[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_sgi->icpend[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int sgi_is_pending(struct gic_rdist_sgi_ppi_map *gic_sgi, int irq)
{
    return !!(gic_sgi->ispend[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline void sgi_set_enable(struct gic_rdist_sgi_ppi_map *gic_sgi, int irq, int v)
{
    if (v) {
        gic_sgi->isenable[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_sgi->icenable[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_sgi->isenable[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_sgi->icenable[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int sgi_is_enabled(struct gic_rdist_sgi_ppi_map *gic_sgi, int irq)
{
    return !!(gic_sgi->isenable[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int is_active(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->active_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int sgi_is_active(struct gic_rdist_sgi_ppi_map *gic_sgi, int irq)
{
    return !!(gic_sgi->isactive[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static enum gic_dist_action gic_rdist_get_action(int offset)
{
    return ACTION_READONLY;
}

static enum gic_dist_action gic_sgi_get_action(int offset)
{
    if (0x100 <= offset && offset < 0x180) {        /* enable_set      */
        return ACTION_ENABLE_SET;
    } else if (0x180 <= offset && offset < 0x200) { /* enable_clr      */
        return ACTION_ENABLE_CLR;
    } else {
        return ACTION_READONLY;
    }
    return ACTION_UNKNOWN;
}

static int vgic_sgi_enable_irq(struct device *d, vm_t *vm, int irq)
{
    struct gic_rdist_sgi_ppi_map *gic_sgi = vgic_priv_get_rdist_sgi(d);
    vgic_t *vgic = vgic_device_get_vgic(d);
    if (irq >= GIC_SGI_IRQ_MIN) {
        DDIST("sgi enabling irq %d\n", irq);
        sgi_set_enable(gic_sgi, irq, true);
        struct virq_handle *virq_data = virq_find_irq_data(vgic, irq);
        if (virq_data) {
            /* STATE b) */
            if (sgi_not_pending(gic_sgi, virq_data->virq)) {
                DDIST("IRQ not pending\n");
                virq_ack(virq_data);
            }
        } else {
            DDIST("enabled irq %d has no handle\n", irq);
        }
    }
    return 0;
}

static int vgic_sgi_disable_irq(struct device *d, vm_t *vm, int irq)
{
    /* STATE g) */
    struct gic_rdist_sgi_ppi_map *gic_sgi = vgic_priv_get_rdist_sgi(d);
    if (irq >= GIC_SGI_IRQ_MIN) {
        DDIST("sgi disabling irq %d\n", irq);
        sgi_set_enable(gic_sgi, irq, false);
    }
    return 0;
}

static int vgic_sgi_set_pending_irq(struct device *d, vm_t *vm, int irq)
{
    /* STATE c) */
    struct gic_rdist_sgi_ppi_map *gic_sgi = vgic_priv_get_rdist_sgi(d);
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    vgic_t *vgic = vgic_device_get_vgic(d);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, irq);
    /* If it is enables, inject the IRQ */
    if (virq_data && gic_dist_is_enabled(gic_dist) && sgi_is_enabled(gic_sgi, irq)) {
        int err;
        DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);

        sgi_set_pending(gic_sgi, virq_data->virq, true);
        err = vgic_vcpu_inject_irq(d, vm, virq_data);
        assert(!err);

        return err;
    } else {
        /* No further action */
        DDIST("IRQ not enabled (%d) for %s\n", irq, vm->name);
    }

    return 0;
}


static int handle_vgic_rdist_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    struct gic_rdist_map *gic_rdist = vgic_priv_get_rdist(d);
    int offset = fault_get_address(fault) - d->pstart;
    uint32_t *reg = (uint32_t *)((uintptr_t)gic_rdist + (offset - (offset % 4)));
    enum gic_dist_action act = gic_rdist_get_action(offset);

    /* Out of range */
    if (offset < 0 || offset >= sizeof(struct gic_rdist_map)) {
        DDIST("rdist offset out of range %x %x\n", offset, sizeof(struct gic_rdist_map));
        return ignore_fault(fault);

        /* Read fault */
    } else if (fault_is_read(fault)) {
        fault_set_data(fault, *reg);
        return advance_fault(fault);
    } else {
        switch (act) {
        case ACTION_READONLY:
            return ignore_fault(fault);

        case ACTION_UNKNOWN:
        default:
            DDIST("Unknown action on offset 0x%x\n", offset);
            return ignore_fault(fault);
        }
    }
    abandon_fault(fault);
    return -1;
}

static int handle_vgic_sgi_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    struct gic_rdist_sgi_ppi_map *gic_rdist_sgi = vgic_priv_get_rdist_sgi(d);
    uint32_t mask = fault_get_data_mask(fault);
    int offset = fault_get_address(fault) - d->pstart;
    uint32_t *reg = (uint32_t *)((uintptr_t)gic_rdist_sgi + (offset - (offset % 4)));
    enum gic_dist_action act = gic_sgi_get_action(offset);

    /* Out of range */
    if (offset < 0 || offset >= sizeof(struct gic_rdist_sgi_ppi_map)) {
        DDIST("sgi offset out of range %x %x\n", offset, sizeof(struct gic_rdist_sgi_ppi_map));
        return ignore_fault(fault);

        /* Read fault */
    } else if (fault_is_read(fault)) {
        fault_set_data(fault, *reg);
        return advance_fault(fault);
    } else {
        uint32_t data;
        switch (act) {
        case ACTION_READONLY:
            return ignore_fault(fault);
        case ACTION_ENABLE_SET:
            data = fault_get_data(fault);

            /* Mask the data to write */
            data &= mask;
            /* Mask bits that are already set */
            data &= ~(*reg);

            while (data) {
                int irq;
                irq = CTZ(data);
                data &= ~(1U << irq);
                irq += (offset - 0x100) * 8;
                vgic_sgi_enable_irq(d, vm, irq);
            }
            return ignore_fault(fault);

        case ACTION_ENABLE_CLR:
            data = fault_get_data(fault);
            /* Mask the data to write */
            data &= mask;
            /* Mask bits that are already clear */
            data &= *reg;
            while (data) {
                int irq;
                irq = CTZ(data);
                data &= ~(1U << irq);
                irq += (offset - 0x180) * 8;
                vgic_sgi_disable_irq(d, vm, irq);
            }
            return ignore_fault(fault);

        case ACTION_UNKNOWN:
        default:
            DDIST("Unknown action on offset 0x%x\n", offset);
            return ignore_fault(fault);
        }
    }
    abandon_fault(fault);
    return -1;
}

static void vgic_dist_reset(struct device *d)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    memset(gic_dist, 0, sizeof(*gic_dist));

    gic_dist->typer            = 0x7B04B0; /* RO */
    gic_dist->iidr             = 0x1043B ; /* RO */

    gic_dist->enable_set[0]    = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]    = 0x0000ffff; /* 16bit RO */

    gic_dist->config[0]        = 0xaaaaaaaa; /* RO */

    gic_dist->pidrn[0]         = 0x44;     /* RO */
    gic_dist->pidrn[4]         = 0x92;     /* RO */
    gic_dist->pidrn[5]         = 0xB4;     /* RO */
    gic_dist->pidrn[6]         = 0x3B;     /* RO */

    gic_dist->cidrn[0]         = 0x0D;     /* RO */
    gic_dist->cidrn[1]         = 0xF0;     /* RO */
    gic_dist->cidrn[2]         = 0x05;     /* RO */
    gic_dist->cidrn[3]         = 0xB1;     /* RO */
}

static void vgic_rdist_reset(struct device *d)
{
    struct gic_rdist_map *gic_rdist = vgic_priv_get_rdist(d);

    memset(gic_rdist, 0, sizeof(*gic_rdist));

    gic_rdist->typer           = 0x1;      /* RO */
    gic_rdist->iidr            = 0x1143B;  /* RO */

    gic_rdist->pidr0           = 0x93;     /* RO */
    gic_rdist->pidr1           = 0xB4;     /* RO */
    gic_rdist->pidr2           = 0x3B;     /* RO */
    gic_rdist->pidr4           = 0x44;     /* RO */

    gic_rdist->cidr0           = 0x0D;     /* RO */
    gic_rdist->cidr1           = 0xF0;     /* RO */
    gic_rdist->cidr2           = 0x05;     /* RO */
    gic_rdist->cidr3           = 0xB1;     /* RO */
}

static void vgic_rdist_sgi_reset(struct device *d)
{
    struct gic_rdist_sgi_ppi_map *gic_sgi = vgic_priv_get_rdist_sgi(d);

    memset(gic_sgi, 0, sizeof(*gic_sgi));

    gic_sgi->isactive[0]       = 0xaaaaaaaa;
}

int vgic_set_pending_virq_if_enabled(struct device *vgic_device, vm_t *vm, int virq)
{
    if (virq >= GIC_SGI_IRQ_MAX) {
        vgic_dist_set_pending_irq(vgic_device, vm, virq);
    } else {
        vgic_sgi_set_pending_irq(vgic_device, vm, virq);
    }
}

void vgic_set_pending_virq(vgic_reg_t *reg, int virq, bool status)
{
    if (virq >= GIC_SGI_IRQ_MAX) {
        gic_dist_set_pending(reg->dist, virq, status);
    } else {
        sgi_set_pending(reg->sgi, virq, status);
    }
}
/*
 * 1) completely virtualize the distributor
 * 2) completely virtualize the redistributor
 * 3) completely virtualize the SGI page of the redistributor
 */
int vgic_init(vm_t *vm, vgic_t *vgic)
{
    /* Distributor */
    vgic->registers = malloc(sizeof(vgic_reg_t));
    if (vgic->registers == NULL) {
        return -1;
    }

    struct device dist = dev_vgic_dist;
    vgic->registers->dist = map_emulated_device(vm, &dev_vgic_dist);
    assert(vgic->registers->dist);
    if (vgic->registers->dist == NULL) {
        return -1;
    }

    dist.priv = (void *)vgic;
    vgic_dist_reset(&dist);
    if (vm_add_device(vm, &dist)) {
        goto out;
    }

    /* Redistributor */
    struct device rdist = dev_vgic_redist;
    vgic->registers->rdist = map_emulated_device(vm, &dev_vgic_redist);
    if (vgic->registers->rdist == NULL) {
        goto out;
    }

    rdist.priv = (void *)vgic;
    vgic_rdist_reset(&rdist);
    if (vm_add_device(vm, &rdist)) {
        goto out;
    }

    /* Redistributor SGI */
    struct device sgi = dev_vgic_redist_sgi;
    vgic->registers->sgi = map_emulated_device(vm, &dev_vgic_redist_sgi);
    if (vgic->registers->sgi == NULL) {
        goto out;
    }

    sgi.priv = (void *)vgic;
    vgic_rdist_sgi_reset(&sgi);
    if (vm_add_device(vm, &sgi)) {
        goto out;
    }

    return 0;
out:
    free(vgic->registers);
    return -1;
}
#if 0
int vm_reset_vgic(vm_t *vm)
{
    struct vgic *vgic;
    struct device dist, rdist, sgi;
    struct device *vgic_device;

    vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    assert(vgic_device);
    vgic = vgic_device_get_vgic(vgic_device);
    assert(vgic);

    vgic->lr_overflow = NULL;

    /* Distributor */
    dist = dev_vgic_dist;

    dist.priv = (void *)vgic;
    vgic_dist_reset(&dist);

    /* Redistributor */
    rdist = dev_vgic_redist;
    rdist.priv = (void *)vgic;

    vgic_rdist_reset(&rdist);

    /* Redistributor SGI */
    sgi = dev_vgic_redist_sgi;
    sgi.priv = (void *)vgic;

    vgic_rdist_sgi_reset(&sgi);

    return 0;
}
#endif
const struct device dev_vgic_dist = {
    .devid = DEV_VGIC_DIST,
    .name = "vgic.distributor",
    .pstart = GIC_DIST_PADDR,
    .size = 0x10000,
    .handle_page_fault = &handle_vgic_dist_fault,
    .priv = NULL,
};

const struct device dev_vgic_redist = {
    .devid = DEV_VGIC_REDIST,
    .name = "vgic.redistributor",
    .pstart = GIC_REDIST_PADDR,
    .size = 0x10000,
    .handle_page_fault = &handle_vgic_rdist_fault,
    .priv = NULL,
};

const struct device dev_vgic_redist_sgi = {
    .devid = DEV_VGIC_REDIST_SGI,
    .name = "vgic.redistributor_sgi",
    .pstart = GIC_REDIST_PADDR + GIC_SGI_OFFSET,
    .size = 0x10000,
    .handle_page_fault = &handle_vgic_sgi_fault,
    .priv = NULL,
};
