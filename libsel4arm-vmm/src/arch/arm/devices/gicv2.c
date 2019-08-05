/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#include "vgic.h"
#include "../../../devices.h"

static inline void set_active(struct gic_dist_map *gic_dist, int irq, int v)
{
    if (v) {
        gic_dist->active[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->active[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int is_active(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->active[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static void vgic_dist_reset(struct device *d)
{
    struct gic_dist_map *gic_dist;
    gic_dist = vgic_priv_get_dist(d);
    memset(gic_dist, 0, sizeof(*gic_dist));
    gic_dist->ic_type         = 0x0000fce7; /* RO */
    gic_dist->dist_ident      = 0x0200043b; /* RO */
    gic_dist->enable_set[0]   = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]   = 0x0000ffff; /* 16bit RO */
    gic_dist->config[0]       = 0xaaaaaaaa; /* RO */
    /* Reset value depends on GIC configuration */
    gic_dist->config[1]       = 0x55540000;
    gic_dist->config[2]       = 0x55555555;
    gic_dist->config[3]       = 0x55555555;
    gic_dist->config[4]       = 0x55555555;
    gic_dist->config[5]       = 0x55555555;
    gic_dist->config[6]       = 0x55555555;
    gic_dist->config[7]       = 0x55555555;
    gic_dist->config[8]       = 0x55555555;
    gic_dist->config[9]       = 0x55555555;
    gic_dist->config[10]      = 0x55555555;
    gic_dist->config[11]      = 0x55555555;
    gic_dist->config[12]      = 0x55555555;
    gic_dist->config[13]      = 0x55555555;
    gic_dist->config[14]      = 0x55555555;
    gic_dist->config[15]      = 0x55555555;
    /* identification */
    gic_dist->periph_id[4]    = 0x00000004; /* RO */
    gic_dist->periph_id[8]    = 0x00000090; /* RO */
    gic_dist->periph_id[9]    = 0x000000b4; /* RO */
    gic_dist->periph_id[10]   = 0x0000002b; /* RO */
    gic_dist->component_id[0] = 0x0000000d; /* RO */
    gic_dist->component_id[1] = 0x000000f0; /* RO */
    gic_dist->component_id[2] = 0x00000005; /* RO */
    gic_dist->component_id[3] = 0x000000b1; /* RO */

    /* This tells Linux that all IRQs are routed to CPU0.
     * When we eventually support multiple vCPUs per guest,
     * this will need to be updated.
     */
    for (int i = 0; i < ARRAY_SIZE(gic_dist->targets); i++) {
        gic_dist->targets[i] = 0x01010101;
    }
}

int vgic_set_pending_virq_if_enabled(struct device *vgic_device, vm_t *vm, int virq)
{
    return vgic_dist_set_pending_irq(vgic_device, vm, virq);
}

void vgic_set_pending_virq(vgic_reg_t *reg, int virq, bool status)
{
    set_pending(reg, virq, status);
}
/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vgic_init(vm_t *vm, vgic_t *vgic)
{
    /* Distributor */
    struct device dist = dev_vgic_dist;
    vgic->registers = map_emulated_device(vm, &dev_vgic_dist);
    assert(vgic->registers);
    if (vgic->registers == NULL) {
        return -1;
    }

    dist.priv = (void *)vgic;
    vgic_dist_reset(&dist);
    if (vm_add_device(vm, &dist)) {
        return -1;
    }

    /* Remap VCPU to CPU */
    struct device vcpu = dev_vgic_vcpu;
    if (!map_vm_device(vm, vcpu.pstart, dev_vgic_cpu.pstart, seL4_AllRights)) {
        return -1;
    }
    vcpu.pstart = dev_vgic_cpu.pstart;
    if (vm_add_device(vm, &vcpu)) {
        return -1;
    }

    return 0;
}

const struct device dev_vgic_dist = {
    .devid = DEV_VGIC_DIST,
    .name = "vgic.distributor",
    .pstart = GIC_DIST_PADDR,
    .size = 0x1000,
    .handle_page_fault = &handle_vgic_dist_fault,
    .priv = NULL,
};

const struct device dev_vgic_cpu = {
    .devid = DEV_VGIC_CPU,
    .name = "vgic.cpu_interface",
    .pstart = GIC_CPU_PADDR,
    .size = 0x1000,
    .handle_page_fault = NULL,
    .priv = NULL,
};

const struct device dev_vgic_vcpu = {
    .devid = DEV_VGIC_VCPU,
    .name = "vgic.vcpu_interface",
    .pstart = GIC_VCPU_PADDR,
    .size = 0x1000,
    .handle_page_fault = NULL,
    .priv = NULL,
};
