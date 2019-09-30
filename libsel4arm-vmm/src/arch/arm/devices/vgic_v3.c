#define ZF_LOG_LEVEL ZF_LOG_ERROR
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
/*
 * This component controls and maintains the GIC for the VM.
 * IRQs must be registered at init time with vm_virq_new(...)
 * This function creates and registers an IRQ data structure which will be used for IRQ maintenance
 * b) ENABLING: When the VM enables the IRQ, it checks the pending flag for the VM.
 *   - If the IRQ is not pending, we either
 *        1) have not received an IRQ so it is still enabled in seL4
 *        2) have received an IRQ, but ignored it because the VM had disabled it.
 *     In either case, we simply ACK the IRQ with seL4. In case 1), the IRQ will come straight through,
       in case 2), we have ACKed an IRQ that was not yet pending anyway.
 *   - If the IRQ is already pending, we can assume that the VM has yet to ACK the IRQ and take no further
 *     action.
 *   Transitions: b->c
 * c) PIRQ: When an IRQ is received from seL4, seL4 disables the IRQ and sends an async message. When the VMM
 *    receives the message.
 *   - If the IRQ is enabled, we set the pending flag in the VM and inject the appropriate IRQ
 *     leading to state d)
 *   - If the IRQ is disabled, the VMM takes no further action, leading to state b)
 *   Transitions: (enabled)? c->d :  c->b
 * d) When the VM acknowledges the IRQ, an exception is raised and delivered to the VMM. When the VMM
 *    receives the exception, it clears the pending flag and acks the IRQ with seL4, leading back to state c)
 *    Transition: d->c
 * g) When/if the VM disables the IRQ, we may still have an IRQ resident in the GIC. We allow
 *    this IRQ to be delivered to the VM, but subsequent IRQs will not be delivered as seen by state c)
 *    Transitions g->c
 *
 *   NOTE: There is a big assumption that the VM will not manually manipulate our pending flags and
 *         destroy our state. The affects of this will be an IRQ that is never acknowledged and hence,
 *         will never occur again.
 */

#include "vgic.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <utils/arith.h>
#include <vka/vka.h>
#include <vka/capops.h>

#include "../../../devices.h"
#include "../../../vm.h"

#include "virq.h"
#include "gicv3.h"
#include "vdist.h"

#define sgi_not_pending(...) !sgi_is_pending(__VA_ARGS__)
#define sgi_not_enabled(...) !sgi_is_enabled(__VA_ARGS__)

static struct gic_dist_map *vgic_priv_get_dist(struct device *d) {
    vgic_t *vgic = vgic_device_get_vgic(d);
    return priv_get_dist(vgic->registers);
}

static inline struct gic_rdist_map *vgic_priv_get_rdist(struct device *d, seL4_Word vcpu_idx)
{
    assert(d);
    assert(d->priv);
    vgic_t *vgic = vgic_device_get_vgic(d);
    vgic_reg_t *reg = (vgic_reg_t *) vgic->registers;
    return reg->rdist[vcpu_idx];
}

static inline struct gic_rdist_sgi_ppi_map *vgic_priv_get_rdist_sgi(struct device *d, seL4_Word vcpu_idx)
{
    assert(d);
    assert(d->priv);
    vgic_t *vgic = vgic_device_get_vgic(d);
    vgic_reg_t *reg = (vgic_reg_t *) vgic->registers;
    return reg->sgi[vcpu_idx];
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

static int vgic_sgi_enable_irq(struct gic_rdist_sgi_ppi_map *gic_sgi, vgic_t *vgic, int irq, seL4_Word vcpu_idx)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        DDIST("\nsgi enabling irq %d\n", irq);
        sgi_set_enable(gic_sgi, irq, true);
        struct virq_handle *virq_data = virq_get_sgi_ppi(vgic, vcpu_idx, irq);
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

static int vgic_sgi_disable_irq(struct gic_rdist_sgi_ppi_map *gic_sgi, int irq)
{
    /* STATE g) */
    if (irq < GIC_SPI_IRQ_MIN) {
        DDIST("sgi disabling irq %d\n", irq);
        sgi_set_enable(gic_sgi, irq, false);
    }
    return 0;
}

static int vgic_sgi_set_pending_irq(struct device *d, vm_t *vm, int irq, seL4_Word vcpu_idx, bool masked)
{
    /* STATE c) */
    struct gic_rdist_sgi_ppi_map *gic_sgi = vgic_priv_get_rdist_sgi(d, vcpu_idx);
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    vgic_t *vgic = vgic_device_get_vgic(d);
    struct virq_handle *virq_data = virq_get_sgi_ppi(vgic, vcpu_idx, irq);
    /* If it is enables, inject the IRQ */
    if (virq_data && gic_dist_is_enabled(gic_dist) && sgi_is_enabled(gic_sgi, irq)) {
        int err;
        sgi_set_pending(gic_sgi, virq_data->virq, true);
        return vgic_vcpu_inject_irq(vgic, vm_get_vcpu(vm, vcpu_idx), virq_data, vcpu_idx, masked);
        //return 0;
    } else {
        /* No further action */
        DDIST("IRQ not enabled (%d) for %s\n", irq, vm->name);
    }

    return 0;
}

// TODO THis shouild be in the general faulit code
static bool is_valid_fault(fault_t *fault, uintptr_t reg)
{
    switch (fault_get_width(fault)) {
    case WIDTH_BYTE:
        return ((reg % sizeof(uint8_t)) == 0);
    case WIDTH_HALFWORD:
        return ((reg % sizeof(uint16_t)) == 0);
    case WIDTH_WORD:
        return ((reg % sizeof(uint32_t)) == 0);
    case WIDTH_DOUBLEWORD:
        return ((reg % sizeof(uint64_t)) == 0);
    default:
        return false;
    }
}

static int handle_read(fault_t *fault, uintptr_t reg)
{
    assert(is_valid_fault(fault, reg));
    switch (fault_get_width(fault)) {
    case WIDTH_BYTE:
        fault_set_data(fault, *((uint8_t *) reg));
        break;
    case WIDTH_HALFWORD:
        fault_set_data(fault, *((uint16_t *) reg));
        break;
    case WIDTH_WORD:
        fault_set_data(fault, *((uint32_t *) reg));
        break;
    case WIDTH_DOUBLEWORD:
        fault_set_data(fault, *((uint64_t *) reg));
        break;
    }

    return advance_fault(fault);

}

static void emulate_write(fault_t *fault, uintptr_t reg, int offset)
{
    uintptr_t data = fault_get_data(fault) & fault_get_data_mask(fault);
    int shift = 8 * (offset % sizeof(uint32_t));
    // TODO this is just fault emulate ??
    switch (fault_get_width(fault)) {
    case WIDTH_BYTE:
        *((uint8_t *) reg) = data >> shift;
        break;
    case WIDTH_HALFWORD:
        *((uint16_t *) reg) = data >> shift;
        break;
    case WIDTH_WORD:
        *((uint32_t *) reg) = data;
        break;
    case WIDTH_DOUBLEWORD:
        *((uint64_t *) reg) = data;
        break;
    }
}

static int handle_vgic_rdist_write(struct gic_rdist_map *gic_rdist, vgic_t *vgic, vm_t *vm, fault_t *fault,
                                   uintptr_t reg, int offset)
{
    enum gic_dist_action act = gic_rdist_get_action(offset);

    switch (act) {
    case ACTION_READONLY:
        ZF_LOGW("RDIST %lu: ignoring fault on offset 0x%x --> 0x%lx\n", fault->vcpu_idx, offset, fault_get_data(fault));
        emulate_write(fault, reg, offset);
        return ignore_fault(fault);
    case ACTION_UNKNOWN:
    default:
        printf("RDIST: unknown fault at offset %d\n", offset);
        return abandon_fault(fault);
    }
}

static int handle_vgic_rdist_sgi_write(struct gic_rdist_sgi_ppi_map *gic_rdist_sgi,
                                       vgic_t *vgic, vm_t *vm, fault_t *fault,
                                       uintptr_t reg, int offset)
{
    enum gic_dist_action act = gic_sgi_get_action(offset);

    /* most gicv3 regs are 32 bits - get a pointer to that  */
    uint32_t *reg32 = (uint32_t *) ALIGN_DOWN(reg, sizeof(uint32_t));
    uintptr_t data = fault_get_data(fault) & fault_get_data_mask(fault);

    switch (act) {
    case ACTION_READONLY:
        ZF_LOGW("RDIST_SGI %lu: ignoring fault on offset 0x%x --> 0x%lx\n", fault->vcpu_idx, offset, data);
        *reg32 = data;//emulate_write(fault, reg, offset);
        return ignore_fault(fault);
    case ACTION_ENABLE_SET: {
        /* Mask bits that are already set */
        // TODO this could be a 16 or 8 bit fault!?

        uint32_t set = fault_get_data(fault) & ~(*reg32);
        while (set) {
            int irq = CTZ(set);
            set &= ~(1U << irq);
            irq += (offset - 0x100) * 8;
            vgic_sgi_enable_irq(gic_rdist_sgi, vgic, irq, fault->vcpu_idx);
            DDIST("Enabled IRQ %d\n", irq);
        }
        assert(fault_get_width(fault) == WIDTH_WORD);
        int i = (offset - 0x100) / sizeof(uint32_t);
        gic_rdist_sgi->isenable[i] |= data;
        *reg32 |= data;
        assert(gic_rdist_sgi->isenable[i] == gic_rdist_sgi->icenable[i]);
        return ignore_fault(fault);
    }
    case ACTION_ENABLE_CLR: {
        /* Mask bits that are already clear */
        uint32_t clear = data & *reg32;
        while (clear) {
            int irq = CTZ(clear);
            clear &= ~(1U << irq);
            irq += (offset - 0x180) * 8;
            vgic_sgi_disable_irq(gic_rdist_sgi, irq);
        }
        /* clear the corresponding bits in the enable set register */
        *reg32 &= ~data;
        int i = (offset - 0x180) / sizeof(uint32_t);
        gic_rdist_sgi->icenable[i] &= ~data;
        assert(gic_rdist_sgi->isenable[i] == gic_rdist_sgi->icenable[i]);
        return ignore_fault(fault);
    }
    case ACTION_UNKNOWN:
    default:
        printf("SGI %lu: Unknown write on offset 0x%x --> 0x%lx\n", fault->vcpu_idx, offset, data);
    }
    abandon_fault(fault);
    return -1;
}


static int handle_vgic_dist_write(struct gic_dist_map *gic_dist, vgic_t *vgic, vm_t *vm,
                                  fault_t *fault, uintptr_t reg, int offset)
{
    uintptr_t data = fault_get_data(fault) & fault_get_data_mask(fault);
    uint32_t *reg32 = (uint32_t *) ALIGN_DOWN(reg, sizeof(uint32_t));

    switch (gic_dist_get_action(offset)) {
    case ACTION_READONLY:
    case ACTION_PASSTHROUGH:
        //TODO stop doing this for read only
        *reg32 = data;
        break;
    case ACTION_ENABLE:
        // TODO handle smaller writes
        if (data & GIC_ENABLED) {
            vgic_dist_enable(gic_dist);
        } else {
            vgic_dist_disable(gic_dist);
        }
        *reg32 = data & 0b10011; // TODO why
        break;
    case ACTION_ENABLE_SET: {
        /* Mask bits that are already set */
        uint32_t enable = data & ~(*reg32);
        while (enable) {
            int irq = CTZ(enable);
            enable &= ~(1U << irq);
            irq += (offset - 0x100) * 8;
            vgic_dist_enable_irq(vgic, gic_dist, irq);
            enable_irq(irq);
        }
        int i = (offset - 0x100) / sizeof(uint32_t);
        /* set the corresponding bits on the enable clear register */
        gic_dist->enable_set[i] |= data;
        *reg32 |= data;
        assert(gic_dist->enable_clr[i] == gic_dist->enable_set[i]);
        break;
    }
    case ACTION_ENABLE_CLR: {
        /* Mask bits that are already clear */
        uint32_t clear = data & (*reg32);
        while (clear) {
            int irq = CTZ(clear);
            clear &= ~(1U << irq);
            irq += (offset - 0x180) * 8;
            vgic_dist_disable_irq(gic_dist, irq);
            disable_irq(irq);
        }
        *reg32 &= ~data;
        int i = (offset - 0x180) / sizeof(uint32_t);
        gic_dist->enable_clr[i] &= ~data;
        assert(gic_dist->enable_clr[i] == gic_dist->enable_set[i]);
        break;
    }
    case ACTION_PENDING_SET: {
        /* Mask bits that are already set */
        uint32_t ps = data & ~(*reg32);
        while (ps) {
             int irq = CTZ(ps);
             ps &= ~(1U << irq);
             irq += (offset - 0x200) * 8;
             vgic_dist_set_pending_irq(vgic, vm_get_vcpu(vm, fault->vcpu_idx), irq,
                     fault->vcpu_idx, false);
        }
        break;
    }
    case ACTION_PENDING_CLR: {
        /* Mask bits that are already clear */
        uint32_t pc = data & *reg32;
        while (pc) {
             int irq = CTZ(pc);
             pc &= ~(1U << irq);
             irq += (offset - 0x280) * 8;
             vgic_dist_clr_pending_irq(gic_dist, irq);
        }
        break;
    }
    case ACTION_SGI:
        ZF_LOGE("vgic SGI not implemented!\n");
        return abandon_fault(fault);
    case ACTION_ROUTE: {
          uint64_t route = data;
          int irq = (offset - 0x6000) / sizeof(uint64_t);
          // TODO pass in a routing function for when this value changes
          if (route  != 0) {
             printf("Attempted to route irq %d to %p\n", irq, (void *) route);
          }
          break;
     }
    case ACTION_SET_TRIGGER: {
        assert(fault_get_width(fault) == WIDTH_WORD);
        /* Mask bits that are already set */
        uint32_t config = data & ~(*reg32);
        while (config) {
             /* get which icfgr register is being modified */
             int n = (offset - 0xC00) / sizeof(uint32_t);
             /* each 32 bit icfgr<n> register covers 16 irqs */
             n = n * 16;
             for (int i = 0; i < 16; i++) {
                 int irq = i + n;
                 set_trigger(irq, config & 0x3);
                 config = config >> 2;
             }
         }
         // TODO pass a function here to change the level
         // for our static system, hopefully it will just check if its correct or not
         emulate_write(fault, reg, offset);
         break;
    }
    default:
        printf("Unknown vgic dist fault\n");
        return abandon_fault(fault);
    }
    return ignore_fault(fault);
}

static int handle_vgic_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    int offset = fault_get_address(fault) - d->pstart;
    if (offset < 0 || offset >= d->size) {
        ZF_LOGF("%s offset %d out of range\n", d->name, offset);
        return -1;
    }

    vgic_t *vgic = vgic_device_get_vgic(d);
    /* address in the device that the guest was attempting to access */
    void *base;
    switch (d->devid) {
    case DEV_VGIC_DIST:
        base = priv_get_dist(vgic->registers);
        break;
    case DEV_VGIC_V3_REDIST:
        base = vgic_priv_get_rdist(d, fault->vcpu_idx);
        break;
    case DEV_VGIC_V3_REDIST_SGI:
        base = vgic_priv_get_rdist_sgi(d, fault->vcpu_idx);
        break;
    }

    uintptr_t reg = offset + (uintptr_t) base;
    if (unlikely(!is_valid_fault(fault, reg))) {
        ZF_LOGE("Invalid or unaligned fault addr %p\n", (void *) reg);
        return abandon_fault(fault);
    }

    if (fault_is_read(fault)) {
        if (fault_get_width(fault) == WIDTH_WORD) {
           ZF_LOGV("%s: read 0x%x: 0x%x\n", d->name, offset, *((uint32_t *) reg));
        } else if (fault_get_width(fault) == WIDTH_DOUBLEWORD) {
           ZF_LOGV("%s: read 0x%x: 0x%lx\n", d->name, offset, *((uint64_t *) reg));
        } else {
            ZF_LOGE("%s: Invalid fault width\n", d->name);
            assert(0);
        }
        return handle_read(fault, reg);
    } else {
        uint64_t data = fault_get_data(fault);
        if (fault_get_width(fault) == WIDTH_WORD) {
           ZF_LOGV("%s: write 0x%x: 0x%x\n", d->name, offset, (uint32_t) data);
        } else if (fault_get_width(fault) == WIDTH_DOUBLEWORD) {
           ZF_LOGV("%s: write 0x%x: 0x%lx\n", d->name, offset, data);
        } else {
            ZF_LOGE("%s: Invalid fault width\n", d->name);
        }
    }

    switch (d->devid) {
    case DEV_VGIC_DIST:
        return handle_vgic_dist_write(base, vgic, vm, fault, reg, offset);
    case DEV_VGIC_V3_REDIST:
        return handle_vgic_rdist_write(base, vgic, vm, fault, reg, offset);
    case DEV_VGIC_V3_REDIST_SGI:
        return handle_vgic_rdist_sgi_write(base, vgic, vm, fault, reg, offset);
    default:
        ZF_LOGE("unknown device id %d\n", d->devid);
        break;
    }

    return ignore_fault(fault);
}

static void vgic_dist_reset(struct device *d)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    memset(gic_dist, 0, sizeof(*gic_dist));

    gic_dist->typer            = 0x7B0418; /* RO */ // TODO yanyan had 0x7b0418
    //gic_dist->typer            = 0x7B04B0; /* RO */ // TODO yanyan had 0x7b0418
    gic_dist->iidr             = 0x0043B ; /* RO */ // TODO yanyan had 0x43b
    //gic_dist->iidr             = 0x1043B ; /* RO */ // TODO yanyan had 0x43b
    //todo yanyan had gic_dist->pidr = 0x30?

    gic_dist->enable_set[0]    = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]    = 0x0000ffff; /* 16bit RO */

    gic_dist->config[0]        = 0xaaaaaaaa; /* RO */

  //  gic_dist->pidrn[0]         = 0x44;     /* RO */
    gic_dist->pidrn[6]         = 0x30;

    //   gic_dist->pidrn[4]         = 0x92;     /* RO */
 //   gic_dist->pidrn[5]         = 0xB4;     /* RO */
 //   gic_dist->pidrn[6]         = 0x3B;     /* RO */

//    gic_dist->cidrn[0]         = 0x0D;     /* RO */
//    gic_dist->cidrn[1]         = 0xF0;     /* RO */
//    gic_dist->cidrn[2]         = 0x05;     /* RO */
  //  gic_dist->cidrn[3]         = 0xB1;     /* RO */
}

static void vgic_rdist_reset(struct gic_rdist_map *gic_rdist)
{
    memset(gic_rdist, 0, sizeof(*gic_rdist));

    //gic_rdist->typer           = 0x1;      /* RO */
    //gic_rdist->iidr            = 0x1143B;  /* RO */ //TODO yanyan had 0x43B
    gic_rdist->iidr             = 0x0043B ; /* RO */ // TODO yanyan had 0x43b

  //  gic_rdist->pidr0           = 0x93;     /* RO */
   // gic_rdist->pidr1           = 0xB4;     /* RO */
    //gic_rdist->pidr2           = 0x3B;     /* RO */ //TODO yanyan had 0x30
    gic_rdist->pidr2           = 0x30;     /* RO */ //TODO yanyan had 0x30
 //   gic_rdist->pidr4           = 0x44;     /* RO */

   // gic_rdist->cidr0           = 0x0D;     /* RO */
   // gic_rdist->cidr1           = 0xF0;     /* RO */
   // gic_rdist->cidr2           = 0x05;     /* RO */
   // gic_rdist->cidr3           = 0xB1;     /* RO */
}

static void vgic_rdist_sgi_reset(struct gic_rdist_sgi_ppi_map *gic_sgi)
{
    memset(gic_sgi, 0, sizeof(*gic_sgi));
    gic_sgi->isactive[0]       = 0xaaaaaaaa;
}

            extern int cur_el[CONFIG_MAX_NUM_NODES];
/* public functions */
int vgic_vcpu_inject_irq(vgic_t *vgic, seL4_CPtr vcpu, struct virq_handle *irq, seL4_Word vcpu_idx, bool masked)
{
    if (vgic_lr_has_virq(vgic, vcpu_idx, irq) != -1) {
        return 0;
    }

    if (irq->virq > 16) {
    enable_irq(irq->virq);
    }
    if (!masked) {
        int i = vgic_find_free_irq(vgic, vcpu_idx);
        if (i >= 0 && i < 4) {
            assert(cur_el[vcpu_idx] == 1);
            assert(irq->virq != 379);
            //printf("Injected irq %d\n", irq->virq);
            seL4_Error err = seL4_ARM_VCPU_InjectIRQ(vcpu, irq->virq, 0, 1, i);
            assert(err == 0);
            vgic_shadow_irq(vgic, i, irq, vcpu_idx);
            return 0;
        } else {
            printf("VDIST: overflow due to out of lrs, %d\n", i);
        }
    } else {
        //printf("VDIST: overflow masked\n");
    }
    /* inject overflow */

    return vgic_add_overflow(vgic, irq, vcpu_idx);
}

int handle_vgic_maintenance(vm_t *vm, int idx, seL4_Word vcpu_idx)
{
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->lock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    assert(cur_el[vcpu_idx] == 1);
    /* STATE d) */
    assert(vcpu_idx < CONFIG_MAX_NUM_NODES);
    struct device *d = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);

    assert(d);
    vgic_t *vgic = vgic_device_get_vgic(d);

    // TODO cache LR
    for (int i = 0; i < 4; i++) {
        if (idx & BIT(i)) {
            assert(vgic->irq[vcpu_idx][i]);
            DIRQ("Maintenance IRQ %d\n", vgic->irq[vcpu_idx][i]->virq);
            if (vgic->irq[vcpu_idx][i]->virq >= GIC_SPI_IRQ_MIN) {
                vgic_dist_set_pending(gic_dist, vgic->irq[vcpu_idx][i]->virq, false);
            } else {
                sgi_set_pending(vgic_priv_get_rdist_sgi(d, vcpu_idx), vgic->irq[vcpu_idx][i]->virq, false);
            }
            /* Clear pending */
           virq_ack(vgic->irq[vcpu_idx][i]);
           vgic->irq[vcpu_idx][i] = NULL;
        }
    }


    extern int cur_el[CONFIG_MAX_NUM_NODES];
    if (cur_el[vcpu_idx] == 2) return 0;
    vgic_handle_overflow(vgic, vm_get_vcpu(vm, vcpu_idx), vcpu_idx);
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    return 0;
}

int vm_inject_IRQ(virq_handle_t virq, seL4_Word vcpu_idx, bool masked)
{
    assert(virq);
    vm_t *vm = virq->vm;

    DIRQ("VM received IRQ %d %d\n", virq->virq, masked);

    /* Grab a handle to the VGIC */
    struct device *vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    if (vgic_device == NULL) {
        return -1;
    }

    vgic_t *vgic = vgic_device_get_vgic(vgic_device);
    if (virq->virq >= GIC_SPI_IRQ_MIN) {
	    vgic_dist_set_pending_irq(vgic, vm_get_vcpu(vm, vcpu_idx), virq->virq, vcpu_idx, masked);
    } else {
        vgic_sgi_set_pending_irq(vgic_device, vm, virq->virq, vcpu_idx, masked);
    }

    fault_t *fault = vm->vcpus[vcpu_idx].fault;
    if (!fault_handled(fault) && fault_is_wfi(fault)) {
        ignore_fault(fault);
    }

    return 0;
}

virq_handle_t vm_virq_vcpu_new(vm_t *vm, seL4_Word vcpu_idx, int irq, void (*ack)(void *), void *token)
{
    if (irq < GIC_SGI_IRQ_MIN || irq > GIC_PPI_IRQ_MAX) {
        ZF_LOGE("Invalid PPI/SIG %d, must be in range (%d,%d]", irq, GIC_PPI_IRQ_MIN,
                GIC_SGI_IRQ_MAX);
        return NULL;
    }

    struct device *vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    vgic_t *vgic = vgic_device_get_vgic(vgic_device);

    assert(vgic_device != NULL);
    assert(vgic != NULL);

    return virq_new_sgi_ppi(vm, vgic, vcpu_idx, irq, ack, token);
}

/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vm_install_vgic(vm_t *vm)
{
    vgic_t *vgic = calloc(1, sizeof(*vgic));
    if (!vgic) {
        ZF_LOGE("Unable to malloc memory for VGIC");
        return -1;
    }

    vgic_reg_t *registers = calloc(1, sizeof(vgic_reg_t));
    vgic->registers = registers;
    if (!vgic->registers) {
        free(vgic);
        ZF_LOGE("Unable to malloc memory for VGIC");
        return -1;
    }

    /* Distributor */
    struct device dist = dev_vgic_dist;
    registers->dist = map_emulated_device(vm, &dev_vgic_dist);
    if (registers->dist == NULL) {
        goto out;
    }

    dist.priv = (void *)vgic;
    vgic_dist_reset(&dist);
    if (vm_add_device(vm, &dist)) {
        goto out;
    }

    /* Redistributor */
    struct device rdist = dev_vgic_redist;
    rdist.priv = (void *)vgic;
    struct device sgi = dev_vgic_redist_sgi;
    sgi.priv = (void *)vgic;
    for (uint64_t i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        registers->rdist[i] = map_emulated_device(vm, &rdist);
        if (registers->rdist == NULL) {
            goto out;
        }

        vgic_rdist_reset(registers->rdist[i]);
        if (vm_add_device(vm, &rdist)) {
            goto out;
        }

        /* Redistributor SGI */
        registers->sgi[i] = map_emulated_device(vm, &sgi);
        if (registers->sgi[i] == NULL) {
            goto out;
        }

        if (i == CONFIG_MAX_NUM_NODES - 1) {
            registers->rdist[i]->typer |= BIT(4);
        }
        registers->rdist[i]->typer |= BIT(24) | (i << 32 + 8);

        vgic_rdist_sgi_reset(registers->sgi[i]);
        if (vm_add_device(vm, &sgi)) {
            goto out;
        }

        sgi.pstart += sgi.size + rdist.size;
        rdist.pstart += sgi.size + rdist.size;
    }

    return 0;
out:
    free(vgic->registers);
    free(vgic);
    return -1;
}

const struct device dev_vgic_dist = {
    .devid = DEV_VGIC_DIST,
    .name = "vgic.distributor",
    .pstart = GIC_DIST_PADDR,
    .size = 0x10000,
    .handle_page_fault = &handle_vgic_fault,
    .priv = NULL,
};

const struct device dev_vgic_redist = {
    .devid = DEV_VGIC_V3_REDIST,
    .name = "vgic.redistributor",
    .pstart = GIC_REDIST_PADDR,
    .size = 0x10000,
    .handle_page_fault = &handle_vgic_fault,
    .priv = NULL,
};

const struct device dev_vgic_redist_sgi = {
    .devid = DEV_VGIC_V3_REDIST_SGI,
    .name = "vgic.redistributor_sgi",
    .pstart = GIC_REDIST_PADDR + GIC_SGI_OFFSET,
    .size = 0x10000,
    .handle_page_fault = &handle_vgic_fault,
    .priv = NULL,
};
