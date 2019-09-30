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

//#define DEBUG_IRQ
//#define DEBUG_DIST

#ifdef DEBUG_IRQ
#define DIRQ(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DIRQ(...) do{}while(0)
#endif

#ifdef DEBUG_DIST
#define DDIST(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DDIST(...) do{}while(0)
#endif

#define IRQ_IDX(irq) ((irq) / 32)
#define IRQ_BIT(irq) (1U << ((irq) % 32))

#define GIC_SGI_IRQ_MIN    0
#define GIC_SGI_IRQ_MAX   15
#define GIC_PPI_IRQ_MIN   16
#define GIC_PPI_IRQ_MAX   31
#define GIC_SPI_IRQ_MIN   32
#define GIC_SPI_IRQ_MAX   1019

#define not_pending(...) !is_pending(__VA_ARGS__)
#define not_active(...)  !is_active(__VA_ARGS__)
#define not_enabled(...) !is_enabled(__VA_ARGS__)

enum gic_dist_action {
    ACTION_READONLY,
    ACTION_PASSTHROUGH,
    ACTION_ENABLE,
    ACTION_ENABLE_SET,
    ACTION_ENABLE_CLR,
    ACTION_PENDING_SET,
    ACTION_PENDING_CLR,
    ACTION_SGI,
    ACTION_ROUTE,
    ACTION_SET_TRIGGER,
    ACTION_UNKNOWN
};

static inline enum gic_dist_action gic_dist_get_action(int offset)
{
    /* Handle the fault
     * The only fields we care about are enable_set/clr
     * We have 2 options for other registers:
     *  a) ignore writes and hope the VM acts appropriately
     *  b) allow write access so the VM thinks there is no problem,
     *     but do not honour them
     */
    if (0x000 <= offset && offset < 0x004) {     /* enable          */
        return ACTION_ENABLE;
    } else if (0x080 <= offset && offset < 0x100) { /* Security        */
        return ACTION_PASSTHROUGH;
    } else if (0x100 <= offset && offset < 0x180) { /* enable_set      */
        return ACTION_ENABLE_SET;
    } else if (0x180 <= offset && offset < 0x200) { /* enable_clr      */
        return ACTION_ENABLE_CLR;
    } else if (0x200 <= offset && offset < 0x280) { /* pending_set     */
        return ACTION_PENDING_SET;
    } else if (0x280 <= offset && offset < 0x300) { /* pending_clr     */
        return ACTION_PENDING_CLR;
    } else if (0x300 <= offset && offset < 0x380) { /* active          */
        return ACTION_READONLY;
    } else if (0x400 <= offset && offset < 0x7FC) { /* priority        */
        return ACTION_READONLY;
    } else if (0x800 <= offset && offset < 0x8FC) { /* targets         */
        return ACTION_READONLY;
    } else if (0xC00 <= offset && offset < 0xD00) { /* config          */
        return ACTION_SET_TRIGGER;
    } else if (0xD00 <= offset && offset < 0xD80) { /* spi config      */
        return ACTION_READONLY;
    } else if (0xDD4 <= offset && offset < 0xDD8) { /* legacy_int      */
        return ACTION_READONLY;
    } else if (0xDE0 <= offset && offset < 0xDE4) { /* match_d         */
        return ACTION_READONLY;
    } else if (0xDE4 <= offset && offset < 0xDE8) { /* enable_d        */
        return ACTION_READONLY;
    } else if (0xF00 <= offset && offset < 0xF04) { /* sgi_control     */
        return ACTION_PASSTHROUGH;
    } else if (0xF10 <= offset && offset < 0xF10) { /* sgi_pending_clr */
        return ACTION_SGI;
    } else if (0x6100 <= offset && offset < 0x7fe0) { /* irouter (gicv3 only) */
        return ACTION_ROUTE;
    } else {
        return ACTION_READONLY;
    }
    return ACTION_UNKNOWN;
}

extern const struct device dev_vgic_dist;

// this must be provided by files including this file, to select the appropriate
// gic dist per the hardware.
struct gic_dist_map;

static inline void vgic_dist_set_pending(struct gic_dist_map *gic_dist, int irq, int v)
{
    if (v) {
        gic_dist->pending_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int is_pending(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->pending_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline void set_enable(struct gic_dist_map *gic_dist, int irq, int v)
{
    if (v) {
        gic_dist->enable_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int is_enabled(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->enable_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int is_active(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->active[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int vgic_dist_enable(struct gic_dist_map *gic_dist)
{
    DDIST("enabling gic distributer\n");
    gic_dist_enable(gic_dist);
    return 0;
}

static inline int vgic_dist_disable(struct gic_dist_map *gic_dist)
{
    DDIST("disabling gic distributer\n");
    gic_dist_disable(gic_dist);
    return 0;
}

static inline int vgic_dist_enable_irq(vgic_t *vgic, struct gic_dist_map *gic_dist, int irq)
{
    struct virq_handle *virq_data = virq_find_irq_data(vgic, irq);
    DDIST("enabling irq %d\n", irq);
    set_enable(gic_dist, irq, true);
    if (virq_data) {
        /* STATE b) */
        if (not_pending(gic_dist, virq_data->virq)) {
            virq_ack(virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle", irq);
    }
    return 0;
}

static inline int vgic_dist_disable_irq(struct gic_dist_map *gic_dist, int irq)
{
    /* STATE g) */
    if (irq >= 32) {
        DDIST("disabling irq %d\n", irq);
        set_enable(gic_dist, irq, false);
    }
    return 0;
}

static inline int vgic_dist_set_pending_irq(vgic_t *vgic, seL4_CPtr vcpu, int irq, seL4_Word vcpu_idx, bool masked)
{
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->lock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    /* STATE c) */
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->registers);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, irq);
    /* If it is enables, inject the IRQ */
    if (virq_data && gic_dist_is_enabled(gic_dist) && is_enabled(gic_dist, irq)) {
        DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);

        vgic_dist_set_pending(gic_dist, virq_data->virq, true);
        int err = vgic_vcpu_inject_irq(vgic, vcpu, virq_data, vcpu_idx, masked);

#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
        vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
        return err;
    } else {
        /* No further action */
        DDIST("IRQ not enabled (%d)\n", irq);
    }

#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    return 0;
}

static inline int vgic_dist_clr_pending_irq(struct gic_dist_map *gic_dist, int irq)
{
    DDIST("clr pending irq %d\n", irq);
    vgic_dist_set_pending(gic_dist, irq, false);
    return 0;
}

extern void enable_irq(int irq);
extern void disable_irq(int irq);
extern void set_trigger(int irq, int trigger);

static inline int handle_vgic_dist_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    vgic_t *vgic = vgic_device_get_vgic(d);
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->registers);
    seL4_Word mask = fault_get_data_mask(fault);
    int offset = fault_get_address(fault) - d->pstart;

    uint8_t *reg8 = 0;
    uint16_t *reg16 = 0;
    uint32_t *reg32 = 0;
    uint64_t *reg64 = 0;
    uint64_t data;

    /* this is the exact address that the guest was attempting
     * to access */
    uintptr_t reg_addr = ((uintptr_t) gic_dist + offset);
    switch (fault_get_width(fault)) {
    case WIDTH_BYTE:
        reg8 = (uint8_t *) reg_addr;
        break;
    case WIDTH_HALFWORD:
        reg16 = (uint16_t *) reg_addr;
        break;
    case WIDTH_WORD:
        reg32 = (uint32_t *) reg_addr;
        break;
    case WIDTH_DOUBLEWORD:
        reg64 = (uint64_t *) reg_addr;
        break;
    default:
        ZF_LOGF("invalid fault width %d!!", fault_get_width(fault));
    }

    /* Out of range */
    if (offset < 0 || offset >= sizeof(struct gic_dist_map)) {
        DDIST("offset out of range %x %x\n", offset, sizeof(struct gic_dist_map));
        return ignore_fault(fault);

        /* Read fault */
    } else if (fault_is_read(fault)) {
        assert(0); // reads not trapped
        switch (fault_get_width(fault)) {
        case WIDTH_BYTE:
            data = *reg8;
            break;
        case WIDTH_HALFWORD:
            data = *reg16;
            break;
        case WIDTH_WORD:
            data = *reg32;
            break;
        case WIDTH_DOUBLEWORD:
            data = *reg64;
            break;
        }
        fault_set_data(fault, data);
        //printf("0x%lx DDIST: reading 0x%x as 0x%lx 0x%lx\n", fault_get_ctx(fault)->pc, offset, fault_get_data(fault), mask);
        return advance_fault(fault);
    } else {
        seL4_Word data = fault_get_data(fault) & fault_get_data_mask(fault);
        enum gic_dist_action act = gic_dist_get_action(offset);

        switch (act) {
        case ACTION_READONLY:
            assert(fault_get_width(fault) == WIDTH_WORD);
           //printf("%lu: Readonly write on offset 0x%x / width %x --> 0x%lx/0x%x\n", fault->vcpu_idx, offset, fault_get_width(fault), fault_get_data(fault), *reg32);
            *reg32 = data;
           return ignore_fault(fault);

        case ACTION_PASSTHROUGH:
           assert(fault_get_width(fault) == WIDTH_WORD);
           *reg32 = data;
           return ignore_fault(fault);
        case ACTION_ENABLE:
            if (data & GIC_ENABLED) {
                vgic_dist_enable(gic_dist);
            } else {
                vgic_dist_disable(gic_dist);
            }
            *reg32 = data & 0b10011;
            return ignore_fault(fault);
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
           *reg32 |= data;
           return ignore_fault(fault);
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
            *reg32 |= data;
            return ignore_fault(fault);
        }
        case ACTION_PENDING_SET: {
            /* Mask the data to write */
            /* Mask bits that are already set */
            uint32_t ps = data & ~(*reg32);
            while (ps) {
                int irq = CTZ(ps);
                ps &= ~(1U << irq);
                irq += (offset - 0x200) * 8;
                vgic_dist_set_pending_irq(vgic, vm_get_vcpu(vm, fault->vcpu_idx), irq,
                        fault->vcpu_idx, false);
            }
            return ignore_fault(fault);
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
            return ignore_fault(fault);
        }
        case ACTION_SGI:
            ZF_LOGF("vgic SGI not implemented!\n");
             return ignore_fault(fault);
        case ACTION_ROUTE: {
             uint64_t route = data;
             int irq = (offset - 0x6000) / sizeof(uint64_t);
             // TODO implement
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
                    set_trigger(irq, (config & 0x3) &0x1);
                    config = config >> 2;
                }
            }
            break;
        }
        case ACTION_UNKNOWN:
        default:
            assert(0);
  //          printf("%lu: Unknown write on offset 0x%x -> 0x%lx\n", fault->vcpu_idx, offset, data);
        }

        int shift = 8 * (offset % 4);
        switch (fault_get_width(fault)) {
        case WIDTH_BYTE:
            *((uint8_t *) reg_addr) = data >> shift;
            break;
        case WIDTH_HALFWORD:
            *((uint16_t *) reg_addr) = data >> shift;
            break;
        case WIDTH_WORD:
            *((uint32_t *) reg_addr) = data;
            break;
        case WIDTH_DOUBLEWORD:
            *((uint64_t *) reg_addr) = data;
            break;
        }
        return ignore_fault(fault);
    }
    abandon_fault(fault);
    return -1;
}
