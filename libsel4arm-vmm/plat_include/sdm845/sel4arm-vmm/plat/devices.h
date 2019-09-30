#ifndef SEL4ARM_VMM_SDM845_DEVICES_H
#define SEL4ARM_VMM_SDM845_DEVICES_H

#include <sel4arm-vmm/plat/device_map.h>
#include <sel4arm-vmm/vm.h>

#define GIC_DIST_PADDR   0x17a00000
#define GIC_REDIST_PADDR   0x17a60000

#define MAX_VIRQS  1000
extern const struct device dev_vram;

#endif
