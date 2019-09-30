#ifndef SEL4ARM_VMM_SDM845_MAP_H
#define SEL4ARM_VMM_SDM845_MAP_H

/* Give the VM 1 GiB memory; remember to change the value in device tree
 * if you change the values here.
 * */
/***** Physical Map ****/
#define RAM_BASE  0x80000000 //0x80000000 //0x140000000 //0x80000000
//#define RAM_END   0xff000000 //0xf0000000 //0xc0000000 //0x170000000 //0xb0000000
#define RAM_END  (0x180000000)
#define RAM_SIZE (RAM_END - RAM_BASE)

#endif /* SEL4ARM_VMM_SDM845_MAP_H */
