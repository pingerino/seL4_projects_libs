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
#include "vm.h"

#include <string.h>

#include <elf/elf.h>
#include <vka/capops.h>

//#define DEBUG_COPYOUT
//#define DEBUG_COPYIN

#ifdef DEBUG_COPYOUT
#define DCOPYOUT(...) printf("copyout: " __VA_ARGS__)
#else
#define DCOPYOUT(...) do{}while(0)
#endif

#ifdef DEBUG_COPYIN
#define DCOPYIN(...) printf("copyin: " __VA_ARGS__)
#else
#define DCOPYIN(...) do{}while(0)
#endif



static int copy_out_page(vspace_t *dst_vspace, vspace_t *src_vspace, vka_t *vka, void *src, void *dst, size_t size)
{
    cspacepath_t dup_cap_path, cap_path;
    seL4_CPtr dup_cap, cap;
    void *tmp_dst;
    int offset;
    size_t copy_size;
    int bits;
    int err;

    /* Create a frame if necessary */
    cap = vspace_get_cap(dst_vspace, dst);
    if (cap == seL4_CapNull) {
        reservation_t res;
        vka_object_t frame;
        bits = 12;
        /* Create a frame */
        err = vka_alloc_frame(vka, 12, &frame);
        assert(!err);
        if (err) {
            return -1;
        }
        /* Map the frame to the dest vspace */
        res = vspace_reserve_range_at(dst_vspace, dst, BIT(bits), seL4_AllRights, 1);
        if (!res.res) {
            assert(res.res);
            return -1;
        }
        err = vspace_map_pages_at_vaddr(dst_vspace, &frame.cptr, NULL, dst, 1, bits, res);
        vspace_free_reservation(dst_vspace, res);
        if (err) {
            return -1;
        }
        cap = vspace_get_cap(dst_vspace, dst);
        assert(cap != seL4_CapNull);
    } else {
        bits = (int)vspace_get_cookie(dst_vspace, dst);
        if (bits == 0) {
            bits = 12;
        }
    }

    /* Copy the cap */
    err = vka_cspace_alloc_path(vka, &dup_cap_path);
    if (err) {
        return -1;
    }
    vka_cspace_make_path(vka, cap, &cap_path);
    dup_cap = dup_cap_path.capPtr;
    err = vka_cnode_copy(&dup_cap_path, &cap_path, seL4_AllRights);
    if (err) {
        vka_cspace_free(vka, dup_cap);
        return -1;
    }

    /* Map it */
    tmp_dst = vspace_map_pages(src_vspace, &dup_cap, NULL, seL4_AllRights, 1, bits, 1);
    assert(tmp_dst);
    if (!tmp_dst) {
        vka_cnode_delete(&dup_cap_path);
        vka_cspace_free(vka, dup_cap);
        return -1;
    }

    /* Copy the data to the frame */
    offset = (uintptr_t)dst & MASK(bits);
    copy_size = BIT(bits) - offset;
    if (copy_size > size) {
        copy_size = size;
    }
    memcpy(tmp_dst + offset, src, copy_size);
#if defined(CONFIG_ARCH_ARM) && !defined(CONFIG_PLAT_EXYNOS5)
    int error = seL4_ARM_Page_CleanInvalidate_Data(dup_cap, 0, BIT(bits));
    ZF_LOGF_IFERR(error, "seL4_ARM_Page_CleanInvalidate_Data failed");
#endif

    /* Clean up */
    vspace_unmap_pages(src_vspace, tmp_dst, 1, bits, VSPACE_PRESERVE);
    vka_cnode_delete(&dup_cap_path);
    vka_cspace_free(vka, dup_cap);

    DCOPYOUT("copied out page 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dst, copy_size);


    /* Done */
    return copy_size;
}

static int copy_out(vspace_t *dst_vspace, vspace_t *src_vspace, vka_t *vka, void *src, uintptr_t dest, size_t size)
{
    DCOPYOUT("copy out 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dest, size);
    while (size) {
        int seg_size;
        seg_size = copy_out_page(dst_vspace, src_vspace, vka, src, (void *)dest, size);
        assert(seg_size > 0);
        if (seg_size <= 0) {
            return -1;
        }
        dest += seg_size;
        src += seg_size;
        size -= seg_size;
    }
    return 0;
}


int vm_copyout(vm_t *vm, void *data, uintptr_t address, size_t size)
{
    return copy_out(vm_get_vspace(vm), vm->vmm_vspace, vm->vka, data, address, size);
}

void *vm_copyout_elf(vm_t *vm, elf_t *elf_file)
{
    uintptr_t entry;
    size_t num_headers;
    size_t i;

    entry = elf_getEntryPoint(elf_file);
    if (entry == 0) {
        return NULL;
    }

    num_headers = elf_getNumProgramHeaders(elf_file);
    for (i = 0; i < num_headers; i++) {
        if (elf_getProgramHeaderType(elf_file, i) == PT_LOAD) {
            unsigned long size, ipa;
            char *data;
            int err;

            data = elf_getProgramSegment(elf_file, i);
            size = elf_getProgramHeaderFileSize(elf_file, i);
            ipa = elf_getProgramHeaderVaddr(elf_file, i);

            err = vm_copyout(vm, data, ipa, size);
            if (err) {
                return NULL;
            }
        }
    }
    return (void *)entry;
}


static int copy_in_page(vspace_t *vmm_vspace, vspace_t *vm_vspace, vka_t *vka, void *dest, void *src, size_t size)
{
    seL4_CPtr cap, vmm_cap;
    cspacepath_t cap_path, vmm_cap_path;
    void *tmp_src;
    int offset;
    size_t copy_size;
    int bits;
    int err;

    /* Find the VM frame */
    cap = vspace_get_cap(vm_vspace, src);
    if (cap == seL4_CapNull) {
        return -1;
    }
    bits = vspace_get_cookie(vm_vspace, src);
    if (bits == 0) {
        bits = 12;
    }
    vka_cspace_make_path(vka, cap, &cap_path);

    /* Copy the cap so that we can map it into the VMM */
    err = vka_cspace_alloc_path(vka, &vmm_cap_path);
    vmm_cap = vmm_cap_path.capPtr;
    if (err) {
        printf("Failed to allocate slot for copyin\n");
        return -1;
    }
    err = vka_cnode_copy(&vmm_cap_path, &cap_path, seL4_AllRights);
    if (err) {
        vka_cspace_free(vka, vmm_cap);
        printf("Failed to copy frame cap for copyin\n");
        return -1;
    }
    /* Map it into the VMM vspace */
    tmp_src = vspace_map_pages(vmm_vspace, &vmm_cap, NULL, seL4_AllRights, 1, bits, 1);
    assert(tmp_src);
    if (tmp_src == NULL) {
        assert(!"Failed to map frame for copyin\n");
        vka_cnode_delete(&vmm_cap_path);
        vka_cspace_free(vka, vmm_cap);
        printf("Failed to map frame cap for copyin\n");
        return -1;
    }

    /* Copy the data from the frame */
    offset = (uintptr_t)src & MASK(bits);
    copy_size = BIT(bits) - offset;
    if (copy_size > size) {
        copy_size = size;
    }
    memcpy(dest, tmp_src + offset, copy_size);

    /* Clean up */
    vspace_unmap_pages(vmm_vspace, tmp_src, 1, bits, VSPACE_PRESERVE);
    vka_cnode_delete(&vmm_cap_path);
    vka_cspace_free(vka, vmm_cap);

    DCOPYIN("copy in page 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dest, copy_size);
    /* Done */
    return copy_size;
}

static int copy_in(vspace_t *dst_vspace, vspace_t *src_vspace, vka_t *vka, void *dest, uintptr_t src, size_t size)
{
    DCOPYIN("copy in 0x%x->0x%x (0x%x bytes)\n", (uint32_t)src, (uint32_t)dest, size);
    while (size) {
        int seg_size;
        seg_size = copy_in_page(dst_vspace, src_vspace, vka, dest, (void *)src, size);
        assert(seg_size > 0);
        if (seg_size <= 0) {
            return -1;
        }
        dest += seg_size;
        src += seg_size;
        size -= seg_size;
    }
    return 0;
}

int vm_copyin(vm_t *vm, void *data, uintptr_t address, size_t size)
{
    return copy_in(vm->vmm_vspace, vm_get_vspace(vm), vm->vka, data, address, size);
}

