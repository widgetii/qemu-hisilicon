/*
 * IVE XNN (Neural Network) Private API — Reverse-Engineered Header
 *
 * These structures and functions are not documented in any public SDK header.
 * Reconstructed from ARM disassembly of libive.a (Hi3516EV200 SDK V1.0.1.2)
 * and error message string analysis of hi3516ev200_ive.ko kernel module.
 *
 * The XNN engine is a CNN inference accelerator embedded within the IVE
 * hardware block on Hi3516EV200/EV300. It shares /dev/ive with the
 * standard IVE pixel ops (Thresh, Sobel, CCL, etc.)
 *
 * Hardware-accelerated layer types: Conv, FC, Eltwise, Flatten, Preproc, DMA
 * Model format: .oms (proprietary, contains quantized weights + layer descriptors)
 *
 * Source: arm-none-eabi-objdump -d libive.a (mpi_ive.o, check_param_user.o)
 */
#ifndef IVE_XNN_H
#define IVE_XNN_H

#include "hi_type.h"

/*
 * Memory descriptor — same layout as hi_ivp_mem_info (24 bytes)
 * Used for model buffer and temp buffer
 */
typedef struct {
    HI_U64 phys_addr;
    HI_U64 virt_addr;
    HI_U32 size;
    HI_U32 reserved;
} IVE_XNN_MEM_S;

/*
 * Blob descriptor — 48 bytes (0x30)
 * Used for input/output tensors in forward pass.
 * Max 16 src blobs, 16 dst blobs.
 *
 * Decoded from LD_PRELOAD ioctl hook capturing live hi_ivp_process_ex() calls.
 * First forward_slice blob for 640x360 YUV420SP input:
 *   08: type=2  0C: w=640  10: virt  14: ???  18: phys=0x42480000
 *   20: num=1   28: stride=640  2C: h=360  30: channels=3
 */
typedef struct {
    HI_U32 blob_type;      /* 0x00: data type enum (2 = YUV420SP?) */
    HI_U32 u32Width;       /* 0x04: width in pixels */
    HI_U32 u32VirAddr;     /* 0x08: virtual address (32-bit on ARM32) */
    HI_U32 reserved0;      /* 0x0C: unknown (second virt addr?) */
    HI_U64 u64PhyAddr;     /* 0x10: physical address (64-bit) */
    HI_U32 u32Num;         /* 0x18: number of images / batch (1) */
    HI_U32 reserved1;      /* 0x1C: padding */
    HI_U32 u32Stride;      /* 0x20: row stride in bytes */
    HI_U32 u32Height;      /* 0x24: height in pixels */
    HI_U32 u32Chn;         /* 0x28: channels (3 for YUV) */
    HI_U32 reserved2;      /* 0x2C: padding */
} IVE_XNN_BLOB_S; /* 48 bytes */

/*
 * Forward control — 8 bytes minimum
 * Offset 0: src_num (u32, range [1,16])
 * Offset 4: dst_num (u32, range [1,16])
 *
 * From forward param check: ctrl is 5th argument (on stack),
 * validated as: (ctrl->src_num-1) <= 15 && (ctrl->dst_num-1) <= 15
 */
typedef struct {
    HI_U32 src_num;
    HI_U32 dst_num;
} IVE_XNN_FORWARD_CTRL_S;

/*
 * Model params — 2160 bytes (0x870)
 * Filled by kernel during loadmodel ioctl.
 * Contains parsed layer descriptors, I/O node info, etc.
 *
 * Known field (from get_tmpbuf_size disassembly):
 *   At handle->field_8->offset_12: tmp buffer size
 */
typedef struct {
    HI_U8 data[2160]; /* opaque — kernel fills during load */
} IVE_XNN_MODEL_S;

/*
 * ioctl commands (from disassembly, _IOC encoding):
 *   loadmodel:     0xc8a04636 = _IOWR('F', 0x36, 2208)  — load OMS model
 *   forward:       0xc620463a = _IOWR('F', 0x3a, 1568)  — single forward pass
 *   forward_slice: 0xc9704638 = _IOWR('F', 0x38, 2416)  — sliced forward (used by IVP)
 *   query:         0x463c     = _IO('F', 0x3c)           — query completion
 *   finish:        0x463d     = _IO('F', 0x3d)           — finish/sync
 *   svp_init:      0x8010463b = _IOR('F', 0x3b, 16)      — map HW registers
 *   unloadmodel:   (nr ~0x37, not yet captured)
 *   preproc:       (nr ~0x39, not yet captured)
 *   open_dev:      0x801046c8 = _IOR('F', 0xc8, 16)      — open IVE device
 *   close:         0x46c9     = _IO('F', 0xc9)           — close/sync
 *   enable:        0x46ca     = _IO('F', 0xca)           — enable
 */

/*
 * API functions — defined in libive.a (static) and libive.so (shared).
 * No public header declares them. Declare them here for direct use.
 *
 * Calling conventions from disassembly:
 *
 * mpi_ive_xnn_loadmodel(model_mem, tmp_mem, model_params):
 *   r0 = IVE_XNN_MEM_S *model_mem   (OMS file in MMZ, 24 bytes)
 *   r1 = IVE_XNN_MEM_S *tmp_mem     (temp buffer in MMZ, 24 bytes)
 *   r2 = IVE_XNN_MODEL_S *model     (output: filled by kernel, 2160 bytes)
 *   Copies model_mem(24) + tmp_mem(24) + model(2160) into 2208-byte ioctl buf
 *   Returns: 0 on success
 *
 * mpi_ive_xnn_forward(handle_ptr, src_blobs, dst_blobs, ???, ctrl, instant):
 *   r0 = HI_S32 *handle_ptr         (in/out: IVE handle for query)
 *   r1 = IVE_XNN_BLOB_S *src_blobs  (array, ctrl->src_num entries)
 *   r2 = IVE_XNN_BLOB_S *dst_blobs  (array, ctrl->dst_num entries)
 *   r3 = ??? (possibly model handle or reserved)
 *   [sp+0] = IVE_XNN_FORWARD_CTRL_S *ctrl
 *   [sp+4] = HI_BOOL instant
 *   Builds 1584-byte ioctl buffer: src_blobs + dst_blobs + ctrl + instant
 *   Returns: 0 on success, handle written to *handle_ptr
 *
 * mpi_ive_xnn_get_tmpbuf_size(model, size_out):
 *   r0 = IVE_XNN_MODEL_S *model     (loaded model)
 *   r1 = HI_U32 *size_out           (output: required tmp buffer size)
 *   Reads model->internal_ptr->offset_12
 *   Purely userspace (no ioctl)
 *
 * mpi_ive_xnn_unloadmodel(model):
 *   r0 = IVE_XNN_MODEL_S *model
 *
 * mpi_ive_xnn_preproc(...):
 *   VGS-based input preprocessing (resize + CSC)
 *   Two modes: CPU preproc or VGS preproc (mutually exclusive)
 */

extern HI_S32 mpi_ive_xnn_loadmodel(IVE_XNN_MEM_S *model_mem,
                                     IVE_XNN_MEM_S *tmp_mem,
                                     IVE_XNN_MODEL_S *model);

extern HI_S32 mpi_ive_xnn_get_tmpbuf_size(IVE_XNN_MODEL_S *model,
                                            HI_U32 *size);

extern HI_S32 mpi_ive_xnn_forward(HI_S32 *handle,
                                   IVE_XNN_BLOB_S *src,
                                   IVE_XNN_BLOB_S *dst,
                                   void *reserved,
                                   IVE_XNN_FORWARD_CTRL_S *ctrl,
                                   HI_BOOL instant);

extern HI_VOID mpi_ive_xnn_unloadmodel(IVE_XNN_MODEL_S *model);

extern HI_S32 mpi_ive_xnn_preproc(void *args); /* TODO: full signature */

/* SVP algorithm init — maps IVE XNN hardware registers via /dev/mem */
extern HI_S32 mp_ive_svp_alg_proc_init(HI_U64 *phys_out, HI_U32 *handle_out);
extern HI_VOID mp_ive_svp_alg_proc_exit(void);

#endif /* IVE_XNN_H */
