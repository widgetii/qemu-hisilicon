/*
 * IVE Byte-Exact Capture — dumps full output for ALL remaining ops.
 * Transfer via base64 to avoid SSH binary corruption.
 *
 * Build: same as test-ive-mpi.c
 * Run: ./test-ive-bytecmp 2>/tmp/log && base64 /tmp/ive_cap.bin
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

int memcpy_s(void *d, size_t dn, const void *s, size_t n) { memcpy(d,s,n<dn?n:dn); return 0; }
int memset_s(void *d, size_t dn, int c, size_t n) { memset(d,c,n<dn?n:dn); return 0; }
int memmove_s(void *d, size_t dn, const void *s, size_t n) { memmove(d,s,n<dn?n:dn); return 0; }
int strncpy_s(char *d, size_t dn, const char *s, size_t n) { strncpy(d,s,n<dn?n:dn); return 0; }
int snprintf_s(char *d, size_t dn, size_t n, const char *f, ...) { va_list a; va_start(a,f); int r=vsnprintf(d,dn,f,a); va_end(a); return r; }
const unsigned short int *__ctype_b;
size_t _stdlib_mb_cur_max(void) { return 0; }
int __fputc_unlocked(int c, FILE *s) { return fputc(c,s); }
int __fgetc_unlocked(FILE *s) { return fgetc(s); }

#include "hi_type.h"
#include "hi_common.h"
#include "hi_comm_ive.h"
#include "hi_ive.h"
#include "mpi_sys.h"
#include "mpi_ive.h"

#define W 64
#define H 64
#define SZ (W*H)
#define STRIDE 64

static void ive_wait(IVE_HANDLE h) {
    HI_BOOL f; do {} while(HI_MPI_IVE_Query(h,&f,HI_TRUE)==HI_ERR_IVE_QUERY_TIMEOUT);
}

static void flush_read(IVE_IMAGE_S *img) {
    HI_MPI_SYS_MmzFlushCache(img->au64PhyAddr[0],
        (HI_VOID*)(HI_UL)img->au64VirAddr[0], img->au32Stride[0] * H);
}

int main() {
    HI_S32 ret;
    IVE_HANDLE handle;
    FILE *fp = fopen("/tmp/ive_cap.bin", "wb");
    if (!fp) return 1;

    ret = HI_MPI_SYS_Init();
    if (ret) { fprintf(stderr, "SYS_Init: 0x%x\n", ret); return 1; }

    /* Source images */
    IVE_IMAGE_S src1, src2, dst;
    memset(&src1,0,sizeof(src1)); memset(&src2,0,sizeof(src2)); memset(&dst,0,sizeof(dst));
    src1.enType = src2.enType = dst.enType = IVE_IMAGE_TYPE_U8C1;
    src1.u32Width = src2.u32Width = dst.u32Width = W;
    src1.u32Height = src2.u32Height = dst.u32Height = H;
    src1.au32Stride[0] = src2.au32Stride[0] = dst.au32Stride[0] = STRIDE;
    HI_MPI_SYS_MmzAlloc(&src1.au64PhyAddr[0],(HI_VOID**)&src1.au64VirAddr[0],NULL,NULL,STRIDE*H);
    HI_MPI_SYS_MmzAlloc(&src2.au64PhyAddr[0],(HI_VOID**)&src2.au64VirAddr[0],NULL,NULL,STRIDE*H);
    HI_MPI_SYS_MmzAlloc(&dst.au64PhyAddr[0],(HI_VOID**)&dst.au64VirAddr[0],NULL,NULL,STRIDE*H);

    uint8_t *sv1 = (uint8_t*)(HI_UL)src1.au64VirAddr[0];
    uint8_t *sv2 = (uint8_t*)(HI_UL)src2.au64VirAddr[0];
    for (int y=0;y<H;y++) for (int x=0;x<W;x++) {
        sv1[y*STRIDE+x] = ((y*W+x)*7+13) & 0xFF;
        sv2[y*STRIDE+x] = ((y*W+x)*11+31) & 0xFF;
    }
    HI_MPI_SYS_MmzFlushCache(src1.au64PhyAddr[0], sv1, STRIDE*H);
    HI_MPI_SYS_MmzFlushCache(src2.au64PhyAddr[0], sv2, STRIDE*H);

    fwrite("BYT3", 4, 1, fp);

    /* === HIST (256 U32 bins) === */
    {
        IVE_MEM_INFO_S hist_mem; memset(&hist_mem,0,sizeof(hist_mem));
        hist_mem.u32Size = 256 * sizeof(HI_U32);
        HI_MPI_SYS_MmzAlloc(&hist_mem.u64PhyAddr,(HI_VOID**)&hist_mem.u64VirAddr,NULL,NULL,hist_mem.u32Size);
        ret = HI_MPI_IVE_Hist(&handle, &src1, (IVE_DST_MEM_INFO_S*)&hist_mem, HI_TRUE);
        if (ret==0) ive_wait(handle);
        HI_MPI_SYS_MmzFlushCache(hist_mem.u64PhyAddr,(HI_VOID*)(HI_UL)hist_mem.u64VirAddr,hist_mem.u32Size);
        fwrite("HST1", 4, 1, fp);
        fwrite((void*)(HI_UL)hist_mem.u64VirAddr, 1, 256*4, fp);
        fprintf(stderr, "hist done\n");
        HI_MPI_SYS_MmzFree(hist_mem.u64PhyAddr,(HI_VOID*)(HI_UL)hist_mem.u64VirAddr);
    }

    /* === INTEG (U64C1 output, COMBINE mode) === */
    {
        IVE_IMAGE_S integ_dst; memset(&integ_dst,0,sizeof(integ_dst));
        integ_dst.enType = IVE_IMAGE_TYPE_U64C1;
        integ_dst.u32Width = W; integ_dst.u32Height = H;
        integ_dst.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&integ_dst.au64PhyAddr[0],(HI_VOID**)&integ_dst.au64VirAddr[0],
                             NULL,NULL,STRIDE*H*sizeof(uint64_t));
        IVE_INTEG_CTRL_S ic = {.enOutCtrl = IVE_INTEG_OUT_CTRL_COMBINE};
        ret = HI_MPI_IVE_Integ(&handle, &src1, &integ_dst, &ic, HI_TRUE);
        if (ret==0) ive_wait(handle);
        HI_MPI_SYS_MmzFlushCache(integ_dst.au64PhyAddr[0],
            (HI_VOID*)(HI_UL)integ_dst.au64VirAddr[0], STRIDE*H*sizeof(uint64_t));
        uint64_t *iv = (uint64_t*)(HI_UL)integ_dst.au64VirAddr[0];
        fwrite("INT1", 4, 1, fp);
        for (int y=0;y<H;y++) fwrite(iv+y*STRIDE, sizeof(uint64_t), W, fp);
        fprintf(stderr, "integ done, sum[63,63]=%llu\n", (unsigned long long)iv[(H-1)*STRIDE+(W-1)]);
        HI_MPI_SYS_MmzFree(integ_dst.au64PhyAddr[0],(HI_VOID*)(HI_UL)integ_dst.au64VirAddr[0]);
    }

    /* === MAG_AND_ANG (U16C1 magnitude, MAG only) === */
    {
        IVE_IMAGE_S mag_dst; memset(&mag_dst,0,sizeof(mag_dst));
        mag_dst.enType = IVE_IMAGE_TYPE_U16C1;
        mag_dst.u32Width = W; mag_dst.u32Height = H;
        mag_dst.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&mag_dst.au64PhyAddr[0],(HI_VOID**)&mag_dst.au64VirAddr[0],
                             NULL,NULL,STRIDE*H*sizeof(uint16_t));
        HI_S8 mask[25]={0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        IVE_MAG_AND_ANG_CTRL_S mc = {.enOutCtrl=IVE_MAG_AND_ANG_OUT_CTRL_MAG, .u16Thr=0};
        memcpy(mc.as8Mask, mask, 25);
        ret = HI_MPI_IVE_MagAndAng(&handle, &src1, &mag_dst, HI_NULL, &mc, HI_TRUE);
        if (ret==0) ive_wait(handle);
        HI_MPI_SYS_MmzFlushCache(mag_dst.au64PhyAddr[0],
            (HI_VOID*)(HI_UL)mag_dst.au64VirAddr[0], STRIDE*H*sizeof(uint16_t));
        uint16_t *mv = (uint16_t*)(HI_UL)mag_dst.au64VirAddr[0];
        fwrite("MAG1", 4, 1, fp);
        for (int y=0;y<H;y++) fwrite(mv+y*STRIDE, sizeof(uint16_t), W, fp);
        fprintf(stderr, "mag_ang done, mag[5,5]=%d\n", mv[5*STRIDE+5]);
        HI_MPI_SYS_MmzFree(mag_dst.au64PhyAddr[0],(HI_VOID*)(HI_UL)mag_dst.au64VirAddr[0]);
    }

    /* === NORMGRAD (S8C1 horizontal output) === */
    {
        IVE_IMAGE_S ng_h, ng_v; memset(&ng_h,0,sizeof(ng_h)); memset(&ng_v,0,sizeof(ng_v));
        ng_h.enType = ng_v.enType = IVE_IMAGE_TYPE_S8C1;
        ng_h.u32Width = ng_v.u32Width = W; ng_h.u32Height = ng_v.u32Height = H;
        ng_h.au32Stride[0] = ng_v.au32Stride[0] = STRIDE;
        HI_MPI_SYS_MmzAlloc(&ng_h.au64PhyAddr[0],(HI_VOID**)&ng_h.au64VirAddr[0],NULL,NULL,STRIDE*H);
        HI_MPI_SYS_MmzAlloc(&ng_v.au64PhyAddr[0],(HI_VOID**)&ng_v.au64VirAddr[0],NULL,NULL,STRIDE*H);
        HI_S8 ng_mask[25]={0,0,0,0,0, 0,-1,0,1,0, 0,-2,0,2,0, 0,-1,0,1,0, 0,0,0,0,0};
        IVE_NORM_GRAD_CTRL_S ngc = {.enOutCtrl=IVE_NORM_GRAD_OUT_CTRL_HOR_AND_VER, .u8Norm=3};
        memcpy(ngc.as8Mask, ng_mask, 25);
        ret = HI_MPI_IVE_NormGrad(&handle, &src1, &ng_h, &ng_v, NULL, &ngc, HI_TRUE);
        if (ret==0) ive_wait(handle);
        HI_MPI_SYS_MmzFlushCache(ng_h.au64PhyAddr[0],(HI_VOID*)(HI_UL)ng_h.au64VirAddr[0],STRIDE*H);
        int8_t *nv = (int8_t*)(HI_UL)ng_h.au64VirAddr[0];
        fwrite("NRG1", 4, 1, fp);
        for (int y=0;y<H;y++) fwrite(nv+y*STRIDE, 1, W, fp);
        fprintf(stderr, "normgrad done, h[5,5]=%d\n", nv[5*STRIDE+5]);
        HI_MPI_SYS_MmzFree(ng_h.au64PhyAddr[0],(HI_VOID*)(HI_UL)ng_h.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(ng_v.au64PhyAddr[0],(HI_VOID*)(HI_UL)ng_v.au64VirAddr[0]);
    }

    /* === GMM2 (30 frames: 20 bg + 10 fg, output fg mask) === */
    {
        int MODEL_NUM = 3;
        IVE_IMAGE_S gsrc, gfactor, gfg, gbg, gmatch;
        memset(&gsrc,0,sizeof(gsrc)); memset(&gfactor,0,sizeof(gfactor));
        memset(&gfg,0,sizeof(gfg)); memset(&gbg,0,sizeof(gbg)); memset(&gmatch,0,sizeof(gmatch));
        gsrc.enType=gfg.enType=gbg.enType=gmatch.enType=IVE_IMAGE_TYPE_U8C1;
        gfactor.enType=IVE_IMAGE_TYPE_U16C1;
        gsrc.u32Width=gfg.u32Width=gbg.u32Width=gmatch.u32Width=gfactor.u32Width=W;
        gsrc.u32Height=gfg.u32Height=gbg.u32Height=gmatch.u32Height=gfactor.u32Height=H;
        gsrc.au32Stride[0]=gfg.au32Stride[0]=gbg.au32Stride[0]=gmatch.au32Stride[0]=gfactor.au32Stride[0]=STRIDE;
        HI_MPI_SYS_MmzAlloc(&gsrc.au64PhyAddr[0],(HI_VOID**)&gsrc.au64VirAddr[0],NULL,NULL,STRIDE*H);
        HI_MPI_SYS_MmzAlloc(&gfactor.au64PhyAddr[0],(HI_VOID**)&gfactor.au64VirAddr[0],NULL,NULL,STRIDE*H*2);
        HI_MPI_SYS_MmzAlloc(&gfg.au64PhyAddr[0],(HI_VOID**)&gfg.au64VirAddr[0],NULL,NULL,STRIDE*H);
        HI_MPI_SYS_MmzAlloc(&gbg.au64PhyAddr[0],(HI_VOID**)&gbg.au64VirAddr[0],NULL,NULL,STRIDE*H);
        HI_MPI_SYS_MmzAlloc(&gmatch.au64PhyAddr[0],(HI_VOID**)&gmatch.au64VirAddr[0],NULL,NULL,STRIDE*H);
        /* Factor: sns=8, life=4 */
        uint8_t *fv=(uint8_t*)(HI_UL)gfactor.au64VirAddr[0];
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) { fv[(y*STRIDE+x)*2]=8; fv[(y*STRIDE+x)*2+1]=4; }
        HI_MPI_SYS_MmzFlushCache(gfactor.au64PhyAddr[0],fv,STRIDE*H*2);
        /* Model: zeroed */
        IVE_MEM_INFO_S gmodel; memset(&gmodel,0,sizeof(gmodel));
        gmodel.u32Size = MODEL_NUM*8*W*H;
        HI_MPI_SYS_MmzAlloc(&gmodel.u64PhyAddr,(HI_VOID**)&gmodel.u64VirAddr,NULL,NULL,gmodel.u32Size);
        memset((void*)(HI_UL)gmodel.u64VirAddr,0,gmodel.u32Size);
        HI_MPI_SYS_MmzFlushCache(gmodel.u64PhyAddr,(HI_VOID*)(HI_UL)gmodel.u64VirAddr,gmodel.u32Size);
        IVE_GMM2_CTRL_S gc; memset(&gc,0,sizeof(gc));
        gc.u8ModelNum=3; gc.u16VarRate=1; gc.u9q7MaxVar=(16*16)<<7; gc.u9q7MinVar=(8*8)<<7;
        gc.u8GlbSnsFactor=8; gc.u16FreqThr=12000; gc.u16FreqInitVal=20000;
        gc.u16FreqAddFactor=0xEF; gc.u16FreqReduFactor=0xFF00; gc.u16LifeThr=5000;
        gc.u16GlbLifeUpdateFactor=4;
        gc.enSnsFactorMode=IVE_GMM2_SNS_FACTOR_MODE_PIX;
        gc.enLifeUpdateFactorMode=IVE_GMM2_LIFE_UPDATE_FACTOR_MODE_GLB;
        uint8_t *gsv=(uint8_t*)(HI_UL)gsrc.au64VirAddr[0];
        /* Phase 1: 20 frames uniform 128 */
        for(int f=0;f<20;f++) {
            memset(gsv,128,STRIDE*H);
            HI_MPI_SYS_MmzFlushCache(gsrc.au64PhyAddr[0],gsv,STRIDE*H);
            gc.u16GlbLifeUpdateFactor = 0xFFFF/(f+1);
            HI_MPI_IVE_GMM2(&handle,&gsrc,&gfactor,&gfg,&gbg,&gmatch,&gmodel,&gc,HI_TRUE);
            ive_wait(handle);
        }
        /* Phase 2: 10 frames with bright spot */
        gc.u16GlbLifeUpdateFactor = 4;
        for(int f=0;f<10;f++) {
            for(int y=0;y<H;y++) for(int x=0;x<W;x++)
                gsv[y*STRIDE+x] = (x>=24 && x<40 && y>=24 && y<40) ? 255 : 128;
            HI_MPI_SYS_MmzFlushCache(gsrc.au64PhyAddr[0],gsv,STRIDE*H);
            HI_MPI_IVE_GMM2(&handle,&gsrc,&gfactor,&gfg,&gbg,&gmatch,&gmodel,&gc,HI_TRUE);
            ive_wait(handle);
        }
        HI_MPI_SYS_MmzFlushCache(gfg.au64PhyAddr[0],(HI_VOID*)(HI_UL)gfg.au64VirAddr[0],STRIDE*H);
        uint8_t *fgv=(uint8_t*)(HI_UL)gfg.au64VirAddr[0];
        fwrite("GMM1", 4, 1, fp);
        for(int y=0;y<H;y++) fwrite(fgv+y*STRIDE, 1, W, fp);
        int fg_in=0,fg_out=0;
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) {
            if(fgv[y*STRIDE+x]>0) { if(x>=24&&x<40&&y>=24&&y<40) fg_in++; else fg_out++; }
        }
        fprintf(stderr, "gmm2 done, fg_in=%d fg_out=%d\n", fg_in, fg_out);
        HI_MPI_SYS_MmzFree(gsrc.au64PhyAddr[0],(HI_VOID*)(HI_UL)gsrc.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gfactor.au64PhyAddr[0],(HI_VOID*)(HI_UL)gfactor.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gfg.au64PhyAddr[0],(HI_VOID*)(HI_UL)gfg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gbg.au64PhyAddr[0],(HI_VOID*)(HI_UL)gbg.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmatch.au64PhyAddr[0],(HI_VOID*)(HI_UL)gmatch.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(gmodel.u64PhyAddr,(HI_VOID*)(HI_UL)gmodel.u64VirAddr);
    }

    /* === STCandiCorner + STCorner === */
    {
        IVE_IMAGE_S st_dst; memset(&st_dst,0,sizeof(st_dst));
        st_dst.enType=IVE_IMAGE_TYPE_U8C1; st_dst.u32Width=W; st_dst.u32Height=H;
        st_dst.au32Stride[0]=STRIDE;
        HI_MPI_SYS_MmzAlloc(&st_dst.au64PhyAddr[0],(HI_VOID**)&st_dst.au64VirAddr[0],NULL,NULL,STRIDE*H);
        IVE_MEM_INFO_S st_mem; memset(&st_mem,0,sizeof(st_mem));
        HI_U32 st_sz = 4*STRIDE*H+16;
        HI_MPI_SYS_MmzAlloc(&st_mem.u64PhyAddr,(HI_VOID**)&st_mem.u64VirAddr,NULL,NULL,st_sz);
        st_mem.u32Size=st_sz;
        IVE_ST_CANDI_CORNER_CTRL_S stcc={.stMem=st_mem,.u0q8QualityLevel=25};
        ret = HI_MPI_IVE_STCandiCorner(&handle,&src1,&st_dst,&stcc,HI_TRUE);
        if(ret==0) ive_wait(handle);
        IVE_DST_MEM_INFO_S corner_mem; memset(&corner_mem,0,sizeof(corner_mem));
        corner_mem.u32Size = 2+500*4;
        HI_MPI_SYS_MmzAlloc(&corner_mem.u64PhyAddr,(HI_VOID**)&corner_mem.u64VirAddr,NULL,NULL,corner_mem.u32Size);
        IVE_ST_CORNER_CTRL_S stcctrl={.u16MaxCornerNum=500,.u16MinDist=5};
        ret = HI_MPI_IVE_STCorner(&st_dst,&corner_mem,&stcctrl);
        HI_MPI_SYS_MmzFlushCache(corner_mem.u64PhyAddr,(HI_VOID*)(HI_UL)corner_mem.u64VirAddr,corner_mem.u32Size);
        uint16_t *cd=(uint16_t*)(HI_UL)corner_mem.u64VirAddr;
        uint16_t ncorners = cd[0];
        fwrite("STC1", 4, 1, fp);
        fwrite(cd, 1, 2+ncorners*4, fp); /* count + corner points */
        fprintf(stderr, "st_corner done, corners=%d\n", ncorners);
        HI_MPI_SYS_MmzFree(st_dst.au64PhyAddr[0],(HI_VOID*)(HI_UL)st_dst.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(st_mem.u64PhyAddr,(HI_VOID*)(HI_UL)st_mem.u64VirAddr);
        HI_MPI_SYS_MmzFree(corner_mem.u64PhyAddr,(HI_VOID*)(HI_UL)corner_mem.u64VirAddr);
    }

    /* === LKOpticalFlowPyr === */
    {
        IVE_SRC_IMAGE_S lk_next; memset(&lk_next,0,sizeof(lk_next));
        lk_next.enType=IVE_IMAGE_TYPE_U8C1; lk_next.u32Width=W; lk_next.u32Height=H;
        lk_next.au32Stride[0]=STRIDE;
        HI_MPI_SYS_MmzAlloc(&lk_next.au64PhyAddr[0],(HI_VOID**)&lk_next.au64VirAddr[0],NULL,NULL,STRIDE*H);
        uint8_t *nv=(uint8_t*)(HI_UL)lk_next.au64VirAddr[0];
        for(int y=0;y<H;y++) for(int x=0;x<W;x++)
            nv[y*STRIDE+x] = (x>=2) ? sv1[y*STRIDE+(x-2)] : sv1[y*STRIDE+x];
        HI_MPI_SYS_MmzFlushCache(lk_next.au64PhyAddr[0],nv,STRIDE*H);
        int npts=5;
        int test_pts[][2]={{20,20},{30,30},{10,40},{40,10},{32,32}};
        HI_U32 pts_sz=(500*8+15)&~15;
        IVE_SRC_MEM_INFO_S pp; IVE_MEM_INFO_S np;
        IVE_DST_MEM_INFO_S st,er;
        memset(&pp,0,sizeof(pp)); memset(&np,0,sizeof(np));
        memset(&st,0,sizeof(st)); memset(&er,0,sizeof(er));
        pp.u32Size=np.u32Size=pts_sz;
        st.u32Size=(500+15)&~15; er.u32Size=(1000+15)&~15;
        HI_MPI_SYS_MmzAlloc(&pp.u64PhyAddr,(HI_VOID**)&pp.u64VirAddr,NULL,NULL,pts_sz);
        HI_MPI_SYS_MmzAlloc(&np.u64PhyAddr,(HI_VOID**)&np.u64VirAddr,NULL,NULL,pts_sz);
        HI_MPI_SYS_MmzAlloc(&st.u64PhyAddr,(HI_VOID**)&st.u64VirAddr,NULL,NULL,st.u32Size);
        HI_MPI_SYS_MmzAlloc(&er.u64PhyAddr,(HI_VOID**)&er.u64VirAddr,NULL,NULL,er.u32Size);
        int32_t *ppv=(int32_t*)(HI_UL)pp.u64VirAddr;
        memset(ppv,0,pts_sz);
        for(int i=0;i<npts;i++) { ppv[i*2]=test_pts[i][0]<<7; ppv[i*2+1]=test_pts[i][1]<<7; }
        HI_MPI_SYS_MmzFlushCache(pp.u64PhyAddr,ppv,pts_sz);
        memcpy((void*)(HI_UL)np.u64VirAddr,ppv,pts_sz);
        HI_MPI_SYS_MmzFlushCache(np.u64PhyAddr,(void*)(HI_UL)np.u64VirAddr,pts_sz);
        IVE_SRC_IMAGE_S prev_pyr[1]={src1}, next_pyr[1]={lk_next};
        IVE_LK_OPTICAL_FLOW_PYR_CTRL_S lkc; memset(&lkc,0,sizeof(lkc));
        lkc.enOutMode=IVE_LK_OPTICAL_FLOW_PYR_OUT_MODE_BOTH;
        lkc.bUseInitFlow=HI_TRUE; lkc.u16PtsNum=npts; lkc.u8MaxLevel=0;
        lkc.u0q8MinEigThr=100; lkc.u8IterCnt=10; lkc.u0q8Eps=2;
        ret = HI_MPI_IVE_LKOpticalFlowPyr(&handle,prev_pyr,next_pyr,&pp,&np,&st,&er,&lkc,HI_TRUE);
        if(ret==0) ive_wait(handle);
        HI_MPI_SYS_MmzFlushCache(np.u64PhyAddr,(void*)(HI_UL)np.u64VirAddr,pts_sz);
        HI_MPI_SYS_MmzFlushCache(st.u64PhyAddr,(void*)(HI_UL)st.u64VirAddr,st.u32Size);
        HI_MPI_SYS_MmzFlushCache(er.u64PhyAddr,(void*)(HI_UL)er.u64VirAddr,er.u32Size);
        int32_t *npv=(int32_t*)(HI_UL)np.u64VirAddr;
        uint8_t *stv=(uint8_t*)(HI_UL)st.u64VirAddr;
        fwrite("LKF1", 4, 1, fp);
        fwrite(npv, 4, npts*2, fp);  /* next points (S25Q7) */
        fwrite(stv, 1, npts, fp);    /* status */
        int tracked=0; for(int i=0;i<npts;i++) if(stv[i]) tracked++;
        fprintf(stderr, "lk_flow done, tracked=%d/%d\n", tracked, npts);
        for(int i=0;i<npts;i++) {
            float dx=(float)(npv[i*2]-ppv[i*2])/128.0f;
            float dy=(float)(npv[i*2+1]-ppv[i*2+1])/128.0f;
            fprintf(stderr, "  pt%d: flow=(%.2f,%.2f) status=%d\n", i, dx, dy, stv[i]);
        }
        HI_MPI_SYS_MmzFree(lk_next.au64PhyAddr[0],(HI_VOID*)(HI_UL)lk_next.au64VirAddr[0]);
        HI_MPI_SYS_MmzFree(pp.u64PhyAddr,(HI_VOID*)(HI_UL)pp.u64VirAddr);
        HI_MPI_SYS_MmzFree(np.u64PhyAddr,(HI_VOID*)(HI_UL)np.u64VirAddr);
        HI_MPI_SYS_MmzFree(st.u64PhyAddr,(HI_VOID*)(HI_UL)st.u64VirAddr);
        HI_MPI_SYS_MmzFree(er.u64PhyAddr,(HI_VOID*)(HI_UL)er.u64VirAddr);
    }

    fclose(fp);
    HI_MPI_SYS_MmzFree(src1.au64PhyAddr[0],(HI_VOID*)(HI_UL)src1.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(src2.au64PhyAddr[0],(HI_VOID*)(HI_UL)src2.au64VirAddr[0]);
    HI_MPI_SYS_MmzFree(dst.au64PhyAddr[0],(HI_VOID*)(HI_UL)dst.au64VirAddr[0]);
    HI_MPI_SYS_Exit();
    fprintf(stderr, "All captures written to /tmp/ive_cap.bin\n");
    return 0;
}
