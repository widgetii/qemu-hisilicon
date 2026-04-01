/*
 * Standalone SquashFS v4 reader for XM (Xiongmai) firmware images.
 *
 * Reproduces the XM U-Boot "squashfsload" logic step by step to debug
 * why it fails in QEMU but works on real Hi3516EV300 hardware.
 *
 * Usage:
 *   ./test-xm-sqfs /tmp/xm_romfs.sqfs [boot/uImage]
 *
 * Build (host):
 *   gcc -O2 -o test-xm-sqfs test-xm-sqfs.c \
 *       xz_dec_stream.c xz_dec_lzma2.c xz_crc32.c -I. -DXZ_DEC_ANY_CHECK
 *
 * Build (ARM, HiSilicon toolchain):
 *   $CC -static -O2 -std=gnu99 -march=armv7-a -o test-xm-sqfs test-xm-sqfs.c \
 *       xz_dec_stream.c xz_dec_lzma2.c xz_crc32.c -I. -DXZ_DEC_ANY_CHECK
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "xz.h"

/* ── SquashFS v4 on-disk structures (from Linux kernel squashfs_fs.h) ── */

#define SQUASHFS_MAGIC          0x73717368
#define SQUASHFS_METADATA_SIZE  8192
#define SQUASHFS_INVALID_FRAG   0xffffffffU

/* Inode types */
#define SQUASHFS_DIR_TYPE       1
#define SQUASHFS_REG_TYPE       2
#define SQUASHFS_LDIR_TYPE      8
#define SQUASHFS_LREG_TYPE      9

/* Metadata block: bit 15 = uncompressed flag */
#define SQUASHFS_COMPRESSED_BIT     (1 << 15)
#define SQUASHFS_COMPRESSED(B)      (!((B) & SQUASHFS_COMPRESSED_BIT))
#define SQUASHFS_COMPRESSED_SIZE(B) ((B) & ~SQUASHFS_COMPRESSED_BIT)

/* Data block: bit 24 = uncompressed flag */
#define SQUASHFS_COMPRESSED_BIT_BLOCK   (1 << 24)
#define SQUASHFS_COMPRESSED_BLOCK(B)    (!((B) & SQUASHFS_COMPRESSED_BIT_BLOCK))
#define SQUASHFS_COMPRESSED_SIZE_BLOCK(B) ((B) & ~SQUASHFS_COMPRESSED_BIT_BLOCK)

/* Inode reference: upper 32 bits = block, lower 16 bits = offset */
#define SQUASHFS_INODE_BLK(A)    ((uint32_t)((A) >> 16))
#define SQUASHFS_INODE_OFFSET(A) ((uint32_t)((A) & 0xffff))

/* Compression types */
#define XZ_COMPRESSION  4

#pragma pack(push, 1)

struct squashfs_super_block {
    uint32_t s_magic;
    uint32_t inodes;
    uint32_t mkfs_time;
    uint32_t block_size;
    uint32_t fragments;
    uint16_t compression;
    uint16_t block_log;
    uint16_t flags;
    uint16_t no_ids;
    uint16_t s_major;
    uint16_t s_minor;
    uint64_t root_inode;
    uint64_t bytes_used;
    uint64_t id_table_start;
    uint64_t xattr_id_table_start;
    uint64_t inode_table_start;
    uint64_t directory_table_start;
    uint64_t fragment_table_start;
    uint64_t lookup_table_start;
};

struct squashfs_base_inode {
    uint16_t inode_type;
    uint16_t mode;
    uint16_t uid;
    uint16_t guid;
    uint32_t mtime;
    uint32_t inode_number;
};

struct squashfs_reg_inode {
    uint16_t inode_type;
    uint16_t mode;
    uint16_t uid;
    uint16_t guid;
    uint32_t mtime;
    uint32_t inode_number;
    uint32_t start_block;
    uint32_t fragment;
    uint32_t offset;
    uint32_t file_size;
    /* followed by block_sizes[] */
};

struct squashfs_lreg_inode {
    uint16_t inode_type;
    uint16_t mode;
    uint16_t uid;
    uint16_t guid;
    uint32_t mtime;
    uint32_t inode_number;
    uint64_t start_block;
    uint64_t file_size;
    uint64_t sparse;
    uint32_t nlink;
    uint32_t fragment;
    uint32_t offset;
    uint32_t xattr;
    /* followed by block_sizes[] */
};

struct squashfs_dir_inode {
    uint16_t inode_type;
    uint16_t mode;
    uint16_t uid;
    uint16_t guid;
    uint32_t mtime;
    uint32_t inode_number;
    uint32_t start_block;
    uint32_t nlink;
    uint16_t file_size;
    uint16_t offset;
    uint32_t parent_inode;
};

struct squashfs_ldir_inode {
    uint16_t inode_type;
    uint16_t mode;
    uint16_t uid;
    uint16_t guid;
    uint32_t mtime;
    uint32_t inode_number;
    uint32_t nlink;
    uint32_t file_size;
    uint32_t start_block;
    uint32_t parent_inode;
    uint16_t i_count;
    uint16_t offset;
    uint32_t xattr;
    /* followed by dir_index[] */
};

struct squashfs_dir_header {
    uint32_t count;
    uint32_t start_block;
    uint32_t inode_number;
};

struct squashfs_dir_entry {
    uint16_t offset;
    int16_t  inode_number;
    uint16_t type;
    uint16_t size;
    /* followed by name[size+1] */
};

struct squashfs_fragment_entry {
    uint64_t start_block;
    uint32_t size;
    uint32_t unused;
};

#pragma pack(pop)

/* ── Global state ─────────────────────────────────────────────────────── */

static uint8_t *sqfs_data;
static uint32_t sqfs_size;
static struct squashfs_super_block *sb;
static struct xz_dec *xz_state;

/* Decompressed metadata tables */
static uint8_t *inode_table;
static uint32_t inode_table_size;
static uint8_t *dir_table;
static uint32_t dir_table_size;

/* ── XZ decompression ─────────────────────────────────────────────────── */

static int xz_decompress(const uint8_t *in, uint32_t in_size,
                          uint8_t *out, uint32_t out_size, uint32_t *out_len)
{
    struct xz_buf buf;
    enum xz_ret ret;

    xz_dec_reset(xz_state);

    buf.in = in;
    buf.in_pos = 0;
    buf.in_size = in_size;
    buf.out = out;
    buf.out_pos = 0;
    buf.out_size = out_size;

    ret = xz_dec_run(xz_state, &buf);
    *out_len = buf.out_pos;

    if (ret == XZ_STREAM_END || ret == XZ_OK)
        return 0;

    printf("  xz_dec_run returned %d (in=%u/%u out=%u/%u)\n",
           (int)ret, (unsigned)buf.in_pos, in_size,
           (unsigned)buf.out_pos, out_size);
    return -1;
}

/* ── Metadata block reader ────────────────────────────────────────────── */

/*
 * Read a metadata block at the given offset in the SquashFS image.
 * Returns the decompressed size, or -1 on error.
 * *next_offset is set to the offset of the next metadata block.
 */
static int read_metadata_block(uint32_t offset, uint8_t *out, uint32_t out_size,
                                uint32_t *next_offset)
{
    if (offset + 2 > sqfs_size)
        return -1;

    uint16_t hdr = *(uint16_t *)(sqfs_data + offset);
    bool compressed = SQUASHFS_COMPRESSED(hdr);
    uint32_t c_size = SQUASHFS_COMPRESSED_SIZE(hdr);

    if (c_size == 0 || c_size > sqfs_size - offset - 2)
        return -1;

    const uint8_t *block_data = sqfs_data + offset + 2;
    *next_offset = offset + 2 + c_size;

    if (!compressed) {
        /* Uncompressed: copy directly */
        uint32_t copy = c_size < out_size ? c_size : out_size;
        memcpy(out, block_data, copy);
        return copy;
    }

    /* Compressed: decompress with XZ */
    uint32_t dec_size = 0;
    if (xz_decompress(block_data, c_size, out, out_size, &dec_size) < 0) {
        printf("  metadata decompress FAILED at offset 0x%x (c_size=%u)\n",
               offset, c_size);
        printf("  first 8 bytes: %02x %02x %02x %02x %02x %02x %02x %02x\n",
               block_data[0], block_data[1], block_data[2], block_data[3],
               block_data[4], block_data[5], block_data[6], block_data[7]);
        return -1;
    }

    return dec_size;
}

/* ── Decompress full metadata table ───────────────────────────────────── */

static int decompress_table(uint32_t start_offset, uint32_t end_offset,
                             uint8_t **out_buf, uint32_t *out_size)
{
    /* Allocate generous buffer — metadata tables are typically < 64KB */
    uint32_t buf_size = 256 * 1024;
    uint8_t *buf = malloc(buf_size);
    uint32_t total = 0;
    uint32_t offset = start_offset;
    int block_num = 0;

    while (offset < end_offset) {
        uint8_t tmp[SQUASHFS_METADATA_SIZE];
        uint32_t next;
        int dec = read_metadata_block(offset, tmp, sizeof(tmp), &next);

        if (dec < 0) {
            printf("  block %d at 0x%x: FAILED\n", block_num, offset);
            free(buf);
            return -1;
        }

        printf("  block %d at 0x%x: %d bytes decompressed\n",
               block_num, offset, dec);

        if (total + dec > buf_size) {
            buf_size *= 2;
            buf = realloc(buf, buf_size);
        }
        memcpy(buf + total, tmp, dec);
        total += dec;
        offset = next;
        block_num++;
    }

    *out_buf = buf;
    *out_size = total;
    return 0;
}

/* ── Inode reader ─────────────────────────────────────────────────────── */

static void dump_inode(uint64_t inode_ref)
{
    uint32_t blk = SQUASHFS_INODE_BLK(inode_ref);
    uint32_t off = SQUASHFS_INODE_OFFSET(inode_ref);

    /* blk is relative to inode_table_start, in terms of decompressed 8KB blocks.
     * For a simple implementation, we treat it as a byte offset in the
     * concatenated decompressed inode table. Actually in SquashFS, blk is the
     * compressed byte offset from inode_table_start, and offset is the byte
     * offset within the decompressed block. We need the block mapping. */

    printf("  inode_ref=0x%llx (blk=0x%x, off=0x%x)\n",
           (unsigned long long)inode_ref, blk, off);

    /* For simplicity, if our inode table is fully decompressed sequentially,
     * we need to map the compressed block offset to the decompressed offset.
     * This requires tracking the compressed→decompressed block boundaries.
     * For now, just try offset-based access. */
    if (off >= inode_table_size) {
        printf("  inode offset 0x%x beyond table size 0x%x\n",
               off, inode_table_size);
        return;
    }

    struct squashfs_base_inode *base =
        (struct squashfs_base_inode *)(inode_table + off);

    printf("  type=%u mode=0%o uid=%u gid=%u inode_number=%u\n",
           base->inode_type, base->mode, base->uid, base->guid,
           base->inode_number);

    if (base->inode_type == SQUASHFS_REG_TYPE) {
        struct squashfs_reg_inode *reg =
            (struct squashfs_reg_inode *)(inode_table + off);
        printf("  REG: start_block=0x%x file_size=%u fragment=%u\n",
               reg->start_block, reg->file_size, reg->fragment);
    } else if (base->inode_type == SQUASHFS_LREG_TYPE) {
        struct squashfs_lreg_inode *lreg =
            (struct squashfs_lreg_inode *)(inode_table + off);
        printf("  LREG: start_block=0x%llx file_size=%llu fragment=%u\n",
               (unsigned long long)lreg->start_block,
               (unsigned long long)lreg->file_size, lreg->fragment);
    } else if (base->inode_type == SQUASHFS_DIR_TYPE) {
        struct squashfs_dir_inode *dir =
            (struct squashfs_dir_inode *)(inode_table + off);
        printf("  DIR: start_block=%u file_size=%u offset=%u nlink=%u\n",
               dir->start_block, dir->file_size, dir->offset, dir->nlink);
    } else if (base->inode_type == SQUASHFS_LDIR_TYPE) {
        struct squashfs_ldir_inode *ldir =
            (struct squashfs_ldir_inode *)(inode_table + off);
        printf("  LDIR: start_block=%u file_size=%u offset=%u nlink=%u i_count=%u\n",
               ldir->start_block, ldir->file_size, ldir->offset,
               ldir->nlink, ldir->i_count);
    }
}

/* ── Directory reader ─────────────────────────────────────────────────── */

static void list_directory(uint32_t start_block, uint32_t offset,
                            uint32_t dir_size)
{
    /* start_block is relative to directory_table_start (compressed offset).
     * For our sequentially-decompressed table, we use offset directly.
     * This is a simplification — proper impl needs block mapping. */

    if (offset >= dir_table_size) {
        printf("  dir offset 0x%x beyond table size 0x%x\n",
               offset, dir_table_size);
        return;
    }

    uint32_t pos = offset;
    uint32_t end = offset + dir_size - 3; /* dir_size includes . and .. */
    int entry_count = 0;

    while (pos < end && pos < dir_table_size) {
        struct squashfs_dir_header *hdr =
            (struct squashfs_dir_header *)(dir_table + pos);
        pos += sizeof(*hdr);

        uint32_t count = hdr->count + 1;
        printf("  dir_header: count=%u start_block=0x%x inode_number=%u\n",
               count, hdr->start_block, hdr->inode_number);

        for (uint32_t i = 0; i < count && pos < dir_table_size; i++) {
            struct squashfs_dir_entry *ent =
                (struct squashfs_dir_entry *)(dir_table + pos);
            pos += sizeof(*ent);

            uint32_t name_len = ent->size + 1;
            char name[257];
            if (name_len > 256) name_len = 256;
            if (pos + name_len > dir_table_size) break;
            memcpy(name, dir_table + pos, name_len);
            name[name_len] = '\0';
            pos += name_len;

            printf("    [%d] type=%u inode_off=%d name=\"%s\"\n",
                   entry_count, ent->type, ent->inode_number, name);
            entry_count++;
        }
    }
    printf("  total entries: %d\n", entry_count);
}

/* ── Main ─────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    const char *sqfs_path = argc > 1 ? argv[1] : "/tmp/xm_romfs.sqfs";

    printf("=== XM SquashFS Unit Test ===\n");
    printf("Image: %s\n\n", sqfs_path);

    /* Load image */
    FILE *f = fopen(sqfs_path, "rb");
    if (!f) {
        printf("FAIL: cannot open %s\n", sqfs_path);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    sqfs_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    sqfs_data = malloc(sqfs_size);
    fread(sqfs_data, 1, sqfs_size, f);
    fclose(f);
    printf("Loaded %u bytes\n", sqfs_size);

    /* Init XZ decoder */
    xz_crc32_init();
    xz_state = xz_dec_init(XZ_DYNALLOC, 1 << 16);
    if (!xz_state) {
        printf("FAIL: xz_dec_init\n");
        return 1;
    }

    /* ── Step 1: Parse superblock ── */
    printf("\n--- Step 1: Superblock ---\n");
    sb = (struct squashfs_super_block *)sqfs_data;

    if (sb->s_magic != SQUASHFS_MAGIC) {
        printf("FAIL: bad magic 0x%08x (expected 0x%08x)\n",
               sb->s_magic, SQUASHFS_MAGIC);
        return 1;
    }

    printf("magic=0x%08x inodes=%u block_size=%u compression=%u\n",
           sb->s_magic, sb->inodes, sb->block_size, sb->compression);
    printf("version=%u.%u flags=0x%04x no_ids=%u\n",
           sb->s_major, sb->s_minor, sb->flags, sb->no_ids);
    printf("root_inode=0x%llx bytes_used=%llu\n",
           (unsigned long long)sb->root_inode,
           (unsigned long long)sb->bytes_used);
    printf("inode_table=0x%llx dir_table=0x%llx\n",
           (unsigned long long)sb->inode_table_start,
           (unsigned long long)sb->directory_table_start);
    printf("frag_table=0x%llx id_table=0x%llx\n",
           (unsigned long long)sb->fragment_table_start,
           (unsigned long long)sb->id_table_start);

    if (sb->compression != XZ_COMPRESSION) {
        printf("WARN: compression=%u, expected XZ(%d)\n",
               sb->compression, XZ_COMPRESSION);
    }

    /* ── Step 2: Decompress inode table ── */
    printf("\n--- Step 2: Inode table (0x%llx → 0x%llx) ---\n",
           (unsigned long long)sb->inode_table_start,
           (unsigned long long)sb->directory_table_start);

    if (decompress_table(sb->inode_table_start, sb->directory_table_start,
                          &inode_table, &inode_table_size) < 0) {
        printf("FAIL: inode table decompression failed\n");
        printf("This indicates the metadata is encrypted/obfuscated.\n");
        printf("The XM firmware has a proprietary decryption step.\n");
        goto cleanup;
    }
    printf("Inode table: %u bytes decompressed\n", inode_table_size);

    /* ── Step 3: Decompress directory table ── */
    printf("\n--- Step 3: Directory table (0x%llx → 0x%llx) ---\n",
           (unsigned long long)sb->directory_table_start,
           (unsigned long long)sb->fragment_table_start);

    if (decompress_table(sb->directory_table_start, sb->fragment_table_start,
                          &dir_table, &dir_table_size) < 0) {
        printf("FAIL: directory table decompression failed\n");
        goto cleanup;
    }
    printf("Directory table: %u bytes decompressed\n", dir_table_size);

    /* ── Step 4: Read root inode ── */
    printf("\n--- Step 4: Root inode ---\n");
    dump_inode(sb->root_inode);

    /* ── Step 5: List root directory ── */
    printf("\n--- Step 5: Root directory listing ---\n");
    {
        uint32_t off = SQUASHFS_INODE_OFFSET(sb->root_inode);
        if (off < inode_table_size) {
            struct squashfs_base_inode *base =
                (struct squashfs_base_inode *)(inode_table + off);
            if (base->inode_type == SQUASHFS_DIR_TYPE) {
                struct squashfs_dir_inode *dir =
                    (struct squashfs_dir_inode *)(inode_table + off);
                list_directory(dir->start_block, dir->offset, dir->file_size);
            } else if (base->inode_type == SQUASHFS_LDIR_TYPE) {
                struct squashfs_ldir_inode *ldir =
                    (struct squashfs_ldir_inode *)(inode_table + off);
                list_directory(ldir->start_block, ldir->offset, ldir->file_size);
            } else {
                printf("Root inode is not a directory (type=%u)\n",
                       base->inode_type);
            }
        }
    }

    printf("\n--- Step 6: Summary ---\n");
    printf("Superblock: OK\n");
    printf("Inode table: %u bytes\n", inode_table_size);
    printf("Directory table: %u bytes\n", dir_table_size);
    printf("=== Test complete ===\n");

cleanup:
    xz_dec_end(xz_state);
    free(inode_table);
    free(dir_table);
    free(sqfs_data);
    return 0;
}
