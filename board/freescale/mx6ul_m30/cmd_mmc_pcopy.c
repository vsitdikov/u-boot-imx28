/**
 * Copyright (C) 2016,  Mentor Graphics, Corp.
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <malloc.h>
#include <mmc.h>
#include <watchdog.h>

static unsigned char *mmcp_buf;
static unsigned long mmcp_buf_size = CONFIG_MMCP_DATA_BUF_SIZE;

unsigned char *mmcp_free_buf(void)
{
        free(mmcp_buf);
        mmcp_buf = NULL;
        return mmcp_buf;
}

unsigned long mmcp_get_buf_size(void)
{
        return mmcp_buf_size;
}

unsigned char *mmcp_get_buf(void)
{
        char *s;

        if (mmcp_buf != NULL)
                return mmcp_buf;

        s = getenv("mmcp_bufsiz");
        mmcp_buf_size = s ? (unsigned long)simple_strtol(s, NULL, 16) :
                CONFIG_MMCP_DATA_BUF_SIZE;

        mmcp_buf = memalign(CONFIG_SYS_CACHELINE_SIZE, mmcp_buf_size);
        if (mmcp_buf == NULL)
                printf("%s: Could not memalign 0x%lx bytes\n",
                       __func__, mmcp_buf_size);

        return mmcp_buf;
}

static int do_copy(struct mmc *mmc, disk_partition_t s_partinfo, disk_partition_t d_partinfo)
{
        u32 blks_copied = 0, nblks, crt_chunk_blk_count;
        u32 blks_left = s_partinfo.size;

        mmcp_buf = mmcp_get_buf();
        if (!mmcp_buf) {
                printf("Failed to allocate scratch buffer!\n");
                return -ENOMEM;
        }

        while (blks_copied < s_partinfo.size) {
                crt_chunk_blk_count = min((mmcp_buf_size/d_partinfo.blksz), blks_left);

                nblks = mmc->block_dev.block_read(mmc->block_dev.dev,
                                                  s_partinfo.start + blks_copied,
                                                  crt_chunk_blk_count,
                                                  mmcp_buf);

                if (nblks != crt_chunk_blk_count) {
                        printf("MMC block read failed!\n");
                        mmcp_free_buf();
                        return -EIO;
                }

                WATCHDOG_RESET();

                nblks = mmc->block_dev.block_write(mmc->block_dev.dev,
                                                   d_partinfo.start + blks_copied,
                                                   crt_chunk_blk_count,
                                                   mmcp_buf);

                if (nblks != crt_chunk_blk_count) {
                        printf("MMC block write failed!\n");
                        mmcp_free_buf();
                        return -EIO;
                }

                blks_copied += nblks;
                blks_left -= nblks;
        }

        mmcp_free_buf();
        return 0;
}

int mmc_part_copy(int dev, int src_part, int dst_part)
{
        struct mmc *mmc = NULL;
        disk_partition_t s_partinfo, d_partinfo;
        block_dev_desc_t *blk_dev;
        int err;

        printf("Restoring partition %d from factory image on partition %d...\n",
               dst_part, src_part);

        mmc = find_mmc_device(dev);
        if (mmc == NULL) {
                printf("Failed to find device %d\n", dev);
                return -ENODEV;
        }

        err = mmc_init(mmc);
        if (err) {
                printf("Failed to init mmc device: %d\n", err);
                return err;
        }

        blk_dev = &mmc->block_dev;

        err = get_partition_info(blk_dev, src_part, &s_partinfo);
        if (err) {
                printf("Failed to get partition info for partition %d\n",
                       src_part);
                return err;
        }

        err = get_partition_info(blk_dev, dst_part, &d_partinfo);
        if (err) {
                printf("Failed to get partition info for partition %d\n",
                       dst_part);
                return err;
        }

        if (d_partinfo.size < s_partinfo.size) {
                printf("Source partition is %lu blocks long, but "
                       "only %lu blocks are available in destination\n",
                       s_partinfo.size, d_partinfo.size);
                return -ENOMEM;
        }

        err = do_copy(mmc, s_partinfo, d_partinfo);
        if (err) {
                printf("Copy failed: %d\n", err);
                return err;
        }

	printf("Done!\n");
        
        return 0;
}

int do_mmcp_copy(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
        int dev, src_part = -1, dst_part = -1;

        if (argc < 5)
                return CMD_RET_USAGE;

        if (strcmp(argv[1], "copy") != 0)
                return CMD_RET_USAGE;

        dev      = (int)simple_strtoul(argv[2], NULL, 10);
        src_part = (int)simple_strtoul(argv[3], NULL, 10);
        dst_part = (int)simple_strtoul(argv[4], NULL, 10);

        if (mmc_part_copy(dev, src_part, dst_part) != 0)
                return 1;

        return 0;
}

U_BOOT_CMD(
           mmcp, 5, 1, do_mmcp_copy,
           "Copy (like dd) a MMC partition to another",
           "copy <dev> <src_part> <dst_part>\n"
           "    - Copy src_part partition to dst_part partition on dev"
           );
