/* ptmax.c -- Partition resize utility.
 *
 * Copyright (C) 2011-2012 George Shuklin (george.shuklin@gmail.com)
 * Copyright (C) 2020 Yaroslav Syrytsia (me@ys.lc)
 *
 * This program is free software.  You can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation: either version 333 or
 * (at your option) any later version.
 *
 * read4_le() and store4_le() is taken from
 * fdisk  utility ((C) 1992  A. V. Le Blanc (LeBlanc@mcc.ac.uk)
 *
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <err.h>

#include <linux/fs.h>

#define PT_OFFSET 0x1BE
#define PTE_SIZE 16

struct pte {
	uint8_t status;
	uint8_t partition;
	uint32_t LBA;
	uint32_t size; /*in sectors*/
};

struct pt {
	struct pte e[4];
};

struct ptmax_context {
	uint8_t *mbr; /* ptr to MBR data */
	struct pte p[4]; /* partition table */
	int fd; /* device's fd */
	uint32_t dev_size; /* total size, in bytes */
	uint32_t sect_size; /* sector size, in bytes */
};

static inline bool ptmax_is_pt(const struct ptmax_context *ctx)
{
	return (ctx->mbr[510] == 0x55) && (ctx->mbr[511] == 0xAA);
}

static int ptmax_open(struct ptmax_context *ctx, const char *device)
{
	ctx->fd = open(device, O_RDWR | O_LARGEFILE | O_NONBLOCK);
	if (ctx->fd < 0) {
		warn("cannot open device [%s]", device);
		return -1;
	}

	if (ioctl(ctx->fd, BLKGETSIZE, &ctx->dev_size) < 0) {
		warn("BLKGETSIZE failed");
		return -2;
	}

	if (ioctl(ctx->fd, BLKSSZGET, &ctx->sect_size) < 0) {
		warn("BLKSSZGET failed");
		return -3;
	}

	if (ctx->dev_size == 0 || ctx->sect_size == 0) {
		warnx("invalid device size: %" PRIu32 "/%" PRIu32, ctx->dev_size, ctx->sect_size);
		return -4;
	}

	warnx("device: %s: %" PRIu32 "/%" PRIu32, device, ctx->dev_size, ctx->sect_size);

	ctx->mbr = calloc(1, ctx->sect_size);
	if (read(ctx->fd, ctx->mbr, ctx->sect_size) != ctx->sect_size) {
		warn("cannot read 1st sector");
		return -5;
	}

	return 0;
}

static void ptmax_close(struct ptmax_context *ctx)
{
	sync();
	ioctl(ctx->fd, BLKRRPART, 0);
	fsync(ctx->fd);
	close(ctx->fd);
	free(ctx->mbr);
}

static int ptmax_write(struct ptmax_context *ctx)
{
	if (ctx->fd < 0) {
		return -1;
	}

	if (lseek(ctx->fd, 0, SEEK_SET) < 0) {
		warn("seek failed");
		return -2;
	}

	if (write(ctx->fd, ctx->mbr, ctx->sect_size) != ctx->sect_size) {
		warn("cannot write 1st sector");
		return -3;
	}

	return 0;
}

static inline void store4_le(uint8_t *ptr, uint32_t val)
{
	ptr[0] = (val & 0xff);
	ptr[1] = ((val >> 8) & 0xff);
	ptr[2] = ((val >> 16) & 0xff);
	ptr[3] = ((val >> 24) & 0xff);
}

static inline uint32_t read4_le(const uint8_t *ptr)
{
	return (uint32_t)(ptr[0]) + ((uint32_t)(ptr[1]) << 8) + ((uint32_t)(ptr[2]) << 16) + ((uint32_t)(ptr[3]) << 24);
}

static void ptmax_pte_read(struct ptmax_context *ctx, int num)
{
	ctx->p[num].status = ctx->mbr[PT_OFFSET + PTE_SIZE * num + 0];
	ctx->p[num].partition = ctx->mbr[PT_OFFSET + PTE_SIZE * num + 4];
	ctx->p[num].LBA = read4_le(ctx->mbr + PT_OFFSET + PTE_SIZE * num + 8);
	ctx->p[num].size = read4_le(ctx->mbr + PT_OFFSET + PTE_SIZE * num + 0xC); /*in sectors*/
}

static inline void ptmax_pte_write(struct ptmax_context *ctx, int num)
{
	store4_le(ctx->mbr + PT_OFFSET + PTE_SIZE * num + 0xC, ctx->p[num].size);
}

static void ptmax_pt_read(struct ptmax_context *ctx)
{
	for (int c = 0; c < 4; c++) {
		ptmax_pte_read(ctx, c);
		warnx("part: %d, status: %x, LBA: %" PRIu32 ", size: %" PRIu32, c, ctx->p[c].status, ctx->p[c].LBA,
		      ctx->p[c].size);
	}
}

static inline bool ptmax_pte_is_valid(struct ptmax_context *ctx, int num)
{
	return (ctx->p[num].status == 0x80 || ctx->p[num].status == 0) && ctx->p[num].partition != 0;
}

static bool ptmax_pte_is_fine(struct ptmax_context *ctx, int num)
{
	for (int c = 0; c < num; c++) {
		struct pte *p = &ctx->p[num];

		if (c == num)
			continue;

		if (!ptmax_pte_is_valid(ctx, c))
			continue;

		if (p[c].LBA < p[num].LBA &&
		    p[c].LBA + p[c].size >
			    p[num].LBA) /*validating p[num] is starting in the middle of the other patition*/
			return false;
		if (p[c].LBA < p[num].LBA + p[num].size &&
		    p[c].LBA > p[num].LBA) /*other paritition is starting in the middle of p[num]*/
			return false;
	}

	return true;
}

static uint32_t ptmax_pte_get_max(struct ptmax_context *ctx, int num)
{
	/*1) validate PT 2) see if there is hole to next partition 3) see if device have free space at the end*/
	uint32_t new_size = 0;

	if (ctx->p[num].size + ctx->p[num].LBA < 256 * 256 * 256 * ctx->sect_size || !ctx->p[num].size) {
		warnx("partition %d is too small: %" PRIu32 ", want: %" PRIu32, num, ctx->p[num].size + ctx->p[num].LBA,
		      (uint32_t)(256 * 256 * 256 * ctx->sect_size));
		return 0;
	}

	if (!ptmax_pte_is_fine(ctx, num)) {
		warnx("overlapping partitions");
		return 0;
	}

	if (num < 4)
		for (int c = num + 1; c < 4; c++) {
			if (!ptmax_pte_is_valid(ctx, c))
				continue;

			new_size = ctx->p[c].LBA - ctx->p[num].LBA; /*we maximizing up to the next parition start*/
			break;
		}

	if (new_size == 0)
		new_size =
			ctx->dev_size - ctx->p[num].LBA; /*nothing interesing found - trying to grow to the disk end*/

	if (new_size < ctx->p[num].size) {
		warnx("something went wrong");
		return 0;
	}
	return new_size;
}

static int ptmax_pte_apply(struct ptmax_context *ctx, int num, uint32_t size)
{
	if (ctx->p[num].size == size) {
		return 0;
	}

	warnx("part %d: %" PRIu32 " -> %" PRIu32, num, ctx->p[num].size, size);
	ctx->p[num].size = size;

	ptmax_pte_write(ctx, num);
	return ptmax_write(ctx);
}

static int get_child_device(const char *device)
{
	char buf[16] = { '\0' };
	char path[1024];

	if (strncmp(device, "/dev/", 5) == 0)
		device += 5;

	snprintf(path, sizeof(path), "/sys/class/block/%s/partition", device);
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		warn("cannot open %s", path);
		return -1;
	}

	if (read(fd, buf, sizeof(buf)) < 0) {
		warn("cannot read from %s", path);
		return -1;
	}

	close(fd);
	return atoi(buf) - 1;
}

static char *get_root_device(const char *device)
{
	char buf[1024] = { '\0' };
	static char path[1024] = { '\0' };

	if (strncmp(device, "/dev/", 5) == 0)
		device += 5;

	snprintf(path, sizeof(path), "/sys/class/block/%s", device);
	if (readlink(path, buf, sizeof(buf)) < 0) {
		warn("cannot read symlink from %s", path);
		return NULL;
	}

	// buf:
	// ../../devices/platform/soc/fe340000.emmc2/mmc_host/mmc0/mmc0:59b4/block/mmcblk0/mmcblk0p3
	// ../../devices/pci0000:00/0000:00:01.1/0000:01:00.0/nvme/nvme0/nvme0n1/nvme0n1p2
	char *ptr = rindex(buf, '/');
	if (!ptr) {
		warnx("invalid link: %s", buf);
		return NULL;
	}

	// ../../devices/platform/soc/fe340000.emmc2/mmc_host/mmc0/mmc0:59b4/block/mmcblk0/mmcblk0p3
	//                                                                                ^
	//                                                                               ptr
	char *ptr_next = ptr - 1;
	while (*ptr_next && *ptr_next != '/') {
		ptr_next--;
	}

	// ../../devices/platform/soc/fe340000.emmc2/mmc_host/mmc0/mmc0:59b4/block/mmcblk0/mmcblk0p3
	//                                                                        ^       \0
	//                                                                      ptr_next
	*ptr = '\0';
	++ptr_next;
	snprintf(path, sizeof(path), "/dev/%s", ptr_next);
	return path;
}

static void usage(void)
{
	extern const char *__progname;

	printf("Usage: %s: <device path>\n", __progname);
}

// clang-format off
static struct option ptmax_options[] = {
	{ "help",                no_argument, 0, 'h' },
	{ 0, 0, 0, 0 }
};
// clang-format on

int main(int argc, char *argv[])
{
	struct ptmax_context ctx = {
		.mbr = NULL,
	};
	int c;
	const char *device = NULL;

	for (;;) {
		c = getopt_long(argc, argv, "h", ptmax_options, NULL);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			usage();
			break;
		}
	}

	if (optind < argc) {
		device = argv[optind++];
	} else {
		usage();
		return 1;
	}
	warnx("device: %s", device);
	/* Devices
	 * /dev/sda1:
	 *   - root:  /dev/sda
	 *   - child: 0
	 * /dev/mmcblk0p3:
	 *   - root:  /dev/mmcblk0
	 *   - child: 2
	 */
	int pt_num = get_child_device(device);
	device = get_root_device(device);
	warnx("root: %s, target: %d", device, pt_num);

	if (!device || pt_num >= 4 || pt_num < 0)
		errx(4, "invalid device (not msdos?)");

	if (ptmax_open(&ctx, device) < 0)
		return 1;

	if (!ptmax_is_pt(&ctx))
		errx(2, "1st sector is not MBR, aborting");

	ptmax_pt_read(&ctx);

	uint32_t new_size = ptmax_pte_get_max(&ctx, pt_num);
	if (new_size == 0)
		return 5;

	if (ptmax_pte_apply(&ctx, pt_num, new_size) < 0)
		errx(3, "cannot apply changes");

	ptmax_close(&ctx);
	return 0;
}
