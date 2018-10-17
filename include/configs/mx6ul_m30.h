/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 EVK board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6UL_M30_CONFIG_H
#define __MX6UL_M30_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

#define CONFIG_USE_PLUGIN

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

/* uncomment for BEE support, needs to enable CONFIG_CMD_FUSE */
/* #define CONFIG_CMD_BEE */

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

#define CONFIG_USE_ARCH_MEMCPY
#define CONFIG_USE_ARCH_MEMSET

#define PHYS_SDRAM_SIZE		SZ_256M
#define CONFIG_BOOTARGS_CMA_SIZE   ""

#define CONFIG_LDO_BYPASS_CHECK

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART2_BASE

/* MMC Configs */
#ifndef CONFIG_FSL_USDHC
#error "i.MX6UL Densowave M30 boots from USDHC. Enable CONFIG_FSL_USDHC"
#endif

#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

#undef CONFIG_DISPLAY_BOARDINFO
#define CONFIG_DISPLAY_CPUINFO

/* I2C configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3000
#define CONFIG_POWER_PFUZE3000_I2C_ADDR  0x08

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_TFTP_SERVERIP 192.168.78.2
#define CONFIG_NFS_SERVERIP 192.168.78.2

#define CONFIG_IPADDR 192.168.78.100
#define CONFIG_SERVERIP CONFIG_TFTP_SERVERIP
#define CONFIG_GATEWAYIP 192.168.78.2
#define CONFIG_NETMASK 255.255.255.0

#define CONFIG_NETBOOT_ENV_SETTINGS \
	"netargs=" \
		"console=ttymxc1,115200" \
		" ignore_loglevel" \
		" loglevel=7" \
		" earlyprintk" \
		" root=/dev/nfs rw" \
		" nfsrootdebug" \
		" nfsroot=" __stringify(CONFIG_NFS_SERVERIP) ":/opt/m30-rootfs-1,tcp,vers=3" \
		" ip=" __stringify(CONFIG_IPADDR) ":" __stringify(CONFIG_NFS_SERVERIP) ":" __stringify(CONFIG_GATEWAYIP) ":" __stringify(CONFIG_NETMASK) ":m30" \
		"\0" \
	"netboot=" \
		"usb start; " \
		"tftp ${loadaddr} ${image}; " \
		"tftp ${fdt_addr} ${fdt_file}; " \
		"setenv bootargs ${netargs}; " \
		"bootz ${loadaddr} - ${fdt_addr}" \
		"\0"

#define CONFIG_SDBOOT_ENV_SETTINGS \
	"sdargs=" \
		" earlyprintk" \
		" root=/dev/mmcblk1p2" \
		"\0" \
	"sdloadimage=fatload mmc ${mmcdev}:1 ${loadaddr} ${image}\0" \
	"sdloadfdt=fatload mmc ${mmcdev}:1 ${fdt_addr} ${fdt_file}\0" \
	"sdload=" \
		"mmc dev ${mmcdev}; " \
		"mmc rescan; " \
		"run sdloadimage; run sdloadfdt; " \
		"setenv bootargs ${sdargs}; " \
		"\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=ttymxc1,115200\0" \
	"mmcdev=0\0" \
	"script=boot.scr\0" \
	"image=/boot/zImage\0" \
	"fdt_file=/boot/imx6ul-densowave-m30.dtb\0" \
	"initrd_addr=0x83800000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr=0x83000000\0" \
	"load_addr=0x80800000\0" \
	"retries_0=0\0"\
	"retries_1=0\0"\
	"bootpart=0\0"\
	"loadbootscript=fatload mmc ${mmcdev}:1 ${loadaddr} ${script};\0" \
	"bootargs=console=ttymxc1,115200 earlyprintk\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
	        "source\0" \
	CONFIG_SDBOOT_ENV_SETTINGS \
	CONFIG_NETBOOT_ENV_SETTINGS \
	"\0"

#define CONFIG_BOOTCOMMAND \
	"echo booting...; " \
	"if run loadbootscript; then " \
	        "setenv bootargs ${sdargs}; " \
	        "run bootscript;" \
	"else " \
        	"run sdload; " \
	        "bootz ${loadaddr} - ${fdt_addr};" \
	"fi "

/* Miscellaneous configurable options */
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(4 * 1024)
#define CONFIG_ENV_OFFSET		(2 * 1024 * 1024)
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV 0


/* USB Configs */
/* #define CONFIG_CMD_USB */

#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#endif

#define CONFIG_IMX_THERMAL

/* Temporarily disable video */
/* #define CONFIG_VIDEO */
#ifdef CONFIG_VIDEO
#undef CONFIG_VIDEO
#endif

#define CONFIG_WATCHDOG

#ifdef CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_SW_CURSOR
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

#define CONFIG_MODULE_FUSE
#define CONFIG_OF_SYSTEM_SETUP

#endif
