// SPDX-License-Identifier: GPL-2.0+
/*
 * MX28_ZOLL_AED3 board
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>

DECLARE_GLOBAL_DATA_PTR;

/* debug uart */
static iomux_cfg_t const duart_pads[] = {
    MX28_PAD_I2C0_SCL__DUART_RX | (MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_PULLUP),
    MX28_PAD_I2C0_SDA__DUART_TX | MXS_PAD_CTRL,
};

/*
 * Functions
 */
int board_early_init_f(void)
{
    printf("board eary init \n");
    mxs_iomux_setup_multiple_pads(duart_pads, ARRAY_SIZE(duart_pads));
    mxs_duart_gpios_init();

	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP2 clock at 160MHz */
	mxs_set_sspclk(MXC_SSPCLK2, 160000, 0);

#ifdef	CONFIG_CMD_USB
	mxs_iomux_setup_pad(MX28_PAD_SSP2_SS1__USB1_OVERCURRENT);
	mxs_iomux_setup_pad(MX28_PAD_AUART2_RX__GPIO_3_8 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL);
	gpio_direction_output(MX28_PAD_AUART2_RX__GPIO_3_8, 1);
#endif

	/* Power on LCD
	gpio_direction_output(MX28_PAD_LCD_RESET__GPIO_3_30, 1);
    */

	/* Set contrast to maximum
	gpio_direction_output(MX28_PAD_PWM2__GPIO_3_18, 1);
    */


	return 0;
}

void mxs_duart_gpios_init(void)
{
    printf(">>mxs_duart_gpios_init\n");
    mxs_iomux_setup_pad(MX28_PAD_I2C0_SCL__GPIO_3_24);
    gpio_direction_output(MX28_PAD_I2C0_SCL__GPIO_3_24, 1);
    gpio_set_value(MX28_PAD_I2C0_SCL__GPIO_3_24, 1);

    mxs_iomux_setup_pad(MX28_PAD_I2C0_SCL__DUART_RX);
}

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
    printf("board init \n");
    mxs_iomux_setup_multiple_pads(duart_pads, ARRAY_SIZE(duart_pads));
    mxs_duart_gpios_init();
    printf("board_init 0");
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef	CONFIG_CMD_MMC
static int mx28evk_mmc_wp(int id)
{
	if (id != 0) {
		printf("MXS MMC: Invalid card selected (card id = %d)\n", id);
		return 1;
	}

	return gpio_get_value(MX28_PAD_SSP1_SCK__GPIO_2_12);
}

int board_mmc_init(bd_t *bis)
{
	/* Configure WP as input */
	gpio_direction_input(MX28_PAD_SSP1_SCK__GPIO_2_12);

	/* Configure MMC0 Power Enable */
	//gpio_direction_output(MX28_PAD_PWM3__GPIO_3_28, 0);

	return mxsmmc_initialize(bis, 0, mx28evk_mmc_wp, NULL);
}
#endif

#ifdef	CONFIG_CMD_NET

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	/* MX28_ZOLL_AED3 uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
	       &clkctrl_regs->hw_clkctrl_enet);

	/* Power-on FECs */
	gpio_direction_output(MX28_PAD_SSP1_DATA3__GPIO_2_15, 0);

	/* Reset FEC PHYs */
	gpio_direction_output(MX28_PAD_ENET0_RX_CLK__GPIO_4_13, 0);
	udelay(200);
	gpio_set_value(MX28_PAD_ENET0_RX_CLK__GPIO_4_13, 1);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	ret = fecmxc_initialize_multi(bis, 1, 3, MXS_ENET1_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC1\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

	dev = eth_get_dev_by_name("FEC1");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC1 device entry\n");
		return -EINVAL;
	}

	return ret;
}

#endif
