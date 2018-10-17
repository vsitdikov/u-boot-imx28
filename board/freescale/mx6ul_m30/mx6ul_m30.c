/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <div64.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-fsl.h>
#include <asm/imx-common/video.h>
#include <watchdog.h>

#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../common/pfuze.h"

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_DAT3_CD_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	\
	PAD_CTL_PUS_100K_DOWN  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define WEIM_NOR_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | \
		PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
		PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

/*from CCM_CSCDR2. source clock for LCD shall be PLL2*/
#define LCDIF1_PRE_CLK_SEL	(1 << 17 | 1 << 16 | 1 << 15)
#define LCDIF1_CLK_SEL		(1 << 11 | 1 << 10 | 1 << 9)

/* I2C2 for PMIC */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode =  MX6_PAD_GPIO1_IO00__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO1_IO00__GPIO1_IO00 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 0),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO01__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO1_IO01__GPIO1_IO01 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 1),
	},
};

/* I2C3 for SHT20 */
static struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART1_TX_DATA__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART1_TX_DATA__GPIO1_IO16 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 16),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART1_RX_DATA__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART1_RX_DATA__GPIO1_IO17 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 17),
	},
};

#define I2C_PMIC 1
int power_init_board(void)
{
	struct pmic *pfuze;
	int ret;
	unsigned int reg, rev_id;

	ret = power_pfuze3000_init(I2C_PMIC);
	if (ret)
		return ret;

	pfuze = pmic_get("PFUZE3000");
	ret = pmic_probe(pfuze);
	if (ret)
		return ret;

	pmic_reg_read(pfuze, PFUZE3000_DEVICEID, &reg);
	pmic_reg_read(pfuze, PFUZE3000_REVID, &rev_id);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n",
	       reg, rev_id);

	/* disable Low Power Mode during standby mode */
	pmic_reg_read(pfuze, PFUZE3000_LDOGCTL, &reg);
	reg |= 0x1;
	pmic_reg_write(pfuze, PFUZE3000_LDOGCTL, reg);

	/* SW1B step ramp up time from 2us to 4us/25mV */
	reg = 0x40;
	pmic_reg_write(pfuze, PFUZE3000_SW1BCONF, reg);

	/* SW1B mode to APS/PFM */
	reg = 0xc;
	pmic_reg_write(pfuze, PFUZE3000_SW1BMODE, reg);

	/* SW1B standby voltage set to 0.975V */
	reg = 0xb;
	pmic_reg_write(pfuze, PFUZE3000_SW1BSTBY, reg);

	return 0;
}

void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	u32 vddarm;

	struct pmic *p = pmic_get("PFUZE3000");

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		prep_anatop_bypass();
		/* decrease VDDARM to 1.275V */
		pmic_reg_read(p, PFUZE3000_SW1BVOLT, &value);
		value &= ~0x1f;
		value |= PFUZE3000_SW1AB_SETP(1275);
		pmic_reg_write(p, PFUZE3000_SW1BVOLT, value);

		set_anatop_bypass(1);
		vddarm = PFUZE3000_SW1AB_SETP(1175);

		pmic_reg_read(p, PFUZE3000_SW1BVOLT, &value);
		value &= ~0x1f;
		value |= vddarm;
		pmic_reg_write(p, PFUZE3000_SW1BVOLT, value);

		finish_anatop_bypass();

	} else {
		printf("ERROR ldo_bypass off!\n");
	}
}

#define WDT_FEED_PIN IMX_GPIO_NR(3, 4)
#define WDT_ENABLE_PIN IMX_GPIO_NR(4, 19)

/* Duration of WDT pulse in microseconds */
#define WDT_PULSE_DURATION_US 1

#ifdef CONFIG_WATCHDOG
static uint64_t usec_to_tick(unsigned long usec)
{
	uint64_t tick = usec;
	tick *= get_tbclk();
	do_div(tick, 1000000);
	return tick;
}

void watchdog_reset(void)
{
	uint64_t tmp;
	static int wdt_enabled = 0;

	gpio_direction_output(WDT_FEED_PIN, 1);

	/* We can't call udelay() directly because the first thing it
	does is call WATCHDOG_RESET(). So let's reimplement it.*/

	tmp = get_ticks() + usec_to_tick(WDT_PULSE_DURATION_US);
	while (get_ticks() < tmp + 1); /* do nothing */


	gpio_direction_output(WDT_FEED_PIN, 0);

	/* From the WDT control requirement doc the WD_ENABLE should be
	*  be enabled after the first toggle of the WD */
	if (wdt_enabled == 0) {
		gpio_direction_output(WDT_ENABLE_PIN, 1);
		wdt_enabled = 1;
	}

}

static iomux_v3_cfg_t const wdt_pads[] = {
	MX6_PAD_LCD_RESET__GPIO3_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI_VSYNC__GPIO4_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void setup_iomux_wdt(void) {
	imx_iomux_v3_setup_multiple_pads(wdt_pads, ARRAY_SIZE(wdt_pads));
}
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_UART2_TX_DATA__UART2_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART2_RX_DATA__UART2_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_cd_pads[] = {
	/*
	 * The evk board uses DAT3 to detect CD card plugin,
	 * in u-boot we mux the pin to GPIO when doing board_mmc_getcd.
	 */
	MX6_PAD_NAND_DATA03__GPIO4_IO05 | MUX_PAD_CTRL(USDHC_DAT3_CD_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_dat3_pads[] = {
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_DAT3_CD_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

#define RELAY_PRT_RES	IMX_GPIO_NR(4, 15)
#define RELAY_Y1_SET	IMX_GPIO_NR(4, 10)
#define RELAY_Y1_RES	IMX_GPIO_NR(4, 9)
#define RELAY_Y2_RES	IMX_GPIO_NR(4, 16)
#define RELAY_Y2_SET	IMX_GPIO_NR(4, 17)
#define RELAY_W1_SET	IMX_GPIO_NR(4, 14)
#define RELAY_W1_RES	IMX_GPIO_NR(4, 8)
#define RELAY_W2_RES	IMX_GPIO_NR(2, 0)
#define RELAY_W2_SET	IMX_GPIO_NR(2, 1)
#define RELAY_DH_RES	IMX_GPIO_NR(2, 5)
#define RELAY_DH_SET	IMX_GPIO_NR(2, 2)
#define RELAY_G_RES	IMX_GPIO_NR(4, 7)
#define RELAY_G_SET	IMX_GPIO_NR(4, 6)
#define RELAY_OB_RES	IMX_GPIO_NR(4, 11)
#define RELAY_OB_SET	IMX_GPIO_NR(4, 18)
#define RELAY_ACC_RES	IMX_GPIO_NR(2, 6)
#define RELAY_ACC_SET	IMX_GPIO_NR(2, 3)


#define RELAY_INIT_NUM	17
const  int iRelayInitInfTbl[RELAY_INIT_NUM] = {
    RELAY_PRT_RES,
    RELAY_DH_RES,
    RELAY_ACC_RES,
    RELAY_W2_RES,
    RELAY_W1_RES,
    RELAY_Y2_RES,
    RELAY_Y1_RES,
    RELAY_G_RES,
    RELAY_OB_RES,
    RELAY_OB_SET,
    RELAY_G_SET,
    RELAY_Y1_SET,
    RELAY_Y2_SET,
    RELAY_W1_SET,
    RELAY_W2_SET,
    RELAY_DH_SET,
    RELAY_DH_RES
};

#define RELAY_RESET_NUM		8
const  int iRelayResetInfTbl[RELAY_RESET_NUM] = {
    RELAY_DH_RES,
    RELAY_ACC_RES,
    RELAY_W2_RES,
    RELAY_W1_RES,
    RELAY_Y2_RES,
    RELAY_Y1_RES,
    RELAY_G_RES,
    RELAY_OB_RES
};

static iomux_v3_cfg_t const relay_pads[] = {
	MX6_PAD_NAND_CLE__GPIO4_IO15		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*PROT*/
	MX6_PAD_NAND_ALE__GPIO4_IO10 		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET Y1*/
	MX6_PAD_NAND_DATA07__GPIO4_IO09 	| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES Y1*/
	MX6_PAD_CSI_MCLK__GPIO4_IO17 		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET Y2*/
	MX6_PAD_NAND_DQS__GPIO4_IO16 		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES Y2*/
	MX6_PAD_NAND_CE1_B__GPIO4_IO14 		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET W1*/
	MX6_PAD_NAND_DATA06__GPIO4_IO08 	| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES W1*/
	MX6_PAD_ENET1_RX_DATA1__GPIO2_IO01	| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET W2*/
	MX6_PAD_ENET1_RX_DATA0__GPIO2_IO00	| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES W2*/
	MX6_PAD_ENET1_RX_EN__GPIO2_IO02		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET DH*/
	MX6_PAD_ENET1_TX_EN__GPIO2_IO05		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES DH*/
	MX6_PAD_NAND_DATA04__GPIO4_IO06		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET G*/
	MX6_PAD_NAND_DATA05__GPIO4_IO07		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES G*/
	MX6_PAD_CSI_PIXCLK__GPIO4_IO18		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET OB*/
	MX6_PAD_NAND_WP_B__GPIO4_IO11		| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES OB*/
	MX6_PAD_ENET1_TX_DATA0__GPIO2_IO03	| MUX_PAD_CTRL(NO_PAD_CTRL),	/*SET ACC*/
	MX6_PAD_ENET1_TX_CLK__GPIO2_IO06	| MUX_PAD_CTRL(NO_PAD_CTRL),	/*RES ACC*/
};

void setup_iomux_relay(void) {
	imx_iomux_v3_setup_multiple_pads(relay_pads, ARRAY_SIZE(relay_pads));
}

void reset_relays(void) {
	int i;

	for(i = 0; i < RELAY_INIT_NUM; i++)
	{
		gpio_direction_output(iRelayInitInfTbl[i], 0);
		if(i <  RELAY_INIT_NUM - 1)
		{
			udelay(10 * 1000);
		}
	}

	for (i = 0; i < RELAY_RESET_NUM; i++ )
	{
		gpio_direction_output(iRelayResetInfTbl[i], 1);
		udelay(20 * 1000);
		gpio_direction_output(iRelayResetInfTbl[i], 0);

		if ( i < RELAY_RESET_NUM - 1 )
		{
			udelay(10 * 1000);
		}
	}

}

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg = {
	USDHC2_BASE_ADDR, 0, 4,
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(4, 5)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	if (cfg->esdhc_base != USDHC2_BASE_ADDR) {
		printf("error: unexpected esdhc_base\n");
		return ret;
	}

	imx_iomux_v3_setup_multiple_pads(usdhc2_cd_pads, ARRAY_SIZE(usdhc2_cd_pads));
	gpio_direction_input(USDHC2_CD_GPIO);

	/*
	 * Since it is the DAT3 pin, this pin is pulled to
	 * low voltage if no card
	 */
	ret = gpio_get_value(USDHC2_CD_GPIO);

	imx_iomux_v3_setup_multiple_pads(usdhc2_dat3_pads, ARRAY_SIZE(usdhc2_dat3_pads));

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	ret = fsl_esdhc_initialize(bis, &usdhc_cfg);
	if (ret) {
		printf("Warning: failed to initialize mmc dev 2\n");
	}

	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

int board_usb_phy_mode(int port)
{
	return USB_INIT_HOST;
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port != 0)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD_CLK__LCDIF_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__LCDIF_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_HSYNC__LCDIF_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__LCDIF_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA00__LCDIF_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA01__LCDIF_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA02__LCDIF_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA03__LCDIF_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA04__LCDIF_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA05__LCDIF_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA06__LCDIF_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA07__LCDIF_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA08__LCDIF_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA09__LCDIF_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA10__LCDIF_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA11__LCDIF_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA12__LCDIF_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA13__LCDIF_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA14__LCDIF_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA15__LCDIF_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA16__LCDIF_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA17__LCDIF_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA18__LCDIF_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA19__LCDIF_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA20__LCDIF_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA21__LCDIF_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA22__LCDIF_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA23__LCDIF_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/* LCD_RST */
	MX6_PAD_SNVS_TAMPER9__GPIO5_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Use GPIO for Brightness adjustment, duty cycle = period. */
	MX6_PAD_GPIO1_IO08__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

void do_enable_parallel_lcd(struct display_info_t const *dev)
{
	enable_lcdif_clock(dev->bus);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Reset the LCD */
	gpio_direction_output(IMX_GPIO_NR(5, 9) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(5, 9) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 8) , 1);
}

struct display_info_t const displays[] = {{
	.bus = MX6UL_LCDIF1_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_parallel_lcd,
	.mode	= {
		.name			= "TFT43AB",
		.xres           = 480,
		.yres           = 272,
		.pixclock       = 108695,
		.left_margin    = 8,
		.right_margin   = 4,
		.upper_margin   = 2,
		.lower_margin   = 4,
		.hsync_len      = 41,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

/*
* Enable source clock for LCD to be on PLL2. Needed it for spread spectrum clock
*/
static void cscdr2_init(void)
{

	unsigned int reg;
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	reg = readl(&ccm->cscdr2);
	reg &= ~( (LCDIF1_PRE_CLK_SEL) | (LCDIF1_CLK_SEL) );

	writel(reg, &ccm->cscdr2);

}

int board_early_init_f(void)
{
	setup_iomux_uart();

#ifdef CONFIG_WATCHDOG
	setup_iomux_wdt();
#endif

	setup_iomux_relay();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);

	cscdr2_init();

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	setenv("board_name", "DENSOWAVE");
	setenv("board_rev", "M30");
#endif
	setenv("boot_reason", get_m30_boot_reason());
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);
	reset_relays();
	return 0;
}

int checkboard(void)
{
	puts("Board: MX6UL DensoWave M30\n");

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
		break;
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#endif /*CONFIG_FSL_FASTBOOT*/
