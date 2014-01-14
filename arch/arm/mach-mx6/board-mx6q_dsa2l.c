/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sabresd.h"
#include "board-mx6dl_sabresd.h"
#include <mach/imx_rfkill.h>

#define DSA2L_VGA_RST			IMX_GPIO_NR(1, 1)
#define SABRESD_MICROPHONE_DET	IMX_GPIO_NR(1, 9)
#define DSA2L_PMIC_GPIO_RST		IMX_GPIO_NR(1, 5)
#define DSA2L_BOARD_ID0			IMX_GPIO_NR(1, 17)
#define DSA2L_BOARD_ID1			IMX_GPIO_NR(1, 19)
#define DSA2L_BOARD_ID2			IMX_GPIO_NR(1, 21)
#define SABRESD_RGMII_RST		IMX_GPIO_NR(1, 25)
#define SABRESD_RGMII_INT		IMX_GPIO_NR(1, 26)
#define DSA2L_EMMC_RST			IMX_GPIO_NR(1, 29)
#define DSA2L_RTC_INT			IMX_GPIO_NR(1, 30)

#define SABRESD_SD2_CD		IMX_GPIO_NR(2, 2)
#define SABRESD_SD2_WP		IMX_GPIO_NR(2, 3)

#define SABRESD_ECSPI1_CS0		IMX_GPIO_NR(4, 9)
#define SABRESD_CODEC_PWR_EN	IMX_GPIO_NR(4, 10) //NC
#define DSA2L_BTN_RESET			IMX_GPIO_NR(4, 15)

#define SABRESD_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define DSA2L_PMIC_INT			IMX_GPIO_NR(7, 11)
#define DSA2L_LED_R				IMX_GPIO_NR(7, 3)
#define DSA2L_LED_B				IMX_GPIO_NR(7, 2)
#define DSA2L_WIFI_DISABLE		IMX_GPIO_NR(7, 4)
#define DSA2L_USB_OTG_PWR_IN	IMX_GPIO_NR(7, 5)
#define DSA2L_VGA_PWR_EN		IMX_GPIO_NR(7, 6)
#define DSA2L_VGA_CABLE_IN		IMX_GPIO_NR(7, 7)
#define DSA2L_BTN_POWER			IMX_GPIO_NR(7, 13)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

static struct clk *sata_clk;
static struct clk *clko;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;

static const struct esdhc_platform_data mx6q_sabresd_sd2_data __initconst = {
	.cd_gpio = SABRESD_SD2_CD,
	.wp_gpio = SABRESD_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct esdhc_platform_data mx6q_sabresd_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_sabresd_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_sabresd_init_uart(void)
{
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(0, NULL);
}

static int mx6q_sabresd_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabresd_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6q_sabresd_spi_cs[] = {
	SABRESD_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_sabresd_spi_data __initconst = {
	.chipselect     = mx6q_sabresd_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabresd_spi_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_sabresd_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_sabresd__spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_sabresd_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_sabresd_spi_nor_partitions),
	.type = "sst25vf016b",
};
#endif

static struct spi_board_info imx6_sabresd_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &imx6_sabresd__spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabresd_spi_nor_device,
				ARRAY_SIZE(imx6_sabresd_spi_nor_device));
}

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sabresd_audio_wm8962_device = {
	.name = "imx-wm8962",
};

static struct mxc_audio_platform_data wm8962_data;

static int wm8962_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static int mxc_wm8962_init(void)
{
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 24000000);
	clk_set_rate(clko, rate);

	wm8962_data.sysclk = rate;

	return 0;
}

static struct wm8962_pdata wm8962_config_data = {
	.gpio_init = {
		[2] = WM8962_GPIO_FN_DMICCLK,
		[4] = 0x8000 | WM8962_GPIO_FN_DMICDAT,
	},
};

static struct mxc_audio_platform_data wm8962_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = SABRESD_HEADPHONE_DET,
	.hp_active_low = 1,
	.mic_gpio = -1,
	.mic_active_low = 1,
	.init = mxc_wm8962_init,
	.clock_enable = wm8962_clk_enable,
};

static struct regulator_consumer_supply sabresd_vwm8962_consumers[] = {
	REGULATOR_SUPPLY("SPKVDD1", "0-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "0-001a"),
};

static struct regulator_init_data sabresd_vwm8962_init = {
	.constraints = {
		.name = "SPKVDD",
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vwm8962_consumers),
	.consumer_supplies = sabresd_vwm8962_consumers,
};

static struct fixed_voltage_config sabresd_vwm8962_reg_config = {
	.supply_name	= "SPKVDD",
	.microvolts		= 4200000,
	.gpio			= SABRESD_CODEC_PWR_EN,
	.enable_high	= 1,
	.enabled_at_boot = 1,
	.init_data		= &sabresd_vwm8962_init,
};

static struct platform_device sabresd_vwm8962_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id		= 4,
	.dev	= {
		.platform_data = &sabresd_vwm8962_reg_config,
	},
};


static struct imxi2c_platform_data mx6q_sabresd_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm89**", 0x1a),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static void __init imx6q_sabresd_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabresd_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabresd_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabresd_sata_data = {
	.init = mx6q_sabresd_sata_init,
	.exit = mx6q_sabresd_sata_exit,
};
#endif

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data sabresd_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sabresd board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_sabresd_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_sabresd_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_i2c2_pads,
			ARRAY_SIZE(mx6dl_sabresd_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_i2c2_pads,
			ARRAY_SIZE(mx6q_sabresd_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 1,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	//.mode = LDB_SEP1,
	.mode = LDB_DUL_DI1, // [Walker Chen], bouth lvds chennal used DISP1
	.sec_ipu_id = 0,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct max8903_pdata charger1_data = {
	.dok =  -1,
	.uok = -1,
	.chg = -1,
	.flt = -1,
	.dcm_always_high = true,
	.dc_valid = true,
	.usb_valid = true,
};

static struct platform_device sabresd_max8903_charger_1 = {
	.name	= "max8903-charger",
	.id	= 1,
	.dev	= {
		.platform_data = &charger1_data,
	},
};

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 1,
		},
	},
};

static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	//gpio_set_value(SABRESD_AUX_5V_EN, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	//gpio_set_value(SABRESD_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sabresd_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabresd_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vmmc_consumers),
	.consumer_supplies = sabresd_vmmc_consumers,
};

static struct fixed_voltage_config sabresd_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabresd_vmmc_init,
};

static struct platform_device sabresd_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabresd_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	platform_device_register(&sabresd_vwm8962_reg_devices);
	mxc_register_device(&mx6_sabresd_audio_wm8962_device,
			    &wm8962_data);
	imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

	mxc_wm8962_init();
	return 0;
}

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= "max8903-"trigger,		\
}

/* use to show a external power source is connected
 * GPIO_LED(SABRESD_CHARGE_DONE, "chg_detect", 0, 1, "ac-online"),
 */
static struct gpio_led imx6q_gpio_leds[] = {
	//GPIO_LED(SABRESD_CHARGE_NOW, "chg_now_led", 0, 1,
	//	"charger-charging"),
	{
		.gpio			= DSA2L_LED_R,
		.name			= "red_led",
		.active_low		= 0,
		.retain_state_suspended = 1,
		.default_state		= 0,
	},
	{
		.gpio			= DSA2L_LED_B,
		.name			= "blue_led",
		.active_low		= 0,
		.retain_state_suspended = 1,
		.default_state		= 1,
		.default_trigger	= "timer",	
	},

/* For the latest B4 board, this GPIO_1 is connected to POR_B,
which will reset the whole board if this pin's level is changed,
so, for the latest board, we have to avoid using this pin as
GPIO.
	GPIO_LED(SABRESD_CHARGE_DONE, "chg_done_led", 0, 1,
			"charger-full"),
*/
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

/* For BT_PWD_L is conflict with charger's LED trigger gpio on sabresd_revC.
 * add mutual exclusion here to be decided which one to be used by board config
 */
static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button new_sabresd_buttons[] = {
	//GPIO_BUTTON(SABRESD_VOLUME_UP, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	//GPIO_BUTTON(SABRESD_VOLUME_DN, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
	//GPIO_BUTTON(SABRESD_POWER_OFF, KEY_POWER, 1, "power-key", 1, 1),
	GPIO_BUTTON(DSA2L_BTN_POWER, KEY_POWER, 0, "power-key", 1, 1),
	GPIO_BUTTON(DSA2L_BTN_RESET, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
};

static struct gpio_keys_platform_data new_sabresd_button_data = {
	.buttons	= new_sabresd_buttons,
	.nbuttons	= ARRAY_SIZE(new_sabresd_buttons),
};

static struct platform_device sabresd_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
};

static void __init imx6q_add_device_buttons(void)
{
	/* fix me */
	/* For new sabresd(RevB4 ane above) change the
	 * ONOFF key(SW1) design, the SW1 now connect
	 * to GPIO_3_29, it can be use as a general power
	 * key that Android reuired. But those old sabresd
	 * such as RevB or older could not support this
	 * change, so it needs a way to distinguish different
	 * boards. Before board id/rev are defined cleary,
	 * there is a simple way to achive this, that is using
	 * SOC revison to identify differnt board revison.
	 *
	 * With the new sabresd change and SW mapping the
	 * SW1 as power key, below function related to power
	 * key are OK on new sabresd board(B4 or above).
	 * 	1 Act as power button to power on the device when device is power off
	 * 	2 Act as power button to power on the device(need keep press SW1 >5s)
	 *	3 Act as power key to let device suspend/resume
	 *	4 Act screenshort(hold power key and volume down key for 2s)
	 */
	platform_device_add_data(&sabresd_button_device,
				&new_sabresd_button_data,
				sizeof(new_sabresd_button_data));

	platform_device_register(&sabresd_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_sabresd_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabresd_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = sabresd_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(sabresd_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
	
}
static void arch_poweroff(void)
{
	// -> [Walker Chen], 2013/12/11 - added system poweroff
	gpio_request( DSA2L_PMIC_GPIO_RST , "GPIO_RST" );
	gpio_direction_output( DSA2L_PMIC_GPIO_RST , 1 );
	gpio_set_value( DSA2L_PMIC_GPIO_RST , 0 );
	//msleep(100);//100ms
	//gpio_set_value( DSA2L_PMIC_GPIO_RST , 1 );
	gpio_free( DSA2L_PMIC_INT );
	// <- End.	
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 0,		/* dissable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_rx_clk		= 0,    /* rx clk from spdif stream */
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

// -> [Walker Chen], 2013/12/26 - DSA2L board gpio init 
static inline void dsa2l_init(void)
{
	// PMIC
	gpio_request( DSA2L_PMIC_GPIO_RST , "GPIO_RST" );
	gpio_direction_output( DSA2L_PMIC_GPIO_RST , 1 );
	gpio_set_value( DSA2L_PMIC_GPIO_RST , 1 );
	gpio_free( DSA2L_PMIC_INT );

	gpio_request( DSA2L_PMIC_INT , "PMIC_INT" );
	gpio_direction_input( DSA2L_PMIC_INT );
	gpio_free( DSA2L_PMIC_INT );

	// EMMC
	gpio_request( DSA2L_EMMC_RST , "EMMC_RST" );
	gpio_direction_output( DSA2L_EMMC_RST , 1 );
	gpio_set_value( DSA2L_EMMC_RST , 1 );
	gpio_free( DSA2L_EMMC_RST);

	// LEDs
	gpio_request( DSA2L_LED_R , "LED_R" );
	gpio_direction_output( DSA2L_LED_R , 1 );
	gpio_set_value( DSA2L_LED_R , 0 );
	gpio_free( DSA2L_LED_R );

	gpio_request( DSA2L_LED_B , "LED_B" );
	gpio_direction_output( DSA2L_LED_B , 1 );
	gpio_set_value( DSA2L_LED_B , 1 );
	gpio_free( DSA2L_LED_B );

	// WIFI disable
	gpio_request( DSA2L_WIFI_DISABLE , "WIFI_DISABLE" );
	gpio_direction_output( DSA2L_WIFI_DISABLE , 1 );
	gpio_set_value( DSA2L_WIFI_DISABLE , 0 );
	gpio_free( DSA2L_WIFI_DISABLE );

	// USB PWR_IN
	gpio_request( DSA2L_USB_OTG_PWR_IN , "USB_OTG_PWR_IN" );
	gpio_direction_input( DSA2L_USB_OTG_PWR_IN );
	gpio_free( DSA2L_USB_OTG_PWR_IN );

	// VGA
	gpio_request( DSA2L_VGA_RST , "VGA_RST" );
	gpio_direction_output( DSA2L_VGA_RST , 1 );
	//gpio_set_value( DSA2L_VGA_RST , 0 );
	//msleep(50);//50ms
	gpio_set_value( DSA2L_VGA_RST , 1 );
	gpio_free( DSA2L_VGA_RST );

	gpio_request( DSA2L_VGA_PWR_EN , "VGA_PWR_EN" );
	gpio_direction_output( DSA2L_VGA_PWR_EN , 1 );
	gpio_set_value( DSA2L_VGA_PWR_EN , 1 );
	gpio_free( DSA2L_VGA_PWR_EN );

	gpio_request( DSA2L_VGA_CABLE_IN , "VGA_CABLE_IN" );
	gpio_direction_input( DSA2L_VGA_CABLE_IN );
	gpio_free( DSA2L_VGA_CABLE_IN );

	// board-id
	gpio_request( DSA2L_BOARD_ID0 , "BOARD_ID0" );
	gpio_direction_input( DSA2L_BOARD_ID0 );
	gpio_free( DSA2L_BOARD_ID0 );

	gpio_request( DSA2L_BOARD_ID1 , "BOARD_ID1" );
	gpio_direction_input( DSA2L_BOARD_ID1 );
	gpio_free( DSA2L_BOARD_ID1 );

	gpio_request( DSA2L_BOARD_ID2 , "BOARD_ID2" );
	gpio_direction_input( DSA2L_BOARD_ID2 );
	gpio_free( DSA2L_BOARD_ID2 );

	// RTC
	gpio_request( DSA2L_RTC_INT , "RTC_INT" );
	gpio_direction_input( DSA2L_RTC_INT );
	gpio_free( DSA2L_RTC_INT );

	// Btns
	gpio_request( DSA2L_BTN_RESET , "BTN_RESET" );
	gpio_direction_input( DSA2L_BTN_RESET );
	gpio_free( DSA2L_BTN_RESET );

	gpio_request( DSA2L_BTN_POWER , "BTN_POWER" );
	gpio_direction_input( DSA2L_BTN_POWER );
	gpio_free( DSA2L_BTN_POWER );
}
// <- End.

/*!
 * Board specific initialization.
 */
static void __init mx6_sabresd_board_init(void)
{
	int i;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_pads,
			ARRAY_SIZE(mx6q_sabresd_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabresd_pads,
			ARRAY_SIZE(mx6dl_sabresd_pads));
	}

	dsa2l_init();

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = sabresd_dvfscore_data.reg_id;
	soc_reg_id = sabresd_dvfscore_data.soc_id;
	mx6q_sabresd_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		/* [Walker Chen], 2014/01/08 - modified for dual display mode
			mx6dl only have one ipu
			ipu0:disp0 for HDMI
			ipu0:disp1 for LVDS
		*/
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 1;
		hdmi_core_data.ipu_id = 0;
		hdmi_core_data.disp_id = 0;
		//mipi_dsi_pdata.ipu_id = 0;
		//mipi_dsi_pdata.disp_id = 1;
		ldb_data.sec_ipu_id = 0;
		ldb_data.sec_disp_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);

	imx6q_add_vdoa();
	//imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	//imx6q_add_v4l2_capture(0, &capture_data[0]);
	//imx6q_add_v4l2_capture(1, &capture_data[1]);
	//imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	strcpy(mxc_i2c0_board_info[0].type, "wm8962");
	mxc_i2c0_board_info[0].platform_data = &wm8962_config_data;
	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabresd_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	//i2c_register_board_info(2, mxc_i2c2_board_info,
	//		ARRAY_SIZE(mxc_i2c2_board_info));
	mx6q_dsa2l_init_wm8326();

	/* spdif */
	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();
	
	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabresd_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);
	imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif

	imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabresd_sd4_data);
	//imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabresd_sd3_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabresd_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabresd_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabresd_sata_data);
#else
		mx6q_sabresd_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabresd_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	//gpio_request(SABRESD_CABC_EN0, "cabc-en0");
	//gpio_direction_output(SABRESD_CABC_EN0, 0);
	//gpio_request(SABRESD_CABC_EN1, "cabc-en1");
	//gpio_direction_output(SABRESD_CABC_EN1, 0);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabresd_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabresd_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_device_buttons();

	///* enable sensor 3v3 and 1v8 */
	//gpio_request(SABRESD_SENSOR_EN, "sensor-en");
	//gpio_direction_output(SABRESD_SENSOR_EN, 1);

	/* enable ecompass intr */
	//gpio_request(SABRESD_eCOMPASS_INT, "ecompass-int");
	//gpio_direction_input(SABRESD_eCOMPASS_INT);
	/* enable light sensor intr */
	//gpio_request(SABRESD_ALS_INT, "als-int");
	//gpio_direction_input(SABRESD_ALS_INT);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Enable Aux_5V */
	//gpio_request(SABRESD_AUX_5V_EN, "aux_5v_en");
	//gpio_direction_output(SABRESD_AUX_5V_EN, 1);
	//gpio_set_value(SABRESD_AUX_5V_EN, 1);

	//gps_power_on(true);
	/* Register charger chips */
	platform_device_register(&sabresd_max8903_charger_1);
	//pm_power_off = mx6_snvs_poweroff;
	pm_power_off = arch_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support
	 * uart5 has pin mux with pcie. or you will use uart5 or use pcie
	 */
	if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_elan_pads,
						ARRAY_SIZE(mx6dl_arm2_elan_pads));
	}

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_sabresd_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabresd_timer = {
	.init   = mx6_sabresd_timer_init,
};

static void __init mx6q_sabresd_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(sabresd_fb_data);
	if (fb_array_size > 0 && sabresd_fb_data[0].res_base[0] &&
	    sabresd_fb_data[0].res_size[0]) {
		if (sabresd_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    sabresd_fb_data[0].res_base[0]);
		memblock_reserve(sabresd_fb_data[0].res_base[0],
				 sabresd_fb_data[0].res_size[0]);
		memblock_remove(sabresd_fb_data[0].res_base[0],
				sabresd_fb_data[0].res_size[0]);
		sabresd_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (sabresd_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(sabresd_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, sabresd_fb_data[i].res_size[0]);
			sabresd_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_SABRESD data structure.
 */
MACHINE_START(MX6Q_SABRESD, "Freescale i.MX 6Quad/DualLite/Solo Sabre-SD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabresd_board_init,
	.timer = &mx6_sabresd_timer,
	.reserve = mx6q_sabresd_reserve,
MACHINE_END
