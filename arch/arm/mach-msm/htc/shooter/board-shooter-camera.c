/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2013 Sebastian Sobczyk <sebastiansobczyk@wp.pl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/mach-types.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>
#include <asm/setup.h>
#include "devices-msm8x60.h"
#include "devices.h"
#include "board-shooter.h"

#include <linux/spi/spi.h>
#include <mach/rpm-regulator.h>

#ifdef CONFIG_MSM_CAMERA_FLASH
#include <linux/htc_flashlight.h>
#include <linux/leds.h>
#endif

struct platform_device msm_camera_sensor_webcam;

static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

#ifdef CONFIG_MSM_CAMERA

static void shooter_set_gpio(int gpio, int state)
{
	gpio_set_value(gpio, state);
}

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

#if 1 //CONFIG_SP3D TODO need to add in
static uint32_t camera_off_gpio_table_sp3d[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_SP3D_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/*sharp INT*/
	GPIO_CFG(SHOOTER_SP3D_SPI_DO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SYS_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_PDX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera reset*/
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera standby*/
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera switch*/
};

static uint32_t camera_on_gpio_table_sp3d[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SYS_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_PDX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#if 0 //Need to add in
static uint32_t sp3d_spi_gpio[] = {
	/* or this? the i/o direction and up/down are much more correct */
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
};
#endif
#endif

#ifdef CONFIG_QS_S5K4E1
static uint32_t camera_off_gpio_table_liteon[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),/*i2c*/
	GPIO_CFG(SHOOTER_SP3D_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/*sharp INT*/
	GPIO_CFG(SHOOTER_SP3D_SPI_DO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_VCM_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_INTB, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera reset*/
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera standby*/
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera switch*/
};

static uint32_t camera_on_gpio_table_liteon[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_SP3D_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DO,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_DI,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CS,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_SP3D_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_S5K4E1_VCM_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_S5K4E1_INTB, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_S5K4E1_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#endif

struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static int shooter_config_camera_on_gpios(void)
{
	if (engineerid == 7) {
		config_gpio_table(camera_on_gpio_table_liteon,
			ARRAY_SIZE(camera_on_gpio_table_liteon));
	} else {
		config_gpio_table(camera_on_gpio_table_sp3d,
			ARRAY_SIZE(camera_on_gpio_table_sp3d));
	}

	return 0;
}

static void shooter_config_camera_off_gpios(void)
{
	if (engineerid == 7) {
		config_gpio_table(camera_off_gpio_table_liteon,
			ARRAY_SIZE(camera_off_gpio_table_liteon));
	} else {
		config_gpio_table(camera_off_gpio_table_sp3d,
			ARRAY_SIZE(camera_off_gpio_table_sp3d));
	}
	shooter_set_gpio(SHOOTER_SP3D_SPI_DO, 0);
	shooter_set_gpio(SHOOTER_SP3D_SPI_CS, 0);
	shooter_set_gpio(SHOOTER_SP3D_SPI_CLK, 0);
	shooter_set_gpio(SHOOTER_SP3D_MCLK, 0);
	shooter_set_gpio(SHOOTER_CAM_SEL, 0);
}

static struct msm_camera_device_platform_data msm_camera_device_data_web_cam = {
	.camera_gpio_on  = shooter_config_camera_on_gpios,
	.camera_gpio_off = shooter_config_camera_off_gpios,
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

static struct regulator *shooter_reg_8058_l3 = NULL;
static struct regulator *shooter_reg_8058_l8 = NULL;
static struct regulator *shooter_reg_8058_l9 = NULL;
// TODO not used yet static struct regulator *shooter_reg_8058_l10 = NULL;
//static struct regulator *shooter_reg_8058_l15 = NULL;

static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power)
{
	int rc;

	if (power == NULL)
		return -ENODEV;

	*sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}
	rc = regulator_set_voltage(*sensor_power, volt, volt);
	if (rc) {
		pr_err("[CAM] %s: unable to set %s voltage to %d rc:%d\n",
			__func__, power, volt, rc);
		regulator_put(*sensor_power);
		*sensor_power = NULL;
		return -ENODEV;
	}
	rc = regulator_enable(*sensor_power);
	if (rc) {
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);
		regulator_put(*sensor_power);
		*sensor_power = NULL;
		return -ENODEV;
	}

	return rc;
}

static struct regulator *shooter_reg_8901_lvs2 = NULL;
static struct regulator *shooter_reg_8901_lvs3 = NULL;

static int camera_sensor_power_enable_8901(char *power, struct regulator **sensor_power)
{
	int rc;
	pr_info("%s %s", __func__, power);
	if (power == NULL)
		return -ENODEV;

	*sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}
	rc = regulator_enable(*sensor_power);
	if (rc) {
		pr_err("[CAM] %s: Enable regulator %s failed\n", __func__, power);
		regulator_put(*sensor_power);
		*sensor_power = NULL;
		return -ENODEV;
	}

	return rc;
}

static int camera_sensor_power_disable(struct regulator *sensor_power)
{
	int rc;

	if (sensor_power == NULL)
		return -ENODEV;

	if (IS_ERR(sensor_power)) {
		pr_err("[CAM] %s: Invalid regulator ptr\n", __func__);
		return -ENODEV;
	}
	rc = regulator_disable(sensor_power);
	if (rc) {
		pr_err("[CAM] %s: Disable regulator failed\n", __func__);
		regulator_put(sensor_power);
		sensor_power = NULL;
		return -ENODEV;
	}
	regulator_put(sensor_power);
	sensor_power = NULL;
	return rc;
}

static void Shooter_seccam_clk_switch(void)
{
	pr_info("[CAM] Doing clk switch (2nd Cam)\n");
	gpio_set_value(SHOOTER_CAM_SEL, 1);
}

static int Shooter_s5k6aafx_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	/* main / 2nd camera analog power */
	rc = camera_sensor_power_enable("8058_l3", 2850000, &shooter_reg_8058_l3);
	pr_info("[CAM] sensor_power_enable(\"8058_l3\", 2850) == %d\n", rc);
	mdelay(5);
	/* main / 2nd camera digital power */
	if (system_rev == 2 && engineerid >= 3) {
		rc = camera_sensor_power_enable_8901("8901_lvs2", &shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
		mdelay(5);
		/*IO*/
		rc = camera_sensor_power_enable_8901("8901_lvs3", &shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
		mdelay(1);
	} else {
		rc = camera_sensor_power_enable("8058_l8", 1800000, &shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
		mdelay(5);
		/*IO*/
		rc = camera_sensor_power_enable("8058_l9", 1800000, &shooter_reg_8058_l9);
		pr_info("[CAM]s ensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
		mdelay(1);
	}
	return rc;
}

static int Shooter_s5k6aafx_vreg_off(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	/* IO power off */
	if (system_rev == 2 && engineerid >= 3) {
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_disable(\"8901_lvs3\") == %d\n", rc);
		mdelay(1);

		/* main / 2nd camera digital power */
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_disable(\"8901_lvs2\") == %d\n", rc);
	} else {
		rc = camera_sensor_power_disable(shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);
		mdelay(1);

		/* main / 2nd camera digital power */
		rc = camera_sensor_power_disable(shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);
	}
	mdelay(1);
	/* main / 2nd camera analog power */
	rc = camera_sensor_power_disable(shooter_reg_8058_l3);
	pr_info("[CAM] sensor_power_disable(\"8058_l3\") == %d\n", rc);
	return rc;
}


static struct msm_camera_sensor_flash_data flash_s5k6aafx = {
	.flash_type		= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name	= "s5k6aafx",
	.sensor_reset	= SHOOTER_WEBCAM_RST,/*2nd Cam RST*/
	.sensor_pwd		= SHOOTER_WEBCAM_STB,/*2nd Cam PWD*/
	.vcm_enable		= 0,
	.camera_power_on = Shooter_s5k6aafx_vreg_on,
	.camera_power_off = Shooter_s5k6aafx_vreg_off,
	.camera_clk_switch = Shooter_seccam_clk_switch,
	.pdata			= &msm_camera_device_data_web_cam,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data             = &flash_s5k6aafx,
	.mirror_mode = 0,
	.csi_if		= 1,
	.dev_node	= 1,
};

#ifdef CONFIG_I2C
static struct i2c_board_info msm_camera_boardinfo[] = {
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
	{
		I2C_BOARD_INFO("s5k6aafx", 0x5a >> 1), /* COB type */
	},
#ifdef CONFIG_QS_S5K4E1
	{
		I2C_BOARD_INFO("qs_s5k4e1", 0x20),
	},
#endif
};
#endif  

void __init msm8x60_init_cam(void)
{
	pr_info("%s", __func__);

	msm_camera_sensor_webcam.name = "msm_camera_webcam";
	msm_camera_sensor_webcam.dev.platform_data = &msm_camera_sensor_s5k6aafx_data;
	platform_device_register(&msm_camera_sensor_webcam);

	i2c_register_board_info(MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo));

	platform_device_register(&msm_camera_server);
}
#endif	
