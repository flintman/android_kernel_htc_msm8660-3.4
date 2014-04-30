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

static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power);
static int camera_sensor_power_disable(struct regulator *sensor_power);
/*
static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};
*/

static int shooter_config_camera_on_gpios(void);
static void shooter_config_camera_off_gpios(void);

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 566231040,
		.ib  = 905969664,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 706199040,
		.ib  = 1129918464,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 320864256,
		.ib  = 513382810,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 320864256,
		.ib  = 513382810,
	},
};

static struct msm_bus_vectors cam_stereo_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 212336640,
		.ib  = 339738624,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 25090560,
		.ib  = 40144896,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 239708160,
		.ib  = 383533056,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 79902720,
		.ib  = 127844352,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_stereo_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 300902400,
		.ib  = 481443840,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 230307840,
		.ib  = 368492544,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 245113344,
		.ib  = 392181351,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 106536960,
		.ib  = 170459136,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 106536960,
		.ib  = 170459136,
	},
};

static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_video_vectors),
		cam_stereo_video_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_snapshot_vectors),
		cam_stereo_snapshot_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
	.camera_gpio_on  = shooter_config_camera_on_gpios,
	.camera_gpio_off = shooter_config_camera_off_gpios,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
	.csid_core = 0,
	.is_csic = 1,
	.is_vpe = 1,
	.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
	.camera_gpio_on  = shooter_config_camera_on_gpios,
	.camera_gpio_off = shooter_config_camera_off_gpios,
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
	.csid_core = 1,
	.is_csic = 1,
	.is_vpe = 1,
	.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

int aat1271_flashlight_control(int mode);
#ifdef CONFIG_MSM_CAMERA_FLASH
static int flashlight_control(int mode)
{
#if CONFIG_FLASHLIGHT_AAT
	return aat1271_flashlight_control(mode);
#else
	return 0;
#endif
}

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	.camera_flash = flashlight_control,
};


static struct regulator *shooter_reg_8058_l9 = NULL;
static struct regulator *shooter_reg_8058_l10 = NULL;
static struct regulator *shooter_reg_8058_l12 = NULL;
static struct regulator *shooter_reg_8058_l15 = NULL;

static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power)
{
	int rc;

	if (power == NULL)
		return -ENODEV;

	*sensor_power = regulator_get(NULL, power);

	if (IS_ERR(*sensor_power)) {
		pr_info("%s: failed to Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	/*
	if (volt != 1800000) {
	*/
		rc = regulator_set_voltage(*sensor_power, volt, volt);
		if (rc < 0) {
			pr_info("%s: failed to unable to set %s voltage to %d rc:%d\n",
					__func__, power, volt, rc);
			regulator_put(*sensor_power);
			*sensor_power = NULL;
			return -ENODEV;
		}
	/*
	}
	*/

	rc = regulator_enable(*sensor_power);
	if (rc < 0) {
		pr_info("%s: failed to Enable regulator %s failed\n", __func__, power);
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
		pr_info("%s: failed to Invalid requlator ptr\n", __func__);
		return -ENODEV;
	}

	rc = regulator_disable(sensor_power);
	if (rc < 0)
		pr_info("%s: disable regulator failed\n", __func__);

	regulator_put(sensor_power);
	sensor_power = NULL;
	return rc;
}

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(SHOOTER_CAM1_RSTz, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM2_RSTz, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM2_PWDN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_MCLK_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_CAM1_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(SHOOTER_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
	GPIO_CFG(SHOOTER_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(SHOOTER_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
	GPIO_CFG(SHOOTER_CAM1_RSTz, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM2_RSTz, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM2_PWDN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_MCLK_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(SHOOTER_CAM_CAM1_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

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

static int shooter_config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void shooter_config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

#ifdef CONFIG_QS_S5K4E1
static int Shooter_qs_s5k4e1_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	mdelay(50);

	rc = camera_sensor_power_enable("8058_l15", 2800000, &shooter_reg_8058_l15);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 1800) == %d\n", rc);
	udelay(50);

	if (system_rev >= 2 && engineerid >= 3) {/*VERSION A*/
		/*IO*//*This is switch power*/
		rc = camera_sensor_power_enable_8901("8901_lvs3", &shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
		mdelay(1);

		rc = camera_sensor_power_enable_8901("8901_lvs2", &shooter_reg_8901_lvs2);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs2\", 1800) == %d\n", rc);
	} else {
		rc = camera_sensor_power_enable("8058_l9", 1800000, &shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
		/* VDDIO*/
		rc = camera_sensor_power_enable("8058_l8", 1800000, &shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
	}
	udelay(50);

	/* main camera AVDD */
	rc = camera_sensor_power_enable("8058_l10", 2800000, &shooter_reg_8058_l10);
	pr_info("[CAM] sensor_power_enable(\"8058_l10\", 2800) == %d\n", rc);
	udelay(50);

	shooter_set_gpio(SHOOTER_S5K4E1_INTB, 1);
	shooter_set_gpio(SHOOTER_S5K4E1_PD, 1);
	shooter_set_gpio(SHOOTER_S5K4E1_VCM_PD, 1);

	return rc;
}

static int Shooter_qs_s5k4e1_vreg_off(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);
	shooter_set_gpio(SHOOTER_S5K4E1_INTB, 0); /* interrupt */
	shooter_set_gpio(SHOOTER_S5K4E1_VCM_PD, 0); /* PDX */
	shooter_set_gpio(SHOOTER_S5K4E1_PD, 0); /* RST */

	/* main camera AVDD */
	rc = camera_sensor_power_disable(shooter_reg_8058_l10);
	udelay(50);

	/*VDDIO*/
	if (system_rev >= 2 && engineerid >= 3) {/*VERSION A*/
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs2);
		/*This is swich power*/
		rc = camera_sensor_power_disable(shooter_reg_8901_lvs3);
		pr_info("[CAM] sensor_power_enable(\"8901_lvs3\", 1800) == %d\n", rc);
	} else {
		/*This is swich power*/
		rc = camera_sensor_power_disable(shooter_reg_8058_l9);
		pr_info("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);
		rc = camera_sensor_power_disable(shooter_reg_8058_l8);
		pr_info("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);
	}

	/* according to logic Jason Kao, do not turn off l15 to avoid current leakage */
	if (!(engineerid == 7)) {
		rc = camera_sensor_power_disable(shooter_reg_8058_l15);
		udelay(50);
	}
	return rc;
}
#endif
#ifdef CONFIG_QS_S5K4E1
static char eeprom_data[864];
static struct msm_camera_sensor_flash_data flash_qs_s5k4e1 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_qs_s5k4e1_data = {
	.sensor_name	= "qs_s5k4e1",
	.sensor_reset	= SHOOTER_S5K4E1_PD,
	.vcm_enable		= 0,
	.camera_power_on = Shooter_qs_s5k4e1_vreg_on,
	.camera_power_off = Shooter_qs_s5k4e1_vreg_off,
	.camera_clk_switch = Shooter_maincam_clk_switch,
	.pdata			= &msm_camera_device_data,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_qs_s5k4e1,
	.flash_cfg = &msm_camera_sensor_flash_cfg,
	.stereo_low_cap_limit = 15,
	.csi_if			= 1,
	.dev_node		= 0,
	.eeprom_data		= eeprom_data,
};
struct platform_device msm_camera_sensor_qs_s5k4e1 = {
	.name	= "msm_camera_qs_s5k4e1",
	.dev	= {
		.platform_data = &msm_camera_sensor_qs_s5k4e1_data,
	},
};
#endif
#ifdef CONFIG_MT9V113
static int shooter_mt9v113_vreg_on(void)
{
	int rc = 0;

	pr_info("[CAM] %s\n", __func__);

	rc = camera_sensor_power_enable("8058_l10", 2850000, &shooter_reg_8058_l10);
	pr_info("[CAM] sensor_power_enable(\"8058_l10\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l10\", 2.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	rc = camera_sensor_power_enable("8058_l12", 1800000, &shooter_reg_8058_l12);
	pr_info("[CAM] sensor_power_enable(\"8058_l12\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l12\", 1.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	rc = camera_sensor_power_enable("8058_l15", 2800000, &shooter_reg_8058_l15);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	rc = camera_sensor_power_enable("8058_l9", 1800000, &shooter_reg_8058_l9);
	pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) FAILED %d\n", rc);
		goto init_fail;
	}

	shooter_config_camera_on_gpios();

	rc = gpio_request(SHOOTER_MCLK_SWITCH, "CAM_SEL");
	if (rc < 0)
	{
		pr_err("[CAM] GPIO (%d) request fail\n", SHOOTER_MCLK_SWITCH);
		goto init_fail;
	}
	gpio_direction_output(SHOOTER_MCLK_SWITCH, 1);
	gpio_free(SHOOTER_MCLK_SWITCH);

init_fail:
	return rc;
}

static int shooter_mt9v113_vreg_off(void)
{
	int rc = 0;

	pr_info("[CAM] %s\n", __func__);

	rc = camera_sensor_power_disable(shooter_reg_8058_l9);
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l9\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l9\") FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);
	
	rc = camera_sensor_power_disable(shooter_reg_8058_l15);
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l15\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l15\") FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);
	
	rc = camera_sensor_power_disable(shooter_reg_8058_l12);
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l12\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l12\") FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);
	
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l10\", 2.8V) == %d\n", rc);
	rc = camera_sensor_power_disable(shooter_reg_8058_l10);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l10\") FAILED %d\n", rc);
		goto init_fail;
	}

	shooter_config_camera_off_gpios();

	pr_info("[CAM] Doing clk switch to sleep state\n");
	rc = gpio_request(SHOOTER_MCLK_SWITCH, "CAM_SEL");
	if (rc < 0)
	{
		pr_err("[CAM] GPIO (%d) request fail\n", SHOOTER_MCLK_SWITCH);
		goto init_fail;
	}
	gpio_direction_output(SHOOTER_MCLK_SWITCH, 0);
	gpio_free(SHOOTER_MCLK_SWITCH);

init_fail:
		return rc;
}

static struct msm_camera_sensor_platform_info sensor_mt9v113_board_info = {
	.mount_angle = 270,
	.mirror_flip = CAMERA_SENSOR_NONE,
	.sensor_reset_enable = 1,
	.sensor_reset = SHOOTER_CAM2_RSTz,
};

static struct msm_camera_sensor_flash_data flash_mt9v113 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v113_data = {
	.sensor_name	= "mt9v113",
	.sensor_reset	= SHOOTER_CAM2_RSTz,
	.camera_power_on = shooter_mt9v113_vreg_on,
	.camera_power_off = shooter_mt9v113_vreg_off,
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_mt9v113,
	.sensor_platform_info = &sensor_mt9v113_board_info,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.use_rawchip = 0, 
};

struct platform_device shooter_camera_sensor_mt9v113 = {
	.name	= "msm_camera_mt9v113",
	.dev	= {
		.platform_data = &msm_camera_sensor_mt9v113_data,
	},
};
#endif  

#ifdef CONFIG_I2C
static struct i2c_board_info msm_camera_boardinfo[] = {
#ifdef CONFIG_MT9V113
	{
	I2C_BOARD_INFO("mt9v113", 0x3C),
	.platform_data = &msm_camera_sensor_mt9v113_data,
	},
#endif
};
#endif  

void __init msm8x60_init_cam(void)
{
	int i = 0;

	struct platform_device *cam_dev[] = {
#ifdef CONFIG_QS_S5K4E1
	    &msm_camera_sensor_qs_s5k4e1,
#endif
#ifdef CONFIG_MT9V113
		&shooter_camera_sensor_mt9v113,
#endif
	};

	pr_err("%s %d", __func__, ARRAY_SIZE(cam_dev));

	i2c_register_board_info(MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo));

	/*
	platform_device_register(&msm_camera_server);
	*/

	for (i = 0; i < ARRAY_SIZE(cam_dev); i++) {
		platform_device_register(cam_dev[i]);
	}

	platform_device_register(&msm_device_csic0);
	platform_device_register(&msm_device_csic1);
	platform_device_register(&msm_device_vfe);
	platform_device_register(&msm_device_vpe);
}
#endif	
