/* linux/arch/arm/mach-msm/board-spade.h
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SHOOTER_H
#define __ARCH_ARM_MACH_MSM_BOARD_SHOOTER_H

#include <mach/board.h>
#include <mach/msm_memtypes.h>
#include <mach/rpm-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <mach/board-msm8660.h>

#define SHOOTER_PROJECT_NAME	"shooter"

#define MSM_RAM_CONSOLE_BASE	MSM_HTC_RAM_CONSOLE_PHYS
#define MSM_RAM_CONSOLE_SIZE	MSM_HTC_RAM_CONSOLE_SIZE

/* Memory map */

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
#define QCE_SIZE		0x10000
#define QCE_0_BASE		0x18500000
#endif

#define PHY_BASE_ADDR1  0x48000000
#define SIZE_ADDR1      0x25600000

/* GPIO definition */

/* Direct Keys */
#define SHOOTER_GPIO_KEY_POWER          (125)

/* Battery */
#define SHOOTER_GPIO_MBAT_IN            (61)
#define SHOOTER_GPIO_CHG_INT		(126)

/* Wifi */
#define SHOOTER_GPIO_WIFI_IRQ              (46)
#define SHOOTER_GPIO_WIFI_SHUTDOWN_N       (96)
/* Sensors */
#define SHOOTER_SENSOR_I2C_SDA		(72)
#define SHOOTER_SENSOR_I2C_SCL		(73)
#define SHOOTER_GYRO_INT               (127)
#define SHOOTER_ECOMPASS_INT           (128)
#define SHOOTER_GSENSOR_INT           (129)

/* Microp */

/* TP */
#define SHOOTER_TP_I2C_SDA           (51)
#define SHOOTER_TP_I2C_SCL           (52)
#define SHOOTER_TP_ATT_N             (65)
#define SHOOTER_TP_ATT_N_XC          (57)

/* LCD */
#define GPIO_LCD_TE	28
#define GPIO_LCM_RST_N			(66)
#define GPIO_LCM_ID			(50)

/* Audio */
#define SHOOTER_AUD_CODEC_RST        (67)

/* BT */
#define SHOOTER_GPIO_BT_HOST_WAKE      (45)
#define SHOOTER_GPIO_BT_UART1_TX       (53)
#define SHOOTER_GPIO_BT_UART1_RX       (54)
#define SHOOTER_GPIO_BT_UART1_CTS      (55)
#define SHOOTER_GPIO_BT_UART1_RTS      (56)
#define SHOOTER_GPIO_BT_SHUTDOWN_N     (100)
#define SHOOTER_GPIO_BT_CHIP_WAKE      (130)
#define SHOOTER_GPIO_BT_RESET_N        (142)

/* USB */
#define SHOOTER_GPIO_USB_ID        (63)
#define SHOOTER_GPIO_MHL_RESET        (70)
#define SHOOTER_GPIO_MHL_INT        (71)
#define SHOOTER_GPIO_MHL_USB_SWITCH        (99)

/* Camera */
#define SHOOTER_CAM_CAM1_ID           (10)
#define SHOOTER_CAM_I2C_SDA           (47)
#define SHOOTER_CAM_I2C_SCL           (48)

/* General */
#define SHOOTER_GENERAL_I2C_SDA		(59)
#define SHOOTER_GENERAL_I2C_SCL		(60)

/* Flashlight */
#define SHOOTER_FLASH_EN             (29)
#define SHOOTER_FLASH_TORCH             (30)

/* Accessory */
#define SHOOTER_GPIO_AUD_HP_DET        (31)

/* SPI */
#define SHOOTER_SPI_DO                 (33)
#define SHOOTER_SPI_DI                 (34)
#define SHOOTER_SPI_CS                 (35)
#define SHOOTER_SPI_CLK                (36)

/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define SHOOTER_VOL_UP             PMGPIO(16)
#define SHOOTER_VOL_DN             PMGPIO(17)
#define SHOOTER_AUD_HP_EN          PMGPIO(18)
#define SHOOTER_HAP_ENABLE         PMGPIO(19)
#define SHOOTER_AUD_QTR_RESET      PMGPIO(21)
#define SHOOTER_TP_RST             PMGPIO(23)
#define SHOOTER_GREEN_LED          PMGPIO(24)
#define SHOOTER_AMBER_LED          PMGPIO(25)
#define SHOOTER_AUD_MIC_SEL        PMGPIO(26)
#define SHOOTER_CHG_STAT	   PMGPIO(33)
#define SHOOTER_SDC3_DET           PMGPIO(34)
#define SHOOTER_PLS_INT            PMGPIO(35)
#define SHOOTER_AUD_REMO_PRES      PMGPIO(37)
#define SHOOTER_WIFI_BT_SLEEP_CLK  PMGPIO(38)

extern struct regulator_init_data msm_saw_regulator_pdata_s0;
extern struct regulator_init_data msm_saw_regulator_pdata_s1;
extern struct rpm_regulator_platform_data shooter_rpm_regulator_pdata __devinitdata;
extern struct platform_device msm8x60_8901_mpp_vreg __devinitdata;
extern struct pm8901_vreg_pdata pm8901_regulator_pdata[];
extern int pm8901_regulator_pdata_len;
extern struct platform_device msm_adc_device;

void __init shooter_init_mmc(void);
int __init shooter_init_wifi_mmc(void);
void __init shooter_audio_init(void);
int __init shooter_init_keypad(void);
void __init shooter_init_fb(void);
void __init shooter_init_pmic(void);
int __init shooter_wifi_init(void);
void __init shooter_gpio_mpp_init(void);
void shooter_init_gpiomux(void);
void __init msm8x60_allocate_fb_region(void);
void __init shooter_pm8901_gpio_mpp_init(void);
void msm8x60_mdp_writeback(struct memtype_reserve *reserve_table);
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
int hdmi_enable_5v(int on);
#endif
#define _GET_REGULATOR(var, name) do {					\
	if (var == NULL) {						\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_err("'%s' regulator not found, rc=%ld\n",	\
				name, PTR_ERR(var));			\
			var = NULL;					\
		}							\
	}								\
} while (0)

#endif /* __ARCH_ARM_MACH_MSM_BOARD_SHOOTER_H */
