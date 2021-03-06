/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include "board-operaul.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4334";

static uint32_t operaul_bt_on_table[] = {

	
	GPIO_CFG(MSM_BT_RTSz,
				2,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_CTSz,
				2,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_RX,
				2,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_TX,
				2,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),

	
	GPIO_CFG(MSM_BT_HOST_WAKE,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_WAKE,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),

	
	GPIO_CFG(MSM_BT_SHUTDOWNz,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
};

static uint32_t operaul_bt_off_table[] = {

	
	GPIO_CFG(MSM_BT_RTSz,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_CTSz,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_RX,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_TX,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),

	
	GPIO_CFG(MSM_BT_SHUTDOWNz,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),

	
	GPIO_CFG(MSM_BT_HOST_WAKE,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA),
	
	GPIO_CFG(MSM_BT_WAKE,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[BT]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void operaul_config_bt_on(void)
{
	printk(KERN_INFO "[BT]== R ON ==\n");

	config_bt_table(operaul_bt_on_table,
				ARRAY_SIZE(operaul_bt_on_table));
	mdelay(2);

	gpio_set_value(MSM_BT_SHUTDOWNz, 0);
	mdelay(5);

	gpio_set_value(MSM_BT_SHUTDOWNz, 1);
	mdelay(50);
}

static void operaul_config_bt_off(void)
{
	gpio_set_value(MSM_BT_SHUTDOWNz, 0);
	mdelay(2);

	
	config_bt_table(operaul_bt_off_table,
				ARRAY_SIZE(operaul_bt_off_table));
	mdelay(2);

	gpio_set_value(MSM_BT_TX, 0);
	gpio_set_value(MSM_BT_RTSz, 0);

	gpio_set_value(MSM_BT_SHUTDOWNz, 0);

	gpio_set_value(MSM_BT_WAKE, 1);

	printk(KERN_INFO "[BT]== R OFF ==\n");
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		operaul_config_bt_on();
	else
		operaul_config_bt_off();

	return 0;
}

static struct rfkill_ops operaul_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int operaul_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  

	
	
	

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&operaul_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	return rc;
}

static int operaul_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	return 0;
}

static struct platform_driver operaul_rfkill_driver = {
	.probe = operaul_rfkill_probe,
	.remove = operaul_rfkill_remove,
	.driver = {
		.name = "operaul_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init operaul_rfkill_init(void)
{
	return platform_driver_register(&operaul_rfkill_driver);
}

static void __exit operaul_rfkill_exit(void)
{
	platform_driver_unregister(&operaul_rfkill_driver);
}

module_init(operaul_rfkill_init);
module_exit(operaul_rfkill_exit);
MODULE_DESCRIPTION("operaul rfkill");
MODULE_AUTHOR("htc_ssdbt <htc_ssdbt@htc.com>");
MODULE_LICENSE("GPL");
