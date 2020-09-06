/*
  mmc host Driver

  Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/
#define THIS_FILE   "mmc_host_driver.c"
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include "dev_model.h"

#define XYZ(fmt, args...)  printk(KERN_INFO THIS_FILE "@%d " fmt "\n", __LINE__, args)

#define SD_CFG_POWER_ON   1
#define SD_CFG_POWER_OFF  2
#define SD_RST_ON         3
#define SD_RST_OFF        4
#define SD_VDD_ON         5
#define SD_VDD_OFF        6
#define SD_CLK_ON         7
#define SD_CLK_OFF        8

static const struct msm_gpio gpio_sdcard_poweron[11] = {
    { GPIO_CFG( 56, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_CLK" },
    { GPIO_CFG( 55, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_CMD" },
    { GPIO_CFG( 54, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D0" },
    { GPIO_CFG( 53, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D1" },
    { GPIO_CFG( 52, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D2" },
    { GPIO_CFG( 51, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D3" },
    { GPIO_CFG(154, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "SD_RST" },
    { GPIO_CFG(105, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), "SD_CLK" },
    { GPIO_CFG( 31, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "SDIF_VDD_ON" },
    { GPIO_CFG(149, 0, GPIO_INPUT , GPIO_PULL_UP, GPIO_2MA), "SD_INT" },
    { GPIO_CFG(104, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "SD_CLK_EN" },
};

/* SD card power off */
static const struct msm_gpio gpio_sdcard_poweroff[11] = {
    { GPIO_CFG( 56, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_CLK" },
    { GPIO_CFG( 55, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_CMD" },
    { GPIO_CFG( 54, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D0" },
    { GPIO_CFG( 53, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D1" },
    { GPIO_CFG( 52, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D2" },
    { GPIO_CFG( 51, 0, GPIO_INPUT , GPIO_NO_PULL, GPIO_6MA), "SDC1_D3" },
    { GPIO_CFG(154, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "SD_RST" },
    { GPIO_CFG(105, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), "SD_CLK" },
    { GPIO_CFG( 31, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "SDIF_VDD_ON" },
    { GPIO_CFG(149, 0, GPIO_INPUT , GPIO_PULL_UP, GPIO_2MA), "SD_INT" },
    { GPIO_CFG(104, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "SD_CLK_EN" },
};


static int mmc_host_sysfs_create_group(struct kobject *kobj, const struct attribute_group *grp)
{
    return sysfs_create_group(kobj, grp);
}

static void mmc_host_sysfs_remove_group(struct kobject *kobj, const struct attribute_group *grp)
{
    sysfs_remove_group(kobj, grp);
}

static int __init mmc_host_init(void)
{
    return 0;
}

static void __exit mmc_host_exit(void)
{
    ;
}

static int mmc_host_driver_register(struct platform_driver *skel_driver)
{
    return platform_driver_register(skel_driver);
}

static void mmc_host_driver_unregister(struct platform_driver *skel_driver)
{
    platform_driver_unregister(skel_driver);
}

static int mmc_host_dev_model_get_model_sd(void)
{
    return dev_model_get_model_sd();
}

static int mmc_host_io_ctrl(int cmd)
{
    switch (cmd){
    case SD_CFG_POWER_ON:
        if (msm_gpios_enable(gpio_sdcard_poweron, ARRAY_SIZE(gpio_sdcard_poweron)) >= 0){
            gpio_set_value(154, 0); /* SD_RST(GPIO[154]) -> 'L' */
            gpio_set_value(31, 0);  /* SD_RST(GPIO[31])  -> 'L' */
            gpio_set_value(104, 0); /* SD_RST(GPIO[104]) -> 'L' */
        }else{
            return -1;
        }
        break;

    case SD_CFG_POWER_OFF:
        if (msm_gpios_enable(gpio_sdcard_poweroff, ARRAY_SIZE(gpio_sdcard_poweroff)) >= 0){
            gpio_set_value(154, 0); /* SD_RST(GPIO[154]) -> 'L' */
            gpio_set_value(105, 0); /* SD_RST(GPIO[105]) -> 'L' */
        }else{
            return -1;
        }
        break;

    case SD_RST_ON:
        XYZ("SD_RST is '%c' +", gpio_get_value(154) ? 'H' : 'L');
        gpio_set_value(154, 1);
        XYZ("SD_RST is '%c' +", gpio_get_value(154) ? 'H' : 'L');
        break;
#if 0
    case SD_RST_OFF:
        XYZ("SD_RST is '%c' +", gpio_get_value(154) ? 'H' : 'L');
        gpio_set_value(154, 0);
        XYZ("SD_RST is '%c' +", gpio_get_value(154) ? 'H' : 'L');
        break;
#endif

    case SD_VDD_ON:
        XYZ("SD_VDD is '%c' +", gpio_get_value(31) ? 'H' : 'L');
        gpio_set_value(31, 1);
        XYZ("SD_VDD is '%c' +", gpio_get_value(31) ? 'H' : 'L');
        break;

    case SD_VDD_OFF:
        XYZ("SD_VDD is '%c' +", gpio_get_value(31) ? 'H' : 'L');
        gpio_set_value(31, 0);
        XYZ("SD_VDD is '%c' +", gpio_get_value(31) ? 'H' : 'L');
        break;

    case SD_CLK_ON:
        XYZ("SD_CLK is '%c' +", gpio_get_value(104) ? 'H' : 'L');
        gpio_set_value(104, 1);
        XYZ("SD_CLK is '%c' +", gpio_get_value(104) ? 'H' : 'L');
        break;

    case SD_CLK_OFF:
        XYZ("SD_CLK is '%c' +", gpio_get_value(104) ? 'H' : 'L');
        gpio_set_value(104, 0);
        XYZ("SD_CLK is '%c' +", gpio_get_value(104) ? 'H' : 'L');
        break;

    default:
        return -1;
    }
    return 0;
}

EXPORT_SYMBOL(mmc_host_sysfs_create_group);
EXPORT_SYMBOL(mmc_host_sysfs_remove_group);
EXPORT_SYMBOL(mmc_host_driver_register);
EXPORT_SYMBOL(mmc_host_driver_unregister);
EXPORT_SYMBOL(mmc_host_dev_model_get_model_sd);

//EXPORT_SYMBOL(mmc_host_dev_set_value);
EXPORT_SYMBOL(mmc_host_io_ctrl);

module_init(mmc_host_init);
module_exit(mmc_host_exit);

MODULE_DESCRIPTION("mmc host driver interface");
MODULE_LICENSE("GPL");

