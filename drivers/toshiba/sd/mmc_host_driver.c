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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#include "tsb_model.h"

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

static int mmc_host_driver_unregister(struct platform_driver *skel_driver)
{
        platform_driver_unregister(skel_driver);
}

static int mmc_host_tsb_model_get_model_sd(void)
{
    return tsb_model_get_model_sd();
}


EXPORT_SYMBOL(mmc_host_sysfs_create_group);
EXPORT_SYMBOL(mmc_host_sysfs_remove_group);
EXPORT_SYMBOL(mmc_host_driver_register);
EXPORT_SYMBOL(mmc_host_driver_unregister);
EXPORT_SYMBOL(mmc_host_tsb_model_get_model_sd);

module_init(mmc_host_init);
module_exit(mmc_host_exit);

MODULE_DESCRIPTION("mmc host driver kernekl interface");
MODULE_LICENSE("GPL");

