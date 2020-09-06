/*
  keypad Driver

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


#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <asm/mach-types.h>

static struct gpio_event_direct_entry keypad_nav_map[] = {
    { 38,   KEY_ZOOM    },
    { 39,   KEY_CAMERA  },
        { 40,   KEY_VOLUMEDOWN },
        { 41,   KEY_VOLUMEUP   },
};

static int keypad_gpio_event_nav_func(    struct input_dev *input_devs,
                      struct gpio_event_info *info,
                      void **data, int func);

static struct gpio_event_input_info keypad_nav_info = {
        .info.func = keypad_gpio_event_nav_func,
        .flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE,
        .type = EV_KEY,
        .keymap = keypad_nav_map,
        .debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(keypad_nav_map)
};

static struct gpio_event_info *keypad_info[] = {
    &keypad_nav_info.info
};

static struct gpio_event_platform_data keypad_data = {
    .name       = "keypad",
    .info       = keypad_info,
    .info_count = ARRAY_SIZE(keypad_info)
};

struct platform_device keypad_device = {
    .name   = GPIO_EVENT_DEV_NAME,
    .id = -1,
    .dev    = {
        .platform_data  = &keypad_data,
    },
};

static struct input_dev *keypad_dev;


static int keypad_gpio_event_nav_func(    struct input_dev *input_dev,
                      struct gpio_event_info *info,
                      void **data, int func)
{
    int err;
    int i;

    err = gpio_event_input_func(input_dev, info, data, func);

    if (func == GPIO_EVENT_FUNC_INIT && !err) {
        keypad_dev = input_dev;
    } else if (func == GPIO_EVENT_FUNC_UNINIT) {
        keypad_dev = NULL;
    }

    return err;
}

struct input_dev *msm_keypad_get_input_dev(void)
{
    return keypad_dev;
}
MODULE_LICENSE("GPL");
