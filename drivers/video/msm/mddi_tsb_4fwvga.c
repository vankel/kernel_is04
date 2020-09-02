/*
  Display Driver

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


/*============================================================================
INCLUDE FILES FOR MODULE
============================================================================*/
#include <linux/clk.h>
#include <mach/clk.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"


#define I2C_SLAVE_BD6088              0x76

/*===========================================================================
	MDP DMA LUT (Gamma Correction)
============================================================================*/
#include "msm_fb_def.h"
#define	MDP_DMA_P_COLOR_CORRECT_CONFIG          (0x90070)
#define	MDP_DMA_P_CSC_LUT1                      (0x93800)
#define	MDP_DMA_P_CSC_LUT2                      (0x93C00)
#define	MDP_DMA_P_COLOR_CORRECT_CONFIG_VAL      (0x17)          /* LUT_POSITION:5	Post-LUT	*/
                                                                /* LUT_C2_EN:2:		enable		*/
                                                                /* LUT_C1_EN:1:		enable		*/
                                                                /* LUT_C0_EN:0:		enable		*/
static byte lut_r[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
	0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
	0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
	0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};
static byte lut_g[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
	0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
	0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
	0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};
static byte lut_b[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
	0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
	0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
	0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};
static void tsb_set_dma_lut(void);
/*== MDP DMA LUT (Gamma Correction) ==*/

typedef enum {
  LCD_STATE_INIT,
  LCD_STATE_OFF,
  LCD_STATE_WAIT_UPDATE,
  LCD_STATE_WAIT_DISPLAY_ON,
  LCD_STATE_ON
} tsb_mddi_lcd_state_e;

/*===========================================================================
    LOCAL FUNCTIONS PROTOTYPES
============================================================================*/
static void drv_display_delay_on(struct work_struct *ignored);
static void drv_display_panel_display_on(void);
static void drv_display_panel_display_off(void);
static void drv_display_panel_power_on(void);
static void drv_display_panel_initialize(void);
static void drv_display_panel_exit_sleep(void);
static void drv_display_panel_enter_sleep(void);
static void drv_display_panel_exit_deepstandby(void);
static void drv_display_panel_enter_deepstandby(void);
static void drv_display_func_on(void);
static void drv_display_func_off(void);

static DECLARE_WORK(display_on_wq, drv_display_delay_on);

static tsb_mddi_lcd_state_e tsb_mddi_lcd_state = LCD_STATE_INIT;
static int backlight_state = 0;
static void tsb_BD6088_backlight_on(void);
static int backlight_first_on = FALSE;

struct i2c_adapter *i2c_bkl;

/*===========================================================================
    GLOBAL FUNCTIONS PROTOTYPES
============================================================================*/
void tsb_mddi_lcd_firstupdate(void);
void tsb_mddi_lcd_firstupdate_jugde(void);



static void drv_display_delay_on(struct work_struct *ignored)
{
  if (tsb_mddi_lcd_state == LCD_STATE_WAIT_DISPLAY_ON)
  {
    drv_display_panel_display_on();

    printk("[LCD]%s power ON\n",__func__);

    tsb_mddi_lcd_state = LCD_STATE_ON;
    
    //wait 20ms
    mddi_wait(20);
    
    tsb_BD6088_backlight_on();
  }
  else
  {
    printk(KERN_ERR "%s: tsb_mddi_lcd_state NG.\n", __func__);
  }
}

void tsb_mddi_lcd_firstupdate(void)
{
  if (tsb_mddi_lcd_state == LCD_STATE_WAIT_UPDATE)
  {
     tsb_mddi_lcd_state = LCD_STATE_WAIT_DISPLAY_ON;
  }
}

void tsb_mddi_lcd_firstupdate_jugde(void)
{
  if(tsb_mddi_lcd_state == LCD_STATE_WAIT_DISPLAY_ON)
  {
     schedule_work(&display_on_wq);
  }
}


static void drv_display_panel_display_on(void)
{
  //command for setting display on - 1
  mddi_queue_register_write(0x2900, 0x0000, TRUE, 0);

  //wait 17ms
  mddi_wait(17);

  //command for setting display on - 2
  mddi_queue_register_write(0x2C80, 0x0022, TRUE, 0);
  mddi_queue_register_write(0x2080, 0x0040, TRUE, 0);
  mddi_queue_register_write(0x2B80, 0x00BA, TRUE, 0);
  mddi_queue_register_write(0x2280, 0x000C, TRUE, 0);

}

static void drv_display_panel_display_off(void)
{
  //command for setting display off
  mddi_queue_register_write(0x2800, 0x0000, TRUE, 0);

  //wait 68ms
  mddi_wait(68);
}
static void drv_display_panel_power_on(void)
{
  //reset low
  gpio_set_value(100, 0);

  //VCI(LCD28V)ON


  //reset high
  gpio_set_value(100, 1);

  //wait 200ms
  mddi_wait(200);
}


static void drv_display_panel_initialize(void)
{
    //command for initializing lcd panel
    mddi_queue_register_write(0x2A00, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2A01, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2A02, 0x0001, TRUE, 0);
    mddi_queue_register_write(0x2A03, 0x00DF, TRUE, 0);
    mddi_queue_register_write(0x2B00, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2B01, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2B02, 0x0003, TRUE, 0);
    mddi_queue_register_write(0x2B03, 0x0055, TRUE, 0);
    mddi_queue_register_write(0x2D00, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2D01, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2D02, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2D03, 0x0000, TRUE, 0);
}
static void drv_display_panel_exit_sleep(void)
{
    //command for exiting sleep mode - 1
    mddi_queue_register_write(0x1100, 0x0000, TRUE, 0);

    //wait 40->120ms
    mddi_wait(120);

    //command for exiting sleep mode - 2
//    mddi_queue_register_write(0x6E4F, 0x0005, TRUE, 0);
    mddi_queue_register_write(0xF280, 0x0055, TRUE, 0);
    mddi_queue_register_write(0xF281, 0x00AA, TRUE, 0);
    mddi_queue_register_write(0xF282, 0x0066, TRUE, 0);
    mddi_queue_register_write(0xF38E, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x0180, 0x0002, TRUE, 0);
    mddi_queue_register_write(0x0380, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x0480, 0x0001, TRUE, 0);
    mddi_queue_register_write(0x0580, 0x002C, TRUE, 0);
    mddi_queue_register_write(0x0680, 0x0023, TRUE, 0);
    mddi_queue_register_write(0x2080, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2280, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2480, 0x0050, TRUE, 0);
    mddi_queue_register_write(0x2580, 0x006B, TRUE, 0);
    mddi_queue_register_write(0x2780, 0x0064, TRUE, 0);
    mddi_queue_register_write(0x2A80, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2B80, 0x00B1, TRUE, 0);
    mddi_queue_register_write(0x2C80, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x2D80, 0x0000, TRUE, 0);
    mddi_queue_register_write(0xD080, 0x0008, TRUE, 0);
    mddi_queue_register_write(0xD180, 0x0016, TRUE, 0);
    mddi_queue_register_write(0xD280, 0x0005, TRUE, 0);
    mddi_queue_register_write(0xD380, 0x0000, TRUE, 0);
    mddi_queue_register_write(0xD480, 0x0062, TRUE, 0);
    mddi_queue_register_write(0xD580, 0x0001, TRUE, 0);
    mddi_queue_register_write(0xD680, 0x005B, TRUE, 0);
    mddi_queue_register_write(0xD780, 0x0001, TRUE, 0);
    mddi_queue_register_write(0xD880, 0x00DE, TRUE, 0);
    mddi_queue_register_write(0xD980, 0x000E, TRUE, 0);
//    mddi_queue_register_write(0xDB80, 0x0000, TRUE, 0);

    //wait 64ms
    mddi_wait(64);

    //command for exiting sleep mode - 3
    mddi_queue_register_write(0x4080, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x4180, 0x0027, TRUE, 0);
    mddi_queue_register_write(0x4280, 0x0042, TRUE, 0);
    mddi_queue_register_write(0x4380, 0x0070, TRUE, 0);
    mddi_queue_register_write(0x4480, 0x0010, TRUE, 0);
    mddi_queue_register_write(0x4580, 0x0037, TRUE, 0);
    mddi_queue_register_write(0x4680, 0x0062, TRUE, 0);
    mddi_queue_register_write(0x4780, 0x008C, TRUE, 0);
    mddi_queue_register_write(0x4880, 0x001E, TRUE, 0);
    mddi_queue_register_write(0x4980, 0x0025, TRUE, 0);
    mddi_queue_register_write(0x4A80, 0x00D4, TRUE, 0);
    mddi_queue_register_write(0x4B80, 0x001E, TRUE, 0);
    mddi_queue_register_write(0x4C80, 0x0041, TRUE, 0);
    mddi_queue_register_write(0x4D80, 0x006C, TRUE, 0);
    mddi_queue_register_write(0x4E80, 0x00BC, TRUE, 0);
    mddi_queue_register_write(0x4F80, 0x00DB, TRUE, 0);
    mddi_queue_register_write(0x5080, 0x0077, TRUE, 0);
    mddi_queue_register_write(0x5180, 0x0073, TRUE, 0);
    mddi_queue_register_write(0x5880, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x5980, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x5A80, 0x0038, TRUE, 0);
    mddi_queue_register_write(0x5B80, 0x0057, TRUE, 0);
    mddi_queue_register_write(0x5C80, 0x0014, TRUE, 0);
    mddi_queue_register_write(0x5D80, 0x003E, TRUE, 0);
    mddi_queue_register_write(0x5E80, 0x0061, TRUE, 0);
    mddi_queue_register_write(0x5F80, 0x0040, TRUE, 0);
    mddi_queue_register_write(0x6080, 0x001A, TRUE, 0);
    mddi_queue_register_write(0x6180, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x6280, 0x0089, TRUE, 0);
    mddi_queue_register_write(0x6380, 0x001D, TRUE, 0);
    mddi_queue_register_write(0x6480, 0x0048, TRUE, 0);
    mddi_queue_register_write(0x6580, 0x006A, TRUE, 0);
    mddi_queue_register_write(0x6680, 0x00A6, TRUE, 0);
    mddi_queue_register_write(0x6780, 0x00D9, TRUE, 0);
    mddi_queue_register_write(0x6880, 0x0078, TRUE, 0);
    mddi_queue_register_write(0x6980, 0x007F, TRUE, 0);
    mddi_queue_register_write(0x7080, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x7180, 0x003A, TRUE, 0);
    mddi_queue_register_write(0x7280, 0x0055, TRUE, 0);
    mddi_queue_register_write(0x7380, 0x007C, TRUE, 0);
    mddi_queue_register_write(0x7480, 0x001A, TRUE, 0);
    mddi_queue_register_write(0x7580, 0x0041, TRUE, 0);
    mddi_queue_register_write(0x7680, 0x0063, TRUE, 0);
    mddi_queue_register_write(0x7780, 0x009B, TRUE, 0);
    mddi_queue_register_write(0x7880, 0x0019, TRUE, 0);
    mddi_queue_register_write(0x7980, 0x0024, TRUE, 0);
    mddi_queue_register_write(0x7A80, 0x00DB, TRUE, 0);
    mddi_queue_register_write(0x7B80, 0x001C, TRUE, 0);
    mddi_queue_register_write(0x7C80, 0x003D, TRUE, 0);
    mddi_queue_register_write(0x7D80, 0x006A, TRUE, 0);
    mddi_queue_register_write(0x7E80, 0x00BB, TRUE, 0);
    mddi_queue_register_write(0x7F80, 0x00DB, TRUE, 0);
    mddi_queue_register_write(0x8080, 0x0077, TRUE, 0);
    mddi_queue_register_write(0x8180, 0x0073, TRUE, 0);
    mddi_queue_register_write(0x8880, 0x0020, TRUE, 0);
    mddi_queue_register_write(0x8980, 0x0022, TRUE, 0);
    mddi_queue_register_write(0x8A80, 0x0038, TRUE, 0);
    mddi_queue_register_write(0x8B80, 0x0058, TRUE, 0);
    mddi_queue_register_write(0x8C80, 0x0016, TRUE, 0);
    mddi_queue_register_write(0x8D80, 0x0041, TRUE, 0);
    mddi_queue_register_write(0x8E80, 0x0061, TRUE, 0);
    mddi_queue_register_write(0x8F80, 0x003A, TRUE, 0);
    mddi_queue_register_write(0x9080, 0x0019, TRUE, 0);
    mddi_queue_register_write(0x9180, 0x0022, TRUE, 0);
    mddi_queue_register_write(0x9280, 0x007B, TRUE, 0);
    mddi_queue_register_write(0x9380, 0x001B, TRUE, 0);
    mddi_queue_register_write(0x9480, 0x003E, TRUE, 0);
    mddi_queue_register_write(0x9580, 0x0063, TRUE, 0);
    mddi_queue_register_write(0x9680, 0x009A, TRUE, 0);
    mddi_queue_register_write(0x9780, 0x00CA, TRUE, 0);
    mddi_queue_register_write(0x9880, 0x0064, TRUE, 0);
    mddi_queue_register_write(0x9980, 0x007F, TRUE, 0);
    mddi_queue_register_write(0xA080, 0x0020, TRUE, 0);
    mddi_queue_register_write(0xA180, 0x003A, TRUE, 0);
    mddi_queue_register_write(0xA280, 0x0060, TRUE, 0);
    mddi_queue_register_write(0xA380, 0x008C, TRUE, 0);
    mddi_queue_register_write(0xA480, 0x0018, TRUE, 0);
    mddi_queue_register_write(0xA580, 0x0045, TRUE, 0);
    mddi_queue_register_write(0xA680, 0x0066, TRUE, 0);
    mddi_queue_register_write(0xA780, 0x00B1, TRUE, 0);
    mddi_queue_register_write(0xA880, 0x001B, TRUE, 0);
    mddi_queue_register_write(0xA980, 0x0026, TRUE, 0);
    mddi_queue_register_write(0xAA80, 0x00E5, TRUE, 0);
    mddi_queue_register_write(0xAB80, 0x001C, TRUE, 0);
    mddi_queue_register_write(0xAC80, 0x0047, TRUE, 0);
    mddi_queue_register_write(0xAD80, 0x0069, TRUE, 0);
    mddi_queue_register_write(0xAE80, 0x00BC, TRUE, 0);
    mddi_queue_register_write(0xAF80, 0x00DB, TRUE, 0);
    mddi_queue_register_write(0xB080, 0x0077, TRUE, 0);
    mddi_queue_register_write(0xB180, 0x0073, TRUE, 0);
    mddi_queue_register_write(0xB880, 0x0020, TRUE, 0);
    mddi_queue_register_write(0xB980, 0x0021, TRUE, 0);
    mddi_queue_register_write(0xBA80, 0x0038, TRUE, 0);
    mddi_queue_register_write(0xBB80, 0x0057, TRUE, 0);
    mddi_queue_register_write(0xBC80, 0x0017, TRUE, 0);
    mddi_queue_register_write(0xBD80, 0x003B, TRUE, 0);
    mddi_queue_register_write(0xBE80, 0x0063, TRUE, 0);
    mddi_queue_register_write(0xBF80, 0x002F, TRUE, 0);
    mddi_queue_register_write(0xC080, 0x001B, TRUE, 0);
    mddi_queue_register_write(0xC180, 0x0026, TRUE, 0);
    mddi_queue_register_write(0xC280, 0x0063, TRUE, 0);
    mddi_queue_register_write(0xC380, 0x001A, TRUE, 0);
    mddi_queue_register_write(0xC480, 0x0036, TRUE, 0);
    mddi_queue_register_write(0xC580, 0x0067, TRUE, 0);
    mddi_queue_register_write(0xC680, 0x0087, TRUE, 0);
    mddi_queue_register_write(0xC780, 0x00B4, TRUE, 0);
    mddi_queue_register_write(0xC880, 0x0064, TRUE, 0);
    mddi_queue_register_write(0xC980, 0x007F, TRUE, 0);
    mddi_queue_register_write(0x3500, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x4400, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x4401, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x3600, 0x0002, TRUE, 0);
    mddi_queue_register_write(0xDA80, 0x0040, TRUE, 0);
    mddi_queue_register_write(0x2100, 0x0000, TRUE, 0);
}
static void drv_display_panel_enter_sleep(void)
{
  //command for setting sleep mode
  mddi_queue_register_write(0xDA80, 0x0000, TRUE, 0);
  mddi_queue_register_write(0x1000, 0x0000, TRUE, 0);

  //wait 68ms
  mddi_wait(68);
}

static void drv_display_panel_enter_deepstandby(void)
{
    mddi_queue_register_write(0xDA80, 0x0000, TRUE, 0);
    mddi_queue_register_write(0x4F00, 0x0001, TRUE, 0);

//****************************************************//
//  Work around  add Wait 200ms
    mddi_wait(200);
//****************************************************//
}
static void drv_display_panel_exit_deepstandby(void)
{
    //gpio init
    gpio_tlmm_config(GPIO_CFG(100, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);

    // Reset 3times
    gpio_set_value(100, 0);
    mddi_wait(5);
    gpio_set_value(100, 1);
    
    //wait 3ms
    mddi_wait(3);
    
    gpio_set_value(100, 0);
    mddi_wait(5);
    gpio_set_value(100, 1);
    
    //wait 3ms
    mddi_wait(3);
    
    gpio_set_value(100, 0);
    mddi_wait(5);
    gpio_set_value(100, 1);
    
    //wait 30ms
    mddi_wait(30);
}

static void drv_display_func_on(void)
{
  printk("[LCD]%s ENTER state[%d]\n",__func__,tsb_mddi_lcd_state);
  
  switch (tsb_mddi_lcd_state)
  {
  case LCD_STATE_INIT:
    //panel power already on at boot
    tsb_mddi_lcd_state = LCD_STATE_ON;
    break;
    
  case LCD_STATE_OFF:
#if 0
    // exit deepstandby
    drv_display_panel_exit_deepstandby();

    //driver initialize
    drv_display_panel_initialize();
#else
    printk("[LCD]%s Exit Sleep\n",__func__);
#endif
    //exit sleep mode
    drv_display_panel_exit_sleep();
    
    tsb_mddi_lcd_state = LCD_STATE_WAIT_UPDATE;
    break;
  case LCD_STATE_WAIT_DISPLAY_ON:
  case LCD_STATE_WAIT_UPDATE:
  case LCD_STATE_ON:
    break;
  default:
    break;
  }
}


static void drv_display_func_off(void)
{
  printk("[LCD]%s ENTER state[%d]\n",__func__,tsb_mddi_lcd_state);

  switch (tsb_mddi_lcd_state)
  {
  case LCD_STATE_ON:
    //ponel display off
    drv_display_panel_display_off();

    //ponel enter sleep mode
    drv_display_panel_enter_sleep();
    
#if 0 // No Use DeepStandby Mode
    //enter deepstandby
    drv_display_panel_enter_deepstandby();
    
    printk("[LCD]%s power OFF\n",__func__);
#else
    printk("[LCD]%s PanelSleep\n",__func__);
#endif
    
    tsb_mddi_lcd_state = LCD_STATE_OFF;
    break;
  case LCD_STATE_WAIT_UPDATE:
    //ponel enter sleep mode
    drv_display_panel_enter_sleep();
    
#if 0 // No Use DeepStandby Mode
    //enter deepstandby
    drv_display_panel_enter_deepstandby();
    
    printk("[LCD]%s power OFF\n",__func__);
#else
    printk("[LCD]%s PanelSleep\n",__func__);
#endif
    
    tsb_mddi_lcd_state = LCD_STATE_OFF;
    break;
  case LCD_STATE_WAIT_DISPLAY_ON:
    tsb_mddi_lcd_state = LCD_STATE_ON;
    
    //ponel enter sleep mode
    drv_display_panel_enter_sleep();
    
#if 0 // No Use DeepStandby Mode
    //enter deepstandby
    drv_display_panel_enter_deepstandby();
    
    printk("[LCD]%s power OFF\n",__func__);
#else
    printk("[LCD]%s PanelSleep\n",__func__);
#endif
    
    tsb_mddi_lcd_state = LCD_STATE_OFF;
    break;
  case LCD_STATE_INIT:
  case LCD_STATE_OFF:
    break;
  default:
    break;
  }
}



static int tsb_mddi_lcd_on(struct platform_device *pdev)
{
  drv_display_func_on();
  
  return 0;
}

static int tsb_mddi_lcd_off(struct platform_device *pdev)
{
  drv_display_func_off();

  return 0;
}

static int __init tsb_mddi_probe(struct platform_device *pdev)
{
  msm_fb_add_device(pdev);

  return 0;
}

static void tsb_BD6088_set_backlight(struct msm_fb_data_type *mfd)
{
  struct i2c_msg msg;
  u_int8_t buf[8];
  int ret;
  int32 level;

  level = mfd->bl_level;

  msg.addr = I2C_SLAVE_BD6088;
  msg.buf = buf;
  msg.len = 2;
  msg.flags = 0;

  if( level )
  {
    if(backlight_state) {

      if(backlight_first_on){
        /* slope */
        buf[0] = 0x06;  //client addr
        buf[1] = 0x44;//data
        ret = i2c_transfer(i2c_bkl, &msg, 1);
        if (ret < 0)
        {
            printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
            return;
        }
        backlight_first_on = FALSE;
      }

      /* Intensity */
      buf[0] = 0x03;  //client addr
      buf[1] = level;//data
      ret = i2c_transfer(i2c_bkl, &msg, 1);
      if (ret < 0)
      {
        printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
        return;
      }

    } else {

      buf[0] = 0x00;  //client addr
      buf[1] = 0;//data
      ret = i2c_transfer(i2c_bkl, &msg, 1);
      if (ret < 0)
      {
        printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
        return;
      }

      buf[0] = 0x01;  //client addr
      buf[1] = 0x4E;//data
      ret = i2c_transfer(i2c_bkl, &msg, 1);
      if (ret < 0)
      {
        printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
        return;
      }

      /* Intensity */
      buf[0] = 0x03;  //client addr
      buf[1] = level;//data
      ret = i2c_transfer(i2c_bkl, &msg, 1);
      if (ret < 0)
      {
        printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
        return;
      }

      /* slope */
      buf[0] = 0x06;  //client addr
//      buf[1] = 0x44;//data
      buf[1] = 0x40;//data
      ret = i2c_transfer(i2c_bkl, &msg, 1);
      if (ret < 0)
      {
        printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
        return;
      }

      if (tsb_mddi_lcd_state == LCD_STATE_ON) {
        /* ON */
        buf[0] = 0x02;  //client addr
        buf[1] = 0x09;//data
        ret = i2c_transfer(i2c_bkl, &msg, 1);
        if (ret < 0)
        {
          printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
          return;
        }
        backlight_first_on = TRUE;
      }
      else {
        printk("[BKL]Request Arrive. TurnON After LCD_PowerON.\n");
        
        /* ALC ON */
        buf[0] = 0x02;  //client addr
        buf[1] = 0x08;//data
        ret = i2c_transfer(i2c_bkl, &msg, 1);
        if (ret < 0)
        {
          printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
          return;
        }
      }

      backlight_state = 1;
      printk("[BKL]%s power ON\n",__func__);
    }
  }
  else
  {
    backlight_state = 0;

    /* OFF */
    buf[0] = 0x02;  //client addr
    buf[1] = 0;//data
    ret = i2c_transfer(i2c_bkl, &msg, 1);
    if (ret < 0)
    {
      printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
      return;
    }

    printk("[BKL]%s power OFF\n",__func__);
  }
}

static void tsb_BD6088_backlight_on(void)
{
    struct i2c_msg msg;
    u_int8_t buf[8];
    int ret;
    
    msg.addr = I2C_SLAVE_BD6088;
    msg.buf = buf;
    msg.len = 2;
    msg.flags = 0;
    
    if (backlight_state ==1) {
        printk("[BKL] Request Arrived. BKL TurnON\n");
        /* ON */
        buf[0] = 0x02;//client addr
        buf[1] = 0x09;//data
        ret = i2c_transfer(i2c_bkl, &msg, 1);
        if (ret < 0)
        {
            printk("BKL_DBG(%d):I2C ERROR ret = %d\n",__LINE__, ret);
            return;
        }
        backlight_first_on = TRUE;
    }
    else {
        printk("[BKL] Request Not Arrived (set_backlight).\n");
    }
}

static struct platform_driver this_driver = {
  .probe  = tsb_mddi_probe,
  .shutdown = NULL,
  .driver = {
    .name   = "mddi_tsb_4fwvga",
  },
};

static struct msm_fb_panel_data tsb_mddi_panel_data = {
  .on   = tsb_mddi_lcd_on,
  .off  = tsb_mddi_lcd_off,
  .set_backlight  = tsb_BD6088_set_backlight
};

static struct platform_device this_device = {
  .name   = "mddi_tsb_4fwvga",
  .id  = 0,
  .dev  = {
    .platform_data = &tsb_mddi_panel_data,
   }
};

static int __init tsb_mddi_init(void)
{
  int ret;
  struct msm_panel_info *pinfo;

  tsb_mddi_lcd_state = LCD_STATE_INIT;
  
  i2c_bkl = i2c_get_adapter(0);
  backlight_state = 0;

  ret = platform_driver_register(&this_driver);

  if (!ret) {
    pinfo = &tsb_mddi_panel_data.panel_info;
    pinfo->xres                       = 480;
    pinfo->yres                       = 854;
    pinfo->type                       = MDDI_PANEL;
    pinfo->pdest                      = DISPLAY_1;
    pinfo->mddi.vdopkt                = MDDI_DEFAULT_PRIM_PIX_ATTR;
    pinfo->wait_cycle                 = 0;
    pinfo->bpp                        = 18;
    pinfo->fb_num                     = 2;
    pinfo->clk_rate                   = 192000000;
    pinfo->clk_min                    = 190000000; //PMDH min
    pinfo->clk_max                    = 200000000; //PMDH max
    pinfo->lcd.vsync_enable           = TRUE;
    pinfo->lcd.refx100                = 60 * 100;
    pinfo->lcd.v_back_porch           = 12;
    pinfo->lcd.v_front_porch          = 2;
    pinfo->lcd.v_pulse_width          = 0;
    pinfo->lcd.hw_vsync_mode          = FALSE;
    pinfo->lcd.vsync_notifier_period  = 0;
    pinfo->bl_min                     = 1;
    pinfo->bl_max                     = 99;

    ret = platform_device_register(&this_device);
    if (ret){
      platform_driver_unregister(&this_driver);
    }
  }

    /* MDP DMA LUT setup call */
    tsb_set_dma_lut();

  return ret;
}

/* MDP DMA LUT setup start */
static void tsb_set_dma_lut(void)
{
    dword	*a;
    int		i;

    a = (dword *)( msm_mdp_base + MDP_DMA_P_CSC_LUT1 );
    for ( i = 0; i < 0x100; ++i ) {
        outpdw( a++, (dword)( lut_r[ i ] << 16 | lut_b[ i ] << 8 | lut_g[ i ] ) );
    }
    outpdw( msm_mdp_base + MDP_DMA_P_COLOR_CORRECT_CONFIG, MDP_DMA_P_COLOR_CORRECT_CONFIG_VAL );
}
/* MDP DMA LUT setup end */

module_init(tsb_mddi_init);
