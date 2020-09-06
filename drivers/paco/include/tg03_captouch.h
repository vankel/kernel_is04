/*
Capacitive touch panel Driver

Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
*/


#ifndef CAT_ATMEL_DATA_H
#define CAT_ATMEL_DATA_H

#define DEBUG_DIAGNOSTIC_T37                    37
#define GEN_MESSAGEPROCESSOR_T5                  5
#define GEN_COMMANDPROCESSOR_T6                  6
#define GEN_POWERCONFIG_T7                       7
#define GEN_ACQUIRECONFIG_T8                     8

#define TOUCH_MULTITOUCHSCREEN_T9                9
#define TOUCH_KEYARRAY_T15                      15
#define TOUCH_PROXIMITY_T23                     23

#define PROCI_GRIPFACESUPPRESSION_T20           20
#define PROCG_NOISESUPPRESSION_T22              22
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24      24
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27      27

#define SPT_COMCONFIG_T18                       18
#define SPT_GPIOPWM_T19                         19
#define SPT_SELFTEST_T25                        25
#define SPT_CTECONFIG_T28                       28
#define SPT_USERDATA_T38                        38


enum tg03cap_device_id {
    MODEL_NO_1_SWC = 0,
    MODEL_NO_PR_WTK,
    MODEL_NO_PR_SWC,
    MODEL_NO_PR_NONE,
};


typedef struct {
    int x;
    int y;
    int area;
    int delta;
}point_data;


typedef struct {
    uint32_t config_crc;
    uint8_t config_t38[8];
    uint8_t config_t7[3];
    uint8_t config_t8[8];
    uint8_t config_t9[31];
    uint8_t config_t15[11];
    uint8_t config_t18[2];
    uint8_t config_t19[16];
    uint8_t config_t20[12];
    uint8_t config_t22[17];
    uint8_t config_t23[15];
    uint8_t config_t24[19];
    uint8_t config_t25[14];
    uint8_t config_t27[7];
    uint8_t config_t28[6];
    uint8_t resume_atchcalsthr;
} hw_config_data;


typedef struct {
    int x_max;
    int y_max;
    int pressure_max;
    int max_point;
    int gpio_int;
    int gpio_reset;
    int touch_addr;
    hw_config_data hw_config;
    void ( *vreg_config )( int on );
}tg03_captouch_platfrom_data ;

#define TG03CAP_IOCTL_BASE          0x55

#define IOCTL_GET_ID        _IOR( TG03CAP_IOCTL_BASE, 1, int )
#define IOCTL_GET_POS       _IOR( TG03CAP_IOCTL_BASE, 2, point_data[3] )
#define IOCTL_SET_CFG       _IOW( TG03CAP_IOCTL_BASE, 3, hw_config_data )
#define IOCTL_GET_CFG       _IOR( TG03CAP_IOCTL_BASE, 4, hw_config_data )
#define IOCTL_START_BACKUP  _IO ( TG03CAP_IOCTL_BASE, 5 )
#define IOCTL_GET_DELTA     _IOR( TG03CAP_IOCTL_BASE, 6, short[256] )
#define IOCTL_SET_CFG_FILE  _IO ( TG03CAP_IOCTL_BASE, 7 )
#define IOCTL_SET_DBGMODE   _IO ( TG03CAP_IOCTL_BASE, 8 )
#define IOCTL_VIEW_DELTA        _IO ( TG03CAP_IOCTL_BASE, 9 )
#define IOCTL_VIEW_CONFIG       _IO ( TG03CAP_IOCTL_BASE, 10)
#define IOCTL_VIEW_REFERENCE    _IO ( TG03CAP_IOCTL_BASE, 11)
#define IOCTL_TEST_REFERENCE    _IO ( TG03CAP_IOCTL_BASE, 12)


#endif /* #define CAT_ATMEL_DATA_H */

