/*

* Certain software is contributed or developed by TOSHIBA CORPORATION.
*
* Copyright (C) 2010 TOSHIBA CORPORATION All rights reserved.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by FSF, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* This code is based on mt9d112.c.
* The original copyright and notice are described below.
*/

/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */


#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
//#include "mt9d112.h"
#include "camsensor_gpio.h"

struct mt9d112_work {
    struct work_struct work;
};

struct mt9d112_ctrl {
    const struct msm_camera_sensor_info *sensordata;
};

static struct mt9d112_ctrl *mt9d112_ctrl;

DEFINE_MUTEX(mt9d112_mtx);
//static int16_t mt9d112_effect = CAMERA_EFFECT_OFF;

static struct CameraSensorI2CCmdType    I2CCmd;

static int curmode = SENSOR_PREVIEW_MODE;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/

/*=============================================================*/

/* ---------------------------------------------------------------------------
 *    FUNCTION        IEE001AA_I2CWriteByte
 * ------------------------------------------------------------------------ */
static int IEE001AA_I2CWriteByte (unsigned short offset, unsigned char data)
{
    unsigned char   ucdata = (unsigned char)data;
    unsigned short  retry = 3;
    int fRet;

    I2CCmd.slave_addr  = 0x34;
    I2CCmd.offset      = offset;
    I2CCmd.buf_ptr     = (unsigned char *)(&ucdata);
    I2CCmd.len         = 1;
    for (++retry, fRet = FALSE ; retry && fRet == FALSE; --retry)
    {
        fRet = camsensor_gpioi2c_write(&I2CCmd);
    }
    return fRet;
}

/* ---------------------------------------------------------------------------
 *    FUNCTION        IEE001AA_I2CWriteWord
 * ------------------------------------------------------------------------ */
static int IEE001AA_I2CWriteWord (unsigned short offset, unsigned short data)
{
    unsigned char   ucdata[2];
    unsigned short  retry = 3;
    int fRet;

    ucdata[0] = (unsigned char)(data);
    ucdata[1] = (unsigned char)(data >> 8);

    I2CCmd.slave_addr  = 0x34;
    I2CCmd.offset      = offset;
    I2CCmd.buf_ptr     = (unsigned char *)(&ucdata[0]);
    I2CCmd.len         = 2;
    for (++retry, fRet = FALSE ; retry && fRet == FALSE; --retry)
    {
        fRet = camsensor_gpioi2c_write(&I2CCmd);
    }
    return fRet;
}

/* ---------------------------------------------------------------------------
 *    FUNCTION        IEE001AA_I2CWriteLong
 * ------------------------------------------------------------------------ */
static int IEE001AA_I2CWriteLong (unsigned short offset, unsigned int data)
{
    unsigned char   ucdata[4];
    unsigned short  retry = 3;
    int fRet;

    ucdata[0] = (unsigned char)(data);
    ucdata[1] = (unsigned char)(data >> 8);
    ucdata[2] = (unsigned char)(data >> 16);
    ucdata[3] = (unsigned char)(data >> 24);

    I2CCmd.slave_addr  = 0x34;
    I2CCmd.offset      = offset;
    I2CCmd.buf_ptr     = (unsigned char *)(&ucdata[0]);
    I2CCmd.len         = 4;

    for (++retry, fRet = FALSE ; retry && fRet == FALSE; --retry)
    {
        fRet = camsensor_gpioi2c_write(&I2CCmd);
    }
    return fRet;
}

/* ---------------------------------------------------------------------------
 *    FUNCTION        IEE001AA_I2CWriteMultiWord
 * ------------------------------------------------------------------------ */
static int IEE001AA_I2CWriteMultiWord (unsigned short offset, unsigned short* data, unsigned short count)
{
    int fRet;

    I2CCmd.slave_addr  = 0x34;
    I2CCmd.offset      = offset;
    I2CCmd.buf_ptr     = (unsigned char *)data;
    I2CCmd.len         = count * 2;
    fRet = camsensor_gpioi2c_write(&I2CCmd);
    return fRet;
}

#if 0
/* ---------------------------------------------------------------------------
 *    FUNCTION        IEE001AA_I2CReadByte
 * ------------------------------------------------------------------------ */
static int IEE001AA_I2CReadByte (unsigned short offset, unsigned char* data)
{
    unsigned char   ucdata;
    int fRet;

    if (data == NULL)
    {
        return FALSE;
    }

    I2CCmd.slave_addr  = 0x34;
    I2CCmd.offset      = offset;
    I2CCmd.buf_ptr     = (unsigned char *)(&ucdata);
    I2CCmd.len         = 1;
    fRet = camsensor_gpioi2c_read(&I2CCmd);
    *data  = ucdata;
    return fRet;
}

/* ---------------------------------------------------------------------------
 *    FUNCTION        IEE001AA_I2CReadWord
 * ------------------------------------------------------------------------ */
static int IEE001AA_I2CReadWord (unsigned char offset, unsigned char* data)
{
    unsigned char   ucdata[2];
    int fRet;

    if (data == NULL)
    {
        return FALSE;
    }

    I2CCmd.slave_addr  = 0x34;
    I2CCmd.offset      = offset;
    I2CCmd.buf_ptr     = (unsigned char *)(&ucdata[0]);
    I2CCmd.len         = 2;
    fRet = camsensor_gpioi2c_read(&I2CCmd);
    *data  = ucdata[0] | (ucdata[1] << 8);
    return fRet;
}
#endif

///////////////////////////////////////////////////////////////////////
// Sensor
///////////////////////////////////////////////////////////////////////
static long mt9d112_reg_init(void)
{
    printk(KERN_INFO "+mt9d112_reg_init\n");

    // 1.Sensor Wake Up Wait
    mdelay(100);

    // 2.INCK_SET Set Register
    IEE001AA_I2CWriteByte (0x0009, 0x15);       // 19.2MHz

    // 3.Wait 1msec
    mdelay(1);

    // 4.Sensor Start Initialize Setting (I2C)
    IEE001AA_I2CWriteByte (0x00FC, 0x01);        // Manufacturer instruction
    IEE001AA_I2CWriteByte (0x000B, 0x01);        // PLL 54MHz
    IEE001AA_I2CWriteWord (0x12D2, 0x0A8C);      // PIXEL CLK(X100) 27MHZ
    IEE001AA_I2CWriteByte (0x12F8, 0x4B);        // PREVIEW FRAM RATE 15FPS, CAPTURE FRAME RATE 7.5FPS
    IEE001AA_I2CWriteWord (0x1318, 0x001B);      // PIXEL CLK 27MHz
    IEE001AA_I2CWriteWord (0x1304, 0x0020);      // FLICKER VADJ_SENSALL
    IEE001AA_I2CWriteWord (0x1586, 0x0578);      // FLICKER FLC_OPD_HEIGHT
    IEE001AA_I2CWriteWord (0x12F2, 0x1CDE);      // FLICKER FLSHT50
    IEE001AA_I2CWriteWord (0x12F4, 0x1BCF);      // FLICKER FLSHT60
    IEE001AA_I2CWriteWord (0x003A, 0x2A20);      // CAM OUTPUT DATA FORMAT(YUV)
    IEE001AA_I2CWriteByte (0x0009, 0x15);        // input clock setting 19.2MHz
    IEE001AA_I2CWriteByte (0x105E, 0x02);        // ROM WAIT
    IEE001AA_I2CWriteByte (0x1041, 0x00);        // Setting of mute count frame on start up
    IEE001AA_I2CWriteByte (0x00FC, 0x01);        // Manufacturer instruction

    // 5.AF Driver Data
    {
        static uint16_t af_data[] = {
            0xB5F3, 0x4D12, 0x4C12, 0x4A13, 0x4F13, 0x4B14, 0x2100, 0x2A00,
            0xD904, 0x5C6E, 0x5466, 0x3101, 0x4291, 0xD3FA, 0x2100, 0x2200,
            0x2B00, 0xD903, 0x547A, 0x3101, 0x4299, 0xD3FB, 0x490C, 0x6008,
            0x9901, 0x480C, 0x6008, 0x480C, 0x60C2, 0x2180, 0x6141, 0x46C0,
            0x46C0, 0x46C0, 0x2001, 0xB002, 0xBDF0, 0x0000, 0x094C, 0x0018,
            0x094C, 0x0018, 0x0004, 0x0000, 0x0950, 0x0018, 0x0004, 0x0000,
            0x0950, 0x0018, 0x094C, 0x0018, 0x7000, 0x0030, 0xB5F8, 0x1C0F,
            0x1C19, 0x9E06, 0x1C14, 0x1C05, 0x1C38, 0x1C32, 0xF000, 0xF826,
            0xAB00, 0x8018, 0xA800, 0x8800, 0x8170, 0xA800, 0x8800, 0x0180,
            0x8018, 0xA800, 0x8800, 0xF000, 0xF853, 0xAB00, 0x8018, 0x480A,
            0x466A, 0x6800, 0x2102, 0x6803, 0x200C, 0xF67F, 0xFCFC, 0x2801,
            0xD001, 0x2000, 0xE006, 0x1C38, 0x1C21, 0x1C32, 0x702F, 0xF000,
            0xF825, 0x2001, 0xBDF8, 0x0000, 0x0950, 0x0018, 0xB590, 0x1C0C,
            0x1C17, 0x7879, 0x89BA, 0x1A40, 0x4350, 0x1C01, 0x2064, 0xF67F,
            0xFCE7, 0x8939, 0x1840, 0x2C01, 0xD102, 0x2305, 0x56F9, 0x1808,
            0x2800, 0xDA01, 0x2000, 0xE003, 0x4B03, 0x4298, 0xDD00, 0x1C18,
            0x0400, 0x0C00, 0xBD90, 0x0000, 0x03FF, 0x0000, 0xB580, 0x1C17,
            0x783A, 0x2A00, 0xD101, 0x7938, 0x78F9, 0x1A40, 0xD500, 0x4240,
            0x89F9, 0x4341, 0x8A38, 0x1809, 0x6978, 0x1809, 0xF67F, 0xFC0A,
            0x4B02, 0x4298, 0xD900, 0x1C18, 0x80F8, 0xBD80, 0x7FFF, 0x0000,
            0x0201, 0x1200, 0x4308, 0x0400, 0x0C00, 0x46F7, 0x086D, 0x0018
        };
        IEE001AA_I2CWriteMultiWord(0x2000, af_data, sizeof(af_data)/sizeof(uint16_t));
        IEE001AA_I2CWriteWord (0x00D6, 0x0000);  // AF_ADJ_INF
        IEE001AA_I2CWriteWord (0x00CE, 0x947D);  // Checksum
        IEE001AA_I2CWriteWord (0x00DC, 0x0150);  // Driver Size
        IEE001AA_I2CWriteByte (0x000A, 0x01);
    }

    // 6.Shading compensation data
    {
        static uint16_t shd_data1[] = {
            0x8282, 0x0006, 0x000F, 0x0006, 0x000F, 0x0003, 0x000E, 0x0003,
            0x000D, 0x0003, 0x0012, 0x0008, 0x0014, 0x0005, 0x0017, 0x0005,
            0x0017, 0x0006, 0x000F, 0x0006, 0x000F, 0x0003, 0x000E, 0x0003,
            0x000D, 0x0003, 0x0012, 0x0008, 0x0014, 0x0005, 0x0017, 0x0005,
            0x0017, 0x000A, 0x000D, 0x000A, 0x000D, 0x0006, 0x0009, 0x0006,
            0x000A, 0x0004, 0x000E, 0x0005, 0x0012, 0x0008, 0x0013, 0x0008,
            0x0013, 0x0008, 0x0009, 0x0008, 0x0009, 0x0008, 0x0005, 0x0003,
            0x0004, 0x0002, 0x0008, 0x0002, 0x000A, 0x0003, 0x000E, 0x0003,
            0x000E, 0x0007, 0x0006, 0x0007, 0x0006, 0x0006, 0x0004, 0x0004,
            0x0001, 0xFFFE, 0x0003, 0xFFFE, 0x0006, 0x0003, 0x0009, 0x0003,
            0x0009, 0x0006, 0x0006, 0x0006, 0x0006, 0x0003, 0x0002, 0x0000,
            0xFFFF, 0xFFFF, 0x0001, 0xFFFD, 0x0002, 0xFFFD, 0x0007, 0xFFFD,
            0x0007, 0x0001, 0x0008, 0x0001, 0x0008, 0x0000, 0x0002, 0xFFFE,
            0x0001, 0xFFFC, 0x0001, 0xFFFD, 0x0001, 0x0000, 0x0006, 0x0000,
            0x0006, 0xFFFD, 0x0007, 0xFFFD, 0x0007, 0x0000, 0x0006, 0xFFFE,
            0x0007, 0xFFFF, 0x0007, 0xFFFC, 0x0003, 0xFFFD, 0x0003, 0xFFFD,
            0x0003, 0xFFFF, 0x0006, 0xFFFF, 0x0006, 0xFFFD, 0x0009, 0xFFFA,
            0x0009, 0xFFFE, 0x0009, 0xFFFD, 0x0007, 0xFFFE, 0x0009, 0xFFFE,
            0x0009, 0xFFFF, 0x0006, 0xFFFF, 0x0006, 0xFFFD, 0x0009, 0xFFFA,
            0x0009, 0xFFFE, 0x0009, 0xFFFD, 0x0007, 0xFFFE, 0x0009, 0xFFFE,
            0x0009, 0xFFFF, 0xFFF9, 0xFFFF, 0xFFF9, 0x0001, 0xFFFB, 0x0002,
            0xFFF9, 0x0000, 0xFFFE, 0x0001, 0xFFFB, 0x0001, 0xFFFC, 0x0001,
            0xFFFC, 0xFFFF, 0xFFF9, 0xFFFF, 0xFFF9, 0x0001, 0xFFFB, 0x0002,
            0xFFF9, 0x0000, 0xFFFE, 0x0001, 0xFFFB, 0x0001, 0xFFFC, 0x0001,
            0xFFFC, 0x0000, 0xFFFB, 0x0000, 0xFFFB, 0xFFFF, 0xFFF9, 0xFFFF,
            0xFFFF, 0x0000, 0xFFFE, 0x0000, 0xFFFB, 0x0001, 0xFFFB, 0x0001,
            0xFFFB, 0x0001, 0xFFF8, 0x0001, 0xFFF8, 0xFFFE, 0xFFFB, 0xFFFC,
            0xFFFC, 0xFFFC, 0xFFFB, 0xFFFE, 0xFFFC, 0x0001, 0xFFFC, 0x0001,
            0xFFFC, 0x0006, 0xFFFB, 0x0006, 0xFFFB, 0x0000, 0xFFFE, 0xFFFE,
            0xFFFF, 0xFFFF, 0xFFFE, 0xFFFF, 0xFFFF, 0x0005, 0xFFFD, 0x0005,
            0xFFFD, 0x0006, 0x0000, 0x0006, 0x0000, 0x0005, 0x0002, 0x0001,
            0x0004, 0x0001, 0x0002, 0x0002, 0x0002, 0x0005, 0x0002, 0x0005,
            0x0002, 0x000D, 0x0003, 0x000D, 0x0003, 0x0007, 0x0009, 0x0005,
            0x0007, 0x0006, 0x0008, 0x0002, 0x0007, 0x0007, 0x0009, 0x0007,
            0x0009, 0x0010, 0x000A, 0x0010, 0x000A, 0x000C, 0x000D, 0x000B,
            0x0010, 0x0008, 0x0011, 0x0007, 0x000B, 0x0009, 0x0009, 0x0009,
            0x0009, 0x0010, 0x000D, 0x0010, 0x000D, 0x000D, 0x000D, 0x000E,
            0x000D, 0x000C, 0x0013, 0x0009, 0x0014, 0x0009, 0x000C, 0x0009,
            0x000C, 0x0010, 0x000D, 0x0010, 0x000D, 0x000D, 0x000D, 0x000E,
            0x000D, 0x000C, 0x0013, 0x0009, 0x0014, 0x0009, 0x000C, 0x0009,
            0x000C
        };
        static uint16_t shd_data2[] = {
            0x9191, 0x0841, 0x08EE, 0x0844, 0x08DA, 0x0769, 0x07DA, 0x0700,
            0x075F, 0x0707, 0x0752, 0x0778, 0x07AD, 0x0845, 0x0874, 0x0825,
            0x0844, 0x0853, 0x08FD, 0x080A, 0x089C, 0x071F, 0x0784, 0x069F,
            0x06F0, 0x06AA, 0x06E5, 0x0735, 0x0760, 0x081B, 0x0848, 0x0844,
            0x0869, 0x0717, 0x0797, 0x06BA, 0x071A, 0x05AD, 0x05DD, 0x051D,
            0x0536, 0x052C, 0x0531, 0x05D0, 0x05CA, 0x06E4, 0x06EC, 0x072C,
            0x0734, 0x0636, 0x069D, 0x05C7, 0x060D, 0x04B4, 0x04CB, 0x043D,
            0x0441, 0x044B, 0x043E, 0x04D9, 0x04C3, 0x05FD, 0x05F6, 0x0662,
            0x0661, 0x05D6, 0x0636, 0x0556, 0x0593, 0x045B, 0x046E, 0x03E9,
            0x03EF, 0x03F4, 0x03EE, 0x047A, 0x046D, 0x0589, 0x0591, 0x0603,
            0x0612, 0x060C, 0x0663, 0x058A, 0x05BB, 0x0487, 0x0490, 0x040D,
            0x040F, 0x0415, 0x0413, 0x049D, 0x049C, 0x05AE, 0x05CB, 0x062B,
            0x0653, 0x06C9, 0x0719, 0x065A, 0x0681, 0x0533, 0x0530, 0x04AD,
            0x04A7, 0x04B1, 0x04B1, 0x0540, 0x054D, 0x066F, 0x06AA, 0x06D2,
            0x071C, 0x07ED, 0x0851, 0x0797, 0x07D3, 0x0679, 0x0686, 0x05D1,
            0x05DC, 0x05D2, 0x05EA, 0x067A, 0x06AF, 0x0792, 0x0803, 0x07CC,
            0x084B, 0x0948, 0x09DE, 0x0904, 0x0978, 0x0810, 0x085A, 0x0784,
            0x07CC, 0x0781, 0x07DC, 0x0804, 0x0884, 0x08E8, 0x099A, 0x0905,
            0x09B8, 0x0924, 0x09C4, 0x0929, 0x09B4, 0x0843, 0x08AD, 0x07CC,
            0x0838, 0x07CB, 0x0846, 0x0835, 0x08D2, 0x08FA, 0x09B6, 0x08DD,
            0x099E, 0x0825, 0x0900, 0x0826, 0x08F2, 0x075E, 0x07F3, 0x0704,
            0x0772, 0x070E, 0x0775, 0x0779, 0x07F5, 0x0843, 0x08D4, 0x081F,
            0x08BA, 0x0841, 0x0917, 0x0803, 0x08BB, 0x0733, 0x07A9, 0x06BF,
            0x0710, 0x06C9, 0x0717, 0x0751, 0x07B0, 0x082D, 0x08AD, 0x0847,
            0x08DB, 0x0713, 0x07C2, 0x06C9, 0x0754, 0x05DF, 0x0620, 0x0557,
            0x0574, 0x0565, 0x057B, 0x0607, 0x062C, 0x0708, 0x0756, 0x0736,
            0x07A4, 0x0627, 0x06BA, 0x05CE, 0x063F, 0x04E0, 0x050B, 0x0471,
            0x047C, 0x0482, 0x047D, 0x050F, 0x050C, 0x061B, 0x063B, 0x0661,
            0x06A6, 0x059C, 0x0629, 0x0535, 0x05A1, 0x0461, 0x048E, 0x03FB,
            0x0405, 0x040C, 0x0400, 0x0492, 0x0480, 0x0585, 0x058F, 0x05DF,
            0x060B, 0x059D, 0x0621, 0x0533, 0x059B, 0x045D, 0x048B, 0x03F4,
            0x0400, 0x0404, 0x03F9, 0x0487, 0x0475, 0x0576, 0x057D, 0x05CE,
            0x05F7, 0x0620, 0x069D, 0x05C6, 0x062C, 0x04D5, 0x0502, 0x0465,
            0x0474, 0x0471, 0x046D, 0x04F5, 0x04ED, 0x05F5, 0x060C, 0x0634,
            0x066D, 0x0700, 0x0782, 0x06BB, 0x072B, 0x05D4, 0x0612, 0x054B,
            0x056C, 0x0555, 0x0569, 0x05EA, 0x0603, 0x06CE, 0x0708, 0x06EA,
            0x0741, 0x0823, 0x08B5, 0x07E9, 0x086C, 0x0723, 0x0780, 0x06B4,
            0x06FC, 0x06BA, 0x06F9, 0x072E, 0x0772, 0x07EA, 0x0840, 0x07F6,
            0x0859, 0x0808, 0x0893, 0x080E, 0x0890, 0x074E, 0x07B2, 0x06F3,
            0x0742, 0x06FB, 0x0743, 0x075A, 0x07A4, 0x0802, 0x0855, 0x07E4,
            0x0836
        };
        IEE001AA_I2CWriteByte (0x0068, 0x00);    // SHD-CXC=OFF
        IEE001AA_I2CWriteMultiWord(0x1900, shd_data1, sizeof(shd_data1)/sizeof(uint16_t));
        IEE001AA_I2CWriteMultiWord(0x1B90, shd_data2, sizeof(shd_data2)/sizeof(uint16_t));
        IEE001AA_I2CWriteByte (0x0068, 0x51);    // SHDON CXC ON
    }

    // 7.AWB
    IEE001AA_I2CWriteWord (0x1444,0x0F11);
    IEE001AA_I2CWriteWord (0x1446,0x0EA8);
    IEE001AA_I2CWriteWord (0x1448,0x011D);
    IEE001AA_I2CWriteWord (0x144A,0x0242);
    IEE001AA_I2CWriteWord (0x1450,0x0A6F);
    IEE001AA_I2CWriteWord (0x1452,0x1C98);
    IEE001AA_I2CWriteByte (0x1454,0x97);
    IEE001AA_I2CWriteByte (0x1455,0x83);
    IEE001AA_I2CWriteWord (0x1330,0x003C);
    IEE001AA_I2CWriteWord (0x1332,0x00F0);
    IEE001AA_I2CWriteWord (0x1338,0x0380);
    IEE001AA_I2CWriteByte (0x1340,0x0C);
    IEE001AA_I2CWriteByte (0x1341,0x0C);
    IEE001AA_I2CWriteByte (0x1342,0x0C);
    IEE001AA_I2CWriteByte (0x1343,0x0C);
    IEE001AA_I2CWriteByte (0x1344,0x04);
    IEE001AA_I2CWriteByte (0x1345,0x04);
    IEE001AA_I2CWriteByte (0x1346,0x04);
    IEE001AA_I2CWriteByte (0x1347,0x04);
    IEE001AA_I2CWriteByte (0x1348,0x0C);
    IEE001AA_I2CWriteByte (0x1349,0x0C);
    IEE001AA_I2CWriteByte (0x134A,0xFF);
    IEE001AA_I2CWriteByte (0x134B,0xFF);
    IEE001AA_I2CWriteByte (0x134C,0x04);
    IEE001AA_I2CWriteByte (0x134D,0x04);
    IEE001AA_I2CWriteByte (0x134E,0x04);
    IEE001AA_I2CWriteByte (0x134F,0x04);
    IEE001AA_I2CWriteByte (0x1350,0x02);
    IEE001AA_I2CWriteByte (0x1351,0x02);
    IEE001AA_I2CWriteByte (0x1352,0x02);
    IEE001AA_I2CWriteByte (0x1353,0x02);
    IEE001AA_I2CWriteWord (0x1378,0x1C98);
    IEE001AA_I2CWriteWord (0x137A,0x190A);
    IEE001AA_I2CWriteWord (0x137C,0x0D3E);
    IEE001AA_I2CWriteWord (0x137E,0x0143);
    IEE001AA_I2CWriteWord (0x1384,0x1B58);
    IEE001AA_I2CWriteWord (0x138C,0x0BBC);
    IEE001AA_I2CWriteWord (0x138E,0x1775);
    IEE001AA_I2CWriteWord (0x13A8,0x7EFE);
    IEE001AA_I2CWriteWord (0x13AA,0x8364);
    IEE001AA_I2CWriteByte (0x13BF,0x01);

    // 8.Hue
    {
        static uint16_t hue_data[] = {
            0xFFF6, 0xFFEF, 0xFFF6, 0xFFEF, 0xFFF6, 0xFFEF, 0xFFF6, 0xFFEF,
            0x4040, 0x4040, 0x4040, 0x4040
        };
        IEE001AA_I2CWriteMultiWord(0x15D0, hue_data, sizeof(hue_data)/sizeof(uint16_t));
    }

    // 9.Gamma
    {
        static uint16_t ganmma_data[] = {
            0x0000, 0x00B2, 0x0107, 0x012F, 0x013F, 0x0149, 0x0155, 0x0160,
            0x0167, 0x016F
        };
        IEE001AA_I2CWriteMultiWord(0x15E8, ganmma_data, sizeof(ganmma_data)/sizeof(uint16_t));
    }

    // 10.Matrix
    {
        static uint16_t mat_data[] = {
            0x00DD, 0x03A4, 0x03FF, 0x03D3, 0x00C8, 0x03E5, 0x03F8, 0x03BD,
            0x00CC
        };
        IEE001AA_I2CWriteWord (0x1624, 0x693D);
        IEE001AA_I2CWriteByte (0x0105, 0x41);
        IEE001AA_I2CWriteByte (0x01B1, 0xAE);
        IEE001AA_I2CWriteMultiWord(0x019C, mat_data, sizeof(mat_data)/sizeof(uint16_t));
    }

    // 11.AE
    IEE001AA_I2CWriteByte (0x0103,0x01);
    IEE001AA_I2CWriteWord (0x12AE,0x2100);
    IEE001AA_I2CWriteWord (0x1442,0x0000);
    IEE001AA_I2CWriteWord (0x1440,0x00B0);
    IEE001AA_I2CWriteWord (0x12B2,0x2311);
    IEE001AA_I2CWriteWord (0x12B4,0x0AAA);
    IEE001AA_I2CWriteWord (0x12B6,0x0000);
    IEE001AA_I2CWriteWord (0x12B8,0x0000);
    IEE001AA_I2CWriteWord (0x12BA,0x26B4);
    IEE001AA_I2CWriteWord (0x12BC,0x08AA);
    IEE001AA_I2CWriteWord (0x12BE,0x0000);          //MAXSHTSCL_L
    IEE001AA_I2CWriteWord (0x12C0,0x0000);          //AGCMAXSCL_L
    IEE001AA_I2CWriteByte (0x12F6,0x20);
    IEE001AA_I2CWriteByte (0x0104,0x01);
    IEE001AA_I2CWriteByte (0x01BD,0x04);
    IEE001AA_I2CWriteByte (0x01BE,0x04);
    IEE001AA_I2CWriteByte (0x01BF,0x04);
    IEE001AA_I2CWriteByte (0x01C0,0x04);
    IEE001AA_I2CWriteByte (0x1491,0x00);            // CAP CONFIG

    // 12.defect cmpensation
    IEE001AA_I2CWriteByte (0x162B,0x20);
    IEE001AA_I2CWriteByte (0x162C,0x10);
    IEE001AA_I2CWriteByte (0x162D,0x20);
    IEE001AA_I2CWriteByte (0x162E,0x10);
    IEE001AA_I2CWriteByte (0x162F,0x10);
    IEE001AA_I2CWriteByte (0x1630,0x08);
    IEE001AA_I2CWriteByte (0x1631,0x10);
    IEE001AA_I2CWriteByte (0x1632,0x08);
    IEE001AA_I2CWriteLong (0x1018,0x00100002);
    IEE001AA_I2CWriteLong (0x101C,0x00080010);
    IEE001AA_I2CWriteLong (0x1020,0x00100010);
    IEE001AA_I2CWriteLong (0x1024,0x00100008);
    IEE001AA_I2CWriteLong (0x1028,0x00000010);
    IEE001AA_I2CWriteByte (0x102F,0x10);
    IEE001AA_I2CWriteLong (0x1030,0x004000A0);
    IEE001AA_I2CWriteByte (0x2D3B,0x02);
    IEE001AA_I2CWriteByte (0x2D6F,0x02);
    IEE001AA_I2CWriteByte (0x2DA3,0x02);
    IEE001AA_I2CWriteByte (0x0062,0x12);
    IEE001AA_I2CWriteByte (0x2D12,0x1A);
    IEE001AA_I2CWriteByte (0x2D13,0x1A);
    IEE001AA_I2CWriteByte (0x2D14,0x34);
    IEE001AA_I2CWriteByte (0x2D15,0x1A);
    IEE001AA_I2CWriteByte (0x2D16,0x1A);
    IEE001AA_I2CWriteByte (0x2D17,0x34);
    IEE001AA_I2CWriteByte (0x2D18,0x1A);
    IEE001AA_I2CWriteByte (0x2D19,0x1A);
    IEE001AA_I2CWriteByte (0x2D1A,0x34);
    IEE001AA_I2CWriteByte (0x2D46,0x1A);
    IEE001AA_I2CWriteByte (0x2D47,0x1A);
    IEE001AA_I2CWriteByte (0x2D48,0x34);
    IEE001AA_I2CWriteByte (0x2D49,0x1A);
    IEE001AA_I2CWriteByte (0x2D4A,0x1A);
    IEE001AA_I2CWriteByte (0x2D4B,0x34);
    IEE001AA_I2CWriteByte (0x2D4C,0x1A);
    IEE001AA_I2CWriteByte (0x2D4D,0x1A);
    IEE001AA_I2CWriteByte (0x2D4E,0x34);
    IEE001AA_I2CWriteByte (0x2D7A,0x1A);
    IEE001AA_I2CWriteByte (0x2D7B,0x1A);
    IEE001AA_I2CWriteByte (0x2D7C,0x34);
    IEE001AA_I2CWriteByte (0x2D7D,0x1A);
    IEE001AA_I2CWriteByte (0x2D7E,0x1A);
    IEE001AA_I2CWriteByte (0x2D7F,0x34);
    IEE001AA_I2CWriteByte (0x2D80,0x1A);
    IEE001AA_I2CWriteByte (0x2D81,0x1A);
    IEE001AA_I2CWriteByte (0x2D82,0x34);
    IEE001AA_I2CWriteByte (0x2D11,0x16);
    IEE001AA_I2CWriteByte (0x2D45,0x16);
    IEE001AA_I2CWriteByte (0x2D79,0x16);
    IEE001AA_I2CWriteByte (0x1634,0x30);
    IEE001AA_I2CWriteByte (0x01C1,0x68);
    IEE001AA_I2CWriteByte (0x01C2,0x60);
    IEE001AA_I2CWriteByte (0x01C3,0x60);
    IEE001AA_I2CWriteWord (0x1646,0x0200);
    IEE001AA_I2CWriteWord (0x1648,0x0A00);
    IEE001AA_I2CWriteWord (0x2D3E,0x03FD);
    IEE001AA_I2CWriteWord (0x2D40,0x06C8);
    IEE001AA_I2CWriteWord (0x2D42,0x0020);
    IEE001AA_I2CWriteWord (0x2D72,0x03FD);
    IEE001AA_I2CWriteWord (0x2D74,0x06C8);
    IEE001AA_I2CWriteWord (0x2D76,0x0020);
    IEE001AA_I2CWriteWord (0x2DA6,0x03FD);
    IEE001AA_I2CWriteWord (0x2DA8,0x0800);
    IEE001AA_I2CWriteWord (0x2DAA,0x0020);
    IEE001AA_I2CWriteWord (0x15C6,0x085F);
    IEE001AA_I2CWriteWord (0x15C8,0x0BF5);
    IEE001AA_I2CWriteWord (0x15CA,0x0100);
    IEE001AA_I2CWriteByte (0x2D2E,0x10);
    IEE001AA_I2CWriteByte (0x2D2F,0x20);
    IEE001AA_I2CWriteByte (0x2D62,0x10);
    IEE001AA_I2CWriteByte (0x2D63,0x20);
    IEE001AA_I2CWriteByte (0x2D96,0x10);
    IEE001AA_I2CWriteByte (0x2D97,0x20);
    IEE001AA_I2CWriteWord (0x168E,0x085F);
    IEE001AA_I2CWriteWord (0x1690,0x0BF5);
    IEE001AA_I2CWriteByte (0x2D30,0x10);
    IEE001AA_I2CWriteByte (0x2D31,0x20);
    IEE001AA_I2CWriteByte (0x2D64,0x10);
    IEE001AA_I2CWriteByte (0x2D65,0x20);
    IEE001AA_I2CWriteByte (0x2D98,0x10);
    IEE001AA_I2CWriteByte (0x2D99,0x20);
    IEE001AA_I2CWriteWord (0x1694,0x085F);
    IEE001AA_I2CWriteWord (0x1696,0x0BF5);
    IEE001AA_I2CWriteByte (0x2D22,0x02);
    IEE001AA_I2CWriteByte (0x2D23,0x02);
    IEE001AA_I2CWriteByte (0x2D24,0x02);
    IEE001AA_I2CWriteByte (0x2D25,0x03);
    IEE001AA_I2CWriteByte (0x2D56,0x02);
    IEE001AA_I2CWriteByte (0x2D57,0x02);
    IEE001AA_I2CWriteByte (0x2D58,0x02);
    IEE001AA_I2CWriteByte (0x2D59,0x03);
    IEE001AA_I2CWriteByte (0x2D8A,0x02);
    IEE001AA_I2CWriteByte (0x2D8B,0x02);
    IEE001AA_I2CWriteByte (0x2D8C,0x02);
    IEE001AA_I2CWriteByte (0x2D8D,0x03);
    IEE001AA_I2CWriteWord (0x1668,0x085F);
    IEE001AA_I2CWriteWord (0x166A,0x094D);
    IEE001AA_I2CWriteWord (0x166C,0x0AA1);
    IEE001AA_I2CWriteByte (0x166E,0x04);
    IEE001AA_I2CWriteByte (0x166F,0x0C);
    IEE001AA_I2CWriteWord (0x1670,0x0200);
    IEE001AA_I2CWriteWord (0x1672,0x0A00);
    IEE001AA_I2CWriteByte (0x2D1B,0x04);
    IEE001AA_I2CWriteByte (0x2D1C,0x04);
    IEE001AA_I2CWriteByte (0x2D1D,0x04);
    IEE001AA_I2CWriteByte (0x2D1E,0x04);
    IEE001AA_I2CWriteByte (0x2D1F,0x04);
    IEE001AA_I2CWriteByte (0x2D4F,0x04);
    IEE001AA_I2CWriteByte (0x2D50,0x04);
    IEE001AA_I2CWriteByte (0x2D51,0x04);
    IEE001AA_I2CWriteByte (0x2D52,0x04);
    IEE001AA_I2CWriteByte (0x2D53,0x04);
    IEE001AA_I2CWriteByte (0x2D83,0x04);
    IEE001AA_I2CWriteByte (0x2D84,0x04);
    IEE001AA_I2CWriteByte (0x2D85,0x04);
    IEE001AA_I2CWriteByte (0x2D86,0x04);
    IEE001AA_I2CWriteByte (0x2D87,0x04);
    IEE001AA_I2CWriteWord (0x1650,0x062D);
    IEE001AA_I2CWriteWord (0x1652,0x085F);
    IEE001AA_I2CWriteWord (0x1654,0x094D);
    IEE001AA_I2CWriteWord (0x1656,0x0AA1);
    IEE001AA_I2CWriteByte (0x1658,0x08);
    IEE001AA_I2CWriteByte (0x1659,0x10);
    IEE001AA_I2CWriteWord (0x165A,0x0200);
    IEE001AA_I2CWriteWord (0x165C,0x0A00);
    IEE001AA_I2CWriteByte (0x2D20,0x00);
    IEE001AA_I2CWriteByte (0x2D21,0x10);
    IEE001AA_I2CWriteByte (0x2D54,0x10);
    IEE001AA_I2CWriteByte (0x2D55,0x10);
    IEE001AA_I2CWriteByte (0x2D88,0x10);
    IEE001AA_I2CWriteByte (0x2D89,0x10);
    IEE001AA_I2CWriteWord (0x1660,0x7289);
    IEE001AA_I2CWriteWord (0x1662,0x748A);
    IEE001AA_I2CWriteByte (0x2D28,0x02);
    IEE001AA_I2CWriteByte (0x2D29,0x02);
    IEE001AA_I2CWriteByte (0x2D2A,0x02);
    IEE001AA_I2CWriteByte (0x2D5C,0x02);
    IEE001AA_I2CWriteByte (0x2D5D,0x02);
    IEE001AA_I2CWriteByte (0x2D5E,0x02);
    IEE001AA_I2CWriteByte (0x2D90,0x02);
    IEE001AA_I2CWriteByte (0x2D91,0x02);
    IEE001AA_I2CWriteByte (0x2D92,0x02);
    IEE001AA_I2CWriteWord (0x1684,0x085F);
    IEE001AA_I2CWriteWord (0x1686,0x09F7);
    IEE001AA_I2CWriteByte (0x1674,0x10);
    IEE001AA_I2CWriteByte (0x1675,0x18);
    IEE001AA_I2CWriteWord (0x1676,0x085F);
    IEE001AA_I2CWriteWord (0x1678,0x0BF5);
    IEE001AA_I2CWriteByte (0x2D26,0x00);
    IEE001AA_I2CWriteByte (0x2D27,0x0A);
    IEE001AA_I2CWriteByte (0x2D5A,0x00);
    IEE001AA_I2CWriteByte (0x2D5B,0x0A);
    IEE001AA_I2CWriteByte (0x2D8E,0x00);
    IEE001AA_I2CWriteByte (0x2D8F,0x0A);
    IEE001AA_I2CWriteWord (0x167C,0x062D);
    IEE001AA_I2CWriteWord (0x167E,0x082C);
    IEE001AA_I2CWriteWord (0x1440,0x00B0);

    // 13.YUV VGA Preview
    IEE001AA_I2CWriteByte (0x0014, 0x00);    // CAPNUM
    IEE001AA_I2CWriteByte (0x0018, 0x01);    // SENSMODE_MOVIE
    IEE001AA_I2CWriteByte (0x001B, 0x01);    // FPSTYPE_MOVIE
    IEE001AA_I2CWriteByte (0x001C, 0x00);    // OUTFMT_MONI  (0: YUV422)
    IEE001AA_I2CWriteByte (0x001D, 0x00);    // OUTFMT_CAP   (0: YUV422)
    IEE001AA_I2CWriteByte (0x001E, 0x00);    // OUTFMT_MOVIE (0: YUV422)
    IEE001AA_I2CWriteByte (0x001F, 0x00);    // READVECT_MONI
    IEE001AA_I2CWriteByte (0x0020, 0x00);    // READVECT_CAP
    IEE001AA_I2CWriteByte (0x0021, 0x00);    // READVECT_MOVIE
    IEE001AA_I2CWriteWord (0x0022, 0x0280);  // HSIZE_MONI  (640)
    IEE001AA_I2CWriteWord (0x0024, 0x0800);  // HSIZE_CAP   (2048)
    IEE001AA_I2CWriteWord (0x0026, 0x0280);  // HSIZE_MOVIE (640)
    IEE001AA_I2CWriteWord (0x0028, 0x01E0);  // VSIZE_MONI  (480)
    IEE001AA_I2CWriteWord (0x002A, 0x0600);  // VSIZE_CAP   (1536)
    IEE001AA_I2CWriteWord (0x002C, 0x01E0);  // VSIZE_MOVIE (480)
    IEE001AA_I2CWriteByte (0x002E, 0x02);    // AFMODE_MONI (2:Manual AF mode)
    IEE001AA_I2CWriteByte (0x0011, 0x00);    // MODESEL (0: moniter mode)
    IEE001AA_I2CWriteByte (0x0012, 0x01);    // MONI_REFLESH (1:update)

    // 14.AF
    IEE001AA_I2CWriteByte (0x00B5, 0x13);
    IEE001AA_I2CWriteByte (0x00B6, 0x00);
    IEE001AA_I2CWriteByte (0x01DF, 0x00);
    IEE001AA_I2CWriteByte (0x01E0, 0x13);
    IEE001AA_I2CWriteWord (0x13F6, 0x7080);
    IEE001AA_I2CWriteByte (0x00C0, 0x64);
    IEE001AA_I2CWriteByte (0x00C1, 0x64);
    IEE001AA_I2CWriteByte (0x00B7, 0x00);
    IEE001AA_I2CWriteByte (0x00B8, 0x01);
    IEE001AA_I2CWriteByte (0x00B9, 0x00);
    IEE001AA_I2CWriteByte (0x00BA, 0x00);
    IEE001AA_I2CWriteWord (0x00C2, 0x0384);
    IEE001AA_I2CWriteWord (0x00C4, 0x0001);
    IEE001AA_I2CWriteWord (0x00C6, 0x0000);

    // 15.AF Window
    {
        static uint16_t afw_data[] = {
            0x0158, 0x0194, 0x0476, 0x0258, 0x0294, 0x0576, 0x0032, 0x02D8,
            0x0032, 0x0032, 0x02D8, 0x0032
        };
        IEE001AA_I2CWriteMultiWord(0x0046, afw_data, sizeof(afw_data)/sizeof(uint16_t));
    }

    // 16.AF OPD-TH
    IEE001AA_I2CWriteWord (0x1424, 0x0138);  // IIR AF-OPD_TH MIN
    IEE001AA_I2CWriteWord (0x1426, 0x01F4);  // IIR AF-OPD_TH MAX
    IEE001AA_I2CWriteWord (0x142A, 0x00C8);  // FIR AF-OPD_TH LOW
    IEE001AA_I2CWriteWord (0x142C, 0x0000);  // FIR AF-OPD_TH HIGH

    // 17.AF WINDOW SEL
    IEE001AA_I2CWriteByte (0x01D3, 0x01);    // CENTER WINDOW

    // 18.Resolution Set
    curmode = SENSOR_PREVIEW_MODE;

    printk(KERN_INFO "-mt9d112_reg_init\n");

    return 0;
}

static long mt9d112_set_effect(int mode, int effect)
{
    long rc = 0;

    printk(KERN_INFO "+mt9d112_set_effect (%d. %d)\n", mode, effect);

    switch (mode) {
    case SENSOR_PREVIEW_MODE:
        break;

    case SENSOR_SNAPSHOT_MODE:
        break;

    default:
        break;
    }

    switch (effect) {
    case CAMERA_EFFECT_OFF:
            break;

    case CAMERA_EFFECT_MONO:
        break;

    case CAMERA_EFFECT_NEGATIVE:
        break;

    case CAMERA_EFFECT_SOLARIZE:
        break;

    case CAMERA_EFFECT_SEPIA:
        break;

    default:
        return -EINVAL;
    }

    printk(KERN_INFO "-mt9d112_set_effect Done.\n");
    return rc;
}

static long mt9d112_set_sensor_mode(int mode)
{
    printk(KERN_INFO "+mt9d112_set_sensor_mode (%d)\n", mode);

    switch (mode) {
    case SENSOR_PREVIEW_MODE:   // Preview MAX : 640*480 & 15FPS
        if (curmode == SENSOR_PREVIEW_MODE)
            break;
        curmode = mode;

        // 1.Mode Change
        IEE001AA_I2CWriteByte (0x00FC, 0x02);    // Manufacturer instruction
        IEE001AA_I2CWriteByte (0x0011, 0x00);    // MODESEL (0: monitoring)
        IEE001AA_I2CWriteByte (0x0012, 0x01);    // MONI_REFLESH (1:update)

        // 2.Stability wait
        mdelay(66);

        break;

    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:  // Still MAX : 2048*1536 & 7.5FPS
        if (curmode == SENSOR_SNAPSHOT_MODE 
         || curmode == SENSOR_RAW_SNAPSHOT_MODE)
            break;
        curmode = mode;

        // 1.Mode Change
        IEE001AA_I2CWriteWord (0x0024, 0x0800);  // HSIZE_CAP   (2048)
        IEE001AA_I2CWriteWord (0x002A, 0x0600);  // VSIZE_CAP   (1536)
        IEE001AA_I2CWriteByte (0x00FC, 0x02);    // Manufacturer instruction
        IEE001AA_I2CWriteByte (0x0011, 0x02);    // MODESEL (2: Capture)

        // 2.Stability wait
        mdelay(66*4);

        break;

    default:
        return -EINVAL;
    }

    printk(KERN_INFO "-mt9d112_set_sensor_mode Done.\n");
    return 0;
}

static void mt9d112_whitebalance_i2cwrite(u8 wbvalue)
{

    CDBG("+mt9d112_whitebalance_i2cwrite = %x.\n", wbvalue);

    if( wbvalue > 0 )
    {
    	IEE001AA_I2CWriteByte(0x0102,wbvalue);
    	IEE001AA_I2CWriteByte(0x0107,wbvalue);
    	IEE001AA_I2CWriteByte(0x010C,wbvalue);
    	IEE001AA_I2CWriteByte(0x0111,wbvalue);
    	IEE001AA_I2CWriteByte(0x0116,wbvalue);
    	IEE001AA_I2CWriteByte(0x011B,wbvalue);
    	IEE001AA_I2CWriteByte(0x0120,wbvalue);
    	IEE001AA_I2CWriteByte(0x0125,wbvalue);
    	IEE001AA_I2CWriteByte(0x012A,wbvalue);
    	IEE001AA_I2CWriteByte(0x012F,wbvalue);
    	IEE001AA_I2CWriteByte(0x0134,wbvalue);
    	IEE001AA_I2CWriteByte(0x0139,wbvalue);
     }
     else
     {
        CDBG("Improper register value for white balance settings.\n");
     }

     CDBG("-mt9d112_whitebalance_i2cwrite Done.\n");
}


static long mt9d112_set_whitebalance(int mode)
{
    long rc = 0;
    u8 wbvalue = 0;

    CDBG("+mt9d112_set_whitebalance = %d.\n", mode);

    switch( mode )
    {
        case CAMERA_WB_AUTO:
            wbvalue = 0x20;
            break;
        case CAMERA_WB_DAYLIGHT:
            wbvalue = 0x24;
            break;
        case CAMERA_WB_CLOUDY_DAYLIGHT:
            wbvalue = 0x26;
            break;
        case CAMERA_WB_FLUORESCENT:
            wbvalue = 0x27;
             break;
        case CAMERA_WB_INCANDESCENT:
            wbvalue = 0x28;
            break;
        case CAMERA_WB_SHADE:
            wbvalue = 0x25;
            break; 
        /*These white balance modes are not supported by ISX001 sensor
        case CAMERA_WB_TWILIGHT:
        case CAMERA_WB_CUSTOM:
        */
        default://default mode auto
            wbvalue = 0x20; 
    }

    mt9d112_whitebalance_i2cwrite( wbvalue );

    CDBG("-mt9d112_set_whitebalance Done.\n");

    return rc;
}

static int mt9d112_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
    int rc = 0;

    CDBG("init entry \n");

    printk(KERN_INFO "+mt9d112_sensor_init_probe\n");

    // 1.Sensor Reset Release       GPIO[0] 'L' -> 'H'
    rc = gpio_request(mt9d112_ctrl->sensordata->sensor_reset, "mt9d112");
    if (!rc) {
        rc = gpio_direction_output(mt9d112_ctrl->sensordata->sensor_reset, 1);
        mdelay(20);
    }
    else
    {
        goto init_probe_fail;
    }

    // 2.Sensor Register Initialize
    rc = mt9d112_reg_init();
    if (rc < 0)
        goto init_probe_fail;

    // 3.Sensor Standby Release     GPIO[1] 'L' -> 'H'
    rc = gpio_request(mt9d112_ctrl->sensordata->sensor_pwd, "mt9d112");
    if (!rc) {
        rc = gpio_direction_output(mt9d112_ctrl->sensordata->sensor_pwd, 1);
        mdelay(20);
    }
    else
    {
        goto init_probe_fail;
    }

    // 4.VFE Stability Wait
    mdelay(800);        // VSYNC * 13 + a

    // 5. Focus Infinity

    printk(KERN_INFO "-mt9d112_sensor_init_probe Done. !\n");
    return rc;

init_probe_fail:
    printk(KERN_INFO "-mt9d112_sensor_init_probe fail ! (%d)\n",rc);
    return rc;
}

int mt9d112_sensor_init(const struct msm_camera_sensor_info *data)
{
    int rc = 0;

    printk(KERN_INFO "+mt9d112_sensor_init\n");

    mt9d112_ctrl = kzalloc(sizeof(struct mt9d112_ctrl), GFP_KERNEL);
    if (!mt9d112_ctrl) {
        CDBG("mt9d112_init failed!\n");
        printk(KERN_INFO " mt9d112_sensor_init kzalloc() failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }

    if (data)
        mt9d112_ctrl->sensordata = data;

    // Input MCLK = 19.2MHz
    msm_camio_clk_rate_set(19200000);
    mdelay(5);

    msm_camio_camif_pad_reg_reset();

    rc = mt9d112_sensor_init_probe(data);
    if (rc < 0) {
        CDBG("mt9d112_sensor_init failed!\n");
        goto init_fail;
    }

init_done:
    printk(KERN_INFO "-mt9d112_sensor_init Done.\n");
    return rc;

init_fail:
    kfree(mt9d112_ctrl);
    printk(KERN_INFO "-mt9d112_sensor_init Fail.\n");
    return rc;
}

int mt9d112_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    int   rc = 0;



    if (copy_from_user(&cfg_data, (void *)argp,
                       sizeof(struct sensor_cfg_data)))
        return -EFAULT;
   printk(KERN_INFO "+mt9d112_sensor_config (%d, %d)\n", cfg_data.cfgtype, cfg_data.mode);

    CDBG("mt9d112_ioctl, cfgtype = %d, mode = %d\n",
            cfg_data.cfgtype, cfg_data.mode);

    mutex_lock(&mt9d112_mtx);
    switch (cfg_data.cfgtype)
    {
    case CFG_SET_MODE:
        rc = mt9d112_set_sensor_mode( cfg_data.mode );
        break;

    case CFG_SET_EFFECT:
        rc = mt9d112_set_effect( cfg_data.mode, cfg_data.cfg.effect );
        break;
    case CFG_SET_WB:
        printk(KERN_INFO "White balance value %d\n",cfg_data.mode); 
        rc = mt9d112_set_whitebalance( cfg_data.mode);
        break;
    case CFG_GET_AF_MAX_STEPS:
    default:
        rc = -EINVAL;
        break;
    }
    mutex_unlock(&mt9d112_mtx);

    printk(KERN_INFO "-mt9d112_sensor_config Done.(%d)\n", rc);
    return rc;
}

int mt9d112_sensor_release(void)
{
    printk(KERN_INFO "+mt9d112_sensor_release\n");

    mutex_lock(&mt9d112_mtx);

    // 1.Sensor Standby     GPIO[1] 'H' -> 'L'
    gpio_direction_output(mt9d112_ctrl->sensordata->sensor_pwd, 0);
    gpio_free(mt9d112_ctrl->sensordata->sensor_pwd);
    mdelay(10);

    // 2.Sensor Reset       GPIO[0] 'H' -> 'L'
    gpio_direction_output(mt9d112_ctrl->sensordata->sensor_reset, 0);
    gpio_free(mt9d112_ctrl->sensordata->sensor_reset);

    // Resource release
    kfree(mt9d112_ctrl);

    CDBG("mt9d112_sensor_release completed\n");

    mutex_unlock(&mt9d112_mtx);

    printk(KERN_INFO "-mt9d112_sensor_release Done.\n");
    return 0;
}

// Sensor Detect Check (for Driver Init)
static int mt9d112_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
    // Camera Sensor ISX001 Detect !
    printk(KERN_INFO "+mt9d112_sensor_probe\n");

    s->s_init = mt9d112_sensor_init;
    s->s_release = mt9d112_sensor_release;
    s->s_config  = mt9d112_sensor_config;

    CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);

    printk(KERN_INFO "-mt9d112_sensor_probe Done.\n");
    return 0;
}

static int __mt9d112_probe(struct platform_device *pdev)
{
    printk(KERN_INFO "+__mt9d112_probe\n");
    return msm_camera_drv_start(pdev, mt9d112_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __mt9d112_probe,
    .driver = {
        .name = "msm_camera_mt9d112",
        .owner = THIS_MODULE,
    },
};

static int __init mt9d112_init(void)
{
    printk(KERN_INFO "+mt9d112_init\n");
    return platform_driver_register(&msm_camera_driver);
}

module_init(mt9d112_init);
