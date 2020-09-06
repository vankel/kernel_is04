/*
  GPIO I2C Driver

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

                    Camera Sensor GPIO Driver Source File

============================================================================*/

/*============================================================================
                        INCLUDE FILES
============================================================================*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include "camsensor_gpio.h"

/*============================================================================
                        INTERNAL FEATURES
============================================================================*/

/*============================================================================
                        CONSTANTS
============================================================================*/
#define TRUE                    1
#define FALSE                   0

#define _DUMP                   1

#define SCL                     2           // I2C SCL GPIO No
#define SDA                     3           // I2C SDA GPIO No

#if 0
#define SCL_Low()               gpio_configure(SCL, GPIOF_DRIVE_OUTPUT)
#define SCL_High()              gpio_configure(SCL, GPIOF_INPUT)
#define SDA_Low()               gpio_configure(SDA, GPIOF_DRIVE_OUTPUT)
#define SDA_High()              gpio_configure(SDA, GPIOF_INPUT)
#else
#define SCL_Low()               gpio_direction_output(SCL, 0)
#define SCL_High()              gpio_direction_input(SCL)
#define SDA_Low()               gpio_direction_output(SDA, 0)
#define SDA_High()              gpio_direction_input(SDA)
#endif

#define SDA_Read()              gpio_get_value(SDA)
#define SCL_Read()              gpio_get_value(SCL)

#if 1
#define LOGI(fmt, args...)      printk(KERN_INFO "gpioI2C: " fmt, ##args)
#define LOGE(fmt, args...)      printk(KERN_ERR "gpioI2C: " fmt, ##args)
#else
#define LOGI(fmt, args...)      do {} while (0)
#define LOGE(fmt, args...)      do {} while (0)
#endif

/*============================================================================
                        MACROS
============================================================================*/

/*============================================================================
                        INTERNAL ABSTRACT DATA TYPES
============================================================================*/

/*============================================================================
                        EXTERNAL VARIABLES DECLARATIONS
============================================================================*/

/*============================================================================
                        EXTERNAL VARIABLES DEFINITIONS
============================================================================*/
int _I2C_LOG_ = 1;

/*============================================================================
                        INTERNAL VARIABLES DEFINITIONS
============================================================================*/

/*============================================================================
                        INTERNAL API DECLARATIONS
============================================================================*/
void WaitHighSpeed(int us) {;}
static void ( *Wait )( int us ) = WaitHighSpeed;

static boolean SCL_IsHigh( void )
{
    int cnt;

    // Check SCL High
    for(cnt = 1000 * 12; cnt; cnt--) // 35ms
    {
        udelay(3);
        if( SCL_Read() )    return TRUE;    // SCL High
    }
    return FALSE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_START_CONDITION
===========================================================================*/
static boolean camsensor_gpio_start_condition(void)
{
    // In : SCL In (Pull-UP), SDA In (Pull-UP)
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    // Bus free Check
    if (!SCL_IsHigh())      // SCL Low!
    {
        LOGE("start_condition Error (SCL Low) !\n");
        return FALSE;
    }
    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(5);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_STOP_CONDITION
===========================================================================*/
static boolean camsensor_gpio_stop_condition(void)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL In (Pull-UP), SDA In (Pull-UP)

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High wait
    SCL_IsHigh();
    Wait(3);

    // SDA HIGH (Pull-UP)
    SDA_High();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_RESTART_CONDITION
===========================================================================*/
static boolean camsensor_gpio_restart_condition(void)
{
    // In : SCL Out (Drive-Low), SDA Out (Drive-Low)
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    // SDA High (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("restart_condition Error (SCL Low) !\n");
        return FALSE;
    }
    Wait(3);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(3);

    // SCL Low (Drive Low)
    SCL_Low();
    Wait(3);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_CHK_ACK
===========================================================================*/
static boolean camsensor_gpio_chk_ack(void)
{
    int sda;

    // In : SCL Out (Drive-Low), SDA In (Pull-UP)
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("chk_ack Error (SCL Low) !\n");
        return FALSE;
    }

    // SDA Signal Read
    sda = SDA_Read();
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // response check
    if (sda)
    {                       // Nack recv
        LOGE("chk_ack Error (Nack. Receive)!\n");
        return FALSE;
    }

    return TRUE;            // Ack Recv
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_SEND_NACK
===========================================================================*/
static boolean camsensor_gpio_send_ack(uint16_t len)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    if( !len )
    {   // Nack
        // SDA High (Pull-UP)
        SDA_High();
        Wait(2);
    }
    else
    {   // Ack
        // SDA Low (Drive-Low)
        SDA_Low();
        Wait(2);
    }

    // SCL High (Pull-UP)
    SCL_High();
    Wait(5);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_SEND_BYTE
===========================================================================*/
static boolean camsensor_gpio_send_byte(uint8_t val)
{
    int     dir;
    uint8_t mask;

    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    mask = 0x80;
    // MSB Bit Output
    dir = val & mask;
    if (dir)
    {
        // SDA High (Pull-UP)
        SDA_High();
    }
    else
    {
        // SDA Low (Drive-Low)
        SDA_Low();
    }
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("send_byte SCL Error !\n");
        return FALSE;
    }
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // 7bit output
    mask >>= 1;
    while( mask )
    {
        // SDA 1Bit out
        dir = val & mask;
        if (dir)
        {
            // SDA High (Pull-UP)
            SDA_High();
        }
        else
        {
            // SDA Low (Drive-Low)
            SDA_Low();
        }
        Wait(2);

        // SCL High (Pull-UP) 
        SCL_High();
        Wait(4);

        // SCL Low (Drive-Low)
        SCL_Low();
        Wait(2);

        mask >>= 1;
    }

    // SDA High (Pull-UP)
    SDA_High();
    Wait(4);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_RECV_BYTE
===========================================================================*/
static boolean camsensor_gpio_recv_byte(uint8_t *val)
{
    int     sda;
    uint8_t mask = 0x80;
    uint8_t data = 0x00;

    // In : SCL Out (Drive-Low), SDA Out (Drive-Low)
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // SDA Hige (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh())
    {
        LOGE("recv_byte SCL Error !\n");
        return FALSE;
    }
    Wait(3);

    // SDA in
    sda = SDA_Read();
    if (sda)
        data |= mask;

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // 7bit output
    mask >>= 1;
    while(mask)
    {
        // SCL High (Pull-UP)
        SCL_High();
        Wait(2);

        // SDA in
        sda = SDA_Read();
        if (sda)
          data |= mask;
        Wait(2);

        // SCL Low (Drive-Low)
        SCL_Low();
        Wait(4);

        mask >>= 1;
    }
    // Done, SCL is in LOW.
    *val = data;

    Wait(2);

    return TRUE;
}

/*============================================================================
                        EXTERNAL API DEFINITIONS
============================================================================*/

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_WRITE
===========================================================================*/
int camsensor_gpioi2c_write(struct CameraSensorI2CCmdType *pI2CCmd)
{
    int rc = TRUE;
    uint16_t len;
    uint8_t  *pd;
    char    tszDump[64] = "";
    int     idx = 0;

    // 1.Start Condition
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;

    // 2.SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(pI2CCmd->slave_addr)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("write. fault Slave addr\n");
        goto fault;
    }

    // 3.SEND - Data
    for(len = pI2CCmd->wlen, pd = pI2CCmd->pwdata ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            LOGE("write. fault Data\n");
            goto fault;
        }
    }

    // 4.Stop Condition
    camsensor_gpio_stop_condition();
    goto exit;

fault : ;
    camsensor_gpio_stop_condition();

exit : ;
#if _DUMP
    for(len = 1, pd = pI2CCmd->pwdata+1 ; len < pI2CCmd->wlen && len < 12; ++len, ++pd)
    {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }
#endif
    if (_I2C_LOG_ != 0 || !rc)
        LOGI("write C:%02X T:%s (%s)\n",
               *pI2CCmd->pwdata, tszDump, rc ? "Ok" : "Error");

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_READ
===========================================================================*/
int camsensor_gpioi2c_read(struct CameraSensorI2CCmdType *pI2CCmd)
{
    int rc = TRUE;
    uint16_t len;
    uint8_t  *pd;
    char    tszDump[72] = "";
    int     idx = 0;

    // 1.Start Condition    
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;

    // 2. SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(pI2CCmd->slave_addr)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("read. fault Slave[W] addr\n");
        goto fault;
    } 

    // 3.SEND - Data
    for(len = pI2CCmd->wlen, pd = pI2CCmd->pwdata ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            LOGE("raed. fault Write Data\n");
            goto fault;
        }
    }

    // 4.Restart Condition
    if( !(rc = camsensor_gpio_restart_condition()) )    goto fault;

    // 5.SEND - Slave Address again(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(pI2CCmd->slave_addr | 1)) )  goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        LOGE("read. fault Slave[R] addr\n");
        goto fault;
    }

    // 6.RCV - Data
    for(len = pI2CCmd->rlen, pd = pI2CCmd->prdata ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_recv_byte(pd)) )    goto fault;
        // Send Ack
        if( !(rc = camsensor_gpio_send_ack(len-1)) )
        {
            LOGE("read. fault ack \n");
            goto fault;
        }
    }

    // 7.Stop Condition
    camsensor_gpio_stop_condition();

#if _DUMP
    if (pI2CCmd->wlen > 1)
        idx = sprintf(&tszDump[idx], "T:", *pd);
    for(len = 1, pd = pI2CCmd->pwdata+1 ; len < pI2CCmd->wlen && len < 8; ++len, ++pd)
    {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }
    idx += sprintf(&tszDump[idx], "R:", *pd);
    for(len = 0, pd = pI2CCmd->prdata ; len < pI2CCmd->rlen && len < 12; ++len, ++pd)
    {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }
#endif

    goto exit;

fault : ;
    camsensor_gpio_stop_condition();
    sprintf(tszDump, "%02X ",*pI2CCmd->pwdata);

exit : ;
    if (_I2C_LOG_ != 0 || !rc)
        LOGI("read  C:%02X %s (%s)\n", 
             *pI2CCmd->pwdata, tszDump, rc ? "Ok" : "Error");

    return rc ? RET_OK : RET_ERR;
}
