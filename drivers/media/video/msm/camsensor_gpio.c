/*
  Camera Sensor Driver
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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA */
/*============================================================================

                    Camera Sensor GPIO Driver Source File

============================================================================*/

/*============================================================================
                        INCLUDE FILES
============================================================================*/
#include <linux/kernel.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include <mach/camera.h>
#include "camsensor_gpio.h"

/*============================================================================
                        INTERNAL FEATURES
============================================================================*/

/*============================================================================
                        CONSTANTS
============================================================================*/
#define SCL                     37          // I2C SCL GPIO No
#define SDA                     38          // I2C SDA GPIO No

#define SCL_Low()               gpio_configure(SCL, GPIOF_DRIVE_OUTPUT)
#define SCL_High()              gpio_configure(SCL, GPIOF_INPUT)
#define SDA_Low()               gpio_configure(SDA, GPIOF_DRIVE_OUTPUT)
#define SDA_High()              gpio_configure(SDA, GPIOF_INPUT)

#define SDA_Read()              gpio_get_value(SDA)
#define SCL_Read()              gpio_get_value(SCL)

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

    for(cnt = 1000 * 1000; cnt; cnt--)
    {
        ;
        if( SCL_Read() )    return TRUE;    // SCL High
    }
    return FALSE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_START_CONDITION
===========================================================================*/
static boolean camsensor_gpio_start_condition(void)
{

    // Bus free Check
    if (SCL_IsHigh() == FALSE)  // SCL Low!
    {
        CDBG("gpioI2C start_condition Error (SCL Low) !\n");
        //printk(KERN_INFO "gpioI2C start_condition Error (SCL Low) !\n");
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
    SCL_Low();
    Wait(2);

    
    SDA_Low();
    Wait(2);

    
    SCL_High();

    SCL_IsHigh();
    Wait(3);

    
    SDA_High();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_RESTART_CONDITION
===========================================================================*/
static boolean camsensor_gpio_restart_condition(void)
{
    
    SDA_High();
    Wait(2);

    
    SCL_High();

    if (SCL_IsHigh() == FALSE)
    {
        
        CDBG("gpioI2C restart_condition Error (SCL Low) !\n");
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

    // SCL High (Pull-UP)
    SCL_High();

    if (SCL_IsHigh() == FALSE)
    {
        //printk(KERN_INFO "gpioI2C chk_ack Error (SCL Low) !\n");
        CDBG("gpioI2C chk_ack Error (SCL Low) !\n");
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
        
        CDBG("gpioI2C chk_ack Error (Nack. Receive)!\n");
        return FALSE;
    }

    return TRUE;            // Ack Recv
}

/*===========================================================================
    FUNCTION  CAMSENSOR_GPIO_SEND_NACK
===========================================================================*/
static boolean camsensor_gpio_send_ack(uint16_t len)
{
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

    mask = 0x80;
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

    if(SCL_IsHigh() == FALSE)
    {   
        CDBG("gpioI2C send_byte SCL Error !\n");
        return FALSE;
    }
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    mask >>= 1;
    while( mask )
    {
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

    // SDA Hige (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    if(SCL_IsHigh() == FALSE)
    {   
        CDBG("gpioI2C recv_byte SCL Error !\n");
        return FALSE;
    }
    Wait(3);

    sda = SDA_Read();
    if (sda)
        data |= mask;

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    mask >>= 1;
    while(mask)
    {
        // SCL High (Pull-UP)
        SCL_High();
        Wait(2);

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
    uint8_t  RegHi = (uint8_t)(pI2CCmd->offset >> 8);
    uint8_t  RegLo = (uint8_t)pI2CCmd->offset;
    uint16_t len;
    uint8_t  *pd;
    char    tszDump[32] = "";
    int     idx = 0;

    // 1.Start Condition
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;

    // 2.SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(pI2CCmd->slave_addr)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_write. fault Slave addr\n");
        CDBG("gpioI2C_write. fault Slave addr\n");
        goto fault;
    }

    // 3.SEND - Sub Address(Virtual Address)
    if( !(rc = camsensor_gpio_send_byte(RegHi)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_write. fault Reg Addr Hi\n");
        CDBG("gpioI2C_write. fault Reg Addr Hi\n");
        goto fault;
    }

    // 4.SEND - Sub Address(Virtual Address)
    if( !(rc = camsensor_gpio_send_byte(RegLo)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_write. fault Reg Addr Lo\n");
        CDBG("gpioI2C_write. fault Reg Addr Lo\n");
        goto fault;
    }

    // 5.SEND - Data
    for(len = pI2CCmd->len, pd = pI2CCmd->buf_ptr ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_send_byte(*pd)) )    goto fault;
        // Chk Acknowledge
        if( !(rc = camsensor_gpio_chk_ack()) )
        {
            //printk(KERN_INFO "gpioI2C_write. fault Data\n");
            CDBG("gpioI2C_write. fault Data\n");
            goto fault;
        }
    }

    // 6.Stop Condition
    camsensor_gpio_stop_condition();

    for(len = 0, pd = pI2CCmd->buf_ptr ; len < pI2CCmd->len && len < 4; ++len, ++pd)
    {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }

    goto exit;

fault : ;
    camsensor_gpio_stop_condition();

exit : ;
    if (pI2CCmd->slave_addr != 0x99 || !rc)
        CDBG("gpioI2C_write. S:0x%02X A:0x%02X%02X, D:%s (%s)\n",
             pI2CCmd->slave_addr, RegHi, RegLo, tszDump, rc ? "Ok" : "Error");
    return rc;
}

/*===========================================================================
    FUNCTION      CAMSENSOR_GPIOI2C_READ
===========================================================================*/
int camsensor_gpioi2c_read(struct CameraSensorI2CCmdType *pI2CCmd)
{
    int rc = TRUE;
    uint8_t  RegHi = (uint8_t)(pI2CCmd->offset >> 8);
    uint8_t  RegLo = (uint8_t)pI2CCmd->offset;
    uint16_t len;
    uint8_t  *pd;
    char    tszDump[32] = "";
    int     idx = 0;

    // 1.Start Condition
    if( !(rc = camsensor_gpio_start_condition()) )  goto fault;

    // 2. SEND - Slave Address(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(pI2CCmd->slave_addr)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_read. fault Slave[W] addr\n");
        CDBG("gpioI2C_read. fault Slave[W] addr\n");
        goto fault;
    }

    // 3. SEND - Sub Address(Virtual Address)
    if( !(rc = camsensor_gpio_send_byte(RegHi)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_read. fault Reg addr Hi\n");
        CDBG("gpioI2C_read. fault Reg addr Hi\n");
        goto fault;
    }

    // 4. SEND - Sub Address(Virtual Address)
    if( !(rc = camsensor_gpio_send_byte(RegLo)) ) goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_read. fault Reg addr Lo\n");
        CDBG("gpioI2C_read. fault Reg addr Lo\n");
        goto fault;
    }

    // 5.Restart Condition
    if( !(rc = camsensor_gpio_restart_condition()) )    goto fault;

    // 6.SEND - Slave Address again(Camera Device)
    if( !(rc = camsensor_gpio_send_byte(pI2CCmd->slave_addr | 1)) )  goto fault;
    // Chk Acknowledge
    if( !(rc = camsensor_gpio_chk_ack()) )
    {
        //printk(KERN_INFO "gpioI2C_read. fault Slave[R] addr\n");
        CDBG("gpioI2C_read. fault Slave[R] addr\n");
        goto fault;
    }

    // 7.RCV - Data
    for(len = pI2CCmd->len, pd = pI2CCmd->buf_ptr ; len > 0 ; --len, ++pd)
    {
        if( !(rc = camsensor_gpio_recv_byte(pd)) )    goto fault;
        // Send Ack
        if( !(rc = camsensor_gpio_send_ack(len-1)) )
        {
            //printk(KERN_INFO "gpioI2C_read. fault ack \n");
            CDBG("gpioI2C_read. fault ack \n");
            goto fault;
        }
    }

    // 8.Stop Condition
    camsensor_gpio_stop_condition();

    for(len = 0, pd = pI2CCmd->buf_ptr ; len < pI2CCmd->len && len < 4; ++len, ++pd)
    {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }

    goto exit;

fault : ;
    camsensor_gpio_stop_condition();

exit : ;

    //printk(KERN_INFO "gpioI2C_read S:0x%02X A:0x%02X%02X D:%s (%s)\n", 
    //         pI2CCmd->slave_addr, RegHi, RegLo, tszDump, rc ? "Ok" : "Error");
    CDBG("gpioI2C_read S:0x%02X A:0x%02X%02X D:%s (%s)\n", 
             pI2CCmd->slave_addr, RegHi, RegLo, tszDump, rc ? "Ok" : "Error");
    return rc;
}
