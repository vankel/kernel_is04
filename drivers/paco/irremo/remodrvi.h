/*
  IRDA Remote Chipset Driver
  This is protocol driver for IRDA Remote Chipset
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

#include <linux/list.h>

#define OK 0
#define ERROR -1

  #define REMODRV_DATA_CARRIER_MAX    266     
#define REMODRV_DATA_START_MIN      0       
#define REMODRV_DATA_START_MAX      668000  
#define REMODRV_DATA_DATAOFF_MIN    20     
#define REMODRV_DATA_DATAOFF_MAX    668000  
#define REMODRV_DATA_DATAON_MIN     20      
#define REMODRV_DATA_DATAON_MAX     668000  
#define REMODRV_DATA_STOP_MIN       0       
#define REMODRV_DATA_STOP_MAX       668000  
#define REMODRV_DATA_CARRIER_FREQUENCY_MIN  193   
#define REMODRV_DATA_CARRIER_FREQUENCY_MAX  408   
#define REMODRV_DATA_REAPET_INTERVAL_MIN    0     
#define REMODRV_DATA_REAPET_INTERVAL_MAX    2674800 
#define REMODRV_DATA_REPEAT_COUNT_MIN       1    
#define REMODRV_DATA_REPEAT_COUNT_MAX       (0xFFFFFFFF) 
#define REMODRV_DATA_DATASIZE_MIN   0       
#define REMODRV_DATA_DATASIZE_MAX   1024    


#define REMODRV_MODE_STANDBY     0    
#define REMODRV_MODE_REMOCON     1   



  #define REMODRV_SYSTEM_CLOCK  19200 /* 19.2MHz */




#define REMODRV_SLAVE_ADDR      (0xEE >> 1)  /* Modified as i2c address is 7 bits    */
                                        
#define REMODRV_SLAVE_WRITE     (REMODRV_SLAVE_ADDR + 0x0000)
#define REMODRV_SLAVE_READ      (REMODRV_SLAVE_ADDR + 0x0001)
#define REMODRV_OPERATION_WRITE 0x0000


#define RC5T_SLAVE_ADDR      (0x34 >> 1)  /* RC5T7710 slave address    */


/* Register */
#define REMODRV_REG_ADDR00    0x00    /* D5 : Opm                     */
                                      /* D4 : Divs                        */
                                      /* D3 : Irqe                    */
                                      /* D2 : Inv1     */
                                      /* D1 : Inv0     */
                                      /* D0 : Pwr              */
#define REMODRV_REG_ADDR01    0x01    /* D5 : Frmb         */
                                      /* D4 : Frme              */
                                      /* D3-D0 : Rpt            */
#define REMODRV_REG_ADDR02    0x02    /* D7-D0 : Base         */
#define REMODRV_REG_ADDR03    0x03    /* D0    : Clo1     */
#define REMODRV_REG_ADDR04    0x04    /* D7-D0 : Clo0    */
#define REMODRV_REG_ADDR05    0x05    /* D0    : Chi1    */
#define REMODRV_REG_ADDR06    0x06    /* D7-D0 : Chi0      */
#define REMODRV_REG_ADDR07    0x07    /* D5-D0 : Hlo1      */
#define REMODRV_REG_ADDR08    0x08    /* D7-D0 : Hlo0    */
#define REMODRV_REG_ADDR09    0x09    /* D5-D0 : Hhi1    */
#define REMODRV_REG_ADDR0A    0x0A    /* D7-D0 : Hhi0    */
#define REMODRV_REG_ADDR0B    0x0B    /* D5-D0 : D0lo1    */
#define REMODRV_REG_ADDR0C    0x0C    /* D7-D0 : D0lo0    */
#define REMODRV_REG_ADDR0D    0x0D    /* D5-D0 : D0hi1  */
#define REMODRV_REG_ADDR0E    0x0E    /* D7-D0 : D0hi1  */
#define REMODRV_REG_ADDR0F    0x0F    /* D5-D0 : D1lo1    */
#define REMODRV_REG_ADDR10    0x10    /* D7-D0 : D1lo0    */
#define REMODRV_REG_ADDR11    0x11    /* D5-D0 : D1hi1  */
#define REMODRV_REG_ADDR12    0x12    /* D7-D0 : D1hi1  */
#define REMODRV_REG_ADDR13    0x13    /* D5-D0 : EndLen1    */
#define REMODRV_REG_ADDR14    0x14    /* D7-D0 : EndLen0    */
#define REMODRV_REG_ADDR15    0x15    /* D7-D0 : BitLen         */
#define REMODRV_REG_ADDR16    0x16    /* D7-D0 : FrmLen1     */
#define REMODRV_REG_ADDR17    0x17    /* D7-D0 : FrmLen0   */
#define REMODRV_REG_ADDR18    0x18    /* D7-D0 : Out0             */
#define REMODRV_REG_ADDR19    0x19    /* D7-D0 : Out1            */
#define REMODRV_REG_ADDR1A    0x1A    /* D7-D0 : Out2             */
#define REMODRV_REG_ADDR1B    0x1B    /* D7-D0 : Out3           */
#define REMODRV_REG_ADDR1C    0x1C    /* D7-D0 : Out4        */
#define REMODRV_REG_ADDR1D    0x1D    /* D7-D0 : Out5         */
#define REMODRV_REG_ADDR1E    0x1E    /* D7-D0 : Out6         */
#define REMODRV_REG_ADDR1F    0x1F    /* D7-D0 : Out7         */
#define REMODRV_REG_ADDR20    0x20    /* D7-D0 : Out8             */
#define REMODRV_REG_ADDR21    0x21    /* D7-D0 : Out9          */
#define REMODRV_REG_ADDR22    0x22    /* D7-D0 : Out10          */
#define REMODRV_REG_ADDR23    0x23    /* D7-D0 : Out11          */
#define REMODRV_REG_ADDR24    0x24    /* D7-D0 : Out12            */
#define REMODRV_REG_ADDR25    0x25    /* D7-D0 : Out13         */
#define REMODRV_REG_ADDR26    0x26    /* D7-D0 : Out14          */
#define REMODRV_REG_ADDR27    0x27    /* D7-D0 : Out15            */
#define REMODRV_REG_ADDR28    0x28    /* D0 : Irqc       */
#define REMODRV_REG_ADDR29    0x29    /* D0 : Send                    */
#define REMODRV_REG_ADDR2A    0x2A    /* D0 : Rst                      */
#define REMODRV_REG_ADDR2B    0x2B    /* D0 : Regs           */

#define REMODRV_OPM_BUFF_EMPTY   0x00  
#define REMODRV_OPM_DATA_TRANS   0x20  
#define REMODRV_DIVS_CARRY       0x00  
#define REMODRV_DIVS_BASE        0x10  
#define REMODRV_IRQ_MSK          0x00  
#define REMODRV_IRQ_ENB          0x08  
#define REMODRV_INV1_HI          0x00  
#define REMODRV_INV1_LO          0x04  
#define REMODRV_INV0_HI          0x00  
#define REMODRV_INV0_LO          0x02  
#define REMODRV_PWR_OFF          0x00  
#define REMODRV_PWR_ON           0x01  
#define REMODRV_ADDR00_TYPICAL    (REMODRV_OPM_DATA_TRANS + REMODRV_DIVS_CARRY + REMODRV_IRQ_ENB + REMODRV_PWR_ON)
#define REMODRV_FRMB_HEAD        0x00  
#define REMODRV_FRMB_END         0x20  
#define REMODRV_FRME_CTRL_OFF    0x00 
#define REMODRV_FRME_CTRL_ON     0x10  
#define REMODRV_RPT_BIT          0x0F 
#define REMODRV_ADDR01_TYPICAL   (REMODRV_FRMB_HEAD + REMODRV_FRME_CTRL_ON)

#define REMODRV_BASE_DIV_RATIO   0x00  

#define REMODRV_IRQ_CLEAR        0x01  

#define REMODRV_SEND_START       0x01  

#define REMODRV_RESET            0x01  

#define REMODRV_REGS_ONE         0x00  
#define REMODRV_REGS_REPEAT      0x01  


#define REMODRV_COUNT_TYPE_SYSTEM1     0  
#define REMODRV_COUNT_TYPE_SYSTEM2     1  
#define REMODRV_COUNT_TYPE_CARRIER1    2  
#define REMODRV_COUNT_TYPE_CARRIER2    3  
#define REMODRV_COUNT_CARRIER		   4
#define REMODRV_COUNT_DATA			   5


#define REMODRV_ENDTYPE_SUCCESS    0x00  
#define REMODRV_ENDTYPE_FAILURE    0x01  
#define REMODRV_ENDTYPE_STOP       0x02  


#define REMODRV_MAKETYPE_FIRST     0x00  
#define REMODRV_MAKETYPE_NEXT      0x01  
#define REMODRV_MAKETYPE_REPEAT    0x02  


#define REMODRV_DATACHK_CARRIER_LOW    0    
#define REMODRV_DATACHK_CARRIER_HIGH   1    
#define REMODRV_DATACHK_START_LOW      2    
#define REMODRV_DATACHK_START_HIGH     3    
#define REMODRV_DATACHK_DATAOFF_LOW    4    
#define REMODRV_DATACHK_DATAOFF_HIGH   5    
#define REMODRV_DATACHK_DATAON_LOW     6    
#define REMODRV_DATACHK_DATAON_HIGH    7    
#define REMODRV_DATACHK_STOP_HIGH      8    
#define REMODRV_DATACHK_REPEAT_LOW     9    


/*----------------------------------------------------------------------*/

#define REMODRV_FRAME_LEN_START_TO_START  0  
#define REMODRV_FRAME_LEN_END_TO_START    1  




#define REMODRV_TIMEOUT    5000    
#define REMODRV_THREAD_POOLING    100


#define REMODRV_NEXT       1

#define IOCTL_REMODRV_SEND_DATA 1
/* WPTF_IR_FRDCM_PRI_1064 START */
#define IOCTL_REMODRV_SET_PROFILE 6
/* WPTF_IR_FRDCM_PRI_1064 END */
#define IOCTL_REMODRV_POWER_ON 3
#define IOCTL_REMODRV_POWER_OFF 4

//To be set from userspace by Aniruddha for sending actual data
struct send_data
{
	unsigned char data[128];
	int bitlen;
};

struct send_data_list
{
	unsigned char data[128];
	int bitlen;
	struct list_head list;
};

typedef enum {
    IRFR_REMO_OUTPUT_PPM_HIGH_TO_LOW,
    IRFR_REMO_OUTPUT_PPM_LOW_TO_HIGH,
    IRFR_REMO_OUTPUT_MANCHESTER
} irfr_remo_offhighlow_enum;

//To be set from userspace by Aniruddha for sending the carrier settings
struct prof_data{
  unsigned long startbitHighDuration; 
  unsigned long startbitLowDuration;  
  unsigned long stopbitHighDuration;  
  irfr_remo_offhighlow_enum isOffHighLow;
  unsigned long offHighDuration;      
  unsigned long offLowDuration;  
  unsigned long onHighDuration;
  unsigned long onLowDuration;
  unsigned long carrierHighDuration;
  unsigned long carrierLowDuration;
  unsigned long repeatInterval;
  unsigned long repeatCount;
};


struct final_prof{
  unsigned short  carrier_lo;       
  unsigned short  carrier_hi;       
  unsigned short  header_lo;        
  unsigned short  header_hi;        
  unsigned short  data0_lo;         
  unsigned short  data0_hi;         
  unsigned short  data1_lo;         
  unsigned short  data1_hi;         
  unsigned short  end_section;      
  unsigned short  frame_interval;   
};

struct power_prof{
int on;
};
