/*
  IRDA Remote Chipset Driver
  COPYRIGHT(C) FUJITSU TOSHIBA MOBILE COMMUNICATIONS LIMITED 2011

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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/i2c.h>
#include <mach/vreg.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/delay.h>
#include "remodrvi.h"
#include "sub_pmic.h"

static int remodrv_major = 0;
module_param(remodrv_major, int, 0);
static struct class* remodrv_class;

static unsigned int g_remodrv_irq;

static struct work_struct g_remodrv_work_data;

unsigned char saved_parameter[6];
struct i2c_adapter *i2c_remodrv;

DEFINE_SPINLOCK(remodrv_slock);

atomic_t g_remodrv_flag;
atomic_t g_power_on;

struct mutex remodrv_mutex;

static int g_total_bitlen = 0;
static int g_send_bytes = 0;
static int g_int_cnt = 0;
static atomic_t g_continue;
static atomic_t g_atomic_sendc;


static atomic_t now_processing;
static atomic_t end_processing;
static atomic_t power_off_status;

static struct workqueue_struct *keventird_wq;
static struct delayed_work pwr_data_work;


static atomic_t device_open_cnt;


static struct send_data g_now_data;
static struct prof_data g_now_prof;
static struct final_prof remodrv_carrier_data;

static struct delayed_work data_work;

static struct send_data_list g_data_list;

static int remodrv_send_data(int bitlen);
static int remodrv_power_on(void);
static int remodrv_power_off(void);
static void remodrv_data_pre_send(void);


/* ----------------------------------------------------------------------

* ---------------------------------------------------------------------- */



/*----------------------------------------------------------------------

*----------------------------------------------------------------------*/

int remodrv_i2c_write(uint16_t reg, uint8_t *data, uint32_t len)
{
  struct i2c_msg msg;
  u_int8_t buf[64];
  int ret = 0;


  msg.addr = REMODRV_SLAVE_ADDR;
  msg.flags = 0;
  buf[0] = reg;
  memcpy(&buf[1],data,len);

  msg.buf  = buf;
  msg.len  = len + 1;
  ret = i2c_transfer(i2c_remodrv, &msg, 1);


  return ret;
}

/* ----------------------------------------------------------------------

* ---------------------------------------------------------------------- */
static int remodrv_i2c_read(uint16_t reg, uint8_t *data, uint32_t len)
{
  struct i2c_msg msg[2];
  u_int8_t msgbuf[2];
  int ret = 0;


  memcpy(msgbuf, &reg, sizeof(reg));

  msg[0].addr  = REMODRV_SLAVE_ADDR;
  msg[0].flags = 0;
  msg[0].buf   = msgbuf;
  msg[0].len   = 1;

  msg[1].addr  = REMODRV_SLAVE_ADDR;
  msg[1].flags = I2C_M_RD;
  msg[1].buf   = data;
  msg[1].len   = len;

  ret = i2c_transfer(i2c_remodrv, msg, 2);

  return ret;
}


/* ----------------------------------------------------------------------

* ---------------------------------------------------------------------- */



static unsigned char byte_reverse(unsigned char byte)
{
	unsigned char result=0;
	result |= ((byte & 1) << (7)) | ((byte & 2) << (5)) | ((byte & 4) << (3)) | ((byte & 8) << (1)) | ((byte & 16) >> (1)) | ((byte & 32) >> (3)) | ((byte & 64) >> (5)) | ((byte & 128) >> (7));

	return result;
}

unsigned char remodrv_send_regs[16] = {0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};


static int remodrv_send_data(int bitlen)
{
	int loop_cnt=0;
	unsigned char wbuf=0;
	int result=0;
	unsigned char addr;
	unsigned short len;
	unsigned char data[16];
	int i=0;
	int sent_bits = 0;
    
    

	gpio_tlmm_config(GPIO_CFG(140, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
    gpio_set_value(140, 0);
    

	mutex_lock(&remodrv_mutex);
	

	memset(data, 0, sizeof(data));
	

	if((bitlen <= 128) && (!(atomic_read(&g_continue))))
	{
	

		len = 1;
		addr = REMODRV_REG_ADDR2B;
		wbuf = REMODRV_REGS_ONE;
		result = remodrv_i2c_write(addr, &wbuf, len);


		len = 1;
		addr = REMODRV_REG_ADDR15;
		wbuf = bitlen;
		result = remodrv_i2c_write(addr, &wbuf, len);

		if((bitlen % 8) == 0)
			loop_cnt = bitlen / 8;
		else
			loop_cnt = (bitlen / 8) + 1;

		for(i=0; i<loop_cnt; i++)
		{
			data[i] = byte_reverse(g_now_data.data[i]);
		}



		sent_bits = bitlen;
		g_total_bitlen = g_total_bitlen - sent_bits;

		for(i=0; i<loop_cnt; i++)
		{
      			result = remodrv_i2c_write(remodrv_send_regs[i], &data[i], 1); 
		}
       

		len = 1; 
		addr = REMODRV_REG_ADDR29;
		wbuf = REMODRV_SEND_START;
		result = remodrv_i2c_write(addr, &wbuf, len);

	}
	else
	{
		if(atomic_read(&g_continue))
		{
			if(bitlen <= 128)
			{

				len = 1;
				addr = REMODRV_REG_ADDR2B;
				wbuf = REMODRV_REGS_ONE;
				result = remodrv_i2c_write(addr, &wbuf, len);


				len = 1;
				addr = REMODRV_REG_ADDR15;
				wbuf = bitlen;
				result = remodrv_i2c_write(addr, &wbuf, len);

				if((bitlen % 8) == 0)
				loop_cnt = bitlen / 8;
				else
				loop_cnt = (bitlen / 8) + 1;

				sent_bits = bitlen;

			}
			else
			{

				len = 1;
				addr = REMODRV_REG_ADDR2B;
				wbuf = REMODRV_REGS_REPEAT;
				result = remodrv_i2c_write(addr, &wbuf, len);



				len = 1;
				addr = REMODRV_REG_ADDR15;
				wbuf = 128;
				result = remodrv_i2c_write(addr, &wbuf, len);

				loop_cnt = 16;
				sent_bits = 128;
			}

			for(i=0; i<loop_cnt; i++)
			{
				data[i] = byte_reverse(g_now_data.data[g_send_bytes++]);
			}


			for(i=0; i<loop_cnt; i++)
                	{
                        	result = remodrv_i2c_write(remodrv_send_regs[i], &data[i], 1);
                	}

			g_total_bitlen = g_total_bitlen - sent_bits;

			if(atomic_read(&g_atomic_sendc))
			{
				atomic_set(&g_atomic_sendc, 0);


				len = 1; 
				addr = REMODRV_REG_ADDR29;                                                           
				wbuf = REMODRV_SEND_START;        
				result = remodrv_i2c_write(addr, &wbuf, len);

			}

			
		}
	}
	mutex_unlock(&remodrv_mutex);

	return result;
}


static int remodrv_power_on(void)
{
	unsigned short len;
	unsigned char addr;   
	unsigned char wbuf = 0;
	unsigned long result = ERROR;
    
	if(!atomic_read(&power_off_status))
	{ 


		gpio_tlmm_config(GPIO_CFG(22, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
		gpio_configure(22, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		gpio_set_value(22, 0); 


		msleep(1);
		    
		gpio_set_value(22, 1); 
		gpio_configure(22, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);

		addr = REMODRV_REG_ADDR2A;
		len = 1;
		wbuf = REMODRV_RESET;
		if ((result = remodrv_i2c_write(addr, &wbuf, len))) {   
		}
		addr = REMODRV_REG_ADDR00;
		len = 1;
		 wbuf = REMODRV_ADDR00_TYPICAL;
		if ((result = remodrv_i2c_write(addr, &wbuf, len))) { 
		}

		atomic_set(&power_off_status, 1);
	    

		queue_delayed_work(keventird_wq, &data_work, msecs_to_jiffies(50) + 1);
	} 
	return result;
}

static int remodrv_power_off(void)
{

	if ((list_empty(&(g_data_list.list))) && (!atomic_read(&now_processing)) && (atomic_read(&end_processing))) {


		schedule_delayed_work(&pwr_data_work, msecs_to_jiffies(3000));
	}
	return 0;
}

static int remodrv_func_termination(void)
{
	unsigned short len;
	unsigned char addr, data[2];    
	int result = -1;




    disable_irq_nosync(g_remodrv_irq);
    

	cancel_delayed_work(&data_work); 

    
	addr = REMODRV_REG_ADDR00;
	len = 1;
	data[0] = REMODRV_PWR_OFF;
	result = remodrv_i2c_write(addr, data, len);
    
	gpio_tlmm_config(GPIO_CFG(22, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
	gpio_configure(22, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	gpio_set_value(22, 0); 
    

	gpio_tlmm_config(GPIO_CFG(140, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
    
	enable_irq(g_remodrv_irq);
	atomic_set(&power_off_status, 0);
	return result;
}





/* ----------------------------------------------------------------------

* ---------------------------------------------------------------------- */


 unsigned char remodrv_test_int_count;

static void
remodrv_work_bh(struct work_struct *work)
{
	unsigned short len;
	unsigned char addr, data[2];    
	int wbuf=0x02;
	int int_src=0;
	int tmp_total_bitlen;

	mutex_lock(&remodrv_mutex);
	g_int_cnt++;
	tmp_total_bitlen = g_total_bitlen;
	mutex_unlock(&remodrv_mutex);

	if(((g_int_cnt % 2) == 0) && (tmp_total_bitlen > 0))
	{
	    remodrv_send_data(g_total_bitlen);
	}

#ifdef REMO_DEBUG
	remodrv_func_debug((unsigned char *)"=RIS", NULL, 0);
#endif

	addr = REMODRV_REG_ADDR28;                  
	len = 1;                                    
	data[0] = REMODRV_IRQ_CLEAR;                
	remodrv_i2c_write(addr, data, len);   
	remodrv_test_int_count++;


	if(tmp_total_bitlen == 0)
	{
		atomic_set(&now_processing, 0);
		atomic_set(&end_processing, 1);
		if((g_int_cnt % 2) == 0)
		{
			remodrv_power_off();
		}
	}
	enable_irq(g_remodrv_irq);

}


static irqreturn_t
remodrv_irq_handler(int irq, void *p)
{
    disable_irq_nosync(g_remodrv_irq);
	schedule_work(&g_remodrv_work_data); 
	return IRQ_HANDLED;
}




static int
remodrv_request_irqs(void)
{
	int err;
	unsigned long req_flags = IRQF_TRIGGER_FALLING;
    
	gpio_configure(33, GPIOF_INPUT );
	gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

	err = g_remodrv_irq = gpio_to_irq(33);
	err = request_irq(g_remodrv_irq, remodrv_irq_handler,
	req_flags, "gpio_remote", NULL);
	if (err) {
		return err;
	}
	return 0;

}



static int remodrv_open (struct inode *inode, struct file *filp)
{

	return 0;
}

static int remodrv_write(struct file * file, const char * buff, size_t count, loff_t *pos)
{
  return 0;
}


static int remodrv_read(struct file * file, char * buff, size_t count, loff_t *pos)
{


  return 0;
}

static int remodrv_release(struct inode *inode, struct file *filp)
{

#if 0
        atomic_dec(&device_open_cnt);
        if(!atomic_read(&device_open_cnt))
        {
           cancel_work_sync(&g_remodrv_work_data);         //interrupt workqueue
           cancel_delayed_work(&data_work);                           //data processing workqueue
           cancel_delayed_work(&pwr_data_work);                       //power-off workqueue
           destroy_workqueue(keventird_wq);
           free_irq(g_remodrv_irq, NULL);
        }



#endif
  return 0;

}


unsigned char remodrv_carrier_regs[21] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14};

unsigned long remodrv_makesend_carrier_data(void)
{
	unsigned short len;
	unsigned char addr, iwork;
	unsigned char data[23];  
	int result = 0;
	int i = 0;

	len = 0;



	memset(data, 0, sizeof(data));

	
	
	iwork = REMODRV_ADDR00_TYPICAL;                   
	switch (g_now_prof.isOffHighLow) {        
	case IRFR_REMO_OUTPUT_PPM_HIGH_TO_LOW:          
		iwork |= REMODRV_INV1_HI + REMODRV_INV0_HI;
		break;
	case IRFR_REMO_OUTPUT_PPM_LOW_TO_HIGH:          
		iwork |= REMODRV_INV1_LO + REMODRV_INV0_LO;
		break;
	case IRFR_REMO_OUTPUT_MANCHESTER:           
		iwork |= REMODRV_INV1_LO + REMODRV_INV0_HI;
		break;
	default:
		break;
	}
	data[len++] = iwork;                 


	
	if(g_now_prof.repeatCount > 1)
	iwork = REMODRV_ADDR01_TYPICAL;               
	else
	iwork = 0x00;									

	iwork |= 0x01;                          
	data[len++] = iwork;                    

	
	data[len++] = REMODRV_BASE_DIV_RATIO;

	
	data[len++] = (unsigned char)((remodrv_carrier_data.carrier_lo >> 8) & 0x0001);
	data[len++] = (unsigned char)(remodrv_carrier_data.carrier_lo & 0x00FF);

	
	data[len++] = (unsigned char)((remodrv_carrier_data.carrier_hi >> 8) & 0x0001);
	data[len++] = (unsigned char)(remodrv_carrier_data.carrier_hi & 0x00FF);

	
	data[len++] = (unsigned char)((remodrv_carrier_data.header_lo >> 8) & 0x003F);
	data[len++] = (unsigned char)(remodrv_carrier_data.header_lo & 0x00FF);

	
	data[len++] = (unsigned char)((remodrv_carrier_data.header_hi >> 8) & 0x003F);
	data[len++] = (unsigned char)(remodrv_carrier_data.header_hi & 0x00FF);

	switch (g_now_prof.isOffHighLow) {

	case IRFR_REMO_OUTPUT_PPM_HIGH_TO_LOW:          
		
		data[len++] = (unsigned char)((remodrv_carrier_data.data0_lo >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data0_lo & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data0_hi >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data0_hi & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data1_lo >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data1_lo & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data1_hi >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data1_hi & 0x00FF);
		break;

	case IRFR_REMO_OUTPUT_PPM_LOW_TO_HIGH:          
		
		data[len++] = (unsigned char)((remodrv_carrier_data.data0_hi >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data0_hi & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data0_lo >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data0_lo & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data1_hi >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data1_hi & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data1_lo >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data1_lo & 0x00FF);
		break;

	case IRFR_REMO_OUTPUT_MANCHESTER:            
		
		data[len++] = (unsigned char)((remodrv_carrier_data.data0_hi >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data0_hi & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data0_lo >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data0_lo & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data1_lo >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data1_lo & 0x00FF);

		
		data[len++] = (unsigned char)((remodrv_carrier_data.data1_hi >> 8) & 0x003F);
		data[len++] = (unsigned char)(remodrv_carrier_data.data1_hi & 0x00FF);
		break;

	default:
		break;
	}

	
	data[len++] = (unsigned char)((remodrv_carrier_data.end_section >> 8) & 0x003F);
	data[len++] = (unsigned char)(remodrv_carrier_data.end_section & 0x00FF);



	for(i=0; i<len; i++)
	{
		result = remodrv_i2c_write(remodrv_carrier_regs[i], &data[i], 1);

	}

    
	addr = REMODRV_REG_ADDR16;
	len = 1;
    data[0] = (unsigned char)((remodrv_carrier_data.frame_interval >> 8) & 0x00FF);
	result = remodrv_i2c_write(addr, &data[0], 1);

	addr = REMODRV_REG_ADDR17;
	len = 1;
	data[1] = (unsigned char)(remodrv_carrier_data.frame_interval & 0x00FF);

   
    result = remodrv_i2c_write(addr, &data[1], 1);

  return OK;
}

unsigned long remodrv_get_clock_count(unsigned char type, unsigned long time, unsigned long ch_time, unsigned long cl_time)
{
	unsigned long ulwork=0;

	unsigned long ulwork_LSB=0;
	unsigned long ulwork_MSB=0;

	switch (type)
	{

		case REMODRV_COUNT_CARRIER:											
			ulwork_MSB = (unsigned long)(time * (REMODRV_SYSTEM_CLOCK / 1000));					
			ulwork_MSB = (unsigned long)(ulwork_MSB / 2560);															

			ulwork_LSB = (unsigned long)(time * (REMODRV_SYSTEM_CLOCK / 1000)) / 10;					
			ulwork_LSB = (unsigned long)(ulwork_LSB % 256);															
			ulwork = (ulwork_MSB << 8) | ulwork_LSB;

			break;



		case REMODRV_COUNT_DATA:											
			ulwork_MSB = (unsigned long)(((time * 10) / (ch_time + cl_time)) / 256);					

			ulwork_LSB = (unsigned long)(((time * 10) / (ch_time + cl_time)) % 256);
			ulwork = (ulwork_MSB << 8) | ulwork_LSB;

			break;

		default:
			ulwork = 0;
			break;
	}

	return  ulwork;
}


unsigned long remodrv_func_set_carrier_data(void)
{
  unsigned long ulreturn = OK;
  unsigned long ulwork;

  unsigned long carr_high = g_now_prof.carrierHighDuration;
  unsigned long carr_low = g_now_prof.carrierLowDuration;



  ulwork = remodrv_get_clock_count(REMODRV_COUNT_CARRIER, g_now_prof.carrierLowDuration, 0, 0);
  remodrv_carrier_data.carrier_lo = (unsigned short)ulwork;



  ulwork = remodrv_get_clock_count(REMODRV_COUNT_CARRIER, g_now_prof.carrierHighDuration, 0, 0);
  remodrv_carrier_data.carrier_hi = (unsigned short)ulwork;

 
  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.startbitLowDuration, carr_high, carr_low);
  remodrv_carrier_data.header_lo = (unsigned short)ulwork;


  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.startbitHighDuration, carr_high, carr_low);
  remodrv_carrier_data.header_hi = (unsigned short)ulwork;


  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.offLowDuration, carr_high, carr_low);
  remodrv_carrier_data.data0_lo = (unsigned short)ulwork;


  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.offHighDuration, carr_high, carr_low);
  remodrv_carrier_data.data0_hi = (unsigned short)ulwork;


  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.onLowDuration, carr_high, carr_low);
  remodrv_carrier_data.data1_lo = (unsigned short)ulwork;


  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.onHighDuration, carr_high, carr_low);
  remodrv_carrier_data.data1_hi = (unsigned short)ulwork;


  ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.stopbitHighDuration, carr_high, carr_low);
  remodrv_carrier_data.end_section = (unsigned short)ulwork;




 ulwork = remodrv_get_clock_count(REMODRV_COUNT_DATA, g_now_prof.repeatInterval, carr_high, carr_low);
 remodrv_carrier_data.frame_interval = (unsigned short)ulwork;


  return ulreturn;
}





static int
remodrv_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	int ret = OK;
	struct send_data *d;
	struct prof_data *pd;
	struct send_data_list *tmp;

	switch (cmd) {
	case IOCTL_REMODRV_POWER_ON:
		remodrv_power_on();
		break;
	  
	case IOCTL_REMODRV_POWER_OFF:

		break;
	  
	case IOCTL_REMODRV_SEND_DATA:
		mutex_lock(&remodrv_mutex); //LAL
		tmp = kmalloc(sizeof(struct send_data_list), GFP_KERNEL);

		d = (struct send_data *)arg;
		memcpy(tmp->data, d->data, 128 * sizeof(unsigned char));
		tmp->bitlen = d->bitlen;

		list_add_tail(&(tmp->list), &(g_data_list.list));
		mutex_unlock(&remodrv_mutex);


	    schedule_delayed_work(&pwr_data_work, msecs_to_jiffies(5000));
		break;

	case IOCTL_REMODRV_SET_PROFILE:
		pd = (struct prof_data *)arg;
		g_now_prof = *pd;
		remodrv_func_set_carrier_data();	
		remodrv_makesend_carrier_data();	
		break;
	  
	default:
		break;
	}


  return ret;
}


static void remodrv_data_pre_send(void)
{

	struct send_data_list *pd;

	g_send_bytes = 0;
    

	mutex_lock(&remodrv_mutex);


	if (list_empty(&(g_data_list.list))) {
		mutex_unlock(&remodrv_mutex);
		queue_delayed_work(keventird_wq, &data_work, msecs_to_jiffies(REMODRV_THREAD_POOLING) + 1);
		return;
	}
	else
	{
		if(((atomic_read(&now_processing) == 0) && (atomic_read(&end_processing) == 0)) || ((atomic_read(&now_processing) == 0) && (atomic_read(&end_processing) == 1)))
		{

			atomic_set(&now_processing, 1);
			atomic_set(&end_processing, 0);

			pd = list_first_entry(&(g_data_list.list), struct send_data_list, list);

			memcpy(&(g_now_data.data),&(pd->data), 128 * sizeof(unsigned char));
			g_now_data.bitlen = pd->bitlen;

			list_del(&(pd->list));

			kfree(pd);			

			g_total_bitlen = g_now_data.bitlen;


			g_int_cnt = 0;
			
			mutex_unlock(&remodrv_mutex);

			if(g_now_data.bitlen > 128)
			{
				atomic_set(&g_continue,1);
				atomic_set(&g_atomic_sendc,1);
			}
            remodrv_send_data(g_now_data.bitlen);


			queue_delayed_work(keventird_wq, &data_work, msecs_to_jiffies(REMODRV_THREAD_POOLING) + 1);
		}
		else	
		{
			mutex_unlock(&remodrv_mutex);
			queue_delayed_work(keventird_wq, &data_work, msecs_to_jiffies(REMODRV_THREAD_POOLING) + 1);
		}
	}
}





static void remodrv_setup_cdev(struct cdev *dev, int minor,
    struct file_operations *fops)
{
	int err, devno = MKDEV(remodrv_major, minor);
	  
	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	dev->ops = fops;
	err = cdev_add (dev, devno, 1);
    IS_ERR(device_create(remodrv_class, NULL, devno, NULL, "remodrv"));
}


static int
remote_probe(struct platform_device *pdev)
{

return 0;
}


static int
remote_remove(struct platform_device *pdev)
{
return 0;
}

static struct platform_driver remote_driver = {
	.probe      = remote_probe,
	.remove     = remote_remove,
	.driver = {
		.name   = "remodrv",
		.owner  = THIS_MODULE,
	},
};


static struct file_operations remodrv_ops = {
  .owner   = THIS_MODULE,
  .open    = remodrv_open,
  .ioctl    = remodrv_ioctl,
  .release = remodrv_release,
  .write   = remodrv_write,
  .read    = remodrv_read,
};

#define MAX_remodrv_DEV 1


static struct cdev remodrvDevs[MAX_remodrv_DEV];


static int remodrv_init(void)
{
	int result = 0;

	dev_t dev = MKDEV(remodrv_major, 0);
	atomic_set(&g_remodrv_flag,0);

	mutex_init(&remodrv_mutex);
	atomic_set(&g_atomic_sendc,0);
	atomic_set(&g_continue,0);
	atomic_set(&now_processing,0);
	atomic_set(&end_processing,0);
	atomic_set(&power_off_status,0);
	atomic_set(&device_open_cnt,0);
	atomic_set(&g_power_on,0);

	remodrv_class = class_create(THIS_MODULE, "remodrv");
	if(IS_ERR(remodrv_class))
	{
		return PTR_ERR(remodrv_class);
	}
	

	if (remodrv_major)
	{
		result = register_chrdev_region(dev, 2, "remodrv");
	}
	else
	{
		result = alloc_chrdev_region(&dev, 0, 2, "remodrv");
		remodrv_major = MAJOR(dev);
	}
	if (result < 0) {
		return result;
	}
	if (remodrv_major == 0)
		remodrv_major = result;
	

	remodrv_setup_cdev(remodrvDevs, 0, &remodrv_ops);

	platform_driver_register(&remote_driver);


	i2c_remodrv = i2c_get_adapter(0);
    

	keventird_wq = create_workqueue("keventsIRDA");
	INIT_WORK(&g_remodrv_work_data, remodrv_work_bh); 
	remodrv_request_irqs(); 
	INIT_LIST_HEAD(&g_data_list.list);
	INIT_DELAYED_WORK(&data_work, remodrv_data_pre_send);
	INIT_DELAYED_WORK(&pwr_data_work, remodrv_func_termination);
	atomic_set(&g_remodrv_flag,1);

	return 0;
}


static void remodrv_cleanup(void)
{


	if(atomic_read(&g_remodrv_flag))
	{  
		cancel_work_sync(&g_remodrv_work_data); 
		cancel_delayed_work(&data_work);    
		cancel_delayed_work(&pwr_data_work);
		destroy_workqueue(keventird_wq);
		free_irq(g_remodrv_irq, NULL);
	}


	cdev_del(remodrvDevs);
	unregister_chrdev_region(MKDEV(remodrv_major, 0), 2);
	device_destroy(remodrv_class, MKDEV(remodrv_major, 0));
	class_destroy(remodrv_class);
	platform_driver_unregister(&remote_driver);
}

module_init(remodrv_init);
module_exit(remodrv_cleanup);


MODULE_AUTHOR("TOSHIBA");
MODULE_DESCRIPTION("TG03 IRDA Remote");

/*Module license is GPL since we are using some of the GPL only symbols
and Linux does not allow GPL only symbols in non-GPL modules
*/

MODULE_LICENSE("GPL");
