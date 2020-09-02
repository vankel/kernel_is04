/*
  Accelerometer Driver
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


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/adxl345.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"

#if 0 /* DEBUG ----> */
#define DBG_MSG printk
#else
#define DBG_MSG(...) do{}while(0)
#endif /* <---- DEBUG */

#define BMA150_CMD_READ                  0x80
#define BMA150_REG_CHIPID                0x00
#define BMA150_REG_ACCX_LS               0x02
#define BMA150_REG_CONTROL_0A            0x0A
#define BMA150_REG_CONTROL_0B            0x0B
#define BMA150_REG_ANY_MOTION_THRESH     0x10
#define BMA150_REG_WIDTH_BANDW           0x14
#define BMA150_REG_CONTROL_15            0x15
#define BMA150_LAST_REG                  0x15

#define BMA150_REG_C0A_RESET_INT         0x40
#define BMA150_REG_C0A_SLEEP             0x01

#define BMA150_REG_C0B_ANY_MOTION        0x40
#define BMA150_REG_C0B_ENABLE_HG         0x02
#define BMA150_REG_C0B_ENABLE_LG         0x01

#define BMA150_REG_WID_BANDW_MASK        0x07

#define BMA150_REG_C15_SPI4              0x80
#define BMA150_REG_C15_EN_ADV_INT        0x40
#define BMA150_REG_C15_NEW_DATA_INT      0x20
#define BMA150_REG_C15_LATCH_INT         0x10

#define BMA150_BANDW_INIT                0x04
#define BMA150_ANY_MOTION_INIT           0x02

/* temperature offset of -30 degrees in units of 0.5 degrees */
#define BMA150_TEMP_OFFSET               60

#define ADXL345_NAME                      "adxl345"
#define ADXL345_NAME_FORMAT               "adxl345-%d.%d"
#define ADXL345_DEVICE_NAME               "/dev/adxl345-%d.%d"
#define ADXL345_VENDORID                  0x0001
#define DEG_LEFT_SHIFT_90				  1
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("adxl345");

/* 
 * Data structure for  mesuring the data for calibration.
 */
#define GSCALC_DATA_NUM		(20)

typedef struct {
	   int x[3];
} Adxl_data_t;

typedef struct {
	int  gCalculatorIndex;
	int gCalculatorNum; 
	Adxl_data_t gCalculatorData[GSCALC_DATA_NUM];
}Adxl_Gs_Data_t;
static Adxl_Gs_Data_t AdxlGs_Data;


/*
 * Data returned from accelerometer.
 * Temp is in units of 0.5 degrees C
 */
struct adxl345_accel_data {
	int              accel_x;
	int              accel_y;
	int              accel_z;
	int              temp;
};

struct driver_data {
	struct input_dev         *ip_dev;
	struct spi_device        *spi;
	char                      rx_buf[16];
	char                      bits_per_transfer;
	struct work_struct        work_data;
	bool                      config;
	struct list_head          next_dd;
	struct dentry            *dbfs_root;
	struct dentry            *dbfs_bpw;
	struct dentry            *dbfs_regs;
	#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend     e_sus_fcn;
	#endif
};

static struct mutex               adxl345_dd_lock;
static struct list_head           dd_list;
static struct mutex               adxl345_work_lock;
static struct driver_data	*g_adxl_dd = NULL;



#if 1
static long g_nv_accm_x = 0;
static long g_nv_accm_y = 0;
static long g_nv_accm_z = 0;
static int g_x_off = 0;
static int g_y_off = 0;
static int g_z_off = 0;
#endif

#if 1
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t g_stop;
static atomic_t g_suspend;
static atomic_t g_halt;
static atomic_t g_probe;
#endif


#define ADXL_IOCTL_SET_CALIB_INIT	(1)
#define ADXL_IOCTL_SET_CALIB_DONE	(2)
#define ADXL_IOCTL_GET_CALIB_DATA	(3)
#define ADXL_IOCTL_SET_CALIB_DATA	(4)
#define ADXL_IOCTL_GET_CALIB_REQ	(5)

static atomic_t adxl_calibration_requested;
static atomic_t adxl_calibration_progress;
static 	int adxl345_start(void);
static void
gsStableData_reset(void)
{
    AdxlGs_Data.gCalculatorIndex = 0;
    AdxlGs_Data.gCalculatorNum = 0;
}

static int
gsStableData_add(Adxl_data_t *pdata)
{
    if (pdata == 0) {
        return -1;
    }
    AdxlGs_Data.gCalculatorData[AdxlGs_Data.gCalculatorIndex++] = *pdata;
    AdxlGs_Data.gCalculatorNum++;

    if (AdxlGs_Data.gCalculatorIndex >= GSCALC_DATA_NUM) {
        AdxlGs_Data.gCalculatorIndex = 0;
    }
    if (AdxlGs_Data.gCalculatorNum >= GSCALC_DATA_NUM) {
        AdxlGs_Data.gCalculatorNum = GSCALC_DATA_NUM;
    }
    return 0;
}


static void adxl345_cs_low(void)
{
/*	gpio_set_value(20, 0x00); */
/*	ndelay(200); */
}
static void adxl345_cs_high(void)
{
/*	ndelay(200); */
/*	gpio_set_value(20, 0x01); */
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void adxl345_early_suspend(struct early_suspend *h)
{
	char                tx_buf[8];
	char                rx_buf[8];
	struct spi_message  m;
	struct spi_transfer t;
	int rc = 0;
	struct driver_data         *dd =
		container_of(h, struct driver_data, e_sus_fcn);
	mutex_lock(&adxl345_work_lock);
	atomic_set(&g_suspend, 1);
	if (atomic_read(&g_halt) == 1)
	{
		mutex_unlock(&adxl345_work_lock);
		return;
	}
	atomic_set(&g_halt, 1);
	disable_irq(dd->spi->irq);
		
	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s ENTER!!\n", __func__);
	
	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = NULL/*rx_buf*/;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	/* 14 */
	adxl345_cs_low();
	tx_buf[0] = 0x2D;
	tx_buf[1] = 0x00;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	adxl345_cs_high();

	DBG_MSG(KERN_INFO "%s(): going to early suspend\n", __func__);

	DBG_MSG(KERN_INFO "%s(): Handled early suspend event." " rc = %d\n", __func__, rc);
	
	mutex_unlock(&adxl345_work_lock);
	return;
}


void adxl345_late_resume(struct early_suspend *h)
{
	struct driver_data         *dd =
		container_of(h, struct driver_data, e_sus_fcn);
	
	mutex_lock(&adxl345_work_lock);
	atomic_set(&g_suspend, 0);
 	if (atomic_read(&g_halt) == 1) 
	{
		if ((atomic_read(&g_suspend) == 0) && (atomic_read(&g_stop) == 0))
		{
			schedule_work(&dd->work_data);
		}
	}  
	mutex_unlock(&adxl345_work_lock);
	return;
} 

#endif


int adxl345_stop_ap(void)
{
    char                tx_buf[8];
    char                rx_buf[8];
    struct spi_message  m;
    struct spi_transfer t;
    int rc=0;
    struct driver_data    *dd = g_adxl_dd;

	if(atomic_read(&g_probe))
	{
		mutex_lock(&adxl345_work_lock);
		atomic_set(&g_stop, 1);
		if (atomic_read(&g_halt) == 1)
		{
			mutex_unlock(&adxl345_work_lock);
			return 0;
		}
		atomic_set(&g_halt, 1);
		dd = g_adxl_dd;

		//disable interrupt
		disable_irq(dd->spi->irq);

		memset(&t, 0, sizeof t);
		t.tx_buf = tx_buf;
		t.rx_buf = NULL/*rx_buf*/;
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		/*disable interrupts from the chip itself by writing 00000000 to 0x2E register*/
		tx_buf[0] = 0x2E;
		tx_buf[1] = 0x00;
		t.len     = 2;
		rc = spi_sync(dd->spi, &m);
		/*put chip to low power mode 0x7 -> 00000111 of 0x2D register, 
		*D0, D1 and D2 bits set high puts the chip to sleep and makes the data_ready frequency minimal
		*i.e. 1 from 8 originally*/

		tx_buf[0] = 0x2D;
		tx_buf[1] = 0x00;
		t.len     = 2;
		rc = spi_sync(dd->spi, &m);
		mutex_unlock(&adxl345_work_lock);
	}
	else
	{
		atomic_set(&g_stop, 1);
	}
  return 0;
}

EXPORT_SYMBOL(adxl345_stop_ap);


int adxl345_start_ap(void)
{

	if(atomic_read(&g_probe))
	{
		mutex_lock(&adxl345_work_lock);
		atomic_set(&g_stop, 0);
		if (atomic_read(&g_halt) == 1) 
		{
			if ((atomic_read(&g_suspend) == 0) && (atomic_read(&g_stop) == 0))
			{
				schedule_work(&g_adxl_dd->work_data);
			}
		}  
		mutex_unlock(&adxl345_work_lock);                    
	 }
	 else
	 {
		atomic_set(&g_stop, 0);
	 }
	 return 0;
}

EXPORT_SYMBOL(adxl345_start_ap);


static int adxl345_start(void)
{
	char                tx_buf[8];
	char                rx_buf[8];
	struct spi_message  m;
	struct spi_transfer t;
	int rc = 0;
	struct driver_data  *dd = g_adxl_dd;

	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s ENTER!!\n", __func__);

	atomic_set(&g_halt, 0);
	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = NULL /*rx_buf*/;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

#if 0
	/* 14 */
	tx_buf[0] = 0x2D;
	tx_buf[1] = 0x08;
	t.len     = 4;

	rc = spi_sync(dd->spi, &m);
	//	enable interrupts
	t.rx_buf = NULL;
	tx_buf[0] = 0x2E;
	tx_buf[1] = 0x80;
	t.len     = 2;
	spi_sync(dd->spi, &m);

#else
	//	enable interrupts
	t.rx_buf = NULL;
	tx_buf[0] = 0x2E;
	tx_buf[1] = 0x80;
	t.len     = 2;
	spi_sync(dd->spi, &m);

	/* 14 */
	tx_buf[0] = 0x2D;
	tx_buf[1] = 0x08;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
#endif

	enable_irq(dd->spi->irq); 
	return rc;
}




#ifdef CONFIG_PM
static int adxl345_suspend(struct spi_device *spi, pm_message_t mesg)
{
	return 0;
}

static int adxl345_resume(struct spi_device *spi)
{
	
	return 0;
}
#else
#define adxl345_suspend NULL
#define adxl345_resume NULL
#endif /* CONFIG_PM */

static irqreturn_t adxl345_irq(int irq, void *dev_id)
{
	struct device      *dev = dev_id;
	struct driver_data *dd;
#if 0 //DEBUG -->
	static int dbg_cnt=0;
	static unsigned char dbg_led=1;
#endif //<-- DEBUG
	
	disable_irq(irq);

#if 0 //DEBUG -->
	if( !(dbg_cnt % 100))
	{
        	gpio_set_value(33, dbg_led);
		dbg_led = !dbg_led;
	}
	dbg_cnt++;
	dbg_cnt %= 1000000;
#endif //<-- DEBUG

	dd = dev_get_drvdata(dev);
	schedule_work(&dd->work_data);
	return IRQ_HANDLED;
}

#if 1
static int adxl345_open(struct input_dev *dev)
{
	int                 rc = 0;
	struct driver_data *dd = input_get_drvdata(dev);

	if (!dd->spi->irq)
		return -1;
	
	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s ADXL345 OPEN\n", __func__);

	rc = request_irq(dd->spi->irq,
			 &adxl345_irq,
			IRQF_TRIGGER_HIGH,
			 ADXL345_NAME,
			 &dd->spi->dev);
	return rc;
}

static int adxl345_open2(struct inode *inode, struct file *file)
{
	int ret = -1;
    DBG_MSG(KERN_INFO "***** ACCEL_DEBUG :ENTER %s\n", __func__);
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			ret = 0;
		}
	}
        DBG_MSG(KERN_INFO "***** ACCEL_DEBUG :LEAVE %s\n", __func__);
	return ret;
	return 0;
}

static int adxl345_release2(struct inode *inode, struct file *file)
{
    DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : ENTER %s \n", __func__);
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
    DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : LEAVE %s \n", __func__);
	return 0;
}

static int adxl_read_nvitem(unsigned int id, long *data)
{
        int rc;
        unsigned long l_zero = 0;

        DBG_MSG("===ADXL func :[%s], line : [%d], id : [%d], data : [%d], ====\n", __func__, __LINE__,id, *data);

        /* NULL check */
        if( data == NULL )
        {
                DBG_MSG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* VALUE check */
        if(( NV_ACCM_X_OFFSET_I > id ) || ( id > NV_ACCM_Z_OFFSET_I ))
        {
                DBG_MSG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* read NV */
label1 :
        rc = msm_proc_comm(PCOM_NV_READ, &id, (unsigned *)data);
        DBG_MSG("ADXL_NV_DEBUG:Error %s(%d), ID [%d], after msm_proc_comm, return val :[%d]! \n", __func__, __LINE__,id, rc);
        if( rc )
        {
                DBG_MSG("ADXL_NV_DEBUG:Error %s(%d), ID [%d], going to write again, return val :[%d]! \n", __func__, __LINE__,id, rc);
                /* write NV */
                rc = msm_proc_comm(PCOM_NV_WRITE, &id, (unsigned *)&l_zero);
                if( rc )
                {
                        DBG_MSG("===ADXL func :[%s], line : [%d], return : [%d], WRITE ERROR====\n", __func__, __LINE__,rc);
                }
                else
                {
                        goto label1;
                }


        }

        return rc;

}


static int adxl_write_nvitem(unsigned int id, long *data)
{
        int rc;

        DBG_MSG("===ADXL func :[%s], line : [%d], id : [%d], data : [%d], ====\n", __func__, __LINE__,id, *data);

        /* NULL check */
        if( data == NULL )
        {
                DBG_MSG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* VALUE check */
        if(( NV_ACCM_X_OFFSET_I > id ) || ( id > NV_ACCM_Z_OFFSET_I ))
        {
                DBG_MSG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
                return -1;
        }

        /* write VN */
        rc = msm_proc_comm(PCOM_NV_WRITE, &id, (unsigned *)data);
        if( rc )
        {
#if 1 //DEBUG -->
                DBG_MSG("ADXL_NV_DEBUG:Error %s(%d)! \n", __func__, __LINE__);
#endif//<-- DEBUG
        }


        return rc;
}

static int convert_int2signedchar(int input, signed char *output)
{
signed char cnt=0;
int i=0;
		DBG_MSG("\n input [%d]===\n", input);
        if(input < 0)
        {
		*output = abs(input);
		DBG_MSG("\n <ADXL> output value [%d]===\n", *output);
        *output = (signed char)((*output) * (-1));
        }
	else
		*output=input;

return 0;
}


#if 1
static int adxl345_get_calibration_data(void)
{
	int rc = -1;
	signed char x,y,z;

        DBG_MSG("***** adxl345_get_calibration_data : ENTER %s \n", __func__);

		/* read X axi offset */
	rc = adxl_read_nvitem( NV_ACCM_X_OFFSET_I, (signed char *)&x);
		/* read Y axi offset */
		if (!rc)
		{
			rc = adxl_read_nvitem( NV_ACCM_Y_OFFSET_I, (signed char *)&y);
		}
		/* read Z axi offset */
		if (!rc)
		{
			rc = adxl_read_nvitem( NV_ACCM_Z_OFFSET_I, (signed char *)&z);
		}
	DBG_MSG("\n<ADXL>====GET calibration values-x[%d],y[%d],z[%d]===\n",x,y,z);
	if(!rc)
	{
	g_x_off = x * 4;
	g_y_off = y * 4;
	g_z_off = z * 4;
	}
	DBG_MSG("\n ADXL NV calibration values ,values [%d,%d,%d]:===\n", g_x_off,g_y_off,g_z_off);
	DBG_MSG("\n===<ADXL>===adxl345_get_calibration_data, nv ram return value:[%d]===\n",rc);

	return rc;
}

static int adxl345_set_calibration_data(void)
{
        int rc = -1;
	signed char x,y,z;

    	DBG_MSG("***** adxl345_set_calibration_data : ENTER %s \n", __func__);
	if((g_nv_accm_x !=0) && (g_nv_accm_y != 0) && (g_nv_accm_z !=0))
	{
		g_nv_accm_x = g_nv_accm_x / 4;
		g_nv_accm_y = g_nv_accm_y / 4;
		g_nv_accm_z = g_nv_accm_z / 4;

		convert_int2signedchar(g_nv_accm_x, &x);
		convert_int2signedchar(g_nv_accm_y, &y);
		convert_int2signedchar(g_nv_accm_z, &z);
		DBG_MSG("\n<ADXL>====set calibration values-x[%d],y[%d],z[%d]===\n",x,y,z);
                /* read X axi offset */
                rc = adxl_write_nvitem( NV_ACCM_X_OFFSET_I, (signed char *)&x);
                /* read Y axi offset */
                if (!rc)
                {
                        rc = adxl_write_nvitem( NV_ACCM_Y_OFFSET_I, (signed char *)&y);
                }
                /* read Z axi offset */
                if (!rc)
                {
                        rc = adxl_write_nvitem( NV_ACCM_Z_OFFSET_I, (signed char *)&z);
                }
	}
	DBG_MSG("\n===<ADXL>===adxl345_set_calibration_data, nv ram return value:[%d]===\n",rc);
        return rc;
}


#endif


static int
adxl345_ioctl2(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	Adxl_data_t  *p = 0, calibdata;
	Adxl_Gs_Data_t *pGsData;
	int result = 0;
    DBG_MSG(KERN_INFO "***** ACCEL_DEBUG :ENTER %s cmd=%x\n", __func__, cmd);
	mutex_lock(&adxl345_work_lock);
	switch (cmd) {
	  case ADXL_IOCTL_SET_CALIB_INIT:
		  atomic_set(&adxl_calibration_requested, 1);
    	  DBG_MSG(KERN_INFO "Request for Calibration:%d\n",atomic_read(&adxl_calibration_requested));
		  break;
	  case ADXL_IOCTL_SET_CALIB_DONE:
		  atomic_set(&adxl_calibration_progress, 0);
    	  DBG_MSG(KERN_INFO "Calibration Done:%d\n",atomic_read(&adxl_calibration_progress));
		  break;
	  case ADXL_IOCTL_SET_CALIB_DATA:
		  {
			DBG_MSG("\n ADXL ADXL_IOCTL_SET_CALIB_DATA:===\n");
			  p = (Adxl_data_t *)argp;
			  if (copy_from_user(&calibdata, p, sizeof(Adxl_data_t))) {
					result = -EFAULT;
			  } else {
				if((calibdata.x[0] == 0) && (calibdata.x[0] == 0) && (calibdata.x[0] == 0))
				{
				  adxl345_get_calibration_data();
				}
				else
				{
				  g_nv_accm_x = calibdata.x[0];
				  g_nv_accm_y = calibdata.x[1];
				  g_nv_accm_z = calibdata.x[2];
				  g_x_off = g_nv_accm_x;
				  g_y_off = g_nv_accm_y;
				  g_z_off = g_nv_accm_z;
				  adxl345_set_calibration_data();
				}
				DBG_MSG("\n ADXL ADXL_IOCTL_SET_CALIB_DATA,values [%d,%d,%d]:===\n", calibdata.x[0],calibdata.x[1],calibdata.x[2]);
			  }
		  }
		  break;
	  case ADXL_IOCTL_GET_CALIB_DATA:
		  {
			DBG_MSG("\n ADXL ADXL_IOCTL_GET_CALIB_DATA:===\n");
			 pGsData = (Adxl_Gs_Data_t *)argp;
			 if (copy_to_user(pGsData, &AdxlGs_Data, sizeof(Adxl_Gs_Data_t))) {
    			 DBG_MSG(KERN_INFO "***** ACCEL_DEBUG :copy to user failed:%d\n",__LINE__);
				  result = -EFAULT;
			 }
		  }
		  break;
	  case ADXL_IOCTL_GET_CALIB_REQ:
		   {
			  int status = atomic_read(&adxl_calibration_progress);
			  if (copy_to_user(argp, &status, sizeof(status))) {
    			  	  DBG_MSG(KERN_INFO "***** ACCEL_DEBUG :copy to user failed:%d\n",__LINE__);
					  result = -EFAULT;
			  }
		   }
		  break;

	   default:
    	  DBG_MSG(KERN_INFO "default case\n");
	}
	mutex_unlock(&adxl345_work_lock);
#if 0 
	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_TFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		atomic_set(&a_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		break;
	case ECS_IOCTL_APP_SET_TFLAG:
		atomic_set(&t_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_TFLAG:
		flag = atomic_read(&t_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		akmd_delay = flag;
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = akmd_delay;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}
#endif
    DBG_MSG(KERN_INFO "***** ACCEL_DEBUG :LEAVE %s -> %d \n", __func__, result);
	return result;
}

static void adxl345_release(struct input_dev *dev)
{
	struct driver_data *dd = input_get_drvdata(dev);

	free_irq(dd->spi->irq, &dd->spi->dev);
}
#endif

static void convert_regdata_to_accel_data(u8 *buf, struct adxl345_accel_data *a)
{
#if 0
	/* The BMA150 returns 10-bit values split over 2 bytes */
	a->accel_x = ((buf[1] & 0xC0) >> 6) | (buf[2] << 2);
	a->accel_y = ((buf[3] & 0xC0) >> 6) | (buf[4] << 2);
	a->accel_z = ((buf[5] & 0xC0) >> 6) | (buf[6] << 2);
	/* convert 10-bit signed value to 32-bit */
	if (a->accel_x & 0x200)
		a->accel_x = a->accel_x - 0x400;
	if (a->accel_y & 0x200)
		a->accel_y = a->accel_y - 0x400;
	if (a->accel_z & 0x200)
		a->accel_z = a->accel_z - 0x400;
	/* 0-based, units are 0.5 degree C */
	a->temp = buf[7] - BMA150_TEMP_OFFSET;
#else
	/* The ADXL345 returns 13-bit values split over 2 bytes */
	a->accel_x = (buf[1] | ((buf[2] & 0x1F) << 8));
	a->accel_y = (buf[3] | ((buf[4] & 0x1F) << 8));
	a->accel_z = (buf[5] | ((buf[6] & 0x1F) << 8));
	/* convert 13-bit signed value to 32-bit */
	if (a->accel_x & 0x1000)
		a->accel_x = a->accel_x - 0x2000;
	if (a->accel_y & 0x1000)
		a->accel_y = a->accel_y - 0x2000;
	if (a->accel_z & 0x1000)
		a->accel_z = a->accel_z - 0x2000;
	/* 0-based, units are 0.5 degree C —Ç‚­•ª‚©‚ç‚È‚¢‚Ì‚Å0‚ÅŒÅ’è */
#if DEG_LEFT_SHIFT_90
	/* Change the x and y co-ordinates for the change of sensors in the H/W positions.*/
	/* +Y -> +X and  -Y -> -X */
	/* +X -> -Y and  -X -> +Y */
	/* So Y = X and X = (-1)Y */
	{
		int  tmp,x,y;
		static int i = 0;
		x = a->accel_x;
		y = a->accel_y;
		i++;
		tmp = a->accel_y;
		a->accel_y = a->accel_x;
		a->accel_x = (-1)*(tmp);
	}
#endif
	a->temp = 0;
#endif
}


static void adxl345_work_f(struct work_struct *work)
{
	u8                          tx_buf[8];
	u8                          rx_buf[8];
	int                         rc;
	int                         i;
	struct spi_message          m;
	struct spi_transfer         t;
	struct driver_data         *dd =
		container_of(work, struct driver_data, work_data);
	struct adxl345_accel_data    acc_data;

	mutex_lock(&adxl345_work_lock);

	/* Resume the adxl driver in case it was stopped */

	if (atomic_read(&g_halt) == 1) 
	{
		if ((atomic_read(&g_suspend) == 0) && (atomic_read(&g_stop) == 0))
		{
			adxl345_start();
		}
		else
		{
			mutex_unlock(&adxl345_work_lock);
			return;
		}
	}

	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s ENTER!!\n", __func__);
	
	for (i = 1; i < 8; i++)
		tx_buf[i] = 0x00;
	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	/* 17 */
	adxl345_cs_low();
	tx_buf[0] = 0xB0;
	t.len     = 2; /* t.len is 2 since in receive 0th byte is not used and 1st byte is used to get the status, so total 2 bytes required */
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto workf_exit;
	
	/* 18 */
	if( (rx_buf[1] & 0x80) != 0x80) {
		DBG_MSG(KERN_INFO "%s : no support this interrupt (0x%02x)\n", __func__,rx_buf[1]);
		goto workf_exit;
	}	
	DBG_MSG(KERN_INFO "%s : support this interrupt (0x%02x)\n", __func__,rx_buf[1]);

	/* read XYZ */
	tx_buf[0] = 0xF2;	/* 1111 0011 : Read(1)/Write(0) , Multi(1)/Single(0), Address(11 0011) */
	t.len     = 7; /* in receive 6 bytes for x,y,z starting from 1, 0th byte is not used, so total 7 bytes */
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto workf_exit;
	adxl345_cs_high();
	convert_regdata_to_accel_data(t.rx_buf, &acc_data);
	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s x=0x%08x, y=0x%08x, z=0x%08x\n", 
			__func__, acc_data.accel_x, acc_data.accel_y, acc_data.accel_z);
	
#if 0	
	input_report_abs(dd->ip_dev, ABS_X, acc_data.accel_x);
	input_report_abs(dd->ip_dev, ABS_Y, acc_data.accel_y);
	input_report_abs(dd->ip_dev, ABS_Z, acc_data.accel_z);
#endif
	{
		int x, y, z;
		int calib_req = atomic_read(&adxl_calibration_requested);
		Adxl_data_t cAdxlData;
		if (calib_req == 1) {
			gsStableData_reset();
		  	atomic_set(&adxl_calibration_progress, 1);
			atomic_set(&adxl_calibration_requested, 0);
		}
		x = acc_data.accel_x;
		y = acc_data.accel_y;
		z = acc_data.accel_z;
		cAdxlData.x[0] = x;
		cAdxlData.x[1] = y;
		cAdxlData.x[2] = z;
		gsStableData_add(&cAdxlData);
		x = x - g_x_off;
		y = y - g_y_off;
		/* z = z - g_z_off; */
		z = z - g_z_off;
		input_report_abs(dd->ip_dev, ABS_X, x);
		input_report_abs(dd->ip_dev, ABS_Y, y);
		input_report_abs(dd->ip_dev, ABS_Z, z);
	}

	input_report_abs(dd->ip_dev, ABS_WHEEL, acc_data.temp);
	input_sync(dd->ip_dev);
	enable_irq(dd->spi->irq);
	mutex_unlock(&adxl345_work_lock);
	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s EXIT!!\n", __func__);
	return;

workf_exit:
	dev_err(&dd->ip_dev->dev, "%s: exit with error %d\n", __func__, rc);
	mutex_unlock(&adxl345_work_lock);
}

static int __devexit adxl345_power_down(struct driver_data *dd)
{
	char                tx_buf[2];
	struct spi_message  m;
	struct spi_transfer t;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	spi_setup(dd->spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return 0;
}

static int __devinit adxl345_power_up(struct driver_data *dd)
{
	char                tx_buf[8];
	struct spi_message  m;
	struct spi_transfer t;

	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	spi_setup(dd->spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return 0;
}

static int __devinit adxl345_config(struct driver_data *dd)
{
	char                tx_buf[8];
	char                rx_buf[8];
	int                 rc;
	struct spi_message  m;
	struct spi_transfer t;
	dd->spi->bits_per_word = 8;
	adxl345_power_up(dd);
	memset(&t, 0, sizeof t);
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	spi_setup(dd->spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
#if 0
	/* 1 - 7 */
	adxl345_cs_low();
	tx_buf[0] = 0x5D;
	tx_buf[1] = 0x31; /* threashold config */

	tx_buf[2] = (unsigned char)g_nv_accm_x; /* X offset */
	tx_buf[3] = (unsigned char)g_nv_accm_y; /* Y offset */
	tx_buf[4] = (unsigned char)g_nv_accm_z; /* Z offset */
	tx_buf[5] = 0x02;
	tx_buf[6] = 0x11;
	tx_buf[7] = 0x11;
	t.len     = 8;
	t.rx_buf  = NULL;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 8 */
	tx_buf[0] = 0x2A;
	tx_buf[1] = 0x00;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	/* 9 */
	tx_buf[0] = 0x2C;
	tx_buf[1] = 0x09;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 10 */
	tx_buf[0] = 0x2E;
	tx_buf[1] = 0x80;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 11 */
	tx_buf[0] = 0x2F;
	tx_buf[1] = 0x00;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 12 */
	tx_buf[0] = 0x31;
	tx_buf[1] = 0x0B;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 13 */
	tx_buf[0] = 0x38;
	tx_buf[1] = 0x00;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	
	/* 14 */
	tx_buf[0] = 0x2D;
	tx_buf[1] = 0x08;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);

#else
	/* 1 - 7 */
	adxl345_cs_low();
	tx_buf[0] = 0x5E;
	tx_buf[1] = (unsigned char)/*g_nv_accm_x*/0; /* X offset */

	tx_buf[2] = (unsigned char)/*g_nv_accm_y*/0; /* Y offset */
	tx_buf[3] = (unsigned char)/*g_nv_accm_z*/0; /* Z offset */
	t.len     = 4;
	t.rx_buf  = NULL;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 9 */
	tx_buf[0] = 0x2C;
	tx_buf[1] = 0x09;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	

	/* 11 */
	tx_buf[0] = 0x2F;
	tx_buf[1] = 0x00;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 12 */
	tx_buf[0] = 0x31;
	tx_buf[1] = 0x0B;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	if (rc)
		goto config_exit;
	
	/* 13 */
	tx_buf[0] = 0x38;
	tx_buf[1] = 0x00;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);

	/* 13 */
	tx_buf[0] = 0x3B;
	tx_buf[1] = 0xF6;
	t.len     = 2;
	rc = spi_sync(dd->spi, &m);
	

#endif
	adxl345_cs_high();
	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s EXIT!!\n", __func__);
config_exit:
	return rc;
}


#if 1
static struct file_operations adxl345_fops = {
	.owner = THIS_MODULE,
	.open = adxl345_open2,
	.release = adxl345_release2,
	.ioctl = adxl345_ioctl2,
};

static struct miscdevice adxl345_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "adxl345_2",
	.fops = &adxl345_fops,
};
#endif


static int __devinit adxl345_probe(struct spi_device *spi)
{
	struct driver_data *dd;
	int                 rc;
	char               *devname;
	struct adxl345_platform_data *pdata = spi->dev.platform_data;
	
	DBG_MSG(KERN_INFO "***** ACCEL_DEBUG : %s ENTER!!\n", __func__);

	atomic_set(&adxl_calibration_progress, 0);
	atomic_set(&adxl_calibration_requested, 0);
	dd = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}

	devname = kzalloc(sizeof(ADXL345_DEVICE_NAME) + 1, GFP_KERNEL);
	if (!devname) {
		rc = -ENOMEM;
		goto probe_exit_alloc;
	}

#if 0
	rc = adxl345_get_calibration_data();
	if (rc)
	{
		DBG_MSG("%s:Calibration data doesn't exist!\n", __func__);
	}
#if 1
	else
	{
		DBG_MSG("%s:Calibration data was found!\n", __func__);
		DBG_MSG("x = %ld, y = %ld, z = %ld\n", g_nv_accm_x, g_nv_accm_y, g_nv_accm_z);
	}
#endif
#endif

	mutex_lock(&adxl345_dd_lock);
	DBG_MSG("\n=======adxl345 probe called!===========\n");
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&adxl345_dd_lock);
	INIT_WORK(&dd->work_data, adxl345_work_f);
	dd->spi = spi;
	rc = adxl345_config(dd);
	if (rc)
		goto probe_err_cfg;


	if (pdata && pdata->setup) {
		rc = pdata->setup(&spi->dev);
		if (rc)
			goto probe_err_cfg;
	}
#if 0
	adxl345_create_dbfs_entry(dd);
#endif
	spi_set_drvdata(spi, dd);

	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
	snprintf(devname, sizeof(ADXL345_DEVICE_NAME) + 1, ADXL345_DEVICE_NAME,
		 spi->master->bus_num, spi->chip_select);
	dd->ip_dev->open       = adxl345_open;
	dd->ip_dev->close      = adxl345_release;
#if 0
	dd->ip_dev->name       = ADXL345_NAME;
#else
	dd->ip_dev->name       = "accelerometer";
#endif
	dd->ip_dev->phys       = devname;
	dd->ip_dev->id.vendor  = ADXL345_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
#if 0
	__set_bit(EV_REL,    dd->ip_dev->evbit);
	__set_bit(REL_X,     dd->ip_dev->relbit);
	__set_bit(REL_Y,     dd->ip_dev->relbit);
	__set_bit(REL_Z,     dd->ip_dev->relbit);
	__set_bit(REL_MISC,  dd->ip_dev->relbit);
#else
	set_bit(EV_ABS, dd->ip_dev->evbit);
	set_bit(ABS_X,     dd->ip_dev->absbit);
	set_bit(ABS_Y,     dd->ip_dev->absbit);
	set_bit(ABS_Z,     dd->ip_dev->absbit);
	set_bit(ABS_WHEEL,  dd->ip_dev->absbit);

	/* x-axis acceleration */
	input_set_abs_params(dd->ip_dev, ABS_X, -1872, 1872, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(dd->ip_dev, ABS_Y, -1872, 1872, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(dd->ip_dev, ABS_Z, -1872, 1872, 0, 0);
#endif
	rc = input_register_device(dd->ip_dev);
	if (rc) {
		dev_err(&dd->ip_dev->dev,
			"adxl345_probe: input_register_device rc=%d\n",
		       rc);
		goto probe_err_reg_dev;
	}
	
	if (misc_register(&adxl345_device))
	{
		DBG_MSG(KERN_ERR
		       "adxl345_probe: adxl345_device register failed\n");
#if 0
		gpio_set_value(34,1);
#endif
		goto probe_err_reg_dev;
	}
	
#if 1
#ifdef CONFIG_HAS_EARLYSUSPEND
	dd->e_sus_fcn.suspend = adxl345_early_suspend;
	dd->e_sus_fcn.resume = adxl345_late_resume;
	register_early_suspend(&dd->e_sus_fcn);
#endif
#endif

	mutex_lock(&adxl345_dd_lock);
	g_adxl_dd = dd;
	atomic_set(&g_probe,1);
	mutex_unlock(&adxl345_dd_lock);	
	adxl345_start();

	return rc;

probe_err_reg_dev:
	dd->ip_dev = NULL;
	input_free_device(dd->ip_dev);
probe_err_reg:
#if 0
	adxl345_remove_dbfs_entry(dd);
#endif
	spi_set_drvdata(spi, NULL);
	if (pdata && pdata->teardown)
		pdata->teardown(&spi->dev);
probe_err_cfg:
	mutex_lock(&adxl345_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&adxl345_dd_lock);
	kfree(devname);
probe_exit_alloc:
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit adxl345_remove(struct spi_device *spi)
{
	struct driver_data *dd;
	int                 rc;
	const char	   *devname;
	struct adxl345_platform_data *pdata = spi->dev.platform_data;

	dd = spi_get_drvdata(spi);
	devname = dd->ip_dev->phys;

	rc = adxl345_power_down(dd);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: power down failed with error %d\n",
			__func__, rc);
	input_unregister_device(dd->ip_dev);
#if 0
	adxl345_remove_dbfs_entry(dd);
#endif
	spi_set_drvdata(spi, NULL);
	if (pdata && pdata->teardown)
		pdata->teardown(&spi->dev);
	mutex_lock(&adxl345_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&adxl345_dd_lock);
	kfree(devname);
	kfree(dd);

	return 0;
}

static struct spi_driver adxl345_driver = {
	.driver = {
		.name  = ADXL345_NAME,
		.owner = THIS_MODULE,
	},
	.probe         = adxl345_probe,
	.remove        = __devexit_p(adxl345_remove),
	.suspend       = adxl345_suspend,
	.resume        = adxl345_resume,
};


static int __init adxl345_init(void)
{
	int rc;
	
	DBG_MSG("***** ACCEL_DEBUG : %s ENTER!!\n", __func__);
	
	atomic_set(&g_stop,0);
	atomic_set(&g_suspend,0);
	atomic_set(&g_halt,0);
	atomic_set(&g_probe,0);

	INIT_LIST_HEAD(&dd_list);
	mutex_init(&adxl345_dd_lock);
	mutex_init(&adxl345_work_lock);

	rc = spi_register_driver(&adxl345_driver);
	
	DBG_MSG("***** ACCEL_DEBUG : %s EXIT!!\n", __func__);
	
	return rc;
}
module_init(adxl345_init);

static void __exit adxl345_exit(void)
{
	spi_unregister_driver(&adxl345_driver);
}
module_exit(adxl345_exit);
