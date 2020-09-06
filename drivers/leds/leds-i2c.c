/*
  LED Class Core Driver
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



#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include "smd_private.h"
#include "leds.h"
#include <linux/ctype.h>
#define LED_COLORS  7
#define TG03_LED_I2C_SLAVE   (0xEC >> 1)
#define BUFFER_SIZE 64

#define FifoSize 5     /* Number of 8 bit data in the Fifo */
unsigned char PutI;    /* Index of where to put next */
unsigned char GetI;    /* Index of where to get next */
unsigned char Size;    /* Number currently in the FIFO */
                       /* FIFO is empty if Size=0 */
                       /* FIFO is full  if Size=FifoSize */
char led_Fifo[FifoSize];   /* The statically allocated data */
static int led_Fifo_Get (char *datapt);
static int led_Fifo_Put (char data);

struct i2c_adapter *i2c_led;
struct work_struct work;
static uint8_t led_notify;
static uint8_t led_charge;

struct smem_led_color_type
   	{
          uint8_t led_alpha;
          uint8_t led_red;
          uint8_t led_green;
          uint8_t led_blue;
	}; 


struct smem_led_color_type smem_led_color;

enum led_color
{
  OP_SET_RED_BRIGHTNESS = 1,
  OP_SET_GREEN_BRIGHTNESS = 2,
  OP_SET_BLUE_BRIGHTNESS = 4,
  OP_SET_YELLOW_BRIGHTNESS = 3,
  OP_SET_MAGENTA_BRIGHTNESS = 5,
  OP_SET_CYAN_BRIGHTNESS = 6,
  OP_SET_WHITE_BRIGHTNESS = 7,
};

struct i2c_led_work_data
{
  enum led_color color;
  int brightness;
};

struct i2c_led_work_data led_status;

enum {
	LED_RED,
	LED_GREEN,
	LED_BLUE,
        LED_YELLOW,
        LED_MAGENTA,
        LED_CYAN,
        LED_WHITE,
	LED_MAX
};

//static led_status_new[LED_MAX];
static int led_status_new[LED_MAX];

/*
Function : led_i2c_write
Description :		
This function will do i2c write by filling the message buffer with
corresponding register and data

@param reg [in] register address
@param data [in] data to be written to register address
@param len [in] size of the data

@return : Returns negative errno, else the number of messages executed. 
*/
static int
led_i2c_write (uint16_t reg, uint8_t * data, uint32_t len)
{
  struct i2c_msg msg;
  u_int8_t buf[BUFFER_SIZE];
  int ret = 0;

  msg.addr = TG03_LED_I2C_SLAVE;
  msg.flags = 0;
  buf[0] = reg;
  memcpy (&buf[1], data, len);

  msg.buf = buf;
  msg.len = len + 1;
  ret = i2c_transfer (i2c_led, &msg, 1);
  if (ret < 0)
    {
      printk ("[i2c_led]i2c_transfer() falied!(%d) in %s()\n", ret, __func__);
    }
  else
    {
     // printk (KERN_INFO "LEDS i2c write=%d id=%d\n", ret, i2c_led->id);
    }

  return ret;
}

/*
Function : led_i2c_read
Description :
This function will do i2c read by filling the message buffer with
corresponding register and data

@param reg [in] register address
@param data [in] Value read from register address is stored in the data buffer 
@param len [in] size of the data

@return : Returns negative errno, else the number of messages executed. 
*/
static int
led_i2c_read (uint16_t reg, uint8_t * data, uint32_t len)
{
  struct i2c_msg msg[2];
  u_int8_t msgbuf[2];
  int ret = 0;

  memcpy (msgbuf, &reg, sizeof (reg));

  msg[0].addr = TG03_LED_I2C_SLAVE;
  msg[0].flags = 0;
  msg[0].buf = msgbuf;
  msg[0].len = 1;

  msg[1].addr = TG03_LED_I2C_SLAVE;
  msg[1].flags = I2C_M_RD;
  msg[1].buf = data;
  msg[1].len = len;

  ret = i2c_transfer (i2c_led, msg, 2);
  if (ret != 2)
    {
      printk ("[i2c_led]i2c_transfer() falied!(%d) in %s()\n", ret, __func__);
    }
  else
    {
     // printk (KERN_INFO "LEDS i2c read=%d\n", ret);
    }

  return ret;
}

ssize_t
led_notify_store (struct device * dev, struct device_attribute * attr,
		  const char *buf, size_t size)
{
  ssize_t ret = -EINVAL;
  char *after;
  unsigned long state = simple_strtoul (buf, &after, 10);
  size_t count = after - buf;

  if (*after && isspace (*after))
    count++;

  if (count == size)
    {
      ret = count;
      led_notify = (uint8_t)(state & 0x000000ff);
	  led_charge = (uint8_t)((state & 0x0000ff00) >> 8);
	  led_charge = led_charge & 0x07;
    }

  return ret;
}

ssize_t
led_notify_show (struct device * dev, struct device_attribute * attr,
		 char *buf)
{

  uint32_t state = led_charge;
  state = (uint32_t)(state << 8);
  state = (uint32_t)(state + led_notify); 

  return sprintf (buf, "%u\n", state);
}


ssize_t
led_color_store (struct device * dev, struct device_attribute * attr,
                  const char *buf, size_t size)
{
  ssize_t ret = -EINVAL;
  char *after;
  unsigned long state = simple_strtoul (buf, &after, 10);
  size_t count = after - buf;

  if (*after && isspace (*after))
    count++;
  
  if (count == size)
    {
      if (state & 0x08)
         smem_led_color.led_alpha = 0xFF;
      else
         smem_led_color.led_alpha = 0x00;

      if (state & 0x04)
         smem_led_color.led_red = 0xFF;
      else
         smem_led_color.led_red = 0x00;

      if (state & 0x02)
         smem_led_color.led_green = 0xFF;
      else
         smem_led_color.led_green = 0x00;

      if (state & 0x01)
         smem_led_color.led_blue = 0xFF;
      else
         smem_led_color.led_blue = 0x00;
    }

  return ret;
}

ssize_t
led_color_show (struct device * dev, struct device_attribute * attr, char *buf)
{
  char state = 0;
 
  if (smem_led_color.led_alpha == 0xFF)
     state = state | 0x08;
  else
     state = state & 0x07;

  if (smem_led_color.led_red == 0xFF)
     state = state | 0x04;
  else
     state = state & 0x0B;

  if (smem_led_color.led_green == 0xFF)
     state = state | 0x02;
  else
     state = state & 0x0D;

  if (smem_led_color.led_blue == 0xFF)
     state = state | 0x01;
  else
     state = state & 0x0E;

  return sprintf (buf, "%d\n", state);
}



/*
Function : i2c_led_work_func
Description :
This function will do i2c write to specified register when the task is scheduled
It sets the brightness value specific to given LED color

@param work [in] pointer to work structure

@return void
*/

extern void led_notify_light(int on_off);

static void
i2c_led_work_func (struct work_struct *work)
{
  uint8_t rbuf_led = 0;
  unsigned char lcolor = 0;
  unsigned char lbrightness = 0;
  unsigned char ldata = 0;

  if(!(led_Fifo_Get (&ldata) == 0))
   	{
  		printk(KERN_ERR "%s Fifo Size = %d\n",__func__,Size);
		return;
  	}

  lcolor = (unsigned char)(ldata & 0xf0) >>4;
  lbrightness = (uint8_t)(ldata & 0x0f);

  /*sensor external function call*/
  led_notify_light(lbrightness); 

  led_i2c_read (0x1c, &rbuf_led, 1);
  rbuf_led = rbuf_led & ~(0x07);

  if (lbrightness > 0)
    {
      switch (lcolor)
	{
	case OP_SET_RED_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_RED_BRIGHTNESS;
	  break;
	case OP_SET_GREEN_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_GREEN_BRIGHTNESS;
	  break;
	case OP_SET_BLUE_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_BLUE_BRIGHTNESS;
	  break;

	case OP_SET_YELLOW_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_YELLOW_BRIGHTNESS;
	  break;
	case OP_SET_MAGENTA_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_MAGENTA_BRIGHTNESS;
	  break;
	case OP_SET_CYAN_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_CYAN_BRIGHTNESS;
	  break;
	case OP_SET_WHITE_BRIGHTNESS:
	  rbuf_led = rbuf_led | OP_SET_WHITE_BRIGHTNESS;
	  break;

	default:
	  break;
	}
    }

  led_i2c_write (0x1c, &rbuf_led, 1);
}

/*
Function : led_set_color
Description :
This function is used to set the LED brightness
setting GPIOs with I2C/etc requires a task context,So new task is created 
and work is scheduled for setting the brightness

@param led_cdev [in] Pointer to led class device
@param value [in] brightness value to be set.

@return void
*/
static void
led_set_color (struct led_classdev *led_cdev, enum led_brightness value)
{
   unsigned char ldata=0;
   int count = 0;

   if (value == 0)
    {
      led_status.brightness = 0;
    }
   else
    {
      led_status.brightness = 1;
    }

   if (!strcmp (led_cdev->name, "red"))
    {
      led_status.color = OP_SET_RED_BRIGHTNESS;
      if(led_status_new[LED_RED] == led_status.brightness) {
         return;
      }
      led_status_new[LED_RED] = led_status.brightness;
    }
   else if (!strcmp (led_cdev->name, "green"))
    {
      led_status.color = OP_SET_GREEN_BRIGHTNESS;
      if(led_status_new[LED_GREEN] == led_status.brightness) {
         return;
      }
      led_status_new[LED_GREEN] = led_status.brightness;
    }

   else if (!strcmp (led_cdev->name, "blue"))
    {
      led_status.color = OP_SET_BLUE_BRIGHTNESS;
      if(led_status_new[LED_BLUE] == led_status.brightness) {
         return;
      }
      led_status_new[LED_BLUE] = led_status.brightness;
    }
   else if (!strcmp (led_cdev->name, "yellow"))
    {
      led_status.color = OP_SET_YELLOW_BRIGHTNESS;
      if(led_status_new[LED_YELLOW] == led_status.brightness) {
         return;
      }
      led_status_new[LED_YELLOW] = led_status.brightness;
    }
   else if (!strcmp (led_cdev->name, "magenta"))
    {
      led_status.color = OP_SET_MAGENTA_BRIGHTNESS;
      if(led_status_new[LED_MAGENTA] == led_status.brightness) {
         return;
      }
      led_status_new[LED_MAGENTA] = led_status.brightness;
    }
   else if (!strcmp (led_cdev->name, "cyan"))
    {
      led_status.color = OP_SET_CYAN_BRIGHTNESS;
      if(led_status_new[LED_CYAN] == led_status.brightness) {
         return;
      }
      led_status_new[LED_CYAN] = led_status.brightness;
    }
   else                                 // White Color is displayed
    {
      led_status.color = OP_SET_WHITE_BRIGHTNESS;
      if(led_status_new[LED_WHITE] == led_status.brightness) {
         return;
      }
      led_status_new[LED_WHITE] = led_status.brightness;
    }


  ldata = led_status.color;
  ldata = (unsigned char)(ldata << 4);
  ldata = (unsigned char)(ldata + (led_status.brightness & 0x0f)); 

  if (led_Fifo_Put(ldata) == 0)
    {     
    while (schedule_work(&work) != 1)
      {
       count++;
       mdelay(5);
          if (count == 5)
          {
             GetI++;
             Size--;
             if(GetI == FifoSize)
             {
                GetI = 0;
             }
             break;
          }
 
      }
    }  
   else
      printk(KERN_ERR "%s Size = %d\n",__func__,Size);
}

static struct led_classdev i2c_led_data[] = {
  {
   .name = "red",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },
  {
   .name = "green",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },
  {
   .name = "blue",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },

  {
   .name = "yellow",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },
  {
   .name = "magenta",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },
  {
   .name = "cyan",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },
  {
   .name = "white",
   .brightness_set = led_set_color,
   .brightness = LED_OFF,
   },

};

static int
i2c_led_probe (struct platform_device *pdev)
{
  int rc, i;

  i2c_led = i2c_get_adapter (0);


  if (IS_ERR (i2c_led))
    return PTR_ERR (i2c_led);

  INIT_WORK (&work, i2c_led_work_func);

  for (i = 0; i < LED_COLORS; i++)
    {
      rc = led_classdev_register (&pdev->dev, &i2c_led_data[i]);
      if (rc)
	{
	  printk (KERN_ERR "unable to register led class driver :%s\n",
		  i2c_led_data[i].name);
	  return rc;
	}
    }
    
    
  return rc;
}

static int __devexit
i2c_led_remove (struct platform_device *pdev)
{
  int i;

  cancel_work_sync (&work);
  for (i = 0; i < LED_COLORS; i++)
    {
      led_classdev_unregister (&i2c_led_data[i]);
    }

  return 0;
}

#ifdef CONFIG_PM
static int
i2c_led_suspend (struct platform_device *dev, pm_message_t state)
{
  int i;
  static int8_t *led_ctrl_notify = NULL;
  static uint8_t *led_ctrl_charge = NULL;
  static struct smem_led_color_type *led_ctrl_color = NULL;	

  for (i = 0; i < LED_COLORS; i++)
    {
      led_classdev_suspend (&i2c_led_data[i]);
    }

  if (led_ctrl_notify == NULL)
    {
      led_ctrl_notify = smem_alloc (SMEM_OEM_011, sizeof (uint8_t));
    }

  if (led_ctrl_charge == NULL)
    {
      led_ctrl_charge = smem_alloc (SMEM_OEM_012, sizeof (uint8_t));
    }
  if (led_ctrl_color == NULL)
    {
      led_ctrl_color = smem_alloc (SMEM_OEM_026, sizeof (smem_led_color));
    }

  
 if (led_ctrl_notify != NULL)
    {
      *led_ctrl_notify = led_notify;
    }
  if (led_ctrl_charge != NULL)
    {
      *led_ctrl_charge = led_charge;
    }
  if (led_ctrl_color != NULL)
    {
      *led_ctrl_color = smem_led_color;
    }

  return 0;
}

static int
i2c_led_resume (struct platform_device *dev)
{
    int i;
	static uint8_t *sm_led_ctrl_charge = NULL;

	if (sm_led_ctrl_charge == NULL){
		sm_led_ctrl_charge = smem_alloc (SMEM_OEM_012, sizeof (uint8_t));
	}

	/* store charge status */
	led_charge = *sm_led_ctrl_charge & 0x07;

	for (i = 0; i < LED_COLORS; i++)
    {
      led_classdev_resume (&i2c_led_data[i]);
    }

  return 0;
}
#else
#define i2c_led_suspend NULL
#define i2c_led_resume NULL
#endif

/*
Function : led_Fifo_Put
Description :
This function is used to save the LED request to FIFO.
This also checks for the condition when the FIFO is FULL.

@param data : this is the LED request data to be pushed into FIFO. 
@return 0 on successful write to FIFO else -1 for failure
*/
int led_Fifo_Put (char data) 
{ 
  
  if (Size == FifoSize )
    {
    printk("<1>size is full size = %d\n",Size); 
    return(-1);                       /* Failed, fifo was full */
    }
  else{ 
    led_Fifo[PutI]=data;
    PutI++;
    Size++;
    if (PutI == FifoSize) PutI = 0;   /* Wrap */
  }
    return(0);                        
}

/*
Function : led_Fifo_Get
Description :
This function is used to get the LED request from FIFO.
This also checks for the condition when the FIFO is EMPTY.

@param *datapt : data pointer to get the LED request from FIFO. 
@return 0 on successful read from FIFO else -1 for failure
*/
int led_Fifo_Get (char *datapt) 
{ 
  if (Size == 0 ) 
    {
     printk("<1> Get Fifo Empty\n");
     return(-1);                       /* Empty if Size=0 */
    }
  else{
     *datapt=led_Fifo[GetI];
     led_Fifo[GetI] = 0xff;
     GetI++;
     Size--;
     if (GetI == FifoSize) GetI = 0;
     }
   return(0);
}

static struct platform_driver i2c_led_driver = {
  .probe = i2c_led_probe,
  .remove = __devexit_p (i2c_led_remove),
  .suspend = i2c_led_suspend,
  .resume = i2c_led_resume,
  .driver = {
	     .name = "i2c-leds",
	     .owner = THIS_MODULE,
	     },
};

static int __init
i2c_led_init (void)
{
  PutI = 0;
  GetI = 0;
  Size = 0;    
  return platform_driver_register (&i2c_led_driver);
}

module_init (i2c_led_init);

static void __exit
i2c_led_exit (void)
{
  platform_driver_unregister (&i2c_led_driver);
}

module_exit (i2c_led_exit);

MODULE_DESCRIPTION ("I2C LED driver");
MODULE_LICENSE ("GPL v2");
MODULE_ALIAS ("platform:i2c-leds");
