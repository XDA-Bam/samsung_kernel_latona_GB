/*
 * drivers/media/video/mt9p012.c
 *
 * mt9p012 sensor driver
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Leverage OV9640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *****************************************************
 *****************************************************
 * modules/camera/s5k6aafx.c
 *
 * S5K6AAFX sensor driver source file
 *
 * Modified by paladin in Samsung Electronics
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>

#include <mach/gpio.h>
#include <mach/hardware.h>

#include <media/v4l2-int-device.h>
#include "isp/isp.h"
#include "omap34xxcam.h"
#include "s5k6aafx.h"

bool front_cam_in_use = false;

#if (CAM_S5K6AAFX_DBG_MSG)
#include "dprintk.h"
#else
#define dprintk(x, y...)
#endif

//#define CONFIG_LOAD_FILE

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
//#define max_size 200000

struct test
{
	char data;
	struct test *nextBuf;
};
struct test *testBuf;
#endif

//#define FUNC_DEBUG

#ifdef FUNC_DEBUG
#define FUNC_ENTR() printk("[~~~~ S5K6AAFX ~~~~] %s entered\n", __func__)
#else
#define FUNC_ENTR() 
#endif

#define I2C_M_WRITE 0x0000 /* write data, from slave to master */
#define I2C_M_READ  0x0001 /* read data, from slave to master */

static u32 s5k6aafx_curr_state = S5K6AAFX_STATE_INVALID;
static u32 s5k6aafx_pre_state = S5K6AAFX_STATE_INVALID;

/* Section Index */
/*
static int reg_init_qcif_index;
static int reg_init_cif_index;
static int reg_init_qvga_index;
static int reg_init_vga_index;
static int reg_init_qcif_vt_index;
static int reg_init_cif_vt_index;
static int reg_init_qvga_vt_index;
static int reg_init_vga_vt_index;
static int reg_ev_index;
static int reg_ev_vt_index;
static int reg_pretty_none_index;
static int reg_pretty_level1_index;
static int reg_pretty_level2_index;
static int reg_pretty_level3_index;
static int reg_pretty_vt_none_index;
static int reg_pretty_vt_level1_index;
static int reg_pretty_vt_level2_index;
static int reg_pretty_vt_level3_index;
static int reg_7fps_index;
static int reg_10fps_index;
static int reg_13fps_index;
static int reg_self_capture_index;
*/
static struct s5k6aafx_sensor s5k6aafx = {
  .timeperframe = {
    .numerator    = 1,
    .denominator  = 15,
  },
  .mode           = S5K6AAFX_MODE_CAMERA,  
  .state          = S5K6AAFX_STATE_PREVIEW,
  .fps            = 15,
  .preview_size   = S5K6AAFX_PREVIEW_SIZE_640_480,
  .capture_size   = S5K6AAFX_IMAGE_SIZE_1280_960,
  .detect         = SENSOR_NOT_DETECTED,
  .zoom           = S5K6AAFX_ZOOM_1P00X,
  .effect         = S5K6AAFX_EFFECT_OFF,
  .ev             = S5K6AAFX_EV_DEFAULT,
  .contrast       = S5K6AAFX_CONTRAST_DEFAULT,
  .wb             = S5K6AAFX_WB_AUTO,
  .pretty         = S5K6AAFX_PRETTY_NONE,
  .flip           = S5K6AAFX_FLIP_NONE,
};

struct v4l2_queryctrl s5k6aafx_ctrl_list[] = {
  {
    .id            = V4L2_CID_SELECT_MODE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "select mode",
    .minimum       = S5K6AAFX_MODE_CAMERA,
    .maximum       = S5K6AAFX_MODE_VT,
    .step          = 1,
    .default_value = S5K6AAFX_MODE_CAMERA,
  }, 
  {
    .id            = V4L2_CID_SELECT_STATE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "select state",
    .minimum       = S5K6AAFX_STATE_PREVIEW,
    .maximum       = S5K6AAFX_STATE_CAPTURE,
    .step          = 1,
    .default_value = S5K6AAFX_STATE_PREVIEW,
  }, 
  {
    .id            = V4L2_CID_ZOOM,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Zoom",
    .minimum       = S5K6AAFX_ZOOM_1P00X,
    .maximum       = S5K6AAFX_ZOOM_4P00X,
    .step          = 1,
    .default_value = S5K6AAFX_ZOOM_1P00X,
  },
  {
    .id            = V4L2_CID_BRIGHTNESS,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Brightness",
    .minimum       = S5K6AAFX_EV_MINUS_2P0,
    .maximum       = S5K6AAFX_EV_PLUS_2P0,
    .step          = 1,
    .default_value = S5K6AAFX_EV_DEFAULT,
  },
  {
    .id            = V4L2_CID_PRETTY,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Pretty",
    .minimum       = S5K6AAFX_PRETTY_NONE,
    .maximum       = S5K6AAFX_PRETTY_LEVEL3,
    .step          = 1,
    .default_value = S5K6AAFX_PRETTY_NONE,
  },  
};
#define NUM_S5K6AAFX_CONTROL ARRAY_SIZE(s5k6aafx_ctrl_list)

/* list of image formats supported by s5k6aafx sensor */
const static struct v4l2_fmtdesc s5k6aafx_formats[] = {
  {
    .description = "YUV422 (UYVY)",
    .pixelformat = V4L2_PIX_FMT_UYVY,
  },
  {
    .description = "YUV422 (YUYV)",
    .pixelformat = V4L2_PIX_FMT_YUYV,
  },
  {
    .description = "JPEG(without header)+ JPEG",
    .pixelformat = V4L2_PIX_FMT_JPEG,
  },
  {
    .description = "JPEG(without header)+ YUV",
    .pixelformat = V4L2_PIX_FMT_MJPEG,
  },  
};
#define NUM_S5K6AAFX_FORMATS ARRAY_SIZE(s5k6aafx_formats)

extern struct s5k6aafx_platform_data nowplus_s5k6aafx_platform_data;
/*
static int s5k6aafx_i2c_write_read(struct i2c_client *client, u8 writedata_num, const u8* writedata, u8 readdata_num, u8* readdata)
{
  int err = 0, i = 0;
  struct i2c_msg msg[1];
  unsigned char writebuf[writedata_num];
  unsigned char readbuf[readdata_num];

  if (!client->adapter)
  {
    printk(S5K6AAFX_MOD_NAME "can't search i2c client adapter\n");
    return -ENODEV;
  }

  // Write
  msg->addr  = client->addr;
  msg->flags = I2C_M_WRITE;
  msg->len   = writedata_num;
  memcpy(writebuf, writedata, writedata_num);    
  msg->buf   = writebuf;
  
  for(i = 0; i < 10; i++)  
  {
    err = i2c_transfer(client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if(err == 0) break;
    mdelay(1);
  }

  if(i == 10)
  {
    printk(S5K6AAFX_MOD_NAME "s5k6aafx_i2c_write_read is failed... %d\n", err);
    return err;  
  }

  // Read 
  msg->addr  = client->addr;
  msg->flags = I2C_M_READ;
  msg->len   = readdata_num;
  memset(readbuf, 0x0, readdata_num);
  msg->buf   = readbuf;
  
  for(i = 0; i < 10; i++)
  {
    err = i2c_transfer(client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if (err == 0) 
    {
      memcpy(readdata, readbuf, readdata_num);
      return 0;
    }
    mdelay(1);
  }

  printk(S5K6AAFX_MOD_NAME "s5k6aafx_i2c_write_read is failed... %d\n", err);

  return err;
}
*/
static inline int s5k6aafx_read(struct i2c_client *client, unsigned short subaddr, unsigned short *data)
{
	unsigned char buf[2];
	int err = 0;
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	*(unsigned short *)buf = cpu_to_be16(subaddr);

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
		printk("%s: %d register read fail\n", __func__, __LINE__);	

	msg.flags = I2C_M_READ;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
		printk("%s: %d register read fail\n", __func__, __LINE__);	

	*data = ((buf[0] << 8) | buf[1]);
		
	return err;
}

static inline int s5k6aafx_write(struct i2c_client *client,
		unsigned long packet)
{
	unsigned char buf[4];

	int err = 0;
	int retry_count = 5;

	struct i2c_msg msg = 
	{
		.addr	= client->addr,
		.flags	= 0,
		.buf	= buf,
		.len	= 4,
	};

	if (!client->adapter)
	{
	  dev_err(&client->dev, "%s: can't search i2c client adapter\n", __func__);
	  return -EIO;
	}

	while(retry_count--)
	{		
		*(unsigned long *)buf = cpu_to_be32(packet);
		err = i2c_transfer(client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
		mdelay(10);
	}

	if (unlikely(err < 0)) 
	{
		dev_err(&client->dev, "%s: 0x%08x write failed\n", __func__, (unsigned int)packet);
		return err;
	}
	
	return (err != 1) ? -1 : 0;
}

/*
 * s5k6aafx sensor i2c write routine
 * <start>--<Device address><2Byte Subaddr><2Byte Value>--<stop>
 */
#ifdef CONFIG_LOAD_FILE
static int loadFile(void)
{
	struct file *fp;
	struct test *nextBuf = testBuf;
	
	char *nBuf;
	int max_size;
	int l;
	int i = 0;
	//int j = 0;
	int starCheck = 0;
	int check = 0;
	int ret = 0;
	loff_t pos;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fp = filp_open("sdcard/sd/s5k6aafx.h", O_RDONLY, 0);

	if (IS_ERR(fp)) 
	{
		printk("%s : file open error\n", __func__);
		return PTR_ERR(fp);
	}

	l = (int) fp->f_path.dentry->d_inode->i_size;

	max_size = l;
	
	printk("l = %d\n", l);
	nBuf = kmalloc(l, GFP_KERNEL);
	testBuf = (struct test*)kmalloc(sizeof(struct test) * l, GFP_KERNEL);

	if (nBuf == NULL) 
	{
		printk( "Out of Memory\n");
		filp_close(fp, current->files);
	}
	
	pos = 0;
	memset(nBuf, 0, l);
	memset(testBuf, 0, l * sizeof(struct test));

	ret = vfs_read(fp, (char __user *)nBuf, l, &pos);

	if (ret != l) 
	{
		printk("failed to read file ret = %d\n", ret);
		kfree(nBuf);
		kfree(testBuf);
		filp_close(fp, current->files);
		return -1;
	}

	filp_close(fp, current->files);

	set_fs(fs);

	i = max_size;

	printk("i = %d\n", i);

	while (i)
	{
		testBuf[max_size - i].data = *nBuf;
		if (i != 1)
		{
			testBuf[max_size - i].nextBuf = &testBuf[max_size - i + 1];
		}
		else
		{
			testBuf[max_size - i].nextBuf = NULL;
			break;
		}
		i--;
		nBuf++;
	}

	i = max_size;
	nextBuf = &testBuf[0];
	
#if 1
	while (i - 1)
	{
		if (!check && !starCheck)
		{
			if (testBuf[max_size - i].data == '/')
			{
				if(testBuf[max_size-i].nextBuf != NULL)
				{
					if (testBuf[max_size-i].nextBuf->data == '/')
					{
						check = 1;// when find '//'
						i--;
					}
					else if (testBuf[max_size-i].nextBuf->data == '*')
					{
						starCheck = 1;// when find '/*'
						i--;
					}
				}	
				else
				{
					break;
				}
			}
			if (!check && !starCheck)
			{
				if (testBuf[max_size - i].data != '\t')//ignore '\t'
				{
					nextBuf->nextBuf = &testBuf[max_size-i];
					nextBuf = &testBuf[max_size - i];
				}
			}
		}
		else if (check && !starCheck)
		{
			if (testBuf[max_size - i].data == '/')
			{
				if(testBuf[max_size-i].nextBuf != NULL)
				{
					if (testBuf[max_size-i].nextBuf->data == '*')
					{
						starCheck = 1;// when find '/*'
						check = 0;
						i--;
					}
				}	
				else 
				{
					break;
				}
			}

			if(testBuf[max_size - i].data == '\n' && check) // when find '\n'
			{
				check = 0;
				nextBuf->nextBuf = &testBuf[max_size - i];
				nextBuf = &testBuf[max_size - i];
			}

		}
		else if (!check && starCheck)
		{
			if (testBuf[max_size - i].data == '*')
			{
				if(testBuf[max_size-i].nextBuf != NULL)
				{
					if (testBuf[max_size-i].nextBuf->data == '/')
					{
						starCheck = 0;// when find '*/'
						i--;
					}
				}	
				else
				{
					break;
				}
			}
		}
		
		i--;
		
		if (i < 2) 
		{
			nextBuf = NULL;
			break;
		}
		
		if (testBuf[max_size - i].nextBuf == NULL)
		{
			nextBuf = NULL;
			break;
		}
	}
#endif

#if 0 // for print
	printk("i = %d\n", i);
	nextBuf = &testBuf[0];
	while (1)
	{
		//printk("sdfdsf\n");
		if (nextBuf->nextBuf == NULL)
			break;
		printk("%c", nextBuf->data);
		nextBuf = nextBuf->nextBuf;
	}
#endif

	return 0;
}
#endif

#ifdef CONFIG_LOAD_FILE

static int s5k6aafx_write_regs_from_sd(struct i2c_client *client, char s_name[])
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct test *tempData;
	
	int ret = -EAGAIN;
	unsigned long temp;
	char delay = 0;
	char data[11];
	int searched = 0;
	int size = strlen(s_name);
	int i;

//	FUNC_ENTR();
	
	printk("size = %d, string = %s\n", size, s_name);
	tempData = &testBuf[0];
	while(!searched)
	{
		searched = 1;
		for (i = 0; i < size; i++)
		{
			if (tempData->data != s_name[i])
			{
				searched = 0;
				break;
			}
			tempData = tempData->nextBuf;
		}
		tempData = tempData->nextBuf;
	}
	//structure is get..

	while(1)
	{
		if (tempData->data == '{')
		{
			break;
		}
		else
		{
			tempData = tempData->nextBuf;
		}
	}

	while (1)
	{
		searched = 0;
		while (1)
		{
			if (tempData->data == 'x')
			{
				//get 10 strings
				data[0] = '0';
				for (i = 1; i < 11; i++)
				{
					data[i] = tempData->data;
					tempData = tempData->nextBuf;
				}
				//printk("%s\n", data);
				temp = simple_strtoul(data, NULL, 16);
				break;
			}
			else if (tempData->data == '}')
			{
				searched = 1;
				break;
			}
			else
			{
				tempData = tempData->nextBuf;
			}
			
			if (tempData->nextBuf == NULL)
			{
				return -1;
			}
		}

		if (searched)
		{
			break;
		}

		//let search...
		if ((temp & S5K6AAFX_DELAY) == S5K6AAFX_DELAY) 
		{                                                    
			delay = temp & 0xFFFF;                                                                              
			//printk("func(%s):line(%d):delay(0x%x):delay(%d)\n",__func__,__LINE__,delay,delay);       
			msleep(delay);                                                                                      
			continue;                                                                                           
		}
		
		ret = s5k6aafx_write(client, temp);

		/* In error circumstances */
		/* Give second shot */
		if (unlikely(ret)) 
		{
			dev_info(&client->dev,
					"s5k6aafx i2c retry one more time\n");
			ret = s5k6aafx_write(client, temp);

			/* Give it one more shot */
			if (unlikely(ret)) 
			{
				dev_info(&client->dev,
						"s5k6aafx i2c retry twice\n");
				ret = s5k6aafx_write(client, temp);
			}
		}
	}

	return ret;
}
#endif


/* program multiple registers */
static int s5k6aafx_write_regs(struct i2c_client *client,
		unsigned long *packet, unsigned int num)
{
	int ret = -EAGAIN;	/* FIXME */
	unsigned long temp;
	char delay = 0;

	while (num--) 
	{
		temp = *packet++;
#if 1
		if ((temp & S5K6AAFX_DELAY) == S5K6AAFX_DELAY) 
		{                                                    
			delay = temp & 0xFFFF;                                                                              
			//printk("func(%s):line(%d):delay(0x%x):delay(%d)\n",__func__,__LINE__,delay,delay);       
			msleep(delay);                                                                                      
			continue;                                                                                           
		}
#endif
		ret = s5k6aafx_write(client, temp);

		/* In error circumstances */
		/* Give second shot */
		if (unlikely(ret)) 
		{
			dev_info(&client->dev,
				"s5k6aafx i2c retry one more time\n");
			ret = s5k6aafx_write(client, temp);

			/* Give it one more shot */
			if (unlikely(ret)) 
			{
				dev_info(&client->dev,
					"s5k6aafx i2c retry twice\n");
				ret = s5k6aafx_write(client, temp);
				break;
			}
		}
	}
	if( ret < 0)
		return -EIO;	
	
	return ret;	/* FIXME */
}

/*
static int s5k6aafx_i2c_write(struct i2c_client *client, unsigned char length, u8 readdata[])
{
  unsigned char buf[length], i = 0;
  struct i2c_msg msg = {client->addr, I2C_M_WRITE, length, buf};
  int err = 0;

  if (!client->adapter)
  {
    printk(S5K6AAFX_MOD_NAME "can't search i2c client adapter\n");
    return -ENODEV;
  }

  for (i = 0; i < length; i++) 
  {
    buf[i] = readdata[i];
  }

  for (i = 0; i < 10; i++)
  {
    err =  i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if(err == 0) return 0;
    mdelay(1);
  }

  printk(S5K6AAFX_MOD_NAME "s5k6aafx_i2c_write is failed... %d\n", err);

  return err;
}
*/
static void s5k6aafx_set_skip(void)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;

  int skip_frame = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_skip is called...\n");

  if(sensor->state == S5K6AAFX_STATE_PREVIEW)
  {
    if(s5k6aafx_curr_state == S5K6AAFX_STATE_PREVIEW)
    {
      skip_frame = 3;
    }
    else
    {
      //wait for overlay creation (250ms ~ 300ms)
      skip_frame = sensor->fps / 3; 
    }
  }
  else
  {
    skip_frame = 0;//3;
  }
  
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "skip frame = %d frame\n", skip_frame);

  isp_set_hs_vs(0,skip_frame);
}
/*
static int s5k6aafx_set_fps(void)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;
  struct i2c_client *client = sensor->i2c_client; 

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_fps is called... state = %d\n", sensor->state);

  if(sensor->state != S5K6AAFX_STATE_CAPTURE)  
  {
    dprintk(CAM_INF, "s5k6aafx_set_fps is called... size = %d\n", sensor->preview_size);
    dprintk(CAM_INF, "s5k6aafx_set_fps is called... fps = %d\n", sensor->fps);
    
    switch(sensor->fps)
    {
      case 13:   
	  	if(s5k6aafx_write_regs(client, s5k6aafx_vt_13fps, sizeof(s5k6aafx_vt_13fps)/ sizeof(s5k6aafx_vt_13fps[0])))
			goto fps_fail;
		break;		
      case 10:
	  	if(s5k6aafx_write_regs(client, s5k6aafx_vt_10fps, sizeof(s5k6aafx_vt_10fps)/ sizeof(s5k6aafx_vt_10fps[0])))
            goto fps_fail;
        break;   
      case 7:
        if(s5k6aafx_write_regs(client, s5k6aafx_vt_7fps, sizeof(s5k6aafx_vt_7fps)/ sizeof(s5k6aafx_vt_7fps[0])))
            goto fps_fail;           
        break;           
      default:
        printk(S5K6AAFX_MOD_NAME "[fps]Invalid value is ordered!!!\n");
        goto fps_fail;
    }
  }    

  return 0;

fps_fail:
  printk(S5K6AAFX_MOD_NAME "s5k6aafx_set_fps is failed!!!\n");
  return -EINVAL;   
}
*/
static int s5k6aafx_get_rev(void)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;
  struct i2c_client *client = sensor->i2c_client;

  int err =0;
  unsigned short chip_id;  
  
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "--------------------------------------------------\n");
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "   [VGA CAM]   camsensor_s5k6aafx_check_sensor_rev\n");
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "--------------------------------------------------\n");

  err = s5k6aafx_write(client, 0x002cd000);
  err = s5k6aafx_write(client, 0x002e1006);	
  err	= s5k6aafx_read(client, 0x0F12, &chip_id);
  printk("s5k6aafx_get_rev chip_id [%x]\n",chip_id);
  if(err < 0)
    goto get_rev_fail;
	   
  return 0;

get_rev_fail:
  printk(S5K6AAFX_MOD_NAME "s5k6aafx_get_rev is failed!!!\n");
  return -EINVAL;   
}

static int s5k6aafx_set_init(void)
{
	struct s5k6aafx_sensor *sensor = &s5k6aafx;
  	struct i2c_client *client = sensor->i2c_client;
	
	int err = -EINVAL;

#ifdef CONFIG_LOAD_FILE
	if (sensor->mode != S5K6AAFX_MODE_VT)
	{
		err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_common");
	}
	else
	{
		err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_vt_common_QCIF");
	}
#else	
	switch(sensor->preview_size)
	{
		case S5K6AAFX_PREVIEW_SIZE_640_480:
			err = s5k6aafx_write_regs(client, s5k6aafx_common,	sizeof(s5k6aafx_common) / sizeof(s5k6aafx_common[0]));
			break;
		case S5K6AAFX_PREVIEW_SIZE_320_240:
			err = s5k6aafx_write_regs(client, s5k6aafx_vt_common_QVGA, sizeof(s5k6aafx_vt_common_QVGA) / sizeof(s5k6aafx_vt_common_QVGA[0]));
			err = s5k6aafx_write_regs(client, s5k6aafx_vt_13fps, sizeof(s5k6aafx_vt_13fps) / sizeof(s5k6aafx_vt_13fps[0])); 
			break;
		case S5K6AAFX_PREVIEW_SIZE_240_320:
			err = s5k6aafx_write_regs(client, s5k6aafx_vt_common_QVGA_Flip, sizeof(s5k6aafx_vt_common_QVGA_Flip) / sizeof(s5k6aafx_vt_common_QVGA_Flip[0]));
			err = s5k6aafx_write_regs(client, s5k6aafx_vt_13fps, sizeof(s5k6aafx_vt_13fps) / sizeof(s5k6aafx_vt_13fps[0])); 
			break;
		case S5K6AAFX_PREVIEW_SIZE_176_144:
			err = s5k6aafx_write_regs(client, s5k6aafx_vt_common_QCIF, sizeof(s5k6aafx_vt_common_QCIF) / sizeof(s5k6aafx_vt_common_QCIF[0]));
			err = s5k6aafx_write_regs(client, s5k6aafx_vt_13fps, sizeof(s5k6aafx_vt_13fps) / sizeof(s5k6aafx_vt_13fps[0])); 
			break;
		default:
			break;
	}
#endif

	if (unlikely(err)) 
	{
		printk("%s: failed to init\n", __func__);
		return err;
	}
	return 0;
}

static int s5k6aafx_set_mode(s32 value)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_mode is called... mode = %d\n", value);  
  
  sensor->mode = value;
  
  return 0;
}

static int s5k6aafx_set_state(s32 value)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_state is called... state = %d\n", value);  
  
  sensor->state = value;
  
  return 0;
}

static int s5k6aafx_set_ev(s32 value)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;
  struct i2c_client *client = sensor->i2c_client;

  int err;
  
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_ev is called... value = %d\n", value);
#ifdef CONFIG_LOAD_FILE
	switch (value)
	{
		case S5K6AAFX_EV_MINUS_2P0:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_m4");
			break;
		case S5K6AAFX_EV_MINUS_1P5:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_m3");
			break;
		case S5K6AAFX_EV_MINUS_1P0:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_m2");
			break;
		case S5K6AAFX_EV_MINUS_0P5:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_m1");
			break;
		case S5K6AAFX_EV_DEFAULT:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_default");
			break;
		case S5K6AAFX_EV_PLUS_0P5:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_p1");
			break;
		case S5K6AAFX_EV_PLUS_1P0:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_p2");
			break;
		case S5K6AAFX_EV_PLUS_1P5:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_p3");
			break;
		case S5K6AAFX_EV_PLUS_2P0:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_bright_p4");
			break;
		default:
			dev_err(&client->dev, "%s : there's no brightness value with [%d]\n", __func__,value);
			return err;
			break;
	}
#else
  switch (value)
	{
		case S5K6AAFX_EV_MINUS_2P0:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_m4, sizeof(s5k6aafx_bright_m4) / sizeof(s5k6aafx_bright_m4[0]));
			break;
		case S5K6AAFX_EV_MINUS_1P5:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_m3, sizeof(s5k6aafx_bright_m3) / sizeof(s5k6aafx_bright_m3[0]));
			break;
		case S5K6AAFX_EV_MINUS_1P0:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_m2, sizeof(s5k6aafx_bright_m2) / sizeof(s5k6aafx_bright_m2[0]));
			break;
		case S5K6AAFX_EV_MINUS_0P5:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_m1, sizeof(s5k6aafx_bright_m1) / sizeof(s5k6aafx_bright_m1[0]));
			break;
		case S5K6AAFX_EV_DEFAULT:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_default, sizeof(s5k6aafx_bright_default) / sizeof(s5k6aafx_bright_default[0]));
			break;
		case S5K6AAFX_EV_PLUS_0P5:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_p1, sizeof(s5k6aafx_bright_p1) / sizeof(s5k6aafx_bright_p1[0]));
			break;
		case S5K6AAFX_EV_PLUS_1P0:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_p2, sizeof(s5k6aafx_bright_p2) / sizeof(s5k6aafx_bright_p2[0]));
			break;
		case S5K6AAFX_EV_PLUS_1P5:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_p3, sizeof(s5k6aafx_bright_p3) / sizeof(s5k6aafx_bright_p3[0]));
			break;
		case S5K6AAFX_EV_PLUS_2P0:
			err = s5k6aafx_write_regs(client, s5k6aafx_bright_p4, sizeof(s5k6aafx_bright_p4) / sizeof(s5k6aafx_bright_p4[0]));
			break;
		default:
			dev_err(&client->dev, "%s : there's no brightness value with [%d]\n", __func__,value);
			break;
	}
#endif
	if (err < 0)
	{
		dev_err(&client->dev, "%s : i2c_write for set brightness\n", __func__);
		return -EIO;
	}
	
	sensor->ev = value;
	return err;  
}

static int s5k6aafx_set_pretty(s32 value)
{
	struct s5k6aafx_sensor *sensor = &s5k6aafx;
	struct i2c_client *client = sensor->i2c_client;

	int err;

	dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_pretty is called... value = %d\n", value);
#ifdef CONFIG_LOAD_FILE
	switch (value)
	{
		case S5K6AAFX_PRETTY_NONE:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_vt_pretty_default");
			break;
		case S5K6AAFX_PRETTY_LEVEL1:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_vt_pretty_1");
			break;
		case S5K6AAFX_PRETTY_LEVEL2:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_vt_pretty_2");
			break;
		case S5K6AAFX_PRETTY_LEVEL3:
			err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_vt_pretty_3");
			break;
		default:
			dev_err(&client->dev, "%s : there's no blur value with [%d]\n", __func__,value);
			return err;
			break;
	}
#else
	switch (value)
	{
	case S5K6AAFX_PRETTY_NONE:
		err = s5k6aafx_write_regs(client, s5k6aafx_vt_pretty_default, sizeof(s5k6aafx_vt_pretty_default) / sizeof(s5k6aafx_vt_pretty_default[0]));
		break;
	case S5K6AAFX_PRETTY_LEVEL1:
		err = s5k6aafx_write_regs(client, s5k6aafx_vt_pretty_1, sizeof(s5k6aafx_vt_pretty_1) / sizeof(s5k6aafx_vt_pretty_1[0]));
		break;
	case S5K6AAFX_PRETTY_LEVEL2:
		err = s5k6aafx_write_regs(client, s5k6aafx_vt_pretty_2, sizeof(s5k6aafx_vt_pretty_2) / sizeof(s5k6aafx_vt_pretty_2[0]));
		break;
	case S5K6AAFX_PRETTY_LEVEL3:
		err = s5k6aafx_write_regs(client, s5k6aafx_vt_pretty_3, sizeof(s5k6aafx_vt_pretty_3) / sizeof(s5k6aafx_vt_pretty_3[0]));
		break;
	default:
		dev_err(&client->dev, "%s : there's no blur value with [%d]\n", __func__, value);
		return err;
		break;
	}
#endif	
	if (err < 0)
	{
		dev_err(&client->dev, "%s : i2c_write for set blur\n", __func__);
		return -EIO;
	}

	sensor->pretty = value;
	return err;
}

static int s5k6aafx_set_preview_start(void)
{
	struct s5k6aafx_sensor *sensor = &s5k6aafx;
	struct i2c_client *client = sensor->i2c_client;

	int err = -EINVAL;

      dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_set_preview_start is called...\n");   

	if(sensor->check_dataline)		// Output Test Pattern from VGA sensor
	{
	     printk(" pattern on setting~~~~~~~~~~~\n");
	     err = s5k6aafx_write_regs(client, s5k6aafx_pattern_on, sizeof(s5k6aafx_pattern_on) / sizeof(s5k6aafx_pattern_on[0]));
          //  mdelay(200);
	}
	else
	{

	/* set initial regster value */
#ifdef CONFIG_LOAD_FILE
        err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_preview");
#else
        err = s5k6aafx_write_regs(client, s5k6aafx_preview,
			sizeof(s5k6aafx_preview) / sizeof(s5k6aafx_preview[0]));
#endif
        if (unlikely(err)) {
                printk("%s: failed to make preview\n", __func__);
                return err;
            }
	}
/*	
	if(state->set_vhflip == 1)
	{
		err = s5k6aafx_write_regs(sd, s5k6aafx_vhflip_on,
					sizeof(s5k6aafx_vhflip_on) / sizeof(s5k6aafx_vhflip_on[0]));		
	}
*/		
//	sensor->runmode = S5K6AAFX_RUNMODE_RUNNING;
	mdelay(200); // add 200 ms for displaying preview

	return err;
}
static int s5k6aafx_set_capture_start(void)
{
	struct s5k6aafx_sensor *sensor = &s5k6aafx;
	struct i2c_client *client = sensor->i2c_client;
	
	int err = -EINVAL;
	unsigned short light_value = 0;

	s5k6aafx_write(client, 0x002C7000);
	s5k6aafx_write(client, 0x002E1AAA); //read light value
	
	s5k6aafx_read(client, 0x0F12, &light_value);

	printk("%s : light value is %x\n", __func__, light_value);

	/* set initial regster value */
#ifdef CONFIG_LOAD_FILE
	err = s5k6aafx_write_regs_from_sd(client, "s5k6aafx_capture");
#else	
	err = s5k6aafx_write_regs(client, s5k6aafx_capture, sizeof(s5k6aafx_capture) / sizeof(s5k6aafx_capture[0]));
#endif	
	if (light_value < 0x40)
	{
		printk("\n----- low light -----\n\n");
		mdelay(100);//add 100 ms delay for capture sequence
	}
	else
	{
		printk("\n----- normal light -----\n\n");
		mdelay(50);
	}

	if (unlikely(err)) 
	{
		printk("%s: failed to make capture\n", __func__);
		return err;
	}

	return err;
}

static int ioctl_streamoff(struct v4l2_int_device *s)
{
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_streamoff is called...\n");

  return 0;
}

static int ioctl_streamon(struct v4l2_int_device *s)
{
  struct s5k6aafx_sensor *sensor = s->priv;

  int err = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_streamon is called...(%x)\n", sensor->state);   

  if(sensor->state != S5K6AAFX_STATE_CAPTURE)
  {
    printk(S5K6AAFX_MOD_NAME "start preview....................\n");
    s5k6aafx_pre_state = s5k6aafx_curr_state;
    s5k6aafx_curr_state = S5K6AAFX_STATE_PREVIEW;     
	if(s5k6aafx_set_preview_start())
      printk(S5K6AAFX_MOD_NAME "Capture Fail!!!\n");	
  }
  else
  {
    printk(S5K6AAFX_MOD_NAME "start capture....................\n");
    s5k6aafx_pre_state = s5k6aafx_curr_state;
    s5k6aafx_curr_state = S5K6AAFX_STATE_CAPTURE;
	if(s5k6aafx_set_capture_start())
      printk(S5K6AAFX_MOD_NAME "Capture Fail!!!\n");
  }
  
  return err;
}


/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the s5k6aafx_ctrl_list[] array.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
  int i;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_queryctrl is called...\n");

  for (i = 0; i < NUM_S5K6AAFX_CONTROL; i++) 
  {
    if (qc->id == s5k6aafx_ctrl_list[i].id)
    {
      break;
    }
  }
  if (i == NUM_S5K6AAFX_CONTROL)
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "Control ID is not supported!!\n");
    qc->flags = V4L2_CTRL_FLAG_DISABLED;

    return -EINVAL;
  }

  *qc = s5k6aafx_ctrl_list[i];

  return 0;
}

static int s5k6aafx_check_dataline_stop(void)
{
	struct s5k6aafx_sensor *sensor = &s5k6aafx;
	struct i2c_client *client = sensor->i2c_client;
	int err = -EINVAL;
	err = 0;
      dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_check_dataline_stop is called...[%x]\n",sensor->check_dataline);   
#if 0
	for (i = 0; i < 2; i++) {
		err = s5k6aafx_i2c_write(client, sizeof(s5k6aafx_dataline_stop[i]), s5k6aafx_dataline_stop[i]);
		if (err < 0)
		{
			v4l_info(client, "%s: register set failed\n", __func__);
			return -EIO;
		}
	}
#endif
	s5k6aafx_write(client, 0xFCFCD000);
	s5k6aafx_write(client, 0x0028D000);
	s5k6aafx_write(client, 0x002A3100);
    	s5k6aafx_write(client, 0x0F120000);


	s5k6aafx_write_regs(client, s5k6aafx_pattern_off, sizeof(s5k6aafx_pattern_off) / sizeof(s5k6aafx_pattern_off[0]));
#if 0
	sensor->pdata->power_set(V4L2_POWER_OFF);
	mdelay(5);
	sensor->pdata->power_set(V4L2_POWER_ON);
	mdelay(5);
#endif
	
	sensor->check_dataline = 0;	
	s5k6aafx_set_preview_start();
#if 0
	err =  s5k6aafx_write_regs(client, s5k6aafx_common,	sizeof(s5k6aafx_common) / sizeof(s5k6aafx_common[0]));
	if (err < 0)
	{
		v4l_info(client, "%s: register set failed\n", __func__);
		return -EIO;
	}
#endif
	return err;
}


/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the ce13 sensor struct.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
  struct s5k6aafx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_g_ctrl is called...(%x)\n", vc->id);

  switch (vc->id) 
  {
    case V4L2_CID_AF:
      vc->value = 2;
      break; 
    case V4L2_CID_SELECT_MODE:
      vc->value = sensor->mode;
      break;  
    case V4L2_CID_SELECT_STATE:
      vc->value = sensor->state;
      break;       
    case V4L2_CID_BRIGHTNESS:
      vc->value = sensor->ev;
      break;
    case V4L2_CID_PRETTY:
      vc->value = sensor->pretty;
      break;    
    default:
       printk(S5K6AAFX_MOD_NAME "[%s:%d] vc->id : %x, Invalid value is ordered!!!\n", 
		       __func__, __LINE__, vc->id);
      break;
  }

  return 0;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the s5k6aafx sensor struct).
 * Otherwise, * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;
  int retval = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_s_ctrl is called...(%x)\n", vc->id);

	if(sensor->check_dataline)
	{
		if(/* ( vc->id != V4L2_CID_CAM_PREVIEW_ONOFF ) &&*/
			( vc->id != V4L2_CID_CAMERA_CHECK_DATALINE_STOP ) &&
			( vc->id != V4L2_CID_CAMERA_CHECK_DATALINE ) )
			{
				return 0;
			}
	} 
  switch (vc->id) 
  {
    case V4L2_CID_SELECT_MODE:
      retval = s5k6aafx_set_mode(vc->value);
      break;  
    case V4L2_CID_SELECT_STATE:
      retval = s5k6aafx_set_state(vc->value);
      break;        
    case V4L2_CID_BRIGHTNESS:
      retval = s5k6aafx_set_ev(vc->value);
	  break;
    case V4L2_CID_PRETTY:
      retval = s5k6aafx_set_pretty(vc->value);
      break;    
	case V4L2_CID_CAMERA_CHECK_DATALINE:
	  sensor->check_dataline = vc->value;
	  retval = 0;
	  break;	
	case V4L2_CID_CAMERA_CHECK_DATALINE_STOP:
	  retval = s5k6aafx_check_dataline_stop();
	  break;	  
    default:
       printk(S5K6AAFX_MOD_NAME "[%s:%d] vc->id : %x, Invalid value is ordered!!!\n", 
		       __func__, __LINE__, vc->id);
      break;
  }

  return retval;
}


/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
  int index = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_enum_fmt_cap is called...\n");

  switch (fmt->type) 
  {
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
      switch(fmt->pixelformat)
      {
        case V4L2_PIX_FMT_UYVY:
          index = 0;
          break;

        case V4L2_PIX_FMT_YUYV:
          index = 1;
          break;

        case V4L2_PIX_FMT_JPEG:
          index = 2;
          break;

        case V4L2_PIX_FMT_MJPEG:
          index = 3;
          break;

        default:
           printk(S5K6AAFX_MOD_NAME "[format]Invalid value is ordered!!!\n");
          return -EINVAL;
      }
      break;
      
    default:
       printk(S5K6AAFX_MOD_NAME "[type]Invalid value is ordered!!!\n");
      return -EINVAL;
  }

  fmt->flags = s5k6aafx_formats[index].flags;
  fmt->pixelformat = s5k6aafx_formats[index].pixelformat;
  strlcpy(fmt->description, s5k6aafx_formats[index].description, sizeof(fmt->description));  

  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_enum_fmt_cap flag : %d\n", fmt->flags);
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_enum_fmt_cap description : %s\n", fmt->description);

  return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
  struct v4l2_pix_format *pix = &f->fmt.pix;
  struct s5k6aafx_sensor *sensor = s->priv;
  struct v4l2_pix_format *pix2 = &sensor->pix;

  int index = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_try_fmt_cap is called...\n");
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_try_fmt_cap. mode : %d\n", sensor->mode);
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_try_fmt_cap. state : %d\n", sensor->state);
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_try_fmt_cap. pix width : %d\n", pix->width);
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_try_fmt_cap. pix height : %d\n", pix->height);    

  s5k6aafx_set_skip();

  if(sensor->state == S5K6AAFX_STATE_CAPTURE)
  {
    for(index = 0; index < ARRAY_SIZE(s5k6aafx_image_sizes); index++)
    {
      if(s5k6aafx_image_sizes[index].width == pix->width
      && s5k6aafx_image_sizes[index].height == pix->height)
      {
        sensor->capture_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5k6aafx_image_sizes))
    {
      printk(S5K6AAFX_MOD_NAME "Capture Image Size is not supported!\n");
      goto try_fmt_fail;
    }  

    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "capture size = %d\n", sensor->capture_size);  
    
    pix->field = V4L2_FIELD_NONE;
    if(pix->pixelformat == V4L2_PIX_FMT_UYVY || pix->pixelformat == V4L2_PIX_FMT_YUYV)
    {
      pix->bytesperline = pix->width * 2;
      pix->sizeimage = pix->bytesperline * pix->height;
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "V4L2_PIX_FMT_YUYV\n");
    }
    else
    {
      /* paladin[08.10.14]: For JPEG Capture, use fixed buffer size @LDK@ */
      pix->bytesperline = YUV_CAPTURE_WIDTH * 2; /***** !!!fixme mingyu diffent sensor!!!**********/
      pix->sizeimage = pix->bytesperline * YUV_CAPTURE_HEIGHT;
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "V4L2_PIX_FMT_JPEG\n");
    }

    if(s5k6aafx_curr_state == S5K6AAFX_STATE_INVALID)
    {
      if(s5k6aafx_set_init())
      {
        printk(S5K6AAFX_MOD_NAME "Unable to detect " S5K6AAFX_DRIVER_NAME " sensor\n");
        goto try_fmt_fail;
      }
    }     
  }  

  switch (pix->pixelformat) 
  {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_JPEG:
    case V4L2_PIX_FMT_MJPEG:
      pix->colorspace = V4L2_COLORSPACE_JPEG;
      break;
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB565X:
    case V4L2_PIX_FMT_RGB555:
    case V4L2_PIX_FMT_SGRBG10:
    case V4L2_PIX_FMT_RGB555X:
    default:
      pix->colorspace = V4L2_COLORSPACE_SRGB;
      break;
  }

  *pix2 = *pix;

  return 0;

try_fmt_fail:
  printk(S5K6AAFX_MOD_NAME "ioctl_try_fmt_cap is failed\n"); 
  return -EINVAL;  
}

/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
  struct v4l2_pix_format *pix = &f->fmt.pix;
  struct s5k6aafx_sensor *sensor = s->priv;
  struct v4l2_pix_format *pix2 = &sensor->pix;

  int index = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_s_fmt_cap is called...\n");
  
  printk(S5K6AAFX_MOD_NAME "camera mode  : %d (1:camera , 2:camcorder, 3:vt)\n", sensor->mode);
  printk(S5K6AAFX_MOD_NAME "camera state : %d (0:preview, 1:snapshot)\n", sensor->state);
  printk(S5K6AAFX_MOD_NAME "set width  : %d\n", pix->width);
  printk(S5K6AAFX_MOD_NAME "set height  : %d\n", pix->height);    

  if(sensor->state == S5K6AAFX_STATE_CAPTURE)
  {
    s5k6aafx_set_skip();
  
    for(index = 0; index < ARRAY_SIZE(s5k6aafx_image_sizes); index++)
    {
      if(s5k6aafx_image_sizes[index].width == pix->width
      && s5k6aafx_image_sizes[index].height == pix->height)
      {
        sensor->capture_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5k6aafx_image_sizes))
    {
      printk(S5K6AAFX_MOD_NAME "Capture Image %d x %d Size is not supported!\n", pix->width, pix->height);
      goto s_fmt_fail;
    }  

    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "capture size = %d\n", sensor->capture_size);  
    
    pix->field = V4L2_FIELD_NONE;
    if(pix->pixelformat == V4L2_PIX_FMT_UYVY || pix->pixelformat == V4L2_PIX_FMT_YUYV)
    {
      pix->bytesperline = pix->width * 2;
      pix->sizeimage = pix->bytesperline * pix->height;
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "V4L2_PIX_FMT_YUYV\n");
    }
    else
    {
      /* paladin[08.10.14]: For JPEG Capture, use fixed buffer size @LDK@ */
      pix->bytesperline = YUV_CAPTURE_WIDTH * 2; /***** !!!fixme mingyu diffent sensor!!!**********/
      pix->sizeimage = pix->bytesperline * YUV_CAPTURE_HEIGHT;
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "V4L2_PIX_FMT_JPEG\n");
    }

    if(s5k6aafx_curr_state == S5K6AAFX_STATE_INVALID)
    {
      if(s5k6aafx_set_init())
      {
        printk(S5K6AAFX_MOD_NAME "Unable to detect " S5K6AAFX_DRIVER_NAME " sensor\n");
        goto s_fmt_fail;
      }
    }     
  }  
  else
  {
    s5k6aafx_set_skip();
  
    for(index = 0; index < ARRAY_SIZE(s5k6aafx_preview_sizes); index++)
    {
      if(s5k6aafx_preview_sizes[index].width == pix->width
      && s5k6aafx_preview_sizes[index].height == pix->height)
      {
        sensor->preview_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5k6aafx_preview_sizes))
    {
      printk(S5K6AAFX_MOD_NAME "Preview Image %d x %d Size is not supported!\n", pix->width, pix->height);
      goto s_fmt_fail;
    }
  
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "preview size = %d\n", sensor->preview_size);
    
    pix->field = V4L2_FIELD_NONE;
    pix->bytesperline = pix->width * 2;
    pix->sizeimage = pix->bytesperline * pix->height;  

    if(s5k6aafx_curr_state == S5K6AAFX_STATE_INVALID)
    {
      if(s5k6aafx_set_init())
      {
        printk(S5K6AAFX_MOD_NAME "Unable to detect " S5K6AAFX_DRIVER_NAME " sensor\n");
        goto s_fmt_fail;
      }
    }    
  }      

  switch (pix->pixelformat) 
  {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_JPEG:
    case V4L2_PIX_FMT_MJPEG:
      pix->colorspace = V4L2_COLORSPACE_JPEG;
      break;
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB565X:
    case V4L2_PIX_FMT_RGB555:
    case V4L2_PIX_FMT_SGRBG10:
    case V4L2_PIX_FMT_RGB555X:
    default:
      pix->colorspace = V4L2_COLORSPACE_SRGB;
      break;
  }

  *pix2 = *pix;

  return 0;

s_fmt_fail:
  printk(S5K6AAFX_MOD_NAME "ioctl_s_fmt_cap is failed\n"); 
  return -EINVAL;    
}


/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
  struct s5k6aafx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_g_fmt_cap is called...\n");
  
  f->fmt.pix = sensor->pix;

  return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
  struct s5k6aafx_sensor *sensor = s->priv;
  struct v4l2_captureparm *cparm = &a->parm.capture;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_g_parm is called...\n");

  if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
  {
    return -EINVAL;
  }

  memset(a, 0, sizeof(*a));
  a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  cparm->capability = V4L2_CAP_TIMEPERFRAME;
  cparm->timeperframe = sensor->timeperframe;

  return 0;
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
  struct s5k6aafx_sensor *sensor = s->priv;
  struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_s_parm is called...\n");

  /* Set mode (camera/camcorder/vt) & state (preview/capture) */
  sensor->mode = a->parm.capture.capturemode;
  sensor->state = a->parm.capture.currentstate;

  if(sensor->mode < 1 || sensor->mode > 3) sensor->mode = S5K6AAFX_MODE_CAMERA;
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "mode = %d, state = %d\n", sensor->mode, sensor->state); 

  /* Set time per frame (FPS) */
  if((timeperframe->numerator == 0)&&(timeperframe->denominator == 0))
  {
    sensor->fps = 15;
  }
  else
  {
    sensor->fps = timeperframe->denominator / timeperframe->numerator;
  }

  sensor->timeperframe = *timeperframe;
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "fps = %d\n", sensor->fps);  
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "numerator : %d, denominator: %d\n", timeperframe->numerator, timeperframe->denominator);
  
  return 0;
}

/**
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
  struct s5k6aafx_sensor *sensor = s->priv;
  int rval;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_g_ifparm is called...\n");

  rval = sensor->pdata->ifparm(p);
  if (rval)
  {
    return rval;
  }

  p->u.bt656.clock_curr = S5K6AAFX_XCLK;

  return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
  struct s5k6aafx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_g_priv is called...\n");
  
  return sensor->pdata->priv_data_set(p);
}


/* added following functins for v4l2 compatibility with omap34xxcam */

/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s, struct v4l2_frmsizeenum *frms)
{
  struct s5k6aafx_sensor* sensor = s->priv;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_enum_framesizes fmt\n");   

  if (sensor->state == S5K6AAFX_STATE_CAPTURE)
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "Size enumeration for image capture = %d\n", sensor->capture_size);
  
    frms->index = sensor->capture_size;
    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = s5k6aafx_image_sizes[sensor->capture_size].width;
    frms->discrete.height = s5k6aafx_image_sizes[sensor->capture_size].height;       
  }
  else
  {    
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "Size enumeration for preview = %d\n", sensor->preview_size);
    
    frms->index = sensor->preview_size;
    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = s5k6aafx_preview_sizes[sensor->preview_size].width;
    frms->discrete.height = s5k6aafx_preview_sizes[sensor->preview_size].height;       
  }

  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "framesizes width : %d\n", frms->discrete.width); 
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "framesizes height : %d\n", frms->discrete.height); 

  return 0;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *frmi)
{
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_enum_frameintervals \n"); 
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_enum_frameintervals numerator : %d\n", frmi->discrete.numerator); 
  dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "ioctl_enum_frameintervals denominator : %d\n", frmi->discrete.denominator); 

  return 0;
}

extern void force_disable_tsp_autocal(void);

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
  struct s5k6aafx_sensor *sensor = s->priv;

  int err = 0;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_s_power is called......ON=%x, detect= %x\n", on, sensor->detect);

  sensor->pdata->power_set(on);

  switch(on)
  {
    case V4L2_POWER_ON:
    {
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "pwr on-----!\n");
	  front_cam_in_use = 1;
      err = s5k6aafx_get_rev();
      if(err)
      {
		front_cam_in_use = 0;
        printk(S5K6AAFX_MOD_NAME "Unable to detect " S5K6AAFX_DRIVER_NAME " sensor\n");
        sensor->pdata->power_set(V4L2_POWER_OFF);
        return err;
      }

      /* Make the default zoom */
      sensor->zoom = S5K6AAFX_ZOOM_1P00X;      

      /* Make the default detect */
      sensor->detect = SENSOR_DETECTED;

      /* Make the state init */
      s5k6aafx_curr_state = S5K6AAFX_STATE_INVALID;      
      force_disable_tsp_autocal();	  
    }
    break;

    case V4L2_POWER_RESUME:
    {
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "pwr resume-----!\n");
    }
    break;

    case V4L2_POWER_STANDBY:
    {
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "pwr stanby-----!\n");
    }
    break;

    case V4L2_POWER_OFF:
    {
      dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "pwr off-----!\n");

      /* Make the default zoom */
      sensor->zoom = S5K6AAFX_ZOOM_1P00X;

      /* Make the default detect */
      sensor->detect = SENSOR_NOT_DETECTED;
      
      /* Make the state init */
      s5k6aafx_pre_state = S5K6AAFX_STATE_INVALID;       
    }
    break;
  }

  return err;
}

static int ioctl_g_exif(struct v4l2_int_device *s, struct v4l2_exif *exif)
{
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_g_exif is called...\n");

  return 0;
}


/**
 * ioctl_deinit - V4L2 sensor interface handler for VIDIOC_INT_DEINIT
 * @s: pointer to standard V4L2 device structure
 *
 * Deinitialize the sensor device
 */
static int ioctl_deinit(struct v4l2_int_device *s)
{
  struct s5k6aafx_sensor *sensor = s->priv;
  
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_init is called...\n");

  sensor->state = S5K6AAFX_STATE_INVALID; //init problem
  
  return 0;
}


/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call s5k6aafx_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
  struct s5k6aafx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "ioctl_init is called...\n");

  //init value
  sensor->timeperframe.numerator    = 1,
  sensor->timeperframe.denominator  = 15,
  sensor->mode                      = S5K6AAFX_MODE_CAMERA;
  sensor->state                     = S5K6AAFX_STATE_INVALID;
  sensor->fps                       = 15;
  sensor->preview_size              = S5K6AAFX_PREVIEW_SIZE_640_480;
  sensor->capture_size              = S5K6AAFX_IMAGE_SIZE_640_480;
  sensor->detect                    = SENSOR_NOT_DETECTED;
  sensor->zoom                      = S5K6AAFX_ZOOM_1P00X;
  sensor->effect                    = S5K6AAFX_EFFECT_OFF;
  sensor->ev                        = S5K6AAFX_EV_DEFAULT;
  sensor->wb                        = S5K6AAFX_WB_AUTO;
  sensor->pretty                    = S5K6AAFX_PRETTY_NONE;
  sensor->flip                      = S5K6AAFX_FLIP_NONE;

  memcpy(&s5k6aafx, sensor, sizeof(struct s5k6aafx_sensor));
  
  return 0;
}

static struct v4l2_int_ioctl_desc s5k6aafx_ioctl_desc[] = {
  { .num = vidioc_int_enum_framesizes_num,
    .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
  { .num = vidioc_int_enum_frameintervals_num,
    .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
  { .num = vidioc_int_s_power_num,
    .func = (v4l2_int_ioctl_func *)ioctl_s_power },
  { .num = vidioc_int_g_priv_num,
    .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
  { .num = vidioc_int_g_ifparm_num,
    .func = (v4l2_int_ioctl_func *)ioctl_g_ifparm },
  { .num = vidioc_int_init_num,
    .func = (v4l2_int_ioctl_func *)ioctl_init },
  { .num = vidioc_int_deinit_num,
    .func = (v4l2_int_ioctl_func *)ioctl_deinit },
  { .num = vidioc_int_enum_fmt_cap_num,
    .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
  { .num = vidioc_int_try_fmt_cap_num,
    .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
  { .num = vidioc_int_g_fmt_cap_num,
    .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
  { .num = vidioc_int_s_fmt_cap_num,
    .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
  { .num = vidioc_int_g_parm_num,
    .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
  { .num = vidioc_int_s_parm_num,
    .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
  { .num = vidioc_int_queryctrl_num,
    .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
  { .num = vidioc_int_g_ctrl_num,
    .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
  { .num = vidioc_int_s_ctrl_num,
    .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
  { .num = vidioc_int_streamon_num,
    .func = (v4l2_int_ioctl_func *)ioctl_streamon },
  { .num = vidioc_int_streamoff_num,
    .func = (v4l2_int_ioctl_func *)ioctl_streamoff },
  { .num = vidioc_int_g_exif_num,
    .func = (v4l2_int_ioctl_func *)ioctl_g_exif },     
};

static struct v4l2_int_slave s5k6aafx_slave = {
  .ioctls = s5k6aafx_ioctl_desc,
  .num_ioctls = ARRAY_SIZE(s5k6aafx_ioctl_desc),
};

static struct v4l2_int_device s5k6aafx_int_device = {
  .module = THIS_MODULE,
  .name = S5K6AAFX_DRIVER_NAME,
  .priv = &s5k6aafx,
  .type = v4l2_int_type_slave,
  .u = {
    .slave = &s5k6aafx_slave,
  },
};


/**
 * s5k6aafx_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
s5k6aafx_probe(struct i2c_client *client, const struct i2c_device_id *device)
{
  struct s5k6aafx_sensor *sensor = &s5k6aafx;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_probe is called...\n");

  if (i2c_get_clientdata(client))
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "can't get i2c client data!!");
    return -EBUSY;
  }

  sensor->pdata = &nowplus_s5k6aafx_platform_data;

  if (!sensor->pdata) 
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "no platform data?\n");
    return -ENODEV;
  }
  client->addr = 0x3C;
  sensor->v4l2_int_device = &s5k6aafx_int_device;
  sensor->i2c_client = client;

  /* Make the default capture size VGA */
  sensor->pix.width = 1280;
  sensor->pix.height = 960;

  /* Make the default capture format V4L2_PIX_FMT_UYVY */
  sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;

  i2c_set_clientdata(client, sensor);

  if (v4l2_int_device_register(sensor->v4l2_int_device))
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "fail to init device register \n");
    i2c_set_clientdata(client, NULL);
  }

  return 0;
}

/**
 * s5k6aafx_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of s5k6aafx_probe().
 */
static int __exit
s5k6aafx_remove(struct i2c_client *client)
{
  struct s5k6aafx_sensor *sensor = i2c_get_clientdata(client);

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_remove is called...\n");

  if (!client->adapter)
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "no i2c client adapter!!");
    return -ENODEV; /* our client isn't attached */
  }

  v4l2_int_device_unregister(sensor->v4l2_int_device);
  i2c_set_clientdata(client, NULL);

  return 0;
}

static const struct i2c_device_id s5k6aafx_id[] = {
  { S5K6AAFX_DRIVER_NAME, 0 },
  { },
};

MODULE_DEVICE_TABLE(i2c, s5k6aafx_id);


static struct i2c_driver s5k6aafxsensor_i2c_driver = {
  .driver = {
    .name = S5K6AAFX_DRIVER_NAME,
  },
  .probe = s5k6aafx_probe,
  .remove = __exit_p(s5k6aafx_remove),
  .id_table = s5k6aafx_id,
};

/**
 * s5k6aafx_sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init s5k6aafx_sensor_init(void)
{
  int err;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_sensor_init is called...\n");

  err = i2c_add_driver(&s5k6aafxsensor_i2c_driver);
  if (err) 
  {
    dprintk(CAM_DBG, S5K6AAFX_MOD_NAME "Failed to register" S5K6AAFX_DRIVER_NAME ".\n");
    return err;
  }
  
  return 0;
}

module_init(s5k6aafx_sensor_init);

/**
 * s5k6aafxsensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of s5k6aafx_sensor_init.
 */
static void __exit s5k6aafxsensor_cleanup(void)
{
  i2c_del_driver(&s5k6aafxsensor_i2c_driver);
}
module_exit(s5k6aafxsensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5K6AAFX camera sensor driver");

