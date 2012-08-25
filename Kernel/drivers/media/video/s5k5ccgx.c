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
 * modules/camera/s5k5ccgx.c
 *
 * S5K5CCGX sensor driver source file
 *
 * Modified by paladin in Samsung Electronics
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-subdev.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>

#include "isp/isp.h"
#include "omap34xxcam.h"
#include "s5k5ccgx.h"

//#define CONFIG_LOAD_FILE // VE_GROUP TODO
#if defined(S5K5CCGX_USE_GPIO_I2C)
    #include <plat/i2c-omap-gpio.h>
    static OMAP_GPIO_I2C_CLIENT * s5k5ccgx_i2c_client;	
#endif  

#if (CAM_S5K5CCGX_DBG_MSG)
#include "dprintk.h"
#else
#define dprintk(x, y...)
#endif
#ifdef S5K5CCGX_FLASH_SUPPORT
//static int flash_mode = 1; 
static int flash_check = 0; //default is FLASH OFF - Auto on/off // VE_GROUP
static int flash_auto_check = 0; 
#endif
static int afcanceled = 0;
static int preflash = 0;
//static int mainflash = 0;

#if defined(S5K5CCGX_TOUCH_AF) // VE_GROUP
static int g_touch_enter = 0;
#endif

#define OMAP_GPIO_CAM_SCL			161
#define OMAP_GPIO_CAM_SDA			162

#define I2C_M_WRITE 0x0000 /* write data, from slave to master */
#define I2C_M_READ  0x0001 /* read data, from slave to master */

static u32 s5k5ccgx_curr_state = S5K5CCGX_STATE_INVALID;
static u32 s5k5ccgx_pre_state = S5K5CCGX_STATE_INVALID;

static bool s5k5ccgx_720p_enable = false;
bool back_cam_in_use = false;

static struct s5k5ccgx_sensor s5k5ccgx = {
  .timeperframe = {
    .numerator    = 1,
    .denominator  = 30,
  },
  .fps            = 30,
  .bv             = 0,
  .state          = S5K5CCGX_STATE_PREVIEW,
  .mode           = S5K5CCGX_MODE_CAMERA,
  .preview_size   = S5K5CCGX_PREVIEW_SIZE_640_480,
  .capture_size   = S5K5CCGX_IMAGE_SIZE_2560_1920,
  .detect         = SENSOR_NOT_DETECTED,
  .focus_mode     = S5K5CCGX_AF_INIT_NORMAL,
  .effect         = S5K5CCGX_EFFECT_OFF,
  .iso            = S5K5CCGX_ISO_AUTO,
  .photometry     = S5K5CCGX_PHOTOMETRY_CENTER,
  .ev             = S5K5CCGX_EV_DEFAULT,
  .wdr            = S5K5CCGX_WDR_OFF,
  .contrast       = S5K5CCGX_CONTRAST_DEFAULT,
  .saturation     = S5K5CCGX_SATURATION_DEFAULT,
  .sharpness      = S5K5CCGX_SHARPNESS_DEFAULT,
  .wb             = S5K5CCGX_WB_AUTO,
  .isc            = S5K5CCGX_ISC_STILL_OFF,
  .scene          = S5K5CCGX_SCENE_OFF,
  .aewb           = S5K5CCGX_AE_UNLOCK_AWB_UNLOCK,
  .antishake      = S5K5CCGX_ANTI_SHAKE_OFF,
  .flash_capture  = S5K5CCGX_FLASH_CAPTURE_OFF,
  .flash_movie    = S5K5CCGX_FLASH_MOVIE_OFF,
  .jpeg_quality   = S5K5CCGX_JPEG_SUPERFINE, 
  .zoom           = S5K5CCGX_ZOOM_1P00X,
  .thumb_offset   = S5K5CCGX_THUMBNAIL_OFFSET,
  .yuv_offset     = S5K5CCGX_YUV_OFFSET,
  .jpeg_capture_w = JPEG_CAPTURE_WIDTH,
  .jpeg_capture_h = JPEG_CAPTURE_HEIGHT,
  .flash_gpio_state = 0,  
};

struct v4l2_queryctrl s5k5ccgx_ctrl_list[] = {
  {
    .id            = V4L2_CID_SELECT_MODE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "select mode",
    .minimum       = S5K5CCGX_MODE_CAMERA,
    .maximum       = S5K5CCGX_MODE_VT,
    .step          = 1,
    .default_value = S5K5CCGX_MODE_CAMERA,
  },    
  {
    .id            = V4L2_CID_SELECT_STATE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "select state",
    .minimum       = S5K5CCGX_STATE_PREVIEW,
    .maximum       = S5K5CCGX_STATE_CAPTURE,
    .step          = 1,
    .default_value = S5K5CCGX_STATE_PREVIEW,
  },    
  {
    .id            = V4L2_CID_FOCUS_MODE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Focus Mode",
    .minimum       = S5K5CCGX_AF_INIT_NORMAL,
    .maximum       = S5K5CCGX_AF_INIT_FACE,
    .step          = 1,
    .default_value = S5K5CCGX_AF_INIT_NORMAL,
  },
  {
    .id            = V4L2_CID_AF,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Focus Status",
    .minimum       = S5K5CCGX_AF_START,
    .maximum       = S5K5CCGX_AF_STOP,
    .step          = 1,
    .default_value = S5K5CCGX_AF_STOP,
  },
  {
    .id            = V4L2_CID_ZOOM,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Digital Zoom",
    .minimum       = S5K5CCGX_ZOOM_1P00X,
    .maximum       = S5K5CCGX_ZOOM_4P00X,
    .step          = 1,
    .default_value = S5K5CCGX_ZOOM_1P00X,
  },
  {
    .id            = V4L2_CID_JPEG_TRANSFER,
    .name          = "Request JPEG Transfer",
  },
  {
    .id            = V4L2_CID_JPEG_QUALITY,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "JPEG Quality",
    .minimum       = S5K5CCGX_JPEG_SUPERFINE,
    .maximum       = S5K5CCGX_JPEG_ECONOMY,
    .step          = 1,
    .default_value = S5K5CCGX_JPEG_SUPERFINE,
  },
  {
    .id            = V4L2_CID_ISO,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "ISO",
    .minimum       = S5K5CCGX_ISO_AUTO,
    .maximum       = S5K5CCGX_ISO_800,
    .step          = 1,
    .default_value = S5K5CCGX_ISO_AUTO,
  },
  {
    .id            = V4L2_CID_BRIGHTNESS,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Brightness",
    .minimum       = S5K5CCGX_EV_MINUS_2P0,
    .maximum       = S5K5CCGX_EV_PLUS_2P0,
    .step          = 1,
    .default_value = S5K5CCGX_EV_DEFAULT,
  },
  {
    .id            = V4L2_CID_WB,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "White Balance",
    .minimum       = S5K5CCGX_WB_AUTO,
    .maximum       = S5K5CCGX_WB_FLUORESCENT,
    .step          = 1,
    .default_value = S5K5CCGX_WB_AUTO,
  },
  {
    .id            = V4L2_CID_CONTRAST,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Contrast",
    .minimum       = S5K5CCGX_CONTRAST_MINUS_3,
    .maximum       = S5K5CCGX_CONTRAST_PLUS_3,
    .step          = 1,
    .default_value = S5K5CCGX_CONTRAST_DEFAULT,
  },  
  {
    .id            = V4L2_CID_SATURATION,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Saturation",
    .minimum       = S5K5CCGX_SATURATION_MINUS_3,
    .maximum       = S5K5CCGX_SATURATION_PLUS_3,
    .step          = 1,
    .default_value = S5K5CCGX_SATURATION_DEFAULT,
  },
  {
    .id            = V4L2_CID_EFFECT,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Effect",
    .minimum       = S5K5CCGX_EFFECT_OFF,
    .maximum       = S5K5CCGX_EFFECT_PURPLE,
    .step          = 1,
    .default_value = S5K5CCGX_EFFECT_OFF,
  },
  {
    .id            = V4L2_CID_SCENE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Scene",
    .minimum       = S5K5CCGX_SCENE_OFF,
    .maximum       = S5K5CCGX_SCENE_FIREWORKS,
    .step          = 1,
    .default_value = S5K5CCGX_SCENE_OFF,
  },
  {
    .id            = V4L2_CID_PHOTOMETRY,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Photometry",
    .minimum       = S5K5CCGX_PHOTOMETRY_CENTER,
    .maximum       = S5K5CCGX_PHOTOMETRY_MATRIX,
    .step          = 1,
    .default_value = S5K5CCGX_PHOTOMETRY_CENTER,
  },
  {
    .id            = V4L2_CID_WDR,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Wide Dynamic Range",
    .minimum       = S5K5CCGX_WDR_OFF,
    .maximum       = S5K5CCGX_WDR_AUTO,
    .step          = 1,
    .default_value = S5K5CCGX_WDR_OFF,
  },
  {
    .id            = V4L2_CID_SHARPNESS,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Sharpness",
    .minimum       = S5K5CCGX_SHARPNESS_MINUS_3,
    .maximum       = S5K5CCGX_SHARPNESS_PLUS_3,
    .step          = 1,
    .default_value = S5K5CCGX_SHARPNESS_DEFAULT,
  },
  {
    .id            = V4L2_CID_ISC,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Image Stabilization",
    .minimum       = S5K5CCGX_ISC_STILL_OFF,
    .maximum       = S5K5CCGX_ISC_MOVIE_ON,
    .step          = 1,
    .default_value = S5K5CCGX_ISC_STILL_OFF,
  },
  {
    .id            = V4L2_CID_AEWB,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Auto Exposure/Auto White Balance",
    .minimum       = S5K5CCGX_AE_LOCK_AWB_LOCK,
    .maximum       = S5K5CCGX_AE_UNLOCK_AWB_UNLOCK,
    .step          = 1,
    .default_value = S5K5CCGX_AE_UNLOCK_AWB_UNLOCK,
  },
  {
    .id            = V4L2_CID_ANTISHAKE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Anti Shake Setting",
    .minimum       = S5K5CCGX_ANTI_SHAKE_OFF,
    .maximum       = S5K5CCGX_ANTI_SHAKE_ON,
    .step          = 1,
    .default_value = S5K5CCGX_ANTI_SHAKE_OFF,
  },  
  {
    .id            = V4L2_CID_FW_UPDATE,
    .name          = "Firmware Update",
  },
  {
    .id            = V4L2_CID_FLASH_CAPTURE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Capture Flash Setting",
    .minimum       = S5K5CCGX_FLASH_CAPTURE_OFF,
    .maximum       = S5K5CCGX_FLASH_CAPTURE_AUTO,
    .step          = 1,
    .default_value = S5K5CCGX_FLASH_CAPTURE_OFF,
  },
  {
    .id            = V4L2_CID_FLASH_MOVIE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Movie Flash Setting",
    .minimum       = S5K5CCGX_FLASH_MOVIE_OFF,
    .maximum       = S5K5CCGX_FLASH_MOVIE_ON,
    .step          = 1,
    .default_value = S5K5CCGX_FLASH_MOVIE_OFF,
  },
  {
    .id            = V4L2_CID_JPEG_SIZE,
    .name          = "JPEG Image Size",
    .flags         = V4L2_CTRL_FLAG_READ_ONLY,
  },
  {
    .id            = V4L2_CID_THUMBNAIL_SIZE,
    .name          = "Thumbnail Image Size",
    .flags         = V4L2_CTRL_FLAG_READ_ONLY,
  },
};
#define NUM_S5K5CCGX_CONTROL ARRAY_SIZE(s5k5ccgx_ctrl_list)

/* list of image formats supported by s5k5ccgx sensor */
const static struct v4l2_fmtdesc s5k5ccgx_formats[] = {
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
#define NUM_S5K5CCGX_FORMATS ARRAY_SIZE(s5k5ccgx_formats)

extern struct s5k5ccgx_platform_data nowplus_s5k5ccgx_platform_data;

u8 ISP_FW_ver[4] = {0x00,};

/*
static int s5k5ccgx_write_read_reg(struct i2c_client *client, u8 writedata_num, const u8* writedata, u8 readdata_num, u8* readdata)
{
  int err, i;
  struct i2c_msg msg[1];
  u8 writebuf[writedata_num];
  u8 readbuf[readdata_num];

  if (!client->adapter) 
  {
    return -ENODEV;
  }

  // Write
  msg->addr  = client->addr;
  msg->flags = I2C_M_WRITE;
  msg->len   = writedata_num;
  memcpy(writebuf, writedata, writedata_num);
  msg->buf   = writebuf;

#if (CAM_S5K5CCGX_I2C_DBG_MSG)
  for (i = 0; i < writedata_num; i++) 
  {
    printk("=0x%02x=\n",writedata[i]);
  }
#endif

  for(i = 0; i < 10; i++)
  {
    err = i2c_transfer(client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if (err == 0) 
		break;
    mdelay(1);
  }

  if(i == 10)
  {
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

  return err;
}
*/
/*
static int s5k5ccgx_write_reg(struct i2c_client *client, u8 length, const u8* readdata)
{
  u8 buf[length], i = 0;
  struct i2c_msg msg = {client->addr, I2C_M_WRITE, length, buf};
  int err = 0;
  if (!client->adapter)
  {
    return -ENODEV;
  }  

#if (CAM_S5K5CCGX_I2C_DBG_MSG)
  for (i = 0; i < length; i++) 
  {
    buf[i] = readdata[i];
    printk("=0x%02x=\n",buf[i]);
  }
#else
  for (i = 0; i < length; i++) 
  {
    buf[i] = readdata[i];
  }
#endif

  for (i = 0; i < 10; i++)
  {
    err =  i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if(err == 0) 
    {
	return 0;
    }
    mdelay(1);
  }

  return err;
}
*/

static int s5k5ccgx_start_preview(void);

#ifdef CONFIG_LOAD_FILE
	static int s5k5ccgx_regs_table_write(struct i2c_client *client, char *name);
#endif

static inline int s5k5ccgx_i2c_read(struct i2c_client *camclient, 
	unsigned short subaddr, unsigned short *data)
{
	unsigned char buf[2];
	int err = 0;
#if defined(S5K5CCGX_USE_GPIO_I2C)	
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#else
	struct i2c_client *client = camclient;
	struct i2c_msg msg = {client->addr, 0, 2, buf};
#endif

#if defined(S5K5CCGX_USE_GPIO_I2C)	
	i2c_rd_param.reg_len = 2;
	i2c_rd_param.reg_addr = &subaddr;
	i2c_rd_param.rdata_len = 2;
	i2c_rd_param.rdata = buf;
	err = omap_gpio_i2c_read(s5k5ccgx_i2c_client, &i2c_rd_param);
#else
	buf[0] = subaddr>> 8;
	buf[1] = subaddr & 0xff;
	err = i2c_transfer(client->adapter, &msg, 1);

	if(err < 0)
	{
	    	printk(S5K5CCGX_MOD_NAME  "%s: register read fail %d\n", __func__, err);		
	}

	msg.flags = I2C_M_RD;

	err = i2c_transfer(client->adapter, &msg, 1);
#endif	
	*data = ((buf[0] << 8) | buf[1]);	

	if(err < 0)
	{
	    	printk(S5K5CCGX_MOD_NAME  "%s: register read fail %d\n", __func__, err);			
	}
		
	return 0;//err;
}

static inline int s5k5ccgx_i2c_read_multi(struct i2c_client *camclient,  
	unsigned short subaddr, unsigned long *data)
{
	unsigned char buf[4];
	int err = 0;	
#if defined(S5K5CCGX_USE_GPIO_I2C)	
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#else
	struct i2c_client *client = camclient;
	struct i2c_msg msg = {client->addr, 0, 2, buf};

	if (!client->adapter)
	{
	    	printk(S5K5CCGX_MOD_NAME  "%s: can't search i2c client adapter\n", __func__);
		return -EIO;
	}
#endif	

#if defined(S5K5CCGX_USE_GPIO_I2C)	
	i2c_rd_param.reg_len = 2;
	i2c_rd_param.reg_addr = &subaddr;
	i2c_rd_param.rdata_len = 4;
	i2c_rd_param.rdata = buf;
	err = omap_gpio_i2c_read(s5k5ccgx_i2c_client, &i2c_rd_param);
#else
	buf[0] = subaddr>> 8;
	buf[1] = subaddr & 0xff;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
	    	printk(S5K5CCGX_MOD_NAME  "%s: %d register read fail\n", __func__, err);			
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = 4;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
	    	printk(S5K5CCGX_MOD_NAME  "%s: %d register read fail\n", __func__, err);
		return -EIO;
	}
#endif

	*data = ((buf[0] << 8) | (buf[1] ) | (buf[2] <<24) | buf[3] <<16);

	return 0;//err;
}

static inline int s5k5ccgx_i2c_write(struct i2c_client *camclient, unsigned short addr, unsigned short data)
{
	struct i2c_msg msg[1];
	unsigned char reg[4];
	int err = 0;
	int retry = 0;
#if defined(S5K5CCGX_USE_GPIO_I2C)	
       OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
#else
	struct i2c_client *client = camclient;

	if (!client->adapter)
		return -ENODEV;
#endif

	
#if defined(S5K5CCGX_USE_GPIO_I2C)	
	i2c_wr_param.reg_len = 0;
	i2c_wr_param.reg_addr = NULL;
	i2c_wr_param.wdata_len = 4;
	i2c_wr_param.wdata = reg;
	
	reg[0] = addr >> 8;
	reg[1] = addr & 0xff;	
	reg[2] = data >> 8;
	reg[3] = data & 0xff;   	
       err = omap_gpio_i2c_write(s5k5ccgx_i2c_client, &i2c_wr_param); 	   
//	  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_i2c_write[%x]!!!\n",err);    		   
#else	
again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = reg;

	reg[0] = addr >> 8;
	reg[1] = addr & 0xff;	
	reg[2] = data >> 8;
	reg[3] = data & 0xff;
	
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;	/* Returns here on success */

	/* abnormal case: retry 5 times */
	if (retry < 5) {
	    	printk(S5K5CCGX_MOD_NAME  "%s: address: 0x%02x%02x, " \
			"value: 0x%02x%02x\n", __func__,  \
			reg[0], reg[1], reg[2], reg[3]);		
		retry++;
		goto again;
	}
#endif
	return err;
}

static inline int s5k5ccgx_i2c_write_block(struct i2c_client *camclient, const s5k5ccgx_short_t regs[], 
							int index, char *name)
{
	struct i2c_client *client = camclient;
	int i =0, err =0;
	
#ifdef CONFIG_LOAD_FILE
	s5k5ccgx_regs_table_write(client, name);
#else
	for (i=0 ; i <index; i++) {
		if(regs[i].addr == 0xffff)
			{
			msleep(regs[i].val);
			}
		else
			err = s5k5ccgx_i2c_write(client, regs[i].addr, regs[i].val);
		
		if (unlikely(err < 0)) {
	 	   	printk(S5K5CCGX_MOD_NAME  "%s: register set failed\n", __func__);
			return err;
		}
	}
#endif
	return 0;
}

static int s5k5ccgx_i2c_write_block_for_burst(struct i2c_client *camclient, u8 *buf, int size)
{
	struct i2c_client *client = camclient;
	int retry_count = 5;
	int ret;
	struct i2c_msg msg = {client->addr, 0, size, buf};

	do {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;
		msleep(10);
	} while (retry_count-- > 0);
	if (ret != 1) {
		dev_err(&client->dev, "%s: I2C is not working.\n", __func__);
		return -EIO;
	}

	return 0;
}

#if CAM_S5K5CCGX_DBG_MSG
static void s5k5ccgx_dump_regset(struct s5k5ccgx_regset *regset)
{
	if ((regset->data[0] == 0x00) && (regset->data[1] == 0x2A)) {
		if (regset->size <= 6)
			pr_err("odd regset size %d\n", regset->size);
		pr_info("regset: addr = 0x%02X%02X, data[0,1] = 0x%02X%02X,"
			" total data size = %d\n",
			regset->data[2], regset->data[3],
			regset->data[6], regset->data[7],
			regset->size-6);
	} else {
		pr_info("regset: 0x%02X%02X%02X%02X\n",
			regset->data[0], regset->data[1],
			regset->data[2], regset->data[3]);
		if (regset->size != 4)
			pr_err("odd regset size %d\n", regset->size);
	}
}
#endif


/*
 * Parse the init_reg2 array into a number of register sets that
 * we can send over as i2c burst writes instead of writing each
 * entry of init_reg2 as a single 4 byte write.  Write the
 * new data structures and then free them.
 */
static int  s5k5ccgx_i2c_burst_write(struct i2c_client *camclient, const s5k5ccgx_short_t regs[], int index, char *name)
{	
	struct s5k5ccgx_regset *regset_table;
	struct s5k5ccgx_regset *regset;
	struct s5k5ccgx_regset *end_regset;
	u8 *regset_data;
	u8 *dst_ptr;
	const u32 *end_src_ptr;
	bool flag_copied;
	int init_reg_2_array_size = S5K5CCGX_INIT_SET_INDEX;
	int init_reg_2_size = init_reg_2_array_size * sizeof(u32);
	const u32 *src_ptr = (const u32 *)regs;
	u32 src_value;
	int err;

//	printk(S5K5CCGX_MOD_NAME "%s : start\n", __func__);

	s5k5ccgx_i2c_write_block(camclient,S5K5CCGX_INIT_START,S5K5CCGX_INIT_START_INDEX,"S5K5CCGX_INIT_START");
        msleep(100);

	regset_data = vmalloc(init_reg_2_size);
	if (regset_data == NULL)
		return -ENOMEM;
	regset_table = vmalloc(sizeof(struct s5k5ccgx_regset) *	init_reg_2_size);
	if (regset_table == NULL) {
		vfree(regset_data);
		return -ENOMEM;
	}

	dst_ptr = regset_data;
	regset = regset_table;
	end_src_ptr = (const u32 *)&(regs[init_reg_2_array_size]);

	src_value = *src_ptr++;
	while (src_ptr <= end_src_ptr) {
		/* initial value for a regset */
		regset->data = dst_ptr;
		flag_copied = false;
		*dst_ptr++ = src_value >> 8;
		*dst_ptr++ = src_value >> 0;
		*dst_ptr++ = src_value >> 24;
		*dst_ptr++ = src_value >> 16;

		/* check subsequent values for a data flag (starts with
		   0x0F12) or something else */
		do {
			src_value = *src_ptr++;
			if ((src_value & 0x0000FFFF) != 0x00000F12) {
				/* src_value is start of next regset */
				regset->size = dst_ptr - regset->data;
				regset++;
				break;
			}
			/* copy the 0x0F12 flag if not done already */
			if (!flag_copied) {
				*dst_ptr++ = src_value >> 8;
				*dst_ptr++ = src_value >> 0;
				flag_copied = true;
			}
			/* copy the data part */
			*dst_ptr++ = src_value >> 24;
			*dst_ptr++ = src_value >> 16;
		} while (src_ptr < end_src_ptr);
	}
//	printk(S5K5CCGX_MOD_NAME "%s : finished creating table\n", __func__);

	end_regset = regset;
//	printk(S5K5CCGX_MOD_NAME "%s : first regset = %p, last regset = %p, count = %d\n",__func__, regset_table, regset, end_regset - regset_table);
//	printk(S5K5CCGX_MOD_NAME "%s : regset_data = %p, end = %p, dst_ptr = %p\n", __func__, regset_data, regset_data + (init_reg_2_size * sizeof(u32)), dst_ptr);

#if CAM_S5K5CCGX_DBG_MSG
		int last_regset_end_addr = 0;
		regset = regset_table;
		do {
			s5k5ccgx_dump_regset(regset);
			if (regset->size > 4) {
				int regset_addr = (regset->data[2] << 8 |
						regset->data[3]);
				if (last_regset_end_addr == regset_addr)
					pr_info("%s : this regset can be"
						" combined with previous\n",
						__func__);
				last_regset_end_addr = (regset_addr +
							regset->size - 6);
			}
			regset++;
		} while (regset < end_regset);
#endif

	regset = regset_table;
//	printk(S5K5CCGX_MOD_NAME "%s : start writing init reg 2 bursts\n", __func__);
	do {
		if (regset->size > 4) {
			/* write the address packet */
			err = s5k5ccgx_i2c_write_block_for_burst(camclient, regset->data, 4);
			if (err)
				break;
			/* write the data in a burst */
			err = s5k5ccgx_i2c_write_block_for_burst(camclient, regset->data+4,	regset->size-4);
		} else
			err = s5k5ccgx_i2c_write_block_for_burst(camclient, regset->data, regset->size);
		if (err)
			break;
		regset++;
	} while (regset < end_regset);

//	printk(S5K5CCGX_MOD_NAME "%s : finished writing init reg 2 bursts\n", __func__);

	vfree(regset_data);
	vfree(regset_table);

	return err;

}

//=============================================================================================

static int s5k5ccgx_check_dataline(void)
{
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;

	if(sensor->check_dataline) //output Test Pattern
	{	
		printk( "pattern on setting~~~~~~~~~~~~~~\n");
		if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_DTP_ON,S5K5CCGX_DTP_ON_INDEX,"S5K5CCGX_DTP_ON"))		
		{
			printk(S5K5CCGX_MOD_NAME "%s: failed: DTP\n", __func__);				
		       return -EIO;
		}
		printk( "pattern on setting done~~~~~~~~~~~~~~\n");	
	}

	return 0;
}

static int s5k5ccgx_check_dataline_stop(void)
{
	int err = 0;
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;

	// 1. DTP off
	if(sensor->check_dataline) //output Test Pattern
	{	
		err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_DTP_OFF,S5K5CCGX_DTP_OFF_INDEX,"S5K5CCGX_DTP_OFF");
		if(err < 0){
			printk(S5K5CCGX_MOD_NAME "%s: failed: DTP\n", __func__);					
		       return -EIO;
		}
		sensor->check_dataline = 0;
	}

	// 2. preview start
	err = s5k5ccgx_start_preview();
	if(err < 0){
		printk(S5K5CCGX_MOD_NAME "%s: failed: DTP2\n", __func__);		
		return -EIO;
	}
	
	return 0;
}

/************************************************************************
CONFIG_LOAD_FILE
************************************************************************/

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5k5ccgx_regs_table = NULL;

static int s5k5ccgx_regs_table_size;

void s5k5ccgx_regs_table_exit(void);

int s5k5ccgx_regs_table_init(void)
{
#ifdef VIEW_FUNCTION_CALL	
	printk("[S5K5CCGX] %s function %d line launched!\n", __func__, __LINE__);
#endif

	printk("[BestIQ] + s5k5ccgx_regs_table_init\n");
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int i;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	s5k5ccgx_regs_table_exit();
		
	set_fs(get_ds());

	filp = filp_open("/sdcard/s5k5ccgx.h", O_RDONLY, 0);

	if (IS_ERR(filp)) {
		printk("file open error\n");
		return PTR_ERR(filp);
	}
	else
	{
		printk("file open success\n");
	}
			
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
//	dp = vmalloc(l);	
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
		kfree(dp);
//		vfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);
		
	set_fs(fs);
	
	s5k5ccgx_regs_table = dp;
		
	s5k5ccgx_regs_table_size = l;
	
	*((s5k5ccgx_regs_table + s5k5ccgx_regs_table_size) - 1) = '\0';
	
	printk("s5k5ccgx_regs_table 0x%08x, %ld\n", dp, l);
	printk("[BestIQ] - s5k5ccgx_reg_table_init\n");

	return 0;
}

void s5k5ccgx_regs_table_exit(void)
{
#ifdef VIEW_FUNCTION_CALL	
	printk("[S5K5CCGX] %s function %d line launched!\n", __func__, __LINE__);
#endif

	printk("[BestIQ] + s5k5ccgx_regs_table_exit\n");
	printk("%s %d\n", __func__, __LINE__);
	if (s5k5ccgx_regs_table) {
		kfree(s5k5ccgx_regs_table);
		s5k5ccgx_regs_table = NULL;
	}
	printk("[BestIQ] - s5k5ccgx_regs_table_exit\n");
}

static int s5k5ccgx_regs_table_write(struct i2c_client *client, char *name)
{
	printk("[BestIQ] + s5k5ccgx_regs_table_write\n");
	char *start, *end, *reg, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];
	#ifdef VIEW_FUNCTION_CALL	
	printk("[S5K5CCGX] %s function %d line launched!\n", __func__, __LINE__);
	#endif
	
	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

//	printk("[BestIQ] + s5k5ccgx_regs_table_write ------- start\n");
	start = strstr(s5k5ccgx_regs_table, name);
	end = strstr(start, "};");
	
	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{0x");		
		if (reg)
			start = (reg + 16);  //{0xFCFC, 0xD000}	
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 6);	
			memcpy(data_buf, (reg + 9), 6);				
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 			
//			printk("addr 0x%04x, value1 0x%04x, value2 0x%04x\n", addr, value1, value2);
			if (addr == 0xffff)
				msleep(value);
			else if(addr == 0xdddd) //get value to tunning
				return value;
			else
				s5k5ccgx_i2c_write(client, addr, value);
		}
	}
	printk("[BestIQ] - s5k5ccgx_regs_table_write\n");
	return 0;
}

#endif

static int s5k5ccgx_detect(struct i2c_client *client)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;

  int err = 0;
  unsigned short read_value = 0;  

  sensor->runmode = S5K5CCGX_RUNMODE_NOTREADY; 
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_detect is called...\n");
	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	if (err >= 0)
	{
		err+=s5k5ccgx_i2c_write(client,0x002C, 0x0000);
		err+=s5k5ccgx_i2c_write(client,0x002E, 0x0040);
		err+=s5k5ccgx_i2c_read(client, 0x0F12, &read_value);
	}
//	printk("%s: VE_GROUP err(%d) id(0x%X)\n", __func__ , err ,read_value);
	printk("s5k5ccgx_detect... (%d) id(0x%x)\n",err ,read_value);	
//	if (err >= 0 && read_value == 0x05CC)
//	{
//		gcamera_sensor_back_type = CAMERA_SENSOR_ID_S5K5CCGX;
//		gcamera_sensor_back_checked = true;
//		return 0;
//	}

  if(s5k5ccgx_i2c_write(client,0xFCFC, 0xD000))
    goto detect_fail;
  if(s5k5ccgx_i2c_write(client,0x002C, 0x0000))
    goto detect_fail;  
  if(s5k5ccgx_i2c_write(client,0x002E, 0x0040))
    goto detect_fail;
  if(s5k5ccgx_i2c_read(client, 0x0F12, &read_value))
    goto detect_fail;  

#ifdef CONFIG_LOAD_FILE
	printk("[BestIQ] +s5k5ccgx_init\n");
	err = s5k5ccgx_regs_table_init();
	if (err) {
		printk(S5K5CCGX_MOD_NAME "%s: config file read fail\n", __func__);			
			return 0;
	}
#endif


//	 if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_INIT_SET,S5K5CCGX_INIT_SET_INDEX,"S5K5CCGX_INIT_SET"))
	 if(s5k5ccgx_i2c_burst_write(client, S5K5CCGX_INIT_SET,S5K5CCGX_INIT_SET_INDEX,"S5K5CCGX_INIT_SET"))
    	goto detect_fail;  
	
  return 0;

detect_fail:
  printk(S5K5CCGX_MOD_NAME " Sensor is not Detected!!!\n");
  return -EINVAL; 
}

static int s5k5ccgx_set_frame_rate(s32 value)
{
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;
	
	int err = 0;
	
	printk("s5k5ccgx_set_frame_rate() : value == 0x%x\n", value);	
	switch(value)
	{
		case 7:
			 err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_7_FPS,
			 		S5K5CCGX_7_FPS_INDEX,"S5K5CCGX_7_FPS");
			break;
		case 10:
			 err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_10_FPS,
			 		S5K5CCGX_10_FPS_INDEX,"S5K5CCGX_10_FPS");
			break;

		case 12:
			 err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_12_FPS,
					S5K5CCGX_12_FPS_INDEX,"S5K5CCGX_12_FPS");
			break;

		case 15:
			if(sensor->mode == S5K5CCGX_MODE_CAMERA) //in case camera mode, fixed auto_15
			{
				printk("S5K5CCGX_AUTO15_FPS\n");					
				err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_AUTO15_FPS,S5K5CCGX_AUTO15_FPS_INDEX,"S5K5CCGX_AUTO15_FPS");
			}
			else
			{
				printk("S5K5CCGX_15_FPS\n");					
				err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_15_FPS,S5K5CCGX_15_FPS_INDEX,"S5K5CCGX_15_FPS");
			}
			break;
		case 30:
			if(sensor->mode == S5K5CCGX_MODE_CAMERA) //in case camera mode, fixed auto_30
			{
				printk("S5K5CCGX_AUTO30_FPS\n");					
				err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_AUTO30_FPS,S5K5CCGX_AUTO30_FPS_INDEX,"S5K5CCGX_AUTO30_FPS");
			}
			else
			{
				printk("S5K5CCGX_30_FPS\n");					
				err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_30_FPS,S5K5CCGX_30_FPS_INDEX,"S5K5CCGX_30_FPS");
			}
			break;
		default:
			printk("Failed to s5k5ccgx_set_frame_rate() : value == 0x%x\n",value);				
			break;
	}
	msleep(133);
	if(err < 0)
	{
	  	printk(S5K5CCGX_MOD_NAME "%s: i2c_write failed\n", __func__);			
		return -EIO;
	}
	
	return 0;
}

static unsigned long s5k5ccgx_get_illumination(void)
{
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;
	int err;
	unsigned long read_value = 0;

	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x002C, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002E, 0x2A3C);
	err=s5k5ccgx_i2c_read_multi(client, 0x0F12, &read_value);

	//s5k5ccgx_msg(&client->dev, "%s: lux_value == 0x%x \n", __func__, read_value);	
	printk("s5k5ccgx_get_illumination() : lux_value == 0x%lx\n", read_value);

	if(err < 0){
	    	printk(S5K5CCGX_MOD_NAME "%s: failed: s5k5ccgx_get_auto_focus_status %d\n", __func__, err);
	      	 return -EIO;
	}
	
	return read_value;
	
}

#ifdef S5K5CCGX_FLASH_SUPPORT
static int s5k5ccgx_set_flash(int lux_val)
{
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	int i = 0;
	int err = 0;

	printk("%s, flash set is %d\n", __func__, lux_val);

	if (lux_val == 100)
	{
		if(!sensor->flash_gpio_state)
		{
			if (gpio_request(OMAP3430_GPIO_FLASH_EN1,"FLASH EN1") != 0)
			{
				printk("s5k5ccgx_set_flash::Could not request GPIO %d\n", OMAP3430_GPIO_FLASH_EN1);
				return -EIO;
			}

			if (gpio_request(OMAP3430_GPIO_FLASH_EN2,"FLASH EN2") != 0) 
			{
				printk( "s5k5ccgx_set_flash::Could not request GPIO %d", OMAP3430_GPIO_FLASH_EN2);
				return -EIO;
			}
			sensor->flash_gpio_state = true;
		}
		
		//movie mode
		lux_val = MOVIEMODE_FLASH;
		gpio_direction_output(OMAP3430_GPIO_FLASH_EN2, 0);
		for (i = lux_val; i > 1; i--)
		{
			//gpio on
			gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 1); 
			udelay(1);
			//gpio off
			gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 0); 
			udelay(1);
		}
		gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 1); 
		msleep(2);
	}
	else if (lux_val == 0)
	{
		if(sensor->flash_gpio_state)	
		{	
		//flash off
		gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 0); 
		gpio_direction_output(OMAP3430_GPIO_FLASH_EN2, 0);
		msleep(2);	
		gpio_free(OMAP3430_GPIO_FLASH_EN1);  
		gpio_free(OMAP3430_GPIO_FLASH_EN2); 		
		sensor->flash_gpio_state = false;			
		}
	}
	else
	{
		if(!sensor->flash_gpio_state)
		{
			if (gpio_request(OMAP3430_GPIO_FLASH_EN1,"FLASH EN1") != 0)
			{
				printk("s5k5ccgx_set_flash::Could not request GPIO %d\n", OMAP3430_GPIO_FLASH_EN1);
				return -EIO;
			}

			if (gpio_request(OMAP3430_GPIO_FLASH_EN2,"FLASH EN2") != 0) 
			{
				printk( "s5k5ccgx_set_flash::Could not request GPIO %d", OMAP3430_GPIO_FLASH_EN2);
				return -EIO;
			}
			sensor->flash_gpio_state = true;
		}		
		gpio_direction_output(OMAP3430_GPIO_FLASH_EN2, 1);
		udelay(20);
		for (i = lux_val; i > 1; i--)
		{
			//gpio on
			gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 1); 
			udelay(1);
			//gpio off
			gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 0); ;
			udelay(1);
		}
		gpio_direction_output(OMAP3430_GPIO_FLASH_EN1, 1); 
		msleep(2);
	}
	return err;
}

static int s5k5ccgx_set_flash_mode(int flash_brightness_value, bool value)
{
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;
	int err;
	unsigned long lux_value;

	if(value)
	{
		switch(sensor->flash_capture)
		{
			case S5K5CCGX_FLASH_CAPTURE_AUTO:
				lux_value = s5k5ccgx_get_illumination();

				if (lux_value < 0x0020 || flash_auto_check ) 
				{
					if (preflash == 1)
					{			
						preflash = 2;					
						printk("S5K5CCGX_PRE_FLASH_START_EVT1\n");
						err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_AE_SPEEDUP,S5K5CCGX_AE_SPEEDUP_INDEX,"S5K5CCGX_AE_SPEEDUP");
						err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_PRE_FLASH_START_EVT1,S5K5CCGX_PRE_FLASH_START_EVT1_INDEX,"S5K5CCGX_PRE_FLASH_START_EVT1");
					}
					err = s5k5ccgx_set_flash(flash_brightness_value);
					sensor->flashstate = true;
					flash_check = true;
					flash_auto_check = true;					
				}
				else
					sensor->flashstate = false;

				break;
				
			case S5K5CCGX_FLASH_CAPTURE_ON:
				if (preflash == 1)
				{
					preflash = 2;
					printk("S5K5CCGX_PRE_FLASH_START_EVT1\n");
					err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_AE_SPEEDUP,S5K5CCGX_AE_SPEEDUP_INDEX,"S5K5CCGX_AE_SPEEDUP");
					err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_PRE_FLASH_START_EVT1,S5K5CCGX_PRE_FLASH_START_EVT1_INDEX,"S5K5CCGX_PRE_FLASH_START_EVT1");
				}
				err =s5k5ccgx_set_flash(flash_brightness_value);
				sensor->flashstate = true;
				flash_check = true;
				break;
				
			case S5K5CCGX_FLASH_CAPTURE_OFF:
					//err =s5k5ccgx_set_flash(flash_brightness_value, sd);
					sensor->flashstate = false;
				break;
				
			default:
  			  	printk(S5K5CCGX_MOD_NAME "%s: Unknown Flash mode \n", __func__);				
				break;
		}
	}
	else
	{
		s5k5ccgx_set_flash(0);
	}
	
	printk(S5K5CCGX_MOD_NAME "%s: done\n", __func__);		
	return 0;
}
#endif


static int s5k5ccgx_set_effect(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_effect is called...[%d]\n",value);

  switch(value)
  {
    case S5K5CCGX_EFFECT_OFF:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_EFFECT_OFF,S5K5CCGX_CAM_EFFECT_OFF_INDEX,"S5K5CCGX_CAM_EFFECT_OFF"))
        goto effect_fail;
      break;

    case S5K5CCGX_EFFECT_BW:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_EFFECT_MONO,S5K5CCGX_CAM_EFFECT_MONO_INDEX,"S5K5CCGX_CAM_EFFECT_MONO"))
        goto effect_fail;
      break;    
 
    case S5K5CCGX_EFFECT_SEPIA:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_EFFECT_SEPIA,S5K5CCGX_CAM_EFFECT_SEPIA_INDEX,"S5K5CCGX_CAM_EFFECT_SEPIA"))
        goto effect_fail;
      break;
   
    case S5K5CCGX_EFFECT_NEGATIVE:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_EFFECT_NEGATIVE,S5K5CCGX_CAM_EFFECT_NEGATIVE_INDEX,"S5K5CCGX_CAM_EFFECT_NEGATIVE"))
        goto effect_fail;
      break;

    case S5K5CCGX_EFFECT_AQUA:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_EFFECT_AQUA,S5K5CCGX_CAM_EFFECT_AQUA_INDEX,"S5K5CCGX_CAM_EFFECT_AQUA"))
        goto effect_fail;
      break;
  
    default:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_EFFECT_OFF,S5K5CCGX_CAM_EFFECT_OFF_INDEX,"S5K5CCGX_CAM_EFFECT_OFF"))		
      goto effect_fail;
      printk(S5K5CCGX_MOD_NAME "Effect value is not supported!!!\n");	  
  }

  sensor->effect = value;

  return 0;

effect_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_effect is failed!!\n");
  return -EINVAL;     
}

static int s5k5ccgx_set_iso(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_iso is called...[%d]\n",value);

  switch(value)
  {
    case S5K5CCGX_ISO_AUTO:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_ISO_AUTO,S5K5CCGX_CAM_ISO_AUTO_INDEX,"S5K5CCGX_CAM_ISO_AUTO"))
        goto iso_fail;
      break;

    case S5K5CCGX_ISO_50:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_ISO_50,S5K5CCGX_CAM_ISO_50_INDEX,"S5K5CCGX_CAM_ISO_50"))
        goto iso_fail;
      break;      
            
    case S5K5CCGX_ISO_100:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_ISO_100,S5K5CCGX_CAM_ISO_100_INDEX,"S5K5CCGX_CAM_ISO_100"))
        goto iso_fail;
      break;      

    case S5K5CCGX_ISO_200:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_ISO_200,S5K5CCGX_CAM_ISO_200_INDEX,"S5K5CCGX_CAM_ISO_200"))
        goto iso_fail;
      break;      
      
    case S5K5CCGX_ISO_400:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_ISO_400,S5K5CCGX_CAM_ISO_400_INDEX,"S5K5CCGX_CAM_ISO_400"))
        goto iso_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "ISO value is not supported!!!\n");
      goto iso_fail;
  }

  sensor->iso = value;

  return 0;

iso_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_iso is failed!!\n");
  return -EINVAL;       
}

static int s5k5ccgx_set_photometry(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_photometry is called...[%d]\n",value);

  switch(value)
  {
    case S5K5CCGX_PHOTOMETRY_CENTER:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_METERING_CENTER,S5K5CCGX_METERING_CENTER_INDEX,"S5K5CCGX_METERING_CENTER"))
        goto photometry_fail;
      break;
      
    case S5K5CCGX_PHOTOMETRY_SPOT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_METERING_SPOT,S5K5CCGX_METERING_SPOT_INDEX,"S5K5CCGX_METERING_SPOT"))
        goto photometry_fail;
      break;
      
    case S5K5CCGX_PHOTOMETRY_MATRIX:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_METERING_NORMAL,S5K5CCGX_METERING_NORMAL_INDEX,"S5K5CCGX_METERING_NORMAL"))
        goto photometry_fail;
      break;
      
    default:
      printk(S5K5CCGX_MOD_NAME "Photometry value is not supported!!!\n");
      goto photometry_fail;
  }

  sensor->photometry = value;

  return 0;

photometry_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_iso is failed!!\n");
  return -EINVAL;         
}

static int s5k5ccgx_set_ev(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_ev is called...[%d]\n",value);
  
  switch(value)
  {
    case S5K5CCGX_EV_MINUS_2P0:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_N_4,S5K5CCGX_BRIGHTNESS_N_4_INDEX,"S5K5CCGX_BRIGHTNESS_N_4"))
        goto ev_fail;
      break;
      
    case S5K5CCGX_EV_MINUS_1P5:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_N_3,S5K5CCGX_BRIGHTNESS_N_3_INDEX,"S5K5CCGX_BRIGHTNESS_N_3"))
        goto ev_fail;
      break;

    case S5K5CCGX_EV_MINUS_1P0:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_N_2,S5K5CCGX_BRIGHTNESS_N_2_INDEX,"S5K5CCGX_BRIGHTNESS_N_2"))
        goto ev_fail;
      break;

    case S5K5CCGX_EV_MINUS_0P5:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_N_1,S5K5CCGX_BRIGHTNESS_N_1_INDEX,"S5K5CCGX_BRIGHTNESS_N_1"))
        goto ev_fail;
      break;      
      
    case S5K5CCGX_EV_DEFAULT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_0,S5K5CCGX_BRIGHTNESS_0_INDEX,"S5K5CCGX_BRIGHTNESS_0"))
        goto ev_fail;
      break;
      
    case S5K5CCGX_EV_PLUS_0P5:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_P_1,S5K5CCGX_BRIGHTNESS_P_1_INDEX,"S5K5CCGX_BRIGHTNESS_P_1"))
        goto ev_fail;
      break;
      
    case S5K5CCGX_EV_PLUS_1P0:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_P_2,S5K5CCGX_BRIGHTNESS_P_2_INDEX,"S5K5CCGX_BRIGHTNESS_P_2"))
        goto ev_fail;
      break;

    case S5K5CCGX_EV_PLUS_1P5:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_P_3,S5K5CCGX_BRIGHTNESS_P_3_INDEX,"S5K5CCGX_BRIGHTNESS_P_3"))
        goto ev_fail;
      break;

    case S5K5CCGX_EV_PLUS_2P0:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_BRIGHTNESS_P_4,S5K5CCGX_BRIGHTNESS_P_4_INDEX,"S5K5CCGX_BRIGHTNESS_P_4"))
        goto ev_fail;
      break;      
      
    default:
      printk(S5K5CCGX_MOD_NAME "EV value is not supported!!!\n");
      goto ev_fail;
  }

  sensor->ev = value;

  return 0;

ev_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_ev is failed!!\n");
  return -EINVAL;           
}

static int s5k5ccgx_set_wdr(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_wdr is called...not support[%d]\n",value);
/*
  switch(value)
  {
    case S5K5CCGX_WDR_OFF:
      if(s5k5ccgx_write_reg(client, sizeof(WDR_OFF_list), WDR_OFF_list))
        goto wdr_fail;
      break;
      
    case S5K5CCGX_WDR_ON:
      if(s5k5ccgx_write_reg(client, sizeof(WDR_ON_list), WDR_ON_list))
        goto wdr_fail;
      break;
      
    default:
      printk(S5K5CCGX_MOD_NAME "WDR value is not supported!!!\n");
      goto wdr_fail;
  }
*/
  sensor->wdr = value;
  
  return 0;
/*  
wdr_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_ev is failed!!\n");
  return -EINVAL;             
  */
}

static int s5k5ccgx_set_saturation(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_saturation is called...[%d]\n",value);
  
  switch(value)
  {  
    case S5K5CCGX_SATURATION_MINUS_2:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SATURATION_M_2,S5K5CCGX_SATURATION_M_2_INDEX,"S5K5CCGX_SATURATION_M_2"))
        goto saturation_fail;
      break;
      
    case S5K5CCGX_SATURATION_MINUS_1:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SATURATION_M_1,S5K5CCGX_SATURATION_M_1_INDEX,"S5K5CCGX_SATURATION_M_1"))
        goto saturation_fail;
      break;
      
    case S5K5CCGX_SATURATION_DEFAULT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SATURATION_0,S5K5CCGX_SATURATION_0_INDEX,"S5K5CCGX_SATURATION_0"))
        goto saturation_fail;
      break;
      
    case S5K5CCGX_SATURATION_PLUS_1:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SATURATION_P_1,S5K5CCGX_SATURATION_P_1_INDEX,"S5K5CCGX_SATURATION_P_1"))
        goto saturation_fail;
      break;
      
    case S5K5CCGX_SATURATION_PLUS_2:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SATURATION_P_2,S5K5CCGX_SATURATION_P_2_INDEX,"S5K5CCGX_SATURATION_P_2"))
        goto saturation_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "Saturation value is not supported!!!\n");
      goto saturation_fail;
  }

  sensor->saturation = value;
  
  return 0;

saturation_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_saturation is failed!!\n");
  return -EINVAL;             
}

static int s5k5ccgx_set_sharpness(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_sharpness is called...[%d]\n",value);
  
  switch(value)
  { 
    case S5K5CCGX_SHARPNESS_MINUS_2:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SHAPNESS_M_2,S5K5CCGX_SHAPNESS_M_2_INDEX,"S5K5CCGX_SHAPNESS_M_2"))
        goto sharpness_fail;
      break;
      
    case S5K5CCGX_SHARPNESS_MINUS_1:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SHAPNESS_M_1,S5K5CCGX_SHAPNESS_M_1_INDEX,"S5K5CCGX_SHAPNESS_M_1"))
        goto sharpness_fail;
      break;
      
    case S5K5CCGX_SHARPNESS_DEFAULT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SHAPNESS_0,S5K5CCGX_SHAPNESS_0_INDEX,"S5K5CCGX_SHAPNESS_0"))
        goto sharpness_fail;
      break;
      
    case S5K5CCGX_SHARPNESS_PLUS_1:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SHAPNESS_P_1,S5K5CCGX_SHAPNESS_P_1_INDEX,"S5K5CCGX_SHAPNESS_P_1"))
        goto sharpness_fail;
      break;
      
    case S5K5CCGX_SHARPNESS_PLUS_2:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_SHAPNESS_P_2,S5K5CCGX_SHAPNESS_P_2_INDEX,"S5K5CCGX_SHAPNESS_P_2"))
        goto sharpness_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "Sharpness value is not supported!!!\n");
      goto sharpness_fail;
  }

  sensor->sharpness = value;

  return 0;

sharpness_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_sharpness is failed!!\n");
  return -EINVAL;               
}

static int s5k5ccgx_set_contrast(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_contrast is called...[%d]\n",value);

  switch(value)
  {
    case S5K5CCGX_CONTRAST_MINUS_2:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CONTRAST_M_2,S5K5CCGX_CONTRAST_M_2_INDEX,"S5K5CCGX_CONTRAST_M_2"))	
        goto contrast_fail;
      break;
      
    case S5K5CCGX_CONTRAST_MINUS_1:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CONTRAST_M_1,S5K5CCGX_CONTRAST_M_1_INDEX,"S5K5CCGX_CONTRAST_M_1"))	
        goto contrast_fail;
      break;
      
    case S5K5CCGX_CONTRAST_DEFAULT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CONTRAST_0,S5K5CCGX_CONTRAST_0_INDEX,"S5K5CCGX_CONTRAST_0"))	
        goto contrast_fail;
      break;
      
    case S5K5CCGX_CONTRAST_PLUS_1:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CONTRAST_P_1,S5K5CCGX_CONTRAST_P_1_INDEX,"S5K5CCGX_CONTRAST_P_1"))	
        goto contrast_fail;
      break;
      
    case S5K5CCGX_CONTRAST_PLUS_2:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CONTRAST_P_2,S5K5CCGX_CONTRAST_P_2_INDEX,"S5K5CCGX_CONTRAST_P_2"))
        goto contrast_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "Contrast value is not supported!!!\n");
      goto contrast_fail;
  }

  sensor->contrast = value;

  return 0;

contrast_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_contrast is failed!!\n");
  return -EINVAL;               
}

static int s5k5ccgx_set_wb(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_wb is called...[%d]\n",value);
  
  switch(value)
  {
    case S5K5CCGX_WB_AUTO:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_WB_AUTO,S5K5CCGX_CAM_WB_AUTO_INDEX,"S5K5CCGX_CAM_WB_AUTO"))
        goto wb_fail;
      break;
      
    case S5K5CCGX_WB_DAYLIGHT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_WB_DAYLIGHT,S5K5CCGX_CAM_WB_DAYLIGHT_INDEX,"S5K5CCGX_CAM_WB_DAYLIGHT"))
        goto wb_fail;
      break;
      
    case S5K5CCGX_WB_INCANDESCENT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_WB_INCANDESCENT,S5K5CCGX_CAM_WB_INCANDESCENT_INDEX,"S5K5CCGX_CAM_WB_INCANDESCENT"))
        goto wb_fail;
      break;
      
    case S5K5CCGX_WB_FLUORESCENT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_WB_FLUORESCENT,S5K5CCGX_CAM_WB_FLUORESCENT_INDEX,"S5K5CCGX_CAM_WB_FLUORESCENT"))
        goto wb_fail;
      break;
      
    case S5K5CCGX_WB_CLOUDY:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_WB_CLOUDY,S5K5CCGX_CAM_WB_CLOUDY_INDEX,"S5K5CCGX_CAM_WB_CLOUDY"))
        goto wb_fail;
      break;
      
    default:
      printk(S5K5CCGX_MOD_NAME "WB value is not supported!!!\n");
      goto wb_fail;
  }

  sensor->wb = value;

  return 0;

wb_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_wb is failed!!\n");
  return -EINVAL;                 
}

static int s5k5ccgx_set_isc(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
 // struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_isc is called...not support[%d]\n",value);
/*
  switch(value)
  {
    case S5K5CCGX_ISC_STILL_OFF:
      if(s5k5ccgx_write_reg(client, sizeof(IS_Still_Off_list), IS_Still_Off_list))
        goto isc_fail;
      break;
    case S5K5CCGX_ISC_STILL_ON:
      if(s5k5ccgx_write_reg(client, sizeof(IS_Still_On_list), IS_Still_On_list))
        goto isc_fail;
      break;
    case S5K5CCGX_ISC_STILL_AUTO:
      if(s5k5ccgx_write_reg(client, sizeof(IS_Still_Auto_list), IS_Still_Auto_list))
        goto isc_fail;
      break;
    case S5K5CCGX_ISC_MOVIE_ON:
      if(s5k5ccgx_write_reg(client, sizeof(IS_Movie_On_list), IS_Movie_On_list))
        goto isc_fail;
      break;
    default:
      printk(S5K5CCGX_MOD_NAME "ISC value is not supported!!!\n");
      goto isc_fail;
  }
*/
  sensor->isc = value;
  
  return 0;
/*
isc_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_isc is failed!!\n");
  return -EINVAL;
  */
}

static int s5k5ccgx_get_scene(struct v4l2_control *vc)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//struct i2c_client *client = sensor->i2c_client;
//  u8 readdata[2] = {0x00,};

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_get_scene is called...\n"); 

//    vc->value = readdata[1];

  return 0;
/*
get_scene_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_get_scene is failed!!\n");
  return -EINVAL;  
  */
}

/**
 *  lock:	1 to lock, 0 to unlock
 */
static int s5k5ccgx_set_ae_lock(int lock)
{
	int err = 0;
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;
	
	if(lock)
	{
		if (sensor->flashstate == false)
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_AE_LOCK,S5K5CCGX_AE_LOCK_INDEX,"S5K5CCGX_AE_LOCK");
	}
	else
	{
		err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_AE_UNLOCK,S5K5CCGX_AE_UNLOCK_INDEX,"S5K5CCGX_AE_UNLOCK");
	}
	
	if(err < 0){
		printk(S5K5CCGX_MOD_NAME "%s: failed: i2c_write for ae_lock\n", __func__);		
		return -EIO;
	}

	printk(S5K5CCGX_MOD_NAME "%s: done\n", __func__);	

	return 0;
}

static int s5k5ccgx_set_scene(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_scene is called...[%d]\n",value);

  if(value != S5K5CCGX_SCENE_OFF)
	{
	if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_OFF,S5K5CCGX_CAM_SCENE_OFF_INDEX,"S5K5CCGX_CAM_SCENE_OFF"))
	goto scene_fail;
	}

  switch(value)
  {
    case S5K5CCGX_SCENE_OFF:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_OFF,S5K5CCGX_CAM_SCENE_OFF_INDEX,"S5K5CCGX_CAM_SCENE_OFF"))
        goto scene_fail;
      break;

    case S5K5CCGX_SCENE_ASD:
      break;    
      
    case S5K5CCGX_SCENE_PORTRAIT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_PORTRAIT,S5K5CCGX_CAM_SCENE_PORTRAIT_INDEX,"S5K5CCGX_CAM_SCENE_PORTRAIT"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_LANDSCAPE:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_LANDSCAPE,S5K5CCGX_CAM_SCENE_LANDSCAPE_INDEX,"S5K5CCGX_CAM_SCENE_LANDSCAPE"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_SUNSET:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_SUNSET,S5K5CCGX_CAM_SCENE_SUNSET_INDEX,"S5K5CCGX_CAM_SCENE_SUNSET"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_DAWN:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_DAWN,S5K5CCGX_CAM_SCENE_DAWN_INDEX,"S5K5CCGX_CAM_SCENE_DAWN"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_NIGHTSHOT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_NIGHT,S5K5CCGX_CAM_SCENE_NIGHT_INDEX,"S5K5CCGX_SCAM_CENE_NIGHT"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_TEXT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_TEXT,S5K5CCGX_CAM_SCENE_TEXT_INDEX,"S5K5CCGX_CAM_SCENE_TEXT"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_SPORTS:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_SPORTS,S5K5CCGX_CAM_SCENE_SPORTS_INDEX,"S5K5CCGX_CAM_SCENE_SPORTS"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_AGAINST_LIGHT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_BACKLIGHT,S5K5CCGX_CAM_SCENE_BACKLIGHT_INDEX,"S5K5CCGX_CAM_SCENE_BACKLIGHT"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_INDOORS:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_PARTY,S5K5CCGX_CAM_SCENE_PARTY_INDEX,"S5K5CCGX_CAM_SCENE_PARTY"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_BEACH_SNOW:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_BEACH,S5K5CCGX_CAM_SCENE_BEACH_INDEX,"S5K5CCGX_CAM_SCENE_BEACH"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_FALLCOLOR:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_FALL,S5K5CCGX_CAM_SCENE_FALL_INDEX,"S5K5CCGX_CAM_SCENE_FALL"))
        goto scene_fail;
      break;
      
    case S5K5CCGX_SCENE_FIREWORKS:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_FIRE,S5K5CCGX_CAM_SCENE_FIRE_INDEX,"S5K5CCGX_CAM_SCENE_FIRE"))
        goto scene_fail;
      break;

    case S5K5CCGX_SCENE_CANDLELIGHT:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_CANDLE,S5K5CCGX_SCENE_CAM_CANDLE_INDEX,"S5K5CCGX_CAM_SCENE_CANDLE"))
        goto scene_fail;
      break;
      
    default:
	if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_CAM_SCENE_OFF,S5K5CCGX_CAM_SCENE_OFF_INDEX,"S5K5CCGX_CAM_SCENE_OFF"))		
      printk(S5K5CCGX_MOD_NAME "Scene value is not supported!!!\n");
      goto scene_fail;
  }

  sensor->scene = value;  

  return 0;

scene_fail:
  printk(S5K5CCGX_MOD_NAME " s5k5ccgx_set_scene is failed!!\n");
  return -EINVAL;    
}

static int s5k5ccgx_set_mode(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  sensor->mode = value;
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_mode is called... mode = %d\n", sensor->mode); 
  return 0;
}

static int s5k5ccgx_set_state(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  sensor->state = value;
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_state is called... state = %d\n", sensor->state); 
  return 0;
}

static int s5k5ccgx_set_aewb(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_aewb is called...[%d]\n",value);
  
  switch(value)
  {
    case S5K5CCGX_AE_LOCK_AWB_LOCK:
	if (sensor->flashstate == false)
	{
	       if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AE_LOCK,S5K5CCGX_AE_LOCK_INDEX,"S5K5CCGX_AE_LOCK"))
	       goto aewb_fail;
	}
	if (sensor->wb == S5K5CCGX_WB_AUTO)
	{
	       if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AWE_LOCK,S5K5CCGX_AWE_LOCK_INDEX,"S5K5CCGX_AWE_LOCK"))
	       goto aewb_fail;		
	}	
      break;
    case S5K5CCGX_AE_LOCK_AWB_UNLOCK:
       if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AE_LOCK,S5K5CCGX_AE_LOCK_INDEX,"S5K5CCGX_AE_LOCK"))
       goto aewb_fail;
      break;
    case S5K5CCGX_AE_UNLOCK_AWB_LOCK:
       if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AWE_LOCK,S5K5CCGX_AWE_LOCK_INDEX,"S5K5CCGX_AWE_LOCK"))
       goto aewb_fail;
      break;
    case S5K5CCGX_AE_UNLOCK_AWB_UNLOCK:
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AE_UNLOCK,S5K5CCGX_AE_UNLOCK_INDEX,"S5K5CCGX_AE_UNLOCK"))
	if (sensor->wb == S5K5CCGX_WB_AUTO)
	{
	       if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AWE_UNLOCK,S5K5CCGX_AWE_UNLOCK_INDEX,"S5K5CCGX_AWE_UNLOCK"))
	       goto aewb_fail;		
	}
      break;
    default:
      printk(S5K5CCGX_MOD_NAME "AE/AWB value is not supported!!!\n");
      goto aewb_fail;
  }

  sensor->aewb = value;
  
  return 0;

aewb_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_aewb is failed!!!\n");
  return -EINVAL;
}

static int s5k5ccgx_set_antishake(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_antishake is called...not support[%d]\n",value);
/*  
  switch(value)
  {
    case S5K5CCGX_ANTI_SHAKE_OFF:
      if(s5k5ccgx_write_reg(client, sizeof(AntiShake_OFF_list), AntiShake_OFF_list))
        goto antishake_fail;
      break;
    case S5K5CCGX_ANTI_SHAKE_ON:
      if(s5k5ccgx_write_reg(client, sizeof(AntiShake_ON_list), AntiShake_ON_list))
        goto antishake_fail;
      break;
    default:
      printk(S5K5CCGX_MOD_NAME "Anti-shake value is not supported!!!\n");
      goto antishake_fail;
  }
*/
  sensor->antishake = value;

  return 0;
/*
antishake_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_antishake is failed!!!\n");
  return -EINVAL;  
  */
}

static int s5k5ccgx_set_flash_capture(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flash_capture is called...[%d]\n",value);

  if(value == 4){ /* Camera FLASH TORCH ON */
	  s5k5ccgx_set_flash(100);
  }
  else if(value == 5) {
	  s5k5ccgx_set_flash(0); /* Camera FLASH TORCH OFF */
  }
#if 0
  switch(value)
  {
    case S5K5CCGX_FLASH_CAPTURE_OFF:
      if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureOff_list), FlashCaptureOff_list))
        goto flash_capture_fail;
      break;
    case S5K5CCGX_FLASH_CAPTURE_ON:
      if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureOn_list), FlashCaptureOn_list))
        goto flash_capture_fail;
      break;
    case S5K5CCGX_FLASH_CAPTURE_AUTO:
      if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureAuto_list), FlashCaptureAuto_list))
        goto flash_capture_fail;
      break;
    default:
      printk(S5K5CCGX_MOD_NAME "Flash value is not supported!!!\n");
      goto flash_capture_fail;
  }
#endif
  sensor->flash_capture = value;  

  return 0;
/*
flash_capture_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_flash_capture is failed!!!\n");  
  return -EINVAL;
  */
}

static int s5k5ccgx_set_flash_movie(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flash_movie is called...%d\n", value);

  switch(value)
  {
    case S5K5CCGX_FLASH_MOVIE_OFF:
      if(s5k5ccgx_set_flash(0))	  	
        goto flash_movie_fail;
      break;
      
    case S5K5CCGX_FLASH_MOVIE_ON:
       if(s5k5ccgx_set_flash_mode(100,true))  	
        goto flash_movie_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "Flash value is not supported!!!\n");
      goto flash_movie_fail;
  }

  sensor->flash_movie = value;

  return 0;

flash_movie_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_flash_movie is failed!!!\n");  
  return -EINVAL;      
}
/*
static int s5k5ccgx_set_flash_ctrl(bool value)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flash_ctrl is called...not support value : %d, mode : %d\n", value, sensor->flash_capture);
  switch(sensor->flash_capture)
  {
    case S5K5CCGX_FLASH_CAPTURE_OFF:
      if(s5k5ccgx_write_reg(client, sizeof(FlashAFOff_list), FlashAFOff_list))
        goto flash_ctrl_fail;
      if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureOff_list), FlashCaptureOff_list))
        goto flash_ctrl_fail;
      break;
      
    case S5K5CCGX_FLASH_CAPTURE_ON:
      if(value)
      {
        if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureOn_list), FlashCaptureOn_list))
          goto flash_ctrl_fail;
        if(s5k5ccgx_write_reg(client, sizeof(FlashAFOff_list), FlashAFOff_list))
          goto flash_ctrl_fail;
      }
      else
      {
        if(s5k5ccgx_write_reg(client, sizeof(FlashAFOn_list), FlashAFOn_list))
          goto flash_ctrl_fail;
        if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureOff_list), FlashCaptureOff_list))
          goto flash_ctrl_fail;
      }
      break;
      
    case S5K5CCGX_FLASH_CAPTURE_AUTO:
      if(value)
      {
        if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureAuto_list), FlashCaptureAuto_list))
          goto flash_ctrl_fail;
        if(s5k5ccgx_write_reg(client, sizeof(FlashAFOff_list), FlashAFOff_list))
          goto flash_ctrl_fail;
      }
      else
      {
        if(s5k5ccgx_write_reg(client, sizeof(FlashAFAuto_list), FlashAFAuto_list))
          goto flash_ctrl_fail;
        if(s5k5ccgx_write_reg(client, sizeof(FlashCaptureOff_list), FlashCaptureOff_list))
          goto flash_ctrl_fail;
      }
      break;
      
    default:
      printk(S5K5CCGX_MOD_NAME "Flash value is not supported!!!\n");
      goto flash_ctrl_fail;
  }
  return 0;
flash_ctrl_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_flash_ctrl is failed!!!\n");
  return -EINVAL;   
}
*/
static int s5k5ccgx_set_jpeg_quality(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_jpeg_quality is called... value : %d\n", value);
  
  switch(value) 
  {
    case S5K5CCGX_JPEG_SUPERFINE:
      sensor->jpeg_quality = S5K5CCGX_JPEG_SUPERFINE;
      break;
    case S5K5CCGX_JPEG_FINE:
      sensor->jpeg_quality = S5K5CCGX_JPEG_FINE;
      break;
    case S5K5CCGX_JPEG_NORMAL:
      sensor->jpeg_quality = S5K5CCGX_JPEG_NORMAL;
      break;
    case S5K5CCGX_JPEG_ECONOMY:
      sensor->jpeg_quality = S5K5CCGX_JPEG_ECONOMY;
      break;
    default:
      printk(S5K5CCGX_MOD_NAME "JPEG quality value is not supported!\n");
      goto jpeg_quality_fail;
  }
  
  return 0;

jpeg_quality_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_jpeg_quality is failed!!!\n");
  return -EINVAL;    
}

static int s5k5ccgx_set_focus_status(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

//  int cnt = 0;
  int err = 0;
//  u8 readdata = 0x00;
//  u8 status = 0x00;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_status is called...[%d]\n",value);

  if(s5k5ccgx_curr_state != S5K5CCGX_STATE_PREVIEW)
  {
    printk(S5K5CCGX_MOD_NAME "Sensor is not preview state!!");
    goto focus_status_fail;
  }

  switch(value) 
  {
    case S5K5CCGX_AF_START :
	    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "AF start.\n");
		afcanceled = false;	  
#ifdef S5K5CCGX_FLASH_SUPPORT
	       flash_auto_check = false; /* For initialization */
		preflash = 1;
		err = s5k5ccgx_set_flash_mode(15,true);
#endif
	/* delay for AE and AWB to settle down */
	if (preflash == 2)
		msleep(200);	
	
	err = s5k5ccgx_set_ae_lock(1);
	err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_AF_DO,S5K5CCGX_AF_DO_INDEX,"S5K5CCGX_AF_DO");
//  	msleep(300); //spec 133m	
  	
  	if(err < 0)
        goto focus_status_fail;
      break;
      
    case S5K5CCGX_AF_STOP :
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "AF stop.\n");
      	afcanceled = true;
	flash_auto_check =false;		
	err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_AF_OFF,S5K5CCGX_AF_OFF_INDEX,"S5K5CCGX_AF_OFF");
	msleep(200); //spec 133m	

#ifdef S5K5CCGX_FLASH_SUPPORT		
		if (preflash == 2)
		{
			printk("S5K5CCGX_PRE_FLASH_END_EVT1\n");
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_AE_SPEEDNORMAL,S5K5CCGX_AE_SPEEDNORMAL_INDEX,"S5K5CCGX_AE_SPEEDNORMAL");			
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_PRE_FLASH_END_EVT1,S5K5CCGX_PRE_FLASH_END_EVT1_INDEX,"S5K5CCGX_PRE_FLASH_END_EVT1");
		}
		preflash = 0;
			
		err = s5k5ccgx_set_flash_mode(0,false);
#endif	
		err = s5k5ccgx_set_ae_lock(0);
	  	if(err < 0)
	    	goto focus_status_fail;
		
		 if(sensor->scene == S5K5CCGX_SCENE_NIGHTSHOT) /* Extra delay for Night mode */
                        msleep(300);		
	    break;	
	 
    default:
      printk(S5K5CCGX_MOD_NAME "[af]Invalid value is ordered!!!\n");
      goto focus_status_fail;
  }	    
  return 0;

focus_status_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_status is failed!!!\n");  
  return -EINVAL;    
}

#ifdef S5K5CCGX_TOUCH_AF  

#define AF_OUTER_WINDOW_WIDTH 768
#define AF_OUTER_WINDOW_HEIGHT 640

static int s5k5ccgx_set_focus_touch(s32 value)
{
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;

	unsigned short FirstWinStartX, FirstWinStartY, SecondWinStartX, SecondWinStartY;
	int preview_width = 0, preview_height = 0;
	int err = 0;

	dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch is called...\n"); 
		
	switch(sensor->preview_size)
	{
		case S5K5CCGX_PREVIEW_SIZE_176_144:
			preview_width = 176;
			preview_height = 144;
			break;
		case S5K5CCGX_PREVIEW_SIZE_240_320:
			preview_width = 240;
			preview_height = 320;
			break;
		case S5K5CCGX_PREVIEW_SIZE_320_240:
			preview_width = 320;
			preview_height = 240;
			break;
		case S5K5CCGX_PREVIEW_SIZE_720_480:
			preview_width = 720;
			preview_height = 480;
			break;
		case S5K5CCGX_PREVIEW_SIZE_800_600:
			preview_width = 800;
			preview_height = 600;
			break;
		case S5K5CCGX_PREVIEW_SIZE_1024_600:
			preview_width = 1024;
			preview_height = 600;
			break;
		default:
			/* When running in image capture mode, the call comes here.
			 * Set the default video resolution - CE147_PREVIEW_VGA
			 */ 
			dev_err(&client->dev, "Setting preview resoution as VGA for image capture mode\n");
			break;
	}

	if(value == 1) // Touch AF start
	{
		// Prevent divided-by-zero.
		if(preview_width == 0 || preview_height == 0)
		{
			dev_err(&client->dev, "%s: Either preview_width or preview_height is zero\n", __func__);
			return -EIO;
		}

		FirstWinStartX = sensor->position.x;
		FirstWinStartY = sensor->position.y;
		
		// AF Position(Round Down)
		if(FirstWinStartX > AF_OUTER_WINDOW_WIDTH/2)
		{
			FirstWinStartX -= AF_OUTER_WINDOW_WIDTH/2;

			if(FirstWinStartX + AF_OUTER_WINDOW_WIDTH > preview_width)
			{
			 dprintk(CAM_INF, S5K5CCGX_MOD_NAME "%s: X Position Overflow : [%d, %d] \n", __func__, FirstWinStartX, AF_OUTER_WINDOW_WIDTH);
			 
			 FirstWinStartX = preview_width - AF_OUTER_WINDOW_WIDTH - 1;
			}
		}
		else
		{
			FirstWinStartX = 0;
		}

		if(FirstWinStartY > AF_OUTER_WINDOW_HEIGHT/2)
		{
			FirstWinStartY -= AF_OUTER_WINDOW_HEIGHT/2;

			if(FirstWinStartY + AF_OUTER_WINDOW_HEIGHT > preview_height)
			{
			 dprintk(CAM_INF, S5K5CCGX_MOD_NAME "%s: Y Position Overflow : [%d, %d] \n", __func__, FirstWinStartY, AF_OUTER_WINDOW_HEIGHT);
			 
			 FirstWinStartY = preview_height - AF_OUTER_WINDOW_HEIGHT - 1;
			}
		}
		else
		{
			FirstWinStartY = 0;
		}

		FirstWinStartX = (unsigned short)((FirstWinStartX * 1024) / preview_width);
		FirstWinStartY = (unsigned short)((FirstWinStartY * 1024) / preview_height);

		SecondWinStartX = FirstWinStartX + 140;
		SecondWinStartY = FirstWinStartY + 131;
		
		err  = s5k5ccgx_i2c_write(client, 0xFCFC, 0xD000);
		err += s5k5ccgx_i2c_write(client, 0x0028, 0x7000);
		err += s5k5ccgx_i2c_write(client, 0x002A, 0x022C);
		err += s5k5ccgx_i2c_write(client, 0x0F12, FirstWinStartX); 		// FirstWinStartX
		err += s5k5ccgx_i2c_write(client, 0x0F12, FirstWinStartY); 		// FirstWinStartY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0200); 				// FirstWinSizeX
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0238); 				// FirstWinSizeY
		err += s5k5ccgx_i2c_write(client, 0x0F12, SecondWinStartX);		// SecondWinStartX
		err += s5k5ccgx_i2c_write(client, 0x0F12, SecondWinStartY);		// SecondWinStartY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x00E6); 				// SecondWinSizeX
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0132); 				// SecondWinSizeY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0001); 				// WindowSizeUpdated

		dprintk(CAM_INF, S5K5CCGX_MOD_NAME "%s: Start AF Pos[%d %d]\n", __func__, FirstWinStartX, FirstWinStartY);
	}
	else if (value == 0) // Touch AF stop
	{
		
		err  = s5k5ccgx_i2c_write(client, 0xFCFC, 0xD000);
		err += s5k5ccgx_i2c_write(client, 0x0028, 0x7000);
		err += s5k5ccgx_i2c_write(client, 0x002A, 0x022C);
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0100);    // FirstWinStartX
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x00E3);    // FirstWinStartY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0200);    // FirstWinSizeX
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0238);    // FirstWinSizeY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x018C);    // SecondWinStartX
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0166);    // SecondWinStartY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x00E6);    // SecondWinSizeX
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0132);    // SecondWinSizeY
		err += s5k5ccgx_i2c_write(client, 0x0F12, 0x0001);    // WindowSizeUpdated
		
		dprintk(CAM_INF, S5K5CCGX_MOD_NAME "%s: Stop AF Pos\n", __func__);
	}
	else if (value == 2) // Stop touch AE
	{
		err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_AE_WEIGHT, S5K5CCGX_AE_WEIGHT_INDEX, "S5K5CCGX_AE_WEIGHT");
		printk("%s: TOUCH AE stop  \n", __func__);

		g_touch_enter = 0;
	}
	else if (value == 3) // start touch AE
	{
		s5k5ccgx_short_t ae_weight[S5K5CCGX_AE_WEIGHT_INDEX] ;
		unsigned int aeX=0, aeY=0;
		unsigned int pos = 0;
		if(preview_width == 0 || preview_height == 0)
		{
			dprintk(CAM_INF, S5K5CCGX_MOD_NAME "%s: Either preview_width or preview_height is zero\n", __func__);
			return -EIO;
		}

		memcpy(&ae_weight[0],S5K5CCGX_AE_WEIGHT,sizeof(S5K5CCGX_AE_WEIGHT)) ;
		
		aeX = sensor->position.x/(preview_width/8);
		aeY = sensor->position.y/(preview_height/8);

		pos = aeY*8/2 + aeX/2 + 3 ;
		
		if (pos < S5K5CCGX_AE_WEIGHT_INDEX)
		{
			if (aeX%2 == 0)
				ae_weight[pos].val |= 0x000F;
			else
				ae_weight[pos].val |= 0x0F00;
			
		}
		err = s5k5ccgx_i2c_write_block(client, ae_weight, S5K5CCGX_AE_WEIGHT_INDEX, "S5K5CCGX_AE_WEIGHT");

		printk("%s: TOUCH AE START [%d,%d] \n", __func__, aeX, aeY);
		g_touch_enter = 1;

		msleep(133);
//		s5k5ccgx_set_ae_lock(1);
	}

	if(err < 0)
	{
		dprintk(CAM_INF, S5K5CCGX_MOD_NAME "[%s : %d] ERROR! touch AF set failed\n", __FILE__, __LINE__);
		return -EIO;
	}

 return 0;
}
/*
static int s5k5ccgx_set_focus_touch(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  u16 touch_x = (u16)(value/1000);
  u16 touch_y = (u16)(value%1000);

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch is called... x : %d, y : %d\n", touch_x, touch_y); 

  u8 x_offset = 0x2b;
  u8 y_offset = 0x1b;  

  u8 Touch_AF_list[12] = {0x4D, 0x01, 0x03, 0x01,
                          0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00};

  Touch_AF_list[4]  = ((touch_x - x_offset) & 0x00FF);
  Touch_AF_list[5]  = (((touch_x - x_offset) & 0xFF00) >> 8);
  Touch_AF_list[6]  = ((touch_y - y_offset) & 0x00FF);
  Touch_AF_list[7]  = (((touch_y - y_offset) & 0xFF00) >> 8);
  Touch_AF_list[8]  = ((touch_x + x_offset) & 0x00FF);
  Touch_AF_list[9]  = (((touch_x + x_offset) & 0xFF00) >> 8);
  Touch_AF_list[10] = ((touch_y + y_offset) & 0x00FF);
  Touch_AF_list[11] = (((touch_y + y_offset) & 0xFF00) >> 8);  
  
  if(s5k5ccgx_write_reg(client, sizeof(Touch_AF_list), Touch_AF_list))
    goto focus_touch_fail;

  return 0;

focus_touch_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch is failed!!!\n");
  return -EINVAL;     
}

static int s5k5ccgx_set_focus_touch_on(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  u16 touch_x = (u16)(s5k5ccgx_preview_sizes[sensor->preview_size].width / 2);
  u16 touch_y = (u16)(s5k5ccgx_preview_sizes[sensor->preview_size].height / 2);

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch_on is called... x : %d, y : %d\n", touch_x, touch_y); 

  u8 x_offset = 0x2b;
  u8 y_offset = 0x1b;  

  u8 Touch_AF_list[12] = {0x4D, 0x01, 0x03, 0x01,
                          0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00};

  Touch_AF_list[4]  = ((touch_x - x_offset) & 0x00FF);
  Touch_AF_list[5]  = (((touch_x - x_offset) & 0xFF00) >> 8);
  Touch_AF_list[6]  = ((touch_y - y_offset) & 0x00FF);
  Touch_AF_list[7]  = (((touch_y - y_offset) & 0xFF00) >> 8);
  Touch_AF_list[8]  = ((touch_x + x_offset) & 0x00FF);
  Touch_AF_list[9]  = (((touch_x + x_offset) & 0xFF00) >> 8);
  Touch_AF_list[10] = ((touch_y + y_offset) & 0x00FF);
  Touch_AF_list[11] = (((touch_y + y_offset) & 0xFF00) >> 8);  
  
  if(s5k5ccgx_write_reg(client, sizeof(Touch_AF_list), Touch_AF_list))
    goto focus_touch_fail;

  return 0;

focus_touch_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch_on is failed!!!\n");
  return -EINVAL;     
}

static int s5k5ccgx_set_focus_touch_off(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch_off is called...\n"); 

  u8 Touch_AF_list[12] = {0x4D, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00};

  if(s5k5ccgx_write_reg(client, sizeof(Touch_AF_list), Touch_AF_list))
    goto focus_touch_fail;

  return 0;

focus_touch_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus_touch_off is failed!!!\n");
  return -EINVAL;     
}
*/
#endif

static int s5k5ccgx_get_focus(struct v4l2_control *vc)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
//	u8 readdata = 0x00;
//	u8 status = 0x00;
	int err;
	unsigned short read_value;	
	unsigned short s5k5ccgx_buf_get_af_status[1] = { 0x00 };
	dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_get_auto_focus is called...\n"); 

        s5k5ccgx_buf_get_af_status[0] = 0x00;

        if(afcanceled == true) {
                s5k5ccgx_buf_get_af_status[0] = 0x04; //cancel
                vc->value = s5k5ccgx_buf_get_af_status[0];

                err = s5k5ccgx_set_ae_lock(0);
#ifdef S5K5CCGX_FLASH_SUPPORT
                if (preflash == 2)
                {
                        preflash = 0;
                        printk("S5K5CCGX_PRE_FLASH_END_EVT1\n");

                        err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_AE_SPEEDNORMAL,S5K5CCGX_AE_SPEEDNORMAL_INDEX,"S5K5CCGX_AE_SPEEDNORMAL");
                        err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_PRE_FLASH_END_EVT1,S5K5CCGX_PRE_FLASH_END_EVT1_INDEX,"S5K5CCGX_PRE_FLASH_END_EVT1");
                        msleep(300);
                }
                err = s5k5ccgx_set_flash_mode(0,false);
#endif
          return 0;
  }

  //Check AF Result
  err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
  err=s5k5ccgx_i2c_write(client,0x002C, 0x7000);
  err=s5k5ccgx_i2c_write(client,0x002E, 0x2D12);
  err=s5k5ccgx_i2c_read(client, 0x0F12, &read_value);


  if(err < 0){
          return -EIO;
  }

  if(read_value == 0x0001){
          s5k5ccgx_buf_get_af_status[0] = 0x01; //progress, AF still ongoing...
          vc->value = s5k5ccgx_buf_get_af_status[0];
          return 0;
  }

  //idle                0x0000
  //progress    0x0001
  //success     0x0002  -> HAL 0x01
  //lowconf     0x0003
  //canceled    0x0004  -> HAL 0x02

   if(err < 0){
          return -EIO;
  }

  if(read_value == 0x0001){
          s5k5ccgx_buf_get_af_status[0] = 0x01; //progress, AF still ongoing...
          vc->value = s5k5ccgx_buf_get_af_status[0];
          return 0;
  }

  //idle                0x0000
  //progress    0x0001
  //success     0x0002  -> HAL 0x01
  //lowconf     0x0003
  //canceled    0x0004  -> HAL 0x02

  if (read_value == 0x0001){
          /* on going */
  }
  else if (read_value == 0x0002) {
          s5k5ccgx_buf_get_af_status[0] = 0x02; //success
          printk(S5K5CCGX_MOD_NAME "KKL::s5k5ccgx_buf_get_af_status == %x \n", s5k5ccgx_buf_get_af_status[0]);
  }
  else if(read_value == 0x0004) {
          s5k5ccgx_buf_get_af_status[0] = 0x04; //cancel
          printk(S5K5CCGX_MOD_NAME "KKL::s5k5ccgx_buf_get_af_status == %x \n", s5k5ccgx_buf_get_af_status[0]);
  }
  else {
          s5k5ccgx_buf_get_af_status[0] = 0x03; //fail
          printk(S5K5CCGX_MOD_NAME "KKL::s5k5ccgx_buf_get_af_status == %x \n", s5k5ccgx_buf_get_af_status[0]);
  }

  err = s5k5ccgx_set_ae_lock(0);

#ifdef S5K5CCGX_FLASH_SUPPORT
  if (preflash == 2)
  {
          preflash = 0;
          printk("S5K5CCGX_PRE_FLASH_END_EVT1\n");
          err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_AE_SPEEDNORMAL,S5K5CCGX_AE_SPEEDNORMAL_INDEX,"S5K5CCGX_AE_SPEEDNORMAL");
          err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_PRE_FLASH_END_EVT1,S5K5CCGX_PRE_FLASH_END_EVT1_INDEX,"S5K5CCGX_PRE_FLASH_END_EVT1");
          msleep(300);
  }
  err = s5k5ccgx_set_flash_mode(0,false);
#endif

  vc->value = s5k5ccgx_buf_get_af_status[0];

  return 0;
}

static int s5k5ccgx_set_focus_mode(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "AF set value = %d\n", value);

  switch(value) 
  {
    case S5K5CCGX_AF_INIT_NORMAL :        
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AF_NORMAL_ON,S5K5CCGX_AF_NORMAL_ON_INDEX,"S5K5CCGX_AF_NORMAL_ON"))
        goto focus_fail;
      break;
      
    case S5K5CCGX_AF_INIT_MACRO : 
      if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_AF_MACRO_ON,S5K5CCGX_AF_MACRO_ON_INDEX,"S5K5CCGX_AF_MACRO_ON"))
        goto focus_fail;
      break;

    case S5K5CCGX_AF_INIT_FACE :

      break;  

#ifdef S5K5CCGX_TOUCH_AF   
    case S5K5CCGX_AF_INIT_TOUCH:      

      break;      
#endif

    default:
      printk(S5K5CCGX_MOD_NAME "[af]Invalid value is ordered!!!\n");
      goto focus_fail;   
  }

  sensor->focus_mode = value;

  return 0;

focus_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_focus is failed!!!\n");
  return -EINVAL;     
}

static int s5k5ccgx_get_zoom(struct v4l2_control *vc)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  //struct i2c_client *client = sensor->i2c_client;

  //u8 readdata[2] = {0x0,};

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_get_zoom is called...\n"); 
  
//  if(s5k5ccgx_write_read_reg(client, 1, Lense_CheckDZoomStatus_List, 2, readdata))
//    goto get_zoom_fail;
  vc->value = 0;//(u8)(2560/(readdata[0] +1));
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "Zoom value... %d \n", vc->value);
  
  return 0;
/*
get_zoom_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_get_zoom is failed!!!\n");
  return -EINVAL;   
  */
}

static int s5k5ccgx_set_zoom(s32 value)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_zoom is called... value = %d\n", value); 

  switch(value)
  {
    case S5K5CCGX_ZOOM_DEFAULT:  
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_00, S5K5CCGX_ZOOM_00_INDEX,"S5K5CCGX_ZOOM_00"))
        goto zoom_fail;
      break;		
	  
    case S5K5CCGX_ZOOM_1P00X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_01, S5K5CCGX_ZOOM_01_INDEX,"S5K5CCGX_ZOOM_01"))
        goto zoom_fail;
      break;
      
    case S5K5CCGX_ZOOM_1P25X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_02, S5K5CCGX_ZOOM_02_INDEX,"S5K5CCGX_ZOOM_02"))
        goto zoom_fail;
      break;    
      
    case S5K5CCGX_ZOOM_1P50X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_03, S5K5CCGX_ZOOM_03_INDEX,"S5K5CCGX_ZOOM_03"))
        goto zoom_fail;
      break;

    case S5K5CCGX_ZOOM_1P75X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_04, S5K5CCGX_ZOOM_04_INDEX,"S5K5CCGX_ZOOM_04"))
        goto zoom_fail;
      break;

    case S5K5CCGX_ZOOM_2P00X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_05, S5K5CCGX_ZOOM_05_INDEX,"S5K5CCGX_ZOOM_05"))
        goto zoom_fail;
      break;

    case S5K5CCGX_ZOOM_2P25X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_06, S5K5CCGX_ZOOM_06_INDEX,"S5K5CCGX_ZOOM_06"))
        goto zoom_fail;
      break;

    case S5K5CCGX_ZOOM_2P50X:
      if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_ZOOM_07, S5K5CCGX_ZOOM_07_INDEX,"S5K5CCGX_ZOOM_07"))
        goto zoom_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "[zoom]Invalid value is ordered!!!\n");
      goto zoom_fail;
  }
  sensor->zoom = value;
  
  return 0;

zoom_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_zoom is failed!!!\n");
  return -EINVAL;   
}


static int s5k5ccgx_get_jpeg_size(struct v4l2_control *vc)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;
//  u8 readdata[8];

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_get_jpeg_size is called...\n"); 
  
  vc->value = 0;//(readdata[3]<<16) + (readdata[2]<<8) + readdata[1];
  //dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "s5k5ccgx_get_jpeg_size::Main JPEG Size reading... 0x%x      Q value...0x%x\n", vc->value, readdata[0]);
  
  return 0;
/*
get_jpeg_size_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_get_jpeg_size is failed!!!\n");
  return -EINVAL;     
  */
}

static int s5k5ccgx_get_thumbnail_size(struct v4l2_control *vc)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;
//  u8 readdata[8] = {0x00,};

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_get_thumbnail_size is called...\n");  

  vc->value = 0;//(readdata[3]<<16) + (readdata[2]<<8) + readdata[1];
  //dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "Thumb JPEG Size reading... 0x%x      Q value...0x%x\n", vc->value, readdata[4]);
  
  return 0;
/*
get_thumbnail_size_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_get_thumbnail_size is failed!!!\n");
  return -EINVAL;     
  */
}

static int s5k5ccgx_prepare_preview(void)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

//  u8  readdata[1] = {0x00};
  
//  int cnt = 0;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_prepare_preview is called...\n");
#if 0
  /* AF check idle status */
  do
  {
    if(s5k5ccgx_write_read_reg(client, 1, Lense_CheckStatus_List, 1, readdata))
      goto prepare_preview_fail;
    if((*readdata & 0x0F) == 0x01 || (*readdata & 0x0F) == 0x05) //moving
    {
      mdelay(10);
      cnt++; 
    }
    else
    {
      break;
    }
  }while(((*readdata & 0x0F) == 0x01 || (*readdata & 0x0F) == 0x05) && cnt < 200);  

  /* stop check */
  if(s5k5ccgx_write_read_reg(client, 1, CameraReadCommand_CaptureStatus, 1, readdata))
    goto prepare_preview_fail;

  if(*readdata != 0x00)
  {
    printk(S5K5CCGX_MOD_NAME "Preview is not stop. Need to stop!!\n");
    if(s5k5ccgx_write_reg(client, sizeof(ConfigSensorPreviewStop_list), ConfigSensorPreviewStop_list))
      goto prepare_preview_fail;
  }
#endif
  return 0;
/*
prepare_preview_fail:    
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_prepare_preview is failed!!!\n");
  return -EINVAL;     
  */
}

static int s5k5ccgx_set_preview_size(s32 value)
{
	int err=0;
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;
//	int index = state->framesize_index;

	dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_preview_size is called...[%d]\n",value);

	switch(value)
	{
		case S5K5CCGX_PREVIEW_SIZE_176_144:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_176,S5K5CCGX_PREVIEW_SIZE_176_INDEX,"S5K5CCGX_PREVIEW_SIZE_176");
			break;
		case S5K5CCGX_PREVIEW_SIZE_144_176:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_144,S5K5CCGX_PREVIEW_SIZE_144_INDEX,"S5K5CCGX_PREVIEW_SIZE_144");
			break;			
		case S5K5CCGX_PREVIEW_SIZE_240_320:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_240,S5K5CCGX_PREVIEW_SIZE_240_INDEX,"S5K5CCGX_PREVIEW_SIZE_240");
			break;
		case S5K5CCGX_PREVIEW_SIZE_320_240:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_320,S5K5CCGX_PREVIEW_SIZE_320_INDEX,"S5K5CCGX_PREVIEW_SIZE_320");
			break;
		case S5K5CCGX_PREVIEW_SIZE_720_480:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_720,S5K5CCGX_PREVIEW_SIZE_720_INDEX,"S5K5CCGX_PREVIEW_SIZE_720");
			break;
		case S5K5CCGX_PREVIEW_SIZE_800_480:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_W480,S5K5CCGX_PREVIEW_SIZE_W480_INDEX,"S5K5CCGX_PREVIEW_SIZE_W480");
			break;
		case S5K5CCGX_PREVIEW_SIZE_800_600:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_800,S5K5CCGX_PREVIEW_SIZE_800_INDEX,"S5K5CCGX_PREVIEW_SIZE_800");
			break;
		case S5K5CCGX_PREVIEW_SIZE_1024_600:
			err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_SIZE_1024,S5K5CCGX_PREVIEW_SIZE_1024_INDEX,"S5K5CCGX_PREVIEW_SIZE_1024");
			break;
		default:
			printk(S5K5CCGX_MOD_NAME "Setting preview resoution as VGA for image preview mode\n");			
			break;
	}
	sensor->preview_size = value; 

	return err;	
}

static int s5k5ccgx_start_preview(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_start_preview is called...\n");

  sensor->runmode = S5K5CCGX_RUNMODE_RUNNING;

	printk(S5K5CCGX_MOD_NAME " S5K5CCGX_FLIP  [%x]\n", sensor->set_vhflip);	
	if(sensor->set_vhflip == 1)
	{
		if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_VHFLIP_ON,
						sizeof(S5K5CCGX_VHFLIP_ON) / sizeof(S5K5CCGX_VHFLIP_ON[0]), "S5K5CCGX_VHFLIP_ON"))
		goto start_preview_fail;	
	}
	else
	{
		if(s5k5ccgx_i2c_write_block(client, S5K5CCGX_VHFLIP_OFF,
							sizeof(S5K5CCGX_VHFLIP_OFF) / sizeof(S5K5CCGX_VHFLIP_OFF[0]), "S5K5CCGX_VHFLIP_OFF"))
		goto start_preview_fail;	
	}
	printk(S5K5CCGX_MOD_NAME " S5K5CCGX_FPS  [%x]\n", sensor->fps);	
	if(s5k5ccgx_set_frame_rate(sensor->fps))
       goto start_preview_fail;			

	if(sensor->scene > 1)
	{
		if(s5k5ccgx_set_scene(sensor->scene))
		{
			dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "s5k5ccgx_start_preview scene fail[%x]\n",sensor->scene);	      	
		}
	}

	// 4. PREVIEW setting
	if (sensor->scene == S5K5CCGX_SCENE_NIGHTSHOT || sensor->scene == S5K5CCGX_SCENE_FIREWORKS) // NIGTSHOT or FIREWORKS PREVIEW
	{
		dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "NIGHTSHOT OR FIREWORKS PREVIEW \n");	
		if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_NIGHT,S5K5CCGX_PREVIEW_NIGHT_INDEX,"S5K5CCGX_PREVIEW_NIGHT"))		
		goto start_preview_fail;
		msleep(200);
	}
	else // NORMAL PREVIEW
	{	
		dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "NORMAL PREVIEW \n");	
		if(s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW,S5K5CCGX_PREVIEW_INDEX,"S5K5CCGX_PREVIEW"))		
			goto start_preview_fail;
	}
	msleep(100);

  return 0;

start_preview_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_start_preview is failed\n"); 
  return -EINVAL;  
}

static int s5k5ccgx_set_preview(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;
  
//	u8  FixedFPS_list[3] = {0x5A, 0x1E, 0x00}; 

	printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_preview is called...%d\n", sensor->preview_size);

	s5k5ccgx_pre_state = s5k5ccgx_curr_state;
	s5k5ccgx_curr_state = S5K5CCGX_STATE_PREVIEW;    

	/* Fliker 60Hz */
//	if(s5k5ccgx_write_reg(client, sizeof(ConfigSensorfliker_list), ConfigSensorfliker_list))
//	goto preview_fail;

	/* Set preview size */  
	dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "Preview size is %d\n", sensor->preview_size);

//	if(s5k5ccgx_write_reg(client, sizeof(ConfigSensorPreview[sensor->preview_size]), ConfigSensorPreview[sensor->preview_size]))
	s5k5ccgx_set_preview_size(sensor->preview_size);		

	/* Set FPS */
	dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "fps is %d. set 0x%x\n", sensor->fps, (u8)sensor->fps);
//	FixedFPS_list[1] = (u8)sensor->fps;
//	if(s5k5ccgx_write_reg(client, sizeof(FixedFPS_list), FixedFPS_list))
//	goto preview_fail;
  return 0;
/*
preview_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_preview is failed\n"); 
  return -EINVAL;
  */
}

static int s5k5ccgx_prepare_capture(void)
{
//  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
//  struct i2c_client *client = sensor->i2c_client;

//  u8  readdata[1] = {0x00};
  
//  int cnt = 0;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_prepare_capture is called...not support\n");
#if 0
  /* AF check idle status */
  do
  {
    if(s5k5ccgx_write_read_reg(client, 1, Lense_CheckStatus_List, 1, readdata))
      goto prepare_capture_fail;
    if((*readdata & 0x0F) == 0x01 || (*readdata & 0x0F) == 0x05) //moving
    {
      mdelay(10);
      cnt++; 
    }
    else
    {
      break;
    }
  }while(((*readdata & 0x0F) == 0x01 || (*readdata & 0x0F) == 0x05) && cnt < 200);  

  /* stop check */
  if(s5k5ccgx_write_read_reg(client, 1, CameraReadCommand_CaptureStatus, 1, readdata))
    goto prepare_capture_fail;
  
  if(*readdata != 0x08)
  {
    printk(S5K5CCGX_MOD_NAME "Preview is not start. Need to start!!\n");
    if(s5k5ccgx_write_reg(client, sizeof(ConfigSensorPreviewStart_list), ConfigSensorPreviewStart_list))
      goto prepare_capture_fail;
  }
#endif
  return 0;
/*
prepare_capture_fail:    
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_prepare_capture is failed!!!\n");
  return -EINVAL;     
  */
}


static int s5k5ccgx_set_capture_size(s32 value)
{
	int err=0;
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;

	int index = value;
	printk("s5k5ccgx_set_capture_size ---------index : %d\n", index);	

	switch(index)
	{
		case S5K5CCGX_IMAGE_SIZE_800_600: /* 800x600 */
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_CAPTURE_SIZE_800,S5K5CCGX_CAPTURE_SIZE_800_INDEX,"S5K5CCGX_CAPTURE_SIZE_800");	
			break;
			
		case S5K5CCGX_IMAGE_SIZE_1024_600: /* 1024x600 */
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_CAPTURE_SIZE_1024W,S5K5CCGX_CAPTURE_SIZE_1024W_INDEX,"S5K5CCGX_CAPTURE_SIZE_1024W");	
			break;
			
		case S5K5CCGX_IMAGE_SIZE_1600_960: /* 1600x960 */
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_CAPTURE_SIZE_1600W,S5K5CCGX_CAPTURE_SIZE_1600W_INDEX,"S5K5CCGX_CAPTURE_SIZE_1600W");	
			break;
			
		case S5K5CCGX_IMAGE_SIZE_1600_1200: /* 1600x1200 */
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_CAPTURE_SIZE_1600,S5K5CCGX_CAPTURE_SIZE_1600_INDEX,"S5K5CCGX_CAPTURE_SIZE_1600");	
			break;
			
		case S5K5CCGX_IMAGE_SIZE_2048_1232: /* 2048x1232 */
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_CAPTURE_SIZE_2048W,S5K5CCGX_CAPTURE_SIZE_2048W_INDEX,"S5K5CCGX_CAPTURE_SIZE_2048W");
			break;
			
		case S5K5CCGX_IMAGE_SIZE_2048_1536: /* 2048x1536 */
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_CAPTURE_SIZE_2048,S5K5CCGX_CAPTURE_SIZE_2048_INDEX,"S5K5CCGX_CAPTURE_SIZE_2048");
			break;
			
		default:		
			printk(S5K5CCGX_MOD_NAME "%s: not support capture size\n", __func__);				
			return -EINVAL;
	}

	/* Set capture image size */
	if(err < 0)
	{
		printk(S5K5CCGX_MOD_NAME "%s: failed: i2c_write for capture_resolution\n", __func__);				
		return -EIO; 
	}

	printk("s5k5ccgx_set_capture_size: %d\n", index);

	return 0;	
}

static int s5k5ccgx_start_capture(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;
//  struct v4l2_pix_format* pix = &sensor->pix;
  
//  u32 value;
//  u8 readdata[3] = {0x0,};
  int err = 0;
  unsigned long lux_value;
  unsigned short read_value = 0;
  int capture_cnt = 0, capture_timeout = 1000;
  
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_start_capture is called...\n");  
#ifdef S5K5CCGX_FLASH_SUPPORT
	// Flash on
	err = s5k5ccgx_set_flash_mode(15,true);
	if (flash_check)
	{
		msleep(200);	
		err = s5k5ccgx_set_flash_mode(0,true);
		err = s5k5ccgx_set_flash_mode(1,true);
		msleep(200);
		printk("S5K5CCGX_FLASH_START_EVT1\n");
		err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_FLASH_START_EVT1,S5K5CCGX_FLASH_START_EVT1_INDEX,"S5K5CCGX_FLASH_START_EVT1");		
		msleep(200);
		// Flash set and delay
		err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_FLASH_SET,S5K5CCGX_FLASH_SET_INDEX,"S5K5CCGX_FLASH_SET");
	}
#endif

	// Capture start
	if (sensor->scene == S5K5CCGX_SCENE_NIGHTSHOT || sensor->scene == S5K5CCGX_SCENE_FIREWORKS) // NIGTSHOT or FIREWORKS CAPTURE
	{
		err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_NIGHT_SNAPSHOT,S5K5CCGX_NIGHT_SNAPSHOT_INDEX,"S5K5CCGX_NIGHT_SNAPSHOT");
	}	
	else
	{
		lux_value = s5k5ccgx_get_illumination();

		if (lux_value > 0xFFFE) 		// highlight snapshot
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_HIGH_SNAPSHOT,S5K5CCGX_HIGH_SNAPSHOT_INDEX,"S5K5CCGX_HIGH_SNAPSHOT");		
		else if (lux_value > 0x0020) 	// Normalt snapshot
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_NORMAL_SNAPSHOT,S5K5CCGX_NORMAL_SNAPSHOT_INDEX,"S5K5CCGX_NORMAL_SNAPSHOT");		
		else 						//lowlight snapshot
			err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_LOWLIGHT_SNAPSHOT,S5K5CCGX_LOWLIGHT_SNAPSHOT_INDEX,"S5K5CCGX_LOWLIGHT_SNAPSHOT");
	}
//	msleep(200);

        if(sensor->scene == S5K5CCGX_SCENE_NIGHTSHOT){
                capture_timeout = 5000;
        }

	do 
	{	
		err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
		err=s5k5ccgx_i2c_write(client,0x002C, 0x7000);
		err=s5k5ccgx_i2c_write(client,0x002E, 0x1C22);	
		err=s5k5ccgx_i2c_read(client, 0x0F12, &read_value);
		if (read_value == 0x0001)
			break;
		else
			msleep(5);
		capture_cnt+=5;
		if(capture_cnt > capture_timeout)
		{
			printk("%s, capture timeout\n", __func__);
			break;
		}
	} while (1);
	printk("%s, capture %d msecs\n", __func__ , capture_cnt*5);

#ifdef S5K5CCGX_FLASH_SUPPORT
	// Flash off
	err = s5k5ccgx_set_flash_mode(0,false);
#endif

	// AE-AWE lock off
	err = s5k5ccgx_set_ae_lock(0);

	if (err < 0) {
		dev_err(&client->dev, "%s: camera capture. err(%d)\n", __func__, err);
		return -EIO;	/* FIXME */	
	}
  //dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "Main Image Transmit Size reading... 0x%x \n", value);

	flash_auto_check = false;
  return 0;
/*
start_capture_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_start_capture is failed\n"); 
  return -EINVAL;  
  */
}

static int s5k5ccgx_set_capture(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
  struct i2c_client *client = sensor->i2c_client;

//  u8 Write_BV_Data[3] = {0x17,0x00,0x00};
//  u8 Read_BV_Data[8] = {0x00,};  
  int err = 0;
  
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_capture is called... %d\n", sensor->capture_size);

  s5k5ccgx_pre_state = s5k5ccgx_curr_state;
  s5k5ccgx_curr_state = S5K5CCGX_STATE_CAPTURE;

  /* AE/AWB unlock */
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "AE/AWB lock.\n");
//  if(s5k5ccgx_write_reg(client, sizeof(AEWB_UnLock_list), AEWB_UnLock_list))
//    goto capture_fail;

#ifdef S5K5CCGX_FLASH_SUPPORT
	// Flash off
	err = s5k5ccgx_set_flash_mode(0,false);
	flash_check = 0;
#endif

	// Capture size
	err =s5k5ccgx_set_capture_size(sensor->capture_size);
	if(err < 0){
		printk(S5K5CCGX_MOD_NAME "%s: failed: i2c_write for capture_resolution\n", __func__);			
		return -EIO; 
	}
#if 0	
  /* Set Capture Size */  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "capture size is %d\n", sensor->capture_size);
  if(s5k5ccgx_write_reg(client, sizeof(ConfigSensorBufferingCapture[sensor->capture_size]), ConfigSensorBufferingCapture[sensor->capture_size]))
    goto capture_fail;

  /* Zoom setting */
  if(s5k5ccgx_set_zoom(sensor->zoom))
    goto capture_fail;

  /* Get BV */ 
  if(s5k5ccgx_write_read_reg(client, sizeof(Write_BV_Data), Write_BV_Data, sizeof(Read_BV_Data), Read_BV_Data))
    goto capture_fail;
  sensor->bv = Read_BV_Data[6]|(Read_BV_Data[7]*256);    

  /* AE/AWB lock */
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "AE/AWB lock.\n");
  if(s5k5ccgx_write_reg(client, sizeof(AEWB_Lock_list), AEWB_Lock_list))
    goto capture_fail;

  /* Buffering Capture Start! */
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "Buffering Capture Start.\n");
  if(s5k5ccgx_write_reg(client, sizeof(BufferingCaptureStart_list), BufferingCaptureStart_list))
    goto capture_fail;
#endif
  /* Set JPEG quality */
  dprintk(CAM_DBG,"[%s:%d] jpeq_quality is %d\n",__func__,__LINE__,sensor->jpeg_quality);

  switch(sensor->jpeg_quality)
  {
    dprintk(CAM_INF, "Quality is %d\n", sensor->jpeg_quality);
    case S5K5CCGX_JPEG_SUPERFINE:
	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x0028, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002A, 0x0426);
	err=s5k5ccgx_i2c_write(client,0x0F12, 0x005F);
	if(err < 0)
        goto capture_fail;
      break;
      
    case S5K5CCGX_JPEG_FINE:
	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x0028, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002A, 0x0426);
	err=s5k5ccgx_i2c_write(client,0x0F12, 0x0059);
	if(err < 0)	
        goto capture_fail;
      break;
      
    case S5K5CCGX_JPEG_NORMAL:
	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x0028, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002A, 0x0426);
	err=s5k5ccgx_i2c_write(client,0x0F12, 0x0052);
	if(err < 0)	
        goto capture_fail;
      break;
      
    case S5K5CCGX_JPEG_ECONOMY:
	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x0028, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002A, 0x0426);
	err=s5k5ccgx_i2c_write(client,0x0F12, 0x0052);
	if(err < 0)		
        goto capture_fail;
      break;

    default:
      printk(S5K5CCGX_MOD_NAME "Capture quality not supported\n");
      goto capture_fail;
  }

  /* Start JPEG compression */
//  if(s5k5ccgx_write_reg(client, sizeof(MakeImagesInLump[sensor->capture_size]), MakeImagesInLump[sensor->capture_size]))
//    goto capture_fail;

  return 0;

capture_fail:
  printk(S5K5CCGX_MOD_NAME "s5k5ccgx_set_capture is failed\n"); 
  return -EINVAL;     
}
/*
static void s5k5ccgx_set_skip(void)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;

  int skip_frame = 0; 

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_skip is called...\n");

  if(sensor->state == S5K5CCGX_STATE_PREVIEW)
  {      
    dprintk(CAM_INF, S5K5CCGX_MOD_NAME "BV level = %d\n", sensor->bv);

    if(s5k5ccgx_curr_state == S5K5CCGX_STATE_PREVIEW)
    {
      skip_frame = 6;
    }
    else
    {
      if(sensor->bv > 500)
      {
        if(sensor->scene == S5K5CCGX_SCENE_NIGHTSHOT)
        {
          skip_frame = sensor->fps; 
        }
        else
        {
          skip_frame = sensor->fps / 2; 
        }
      }
      else
      {
        //wait for overlay creation (250ms ~ 300ms)
        skip_frame = sensor->fps / 3; 
      }
    }
  }
  else
  {
    skip_frame = 0;
  }
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "skip frame = %d frame\n", skip_frame);

  isp_set_hs_vs(0,skip_frame);
}
*/
/*
static int s5k5ccgx_set_flip(int value)
{
	int err=-1;
	struct s5k5ccgx_sensor *sensor = &s5k5ccgx;
	struct i2c_client *client = sensor->i2c_client;
       dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flip is called...runmode[%d],value[%d]\n",sensor->runmode,value);
	   
      if(sensor->runmode != S5K5CCGX_RUNMODE_RUNNING)
		return 0;

	if(value == 1)
	{
	        dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flip::S5K5CCGX_VHFLIP_ON\n");		
		err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_VHFLIP_ON, sizeof(S5K5CCGX_VHFLIP_ON) / sizeof(S5K5CCGX_VHFLIP_ON[0]), "S5K5CCGX_VHFLIP_ON");
	}
	else
	{
	        dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flip::S5K5CCGX_VHFLIP_OFF\n");	
		err = s5k5ccgx_i2c_write_block(client, S5K5CCGX_VHFLIP_OFF, sizeof(S5K5CCGX_VHFLIP_OFF) / sizeof(S5K5CCGX_VHFLIP_OFF[0]), "S5K5CCGX_VHFLIP_OFF");
	}

	// 4. PREVIEW setting
	if (sensor->scene == S5K5CCGX_SCENE_NIGHTSHOT || sensor->scene == S5K5CCGX_SCENE_FIREWORKS) // NIGTSHOT or FIREWORKS PREVIEW
	{
	        dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flip::NIGHTSHOT OR FIREWORKS PREVIEW\n");		
		err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW_NIGHT,S5K5CCGX_PREVIEW_NIGHT_INDEX,"S5K5CCGX_PREVIEW_NIGHT");
		msleep(200);
	}
	else // NORMAL PREVIEW
	{
	        dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "s5k5ccgx_set_flip::NORMAL PREVIEW\n");				
		err = s5k5ccgx_i2c_write_block(client,S5K5CCGX_PREVIEW,S5K5CCGX_PREVIEW_INDEX,"S5K5CCGX_PREVIEW");
	}

	if(err < 0)
	{
		printk(S5K5CCGX_MOD_NAME "%s: failed: preview\n", __func__);
		return -EIO;
	}
	msleep(100);
	return err;
}
*/
static int ioctl_streamoff(struct v4l2_int_device *s)
{
  struct s5k5ccgx_sensor *sensor = s->priv;

	dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_streamoff is called...\n");
	sensor->runmode = S5K5CCGX_RUNMODE_NOTREADY;
  return 0;
}

static int ioctl_streamon(struct v4l2_int_device *s)
{
  struct s5k5ccgx_sensor *sensor = s->priv;  

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_streamon is called...(%x)\n", sensor->state);  
//  isp_set_hs_vs(0,3);
  if(sensor->state != S5K5CCGX_STATE_CAPTURE)
  {
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "start preview....................\n");
	if(sensor->check_dataline)
	{
		if(s5k5ccgx_check_dataline())
			goto streamon_fail;
	}
	else
	{
		if(s5k5ccgx_start_preview())
			goto streamon_fail;    
	}

    if(sensor->mode == S5K5CCGX_MODE_CAMCORDER)    
    {
      /* Lens focus setting */
      if(s5k5ccgx_set_focus_mode(S5K5CCGX_AF_INIT_NORMAL))
        goto streamon_fail;       
    }
#if 0 
    /* Zoom setting */
    if(s5k5ccgx_set_zoom(sensor->zoom))
      goto streamon_fail;      
#endif
  }
  else
  {
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "start capture....................\n");
    if(s5k5ccgx_start_capture())
      goto streamon_fail;
  }

  return 0;

streamon_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_streamon is failed\n"); 
  return -EINVAL;   
}

/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the s5k5ccgx_ctrl_list[] array.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
  struct v4l2_queryctrl *qc)
{
  int i;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_queryctrl is called...\n");

  for (i = 0; i < NUM_S5K5CCGX_CONTROL; i++) 
  {
    if (qc->id == s5k5ccgx_ctrl_list[i].id)
    {
      break;
    }
  }
  if (i == NUM_S5K5CCGX_CONTROL)
  {
    printk(S5K5CCGX_MOD_NAME "Control ID is not supported!!\n");
    qc->flags = V4L2_CTRL_FLAG_DISABLED;
    goto queryctrl_fail;
  }

  *qc = s5k5ccgx_ctrl_list[i];

  return 0;

queryctrl_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_queryctrl is failed\n"); 
  return -EINVAL;     
}

#define CMD_VERSION			0x00
#define DATA_VERSION_FW		0x00
#define DATA_VERSION_DATE	0x01
#define DATA_VERSION_SENSOR	0x03
#define DATA_VERSION_AF		0x05
#define DATA_VERSION_SENSOR_MAKER 0xE0
/*
static int s5k5ccgx_get_version(struct v4l2_int_device *sd, int object_id, unsigned char version_info[])
{
	struct s5k5ccgx_sensor *sensor = sd->priv;
	struct i2c_client *client = sensor->i2c_client;
	
	unsigned char cmd_buf[3] = {0x00,0x00,0x00};
	int err;
	
	cmd_buf[0] = CMD_VERSION;
	switch(object_id)
	{
	case DATA_VERSION_FW:
	case DATA_VERSION_DATE:
	case DATA_VERSION_SENSOR:
	case DATA_VERSION_AF:	
	case DATA_VERSION_SENSOR_MAKER:
		cmd_buf[1] = object_id;
		break;
	default:
		return -EINVAL;
	}
		err = s5k5ccgx_write_read_reg(client, sizeof(cmd_buf), cmd_buf, 4, version_info);
        if(err < 0)
                return -EIO;

	return 0;
}
*/
/*
static int s5k5ccgx_get_dateinfo(struct v4l2_int_device *sd)
{
    struct s5k5ccgx_sensor *sensor = sd->priv;
	unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
	int err = -1;

	err = s5k5ccgx_get_version(sd, DATA_VERSION_DATE, version_info);

	if(err < 0) 
		return  err;

	sensor->dateinfo.year  = version_info[0] - 'A' + 2007;
	sensor->dateinfo.month = version_info[1] - 'A' + 1;
	sensor->dateinfo.date  = version_info[2];

	return 0;
}
*/
/*
static int s5k5ccgx_get_sensor_version(struct v4l2_int_device *sd)
{
    struct s5k5ccgx_sensor *sensor = sd->priv;
	unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
	int err = -1;

	err = s5k5ccgx_get_version(sd, DATA_VERSION_SENSOR, version_info);

	if(err < 0) 
		return  err;

	sensor->sensor_version = version_info[0];

	return 0;
}
*/
/*
static int s5k5ccgx_get_sensor_maker_version(struct v4l2_int_device *sd)
{
    struct s5k5ccgx_sensor *sensor = sd->priv;
	unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
	int err = -1;

	err = s5k5ccgx_get_version(sd, DATA_VERSION_SENSOR_MAKER, version_info);

	if(err < 0) 
		return  err;

	sensor->sensor_info.maker = version_info[0];
	sensor->sensor_info.optical = version_info[1];

	return 0;
}
*/
/*
static int s5k5ccgx_get_af_version(struct v4l2_int_device *sd)
{
    struct s5k5ccgx_sensor *sensor = sd->priv;
	unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
	int err = -1;

	err = s5k5ccgx_get_version(sd, DATA_VERSION_AF, version_info);

	if(err < 0) 
		return  err;

	//printk("s5k5ccgx_get_af_version: data0: 0x%02x, data1: 0x%02x\n", version_info[0], version_info[1]);	

	sensor->af_info.low = version_info[1];
	sensor->af_info.high = version_info[0];

	return 0;
}
*/
/*
static int s5k5ccgx_get_gamma_version(struct v4l2_int_device *sd)
{
	struct s5k5ccgx_sensor *sensor = sd->priv;
	struct i2c_client *client = sensor->i2c_client;
	
	unsigned char gamma_info[2] = {0x00, 0x00};
	unsigned int info_len = 2;	
	int err = -1;

	unsigned char rg_low_buf[3]  = {0xE0, 0x0C, 0x00};
	unsigned char rg_high_buf[3] = {0xE0, 0x0D, 0x00};
	unsigned char bg_low_buf[3]  = {0xE0, 0x0E, 0x00};
	unsigned char bg_high_buf[3] = {0xE0, 0x0F, 0x00};	

	err = s5k5ccgx_write_read_reg(client, sizeof(rg_low_buf), rg_low_buf, info_len, gamma_info);
    if(err < 0)
            return -EIO;	

	sensor->gamma.rg_low = gamma_info[1];
	//printk("s5k5ccgx_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);		

	err = s5k5ccgx_write_read_reg(client, sizeof(rg_high_buf), rg_high_buf, info_len, gamma_info);
    if(err < 0)
            return -EIO;	

	sensor->gamma.rg_high = gamma_info[1];
	//printk("s5k5ccgx_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);	

	err = s5k5ccgx_write_read_reg(client, sizeof(bg_low_buf), bg_low_buf, info_len, gamma_info);
    if(err < 0)
            return -EIO;	
	
	sensor->gamma.bg_low = gamma_info[1];
	//printk("s5k5ccgx_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);	

	err = s5k5ccgx_write_read_reg(client, sizeof(bg_high_buf), bg_high_buf, info_len, gamma_info);
    if(err < 0)
            return -EIO;		

	sensor->gamma.bg_high= gamma_info[1];
	//printk("s5k5ccgx_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);	

	return 0;
}
*/
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
  struct s5k5ccgx_sensor *sensor = s->priv;

  int retval = 0;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_g_ctrl is called...id(%x)\n", vc->id);

  switch (vc->id) 
  {
    case V4L2_CID_SELECT_MODE:
      vc->value = sensor->mode;
      break;  
    case V4L2_CID_SELECT_STATE:
      vc->value = sensor->state;
      break;       
    case V4L2_CID_FOCUS_MODE:
      vc->value = sensor->focus_mode;
      break;  
    case V4L2_CID_AF:
      retval = s5k5ccgx_get_focus(vc);
      break;
    case V4L2_CID_ZOOM:
      retval = s5k5ccgx_get_zoom(vc);
      break;
    case V4L2_CID_JPEG_SIZE:
      retval = s5k5ccgx_get_jpeg_size(vc);
      break;
    case V4L2_CID_THUMBNAIL_SIZE:
      retval = s5k5ccgx_get_thumbnail_size(vc);
      break;
    case V4L2_CID_JPEG_QUALITY:
      vc->value = sensor->jpeg_quality;
      break;
    case V4L2_CID_ISO:
      vc->value = sensor->iso;
      break;
    case V4L2_CID_BRIGHTNESS:
      vc->value = sensor->ev;
      break;
    case V4L2_CID_CONTRAST:
      vc->value = sensor->contrast;
      break; 
    case V4L2_CID_WB:
      vc->value = sensor->wb;
      break;      
    case V4L2_CID_SATURATION:
      vc->value = sensor->saturation;
      break;
    case V4L2_CID_EFFECT:
      vc->value = sensor->effect;
      break;
    case V4L2_CID_PHOTOMETRY:
      vc->value = sensor->photometry;
      break;
    case V4L2_CID_WDR:
      vc->value = sensor->wdr;
      break;
    case V4L2_CID_SHARPNESS:
      vc->value = sensor->sharpness;
      break;
    case V4L2_CID_ISC:
      vc->value = sensor->isc;
      break;
    case V4L2_CID_AEWB:
      vc->value = sensor->aewb;
      break;
    case V4L2_CID_ANTISHAKE:
      vc->value = sensor->antishake;  
      break;
    case V4L2_CID_SCENE:
      retval = s5k5ccgx_get_scene(vc);
      break;      
    case V4L2_CID_FLASH_CAPTURE:
      vc->value = sensor->flash_capture;
      break;
    case V4L2_CID_FLASH_MOVIE:
      vc->value = sensor->flash_movie;
      break;
    case V4L2_CID_FW_THUMBNAIL_OFFSET:
      vc->value = sensor->thumb_offset;
      break;
    case V4L2_CID_FW_YUV_OFFSET:
      vc->value = sensor->yuv_offset;
      break;      
    case V4L2_CID_JPEG_CAPTURE_WIDTH:
      vc->value = sensor->jpeg_capture_w;
      break; 
    case V4L2_CID_JPEG_CAPTURE_HEIGHT:
      vc->value = sensor->jpeg_capture_h;
      break;
    case V4L2_CID_FW_VERSION:
      vc->value = ISP_FW_ver[3] | (ISP_FW_ver[2] << 8) | (ISP_FW_ver[1] << 16) | (ISP_FW_ver[0] << 24);
      break;
    case V4L2_CID_FW_LASTEST:
      vc->value = S5K5CCGX_PRM_MAJOR_VERSION | (S5K5CCGX_PRM_MINOR_VERSION << 8) |(S5K5CCGX_FW_MAJOR_VERSION << 16) | (S5K5CCGX_FW_MINOR_VERSION << 24);
      break;
    case V4L2_CID_FW_DATE:
      printk(S5K5CCGX_MOD_NAME "[S5K5CCGX] V4L2_CID_FW_DATE not supported!!!\n");	  
      break;
	case V4L2_CID_CAM_DATE_INFO_YEAR:
	  vc->value = sensor->dateinfo.year;
	  break; 
	case V4L2_CID_CAM_DATE_INFO_MONTH:
	  vc->value = sensor->dateinfo.month;
	  break; 
	case V4L2_CID_CAM_DATE_INFO_DATE:
	  vc->value = sensor->dateinfo.date;
	  break; 
	case V4L2_CID_CAM_SENSOR_VER:
	  vc->value = sensor->sensor_version;
	  break; 
	case V4L2_CID_CAM_FW_MINOR_VER:
	  vc->value = sensor->fw.minor;
	  break; 
	case V4L2_CID_CAM_FW_MAJOR_VER:
	  vc->value = sensor->fw.major;
	  break; 
	case V4L2_CID_CAM_PRM_MINOR_VER:
	  vc->value = sensor->prm.minor;
	  break; 
	case V4L2_CID_CAM_PRM_MAJOR_VER:
	  vc->value = sensor->prm.major;
	  break; 
	case V4L2_CID_CAM_SENSOR_MAKER:
	  vc->value = sensor->sensor_info.maker;
	  break; 
	case V4L2_CID_CAM_SENSOR_OPTICAL:
	  vc->value = sensor->sensor_info.optical;
	  break; 		
	case V4L2_CID_CAM_AF_VER_LOW:
	  vc->value = sensor->af_info.low;
	  break; 
	case V4L2_CID_CAM_AF_VER_HIGH:
	  vc->value = sensor->af_info.high;
	  break; 	
	case V4L2_CID_CAM_GAMMA_RG_LOW:
	  vc->value = sensor->gamma.rg_low;
	  break; 
	case V4L2_CID_CAM_GAMMA_RG_HIGH:
	  vc->value = sensor->gamma.rg_high;
	  break; 		
	case V4L2_CID_CAM_GAMMA_BG_LOW:
	  vc->value = sensor->gamma.bg_low;
	  break; 
	case V4L2_CID_CAM_GAMMA_BG_HIGH:
	  vc->value = sensor->gamma.bg_high;
	  break; 	
	case V4L2_CID_CAM_GET_DUMP_SIZE:
	  vc->value = sensor->fw_dump_size;
	  break;		
	case V4L2_CID_MAIN_SW_DATE_INFO_YEAR:
	  vc->value = sensor->main_sw_dateinfo.year;
	  break; 
	case V4L2_CID_MAIN_SW_DATE_INFO_MONTH:
	  vc->value = sensor->main_sw_dateinfo.month;
	  break; 
	case V4L2_CID_MAIN_SW_DATE_INFO_DATE:
	  vc->value = sensor->main_sw_dateinfo.date;
	  break; 
	case V4L2_CID_MAIN_SW_FW_MINOR_VER:
	  vc->value = sensor->main_sw_fw.minor;
	  break; 
	case V4L2_CID_MAIN_SW_FW_MAJOR_VER:
	  vc->value = sensor->main_sw_fw.major;
	  break; 
	case V4L2_CID_MAIN_SW_PRM_MINOR_VER:
	  vc->value = sensor->main_sw_prm.minor;
	  break; 
	case V4L2_CID_MAIN_SW_PRM_MAJOR_VER:
	  vc->value = sensor->main_sw_prm.major;
	  break; 	  
    default:
      printk(S5K5CCGX_MOD_NAME "[id]Invalid value is ordered!!!\n");
      break;
  }

  return retval;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the s5k5ccgx sensor struct).
 * Otherwise, * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
  struct s5k5ccgx_sensor *sensor = s->priv;
  int retval = 0;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_s_ctrl is called...id(%x), value(%x)\n", vc->id, vc->value);

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "sensor->check_dataline...(%x)\n", sensor->check_dataline);
	if(sensor->check_dataline)
	{
        		if(/* ( ctrl->id != V4L2_CID_CAM_PREVIEW_ONOFF ) &&*/
            		( vc->id != V4L2_CID_CAMERA_CHECK_DATALINE_STOP ) &&
            		( vc->id != V4L2_CID_CAMERA_CHECK_DATALINE ) )
       		 {
            		return 0;
        		}
	}

  switch (vc->id) 
  {
    case V4L2_CID_SELECT_MODE:
      retval = s5k5ccgx_set_mode(vc->value);
      break;  
    case V4L2_CID_SELECT_STATE:
      retval = s5k5ccgx_set_state(vc->value);
      break;       
    case V4L2_CID_FOCUS_MODE:
      retval = s5k5ccgx_set_focus_mode(vc->value);
      break;
    case V4L2_CID_AF:
      retval = s5k5ccgx_set_focus_status(vc->value);
      break;
#ifdef S5K5CCGX_TOUCH_AF         
    case V4L2_CID_AF_TOUCH:
      retval = s5k5ccgx_set_focus_touch(vc->value);
      break;      
#endif      
    case V4L2_CID_ZOOM:
      retval = s5k5ccgx_set_zoom(vc->value);
      break;
    case V4L2_CID_JPEG_QUALITY:
      retval = s5k5ccgx_set_jpeg_quality(vc->value);
      break;
    case V4L2_CID_ISO:
      retval = s5k5ccgx_set_iso(vc->value);
      break;
    case V4L2_CID_BRIGHTNESS:
      retval = s5k5ccgx_set_ev(vc->value);
      break;
    case V4L2_CID_CONTRAST:
      retval = s5k5ccgx_set_contrast(vc->value);
      break;      
    case V4L2_CID_WB:
      retval = s5k5ccgx_set_wb(vc->value);
      break;
    case V4L2_CID_SATURATION:
      retval = s5k5ccgx_set_saturation(vc->value);
      break;
    case V4L2_CID_EFFECT:
      retval = s5k5ccgx_set_effect(vc->value);
      break;
    case V4L2_CID_SCENE:
      retval = s5k5ccgx_set_scene(vc->value);
      break;
    case V4L2_CID_PHOTOMETRY:
      retval = s5k5ccgx_set_photometry(vc->value);
      break;
    case V4L2_CID_WDR:
      retval = s5k5ccgx_set_wdr(vc->value);
      break;
    case V4L2_CID_SHARPNESS:
      retval = s5k5ccgx_set_sharpness(vc->value);
      break;
    case V4L2_CID_ISC:
      retval = s5k5ccgx_set_isc(vc->value);
      break;
    case V4L2_CID_AEWB:
      retval = s5k5ccgx_set_aewb(vc->value);
      break;
    case V4L2_CID_ANTISHAKE:
      retval = s5k5ccgx_set_antishake(vc->value);
      break;      
    case V4L2_CID_FW_UPDATE:
      printk(S5K5CCGX_MOD_NAME "[S5K5CCGX] V4L2_CID_FW_UPDATE not supported!!!\n");
      break;
    case V4L2_CID_FLASH_CAPTURE:
      retval = s5k5ccgx_set_flash_capture(vc->value);
      break;
    case V4L2_CID_FLASH_MOVIE:
      retval = s5k5ccgx_set_flash_movie(vc->value);
      break;
	case V4L2_CID_CAM_FW_VER:
         printk(S5K5CCGX_MOD_NAME "[S5K5CCGX] V4L2_CID_CAM_FW_VER not supported!!!\n");		
	  break;
	case V4L2_CID_CAM_DUMP_FW:
	  retval = 0; //s5k5ccgx_dump_fw(s);
	  break;	  
	case V4L2_CID_CAMERA_CHECK_DATALINE:
	  sensor->check_dataline = vc->value;
	  retval = 0;
	  break;	
	case V4L2_CID_CAMERA_CHECK_DATALINE_STOP:
	  retval = s5k5ccgx_check_dataline_stop();
	  break;
	case V4L2_CID_CAMERA_OBJECT_POSITION_X:	  
	  sensor->position.x = vc->value;	  
	  retval = 0;
	  break;
	case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
	  sensor->position.y = vc->value;	 
	  retval = 0;	  
	  break;
	  /*
	case V4L2_CID_CAMERA_CHECK_FLIP :
         sensor->set_vhflip = vc->value;		
	  retval =  s5k5ccgx_set_flip(vc->value);	
	  break;	  
	  */
    default:
      printk(S5K5CCGX_MOD_NAME "[id]Invalid value is ordered[%x]!!!\n",vc->id);
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

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_enum_fmt_cap is called...\n");

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
          printk(S5K5CCGX_MOD_NAME "[format]Invalid value is ordered!!!\n");
          goto enum_fmt_cap_fail;
      }
      break;
      
    default:
      printk(S5K5CCGX_MOD_NAME "[type]Invalid value is ordered!!!\n");
      goto enum_fmt_cap_fail;
  }

  fmt->flags = s5k5ccgx_formats[index].flags;
  fmt->pixelformat = s5k5ccgx_formats[index].pixelformat;
  strlcpy(fmt->description, s5k5ccgx_formats[index].description, sizeof(fmt->description));

  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "ioctl_enum_fmt_cap flag : %d\n", fmt->flags);
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "ioctl_enum_fmt_cap description : %s\n", fmt->description);

  return 0;

enum_fmt_cap_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_enum_fmt_cap is failed\n"); 
  return -EINVAL;     
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
  struct s5k5ccgx_sensor *sensor = s->priv;
  struct v4l2_pix_format *pix2 = &sensor->pix;

  int index = 0;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_try_fmt_cap is called...\n");
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "ioctl_try_fmt_cap. mode : %d\n", sensor->mode);
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "ioctl_try_fmt_cap. state : %d\n", sensor->state);

//  s5k5ccgx_set_skip();  

  if(sensor->state == S5K5CCGX_STATE_CAPTURE)
  { 
    for(index = 0; index < ARRAY_SIZE(s5k5ccgx_image_sizes); index++)
    {
    if(s5k5ccgx_image_sizes[index].width == pix->width && s5k5ccgx_image_sizes[index].height == pix->height)
    {
      sensor->capture_size = index;
      break;
    }
  }   

  if(index == ARRAY_SIZE(s5k5ccgx_image_sizes))
  {
    printk(S5K5CCGX_MOD_NAME "Capture Image Size is not supported!\n");
    goto try_fmt_fail;
  }

  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--capture size = %d\n", sensor->capture_size);  
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--capture width : %d\n", s5k5ccgx_image_sizes[index].width);
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--capture height : %d\n", s5k5ccgx_image_sizes[index].height);      

  if(pix->pixelformat == V4L2_PIX_FMT_UYVY || pix->pixelformat == V4L2_PIX_FMT_YUYV)
  {
    pix->field = V4L2_FIELD_NONE;
    pix->bytesperline = pix->width * 2;
    pix->sizeimage = pix->bytesperline * pix->height;
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "V4L2_PIX_FMT_UYVY\n");
  }
  else
  {
    pix->field = V4L2_FIELD_NONE;
    pix->bytesperline = JPEG_CAPTURE_WIDTH * 2;
    pix->sizeimage = pix->bytesperline * JPEG_CAPTURE_HEIGHT;
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "V4L2_PIX_FMT_JPEG\n");
  }

  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "set capture....................\n");

  if(s5k5ccgx_set_capture())
    goto try_fmt_fail;
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
  printk(S5K5CCGX_MOD_NAME "ioctl_try_fmt_cap is failed\n"); 
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
  struct s5k5ccgx_sensor *sensor = s->priv;
  struct v4l2_pix_format *pix2 = &sensor->pix;
  struct i2c_client *client = sensor->i2c_client;

  int index = 0;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_s_fmt_cap is called...\n");

  s5k5ccgx_720p_enable = false;

  printk(S5K5CCGX_MOD_NAME "camera mode  : %d (1:camera , 2:camcorder, 3:vt)\n", sensor->mode);
  printk(S5K5CCGX_MOD_NAME "camera state : %d (0:preview, 1:snapshot)\n", sensor->state);
  printk(S5K5CCGX_MOD_NAME "set width  : %d\n", pix->width);
  printk(S5K5CCGX_MOD_NAME "set height : %d\n", pix->height); 

  if(sensor->state == S5K5CCGX_STATE_CAPTURE)
  { 
    /* check for capture */
    if(s5k5ccgx_prepare_capture())
      goto s_fmt_fail;   

//    s5k5ccgx_set_skip();  
    
    for(index = 0; index < ARRAY_SIZE(s5k5ccgx_image_sizes); index++)
    {
      if(s5k5ccgx_image_sizes[index].width == pix->width && s5k5ccgx_image_sizes[index].height == pix->height)
      {
        sensor->capture_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5k5ccgx_image_sizes))
    {
      printk(S5K5CCGX_MOD_NAME "Capture Image %d x %d Size is not supported!\n", pix->width, pix->height);
      goto s_fmt_fail;
    }

    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--capture size = %d\n", sensor->capture_size);  
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--capture width : %d\n", s5k5ccgx_image_sizes[index].width);
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--capture height : %d\n", s5k5ccgx_image_sizes[index].height);      

    if(pix->pixelformat == V4L2_PIX_FMT_UYVY || pix->pixelformat == V4L2_PIX_FMT_YUYV)
    {
      pix->field = V4L2_FIELD_NONE;
      pix->bytesperline = pix->width * 2;
      pix->sizeimage = pix->bytesperline * pix->height;
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "V4L2_PIX_FMT_UYVY\n");
    }
    else
    {
      pix->field = V4L2_FIELD_NONE;
      pix->bytesperline = JPEG_CAPTURE_WIDTH * 2;
      pix->sizeimage = pix->bytesperline * JPEG_CAPTURE_HEIGHT;
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "V4L2_PIX_FMT_JPEG\n");
    }

#if 1
  /* For CTS Test #testPreviewPictureSizesCombination */
  if((sensor->preview_size == S5K5CCGX_PREVIEW_SIZE_1024_600) && 
		  (sensor->capture_size == S5K5CCGX_IMAGE_SIZE_2048_1536)) {

	  printk("[%s:%d] Abnormal capture case!!....................\n",__func__, __LINE__);
	  s5k5ccgx_i2c_write_block(client, S5K5CCGX_ABNORMAL_CAPTURE,S5K5CCGX_ABNORMAL_CAPTURE_INDEX ,"S5K5CCGX_ABNORMAL_CAPTURE");	
	  mdelay(150);
	  if(s5k5ccgx_set_capture())
		  goto s_fmt_fail;
  }
  if(s5k5ccgx_set_capture())
	  goto s_fmt_fail;
#else
    if(s5k5ccgx_set_capture())
      goto s_fmt_fail;
#endif
  } 
  else
  {  
    /* check for preview */
    if(s5k5ccgx_prepare_preview())
      goto s_fmt_fail;

//    s5k5ccgx_set_skip();  
  
    for(index = 0; index < ARRAY_SIZE(s5k5ccgx_preview_sizes); index++)
    {
      if(s5k5ccgx_preview_sizes[index].width == pix->width && s5k5ccgx_preview_sizes[index].height == pix->height)
      {
        sensor->preview_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5k5ccgx_preview_sizes))
    {
      printk(S5K5CCGX_MOD_NAME "Preview Image %d x %d Size is not supported!\n", pix->width, pix->height);
      goto s_fmt_fail;
    }

    if(sensor->mode == S5K5CCGX_MODE_CAMCORDER)
    {
      if(pix->width == 1280 && pix->height == 720)
      {
        dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "Preview Image Size is 720P!\n");
        s5k5ccgx_720p_enable = true;
      }
    }

    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--preview size = %d\n", sensor->preview_size); 
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--preview width : %d\n", s5k5ccgx_preview_sizes[index].width);
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "S5K5CCGX--preview height : %d\n", s5k5ccgx_preview_sizes[index].height);      
    
    pix->field = V4L2_FIELD_NONE;
    pix->bytesperline = pix->width * 2;
    pix->sizeimage = pix->bytesperline * pix->height;  
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "V4L2_PIX_FMT_UYVY\n");

    if(s5k5ccgx_set_preview())
      goto s_fmt_fail;
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
  printk(S5K5CCGX_MOD_NAME "ioctl_s_fmt_cap is failed\n"); 
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
  struct s5k5ccgx_sensor *sensor = s->priv;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_g_fmt_cap is called...\n");

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
  struct s5k5ccgx_sensor *sensor = s->priv;
  struct v4l2_captureparm *cparm = &a->parm.capture;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_g_parm is called...\n");

  if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
  {
    printk(S5K5CCGX_MOD_NAME "ioctl_g_parm type not supported.\n");
    goto g_parm_fail;
  }

  memset(a, 0, sizeof(*a));
  a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  cparm->capability = V4L2_CAP_TIMEPERFRAME;
  cparm->timeperframe = sensor->timeperframe;

  return 0;

g_parm_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_g_parm is failed\n"); 
  return -EINVAL;  
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
  struct s5k5ccgx_sensor *sensor = s->priv;
  struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_s_parm is called...\n");

  /* Set mode (camera/camcorder/vt) & state (preview/capture) */
  sensor->mode = a->parm.capture.capturemode;
  sensor->state = a->parm.capture.currentstate;

  if(sensor->mode < 1 || sensor->mode > 3) sensor->mode = S5K5CCGX_MODE_CAMERA;
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "mode = %d, state = %d\n", sensor->mode, sensor->state);   

  /* Set time per frame (FPS) */
  if((timeperframe->numerator == 0)&&(timeperframe->denominator == 0))
  {
    sensor->fps = 30;
  }
  else
  {
    sensor->fps = timeperframe->denominator / timeperframe->numerator;
  }
  sensor->timeperframe = *timeperframe;
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "fps = %d\n", sensor->fps);  
  
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
  struct s5k5ccgx_sensor *sensor = s->priv;
  int rval;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_g_ifparm is called...\n");

  rval = sensor->pdata->ifparm(p);
  if (rval)
  {
    return rval;
  }

  p->u.bt656.clock_curr = S5K5CCGX_XCLK;

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
  struct s5k5ccgx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_g_priv is called...\n");

  if(p == NULL)
  {
    printk(S5K5CCGX_MOD_NAME "ioctl_g_priv is failed because of null pointer\n"); 
    return -EINVAL;
  }
  
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
  struct s5k5ccgx_sensor* sensor = s->priv;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_enum_framesizes called... sensor state : %d\n", sensor->state);   

  if (sensor->state == S5K5CCGX_STATE_CAPTURE)
  {    
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "[%s:%d]Size enumeration for image capture size = %d\n",
		    __func__,__LINE__, sensor->capture_size);

    if(sensor->preview_size == ARRAY_SIZE(s5k5ccgx_image_sizes))
      goto enum_framesizes_fail;

    frms->index = sensor->capture_size;
    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = s5k5ccgx_image_sizes[sensor->capture_size].width;
    frms->discrete.height = s5k5ccgx_image_sizes[sensor->capture_size].height;        
  }
  else
  {
    dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "[%s:%d]Size enumeration for image preview size = %d\n", 
		    __func__, __LINE__, sensor->preview_size);

    if(sensor->preview_size == ARRAY_SIZE(s5k5ccgx_preview_sizes))
      goto enum_framesizes_fail;
    
    frms->index = sensor->preview_size;
    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = s5k5ccgx_preview_sizes[sensor->preview_size].width;
    frms->discrete.height = s5k5ccgx_preview_sizes[sensor->preview_size].height;        
  }
  
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "framesizes width : %d\n", frms->discrete.width); 
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "framesizes height : %d\n", frms->discrete.height); 
  
  return 0;

enum_framesizes_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_enum_framesizes is failed\n"); 
  return -EINVAL;   
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *frmi)
{
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_enum_frameintervals \n"); 
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "ioctl_enum_frameintervals numerator : %d\n", frmi->discrete.numerator); 
  dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "ioctl_enum_frameintervals denominator : %d\n", frmi->discrete.denominator); 

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
  struct s5k5ccgx_sensor *sensor = s->priv;
  struct i2c_client *client = sensor->i2c_client;

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_s_power is called......ON=%x, detect= %x\n", on, sensor->detect);

  if(sensor->pdata->power_set(on))
  {
    printk(S5K5CCGX_MOD_NAME "Can not power on/off " S5K5CCGX_DRIVER_NAME " sensor\n"); 
    goto s_power_fail;
  }
//idle current optimisation 
  if(on == V4L2_POWER_ON)
	  back_cam_in_use= 1 ;
  else if(on == V4L2_POWER_OFF)
	  back_cam_in_use = 0 ;
  
  switch(on)
  {
    case V4L2_POWER_ON:
    {
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "pwr on-----!\n");
      if(s5k5ccgx_detect(client)) 
      {
        printk(S5K5CCGX_MOD_NAME "Unable to detect " S5K5CCGX_DRIVER_NAME " sensor\n");
        sensor->pdata->power_set(V4L2_POWER_OFF);
        goto s_power_fail;
      }

      /* Make the default detect */
      sensor->detect = SENSOR_DETECTED;     

      /* Make the state init */
      s5k5ccgx_curr_state = S5K5CCGX_STATE_INVALID;
      force_disable_tsp_autocal();	  
    }
    break;

    case V4L2_POWER_RESUME:
    {
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "pwr resume-----!\n");
    }  
    break;

    case V4L2_POWER_STANDBY:
    {
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "pwr stanby-----!\n");
    }
    break;

    case V4L2_POWER_OFF:
    {
      dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "pwr off-----!\n");

      /* Make the default detect */
      sensor->detect = SENSOR_NOT_DETECTED;  

      /* Make the state init */
      s5k5ccgx_pre_state = S5K5CCGX_STATE_INVALID;      
    }
    break;
  }

  return 0;

s_power_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_s_power is failed\n");
  return -EINVAL;
}



static int ioctl_g_exif(struct v4l2_int_device *s, struct v4l2_exif *exif)
{
  struct s5k5ccgx_sensor *sensor = s->priv;
  struct i2c_client *client = sensor->i2c_client;
  
  	int err = 0;
	unsigned short iso_read_value;
	unsigned long shutter_read_value = 0;
	dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_g_exif is called...\n");

	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x002C, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002E, 0x2A18);
	err=s5k5ccgx_i2c_read(client, 0x0F12, &iso_read_value);

	if ( 256 <= iso_read_value  && iso_read_value <= 486 ) exif->iso=50;
	else if ( 486 < iso_read_value  && iso_read_value <= 588 ) exif->iso=100;
	else if ( 588 < iso_read_value  && iso_read_value <= 716 ) exif->iso=200;
	else exif->iso=400;
  	dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "EXIF->iso: 0x%x\n", exif->iso);
	
	err=s5k5ccgx_i2c_write(client,0xFCFC, 0xD000);
	err=s5k5ccgx_i2c_write(client,0x002C, 0x7000);
	err=s5k5ccgx_i2c_write(client,0x002E, 0x2A14);
	err=s5k5ccgx_i2c_read_multi(client, 0x0F12, &shutter_read_value);

	exif->shutter_speed_numerator = (shutter_read_value * 10) / 4 ; // (ms) -> us

	exif->flash =  sensor->flashstate;

  	dprintk(CAM_DBG, S5K5CCGX_MOD_NAME "EXIF->shutter_speed: 0x%x\n", exif->shutter_speed_numerator);
	
  return 0;
/*
g_exif_fail:
  printk(S5K5CCGX_MOD_NAME "ioctl_g_exif is failed\n");
  return -EINVAL;  
  */
}

/**
 * ioctl_deinit - V4L2 sensor interface handler for VIDIOC_INT_DEINIT
 * @s: pointer to standard V4L2 device structure
 *
 * Deinitialize the sensor device
 */
static int ioctl_deinit(struct v4l2_int_device *s)
{
  struct s5k5ccgx_sensor *sensor = s->priv;

  dprintk(CAM_INF, "ioctl_deinit is called...\n");

  sensor->state = S5K5CCGX_STATE_INVALID; //init problem
  
  return 0;
}


/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call s5k5ccgx_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
  struct s5k5ccgx_sensor *sensor = s->priv;
  
  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "ioctl_init is called...\n");

  //init value
  sensor->timeperframe.numerator    = 1;
  sensor->timeperframe.denominator  = 30;
  sensor->fps                       = 30;
  sensor->bv                        = 0;
  sensor->state                     = S5K5CCGX_STATE_INVALID;
  sensor->mode                      = S5K5CCGX_MODE_CAMERA;
  sensor->preview_size              = S5K5CCGX_PREVIEW_SIZE_640_480;
  sensor->capture_size              = S5K5CCGX_IMAGE_SIZE_2560_1920;
  sensor->detect                    = SENSOR_NOT_DETECTED;
  sensor->focus_mode                = S5K5CCGX_AF_INIT_NORMAL;
  sensor->effect                    = S5K5CCGX_EFFECT_OFF;
  sensor->iso                       = S5K5CCGX_ISO_AUTO;
  sensor->photometry                = S5K5CCGX_PHOTOMETRY_CENTER;
  sensor->ev                        = S5K5CCGX_EV_DEFAULT;
  sensor->wdr                       = S5K5CCGX_WDR_OFF;
  sensor->contrast                  = S5K5CCGX_CONTRAST_DEFAULT;
  sensor->saturation                = S5K5CCGX_SATURATION_DEFAULT;
  sensor->sharpness                 = S5K5CCGX_SHARPNESS_DEFAULT;
  sensor->wb                        = S5K5CCGX_WB_AUTO;
  sensor->isc                       = S5K5CCGX_ISC_STILL_OFF;
  sensor->scene                     = S5K5CCGX_SCENE_OFF;
  sensor->aewb                      = S5K5CCGX_AE_UNLOCK_AWB_UNLOCK;
  sensor->antishake                 = S5K5CCGX_ANTI_SHAKE_OFF;
  sensor->flash_capture             = S5K5CCGX_FLASH_CAPTURE_OFF;
  sensor->flash_movie               = S5K5CCGX_FLASH_MOVIE_OFF;
  sensor->jpeg_quality              = S5K5CCGX_JPEG_SUPERFINE;
  sensor->zoom                      = S5K5CCGX_ZOOM_1P00X;
  sensor->thumb_offset              = S5K5CCGX_THUMBNAIL_OFFSET;
  sensor->yuv_offset                = S5K5CCGX_YUV_OFFSET;
  sensor->jpeg_capture_w            = JPEG_CAPTURE_WIDTH;
  sensor->jpeg_capture_h            = JPEG_CAPTURE_HEIGHT;  

  memcpy(&s5k5ccgx, sensor, sizeof(struct s5k5ccgx_sensor));
  	
  return 0;
}

static struct v4l2_int_ioctl_desc s5k5ccgx_ioctl_desc[] = {
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

static struct v4l2_int_slave s5k5ccgx_slave = {
  .ioctls = s5k5ccgx_ioctl_desc,
  .num_ioctls = ARRAY_SIZE(s5k5ccgx_ioctl_desc),
};

static struct v4l2_int_device s5k5ccgx_int_device = {
  .module = THIS_MODULE,
  .name = S5K5CCGX_DRIVER_NAME,
  .priv = &s5k5ccgx,
  .type = v4l2_int_type_slave,
  .u = {
    .slave = &s5k5ccgx_slave,
  },
};


/**
 * s5k5ccgx_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int s5k5ccgx_probe(struct i2c_client *client, const struct i2c_device_id *device)
{
  struct s5k5ccgx_sensor *sensor = &s5k5ccgx;

  if (i2c_get_clientdata(client))
  {
    printk(S5K5CCGX_MOD_NAME "can't get i2c client data!!\n");
    return -EBUSY;
  }

  sensor->pdata = &nowplus_s5k5ccgx_platform_data;

  if (!sensor->pdata) 
  {
    printk(S5K5CCGX_MOD_NAME "no platform data!!\n");
    return -ENODEV;
  }

  sensor->v4l2_int_device = &s5k5ccgx_int_device;

#if defined(S5K5CCGX_USE_GPIO_I2C)
// s5k5ccgx_i2c_client = (OMAP_GPIO_I2C_CLIENT *)client;
#else
  sensor->i2c_client = client;
#endif    

  /* Make the default capture size VGA */
  sensor->pix.width = 640;
  sensor->pix.height = 480;

  /* Make the default capture format V4L2_PIX_FMT_UYVY */
  sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;

  i2c_set_clientdata(client, sensor);

  if (v4l2_int_device_register(sensor->v4l2_int_device))
  {
    printk(S5K5CCGX_MOD_NAME "fail to init device register \n");
    i2c_set_clientdata(client, NULL);
  }

  return 0;
}

/**
 * s5k5ccgx_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of s5k5ccgx_probe().
 */
static int __exit s5k5ccgx_remove(struct i2c_client *client)
{
  struct s5k5ccgx_sensor *sensor = i2c_get_clientdata(client);

  dprintk(CAM_INF, S5K5CCGX_MOD_NAME "s5k5ccgx_remove is called...\n");

  if (!client->adapter)
  {
    printk(S5K5CCGX_MOD_NAME "no i2c client adapter!!");
    return -ENODEV; /* our client isn't attached */
  }

  v4l2_int_device_unregister(sensor->v4l2_int_device);
  i2c_set_clientdata(client, NULL);

  return 0;
}

static const struct i2c_device_id s5k5ccgx_id[] = {
  { S5K5CCGX_DRIVER_NAME, 0 },
  { },
};

MODULE_DEVICE_TABLE(i2c, s5k5ccgx_id);


static struct i2c_driver s5k5ccgxsensor_i2c_driver = {
  .driver = {
    .name = S5K5CCGX_DRIVER_NAME,
  },
  .probe = s5k5ccgx_probe,
  .remove = __exit_p(s5k5ccgx_remove),
  .id_table = s5k5ccgx_id,
};

/**
 * s5k5ccgx_sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init s5k5ccgx_sensor_init(void)
{
  int err;

  dprintk(CAM_INF, "s5k5ccgx_sensor_init is called...\n");

#if defined(S5K5CCGX_USE_GPIO_I2C)
	s5k5ccgx_i2c_client = omap_gpio_i2c_init(OMAP_GPIO_CAM_SDA,
						  OMAP_GPIO_CAM_SCL,
						  S5K5CCGX_I2C_ADDR,
						  200);
		printk("[S5K5CCGX] omap_gpio_i2c_init [%x]!\n",s5k5ccgx_i2c_client);
	if(s5k5ccgx_i2c_client == NULL)
	{
		printk(KERN_ERR "[S5K5CCGX] omap_gpio_i2c_init failed!\n");
	}
#endif  
  err = i2c_add_driver(&s5k5ccgxsensor_i2c_driver);
  if (err) 
  {
    printk(S5K5CCGX_MOD_NAME "Failed to register" S5K5CCGX_DRIVER_NAME ".\n");
    return err;
  }

  return 0;
}

module_init(s5k5ccgx_sensor_init);

/**
 * s5k5ccgxsensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of s5k5ccgx_sensor_init.
 */
static void __exit s5k5ccgxsensor_cleanup(void)
{
#if defined(S5K5CCGX_USE_GPIO_I2C)
	omap_gpio_i2c_deinit(s5k5ccgx_i2c_client);
#else
 	 i2c_del_driver(&s5k5ccgxsensor_i2c_driver);
#endif
}
module_exit(s5k5ccgxsensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5K5CCGX camera sensor driver");
