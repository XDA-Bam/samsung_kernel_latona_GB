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
 * modules/camera/s5ka3dfx.c
 *
 * S5KA3DFX sensor driver source file
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
#include "s5ka3dfx.h"
#include "s5ka3dfx_tune.h"

bool front_cam_in_use= false;


#if (CAM_S5KA3DFX_DBG_MSG)
#include "dprintk.h"
#else
#define dprintk(x, y...)
#endif

#define I2C_M_WRITE 0x0000 /* write data, from slave to master */
#define I2C_M_READ  0x0001 /* read data, from slave to master */

static u32 s5ka3dfx_curr_state = S5KA3DFX_STATE_INVALID;
static u32 s5ka3dfx_pre_state = S5KA3DFX_STATE_INVALID;

/* Section Index */
static int reg_init_qcif_index;
static int reg_init_cif_index;
static int reg_init_qvga_index;
static int reg_init_vga_index;
static int reg_init_qcif_vt_index;
static int reg_init_cif_vt_index;
static int reg_init_qvga_vt_index;
static int reg_init_vga_vt_index;
static int reg_wb_auto_index;
static int reg_wb_daylight_index;
static int reg_wb_cloudy_index;
static int reg_wb_incandescent_index;
static int reg_wb_fluorescent_index;
static int reg_ev_index;
static int reg_ev_vt_index;
static int reg_contrast_level_m5_index;
static int reg_contrast_level_m4_index;
static int reg_contrast_level_m3_index;
static int reg_contrast_level_m2_index;
static int reg_contrast_level_m1_index;
static int reg_contrast_level_default_index;
static int reg_contrast_level_p1_index;
static int reg_contrast_level_p2_index;
static int reg_contrast_level_p3_index;
static int reg_contrast_level_p4_index;
static int reg_contrast_level_p5_index;
static int reg_effect_none_index;
static int reg_effect_red_index;
static int reg_effect_gray_index;
static int reg_effect_sepia_index;
static int reg_effect_green_index;
static int reg_effect_aqua_index;
static int reg_effect_negative_index;
static int reg_flip_none_index;
static int reg_flip_water_index;
static int reg_flip_mirror_index;
static int reg_flip_water_mirror_index;
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
static int reg_15fps_index;
static int reg_self_capture_index;

static struct s5ka3dfx_sensor s5ka3dfx = {
  .timeperframe = {
    .numerator    = 1,
    .denominator  = 15,
  },
  .mode           = S5KA3DFX_MODE_CAMERA,  
  .state          = S5KA3DFX_STATE_PREVIEW,
  .fps            = 15,
  .preview_size   = S5KA3DFX_PREVIEW_SIZE_640_480,
  .capture_size   = S5KA3DFX_IMAGE_SIZE_640_480,
  .detect         = SENSOR_NOT_DETECTED,
  .zoom           = S5KA3DFX_ZOOM_1P00X,
  .effect         = S5KA3DFX_EFFECT_OFF,
  .ev             = S5KA3DFX_EV_DEFAULT,
  .contrast       = S5KA3DFX_CONTRAST_DEFAULT,
  .wb             = S5KA3DFX_WB_AUTO,
  .pretty         = S5KA3DFX_PRETTY_NONE,
  .flip           = S5KA3DFX_FLIP_NONE,
};

struct v4l2_queryctrl s5ka3dfx_ctrl_list[] = {
  {
    .id            = V4L2_CID_SELECT_MODE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "select mode",
    .minimum       = S5KA3DFX_MODE_CAMERA,
    .maximum       = S5KA3DFX_MODE_VT,
    .step          = 1,
    .default_value = S5KA3DFX_MODE_CAMERA,
  }, 
  {
    .id            = V4L2_CID_SELECT_STATE,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "select state",
    .minimum       = S5KA3DFX_STATE_PREVIEW,
    .maximum       = S5KA3DFX_STATE_CAPTURE,
    .step          = 1,
    .default_value = S5KA3DFX_STATE_PREVIEW,
  }, 
  {
    .id            = V4L2_CID_ZOOM,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Zoom",
    .minimum       = S5KA3DFX_ZOOM_1P00X,
    .maximum       = S5KA3DFX_ZOOM_4P00X,
    .step          = 1,
    .default_value = S5KA3DFX_ZOOM_1P00X,
  },
  {
    .id            = V4L2_CID_BRIGHTNESS,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Brightness",
    .minimum       = S5KA3DFX_EV_MINUS_2P0,
    .maximum       = S5KA3DFX_EV_PLUS_2P0,
    .step          = 1,
    .default_value = S5KA3DFX_EV_DEFAULT,
  },
  {
    .id            = V4L2_CID_WB,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "White Balance",
    .minimum       = S5KA3DFX_WB_AUTO,
    .maximum       = S5KA3DFX_WB_FLUORESCENT,
    .step          = 1,
    .default_value = S5KA3DFX_WB_AUTO,
  },
  {
    .id            = V4L2_CID_CONTRAST,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Contrast",
    .minimum       = S5KA3DFX_CONTRAST_MINUS_3,
    .maximum       = S5KA3DFX_CONTRAST_PLUS_3,
    .step          = 1,
    .default_value = S5KA3DFX_CONTRAST_DEFAULT,
  },
  {
    .id            = V4L2_CID_EFFECT,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Effect",
    .minimum       = S5KA3DFX_EFFECT_OFF,
    .maximum       = S5KA3DFX_EFFECT_PURPLE,
    .step          = 1,
    .default_value = S5KA3DFX_EFFECT_OFF,
  },
  {
    .id            = V4L2_CID_FLIP,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Flip",
    .minimum       = S5KA3DFX_FLIP_NONE,
    .maximum       = S5KA3DFX_FLIP_WATER_MIRROR,
    .step          = 1,
    .default_value = S5KA3DFX_FLIP_NONE,
  },
  {
    .id            = V4L2_CID_PRETTY,
    .type          = V4L2_CTRL_TYPE_INTEGER,
    .name          = "Pretty",
    .minimum       = S5KA3DFX_PRETTY_NONE,
    .maximum       = S5KA3DFX_PRETTY_LEVEL3,
    .step          = 1,
    .default_value = S5KA3DFX_PRETTY_NONE,
  },  
};
#define NUM_S5KA3DFX_CONTROL ARRAY_SIZE(s5ka3dfx_ctrl_list)

/* list of image formats supported by s5ka3dfx sensor */
const static struct v4l2_fmtdesc s5ka3dfx_formats[] = {
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
#define NUM_S5KA3DFX_FORMATS ARRAY_SIZE(s5ka3dfx_formats)

extern struct s5ka3dfx_platform_data nowplus_s5ka3dfx_platform_data;

static int s5ka3dfx_i2c_write_read(struct i2c_client *client, u8 writedata_num, const u8* writedata, u8 readdata_num, u8* readdata)
{
  int err = 0, i = 0;
  struct i2c_msg msg[1];
  unsigned char writebuf[writedata_num];
  unsigned char readbuf[readdata_num];

  if (!client->adapter)
  {
    printk(S5KA3DFX_MOD_NAME "can't search i2c client adapter\n");
    return -ENODEV;
  }

  /* Write */
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
    printk(S5KA3DFX_MOD_NAME "s5ka3dfx_i2c_write_read is failed... %d\n", err);
    return err;  
  }

  /* Read */
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

  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_i2c_write_read is failed... %d\n", err);

  return err;
}

static int s5ka3dfx_i2c_write(struct i2c_client *client, unsigned char length, u8 readdata[])
{
  unsigned char buf[length], i = 0;
  struct i2c_msg msg = {client->addr, I2C_M_WRITE, length, buf};
  int err = 0;

  if (!client->adapter)
  {
    printk(S5KA3DFX_MOD_NAME "can't search i2c client adapter\n");
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

  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_i2c_write is failed... %d\n", err);

  return err;
}

static int s5ka3dfx_set_data(struct i2c_client *client, u32 code)
{
  u8 data[2] = {0x00,};

  u32 cnt = 0;
  int ret = 0;

  data[0] = (u8)((code & 0x0000ff00) >> 8);
  data[1] = (u8)((code & 0x000000ff) >> 0);

  if (data[0] == 0xff)
  {
    goto data_fail;
  }
  else
  {
    while(cnt < 5)
    {
      ret = s5ka3dfx_i2c_write(client, sizeof(data), data);

      if(ret == 0)
      {
        break;
      }
      else
      {
        printk(S5KA3DFX_MOD_NAME "s5ka3dfx_i2c_write i2c write error....retry...ret=%d \n",ret);
        mdelay(5);
        cnt++;
      }
    }
  }

  if(cnt == 5)
  {
    goto data_fail;
  }

  return 0;

data_fail:  
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_data is failed!!\n");
  return -EINVAL; 
}	/* camsensor_s5ka3dfx_set_data */

static int s5ka3dfx_set_flip(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;

  int i;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_flip is called... value = %d\n", value);

  switch(value)
  {
    case S5KA3DFX_FLIP_NONE:
      for (i = 0; i < reg_flip_none_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_flip_none_table[i]))
          goto flip_fail;
      }       
      break;      
      
    case S5KA3DFX_FLIP_MIRROR:
      for (i = 0; i < reg_flip_mirror_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_flip_mirror_table[i]))
          goto flip_fail;
      }       
      break;      
     
    case S5KA3DFX_FLIP_WATER:
      for (i = 0; i < reg_flip_water_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_flip_water_table[i]))
          goto flip_fail;
      }       
      break; 
      
    case S5KA3DFX_FLIP_WATER_MIRROR:
      for (i = 0; i < reg_flip_water_mirror_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_flip_water_mirror_table[i]))
          goto flip_fail;
      }       
      break;       
      
    default:
      printk(S5KA3DFX_MOD_NAME "[Flip]Invalid value is ordered!!!\n");
      goto flip_fail;
  }
  
  sensor->flip = value;

  return 0;

flip_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_flip is failed!!!\n");
  return -EINVAL; 
}

static void s5ka3dfx_set_skip(void)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;

  int skip_frame = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_skip is called...\n");

  if(sensor->state == S5KA3DFX_STATE_PREVIEW)
  {
    if(s5ka3dfx_curr_state == S5KA3DFX_STATE_PREVIEW)
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
    skip_frame = 3;
  }
  
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "skip frame = %d frame\n", skip_frame);

  isp_set_hs_vs(0,skip_frame);
}

static int s5ka3dfx_set_fps(void)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client; 

  int i = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_fps is called... state = %d\n", sensor->state);

  if(sensor->state != S5KA3DFX_STATE_CAPTURE)  
  {
    dprintk(CAM_INF, "s5ka3dfx_set_fps is called... size = %d\n", sensor->preview_size);
    dprintk(CAM_INF, "s5ka3dfx_set_fps is called... fps = %d\n", sensor->fps);
    
    switch(sensor->fps)
    {
      case 15:
        for (i = 0; i < reg_15fps_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_15fps_table[i]))
            goto fps_fail;
        }            
        break;

		
      case 10:
        for (i = 0; i < reg_10fps_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_10fps_table[i]))
            goto fps_fail;
        }            
        break;
           
      case 7:
        for (i = 0; i < reg_7fps_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_7fps_table[i]))
            goto fps_fail;
        }            
        break;   
            
      default:
        printk(S5KA3DFX_MOD_NAME "[fps]Invalid value is ordered!!!\n");
        goto fps_fail;
    }
  }    

  return 0;

fps_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_fps is failed!!!\n");
  return -EINVAL;   
}

static int s5ka3dfx_set_ev(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;
  
  int i,j;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_ev is called... value = %d\n", value);
  
  i = 0;
  j = value;

  if(s5ka3dfx_set_data(client, reg_ev_table[i]))
    goto ev_fail;
  if(s5ka3dfx_set_data(client, reg_ev_table[2*j-1]))
    goto ev_fail;
  if(s5ka3dfx_set_data(client, reg_ev_table[2*j]))
    goto ev_fail;

  sensor->ev = value;
  
  return 0;
  
ev_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_ev is failed!!!\n");
  return -EINVAL;   
}

static int s5ka3dfx_get_rev(void)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;

  u8 vendor_id = 0xff, sw_ver = 0xff;
  int ret0 = 0, ret1 = 0, ret2 = 0;

  u8 data[2] = {0xEF, 0x01};
  u8 vender[1] = {0xC5};
  u8 version[1] = {0xC6};

  
  printk("----------------------------------------------------\n");
  printk("   [VGA CAM]   camsensor_s5ka3dfx_check_sensor_rev\n");
  printk("----------------------------------------------------\n");

  /*===========================================================================
  * there's no way to decide which one to use before sensor revision check,
  * so we use reset-default page number (0x00) without specifying explicitly
  ===========================================================================*/

  ret0 = s5ka3dfx_i2c_write(client, sizeof(data), data);
  ret1 = s5ka3dfx_i2c_write_read(client, 1, vender, 1, &vendor_id);
  ret2 = s5ka3dfx_i2c_write_read(client, 1, version, 1, &sw_ver);

  if (!(ret0 & ret1 & ret2))
  {
      printk(S5KA3DFX_MOD_NAME"=================================\n");
      printk(S5KA3DFX_MOD_NAME"   [VGA CAM] vendor_id ID : 0x%x\n", vendor_id);
      printk(S5KA3DFX_MOD_NAME"   [VGA CAM] software version : 0x%x\n", sw_ver);        
      printk(S5KA3DFX_MOD_NAME"=================================\n");            

      if(vendor_id == 0xAB && sw_ver == 0x03)
      {
          printk(S5KA3DFX_MOD_NAME"===============================\n");
          printk(S5KA3DFX_MOD_NAME"   [VGA CAM] s5ka3dfx  OK!!\n");
          printk(S5KA3DFX_MOD_NAME"===============================\n");
      }
      else
      {
          printk(S5KA3DFX_MOD_NAME"==========================================\n");
          printk(S5KA3DFX_MOD_NAME"   [VGA CAM] camsemsor operation fail!!\n");
          printk(S5KA3DFX_MOD_NAME"===========================================\n");
          return -EINVAL;
      }
  }
  else
  {
      printk(S5KA3DFX_MOD_NAME"-------------------------------------------------\n");
      printk(S5KA3DFX_MOD_NAME"   [VGA CAM] sensor reset failure detected!!\n");
      printk(S5KA3DFX_MOD_NAME"-------------------------------------------------\n");
      goto get_rev_fail;
  }
 
  return 0;

get_rev_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_get_rev is failed!!!\n");
  return -EINVAL;   
}

static int s5ka3dfx_set_table (void)
{
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_table is called...\n");

    reg_init_qcif_index = 0;
    reg_init_cif_index = 0;
    reg_init_qvga_index = 0;
    reg_init_vga_index = 0;
    reg_init_qcif_vt_index = 0;
    reg_init_cif_vt_index = 0;
    reg_init_qvga_vt_index = 0;
    reg_init_vga_vt_index = 0;
    reg_wb_auto_index = 0;
    reg_wb_daylight_index = 0;
    reg_wb_cloudy_index = 0;
    reg_wb_incandescent_index = 0;
    reg_wb_fluorescent_index = 0;
    reg_ev_index = 0;
    reg_ev_vt_index = 0;
	reg_contrast_level_m5_index = 0;
	reg_contrast_level_m4_index = 0;
	reg_contrast_level_m3_index = 0;
	reg_contrast_level_m2_index = 0;
	reg_contrast_level_m1_index = 0;
	reg_contrast_level_default_index = 0;
	reg_contrast_level_p1_index = 0;
	reg_contrast_level_p2_index = 0;
	reg_contrast_level_p3_index = 0;
	reg_contrast_level_p4_index = 0;
	reg_contrast_level_p5_index = 0;
    reg_effect_none_index = 0;
    reg_effect_red_index = 0;
    reg_effect_gray_index = 0;
    reg_effect_sepia_index = 0;
    reg_effect_green_index = 0;
    reg_effect_aqua_index = 0;
    reg_effect_negative_index = 0;
    reg_flip_none_index = 0;
    reg_flip_water_index = 0;
    reg_flip_mirror_index = 0;
    reg_flip_water_mirror_index = 0;
    reg_pretty_none_index = 0;
    reg_pretty_level1_index = 0;
    reg_pretty_level2_index = 0;
    reg_pretty_level3_index = 0;
    reg_pretty_vt_none_index = 0;
    reg_pretty_vt_level1_index = 0;
    reg_pretty_vt_level2_index = 0;
    reg_pretty_vt_level3_index = 0;
    reg_7fps_index = 0;
    reg_10fps_index = 0;
    reg_15fps_index = 0;    
	reg_self_capture_index = 0;

#if !(IS_USE_REGISTER_CONFIGURE_FILE_LSI)

    /* Section Index */
    reg_init_qcif_index = sizeof(reg_init_qcif_table)/sizeof(u32);
    reg_init_cif_index = sizeof(reg_init_cif_table)/sizeof(u32);
    reg_init_qvga_index = sizeof(reg_init_qvga_table)/sizeof(u32);
    reg_init_vga_index = sizeof(reg_init_vga_table)/sizeof(u32);
    reg_init_qcif_vt_index = sizeof(reg_init_qcif_vt_table)/sizeof(u32);
    reg_init_cif_vt_index = sizeof(reg_init_cif_vt_table)/sizeof(u32);
    reg_init_qvga_vt_index = sizeof(reg_init_qvga_vt_table)/sizeof(u32);
    reg_init_vga_vt_index = sizeof(reg_init_vga_vt_table)/sizeof(u32);
    reg_ev_index = sizeof(reg_ev_table)/sizeof(u32);
    reg_ev_vt_index = sizeof(reg_ev_vt_table)/sizeof(u32);    
	reg_contrast_level_m5_index = sizeof(reg_contrast_level_m5_table)/sizeof(u32);
	reg_contrast_level_m4_index = sizeof(reg_contrast_level_m4_table)/sizeof(u32);
	reg_contrast_level_m3_index = sizeof(reg_contrast_level_m3_table)/sizeof(u32);
	reg_contrast_level_m2_index = sizeof(reg_contrast_level_m2_table)/sizeof(u32);
	reg_contrast_level_m1_index = sizeof(reg_contrast_level_m1_table)/sizeof(u32);
	reg_contrast_level_default_index = sizeof(reg_contrast_level_default_table)/sizeof(u32);
	reg_contrast_level_p1_index = sizeof(reg_contrast_level_p1_table)/sizeof(u32);
	reg_contrast_level_p2_index = sizeof(reg_contrast_level_p2_table)/sizeof(u32);
	reg_contrast_level_p3_index = sizeof(reg_contrast_level_p3_table)/sizeof(u32);
	reg_contrast_level_p4_index = sizeof(reg_contrast_level_p4_table)/sizeof(u32);
	reg_contrast_level_p5_index = sizeof(reg_contrast_level_p5_table)/sizeof(u32);
    reg_wb_auto_index = sizeof(reg_wb_auto_table)/sizeof(u32);
    reg_wb_daylight_index = sizeof(reg_wb_daylight_table)/sizeof(u32);
    reg_wb_cloudy_index = sizeof(reg_wb_cloudy_table)/sizeof(u32);
    reg_wb_incandescent_index = sizeof(reg_wb_incandescent_table)/sizeof(u32);
    reg_wb_fluorescent_index = sizeof(reg_wb_fluorescent_table)/sizeof(u32);
    reg_effect_none_index = sizeof(reg_effect_none_table)/sizeof(u32);
    reg_effect_gray_index = sizeof(reg_effect_gray_table)/sizeof(u32);
    reg_effect_red_index = sizeof(reg_effect_red_table)/sizeof(u32);    
    reg_effect_sepia_index = sizeof(reg_effect_sepia_table)/sizeof(u32);
    reg_effect_green_index = sizeof(reg_effect_green_table)/sizeof(u32);
    reg_effect_aqua_index = sizeof(reg_effect_aqua_table)/sizeof(u32);
    reg_effect_negative_index = sizeof(reg_effect_negative_table)/sizeof(u32);
    reg_flip_none_index = sizeof(reg_flip_none_table)/sizeof(u32);
    reg_flip_water_index = sizeof(reg_flip_water_table)/sizeof(u32);
    reg_flip_mirror_index = sizeof(reg_flip_mirror_table)/sizeof(u32);
    reg_flip_water_mirror_index = sizeof(reg_flip_water_mirror_table)/sizeof(u32);
    reg_pretty_none_index = sizeof(reg_pretty_none_table)/sizeof(u32);
    reg_pretty_level1_index = sizeof(reg_pretty_level1_table)/sizeof(u32);
    reg_pretty_level2_index = sizeof(reg_pretty_level2_table)/sizeof(u32);
    reg_pretty_level3_index = sizeof(reg_pretty_level3_table)/sizeof(u32);   
    reg_pretty_vt_none_index = sizeof(reg_pretty_vt_none_table)/sizeof(u32);
    reg_pretty_vt_level1_index = sizeof(reg_pretty_vt_level1_table)/sizeof(u32);
    reg_pretty_vt_level2_index = sizeof(reg_pretty_vt_level2_table)/sizeof(u32);
    reg_pretty_vt_level3_index = sizeof(reg_pretty_vt_level3_table)/sizeof(u32);
    reg_7fps_index = sizeof(reg_7fps_table)/sizeof(u32);
    reg_10fps_index = sizeof(reg_10fps_table)/sizeof(u32);
    reg_15fps_index = sizeof(reg_15fps_table)/sizeof(u32);    
	reg_self_capture_index = sizeof(reg_self_capture_table)/sizeof(u32);  

#else

    memset(&reg_init_qcif_table, 0, sizeof(reg_init_qcif_table));
    memset(&reg_init_cif_table, 0, sizeof(reg_init_cif_table));
    memset(&reg_init_qvga_table, 0, sizeof(reg_init_qvga_table));
    memset(&reg_init_vga_table, 0, sizeof(reg_init_vga_table));
    memset(&reg_init_qcif_vt_table, 0, sizeof(reg_init_qcif_vt_table));
    memset(&reg_init_cif_vt_table, 0, sizeof(reg_init_cif_vt_table));
    memset(&reg_init_qvga_vt_table, 0, sizeof(reg_init_qvga_vt_table));
    memset(&reg_init_vga_vt_table, 0, sizeof(reg_init_vga_vt_table));
    memset(&reg_wb_auto_table, 0, sizeof(reg_wb_auto_table));
    memset(&reg_wb_daylight_table, 0, sizeof(reg_wb_daylight_table));
    memset(&reg_wb_cloudy_table, 0, sizeof(reg_wb_cloudy_table));
    memset(&reg_wb_incandescent_table, 0, sizeof(reg_wb_incandescent_table));
    memset(&reg_wb_fluorescent_table, 0, sizeof(reg_wb_fluorescent_table));
    memset(&reg_ev_table, 0, sizeof(reg_ev_table));
    memset(&reg_ev_vt_table, 0, sizeof(reg_ev_vt_table));
	memset(&reg_contrast_level_m5_table, 0, sizeof(reg_contrast_level_m5_table));
	memset(&reg_contrast_level_m4_table, 0, sizeof(reg_contrast_level_m4_table));
	memset(&reg_contrast_level_m3_table, 0, sizeof(reg_contrast_level_m3_table));
	memset(&reg_contrast_level_m2_table, 0, sizeof(reg_contrast_level_m2_table));
	memset(&reg_contrast_level_m1_table, 0, sizeof(reg_contrast_level_m1_table));
	memset(&reg_contrast_level_default_table, 0, sizeof(reg_contrast_level_default_table));
	memset(&reg_contrast_level_p1_table, 0, sizeof(reg_contrast_level_p1_table));
	memset(&reg_contrast_level_p2_table, 0, sizeof(reg_contrast_level_p2_table));
	memset(&reg_contrast_level_p3_table, 0, sizeof(reg_contrast_level_p3_table));
	memset(&reg_contrast_level_p4_table, 0, sizeof(reg_contrast_level_p4_table));
	memset(&reg_contrast_level_p5_table, 0, sizeof(reg_contrast_level_p5_table));
    memset(&reg_effect_none_table, 0, sizeof(reg_effect_none_table));
    memset(&reg_effect_gray_table, 0, sizeof(reg_effect_gray_table));
    memset(&reg_effect_red_table, 0, sizeof(reg_effect_red_table));    
    memset(&reg_effect_sepia_table, 0, sizeof(reg_effect_sepia_table));
    memset(&reg_effect_green_table, 0, sizeof(reg_effect_green_table));
    memset(&reg_effect_aqua_table, 0, sizeof(reg_effect_aqua_table));
    memset(&reg_effect_negative_table, 0, sizeof(reg_effect_negative_table));
    memset(&reg_flip_none_table, 0, sizeof(reg_flip_none_table));
    memset(&reg_flip_water_table, 0, sizeof(reg_flip_water_table));
    memset(&reg_flip_mirror_table, 0, sizeof(reg_flip_mirror_table));
    memset(&reg_flip_water_mirror_table, 0, sizeof(reg_flip_water_mirror_table));
    memset(&reg_pretty_none_table, 0, sizeof(reg_pretty_none_table));
    memset(&reg_pretty_level1_table, 0, sizeof(reg_pretty_level1_table));
    memset(&reg_pretty_level2_table, 0, sizeof(reg_pretty_level2_table));
    memset(&reg_pretty_level3_table, 0, sizeof(reg_pretty_level3_table));
    memset(&reg_pretty_vt_none_table, 0, sizeof(reg_pretty_vt_none_table));
    memset(&reg_pretty_vt_level1_table, 0, sizeof(reg_pretty_vt_level1_table));
    memset(&reg_pretty_vt_level2_table, 0, sizeof(reg_pretty_vt_level2_table));
    memset(&reg_pretty_vt_level3_table, 0, sizeof(reg_pretty_vt_level3_table));  
    memset(&reg_7fps_table, 0, sizeof(reg_7fps_table));  
    memset(&reg_10fps_table, 0, sizeof(reg_10fps_table)); 
    memset(&reg_15fps_table, 0, sizeof(reg_15fps_table)); 
	memset(&reg_self_capture_table, 0, sizeof(reg_self_capture_table)); 
    
#endif    

  return 0;
}

static int s5ka3dfx_set_init(void)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;

  int i = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_init is called...\n");
  
  s5ka3dfx_set_table();

#if (IS_USE_REGISTER_CONFIGURE_FILE_LSI)
  s5ka3dfx_make_table();
#endif

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_init vga mode = %d\n", sensor->mode);
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_init preview size = %d\n", sensor->preview_size);
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "sensor->check_dataline : %d \n", sensor->check_dataline);

  if(sensor->mode == S5KA3DFX_MODE_VT)
  {
    switch(sensor->preview_size)
    {
      case S5KA3DFX_PREVIEW_SIZE_640_480:
        for (i = 0; i < reg_init_vga_vt_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_vga_vt_table[i]))
            goto init_fail;
        }
        break;

      case S5KA3DFX_PREVIEW_SIZE_320_240:
        for (i = 0; i < reg_init_qvga_vt_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_qvga_vt_table[i]))
            goto init_fail;
        }
        break;

      case S5KA3DFX_PREVIEW_SIZE_352_288:
        for (i = 0; i < reg_init_cif_vt_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_cif_vt_table[i]))
            goto init_fail;
        }
        break;

      case S5KA3DFX_PREVIEW_SIZE_176_144:
        for (i = 0; i < reg_init_qcif_vt_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_qcif_vt_table[i]))
            goto init_fail;
        }
        break;        

      default:
        printk(S5KA3DFX_MOD_NAME "[size]Invalid value is ordered!!!\n");
        goto init_fail;
    }
  }
  else
  {
	if(sensor->check_dataline)
	{
		dprintk(CAM_INF, S5KA3DFX_MOD_NAME "dataline set\n");
		for (i = 0; i < sizeof(s5ka3dfx_dataline)/sizeof(u32); i++) {
			if(s5ka3dfx_set_data(client, s5ka3dfx_dataline[i]))
				goto init_fail;
		}	
		sensor->check_dataline = 0;
		return 0;
	}
	
    switch(sensor->preview_size)
    {
      case S5KA3DFX_PREVIEW_SIZE_640_480:
        for (i = 0; i < reg_init_vga_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_vga_table[i]))
            goto init_fail;
        }
        break;

      case S5KA3DFX_PREVIEW_SIZE_320_240:
        for (i = 0; i < reg_init_qvga_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_qvga_table[i]))
            goto init_fail;
        }
        break;

      case S5KA3DFX_PREVIEW_SIZE_352_288:
        for (i = 0; i < reg_init_cif_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_cif_table[i]))
            goto init_fail;
        }
        break;

      case S5KA3DFX_PREVIEW_SIZE_176_144:
        for (i = 0; i < reg_init_qcif_index; i++) 
        {
          if(s5ka3dfx_set_data(client, reg_init_qcif_table[i]))
            goto init_fail;
        }
        break;        

      default:
        printk(S5KA3DFX_MOD_NAME "[size]Invalid value is ordered!!!\n");
        goto init_fail;
    }    
  }

  s5ka3dfx_set_fps();
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_ev = %d\n", sensor->ev);  
  s5ka3dfx_set_ev(sensor->ev);  

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_init success...\n");

  return 0;

init_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_init is failed!!!\n");
  return -EINVAL;     
}

static int s5ka3dfx_set_mode(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_mode is called... mode = %d\n", value);  
  
  sensor->mode = value;
  
  return 0;
}

static int s5ka3dfx_set_state(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_state is called... state = %d\n", value);  
  
  sensor->state = value;
  
  return 0;
}

static int s5ka3dfx_set_zoom(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;

  sensor->zoom = value;

  return 0;
}

static int s5ka3dfx_set_effect(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;

  int i;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_effect is called... effect = %d\n", value);
  
  switch(value)
  {
    case S5KA3DFX_EFFECT_OFF:
      for (i = 0; i < reg_effect_none_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_none_table[i]))
          goto effect_fail;
      }
      break;

    case S5KA3DFX_EFFECT_BW:
      break;    

    case S5KA3DFX_EFFECT_GREY:
      for (i = 0; i < reg_effect_gray_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_gray_table[i]))
          goto effect_fail;
      }      
      break;      
      
    case S5KA3DFX_EFFECT_SEPIA:
      for (i = 0; i < reg_effect_sepia_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_sepia_table[i]))
          goto effect_fail;
      }        
      break;

    case S5KA3DFX_EFFECT_SHARPEN:
      break;      
      
    case S5KA3DFX_EFFECT_NEGATIVE:
      for (i = 0; i < reg_effect_negative_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_negative_table[i]))
          goto effect_fail;
      }      
      break;
      
    case S5KA3DFX_EFFECT_ANTIQUE:
      break;
      
    case S5KA3DFX_EFFECT_AQUA:
      for (i = 0; i < reg_effect_aqua_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_aqua_table[i]))
          goto effect_fail;
      }      
      break;

    case S5KA3DFX_EFFECT_RED:
      for (i = 0; i < reg_effect_red_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_red_table[i]))
          goto effect_fail;
      }      
      break; 

    case S5KA3DFX_EFFECT_PINK:
      break; 

    case S5KA3DFX_EFFECT_YELLOW:
      break;       

    case S5KA3DFX_EFFECT_GREEN:
      for (i = 0; i < reg_effect_green_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_effect_green_table[i]))
          goto effect_fail;
      }      
      break; 

    case S5KA3DFX_EFFECT_BLUE:
      break; 

    case S5KA3DFX_EFFECT_PURPLE:
      break;       
      
    default:
      printk(S5KA3DFX_MOD_NAME "[Effect]Invalid value is ordered!!!\n");
      goto effect_fail;
  }

  sensor->effect = value;
  
  return 0;

effect_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_effect is failed!!!\n");
  return -EINVAL;       
}

static int s5ka3dfx_set_contrast(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;
  
  int i;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_contrast is called... value = %d\n", value);

  sensor->contrast = value;
  switch(sensor->contrast)
  {
    case S5KA3DFX_CONTRAST_MINUS_5:
      for (i = 0; i < reg_contrast_level_m5_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_m5_table[i]);
      }       
      break;
      
    case S5KA3DFX_CONTRAST_MINUS_4:
      for (i = 0; i < reg_contrast_level_m4_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_m4_table[i]);
      }       
      break;
      
    case S5KA3DFX_CONTRAST_MINUS_3:
      for (i = 0; i < reg_contrast_level_m3_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_m3_table[i]);
      }       
      break;
      
    case S5KA3DFX_CONTRAST_MINUS_2:
      for (i = 0; i < reg_contrast_level_m2_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_m2_table[i]);
      }       
      break;
      
    case S5KA3DFX_CONTRAST_MINUS_1:
      for (i = 0; i < reg_contrast_level_m1_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_m1_table[i]);
      }       
      break;

    case S5KA3DFX_CONTRAST_DEFAULT:
      for (i = 0; i < reg_contrast_level_default_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_default_table[i]);
      }       
      break;

    case S5KA3DFX_CONTRAST_PLUS_1:
      for (i = 0; i < reg_contrast_level_p1_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_p1_table[i]);
      }       
      break;
      
    case S5KA3DFX_CONTRAST_PLUS_2:
      for (i = 0; i < reg_contrast_level_p2_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_p2_table[i]);
      }       
      break;

    case S5KA3DFX_CONTRAST_PLUS_3:
      for (i = 0; i < reg_contrast_level_p3_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_p3_table[i]);
      }       
      break;

    case S5KA3DFX_CONTRAST_PLUS_4:
      for (i = 0; i < reg_contrast_level_p4_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_p4_table[i]);
      }       
      break;

    case S5KA3DFX_CONTRAST_PLUS_5:
      for (i = 0; i < reg_contrast_level_p5_index; i++) 
      {
        s5ka3dfx_set_data(client, reg_contrast_level_p5_table[i]);
      }       
      break;

    default:
      printk(S5KA3DFX_MOD_NAME "[WB]Invalid value is ordered!!!\n");
      return -EINVAL;
  }

  return 0;
}


static int s5ka3dfx_set_wb(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;

  int i;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_wb is called... value = %d\n",value);
  
  switch(value)
  {
    case S5KA3DFX_WB_AUTO:
      for (i = 0; i < reg_wb_auto_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_wb_auto_table[i]))
          goto wb_fail;
      }       
      break;
      
    case S5KA3DFX_WB_DAYLIGHT:
      for (i = 0; i < reg_wb_daylight_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_wb_daylight_table[i]))
          goto wb_fail;
      }       
      break;
      
    case S5KA3DFX_WB_INCANDESCENT:
      for (i = 0; i < reg_wb_incandescent_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_wb_incandescent_table[i]))
          goto wb_fail;
      }       
      break;
      
    case S5KA3DFX_WB_FLUORESCENT:
      for (i = 0; i < reg_wb_fluorescent_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_wb_fluorescent_table[i]))
          goto wb_fail;
      }       
      break;
      
    case S5KA3DFX_WB_CLOUDY:
      for (i = 0; i < reg_wb_cloudy_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_wb_cloudy_table[i]))
          goto wb_fail;
      }       
      break;
      
    default:
      printk(S5KA3DFX_MOD_NAME "[WB]Invalid value is ordered!!!\n");
      goto wb_fail;
  }

  sensor->wb = value;

  return 0;

wb_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_wb is failed!!!\n");
  return -EINVAL;   
}

static int s5ka3dfx_set_pretty(s32 value)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  struct i2c_client *client = sensor->i2c_client;

  int i;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_set_pretty is called... value = %d\n", value);

  switch(value)
  {
    case S5KA3DFX_PRETTY_NONE:
      for (i = 0; i < reg_pretty_none_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_pretty_none_table[i]))
          goto pretty_fail;
      }       
      break;
      
    case S5KA3DFX_PRETTY_LEVEL1:
      for (i = 0; i < reg_pretty_level1_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_pretty_level1_table[i]))
          goto pretty_fail;
      }       
      break;
      
    case S5KA3DFX_PRETTY_LEVEL2:
      for (i = 0; i < reg_pretty_level2_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_pretty_level2_table[i]))
          goto pretty_fail;
      }       
      break;
      
    case S5KA3DFX_PRETTY_LEVEL3:
      for (i = 0; i < reg_pretty_level3_index; i++) 
      {
        if(s5ka3dfx_set_data(client, reg_pretty_level3_table[i]))
          goto pretty_fail;
      }       
      break;
      
    default:
      printk(S5KA3DFX_MOD_NAME "[Pretty]Invalid value is ordered!!!\n");
      goto pretty_fail;
  }

  sensor->pretty = value;

  return 0;

pretty_fail:
  printk(S5KA3DFX_MOD_NAME "s5ka3dfx_set_pretty is failed!!!\n");
  return -EINVAL;     
}

#if (IS_USE_REGISTER_CONFIGURE_FILE_LSI)

int parsing_section;

static u32 s5ka3dfx_util_hex_val(char hex)
{
  if ( hex >= 'a' && hex <= 'f' )
  {
    return (hex-'a'+10);
  }
  else if ( hex >= 'A' && hex <= 'F' )
  {
    return (hex - 'A' + 10 );
  }
  else if ( hex >= '0' && hex <= '9' )
  {
    return (hex - '0');
  }
  else
  {
    return 0;
  }
}

static u32 s5ka3dfx_util_gets(char* buffer, char* line, int is_start)
{
  int          i;
  char*        _r_n_ptr;
  static char* buffer_ptr;

  memset(line, 0, 1024);

  if ( is_start )
      buffer_ptr = buffer;

  _r_n_ptr = strstr(buffer_ptr, "\r\n");

  //\n  
  if ( _r_n_ptr )
  {
    for ( i = 0 ; ; i++ )
    {
      if ( buffer_ptr+i == _r_n_ptr )
      {
        buffer_ptr = _r_n_ptr+1;
        break;
      }
      line[i] = buffer_ptr[i];
    }
    line[i] = '\0';

    return 1;
  }
  //\n  
  else
  {
    if ( strlen(buffer_ptr) > 0 )
    {
      strcpy(line, buffer_ptr);
      return 0;
    }
    else
    {
      return 0;
    }
  }
}

static u32 s5ka3dfx_util_atoi(char* str)
{
  unsigned int i,j=0;
  unsigned int val_len;
  unsigned int ret_val=0;

  if (str == NULL)
      return 0;

  //decimal
  if(strlen(str) <= 4 || (strstr(str, "0x")==NULL && strstr(str, "0X")==NULL ))
  {
    for( ; ; str++ ) {
      switch( *str ) {
        case '0'...'9':
          ret_val= 10 * ret_val + ( *str - '0' ) ;
          break ;
        default:
          break ;
      }
    }

    return ret_val;
  }

  //hex ex:0xa0c
  val_len = strlen(str);

  for (i = val_len-1 ; i >= 2 ; i--)
  {
    ret_val = ret_val + (s5ka3dfx_util_hex_val(str[i])<<(j*4));
    j++;
  }

  return ret_val;
}

static int s5ka3dfx_util_trim(char* buff)
{
  int         left_index;
  int         right_index;
  int         buff_len;
  int         i;

  buff_len	= strlen(buff);
  left_index	= -1;
  right_index = -1;

  if ( buff_len == 0 )
  {
    return 0;
  }

  /* left index(  white space  ) */
  for ( i = 0 ; i < buff_len ; i++ )
  {
    if ( buff[i] != ' ' && buff[i] != '\t' && buff[i] != '\n' && buff[i] != '\r')
    {
      left_index = i;
      break;
    }
  }

  /* right index(  white space  ) */
  for ( i = buff_len-1 ; i >= 0 ; i-- )
  {
    if ( buff[i] != ' ' && buff[i] != '\t' && buff[i] != '\n' && buff[i] != '\r')
    {
      right_index = i;
      buff[i+1] = '\0';
      break;
    }
  }

  if ( left_index == -1 && right_index == -1 )
  {
    strcpy(buff, "");
  }
  else if ( left_index <= right_index )
  {
    strcpy(buff, buff+left_index);
  }
  else
  {
    return -EINVAL;
  }

  return 0;
}


static u32 s5ka3dfx_insert_register_table(char* line)
{
  int   i;
  char  reg_val_str[7];
  int   reg_val_str_idx=0;

  unsigned int  reg_val;

  s5ka3dfx_util_trim(line);
  
  if ( strlen(line) == 0 || (line[0] == '/' && line[1] == '/' ) || (line[0] == '/' && line[1] == '*' ) || line[0] == '{' || line[0] == '}' )
  {
    return 0;
  }

  for (i = 0 ; ; i++)
  {
    if ( line[i] == ' ' || line[i] == '\t' || line[i] == '/' || line[i] == '\0')
      continue;

    if ( line[i] == ',' )
      break;

      reg_val_str[reg_val_str_idx++] = line[i];
  }

  reg_val_str[reg_val_str_idx] = '\0';

  reg_val = s5ka3dfx_util_atoi(reg_val_str);

  if      ( parsing_section == REG_INIT_QCIF_SECTION)         reg_init_cif_table[reg_init_qcif_index++] = reg_val;
  else if ( parsing_section == REG_INIT_CIF_SECTION)          reg_init_qcif_table[reg_init_cif_index++] = reg_val;
  else if ( parsing_section == REG_INIT_QVGA_SECTION)         reg_init_qvga_table[reg_init_qvga_index++] = reg_val;
  else if ( parsing_section == REG_INIT_VGA_SECTION)          reg_init_vga_table[reg_init_vga_index++] = reg_val;
  else if ( parsing_section == REG_INIT_QCIF_VT_SECTION)      reg_init_qcif_vt_table[reg_init_qcif_vt_index++] = reg_val;
  else if ( parsing_section == REG_INIT_CIF_VT_SECTION)       reg_init_cif_vt_table[reg_init_cif_vt_index++] = reg_val;
  else if ( parsing_section == REG_INIT_QVGA_VT_SECTION)      reg_init_qvga_vt_table[reg_init_qvga_vt_index++] = reg_val;
  else if ( parsing_section == REG_INIT_VGA_VT_SECTION)       reg_init_vga_vt_table[reg_init_vga_vt_index++] = reg_val;
  else if ( parsing_section == REG_WB_AUTO_SECTION)           reg_wb_auto_table[reg_wb_auto_index++] = reg_val;
  else if ( parsing_section == REG_WB_DAYLIGHT_SECTION)       reg_wb_daylight_table[reg_wb_daylight_index++] = reg_val;
  else if ( parsing_section == REG_WB_CLOUDY_SECTION)         reg_wb_cloudy_table[reg_wb_cloudy_index++] = reg_val;
  else if ( parsing_section == REG_WB_INCANDESCENT_SECTION)   reg_wb_incandescent_table[reg_wb_incandescent_index++] = reg_val;
  else if ( parsing_section == REG_WB_FLUORESCENT_SECTION)    reg_wb_fluorescent_table[reg_wb_fluorescent_index++] = reg_val;
  else if ( parsing_section == REG_EV_SECTION)                reg_ev_table[reg_ev_index++] = reg_val;
  else if ( parsing_section == REG_EV_VT_SECTION)             reg_ev_vt_table[reg_ev_vt_index++]	= reg_val;    
  else if ( parsing_section == REG_CONTRAST_LEVEL_M5_SECTION) reg_contrast_level_m5_table[reg_contrast_level_m5_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_M4_SECTION) reg_contrast_level_m4_table[reg_contrast_level_m4_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_M3_SECTION) reg_contrast_level_m3_table[reg_contrast_level_m3_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_M2_SECTION) reg_contrast_level_m2_table[reg_contrast_level_m2_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_M1_SECTION) reg_contrast_level_m1_table[reg_contrast_level_m1_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_DEFAULT_SECTION) reg_contrast_level_default_table[reg_contrast_level_default_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_P1_SECTION) reg_contrast_level_p1_table[reg_contrast_level_p1_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_P2_SECTION) reg_contrast_level_p2_table[reg_contrast_level_p2_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_P3_SECTION) reg_contrast_level_p3_table[reg_contrast_level_p3_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_P4_SECTION) reg_contrast_level_p4_table[reg_contrast_level_p4_index++] = reg_val;
  else if ( parsing_section == REG_CONTRAST_LEVEL_P5_SECTION) reg_contrast_level_p5_table[reg_contrast_level_p5_index++] = reg_val;
  else if ( parsing_section == REG_EFFECT_NONE_SECTION)       reg_effect_none_table[reg_effect_none_index++] = reg_val;
  else if ( parsing_section == REG_EFFECT_GRAY_SECTION)       reg_effect_gray_table[reg_effect_gray_index++] = reg_val;
  else if ( parsing_section == REG_EFFECT_RED_SECTION)        reg_effect_red_table[reg_effect_red_index++] = reg_val;    
  else if ( parsing_section == REG_EFFECT_SEPIA_SECTION)      reg_effect_sepia_table[reg_effect_sepia_index++] = reg_val;
  else if ( parsing_section == REG_EFFECT_GREEN_SECTION)      reg_effect_green_table[reg_effect_green_index++] = reg_val;
  else if ( parsing_section == REG_EFFECT_AQUA_SECTION)       reg_effect_aqua_table[reg_effect_aqua_index++] = reg_val;
  else if ( parsing_section == REG_EFFECT_NEGATIVE_SECTION)   reg_effect_negative_table[reg_effect_negative_index++] = reg_val;
  else if ( parsing_section == REG_FLIP_NONE_SECTION)         reg_flip_none_table[reg_flip_none_index++] = reg_val;
  else if ( parsing_section == REG_FLIP_WATER_SECTION)        reg_flip_water_table[reg_flip_water_index++] = reg_val;
  else if ( parsing_section == REG_FLIP_MIRROR_SECTION)       reg_flip_mirror_table[reg_flip_mirror_index++] = reg_val;
  else if ( parsing_section == REG_FLIP_WATER_MIRROR_SECTION) reg_flip_water_mirror_table[reg_flip_water_mirror_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_NONE_SECTION)       reg_pretty_none_table[reg_pretty_none_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_LEVEL1_SECTION)     reg_pretty_level1_table[reg_pretty_level1_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_LEVEL2_SECTION)     reg_pretty_level2_table[reg_pretty_level2_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_LEVEL3_SECTION)     reg_pretty_level3_table[reg_pretty_level3_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_VT_NONE_SECTION)    reg_pretty_vt_none_table[reg_pretty_vt_none_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_VT_LEVEL1_SECTION)  reg_pretty_vt_level1_table[reg_pretty_vt_level1_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_VT_LEVEL2_SECTION)  reg_pretty_vt_level2_table[reg_pretty_vt_level2_index++] = reg_val;
  else if ( parsing_section == REG_PRETTY_VT_LEVEL3_SECTION)  reg_pretty_vt_level3_table[reg_pretty_vt_level3_index++] = reg_val;
  else if ( parsing_section == REG_7FPS_SECTION)			  reg_7fps_table[reg_7fps_index++] = reg_val;  
  else if ( parsing_section == REG_10FPS_SECTION)			  reg_10fps_table[reg_10fps_index++] = reg_val;  
  else if ( parsing_section == REG_15FPS_SECTION)			  reg_15fps_table[reg_15fps_index++] = reg_val;    
  else if ( parsing_section == REG_SELF_CAPTURE_SECTION)	  reg_self_capture_table[reg_self_capture_index++] = reg_val;    

  return 0;
}

static u32 s5ka3dfx_parsing_section(char* line)
{
  if ( strstr(line, CAMIF_SET_SENSOR_QCIF_INIT) != NULL )
    parsing_section = REG_INIT_QCIF_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_CIF_INIT) != NULL )
    parsing_section = REG_INIT_CIF_SECTION;  
  else if ( strstr(line, CAMIF_SET_SENSOR_QVGA_INIT) != NULL )
    parsing_section = REG_INIT_QVGA_SECTION;  
  else if ( strstr(line, CAMIF_SET_SENSOR_VGA_INIT) != NULL )
    parsing_section = REG_INIT_VGA_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_QCIF_VT_INIT) != NULL )
    parsing_section = REG_INIT_QCIF_VT_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_CIF_VT_INIT) != NULL )
    parsing_section = REG_INIT_CIF_VT_SECTION;  
  else if ( strstr(line, CAMIF_SET_SENSOR_QVGA_VT_INIT) != NULL )
    parsing_section = REG_INIT_QVGA_VT_SECTION;  
  else if ( strstr(line, CAMIF_SET_SENSOR_VGA_VT_INIT) != NULL )
    parsing_section = REG_INIT_VGA_VT_SECTION;     
  else if ( strstr(line, CAMIF_SET_SENSOR_WB_AUTO) != NULL )
    parsing_section = REG_WB_AUTO_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_WB_DAYLIGHT ) != NULL )
    parsing_section = REG_WB_DAYLIGHT_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_WB_CLOUDY ) != NULL )
    parsing_section = REG_WB_CLOUDY_SECTION;
  else if( strstr(line, CAMIF_SET_SENSOR_WB_INCANDESCENT) != NULL)
    parsing_section = REG_WB_INCANDESCENT_SECTION;
  else if( strstr(line, CAMIF_SET_SENSOR_WB_FLUORESCENT) != NULL)
    parsing_section = REG_WB_FLUORESCENT_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EV) != NULL )
    parsing_section = REG_EV_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EV_VT) != NULL )
    parsing_section = REG_EV_VT_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_M5_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_M5_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_M4_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_M4_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_M3_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_M3_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_M2_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_M2_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_M1_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_M1_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_DEFAULT_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_DEFAULT_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_P1_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_P1_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_P2_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_P2_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_P3_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_P3_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_P4_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_P4_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_LEVEL_P5_CONTRAST) != NULL )
    parsing_section = REG_CONTRAST_LEVEL_P5_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_NONE) != NULL )
    parsing_section = REG_EFFECT_NONE_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_GRAY) != NULL )
    parsing_section = REG_EFFECT_GRAY_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_RED) != NULL )
    parsing_section = REG_EFFECT_RED_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_SEPIA) != NULL )
    parsing_section = REG_EFFECT_SEPIA_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_GREEN) != NULL )
    parsing_section = REG_EFFECT_GREEN_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_AQUA) != NULL )
    parsing_section = REG_EFFECT_AQUA_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_EFFECT_NEGATIVE) != NULL )
    parsing_section = REG_EFFECT_NEGATIVE_SECTION;
  else if ( strstr(line, CAMIF_SET_SENSOR_FLIP_NONE) != NULL )
    parsing_section = REG_FLIP_NONE_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_FLIP_WATER) != NULL )
    parsing_section = REG_FLIP_WATER_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_FLIP_MIRROR) != NULL )
    parsing_section = REG_FLIP_MIRROR_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_FLIP_WATER_MIRROR) != NULL )
    parsing_section = REG_FLIP_WATER_MIRROR_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_NONE) != NULL )
    parsing_section = REG_PRETTY_NONE_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_LEVEL1) != NULL )
    parsing_section = REG_PRETTY_LEVEL1_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_LEVEL2) != NULL )
    parsing_section = REG_PRETTY_LEVEL2_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_LEVEL3) != NULL )
    parsing_section = REG_PRETTY_LEVEL3_SECTION;        
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_VT_NONE) != NULL )
    parsing_section = REG_PRETTY_VT_NONE_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_VT_LEVEL1) != NULL )
    parsing_section = REG_PRETTY_VT_LEVEL1_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_VT_LEVEL2) != NULL )
    parsing_section = REG_PRETTY_VT_LEVEL2_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_PRETTY_VT_LEVEL3) != NULL )
    parsing_section = REG_PRETTY_VT_LEVEL3_SECTION;     
  else if ( strstr(line, CAMIF_SET_SENSOR_FPS7) != NULL )
    parsing_section = REG_7FPS_SECTION;    
  else if ( strstr(line, CAMIF_SET_SENSOR_FPS10) != NULL )
    parsing_section = REG_10FPS_SECTION;   
  else if ( strstr(line, CAMIF_SET_SENSOR_FPS15) != NULL )
    parsing_section = REG_15FPS_SECTION;   
  else if ( strstr(line, CAMIF_SET_SENSOR_SELF_CAPTURE) != NULL )
    parsing_section = REG_SELF_CAPTURE_SECTION;
  else
    return -EINVAL;

  return 0;
}

static int s5ka3dfx_make_table(void)
{
  char*        buffer = NULL;
  char         line[1024];
  unsigned int file_size = 0;

  struct file *filep = NULL;
  mm_segment_t oldfs;
  struct firmware *firmware;
  int ret = 0;  

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_make_table is called...\n");

  filep = filp_open(CAMIF_CONFIGURE_FILE_LSI, O_RDONLY, 0) ;

  if (filep && (filep!= 0xfffffffe))
  {
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    file_size = filep->f_op->llseek(filep, 0, SEEK_END);
    filep->f_op->llseek(filep, 0, SEEK_SET);
    buffer = (char*)kmalloc(file_size+1, GFP_KERNEL);
    filep->f_op->read(filep, buffer, file_size, &filep->f_pos);
    buffer[file_size] = '\0';
    filp_close(filep, current->files);
    set_fs(oldfs);
    printk(S5KA3DFX_MOD_NAME "File size : %d\n", file_size);
  }
  else
  {
    return -EINVAL;
  }

  // init table index
  parsing_section = 0;

  s5ka3dfx_util_gets(buffer, line, 1);
  if ( s5ka3dfx_parsing_section(line) )
  {
    s5ka3dfx_insert_register_table(line);
  }

  while(s5ka3dfx_util_gets(buffer, line, 0))
  {
    if ( s5ka3dfx_parsing_section(line) )
    {
      s5ka3dfx_insert_register_table(line);
    }
  }

  s5ka3dfx_insert_register_table(line);

  kfree(buffer);

  return 0;
}

#endif


static int ioctl_streamoff(struct v4l2_int_device *s)
{
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_streamoff is called...\n");

  return 0;
}

static int ioctl_streamon(struct v4l2_int_device *s)
{
  struct s5ka3dfx_sensor *sensor = s->priv;

  int err = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_streamon is called...(%x)\n", sensor->state);   

  if(sensor->state != S5KA3DFX_STATE_CAPTURE)
  {
    printk(S5KA3DFX_MOD_NAME "start preview....................\n");
    s5ka3dfx_pre_state = s5ka3dfx_curr_state;
    s5ka3dfx_curr_state = S5KA3DFX_STATE_PREVIEW;     
  }
  else
  {
    printk(S5KA3DFX_MOD_NAME "start capture....................\n");
    s5ka3dfx_pre_state = s5ka3dfx_curr_state;
    s5ka3dfx_curr_state = S5KA3DFX_STATE_CAPTURE;        
  }
  
  return err;
}


/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the s5ka3dfx_ctrl_list[] array.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
  int i;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_queryctrl is called...\n");

  for (i = 0; i < NUM_S5KA3DFX_CONTROL; i++) 
  {
    if (qc->id == s5ka3dfx_ctrl_list[i].id)
    {
      break;
    }
  }
  if (i == NUM_S5KA3DFX_CONTROL)
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "Control ID is not supported!!\n");
    qc->flags = V4L2_CTRL_FLAG_DISABLED;

    return -EINVAL;
  }

  *qc = s5ka3dfx_ctrl_list[i];

  return 0;
}

static int s5ka3dfx_check_dataline_stop()
{
	struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
	struct i2c_client *client = sensor->i2c_client;
	int err = -EINVAL, i;

	for (i = 0; i < 2; i++) {
		err = s5ka3dfx_i2c_write(client, sizeof(s5ka3dfx_dataline_stop[i]), s5ka3dfx_dataline_stop[i]);
		if (err < 0)
		{
			v4l_info(client, "%s: register set failed\n", __func__);
			return -EIO;
		}
	}
	sensor->check_dataline = 0;
	sensor->pdata->power_set(V4L2_POWER_OFF);
	mdelay(5);
	sensor->pdata->power_set(V4L2_POWER_ON);
	mdelay(5);
	s5ka3dfx_set_init();
	
	if (err < 0)
	{
		v4l_info(client, "%s: register set failed\n", __func__);
		return -EIO;
	}
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
  struct s5ka3dfx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_g_ctrl is called...(%d)\n", vc->id);

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
    case V4L2_CID_CONTRAST:
      vc->value = sensor->contrast;
	  break;
    case V4L2_CID_WB:
      vc->value = sensor->wb;
      break;      
    case V4L2_CID_EFFECT:
      vc->value = sensor->effect;
      break;
    case V4L2_CID_FLIP:
      vc->value = sensor->flip;
      break;
    case V4L2_CID_PRETTY:
      vc->value = sensor->pretty;
      break;    
    default:
       printk(S5KA3DFX_MOD_NAME "[id]Invalid value is ordered!!!\n");
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
 * value in HW (and updates the s5ka3dfx sensor struct).
 * Otherwise, * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;
  int retval = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_s_ctrl is called...(%d)\n", vc->id);

  switch (vc->id) 
  {
    case V4L2_CID_SELECT_MODE:
      retval = s5ka3dfx_set_mode(vc->value);
      break;  
    case V4L2_CID_SELECT_STATE:
      retval = s5ka3dfx_set_state(vc->value);
      break;  
    case V4L2_CID_ZOOM:
      retval = s5ka3dfx_set_zoom(vc->value);
      break;      
    case V4L2_CID_BRIGHTNESS:
      retval = s5ka3dfx_set_ev(vc->value);
      break;
    case V4L2_CID_CONTRAST:
      retval = s5ka3dfx_set_contrast(vc->value);
      break;
    case V4L2_CID_WB:
      retval = s5ka3dfx_set_wb(vc->value);
      break;
    case V4L2_CID_EFFECT:
      retval = s5ka3dfx_set_effect(vc->value);
      break;
    case V4L2_CID_FLIP:
      retval = s5ka3dfx_set_flip(vc->value);
      break;
    case V4L2_CID_PRETTY:
      retval = s5ka3dfx_set_pretty(vc->value);
      break;    
	case V4L2_CID_CAMERA_CHECK_DATALINE:
	  sensor->check_dataline = vc->value;
	  retval = 0;
	  break;	
	case V4L2_CID_CAMERA_CHECK_DATALINE_STOP:
	  retval = s5ka3dfx_check_dataline_stop();
	  break;	  
    default:
       printk(S5KA3DFX_MOD_NAME "[id]Invalid value is ordered!!!\n");
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

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_enum_fmt_cap is called...\n");

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
           printk(S5KA3DFX_MOD_NAME "[format]Invalid value is ordered!!!\n");
          return -EINVAL;
      }
      break;
      
    default:
       printk(S5KA3DFX_MOD_NAME "[type]Invalid value is ordered!!!\n");
      return -EINVAL;
  }

  fmt->flags = s5ka3dfx_formats[index].flags;
  fmt->pixelformat = s5ka3dfx_formats[index].pixelformat;
  strlcpy(fmt->description, s5ka3dfx_formats[index].description, sizeof(fmt->description));  

  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_enum_fmt_cap flag : %d\n", fmt->flags);
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_enum_fmt_cap description : %s\n", fmt->description);

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
  struct s5ka3dfx_sensor *sensor = s->priv;
  struct v4l2_pix_format *pix2 = &sensor->pix;

  int index = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_try_fmt_cap is called...\n");
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_try_fmt_cap. mode : %d\n", sensor->mode);
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_try_fmt_cap. state : %d\n", sensor->state);
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_try_fmt_cap. pix width : %d\n", pix->width);
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_try_fmt_cap. pix height : %d\n", pix->height);    

  s5ka3dfx_set_skip();

  if(sensor->state == S5KA3DFX_STATE_CAPTURE)
  {
    for(index = 0; index < ARRAY_SIZE(s5ka3dfx_image_sizes); index++)
    {
      if(s5ka3dfx_image_sizes[index].width == pix->width
      && s5ka3dfx_image_sizes[index].height == pix->height)
      {
        sensor->capture_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5ka3dfx_image_sizes))
    {
      printk(S5KA3DFX_MOD_NAME "Capture Image Size is not supported!\n");
      goto try_fmt_fail;
    }  

    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "capture size = %d\n", sensor->capture_size);  
    
    pix->field = V4L2_FIELD_NONE;
    if(pix->pixelformat == V4L2_PIX_FMT_UYVY || pix->pixelformat == V4L2_PIX_FMT_YUYV)
    {
      pix->bytesperline = pix->width * 2;
      pix->sizeimage = pix->bytesperline * pix->height;
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "V4L2_PIX_FMT_YUYV\n");
    }
    else
    {
      /* paladin[08.10.14]: For JPEG Capture, use fixed buffer size @LDK@ */
      pix->bytesperline = JPEG_CAPTURE_WIDTH * 2; /***** !!!fixme mingyu diffent sensor!!!**********/
      pix->sizeimage = pix->bytesperline * JPEG_CAPTURE_HEIGHT;
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "V4L2_PIX_FMT_JPEG\n");
    }

    if(s5ka3dfx_curr_state == S5KA3DFX_STATE_INVALID)
    {
      if(s5ka3dfx_set_init())
      {
        printk(S5KA3DFX_MOD_NAME "Unable to detect " S5KA3DFX_DRIVER_NAME " sensor\n");
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
  printk(S5KA3DFX_MOD_NAME "ioctl_try_fmt_cap is failed\n"); 
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
  struct s5ka3dfx_sensor *sensor = s->priv;
  struct v4l2_pix_format *pix2 = &sensor->pix;

  int index = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_s_fmt_cap is called...\n");
  
  printk(S5KA3DFX_MOD_NAME "camera mode  : %d (1:camera , 2:camcorder, 3:vt)\n", sensor->mode);
  printk(S5KA3DFX_MOD_NAME "camera state : %d (0:preview, 1:snapshot)\n", sensor->state);
  printk(S5KA3DFX_MOD_NAME "set width  : %d\n", pix->width);
  printk(S5KA3DFX_MOD_NAME "set height  : %d\n", pix->height);    

  if(sensor->state == S5KA3DFX_STATE_CAPTURE)
  {
    s5ka3dfx_set_skip();
  
    for(index = 0; index < ARRAY_SIZE(s5ka3dfx_image_sizes); index++)
    {
      if(s5ka3dfx_image_sizes[index].width == pix->width
      && s5ka3dfx_image_sizes[index].height == pix->height)
      {
        sensor->capture_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5ka3dfx_image_sizes))
    {
      printk(S5KA3DFX_MOD_NAME "Capture Image %d x %d Size is not supported!\n", pix->width, pix->height);
      goto s_fmt_fail;
    }  

    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "capture size = %d\n", sensor->capture_size);  
    
    pix->field = V4L2_FIELD_NONE;
    if(pix->pixelformat == V4L2_PIX_FMT_UYVY || pix->pixelformat == V4L2_PIX_FMT_YUYV)
    {
      pix->bytesperline = pix->width * 2;
      pix->sizeimage = pix->bytesperline * pix->height;
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "V4L2_PIX_FMT_YUYV\n");
    }
    else
    {
      /* paladin[08.10.14]: For JPEG Capture, use fixed buffer size @LDK@ */
      pix->bytesperline = JPEG_CAPTURE_WIDTH * 2; /***** !!!fixme mingyu diffent sensor!!!**********/
      pix->sizeimage = pix->bytesperline * JPEG_CAPTURE_HEIGHT;
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "V4L2_PIX_FMT_JPEG\n");
    }

    if(s5ka3dfx_curr_state == S5KA3DFX_STATE_INVALID)
    {
      if(s5ka3dfx_set_init())
      {
        printk(S5KA3DFX_MOD_NAME "Unable to detect " S5KA3DFX_DRIVER_NAME " sensor\n");
        goto s_fmt_fail;
      }
    }     
  }  
  else
  {
    s5ka3dfx_set_skip();
  
    for(index = 0; index < ARRAY_SIZE(s5ka3dfx_preview_sizes); index++)
    {
      if(s5ka3dfx_preview_sizes[index].width == pix->width
      && s5ka3dfx_preview_sizes[index].height == pix->height)
      {
        sensor->preview_size = index;
        break;
      }
    }   

    if(index == ARRAY_SIZE(s5ka3dfx_preview_sizes))
    {
      printk(S5KA3DFX_MOD_NAME "Preview Image %d x %d Size is not supported!\n", pix->width, pix->height);
      goto s_fmt_fail;
    }
  
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "preview size = %d\n", sensor->preview_size);
    
    pix->field = V4L2_FIELD_NONE;
    pix->bytesperline = pix->width * 2;
    pix->sizeimage = pix->bytesperline * pix->height;  

    if(s5ka3dfx_curr_state == S5KA3DFX_STATE_INVALID)
    {
      if(s5ka3dfx_set_init())
      {
        printk(S5KA3DFX_MOD_NAME "Unable to detect " S5KA3DFX_DRIVER_NAME " sensor\n");
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
  printk(S5KA3DFX_MOD_NAME "ioctl_s_fmt_cap is failed\n"); 
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
  struct s5ka3dfx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_g_fmt_cap is called...\n");
  
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
  struct s5ka3dfx_sensor *sensor = s->priv;
  struct v4l2_captureparm *cparm = &a->parm.capture;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_g_parm is called...\n");

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
  struct s5ka3dfx_sensor *sensor = s->priv;
  struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_s_parm is called...\n");

  /* Set mode (camera/camcorder/vt) & state (preview/capture) */
  sensor->mode = a->parm.capture.capturemode;
  sensor->state = a->parm.capture.currentstate;

  if(sensor->mode < 1 || sensor->mode > 3) sensor->mode = S5KA3DFX_MODE_CAMERA;
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "mode = %d, state = %d\n", sensor->mode, sensor->state); 

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
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "fps = %d\n", sensor->fps);  
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "numerator : %d, denominator: %d\n", timeperframe->numerator, timeperframe->denominator);
  
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
  struct s5ka3dfx_sensor *sensor = s->priv;
  int rval;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_g_ifparm is called...\n");

  rval = sensor->pdata->ifparm(p);
  if (rval)
  {
    return rval;
  }

  p->u.bt656.clock_curr = S5KA3DFX_XCLK;

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
  struct s5ka3dfx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_g_priv is called...\n");
  
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
  struct s5ka3dfx_sensor* sensor = s->priv;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_enum_framesizes fmt\n");   

  if (sensor->state == S5KA3DFX_STATE_CAPTURE)
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "Size enumeration for image capture = %d\n", sensor->capture_size);
  
    frms->index = sensor->capture_size;
    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = s5ka3dfx_image_sizes[sensor->capture_size].width;
    frms->discrete.height = s5ka3dfx_image_sizes[sensor->capture_size].height;       
  }
  else
  {    
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "Size enumeration for preview = %d\n", sensor->preview_size);
    
    frms->index = sensor->preview_size;
    frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    frms->discrete.width = s5ka3dfx_preview_sizes[sensor->preview_size].width;
    frms->discrete.height = s5ka3dfx_preview_sizes[sensor->preview_size].height;       
  }

  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "framesizes width : %d\n", frms->discrete.width); 
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "framesizes height : %d\n", frms->discrete.height); 

  return 0;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *frmi)
{
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_enum_frameintervals \n"); 
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_enum_frameintervals numerator : %d\n", frmi->discrete.numerator); 
  dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "ioctl_enum_frameintervals denominator : %d\n", frmi->discrete.denominator); 

  return 0;
}


/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
  struct s5ka3dfx_sensor *sensor = s->priv;

  int err = 0;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_s_power is called......ON=%x, detect= %x\n", on, sensor->detect);

  sensor->pdata->power_set(on);

  switch(on)
  {
    case V4L2_POWER_ON:
    {
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "pwr on-----!\n");
      front_cam_in_use= 1 ;
      err = s5ka3dfx_get_rev();
      if(err)
      {

        front_cam_in_use= 0 ;
        printk(S5KA3DFX_MOD_NAME "Unable to detect " S5KA3DFX_DRIVER_NAME " sensor\n");
        sensor->pdata->power_set(V4L2_POWER_OFF);
        return err;
      }

      /* Make the default zoom */
      sensor->zoom = S5KA3DFX_ZOOM_1P00X;      

      /* Make the default detect */
      sensor->detect = SENSOR_DETECTED;

      /* Make the state init */
      s5ka3dfx_curr_state = S5KA3DFX_STATE_INVALID;      
    }
    break;

    case V4L2_POWER_RESUME:
    {
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "pwr resume-----!\n");
    }
    break;

    case V4L2_POWER_STANDBY:
    {
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "pwr stanby-----!\n");
    }
    break;

    case V4L2_POWER_OFF:
    {
      dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "pwr off-----!\n");
        front_cam_in_use= 0 ;

      /* Make the default zoom */
      sensor->zoom = S5KA3DFX_ZOOM_1P00X;

      /* Make the default detect */
      sensor->detect = SENSOR_NOT_DETECTED;
      
      /* Make the state init */
      s5ka3dfx_pre_state = S5KA3DFX_STATE_INVALID;       
    }
    break;
  }

  return err;
}

static int ioctl_g_exif(struct v4l2_int_device *s, struct v4l2_exif *exif)
{
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_g_exif is called...\n");

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
  struct s5ka3dfx_sensor *sensor = s->priv;
  
  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_init is called...\n");

  sensor->state = S5KA3DFX_STATE_INVALID; //init problem
  
  return 0;
}


/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call s5ka3dfx_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
  struct s5ka3dfx_sensor *sensor = s->priv;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "ioctl_init is called...\n");

  //init value
  sensor->timeperframe.numerator    = 1,
  sensor->timeperframe.denominator  = 15,
  sensor->mode                      = S5KA3DFX_MODE_CAMERA;
  sensor->state                     = S5KA3DFX_STATE_INVALID;
  sensor->fps                       = 15;
  sensor->preview_size              = S5KA3DFX_PREVIEW_SIZE_640_480;
  sensor->capture_size              = S5KA3DFX_IMAGE_SIZE_640_480;
  sensor->detect                    = SENSOR_NOT_DETECTED;
  sensor->zoom                      = S5KA3DFX_ZOOM_1P00X;
  sensor->effect                    = S5KA3DFX_EFFECT_OFF;
  sensor->ev                        = S5KA3DFX_EV_DEFAULT;
  sensor->wb                        = S5KA3DFX_WB_AUTO;
  sensor->pretty                    = S5KA3DFX_PRETTY_NONE;
  sensor->flip                      = S5KA3DFX_FLIP_NONE;

  memcpy(&s5ka3dfx, sensor, sizeof(struct s5ka3dfx_sensor));
  
  return 0;
}

static struct v4l2_int_ioctl_desc s5ka3dfx_ioctl_desc[] = {
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

static struct v4l2_int_slave s5ka3dfx_slave = {
  .ioctls = s5ka3dfx_ioctl_desc,
  .num_ioctls = ARRAY_SIZE(s5ka3dfx_ioctl_desc),
};

static struct v4l2_int_device s5ka3dfx_int_device = {
  .module = THIS_MODULE,
  .name = S5KA3DFX_DRIVER_NAME,
  .priv = &s5ka3dfx,
  .type = v4l2_int_type_slave,
  .u = {
    .slave = &s5ka3dfx_slave,
  },
};


/**
 * s5ka3dfx_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
s5ka3dfx_probe(struct i2c_client *client, const struct i2c_device_id *device)
{
  struct s5ka3dfx_sensor *sensor = &s5ka3dfx;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_probe is called...\n");

  if (i2c_get_clientdata(client))
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "can't get i2c client data!!");
    return -EBUSY;
  }

  sensor->pdata = &nowplus_s5ka3dfx_platform_data;

  if (!sensor->pdata) 
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "no platform data?\n");
    return -ENODEV;
  }

  sensor->v4l2_int_device = &s5ka3dfx_int_device;
  sensor->i2c_client = client;

  /* Make the default capture size VGA */
  sensor->pix.width = 640;
  sensor->pix.height = 480;

  /* Make the default capture format V4L2_PIX_FMT_UYVY */
  sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;

  i2c_set_clientdata(client, sensor);

  if (v4l2_int_device_register(sensor->v4l2_int_device))
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "fail to init device register \n");
    i2c_set_clientdata(client, NULL);
  }

  return 0;
}

/**
 * s5ka3dfx_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of s5ka3dfx_probe().
 */
static int __exit
s5ka3dfx_remove(struct i2c_client *client)
{
  struct s5ka3dfx_sensor *sensor = i2c_get_clientdata(client);

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_remove is called...\n");

  if (!client->adapter)
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "no i2c client adapter!!");
    return -ENODEV; /* our client isn't attached */
  }

  v4l2_int_device_unregister(sensor->v4l2_int_device);
  i2c_set_clientdata(client, NULL);

  return 0;
}

static const struct i2c_device_id s5ka3dfx_id[] = {
  { S5KA3DFX_DRIVER_NAME, 0 },
  { },
};

MODULE_DEVICE_TABLE(i2c, s5ka3dfx_id);


static struct i2c_driver s5ka3dfxsensor_i2c_driver = {
  .driver = {
    .name = S5KA3DFX_DRIVER_NAME,
  },
  .probe = s5ka3dfx_probe,
  .remove = __exit_p(s5ka3dfx_remove),
  .id_table = s5ka3dfx_id,
};

/**
 * s5ka3dfx_sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init s5ka3dfx_sensor_init(void)
{
  int err;

  dprintk(CAM_INF, S5KA3DFX_MOD_NAME "s5ka3dfx_sensor_init is called...\n");

  err = i2c_add_driver(&s5ka3dfxsensor_i2c_driver);
  if (err) 
  {
    dprintk(CAM_DBG, S5KA3DFX_MOD_NAME "Failed to register" S5KA3DFX_DRIVER_NAME ".\n");
    return err;
  }
  
  return 0;
}

module_init(s5ka3dfx_sensor_init);

/**
 * s5ka3dfxsensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of s5ka3dfx_sensor_init.
 */
static void __exit s5ka3dfxsensor_cleanup(void)
{
  i2c_del_driver(&s5ka3dfxsensor_i2c_driver);
}
module_exit(s5ka3dfxsensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5KA3DFX camera sensor driver");
