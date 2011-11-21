/*
 * linux/arch/arm/mach-omap2/board-3430ldp.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *****************************************************
 *****************************************************
 * modules/camera/ce147_platform.c
 *
 * CE147 sensor driver file related to platform
 *
 * Modified by paladin in Samsung Electronics
 */
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <linux/mm.h>
#include <media/v4l2-int-device.h>
#include "omap34xxcam.h"
#include <../drivers/media/video/isp/ispreg.h>
#include "ce147.h"
#include "cam_pmic.h"

#if (!CAM_CE147_DBG_MSG)
#include "dprintk.h"
#else
#define dprintk(x, y...)
#endif

#define CE147_BIGGEST_FRAME_BYTE_SIZE  PAGE_ALIGN(1280 * 720 * 2 *6) //fix for usage of 6 buffers for 720p capture and avoiding camera launch issues.

static struct v4l2_ifparm ifparm_ce147 = {
  .if_type = V4L2_IF_TYPE_BT656, // fix me
  .u = {
    .bt656 = {
      .frame_start_on_rising_vs = 0,
      .latch_clk_inv = 0,
      .mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT, 
      .clock_min = CE147_XCLK,
      .clock_max = CE147_XCLK,
      .clock_curr = CE147_XCLK,
    },
  },
};

static struct omap34xxcam_sensor_config ce147_hwc = {
  .sensor_isp = 1,
//  .xclk = OMAP34XXCAM_XCLK_A,
  .capture_mem =  CE147_BIGGEST_FRAME_BYTE_SIZE, 
};

struct isp_interface_config ce147_if_config = {
  .ccdc_par_ser = ISP_PARLL,
  .dataline_shift = 0x2,
  .hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
  .wait_hs_vs = 0x03,
  .strobe = 0x0,
  .prestrobe = 0x0,
  .shutter = 0x0,
  .u.par.par_bridge = 0x3,
  .u.par.par_clk_pol = 0x0,
};

static int ce147_enable_gpio(void)
{
  dprintk(CAM_INF, CE147_MOD_NAME "ce147_enable_gpio is called...\n");

  gpio_free(OMAP3430_GPIO_CAMERA_EN);
  gpio_free(OMAP3430_GPIO_CAMERA_EN2);
#if (CONFIG_SAMSUNG_REL_HW_REV >= 1)    
  gpio_free(OMAP3430_GPIO_CAMERA_EN3);
#endif  
  gpio_free(OMAP3430_GPIO_CAMERA_RST);
  gpio_free(OMAP3430_GPIO_CAMERA_STBY);
  gpio_free(OMAP3430_GPIO_VGA_RST);
  gpio_free(OMAP3430_GPIO_VGA_STBY);  

  if (gpio_request(OMAP3430_GPIO_CAMERA_EN,"CAM EN") != 0)
  {
    printk(CE147_MOD_NAME "Could not request GPIO %d\n", OMAP3430_GPIO_CAMERA_EN);
    return -EIO;
  }

  if (gpio_request(OMAP3430_GPIO_CAMERA_EN2,"CAM EN2") != 0)
  {
	printk(CE147_MOD_NAME "Could not request GPIO %d\n", OMAP3430_GPIO_CAMERA_EN2);
	return -EIO;
  }

#if (CONFIG_SAMSUNG_REL_HW_REV >= 1)  
  if (gpio_request(OMAP3430_GPIO_CAMERA_EN3,"CAM EN3") != 0) 
  {
    printk(CE147_MOD_NAME "Could not request GPIO %d", OMAP3430_GPIO_CAMERA_EN3);
    return -EIO;
  }
#endif  

  if (gpio_request(OMAP3430_GPIO_CAMERA_RST,"CAM RST") != 0)
  {
    printk(CE147_MOD_NAME "Could not request GPIO %d\n", OMAP3430_GPIO_CAMERA_RST);
    return -EIO;
  }

  if (gpio_request(OMAP3430_GPIO_CAMERA_STBY,"CAM STBY") != 0)
  {
    printk(CE147_MOD_NAME "Could not request GPIO %d\n",  OMAP3430_GPIO_CAMERA_STBY);
    return -EIO;
  }

  if (gpio_request(OMAP3430_GPIO_VGA_RST,"VGA RST") != 0)
  {
    printk(CE147_MOD_NAME "Could not request GPIO %d\n",  OMAP3430_GPIO_VGA_RST);
    return -EIO;
  }

  if (gpio_request(OMAP3430_GPIO_VGA_STBY,"VGA STBY") != 0)
  {
    printk(CE147_MOD_NAME "Could not request GPIO %d\n",  OMAP3430_GPIO_VGA_STBY);
    return -EIO;
  }

  /* PMIC init */
  if(cam_pmic_write_reg(0x01, 0x00) != 0) //LDO1 CAM_SENSOR_CORE_1.2V
  {
    printk(CE147_MOD_NAME "Could not request voltage LDO1 1.2V\n");
    return -EIO;      
  }   
  
  if(cam_pmic_write_reg(0x02, 0xF9) != 0) //LDO2 CAM_ISP_SYS_2.8V
  {
    printk(CE147_MOD_NAME "Could not request voltage LDO2 2.8V\n");
    return -EIO;       
  }
  
  if(cam_pmic_write_reg(0x03, 0xEC) != 0) //LDO3 CAM_ISP_HOST_1.8V
  {
    printk(CE147_MOD_NAME "Could not request voltage LDO3 1.8V\n");
    return -EIO;       
  }

  if(cam_pmic_write_reg(0x04, 0xF1) != 0) //LDO4 CAM_ISP_RAM_1.8V  
  {
    printk(CE147_MOD_NAME "Could not request voltage LDO4 1.8V\n");
    return -EIO;       
  }
  
  if(cam_pmic_write_reg(0x05, 0xF9) != 0) //LDO5 CAM_AF_2.8V
  {
    printk(CE147_MOD_NAME "Could not request voltage LDO5 2.8V\n");
    return -EIO;       
  }
  
  if(cam_pmic_write_reg(0x06, 0x0B) != 0) //BUCK CAM_ISP_CORE_1.3V  
  {
    printk(CE147_MOD_NAME "Could not request voltage BUCK 1.3V\n");
    return -EIO;       
  }

  /* Reset the GPIO pins */
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN, 0);
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN2, 0);
#if (CONFIG_SAMSUNG_REL_HW_REV >= 1)    
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN3, 0);      
#endif  
  gpio_direction_output(OMAP3430_GPIO_CAMERA_RST, 0);
  gpio_direction_output(OMAP3430_GPIO_VGA_RST, 0);
  gpio_direction_output(OMAP3430_GPIO_VGA_STBY, 0);  

  isp_set_xclk(0,0,0);

  /* Enable sensor module power */
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN, 1);
  udelay(250);
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN2, 1);
#if (CONFIG_SAMSUNG_REL_HW_REV >= 1)  
  mdelay(2);
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN3, 1);
#endif  
  mdelay(5);

  gpio_direction_output(OMAP3430_GPIO_VGA_STBY, 1);  
  mdelay(1);  
  /* Clock Enable */
  isp_set_xclk(0,CE147_XCLK,0);
  mdelay(5);
  gpio_direction_output(OMAP3430_GPIO_VGA_RST, 1);  
  mdelay(8); 
  gpio_direction_output(OMAP3430_GPIO_VGA_STBY, 0); 
  mdelay(5); 
  
  gpio_direction_output(OMAP3430_GPIO_CAMERA_STBY, 1);
  mdelay(10);

  /* Activate Reset */
  gpio_direction_output(OMAP3430_GPIO_CAMERA_RST, 1);
  mdelay(10);

  return 0; 
}

static int ce147_disable_gpio(void)
{
  dprintk(CAM_INF, CE147_MOD_NAME "ce147_disable_gpio is called...\n");
  
  gpio_direction_output(OMAP3430_GPIO_CAMERA_RST, 0);
  mdelay(2);

  isp_set_xclk(0,0,0);

  gpio_direction_output(OMAP3430_GPIO_CAMERA_STBY, 0);
  mdelay(2);

  gpio_direction_output(OMAP3430_GPIO_VGA_RST, 0);  
  mdelay(2); 

#if (CONFIG_SAMSUNG_REL_HW_REV >= 1)  
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN3, 0);
#endif  
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN2, 0);
  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN, 0);
  mdelay(2);

  gpio_free(OMAP3430_GPIO_CAMERA_EN);
  gpio_free(OMAP3430_GPIO_CAMERA_EN2);
#if (CONFIG_SAMSUNG_REL_HW_REV >= 1)    
  gpio_free(OMAP3430_GPIO_CAMERA_EN3);
#endif  
  gpio_free(OMAP3430_GPIO_CAMERA_RST);
  gpio_free(OMAP3430_GPIO_CAMERA_STBY);
  gpio_free(OMAP3430_GPIO_VGA_RST);
  gpio_free(OMAP3430_GPIO_VGA_STBY);  
  
  return 0;     
}

static int ce147_sensor_power_set(enum v4l2_power power)
{
  static enum v4l2_power c_previous_pwr = V4L2_POWER_OFF;

  int err = 0;

  printk("ce147_sensor_power_set is called...[%x]\n", power);

  switch (power) 
  {
    case V4L2_POWER_OFF:
    {
      err = ce147_disable_gpio();
    }
    break;

    case V4L2_POWER_ON:
    {
      isp_configure_interface(0,&ce147_if_config);

      err = ce147_enable_gpio();    
	if(err)
	{
	       printk("ce147_sensor_power_set RETRY...[%x]\n", err);	
	       err = ce147_disable_gpio();	
		err = ce147_enable_gpio();    	
	}			  
    }
    break;

    case V4L2_POWER_STANDBY:
      break;

    case V4L2_POWER_RESUME:
      break;
  }
  
  c_previous_pwr = power;

  return err;
}


static int ce147_ifparm(struct v4l2_ifparm *p)
{
  printk("ce147_ifparm is called...\n");
  
  *p = ifparm_ce147;
  
  return 0;
}


static int ce147_sensor_set_prv_data(void *priv)
{
  struct omap34xxcam_hw_config *hwc = priv;

  printk("ce147_sensor_set_prv_data is called...\n");

//  hwc->u.sensor.xclk = ce147_hwc.xclk;
  hwc->u.sensor.sensor_isp = ce147_hwc.sensor_isp;
  hwc->u.sensor.capture_mem = ce147_hwc.capture_mem;
  hwc->dev_index = 0;
  hwc->dev_minor = 0;
  hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;

  return 0;
}


struct ce147_platform_data nowplus_ce147_platform_data = {
  .power_set      = ce147_sensor_power_set,
  .priv_data_set  = ce147_sensor_set_prv_data,
  .ifparm         = ce147_ifparm,
};
