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
 * modules/camera/s5k6aafx_platform.c
 *
 * S5K6AAFX sensor driver file related to platform
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
//#include <mach/mux.h>
#include <media/v4l2-int-device.h>
#include "omap34xxcam.h"
#include <../drivers/media/video/isp/ispreg.h>
#include "s5k6aafx.h"
#include "cam_pmic.h"

#if (CAM_S5K6AAFX_DBG_MSG)
#include "dprintk.h"
#else
#define dprintk(x, y...)
#endif

static struct v4l2_ifparm ifparm_s5k6aafx = {
  .if_type = V4L2_IF_TYPE_BT656, // fix me
  .u = {
    .bt656 = {
      .frame_start_on_rising_vs = 0,
      .latch_clk_inv = 0,
      .mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT, 
      .clock_min = S5K6AAFX_XCLK,
      .clock_max = S5K6AAFX_XCLK,
      .clock_curr = S5K6AAFX_XCLK,
    },
  },
};

static struct omap34xxcam_sensor_config s5k6aafx_hwc = {
  .sensor_isp = 1,
//  .xclk = OMAP34XXCAM_XCLK_A,
};

struct isp_interface_config s5k6aafx_if_config = {
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

static int s5k6aafx_enable_gpio(void)
{

	int retry = 0;
	
	dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_enable_gpio is called...\n");

retry_gpio_request :

	/* Request and configure gpio pins */
	if (gpio_request(OMAP3430_GPIO_CAMERA_EN,"CAM EN") != 0)
	{
		if( retry == 0) {
			printk("[%s:%d] retry gpio request",__func__, __LINE__);
			mdelay(200);
			retry++;
			goto retry_gpio_request;
		}
		printk(S5K6AAFX_MOD_NAME "Could not request GPIO %d", OMAP3430_GPIO_CAMERA_EN);
		return -EIO;
	}

	if (gpio_request(OMAP3430_GPIO_VGA_STBY,"VGA STDBY") != 0)
	{
		printk(S5K6AAFX_MOD_NAME "Could not request GPIO %d", OMAP3430_GPIO_VGA_STBY);
		return -EIO;
	}

	if (gpio_request(OMAP3430_GPIO_VGA_RST,"CAM VGA RST") != 0)
	{
		printk(S5K6AAFX_MOD_NAME "Could not request GPIO %d", OMAP3430_GPIO_VGA_RST);
		return -EIO;
	}

	/* PMIC init */
    if(cam_pmic_write_reg(0x03, 0x86) != 0) //LDO3 CAM_CIF_1.6V
	{
		printk(S5K6AAFX_MOD_NAME "Could not request voltage LDO3 1.6V\n");
		return -EIO;
	}

	if(cam_pmic_write_reg(0x04, 0xB1) != 0) //LDO4 CAM_IO_1.8V
	{
		printk(S5K6AAFX_MOD_NAME "Could not request voltage LDO4 1.8V\n");
		return -EIO;
	}
	
	if(cam_pmic_write_reg(0x08, 0x8E) != 0) //LDO8 SETTING disable pin(LDO3)
	{
		printk(S5K6AAFX_MOD_NAME "Could not request SETTING disable pin(LDO3)\n");
		return -EIO;
	}

	/* Reset the GPIO pins */
	gpio_direction_output(OMAP3430_GPIO_CAMERA_EN, 0);
	gpio_direction_output(OMAP3430_GPIO_VGA_STBY, 0);
	gpio_direction_output(OMAP3430_GPIO_VGA_RST, 0);

	isp_set_xclk(0,0,0);

	/* Enable sensor module power */
	gpio_direction_output(OMAP3430_GPIO_CAMERA_EN, 1);
	mdelay(5);

	/* Activate STBY */
	gpio_direction_output(OMAP3430_GPIO_VGA_STBY, 1);
	mdelay(5);

	/* Clock Enable */
	isp_set_xclk(0,S5K6AAFX_XCLK,0);
	mdelay(5);

	/* Activate Reset */
	gpio_direction_output(OMAP3430_GPIO_VGA_RST, 1);
	mdelay(10);

	return 0;
}

static int s5k6aafx_disable_gpio(void)
{

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_disable_gpio is called...\n");

  gpio_direction_output(OMAP3430_GPIO_VGA_RST, 0);
  mdelay(10);

  isp_set_xclk(0,0, 0);

  gpio_direction_output(OMAP3430_GPIO_VGA_STBY, 0);
  mdelay(2);         

  gpio_direction_output(OMAP3430_GPIO_CAMERA_EN, 0);
  mdelay(2);

  gpio_free(OMAP3430_GPIO_VGA_RST);
  gpio_free(OMAP3430_GPIO_VGA_STBY);
  gpio_free(OMAP3430_GPIO_CAMERA_EN);     

  return 0;
}

static int s5k6aafx_sensor_power_set(enum v4l2_power power)
{
  static enum v4l2_power s_previous_pwr = V4L2_POWER_OFF;

  int err = 0;

  printk(S5K6AAFX_MOD_NAME "s5k6aafx_sensor_power_set is called...[%x] (0:OFF, 1:ON)\n", power);

  switch (power) 
  {
    case V4L2_POWER_OFF:
    {
      err = s5k6aafx_disable_gpio();
    }      
    break;

    case V4L2_POWER_ON:
    {
      isp_configure_interface(0,&s5k6aafx_if_config);

      err = s5k6aafx_enable_gpio();       
    }
    break;

    case V4L2_POWER_STANDBY:
      break;

    case V4L2_POWER_RESUME:
      break;
  }
  
  s_previous_pwr = power;

  return err;
}


static int s5k6aafx_ifparm(struct v4l2_ifparm *p)
{
  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_ifparm is called...\n");
  
  *p = ifparm_s5k6aafx;
  
  return 0;
}


static int s5k6aafx_sensor_set_prv_data(void *priv)
{
  struct omap34xxcam_hw_config *hwc = priv;

  dprintk(CAM_INF, S5K6AAFX_MOD_NAME "s5k6aafx_sensor_set_prv_data is called...\n");

//  hwc->u.sensor.xclk = s5k6aafx_hwc.xclk;
  hwc->u.sensor.sensor_isp = s5k6aafx_hwc.sensor_isp;
  hwc->dev_index = 1;
  hwc->dev_minor = 5;
  hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;

  return 0;
}


struct s5k6aafx_platform_data nowplus_s5k6aafx_platform_data = {
  .power_set      = s5k6aafx_sensor_power_set,
  .priv_data_set  = s5k6aafx_sensor_set_prv_data,
  .ifparm         = s5k6aafx_ifparm,
};

