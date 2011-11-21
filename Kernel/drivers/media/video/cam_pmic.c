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
 * modules/camera/cam_pmic.c
 *
 * Camera PMIC driver source file
 *
 * Modified by paladin in Samsung Electronics
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

#include "cam_pmic.h"

#if (CAM_PMIC_DBG)
#include "dprintk.h"
#else
#define dprintk(x, y...)
#endif

static struct cam_pmic pmic;
static struct i2c_driver cam_pmic_i2c_driver;

int cam_pmic_read_reg(u8 reg, u8* val)
{
  struct cam_pmic * camera_pmic = &pmic;
  struct i2c_client *client = camera_pmic->i2c_client;

  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];

  dprintk(CAM_DBG, "cam_pmic_read_reg is called...\n");

  if (!client->adapter)
    return -ENODEV;

  msg->addr = client->addr;
  msg->flags =  0;
  msg->len = 1;
  msg->buf = data;

  /* high byte goes out first */
  data[0] = (u8) reg;
  err = i2c_transfer(client->adapter, msg, 1);
  if (err >= 0) {
    msg->len = 1;
    msg->flags = I2C_M_RD;
    err = i2c_transfer(client->adapter, msg, 1);
  }

  if (err >= 0) {
    *val = data[0];

    dprintk(CAM_DBG, CAM_PMIC_MOD_NAME "Value from Reg:0x%x ==> 0x%x\n", reg, *val);
    return 0;
  }

  dprintk(CAM_DBG, CAM_PMIC_MOD_NAME "read from Reg:0x%x error %d\n", reg, err);

  return err;
}
EXPORT_SYMBOL(cam_pmic_read_reg);

int cam_pmic_write_reg(u8 reg, u8 val)
{
  struct cam_pmic * camera_pmic = &pmic;
  struct i2c_client *client = camera_pmic->i2c_client;

  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];
  int retry = 0;

  dprintk(CAM_DBG, "cam_pmic_write_reg is called...\n");

  if (!client->adapter)
  {
    printk("no i2c client adapter!!");
    return -ENODEV;
  }

  again:
  msg->addr = client->addr;
  msg->flags = 0;
  msg->len = 2;
  msg->buf = data;

  /* high byte goes out first */
  data[0] = reg;
  data[1] = val;

  err = i2c_transfer(client->adapter, msg, 1);
  if (err >= 0)
  {
    dprintk(CAM_DBG, CAM_PMIC_MOD_NAME "wrote 0x%x to offset 0x%x success!\n", val, reg);
    return 0;
  }

  dprintk(CAM_ERR, CAM_PMIC_MOD_NAME "wrote 0x%x to offset 0x%x error %d\n", val, reg, err);
  if (retry <= CAM_PMIC_I2C_RETRY) {
    dprintk(CAM_ERR, CAM_PMIC_MOD_NAME "retry ... %d\n", retry);	
    retry++;
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule_timeout(msecs_to_jiffies(20));
    goto again;
  }
  return err;
}
EXPORT_SYMBOL(cam_pmic_write_reg);

/**
 * cam_pmic_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
cam_pmic_probe(struct i2c_client *client, const struct i2c_device_id *device)
{   
  dprintk(CAM_INF, "cam_pmic_probe is called...\n");

  if (i2c_get_clientdata(client))
  {
    printk("can't get i2c client data!!\n");
    return -EBUSY;
  }

  pmic.i2c_client = client;
  i2c_set_clientdata(client, &pmic);

  return 0;
}

/**
 * cam_pmic_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of m4mo_probe().
 */
static int __exit
cam_pmic_remove(struct i2c_client *client)
{
  dprintk(CAM_INF, "cam_pmic_remove is called...\n");
  
  if (!client->adapter)
  {
    printk("no i2c client adapter!!");
    return -ENODEV;
  }

  i2c_set_clientdata(client, NULL);

  return 0;
}

static const struct i2c_device_id cam_pmic_id[] = {
  { CAM_PMIC_DRIVER_NAME, 0 },
  { },
};
MODULE_DEVICE_TABLE(i2c, cam_pmic_id);

static struct i2c_driver cam_pmic_i2c_driver = {
  .driver = {
    .name = CAM_PMIC_DRIVER_NAME,
    .owner = THIS_MODULE,
  },
  .probe = cam_pmic_probe,
  .remove = __exit_p(cam_pmic_remove),
  .id_table = cam_pmic_id,
};



/**
 * cam_pmic_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init cam_pmic_init(void)
{
  int err;

  dprintk(CAM_INF, "cam_pmic_init is called...\n");

  err = i2c_add_driver(&cam_pmic_i2c_driver);
  if (err) {
    dprintk(CAM_ERR, CAM_PMIC_MOD_NAME "Failed to register" CAM_PMIC_DRIVER_NAME ".\n");
    return err;
  }
  return 0;
}
module_init(cam_pmic_init);

/**
 * cam_pmic_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of m4mosensor_init.
 */
static void __exit cam_pmic_cleanup(void)
{
  i2c_del_driver(&cam_pmic_i2c_driver);
}
module_exit(cam_pmic_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Camera PMIC Driver");
