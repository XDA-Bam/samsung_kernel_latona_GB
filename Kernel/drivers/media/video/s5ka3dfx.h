/*
 * drivers/media/video/s5ka3dfx.h
 *
 * Register definitions for the NEC S5KA3DFX CameraChip.
 *
 * Author: Sameer Venkatraman, Mohit Jalori (ti.com)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *****************************************************
 *****************************************************
 * modules/camera/s5ka3dfx.h
 *
 * S5KA3DFX sensor driver header file
 *
 * Modified by paladin in Samsung Electronics
 */

#ifndef S5KA3DFX_H
#define S5KA3DFX_H
#define CAM_S5KA3DFX_DBG_MSG              0
#define S5KA3DFX_DRIVER_NAME              "s5ka3dfx"
#define S5KA3DFX_MOD_NAME                 "S5KA3DFX: "

#define S5KA3DFX_I2C_ADDR                 0xC4>>1
#define S5KA3DFX_I2C_RETRY                10
#define S5KA3DFX_XCLK                     24000000

#define SENSOR_DETECTED                   1
#define SENSOR_NOT_DETECTED               0


#define OMAP3430_GPIO_CAMERA_EN           152
#define OMAP3430_GPIO_CAMERA_EN2          186
#define OMAP3430_GPIO_CAMERA_EN3          177

#define OMAP3430_GPIO_CAMERA_RST          98
#define OMAP3430_GPIO_CAMERA_STBY         153

#define OMAP3430_GPIO_VGA_RST             64 
#define OMAP3430_GPIO_VGA_STBY            101


#define CONFIG_NOWPLUS_HW_REV CONFIG_NOWPLUS_REV08 // think so!

#define CONFIG_NOWPLUS_REV01                    10      /* REV01 */
#define CONFIG_NOWPLUS_REV01_N01                11      /* REV01 */
#define CONFIG_NOWPLUS_REV01_N02                12      /* REV01 ONEDRAM*/
#define CONFIG_NOWPLUS_REV01_N03                13      /* REV01 REAL*/
#define CONFIG_NOWPLUS_REV01_N04                14      /* REV01 ONEDRAM1G*/
#define CONFIG_NOWPLUS_REV02                    20      /* REV02 UNIVERSAL*/
#define CONFIG_NOWPLUS_REV02_N01                21      /* REV02 REAL*/
#define CONFIG_NOWPLUS_REV03                    30      /* REV03 REAL*/
#define CONFIG_NOWPLUS_REV03_N01                31      /* REV03 DV*/
#define CONFIG_NOWPLUS_REV03_N02                32      /* REV03 AR*/
#define CONFIG_NOWPLUS_REV04                    40      /* REV04 */
#define CONFIG_NOWPLUS_REV05                    50      /* REV05 */
#define CONFIG_NOWPLUS_REV06                    60      /* REV06 */
#define CONFIG_NOWPLUS_REV07                    70      /* REV07 */
#define CONFIG_NOWPLUS_REV08                    80      /* REV08 */
#define CONFIG_NOWPLUS_REV09                    90      /* REV09 */
#define CONFIG_NOWPLUS_REV10                    100     /* REV10 */

#define CONFIG_NOWPLUS_REV                      CONFIG_NOWPLUS_HW_REV

enum s5ka3dfx_op_mode {
  S5KA3DFX_MODE_VIDEO = 0,
  S5KA3DFX_MODE_IMAGE = 1,
};


/**
 * struct s5ka3dfx_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct s5ka3dfx_platform_data {
  int (*power_set)(enum v4l2_power power);
  int (*ifparm)(struct v4l2_ifparm *p);
  int (*priv_data_set)(void *);
};

/**   
 * struct s5ka3dfx_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: s5ka3dfx chip version
 * @fps: frames per second value   
 */
struct s5ka3dfx_sensor {
  const struct s5ka3dfx_platform_data *pdata;
  struct mutex s5ka3dfx_lock;
  struct v4l2_int_device *v4l2_int_device;
  struct i2c_client *i2c_client;
  struct v4l2_pix_format pix;
  struct v4l2_fract timeperframe;
  u32 state;
  int scaler;
  int ver;
  u8 mode;  
  u8 fps;
  u8 preview_size;
  u8 capture_size;
  u8 detect;
  s32 zoom;
  u8 effect;
  u8 ev;
  u8 contrast;
  u8 wb;
  u8 pretty;
  u8 flip;
  int check_dataline;
};

/* delay define */
#define WAIT_CAM_AEAWB           100

/* State */
#define S5KA3DFX_STATE_PREVIEW	 0x0000	/*  preview state */
#define S5KA3DFX_STATE_CAPTURE	 0x0001	/*  capture state */
#define S5KA3DFX_STATE_INVALID   0x0002	/*  invalid state */

/* Mode */
#define S5KA3DFX_MODE_CAMERA     1
#define S5KA3DFX_MODE_CAMCORDER  2
#define S5KA3DFX_MODE_VT         3

/* Preview Size */
#define S5KA3DFX_PREVIEW_SIZE_640_480    0
#define S5KA3DFX_PREVIEW_SIZE_320_240    1
#define S5KA3DFX_PREVIEW_SIZE_352_288    2
#define S5KA3DFX_PREVIEW_SIZE_176_144    3

/* Image Size */
#define S5KA3DFX_IMAGE_SIZE_640_480    0
#define S5KA3DFX_IMAGE_SIZE_320_240    1
#define S5KA3DFX_IMAGE_SIZE_352_288    2
#define S5KA3DFX_IMAGE_SIZE_176_144    3

/* Image Effect */
#define S5KA3DFX_EFFECT_OFF      1
#define S5KA3DFX_EFFECT_SHARPEN  2
#define S5KA3DFX_EFFECT_PURPLE   3
#define S5KA3DFX_EFFECT_NEGATIVE 4
#define S5KA3DFX_EFFECT_SEPIA    5
#define S5KA3DFX_EFFECT_AQUA     6
#define S5KA3DFX_EFFECT_GREEN    7
#define S5KA3DFX_EFFECT_BLUE     8
#define S5KA3DFX_EFFECT_PINK     9
#define S5KA3DFX_EFFECT_YELLOW   10
#define S5KA3DFX_EFFECT_GREY     11
#define S5KA3DFX_EFFECT_RED      12
#define S5KA3DFX_EFFECT_BW       13
#define S5KA3DFX_EFFECT_ANTIQUE  14

/* EV */
#define S5KA3DFX_EV_MINUS_2P0    1
#define S5KA3DFX_EV_MINUS_1P5    2
#define S5KA3DFX_EV_MINUS_1P0    3
#define S5KA3DFX_EV_MINUS_0P5    4
#define S5KA3DFX_EV_DEFAULT      5
#define S5KA3DFX_EV_PLUS_0P5     6
#define S5KA3DFX_EV_PLUS_1P0     7
#define S5KA3DFX_EV_PLUS_1P5     8
#define S5KA3DFX_EV_PLUS_2P0     9

/* Contrast */
#define S5KA3DFX_CONTRAST_MINUS_5      10 //1
#define S5KA3DFX_CONTRAST_MINUS_4      11 //2
#define S5KA3DFX_CONTRAST_MINUS_3      1 //3
#define S5KA3DFX_CONTRAST_MINUS_2      2 //4
#define S5KA3DFX_CONTRAST_MINUS_1      3 //5
#define S5KA3DFX_CONTRAST_DEFAULT      4 //6
#define S5KA3DFX_CONTRAST_PLUS_1       5 //7
#define S5KA3DFX_CONTRAST_PLUS_2       6 //8
#define S5KA3DFX_CONTRAST_PLUS_3       7 //9
#define S5KA3DFX_CONTRAST_PLUS_4       12 //10
#define S5KA3DFX_CONTRAST_PLUS_5       13 //11

/* White Balance */
#define S5KA3DFX_WB_AUTO               1
#define S5KA3DFX_WB_DAYLIGHT           2
#define S5KA3DFX_WB_CLOUDY             3
#define S5KA3DFX_WB_INCANDESCENT       4
#define S5KA3DFX_WB_FLUORESCENT        5

/* Flip */
#define S5KA3DFX_FLIP_NONE             1
#define S5KA3DFX_FLIP_MIRROR           2
#define S5KA3DFX_FLIP_WATER            3
#define S5KA3DFX_FLIP_WATER_MIRROR     4

/* Pretty */
#define S5KA3DFX_PRETTY_NONE           1
#define S5KA3DFX_PRETTY_LEVEL1         2
#define S5KA3DFX_PRETTY_LEVEL2         3
#define S5KA3DFX_PRETTY_LEVEL3         4

/* Auto Exposure & Auto White Balance */
#define S5KA3DFX_AE_LOCK_AWB_LOCK      1
#define S5KA3DFX_AE_LOCK_AWB_UNLOCK    2
#define S5KA3DFX_AE_UNLOCK_AWB_LOCK    3
#define S5KA3DFX_AE_UNLOCK_AWB_UNLOCK  4

/* Digital Zoom */
#define S5KA3DFX_ZOOM_1P00X            1
#define S5KA3DFX_ZOOM_1P25X            2
#define S5KA3DFX_ZOOM_1P50X            3
#define S5KA3DFX_ZOOM_1P75X            4
#define S5KA3DFX_ZOOM_2P00X            5
#define S5KA3DFX_ZOOM_2P25X            6
#define S5KA3DFX_ZOOM_2P50X            7
#define S5KA3DFX_ZOOM_2P75X            8
#define S5KA3DFX_ZOOM_3P00X            9
#define S5KA3DFX_ZOOM_3P25X            10
#define S5KA3DFX_ZOOM_3P50X            11
#define S5KA3DFX_ZOOM_3P75X            12
#define S5KA3DFX_ZOOM_4P00X            13

/* JPEG Quality */
#define S5KA3DFX_JPEG_SUPERFINE        1
#define S5KA3DFX_JPEG_FINE             2
#define S5KA3DFX_JPEG_NORMAL           3
#define S5KA3DFX_JPEG_ECONOMY          4

/* Preview Sizes */
struct s5ka3dfx_preview_size {
  unsigned int width;
  unsigned int height;
};

const static struct s5ka3dfx_preview_size s5ka3dfx_preview_sizes[] = {
  {640,480},   
  {320,240},   
  {352,288},   
  {176,144},
};

/* Image Sizes */
struct s5ka3dfx_capture_size {
  unsigned int width;
  unsigned int height;
};


const static struct s5ka3dfx_capture_size s5ka3dfx_image_sizes[] = {
  {640,480}, 
  {320,240}, 
  {352,288},  
  {176,144},
};

#endif /* ifndef S5KA3DFX_H */
