/*
 * drivers/media/video/s5k5ccgx.h
 *
 * Register definitions for the NEC S5K5CCGX CameraChip.
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
 * modules/camera/s5k5ccgx.h
 *
 * S5K5CCGX sensor driver header file
 *
 * Modified by paladin in Samsung Electronics
 */

#include<linux/videodev2.h>


typedef struct s5k5ccgx_reg {
	unsigned short addr;
	unsigned short val;
}__attribute__ ((packed)) s5k5ccgx_short_t;

#ifndef S5K5CCGX_H
#define S5K5CCGX_H
#define S5K5CCGX_TOUCH_AF 
#define CAM_S5K5CCGX_DBG_MSG            0
#define CAM_S5K5CCGX_I2C_DBG_MSG        0
#define CAM_S5K5CCGX_TUNE               0
#define S5K5CCGX_DRIVER_NAME         "s5k5ccgx"
#define S5K5CCGX_MOD_NAME               "S5K5CCGX: "
#if ( defined( CONFIG_MACH_SAMSUNG_P1LITE ) && ( CONFIG_SAMSUNG_REL_HW_REV <= 1 ) ) 
#define S5K5CCGX_USE_GPIO_I2C               1
#endif
#define S5K5CCGX_FLASH_SUPPORT
#define IN_IMAGE                     0
#define IN_SDCARD                    1

#define S5K5CCGX_NOT_FW_UPDATE_OPERATION 0xFF

#define S5K5CCGX_FW_MINOR_VERSION    0x0F
#define S5K5CCGX_FW_MAJOR_VERSION    0x05
#define S5K5CCGX_PRM_MINOR_VERSION   0x38
#define S5K5CCGX_PRM_MAJOR_VERSION   0x07

#define S5K5CCGX_THUMBNAIL_OFFSET    0x271000
#define S5K5CCGX_YUV_OFFSET          0x280A00

#define S5K5CCGX_I2C_ADDR         0x78>>1
#define S5K5CCGX_I2C_RETRY           10
#define S5K5CCGX_XCLK                24000000      //have to be fixed

#define SENSOR_DETECTED           1
#define SENSOR_NOT_DETECTED       0

#define OMAP3430_GPIO_CAMERA_EN           152
#define OMAP3430_GPIO_CAMERA_RST          98
#define OMAP3430_GPIO_CAMERA_MEGA_EN      153

#define OMAP3430_GPIO_VGA_RST             64 
#define OMAP3430_GPIO_VGA_STBY            101

#define OMAP3430_GPIO_FLASH_EN1           156
#define OMAP3430_GPIO_FLASH_EN2           158

//#define CONFIG_NOWPLUS_HW_REV CONFIG_NOWPLUS_REV08 // think so!

enum s5k5ccgx_op_mode {
  S5K5CCGX_MODE_VIDEO = 0,
  S5K5CCGX_MODE_IMAGE = 1,
};

typedef enum {
  AF_LENS_UNFOCUSED_STOP     = 0x00, // 0x00(&0x0F) 
  AF_LENS_UNFOCUSED_MOVING   = 0x00, // 0x01(&0x0F)   
  AF_LENZ_FOCUSED_STOP       = 0x02, // 0x02(&0x0F)
  AF_LENZ_INVALID_STOP       = 0x04, // 0x04(&0x0F)
  AF_LENS_INVALID_MOVING     = 0x05, // 0x05(&0x0F)     
} AFZOOM_Status;


/**
 * struct s5k5ccgx_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct s5k5ccgx_platform_data {
  int (*power_set)(enum v4l2_power power);
  int (*ifparm)(struct v4l2_ifparm *p);
  int (*priv_data_set)(void *);
};

struct s5k5ccgx_version {
	unsigned int major;
	unsigned int minor;
};

struct s5k5ccgx_date_info {
	unsigned int year;
	unsigned int month;
	unsigned int date;
};

struct s5k5ccgx_sensor_maker{
	unsigned int maker;
	unsigned int optical;
};

struct s5k5ccgx_version_af{
	unsigned int low;
	unsigned int high;
};

struct s5k5ccgx_gamma{
	unsigned int rg_low;
	unsigned int rg_high;
	unsigned int bg_low;
	unsigned int bg_high;	
};

struct s5k5ccgx_regset {
	u32 size;
	u8 *data;
};

struct s5k5ccgx_position {	int x;	int y;} ; 

enum s5k5ccgx_runmode {
	S5K5CCGX_RUNMODE_NOTREADY,
	S5K5CCGX_RUNMODE_IDLE, 
	S5K5CCGX_RUNMODE_RUNNING, 
};
/**   
 * struct s5k5ccgx_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: s5k5ccgx chip version
 * @fps: frames per second value   
 */
struct s5k5ccgx_sensor {
  const struct s5k5ccgx_platform_data *pdata;
  struct mutex s5k5ccgx_previewlock;
  struct mutex s5k5ccgx_capturelock;
  struct mutex s5k5ccgx_setlock;
  struct v4l2_int_device *v4l2_int_device;
  struct i2c_client *i2c_client;
  struct v4l2_pix_format pix;
  struct v4l2_fract timeperframe;
  struct s5k5ccgx_version fw;
  struct s5k5ccgx_version prm;
  struct s5k5ccgx_date_info dateinfo;  
  struct s5k5ccgx_sensor_maker sensor_info;
  struct s5k5ccgx_version_af af_info;
  struct s5k5ccgx_gamma gamma;  
  struct s5k5ccgx_version main_sw_fw;
  struct s5k5ccgx_version main_sw_prm;
  struct s5k5ccgx_date_info main_sw_dateinfo;
  struct s5k5ccgx_position position;
  enum s5k5ccgx_runmode runmode;  
  int check_dataline;  
  int sensor_version;
  unsigned int fw_dump_size;    
  u32 state;
  u8 mode;
  u8 fps;
  u16 bv;
  u8 preview_size;
  u8 capture_size;
  u8 focus_mode;
  u8 detect;
  u8 effect;
  u8 iso;
  u8 photometry;
  u8 ev;
  u8 wdr;
  u8 contrast;
  u8 saturation;
  u8 sharpness;
  u8 wb;
  u8 isc;
  u8 scene;
  u8 aewb;
  u8 antishake;
  u8 flash_capture;
  u8 flash_movie;
  u8 jpeg_quality;
  s32 zoom;
  u32 thumb_offset;
  u32 yuv_offset;
  u32 jpeg_capture_w;
  u32 jpeg_capture_h;
  bool flashstate;  
  bool flash_gpio_state;    
  int set_vhflip;  
  int scenemode;  
};

#define MOVIEMODE_FLASH 	3
#define FLASHMODE_AUTO	2
#define FLASHMODE_ON	3
#define FLASHMODE_OFF	1

/* delay define */
#define WAIT_CAM_AEAWB        100

/* State */
#define S5K5CCGX_STATE_PREVIEW	  0x0000	/*  preview state */
#define S5K5CCGX_STATE_CAPTURE	  0x0001	/*  capture state */
#define S5K5CCGX_STATE_INVALID	  0x0002	/*  invalid state */

/* Mode */
#define S5K5CCGX_MODE_CAMERA     1
#define S5K5CCGX_MODE_CAMCORDER  2
#define S5K5CCGX_MODE_VT         3

/* Preview Size */
#define S5K5CCGX_PREVIEW_SIZE_1280_720   0
#define S5K5CCGX_PREVIEW_SIZE_1024_600   1
#define S5K5CCGX_PREVIEW_SIZE_800_600    2
#define S5K5CCGX_PREVIEW_SIZE_800_480    3
#define S5K5CCGX_PREVIEW_SIZE_720_480    4
#define S5K5CCGX_PREVIEW_SIZE_640_480    5
#define S5K5CCGX_PREVIEW_SIZE_400_240    6
#define S5K5CCGX_PREVIEW_SIZE_352_288    7
#define S5K5CCGX_PREVIEW_SIZE_320_240    8
#define S5K5CCGX_PREVIEW_SIZE_240_320	 9
#define S5K5CCGX_PREVIEW_SIZE_200_120    10
#define S5K5CCGX_PREVIEW_SIZE_176_144    11
#define S5K5CCGX_PREVIEW_SIZE_144_176    12
#define S5K5CCGX_PREVIEW_SIZE_160_120    13

/* Image Size */
#define S5K5CCGX_IMAGE_SIZE_3264_2448  0
#define S5K5CCGX_IMAGE_SIZE_2560_1920  1
#define S5K5CCGX_IMAGE_SIZE_2560_1536  2
#define S5K5CCGX_IMAGE_SIZE_2560_1440  3
#define S5K5CCGX_IMAGE_SIZE_2048_1536  4
#define S5K5CCGX_IMAGE_SIZE_2048_1232  5
#define S5K5CCGX_IMAGE_SIZE_1920_1080  6
#define S5K5CCGX_IMAGE_SIZE_1600_1200  7
#define S5K5CCGX_IMAGE_SIZE_1600_960   8
#define S5K5CCGX_IMAGE_SIZE_1280_960   9
#define S5K5CCGX_IMAGE_SIZE_1280_768   10
#define S5K5CCGX_IMAGE_SIZE_1280_720   11
#define S5K5CCGX_IMAGE_SIZE_1024_768   12
#define S5K5CCGX_IMAGE_SIZE_1024_600   13
#define S5K5CCGX_IMAGE_SIZE_800_600    14
#define S5K5CCGX_IMAGE_SIZE_800_480    15
#define S5K5CCGX_IMAGE_SIZE_720_480    16
#define S5K5CCGX_IMAGE_SIZE_640_480    17
#define S5K5CCGX_IMAGE_SIZE_400_240    18
#define S5K5CCGX_IMAGE_SIZE_352_288    19
#define S5K5CCGX_IMAGE_SIZE_320_240    20
#define S5K5CCGX_IMAGE_SIZE_200_120    21
#define S5K5CCGX_IMAGE_SIZE_176_144    22
#define S5K5CCGX_IMAGE_SIZE_160_120    23

  /* Image Effect */
#define S5K5CCGX_EFFECT_OFF      1
#define S5K5CCGX_EFFECT_SHARPEN  2
#define S5K5CCGX_EFFECT_PURPLE   3
#define S5K5CCGX_EFFECT_NEGATIVE 4
#define S5K5CCGX_EFFECT_SEPIA    5
#define S5K5CCGX_EFFECT_AQUA     6
#define S5K5CCGX_EFFECT_GREEN    7
#define S5K5CCGX_EFFECT_BLUE     8
#define S5K5CCGX_EFFECT_PINK     9
#define S5K5CCGX_EFFECT_YELLOW   10
#define S5K5CCGX_EFFECT_BW   	 11
#define S5K5CCGX_EFFECT_RED      12
#define S5K5CCGX_EFFECT_GREY     13
#define S5K5CCGX_EFFECT_ANTIQUE  14

/* ISO */
#define S5K5CCGX_ISO_AUTO        1
#define S5K5CCGX_ISO_50          2
#define S5K5CCGX_ISO_100         3
#define S5K5CCGX_ISO_200         4
#define S5K5CCGX_ISO_400         5
#define S5K5CCGX_ISO_800         6
#define S5K5CCGX_ISO_1600        7

/* Photometry */
#define S5K5CCGX_PHOTOMETRY_MATRIX   1
#define S5K5CCGX_PHOTOMETRY_CENTER   2
#define S5K5CCGX_PHOTOMETRY_SPOT     3

/* EV */
#define S5K5CCGX_EV_MINUS_2P0    1
#define S5K5CCGX_EV_MINUS_1P5    2
#define S5K5CCGX_EV_MINUS_1P0    3
#define S5K5CCGX_EV_MINUS_0P5    4
#define S5K5CCGX_EV_DEFAULT      5
#define S5K5CCGX_EV_PLUS_0P5     6
#define S5K5CCGX_EV_PLUS_1P0     7
#define S5K5CCGX_EV_PLUS_1P5     8
#define S5K5CCGX_EV_PLUS_2P0     9

/* WDR */
#define S5K5CCGX_WDR_OFF         1
#define S5K5CCGX_WDR_ON          2
#define S5K5CCGX_WDR_AUTO        3

/* Contrast */
#define S5K5CCGX_CONTRAST_MINUS_3      1
#define S5K5CCGX_CONTRAST_MINUS_2      2
#define S5K5CCGX_CONTRAST_MINUS_1      3
#define S5K5CCGX_CONTRAST_DEFAULT      4
#define S5K5CCGX_CONTRAST_PLUS_1       5
#define S5K5CCGX_CONTRAST_PLUS_2       6
#define S5K5CCGX_CONTRAST_PLUS_3       7

/* Saturation */
#define S5K5CCGX_SATURATION_MINUS_3    1
#define S5K5CCGX_SATURATION_MINUS_2    2
#define S5K5CCGX_SATURATION_MINUS_1    3
#define S5K5CCGX_SATURATION_DEFAULT    4
#define S5K5CCGX_SATURATION_PLUS_1     5
#define S5K5CCGX_SATURATION_PLUS_2     6
#define S5K5CCGX_SATURATION_PLUS_3     7

/* Sharpness */
#define S5K5CCGX_SHARPNESS_MINUS_3     1
#define S5K5CCGX_SHARPNESS_MINUS_2     2
#define S5K5CCGX_SHARPNESS_MINUS_1     3
#define S5K5CCGX_SHARPNESS_DEFAULT     4
#define S5K5CCGX_SHARPNESS_PLUS_1      5
#define S5K5CCGX_SHARPNESS_PLUS_2      6
#define S5K5CCGX_SHARPNESS_PLUS_3      7

/* White Balance */
#define S5K5CCGX_WB_AUTO               1
#define S5K5CCGX_WB_DAYLIGHT           2
#define S5K5CCGX_WB_CLOUDY             3
#define S5K5CCGX_WB_INCANDESCENT       4
#define S5K5CCGX_WB_FLUORESCENT        5

/* Image Stabilization */
#define S5K5CCGX_ISC_STILL_OFF         1
#define S5K5CCGX_ISC_STILL_ON          2
#define S5K5CCGX_ISC_STILL_AUTO        3
#define S5K5CCGX_ISC_MOVIE_ON          4

/* Scene Mode */
#define S5K5CCGX_SCENE_OFF             1
#define S5K5CCGX_SCENE_ASD             2
#define S5K5CCGX_SCENE_SUNSET          3
#define S5K5CCGX_SCENE_DAWN            4
#define S5K5CCGX_SCENE_CANDLELIGHT     5
#define S5K5CCGX_SCENE_BEACH_SNOW      6
#define S5K5CCGX_SCENE_AGAINST_LIGHT   7
#define S5K5CCGX_SCENE_TEXT            8
#define S5K5CCGX_SCENE_NIGHTSHOT       9
#define S5K5CCGX_SCENE_LANDSCAPE       10
#define S5K5CCGX_SCENE_FIREWORKS       11
#define S5K5CCGX_SCENE_PORTRAIT        12
#define S5K5CCGX_SCENE_FALLCOLOR       13
#define S5K5CCGX_SCENE_INDOORS         14
#define S5K5CCGX_SCENE_SPORTS          15

/* Auto Exposure & Auto White Balance */
#define S5K5CCGX_AE_LOCK_AWB_LOCK      1
#define S5K5CCGX_AE_LOCK_AWB_UNLOCK    2
#define S5K5CCGX_AE_UNLOCK_AWB_LOCK    3
#define S5K5CCGX_AE_UNLOCK_AWB_UNLOCK  4

/* Anti-Shake */
#define S5K5CCGX_ANTI_SHAKE_OFF        1
#define S5K5CCGX_ANTI_SHAKE_ON         2

/* Flash Setting */
#define S5K5CCGX_FLASH_CAPTURE_OFF     1
#define S5K5CCGX_FLASH_CAPTURE_ON      2
#define S5K5CCGX_FLASH_CAPTURE_AUTO    3

#define S5K5CCGX_FLASH_MOVIE_OFF       1
#define S5K5CCGX_FLASH_MOVIE_ON        2

/* Focus Mode */
//#define FEATURE_TOUCH_AF
#define S5K5CCGX_AF_INIT_NORMAL        1
#define S5K5CCGX_AF_INIT_MACRO         2
#define S5K5CCGX_AF_INIT_FACE          3
#ifdef S5K5CCGX_TOUCH_AF   
#define S5K5CCGX_AF_INIT_TOUCH         4
#endif

/* Focust start/stop */
#define S5K5CCGX_AF_START              1
#define S5K5CCGX_AF_STOP               2

/* Auto Focus Status */
#define S5K5CCGX_AF_STATUS_PROGRESS    1
#define S5K5CCGX_AF_STATUS_SUCCESS     2
#define S5K5CCGX_AF_STATUS_FAIL        3

/* Digital Zoom */
#define S5K5CCGX_ZOOM_DEFAULT          0
#define S5K5CCGX_ZOOM_1P00X            1
#define S5K5CCGX_ZOOM_1P25X            2
#define S5K5CCGX_ZOOM_1P50X            3
#define S5K5CCGX_ZOOM_1P75X            4
#define S5K5CCGX_ZOOM_2P00X            5
#define S5K5CCGX_ZOOM_2P25X            6
#define S5K5CCGX_ZOOM_2P50X            7
#define S5K5CCGX_ZOOM_2P75X            8
#define S5K5CCGX_ZOOM_3P00X            9
#define S5K5CCGX_ZOOM_3P25X            10
#define S5K5CCGX_ZOOM_3P50X            11
#define S5K5CCGX_ZOOM_3P75X            12
#define S5K5CCGX_ZOOM_4P00X            13

/* JPEG Quality */
#define S5K5CCGX_JPEG_SUPERFINE        1
#define S5K5CCGX_JPEG_FINE             2
#define S5K5CCGX_JPEG_NORMAL           3
#define S5K5CCGX_JPEG_ECONOMY          4

static const u8 BatchReflectionRequest_list[] = {0x01, 0x00};
static const u8 BatchReflectionCheck_list[]   = {0x02};

/* Change YUV order & Modify module current */
static const u8 ConfigSensorYCOsetting_list[] = {0x03,0x00};


//==========================================================
//flash set & flash delay
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_FLASH_SET[] = 
{
// flash�� ���� �߰� ����
// 0xFCFC, 0xD000
// 0x0028, 0x7000
// 0x002A, 0x025A

// flash on -> flash setting -> Delay -> capture -> flash off
{0xffff, 0x0000}, //0md delay
};
#define S5K5CCGX_FLASH_SET_INDEX (sizeof(S5K5CCGX_FLASH_SET) / sizeof(S5K5CCGX_FLASH_SET[0]))

//==========================================================
//INIT SETTING 
//==========================================================

 static const s5k5ccgx_short_t S5K5CCGX_INIT_START[]=
{
{0xFCFC, 0xD000},
{0x0010, 0x0001},
{0x1030, 0x0000},
{0x0014, 0x0001},
};
  #define S5K5CCGX_INIT_START_INDEX (sizeof(S5K5CCGX_INIT_START) / sizeof(S5K5CCGX_INIT_START[0]))

static const s5k5ccgx_short_t S5K5CCGX_INIT_SET[]=
{
//****************************************/
// Start of Patch data
{0x0028, 0x7000},
{0x002A, 0x352C},
{0x0F12, 0xB510},    // 7000352C 
{0x0F12, 0x4A2C},    // 7000352E 
{0x0F12, 0x213F},    // 70003530 
{0x0F12, 0x482C},    // 70003532 
{0x0F12, 0x4B2C},    // 70003534 
{0x0F12, 0x2400},    // 70003536 
{0x0F12, 0x801C},    // 70003538 
{0x0F12, 0xC004},    // 7000353A 
{0x0F12, 0x6001},    // 7000353C 
{0x0F12, 0x492B},    // 7000353E 
{0x0F12, 0x482B},    // 70003540 
{0x0F12, 0xF000},    // 70003542 
{0x0F12, 0xFBE9},    // 70003544 
{0x0F12, 0x2115},    // 70003546 
{0x0F12, 0x482A},    // 70003548 
{0x0F12, 0x01C9},    // 7000354A 
{0x0F12, 0xF000},    // 7000354C 
{0x0F12, 0xF88E},    // 7000354E 
{0x0F12, 0x4828},    // 70003550 
{0x0F12, 0x210B},    // 70003552 
{0x0F12, 0x0189},    // 70003554 
{0x0F12, 0x3018},    // 70003556 
{0x0F12, 0xF000},    // 70003558 
{0x0F12, 0xF888},    // 7000355A 
{0x0F12, 0x4825},    // 7000355C 
{0x0F12, 0x4926},    // 7000355E 
{0x0F12, 0x300C},    // 70003560 
{0x0F12, 0xF000},    // 70003562 
{0x0F12, 0xF883},    // 70003564 
{0x0F12, 0x4823},    // 70003566 
{0x0F12, 0x4924},    // 70003568 
{0x0F12, 0x3010},    // 7000356A 
{0x0F12, 0xF000},    // 7000356C 
{0x0F12, 0xF87E},    // 7000356E 
{0x0F12, 0x4923},    // 70003570 
{0x0F12, 0x4824},    // 70003572 
{0x0F12, 0xF000},    // 70003574 
{0x0F12, 0xFBD0},    // 70003576 
{0x0F12, 0x4923},    // 70003578 
{0x0F12, 0x4824},    // 7000357A 
{0x0F12, 0xF000},    // 7000357C 
{0x0F12, 0xFBCC},    // 7000357E 
{0x0F12, 0x4923},    // 70003580 
{0x0F12, 0x4824},    // 70003582 
{0x0F12, 0xF000},    // 70003584 
{0x0F12, 0xFBC8},    // 70003586 
{0x0F12, 0x4923},    // 70003588 
{0x0F12, 0x4824},    // 7000358A 
{0x0F12, 0xF000},    // 7000358C 
{0x0F12, 0xFBC4},    // 7000358E 
{0x0F12, 0x4923},    // 70003590 
{0x0F12, 0x4824},    // 70003592 
{0x0F12, 0xF000},    // 70003594 
{0x0F12, 0xFBC0},    // 70003596 
{0x0F12, 0x4823},    // 70003598 
{0x0F12, 0x4924},    // 7000359A 
{0x0F12, 0x6408},    // 7000359C 
{0x0F12, 0x4924},    // 7000359E 
{0x0F12, 0x4824},    // 700035A0 
{0x0F12, 0xF000},    // 700035A2 
{0x0F12, 0xFBB9},    // 700035A4 
{0x0F12, 0x4924},    // 700035A6 
{0x0F12, 0x4824},    // 700035A8 
{0x0F12, 0xF000},    // 700035AA 
{0x0F12, 0xFBB5},    // 700035AC 
{0x0F12, 0x4924},    // 700035AE 
{0x0F12, 0x4824},    // 700035B0 
{0x0F12, 0xF000},    // 700035B2 
{0x0F12, 0xFBB1},    // 700035B4 
{0x0F12, 0x4924},    // 700035B6 
{0x0F12, 0x4824},    // 700035B8 
{0x0F12, 0xF000},    // 700035BA 
{0x0F12, 0xFBAD},    // 700035BC 
{0x0F12, 0x4924},    // 700035BE 
{0x0F12, 0x4824},    // 700035C0 
{0x0F12, 0xF000},    // 700035C2 
{0x0F12, 0xFBA9},    // 700035C4 
{0x0F12, 0x4824},    // 700035C6 
{0x0F12, 0x8104},    // 700035C8 
{0x0F12, 0x4924},    // 700035CA 
{0x0F12, 0x4824},    // 700035CC 
{0x0F12, 0xF000},    // 700035CE 
{0x0F12, 0xFBA3},    // 700035D0 
{0x0F12, 0x4924},    // 700035D2 
{0x0F12, 0x4824},    // 700035D4 
{0x0F12, 0xF000},    // 700035D6 
{0x0F12, 0xFB9F},    // 700035D8 
{0x0F12, 0xBC10},    // 700035DA 
{0x0F12, 0xBC08},    // 700035DC 
{0x0F12, 0x4718},    // 700035DE 
{0x0F12, 0x00BA},    // 700035E0 
{0x0F12, 0x5CC1},    // 700035E2 
{0x0F12, 0x1C08},    // 700035E4 
{0x0F12, 0x7000},    // 700035E6 
{0x0F12, 0x3290},    // 700035E8 
{0x0F12, 0x7000},    // 700035EA 
{0x0F12, 0x368B},    // 700035EC 
{0x0F12, 0x7000},    // 700035EE 
{0x0F12, 0xD9E7},    // 700035F0 
{0x0F12, 0x0000},    // 700035F2 
{0x0F12, 0x6FC0},    // 700035F4 
{0x0F12, 0x0000},    // 700035F6 
{0x0F12, 0x0A91},    // 700035F8 
{0x0F12, 0x0000},    // 700035FA 
{0x0F12, 0x02C9},    // 700035FC 
{0x0F12, 0x0000},    // 700035FE 
{0x0F12, 0x36C3},    // 70003600 
{0x0F12, 0x7000},    // 70003602 
{0x0F12, 0xA607},    // 70003604 
{0x0F12, 0x0000},    // 70003606 
{0x0F12, 0x3733},    // 70003608 
{0x0F12, 0x7000},    // 7000360A 
{0x0F12, 0x7C0D},    // 7000360C 
{0x0F12, 0x0000},    // 7000360E 
{0x0F12, 0x374F},    // 70003610 
{0x0F12, 0x7000},    // 70003612 
{0x0F12, 0x7C2B},    // 70003614 
{0x0F12, 0x0000},    // 70003616 
{0x0F12, 0x376B},    // 70003618 
{0x0F12, 0x7000},    // 7000361A 
{0x0F12, 0x9E89},    // 7000361C 
{0x0F12, 0x0000},    // 7000361E 
{0x0F12, 0x394F},    // 70003620 
{0x0F12, 0x7000},    // 70003622 
{0x0F12, 0x395D},    // 70003624 
{0x0F12, 0x0000},    // 70003626 
{0x0F12, 0x39DB},    // 70003628 
{0x0F12, 0x7000},    // 7000362A 
{0x0F12, 0x0000},    // 7000362C 
{0x0F12, 0x7000},    // 7000362E 
{0x0F12, 0x3B01},    // 70003630 
{0x0F12, 0x7000},    // 70003632 
{0x0F12, 0xF903},    // 70003634 
{0x0F12, 0x0000},    // 70003636 
{0x0F12, 0x37B3},    // 70003638 
{0x0F12, 0x7000},    // 7000363A 
{0x0F12, 0x495F},    // 7000363C 
{0x0F12, 0x0000},    // 7000363E 
{0x0F12, 0x380D},    // 70003640 
{0x0F12, 0x7000},    // 70003642 
{0x0F12, 0xE421},    // 70003644 
{0x0F12, 0x0000},    // 70003646 
{0x0F12, 0x38BB},    // 70003648 
{0x0F12, 0x7000},    // 7000364A 
{0x0F12, 0x216D},    // 7000364C 
{0x0F12, 0x0000},    // 7000364E 
{0x0F12, 0x392F},    // 70003650 
{0x0F12, 0x7000},    // 70003652 
{0x0F12, 0x0179},    // 70003654 
{0x0F12, 0x0001},    // 70003656 
{0x0F12, 0x3FC8},    // 70003658 
{0x0F12, 0x7000},    // 7000365A 
{0x0F12, 0x3CE3},    // 7000365C 
{0x0F12, 0x7000},    // 7000365E 
{0x0F12, 0x04C9},    // 70003660 
{0x0F12, 0x0000},    // 70003662 
{0x0F12, 0x3C2F},    // 70003664 
{0x0F12, 0x7000},    // 70003666 
{0x0F12, 0x5027},    // 70003668 
{0x0F12, 0x0000},    // 7000366A 
{0x0F12, 0xB570},    // 7000366C 
{0x0F12, 0x000D},    // 7000366E 
{0x0F12, 0x4CFF},    // 70003670 
{0x0F12, 0x8821},    // 70003672 
{0x0F12, 0xF000},    // 70003674 
{0x0F12, 0xFB58},    // 70003676 
{0x0F12, 0x8820},    // 70003678 
{0x0F12, 0x4AFE},    // 7000367A 
{0x0F12, 0x0081},    // 7000367C 
{0x0F12, 0x5055},    // 7000367E 
{0x0F12, 0x1C40},    // 70003680 
{0x0F12, 0x8020},    // 70003682 
{0x0F12, 0xBC70},    // 70003684 
{0x0F12, 0xBC08},    // 70003686 
{0x0F12, 0x4718},    // 70003688 
{0x0F12, 0x6801},    // 7000368A 
{0x0F12, 0x0409},    // 7000368C 
{0x0F12, 0x0C09},    // 7000368E 
{0x0F12, 0x6840},    // 70003690 
{0x0F12, 0x0400},    // 70003692 
{0x0F12, 0x0C00},    // 70003694 
{0x0F12, 0x4AF8},    // 70003696 
{0x0F12, 0x8992},    // 70003698 
{0x0F12, 0x2A00},    // 7000369A 
{0x0F12, 0xD00D},    // 7000369C 
{0x0F12, 0x2300},    // 7000369E 
{0x0F12, 0x1A80},    // 700036A0 
{0x0F12, 0xD400},    // 700036A2 
{0x0F12, 0x0003},    // 700036A4 
{0x0F12, 0x0418},    // 700036A6 
{0x0F12, 0x0C00},    // 700036A8 
{0x0F12, 0x4BF4},    // 700036AA 
{0x0F12, 0x1851},    // 700036AC 
{0x0F12, 0x891B},    // 700036AE 
{0x0F12, 0x428B},    // 700036B0 
{0x0F12, 0xD300},    // 700036B2 
{0x0F12, 0x000B},    // 700036B4 
{0x0F12, 0x0419},    // 700036B6 
{0x0F12, 0x0C09},    // 700036B8 
{0x0F12, 0x4AF1},    // 700036BA 
{0x0F12, 0x8151},    // 700036BC 
{0x0F12, 0x8190},    // 700036BE 
{0x0F12, 0x4770},    // 700036C0 
{0x0F12, 0xB510},    // 700036C2 
{0x0F12, 0x48EF},    // 700036C4 
{0x0F12, 0x4CF0},    // 700036C6 
{0x0F12, 0x88C1},    // 700036C8 
{0x0F12, 0x8061},    // 700036CA 
{0x0F12, 0x2101},    // 700036CC 
{0x0F12, 0x8021},    // 700036CE 
{0x0F12, 0x8840},    // 700036D0 
{0x0F12, 0xF000},    // 700036D2 
{0x0F12, 0xFB31},    // 700036D4 
{0x0F12, 0x88E0},    // 700036D6 
{0x0F12, 0x4AEC},    // 700036D8 
{0x0F12, 0x2800},    // 700036DA 
{0x0F12, 0xD003},    // 700036DC 
{0x0F12, 0x49EC},    // 700036DE 
{0x0F12, 0x8849},    // 700036E0 
{0x0F12, 0x2900},    // 700036E2 
{0x0F12, 0xD009},    // 700036E4 
{0x0F12, 0x2001},    // 700036E6 
{0x0F12, 0x03C0},    // 700036E8 
{0x0F12, 0x8050},    // 700036EA 
{0x0F12, 0x80D0},    // 700036EC 
{0x0F12, 0x2000},    // 700036EE 
{0x0F12, 0x8090},    // 700036F0 
{0x0F12, 0x8110},    // 700036F2 
{0x0F12, 0xBC10},    // 700036F4 
{0x0F12, 0xBC08},    // 700036F6 
{0x0F12, 0x4718},    // 700036F8 
{0x0F12, 0x8050},    // 700036FA 
{0x0F12, 0x8920},    // 700036FC 
{0x0F12, 0x80D0},    // 700036FE 
{0x0F12, 0x8960},    // 70003700 
{0x0F12, 0x0400},    // 70003702 
{0x0F12, 0x1400},    // 70003704 
{0x0F12, 0x8090},    // 70003706 
{0x0F12, 0x89A1},    // 70003708 
{0x0F12, 0x0409},    // 7000370A 
{0x0F12, 0x1409},    // 7000370C 
{0x0F12, 0x8111},    // 7000370E 
{0x0F12, 0x89E3},    // 70003710 
{0x0F12, 0x8A24},    // 70003712 
{0x0F12, 0x2B00},    // 70003714 
{0x0F12, 0xD104},    // 70003716 
{0x0F12, 0x17C3},    // 70003718 
{0x0F12, 0x0F5B},    // 7000371A 
{0x0F12, 0x1818},    // 7000371C 
{0x0F12, 0x10C0},    // 7000371E 
{0x0F12, 0x8090},    // 70003720 
{0x0F12, 0x2C00},    // 70003722 
{0x0F12, 0xD1E6},    // 70003724 
{0x0F12, 0x17C8},    // 70003726 
{0x0F12, 0x0F40},    // 70003728 
{0x0F12, 0x1840},    // 7000372A 
{0x0F12, 0x10C0},    // 7000372C 
{0x0F12, 0x8110},    // 7000372E 
{0x0F12, 0xE7E0},    // 70003730 
{0x0F12, 0xB510},    // 70003732 
{0x0F12, 0x0004},    // 70003734 
{0x0F12, 0x49D5},    // 70003736 
{0x0F12, 0x2204},    // 70003738 
{0x0F12, 0x6820},    // 7000373A 
{0x0F12, 0x5E8A},    // 7000373C 
{0x0F12, 0x0140},    // 7000373E 
{0x0F12, 0x1A80},    // 70003740 
{0x0F12, 0x0280},    // 70003742 
{0x0F12, 0x8849},    // 70003744 
{0x0F12, 0xF000},    // 70003746 
{0x0F12, 0xFAFF},    // 70003748 
{0x0F12, 0x6020},    // 7000374A 
{0x0F12, 0xE7D2},    // 7000374C 
{0x0F12, 0xB510},    // 7000374E 
{0x0F12, 0x0004},    // 70003750 
{0x0F12, 0x49CE},    // 70003752 
{0x0F12, 0x2208},    // 70003754 
{0x0F12, 0x6820},    // 70003756 
{0x0F12, 0x5E8A},    // 70003758 
{0x0F12, 0x0140},    // 7000375A 
{0x0F12, 0x1A80},    // 7000375C 
{0x0F12, 0x0280},    // 7000375E 
{0x0F12, 0x88C9},    // 70003760 
{0x0F12, 0xF000},    // 70003762 
{0x0F12, 0xFAF1},    // 70003764 
{0x0F12, 0x6020},    // 70003766 
{0x0F12, 0xE7C4},    // 70003768 
{0x0F12, 0xB510},    // 7000376A 
{0x0F12, 0x0004},    // 7000376C 
{0x0F12, 0x49C7},    // 7000376E 
{0x0F12, 0x48C8},    // 70003770 
{0x0F12, 0x884A},    // 70003772 
{0x0F12, 0x8B43},    // 70003774 
{0x0F12, 0x435A},    // 70003776 
{0x0F12, 0x2304},    // 70003778 
{0x0F12, 0x5ECB},    // 7000377A 
{0x0F12, 0x0A92},    // 7000377C 
{0x0F12, 0x18D2},    // 7000377E 
{0x0F12, 0x02D2},    // 70003780 
{0x0F12, 0x0C12},    // 70003782 
{0x0F12, 0x88CB},    // 70003784 
{0x0F12, 0x8B80},    // 70003786 
{0x0F12, 0x4343},    // 70003788 
{0x0F12, 0x0A98},    // 7000378A 
{0x0F12, 0x2308},    // 7000378C 
{0x0F12, 0x5ECB},    // 7000378E 
{0x0F12, 0x18C0},    // 70003790 
{0x0F12, 0x02C0},    // 70003792 
{0x0F12, 0x0C00},    // 70003794 
{0x0F12, 0x0411},    // 70003796 
{0x0F12, 0x0400},    // 70003798 
{0x0F12, 0x1409},    // 7000379A 
{0x0F12, 0x1400},    // 7000379C 
{0x0F12, 0x1A08},    // 7000379E 
{0x0F12, 0x49BC},    // 700037A0 
{0x0F12, 0x3980},    // 700037A2 
{0x0F12, 0x6348},    // 700037A4 
{0x0F12, 0x0020},    // 700037A6 
{0x0F12, 0xC80F},    // 700037A8 
{0x0F12, 0xF000},    // 700037AA 
{0x0F12, 0xFAD3},    // 700037AC 
{0x0F12, 0x6020},    // 700037AE 
{0x0F12, 0xE7A0},    // 700037B0 
{0x0F12, 0xB510},    // 700037B2 
{0x0F12, 0x4CB8},    // 700037B4 
{0x0F12, 0x48B9},    // 700037B6 
{0x0F12, 0x78A1},    // 700037B8 
{0x0F12, 0x2900},    // 700037BA 
{0x0F12, 0xD101},    // 700037BC 
{0x0F12, 0x87C1},    // 700037BE 
{0x0F12, 0xE004},    // 700037C0 
{0x0F12, 0x7AE1},    // 700037C2 
{0x0F12, 0x2900},    // 700037C4 
{0x0F12, 0xD001},    // 700037C6 
{0x0F12, 0x2101},    // 700037C8 
{0x0F12, 0x87C1},    // 700037CA 
{0x0F12, 0xF000},    // 700037CC 
{0x0F12, 0xFACA},    // 700037CE 
{0x0F12, 0x49B3},    // 700037D0 
{0x0F12, 0x8B08},    // 700037D2 
{0x0F12, 0x06C2},    // 700037D4 
{0x0F12, 0xD50A},    // 700037D6 
{0x0F12, 0x7AA2},    // 700037D8 
{0x0F12, 0x0652},    // 700037DA 
{0x0F12, 0xD507},    // 700037DC 
{0x0F12, 0x2210},    // 700037DE 
{0x0F12, 0x4390},    // 700037E0 
{0x0F12, 0x8308},    // 700037E2 
{0x0F12, 0x48AF},    // 700037E4 
{0x0F12, 0x7AE1},    // 700037E6 
{0x0F12, 0x6B00},    // 700037E8 
{0x0F12, 0xF000},    // 700037EA 
{0x0F12, 0xFAC3},    // 700037EC 
{0x0F12, 0x48A2},    // 700037EE 
{0x0F12, 0x89C0},    // 700037F0 
{0x0F12, 0x2801},    // 700037F2 
{0x0F12, 0xD109},    // 700037F4 
{0x0F12, 0x78A0},    // 700037F6 
{0x0F12, 0x2800},    // 700037F8 
{0x0F12, 0xD006},    // 700037FA 
{0x0F12, 0x7AE0},    // 700037FC 
{0x0F12, 0x2800},    // 700037FE 
{0x0F12, 0xD003},    // 70003800 
{0x0F12, 0x7AA0},    // 70003802 
{0x0F12, 0x2140},    // 70003804 
{0x0F12, 0x4308},    // 70003806 
{0x0F12, 0x72A0},    // 70003808 
{0x0F12, 0xE773},    // 7000380A 
{0x0F12, 0xB570},    // 7000380C 
{0x0F12, 0x4DA4},    // 7000380E 
{0x0F12, 0x4CA4},    // 70003810 
{0x0F12, 0x8B28},    // 70003812 
{0x0F12, 0x0701},    // 70003814 
{0x0F12, 0xD507},    // 70003816 
{0x0F12, 0x2108},    // 70003818 
{0x0F12, 0x4388},    // 7000381A 
{0x0F12, 0x8328},    // 7000381C 
{0x0F12, 0x49A2},    // 7000381E 
{0x0F12, 0x6B20},    // 70003820 
{0x0F12, 0x6B89},    // 70003822 
{0x0F12, 0xF000},    // 70003824 
{0x0F12, 0xFAAE},    // 70003826 
{0x0F12, 0x8B28},    // 70003828 
{0x0F12, 0x06C1},    // 7000382A 
{0x0F12, 0xD50A},    // 7000382C 
{0x0F12, 0x499A},    // 7000382E 
{0x0F12, 0x7A8A},    // 70003830 
{0x0F12, 0x0652},    // 70003832 
{0x0F12, 0xD406},    // 70003834 
{0x0F12, 0x2210},    // 70003836 
{0x0F12, 0x4390},    // 70003838 
{0x0F12, 0x8328},    // 7000383A 
{0x0F12, 0x7AC9},    // 7000383C 
{0x0F12, 0x6B20},    // 7000383E 
{0x0F12, 0xF000},    // 70003840 
{0x0F12, 0xFA98},    // 70003842 
{0x0F12, 0xE71E},    // 70003844 
{0x0F12, 0xB5F8},    // 70003846 
{0x0F12, 0x4C98},    // 70003848 
{0x0F12, 0x26FF},    // 7000384A 
{0x0F12, 0x8820},    // 7000384C 
{0x0F12, 0x4D98},    // 7000384E 
{0x0F12, 0x1C76},    // 70003850 
{0x0F12, 0x2702},    // 70003852 
{0x0F12, 0x2803},    // 70003854 
{0x0F12, 0xD112},    // 70003856 
{0x0F12, 0x8860},    // 70003858 
{0x0F12, 0x2800},    // 7000385A 
{0x0F12, 0xD10F},    // 7000385C 
{0x0F12, 0x88E0},    // 7000385E 
{0x0F12, 0x2800},    // 70003860 
{0x0F12, 0xD10C},    // 70003862 
{0x0F12, 0xF000},    // 70003864 
{0x0F12, 0xFA96},    // 70003866 
{0x0F12, 0x2800},    // 70003868 
{0x0F12, 0xD008},    // 7000386A 
{0x0F12, 0x8F28},    // 7000386C 
{0x0F12, 0x2800},    // 7000386E 
{0x0F12, 0xD001},    // 70003870 
{0x0F12, 0x80E6},    // 70003872 
{0x0F12, 0x80A7},    // 70003874 
{0x0F12, 0x2001},    // 70003876 
{0x0F12, 0x7260},    // 70003878 
{0x0F12, 0xF000},    // 7000387A 
{0x0F12, 0xFA93},    // 7000387C 
{0x0F12, 0x8820},    // 7000387E 
{0x0F12, 0x2802},    // 70003880 
{0x0F12, 0xD10E},    // 70003882 
{0x0F12, 0x8860},    // 70003884 
{0x0F12, 0x2800},    // 70003886 
{0x0F12, 0xD10B},    // 70003888 
{0x0F12, 0x88E0},    // 7000388A 
{0x0F12, 0x2800},    // 7000388C 
{0x0F12, 0xD108},    // 7000388E 
{0x0F12, 0x8F28},    // 70003890 
{0x0F12, 0x2800},    // 70003892 
{0x0F12, 0xD001},    // 70003894 
{0x0F12, 0x80E6},    // 70003896 
{0x0F12, 0x80A7},    // 70003898 
{0x0F12, 0x2001},    // 7000389A 
{0x0F12, 0x7260},    // 7000389C 
{0x0F12, 0xF000},    // 7000389E 
{0x0F12, 0xFA81},    // 700038A0 
{0x0F12, 0x88E0},    // 700038A2 
{0x0F12, 0x2800},    // 700038A4 
{0x0F12, 0xD006},    // 700038A6 
{0x0F12, 0x1FC1},    // 700038A8 
{0x0F12, 0x39FD},    // 700038AA 
{0x0F12, 0xD003},    // 700038AC 
{0x0F12, 0x2001},    // 700038AE 
{0x0F12, 0xBCF8},    // 700038B0 
{0x0F12, 0xBC08},    // 700038B2 
{0x0F12, 0x4718},    // 700038B4 
{0x0F12, 0x2000},    // 700038B6 
{0x0F12, 0xE7FA},    // 700038B8 
{0x0F12, 0xB570},    // 700038BA 
{0x0F12, 0x4C7D},    // 700038BC 
{0x0F12, 0x8860},    // 700038BE 
{0x0F12, 0x2800},    // 700038C0 
{0x0F12, 0xD00C},    // 700038C2 
{0x0F12, 0x8820},    // 700038C4 
{0x0F12, 0x4D74},    // 700038C6 
{0x0F12, 0x2800},    // 700038C8 
{0x0F12, 0xD009},    // 700038CA 
{0x0F12, 0x0029},    // 700038CC 
{0x0F12, 0x31A0},    // 700038CE 
{0x0F12, 0x7AC9},    // 700038D0 
{0x0F12, 0x2900},    // 700038D2 
{0x0F12, 0xD004},    // 700038D4 
{0x0F12, 0x7AA8},    // 700038D6 
{0x0F12, 0x2180},    // 700038D8 
{0x0F12, 0x4308},    // 700038DA 
{0x0F12, 0x72A8},    // 700038DC 
{0x0F12, 0xE6D1},    // 700038DE 
{0x0F12, 0x2800},    // 700038E0 
{0x0F12, 0xD003},    // 700038E2 
{0x0F12, 0xF7FF},    // 700038E4 
{0x0F12, 0xFFAF},    // 700038E6 
{0x0F12, 0x2800},    // 700038E8 
{0x0F12, 0xD1F8},    // 700038EA 
{0x0F12, 0x2000},    // 700038EC 
{0x0F12, 0x8060},    // 700038EE 
{0x0F12, 0x8820},    // 700038F0 
{0x0F12, 0x2800},    // 700038F2 
{0x0F12, 0xD003},    // 700038F4 
{0x0F12, 0x2008},    // 700038F6 
{0x0F12, 0xF000},    // 700038F8 
{0x0F12, 0xFA5C},    // 700038FA 
{0x0F12, 0xE00B},    // 700038FC 
{0x0F12, 0x486D},    // 700038FE 
{0x0F12, 0x3020},    // 70003900 
{0x0F12, 0x8880},    // 70003902 
{0x0F12, 0x2800},    // 70003904 
{0x0F12, 0xD103},    // 70003906 
{0x0F12, 0x7AA8},    // 70003908 
{0x0F12, 0x2101},    // 7000390A 
{0x0F12, 0x4308},    // 7000390C 
{0x0F12, 0x72A8},    // 7000390E 
{0x0F12, 0x2010},    // 70003910 
{0x0F12, 0xF000},    // 70003912 
{0x0F12, 0xFA4F},    // 70003914 
{0x0F12, 0x8820},    // 70003916 
{0x0F12, 0x2800},    // 70003918 
{0x0F12, 0xD1E0},    // 7000391A 
{0x0F12, 0x4856},    // 7000391C 
{0x0F12, 0x89C0},    // 7000391E 
{0x0F12, 0x2801},    // 70003920 
{0x0F12, 0xD1DC},    // 70003922 
{0x0F12, 0x7AA8},    // 70003924 
{0x0F12, 0x21BF},    // 70003926 
{0x0F12, 0x4008},    // 70003928 
{0x0F12, 0x72A8},    // 7000392A 
{0x0F12, 0xE6AA},    // 7000392C 
{0x0F12, 0x6800},    // 7000392E 
{0x0F12, 0x4961},    // 70003930 
{0x0F12, 0x8188},    // 70003932 
{0x0F12, 0x4861},    // 70003934 
{0x0F12, 0x2201},    // 70003936 
{0x0F12, 0x8981},    // 70003938 
{0x0F12, 0x4861},    // 7000393A 
{0x0F12, 0x0252},    // 7000393C 
{0x0F12, 0x4291},    // 7000393E 
{0x0F12, 0xD902},    // 70003940 
{0x0F12, 0x2102},    // 70003942 
{0x0F12, 0x8181},    // 70003944 
{0x0F12, 0x4770},    // 70003946 
{0x0F12, 0x2101},    // 70003948 
{0x0F12, 0x8181},    // 7000394A 
{0x0F12, 0x4770},    // 7000394C 
{0x0F12, 0xB5F1},    // 7000394E 
{0x0F12, 0x4E51},    // 70003950 
{0x0F12, 0x8834},    // 70003952 
{0x0F12, 0x2C00},    // 70003954 
{0x0F12, 0xD03C},    // 70003956 
{0x0F12, 0x2001},    // 70003958 
{0x0F12, 0x2C08},    // 7000395A 
{0x0F12, 0xD000},    // 7000395C 
{0x0F12, 0x2000},    // 7000395E 
{0x0F12, 0x70B0},    // 70003960 
{0x0F12, 0x4D50},    // 70003962 
{0x0F12, 0x2700},    // 70003964 
{0x0F12, 0x2800},    // 70003966 
{0x0F12, 0xD009},    // 70003968 
{0x0F12, 0xF000},    // 7000396A 
{0x0F12, 0xFA2B},    // 7000396C 
{0x0F12, 0x0028},    // 7000396E 
{0x0F12, 0x38F0},    // 70003970 
{0x0F12, 0x6328},    // 70003972 
{0x0F12, 0x7AB0},    // 70003974 
{0x0F12, 0x217E},    // 70003976 
{0x0F12, 0x4008},    // 70003978 
{0x0F12, 0x72B0},    // 7000397A 
{0x0F12, 0xE00C},    // 7000397C 
{0x0F12, 0x484C},    // 7000397E 
{0x0F12, 0x8F00},    // 70003980 
{0x0F12, 0x2800},    // 70003982 
{0x0F12, 0xD003},    // 70003984 
{0x0F12, 0xF000},    // 70003986 
{0x0F12, 0xFA25},    // 70003988 
{0x0F12, 0x4849},    // 7000398A 
{0x0F12, 0x8707},    // 7000398C 
{0x0F12, 0x2000},    // 7000398E 
{0x0F12, 0xF000},    // 70003990 
{0x0F12, 0xFA28},    // 70003992 
{0x0F12, 0x484B},    // 70003994 
{0x0F12, 0x6328},    // 70003996 
{0x0F12, 0x78B1},    // 70003998 
{0x0F12, 0x0038},    // 7000399A 
{0x0F12, 0x2900},    // 7000399C 
{0x0F12, 0xD008},    // 7000399E 
{0x0F12, 0x4944},    // 700039A0 
{0x0F12, 0x3920},    // 700039A2 
{0x0F12, 0x8ACA},    // 700039A4 
{0x0F12, 0x2A00},    // 700039A6 
{0x0F12, 0xD003},    // 700039A8 
{0x0F12, 0x8B09},    // 700039AA 
{0x0F12, 0x2900},    // 700039AC 
{0x0F12, 0xD000},    // 700039AE 
{0x0F12, 0x2001},    // 700039B0 
{0x0F12, 0x7170},    // 700039B2 
{0x0F12, 0x2C02},    // 700039B4 
{0x0F12, 0xD102},    // 700039B6 
{0x0F12, 0x483A},    // 700039B8 
{0x0F12, 0x3860},    // 700039BA 
{0x0F12, 0x6328},    // 700039BC 
{0x0F12, 0x2201},    // 700039BE 
{0x0F12, 0x2C02},    // 700039C0 
{0x0F12, 0xD000},    // 700039C2 
{0x0F12, 0x2200},    // 700039C4 
{0x0F12, 0x4834},    // 700039C6 
{0x0F12, 0x2110},    // 700039C8 
{0x0F12, 0x300A},    // 700039CA 
{0x0F12, 0xF000},    // 700039CC 
{0x0F12, 0xFA12},    // 700039CE 
{0x0F12, 0x8037},    // 700039D0 
{0x0F12, 0x9900},    // 700039D2 
{0x0F12, 0x0020},    // 700039D4 
{0x0F12, 0x600C},    // 700039D6 
{0x0F12, 0xE76A},    // 700039D8 
{0x0F12, 0xB538},    // 700039DA 
{0x0F12, 0x4837},    // 700039DC 
{0x0F12, 0x4669},    // 700039DE 
{0x0F12, 0x3848},    // 700039E0 
{0x0F12, 0xF000},    // 700039E2 
{0x0F12, 0xFA0F},    // 700039E4 
{0x0F12, 0x4A32},    // 700039E6 
{0x0F12, 0x4834},    // 700039E8 
{0x0F12, 0x8F51},    // 700039EA 
{0x0F12, 0x2400},    // 700039EC 
{0x0F12, 0x3020},    // 700039EE 
{0x0F12, 0x2900},    // 700039F0 
{0x0F12, 0xD00A},    // 700039F2 
{0x0F12, 0x8754},    // 700039F4 
{0x0F12, 0x6941},    // 700039F6 
{0x0F12, 0x6451},    // 700039F8 
{0x0F12, 0x6491},    // 700039FA 
{0x0F12, 0x466B},    // 700039FC 
{0x0F12, 0x8819},    // 700039FE 
{0x0F12, 0x87D1},    // 70003A00 
{0x0F12, 0x885B},    // 70003A02 
{0x0F12, 0x0011},    // 70003A04 
{0x0F12, 0x3140},    // 70003A06 
{0x0F12, 0x800B},    // 70003A08 
{0x0F12, 0x8F91},    // 70003A0A 
{0x0F12, 0x2900},    // 70003A0C 
{0x0F12, 0xD002},    // 70003A0E 
{0x0F12, 0x8794},    // 70003A10 
{0x0F12, 0x6940},    // 70003A12 
{0x0F12, 0x6490},    // 70003A14 
{0x0F12, 0xF000},    // 70003A16 
{0x0F12, 0xF9FD},    // 70003A18 
{0x0F12, 0xBC38},    // 70003A1A 
{0x0F12, 0xBC08},    // 70003A1C 
{0x0F12, 0x4718},    // 70003A1E 
{0x0F12, 0xB5F8},    // 70003A20 
{0x0F12, 0x4C29},    // 70003A22 
{0x0F12, 0x89E0},    // 70003A24 
{0x0F12, 0xF000},    // 70003A26 
{0x0F12, 0xF9FD},    // 70003A28 
{0x0F12, 0x0006},    // 70003A2A 
{0x0F12, 0x8A20},    // 70003A2C 
{0x0F12, 0xF000},    // 70003A2E 
{0x0F12, 0xFA01},    // 70003A30 
{0x0F12, 0x0007},    // 70003A32 
{0x0F12, 0x4821},    // 70003A34 
{0x0F12, 0x4D1E},    // 70003A36 
{0x0F12, 0x3020},    // 70003A38 
{0x0F12, 0x6CA9},    // 70003A3A 
{0x0F12, 0x6940},    // 70003A3C 
{0x0F12, 0x1809},    // 70003A3E 
{0x0F12, 0x0200},    // 70003A40 
{0x0F12, 0xF000},    // 70003A42 
{0x0F12, 0xF9FF},    // 70003A44 
{0x0F12, 0x0400},    // 70003A46 
{0x0F12, 0x0C00},    // 70003A48 
{0x0F12, 0x002A},    // 70003A4A 
{0x0F12, 0x326E},    // 70003A4C 
{0x0F12, 0x0011},    // 70003A4E 
{0x0F12, 0x390A},    // 70003A50 
{0x0F12, 0x2305},    // 70003A52 
{0x0F12, 0xF000},    // 70003A54 
{0x0F12, 0xF9FC},    // 70003A56 
{0x0F12, 0x4C14},    // 70003A58 
{0x0F12, 0x61A0},    // 70003A5A 
{0x0F12, 0x8FEB},    // 70003A5C 
{0x0F12, 0x0002},    // 70003A5E 
{0x0F12, 0x0031},    // 70003A60 
{0x0F12, 0x0018},    // 70003A62 
{0x0F12, 0xF000},    // 70003A64 
{0x0F12, 0xF9FC},    // 70003A66 
{0x0F12, 0x466B},    // 70003A68 
{0x0F12, 0x0005},    // 70003A6A 
{0x0F12, 0x8018},    // 70003A6C 
{0x0F12, 0xE02D},    // 70003A6E 
{0x0F12, 0x3290},    // 70003A70 
{0x0F12, 0x7000},    // 70003A72 
{0x0F12, 0x3294},    // 70003A74 
{0x0F12, 0x7000},    // 70003A76 
{0x0F12, 0x04A8},    // 70003A78 
{0x0F12, 0x7000},    // 70003A7A 
{0x0F12, 0x15DC},    // 70003A7C 
{0x0F12, 0x7000},    // 70003A7E 
{0x0F12, 0x5000},    // 70003A80 
{0x0F12, 0xD000},    // 70003A82 
{0x0F12, 0x064C},    // 70003A84 
{0x0F12, 0x7000},    // 70003A86 
{0x0F12, 0xA000},    // 70003A88 
{0x0F12, 0xD000},    // 70003A8A 
{0x0F12, 0x2468},    // 70003A8C 
{0x0F12, 0x7000},    // 70003A8E 
{0x0F12, 0x11DC},    // 70003A90 
{0x0F12, 0x7000},    // 70003A92 
{0x0F12, 0x2828},    // 70003A94 
{0x0F12, 0x7000},    // 70003A96 
{0x0F12, 0x1E84},    // 70003A98 
{0x0F12, 0x7000},    // 70003A9A 
{0x0F12, 0x1BE4},    // 70003A9C 
{0x0F12, 0x7000},    // 70003A9E 
{0x0F12, 0x2EA8},    // 70003AA0 
{0x0F12, 0x7000},    // 70003AA2 
{0x0F12, 0x21A4},    // 70003AA4 
{0x0F12, 0x7000},    // 70003AA6 
{0x0F12, 0x0100},    // 70003AA8 
{0x0F12, 0x7000},    // 70003AAA 
{0x0F12, 0x31A0},    // 70003AAC 
{0x0F12, 0x7000},    // 70003AAE 
{0x0F12, 0x3F48},    // 70003AB0 
{0x0F12, 0x7000},    // 70003AB2 
{0x0F12, 0x01E8},    // 70003AB4 
{0x0F12, 0x7000},    // 70003AB6 
{0x0F12, 0xF2A0},    // 70003AB8 
{0x0F12, 0xD000},    // 70003ABA 
{0x0F12, 0x2A44},    // 70003ABC 
{0x0F12, 0x7000},    // 70003ABE 
{0x0F12, 0xF400},    // 70003AC0 
{0x0F12, 0xD000},    // 70003AC2 
{0x0F12, 0x2024},    // 70003AC4 
{0x0F12, 0x7000},    // 70003AC6 
{0x0F12, 0x1650},    // 70003AC8 
{0x0F12, 0x7000},    // 70003ACA 
{0x0F12, 0x4888},    // 70003ACC 
{0x0F12, 0x69A2},    // 70003ACE 
{0x0F12, 0x8800},    // 70003AD0 
{0x0F12, 0x0039},    // 70003AD2 
{0x0F12, 0xF000},    // 70003AD4 
{0x0F12, 0xF9C4},    // 70003AD6 
{0x0F12, 0x466B},    // 70003AD8 
{0x0F12, 0x0006},    // 70003ADA 
{0x0F12, 0x8058},    // 70003ADC 
{0x0F12, 0x0021},    // 70003ADE 
{0x0F12, 0x9800},    // 70003AE0 
{0x0F12, 0x311C},    // 70003AE2 
{0x0F12, 0xF000},    // 70003AE4 
{0x0F12, 0xF9C4},    // 70003AE6 
{0x0F12, 0x4981},    // 70003AE8 
{0x0F12, 0x3140},    // 70003AEA 
{0x0F12, 0x808D},    // 70003AEC 
{0x0F12, 0x80CE},    // 70003AEE 
{0x0F12, 0x8BA1},    // 70003AF0 
{0x0F12, 0x4880},    // 70003AF2 
{0x0F12, 0x8001},    // 70003AF4 
{0x0F12, 0x8BE1},    // 70003AF6 
{0x0F12, 0x8041},    // 70003AF8 
{0x0F12, 0x8C21},    // 70003AFA 
{0x0F12, 0x8081},    // 70003AFC 
{0x0F12, 0xE6D7},    // 70003AFE 
{0x0F12, 0xB5F8},    // 70003B00 
{0x0F12, 0x4E7B},    // 70003B02 
{0x0F12, 0x3E40},    // 70003B04 
{0x0F12, 0x6C70},    // 70003B06 
{0x0F12, 0x6CB1},    // 70003B08 
{0x0F12, 0x0200},    // 70003B0A 
{0x0F12, 0xF000},    // 70003B0C 
{0x0F12, 0xF99A},    // 70003B0E 
{0x0F12, 0x0400},    // 70003B10 
{0x0F12, 0x0C00},    // 70003B12 
{0x0F12, 0x2401},    // 70003B14 
{0x0F12, 0x0364},    // 70003B16 
{0x0F12, 0x42A0},    // 70003B18 
{0x0F12, 0xD200},    // 70003B1A 
{0x0F12, 0x0004},    // 70003B1C 
{0x0F12, 0x4A74},    // 70003B1E 
{0x0F12, 0x0020},    // 70003B20 
{0x0F12, 0x323E},    // 70003B22 
{0x0F12, 0x1F91},    // 70003B24 
{0x0F12, 0x2303},    // 70003B26 
{0x0F12, 0xF000},    // 70003B28 
{0x0F12, 0xF992},    // 70003B2A 
{0x0F12, 0x0405},    // 70003B2C 
{0x0F12, 0x0C2D},    // 70003B2E 
{0x0F12, 0x4A6F},    // 70003B30 
{0x0F12, 0x0020},    // 70003B32 
{0x0F12, 0x321A},    // 70003B34 
{0x0F12, 0x0011},    // 70003B36 
{0x0F12, 0x390A},    // 70003B38 
{0x0F12, 0x2305},    // 70003B3A 
{0x0F12, 0xF000},    // 70003B3C 
{0x0F12, 0xF988},    // 70003B3E 
{0x0F12, 0x496B},    // 70003B40 
{0x0F12, 0x3940},    // 70003B42 
{0x0F12, 0x64C8},    // 70003B44 
{0x0F12, 0x496C},    // 70003B46 
{0x0F12, 0x4E6A},    // 70003B48 
{0x0F12, 0x88C8},    // 70003B4A 
{0x0F12, 0x2701},    // 70003B4C 
{0x0F12, 0x3620},    // 70003B4E 
{0x0F12, 0x2800},    // 70003B50 
{0x0F12, 0xD009},    // 70003B52 
{0x0F12, 0x4C69},    // 70003B54 
{0x0F12, 0x38FF},    // 70003B56 
{0x0F12, 0x1E40},    // 70003B58 
{0x0F12, 0xD00A},    // 70003B5A 
{0x0F12, 0x2804},    // 70003B5C 
{0x0F12, 0xD01D},    // 70003B5E 
{0x0F12, 0x2806},    // 70003B60 
{0x0F12, 0xD101},    // 70003B62 
{0x0F12, 0x2000},    // 70003B64 
{0x0F12, 0x80C8},    // 70003B66 
{0x0F12, 0x82B7},    // 70003B68 
{0x0F12, 0x2001},    // 70003B6A 
{0x0F12, 0xF000},    // 70003B6C 
{0x0F12, 0xF988},    // 70003B6E 
{0x0F12, 0xE69E},    // 70003B70 
{0x0F12, 0x000D},    // 70003B72 
{0x0F12, 0x724F},    // 70003B74 
{0x0F12, 0x2001},    // 70003B76 
{0x0F12, 0xF000},    // 70003B78 
{0x0F12, 0xF98A},    // 70003B7A 
{0x0F12, 0xF000},    // 70003B7C 
{0x0F12, 0xF990},    // 70003B7E 
{0x0F12, 0x485B},    // 70003B80 
{0x0F12, 0x3840},    // 70003B82 
{0x0F12, 0x6C81},    // 70003B84 
{0x0F12, 0x6CC0},    // 70003B86 
{0x0F12, 0x4341},    // 70003B88 
{0x0F12, 0x0A08},    // 70003B8A 
{0x0F12, 0x6160},    // 70003B8C 
{0x0F12, 0x20FF},    // 70003B8E 
{0x0F12, 0x1D40},    // 70003B90 
{0x0F12, 0x80E8},    // 70003B92 
{0x0F12, 0x4858},    // 70003B94 
{0x0F12, 0x3040},    // 70003B96 
{0x0F12, 0x7707},    // 70003B98 
{0x0F12, 0xE7E5},    // 70003B9A 
{0x0F12, 0x4856},    // 70003B9C 
{0x0F12, 0x7247},    // 70003B9E 
{0x0F12, 0x21FF},    // 70003BA0 
{0x0F12, 0x1DC9},    // 70003BA2 
{0x0F12, 0x80C1},    // 70003BA4 
{0x0F12, 0xF000},    // 70003BA6 
{0x0F12, 0xF983},    // 70003BA8 
{0x0F12, 0x4952},    // 70003BAA 
{0x0F12, 0x3940},    // 70003BAC 
{0x0F12, 0x2800},    // 70003BAE 
{0x0F12, 0xD007},    // 70003BB0 
{0x0F12, 0x684A},    // 70003BB2 
{0x0F12, 0x0001},    // 70003BB4 
{0x0F12, 0x436A},    // 70003BB6 
{0x0F12, 0x0010},    // 70003BB8 
{0x0F12, 0xF000},    // 70003BBA 
{0x0F12, 0xF943},    // 70003BBC 
{0x0F12, 0x6160},    // 70003BBE 
{0x0F12, 0xE002},    // 70003BC0 
{0x0F12, 0x6848},    // 70003BC2 
{0x0F12, 0x4368},    // 70003BC4 
{0x0F12, 0x6160},    // 70003BC6 
{0x0F12, 0x8BF0},    // 70003BC8 
{0x0F12, 0x2800},    // 70003BCA 
{0x0F12, 0xD001},    // 70003BCC 
{0x0F12, 0xF7FF},    // 70003BCE 
{0x0F12, 0xFF27},    // 70003BD0 
{0x0F12, 0x2000},    // 70003BD2 
{0x0F12, 0xF000},    // 70003BD4 
{0x0F12, 0xF95C},    // 70003BD6 
{0x0F12, 0x4947},    // 70003BD8 
{0x0F12, 0x20FF},    // 70003BDA 
{0x0F12, 0x1DC0},    // 70003BDC 
{0x0F12, 0x80C8},    // 70003BDE 
{0x0F12, 0xE7C2},    // 70003BE0 
{0x0F12, 0xB5F8},    // 70003BE2 
{0x0F12, 0x2400},    // 70003BE4 
{0x0F12, 0x4D46},    // 70003BE6 
{0x0F12, 0x4846},    // 70003BE8 
{0x0F12, 0x210E},    // 70003BEA 
{0x0F12, 0x8041},    // 70003BEC 
{0x0F12, 0x2101},    // 70003BEE 
{0x0F12, 0x8001},    // 70003BF0 
{0x0F12, 0xF000},    // 70003BF2 
{0x0F12, 0xF965},    // 70003BF4 
{0x0F12, 0x4844},    // 70003BF6 
{0x0F12, 0x8840},    // 70003BF8 
{0x0F12, 0xF000},    // 70003BFA 
{0x0F12, 0xF89D},    // 70003BFC 
{0x0F12, 0x4E3C},    // 70003BFE 
{0x0F12, 0x270D},    // 70003C00 
{0x0F12, 0x073F},    // 70003C02 
{0x0F12, 0x3E40},    // 70003C04 
{0x0F12, 0x19E8},    // 70003C06 
{0x0F12, 0x8803},    // 70003C08 
{0x0F12, 0x00E2},    // 70003C0A 
{0x0F12, 0x1991},    // 70003C0C 
{0x0F12, 0x804B},    // 70003C0E 
{0x0F12, 0x8843},    // 70003C10 
{0x0F12, 0x52B3},    // 70003C12 
{0x0F12, 0x8882},    // 70003C14 
{0x0F12, 0x80CA},    // 70003C16 
{0x0F12, 0x88C0},    // 70003C18 
{0x0F12, 0x8088},    // 70003C1A 
{0x0F12, 0x3508},    // 70003C1C 
{0x0F12, 0x042D},    // 70003C1E 
{0x0F12, 0x0C2D},    // 70003C20 
{0x0F12, 0x1C64},    // 70003C22 
{0x0F12, 0x0424},    // 70003C24 
{0x0F12, 0x0C24},    // 70003C26 
{0x0F12, 0x2C07},    // 70003C28 
{0x0F12, 0xD3EC},    // 70003C2A 
{0x0F12, 0xE640},    // 70003C2C 
{0x0F12, 0xB5F0},    // 70003C2E 
{0x0F12, 0xB085},    // 70003C30 
{0x0F12, 0x6801},    // 70003C32 
{0x0F12, 0x9103},    // 70003C34 
{0x0F12, 0x6881},    // 70003C36 
{0x0F12, 0x040A},    // 70003C38 
{0x0F12, 0x0C12},    // 70003C3A 
{0x0F12, 0x4933},    // 70003C3C 
{0x0F12, 0x8B89},    // 70003C3E 
{0x0F12, 0x2900},    // 70003C40 
{0x0F12, 0xD001},    // 70003C42 
{0x0F12, 0x0011},    // 70003C44 
{0x0F12, 0xE000},    // 70003C46 
{0x0F12, 0x2100},    // 70003C48 
{0x0F12, 0x9102},    // 70003C4A 
{0x0F12, 0x6840},    // 70003C4C 
{0x0F12, 0x0401},    // 70003C4E 
{0x0F12, 0x9803},    // 70003C50 
{0x0F12, 0x0C09},    // 70003C52 
{0x0F12, 0xF000},    // 70003C54 
{0x0F12, 0xF93C},    // 70003C56 
{0x0F12, 0x4825},    // 70003C58 
{0x0F12, 0x3040},    // 70003C5A 
{0x0F12, 0x8900},    // 70003C5C 
{0x0F12, 0x2800},    // 70003C5E 
{0x0F12, 0xD03B},    // 70003C60 
{0x0F12, 0x2100},    // 70003C62 
{0x0F12, 0x4825},    // 70003C64 
{0x0F12, 0x4D2A},    // 70003C66 
{0x0F12, 0x30C0},    // 70003C68 
{0x0F12, 0x4684},    // 70003C6A 
{0x0F12, 0x4B29},    // 70003C6C 
{0x0F12, 0x4C20},    // 70003C6E 
{0x0F12, 0x88DA},    // 70003C70 
{0x0F12, 0x3C40},    // 70003C72 
{0x0F12, 0x0048},    // 70003C74 
{0x0F12, 0x00D7},    // 70003C76 
{0x0F12, 0x193E},    // 70003C78 
{0x0F12, 0x197F},    // 70003C7A 
{0x0F12, 0x183F},    // 70003C7C 
{0x0F12, 0x5A36},    // 70003C7E 
{0x0F12, 0x8AFF},    // 70003C80 
{0x0F12, 0x437E},    // 70003C82 
{0x0F12, 0x00B6},    // 70003C84 
{0x0F12, 0x0C37},    // 70003C86 
{0x0F12, 0x1906},    // 70003C88 
{0x0F12, 0x3680},    // 70003C8A 
{0x0F12, 0x8177},    // 70003C8C 
{0x0F12, 0x1C52},    // 70003C8E 
{0x0F12, 0x00D2},    // 70003C90 
{0x0F12, 0x1914},    // 70003C92 
{0x0F12, 0x1952},    // 70003C94 
{0x0F12, 0x1812},    // 70003C96 
{0x0F12, 0x5A24},    // 70003C98 
{0x0F12, 0x8AD2},    // 70003C9A 
{0x0F12, 0x4354},    // 70003C9C 
{0x0F12, 0x00A2},    // 70003C9E 
{0x0F12, 0x0C12},    // 70003CA0 
{0x0F12, 0x8272},    // 70003CA2 
{0x0F12, 0x891C},    // 70003CA4 
{0x0F12, 0x895B},    // 70003CA6 
{0x0F12, 0x4367},    // 70003CA8 
{0x0F12, 0x435A},    // 70003CAA 
{0x0F12, 0x1943},    // 70003CAC 
{0x0F12, 0x3340},    // 70003CAE 
{0x0F12, 0x89DB},    // 70003CB0 
{0x0F12, 0x9C02},    // 70003CB2 
{0x0F12, 0x18BA},    // 70003CB4 
{0x0F12, 0x4363},    // 70003CB6 
{0x0F12, 0x18D2},    // 70003CB8 
{0x0F12, 0x0212},    // 70003CBA 
{0x0F12, 0x0C12},    // 70003CBC 
{0x0F12, 0x466B},    // 70003CBE 
{0x0F12, 0x521A},    // 70003CC0 
{0x0F12, 0x4663},    // 70003CC2 
{0x0F12, 0x7DDB},    // 70003CC4 
{0x0F12, 0x435A},    // 70003CC6 
{0x0F12, 0x9B03},    // 70003CC8 
{0x0F12, 0x0252},    // 70003CCA 
{0x0F12, 0x0C12},    // 70003CCC 
{0x0F12, 0x521A},    // 70003CCE 
{0x0F12, 0x1C49},    // 70003CD0 
{0x0F12, 0x0409},    // 70003CD2 
{0x0F12, 0x0C09},    // 70003CD4 
{0x0F12, 0x2904},    // 70003CD6 
{0x0F12, 0xD3C8},    // 70003CD8 
{0x0F12, 0xB005},    // 70003CDA 
{0x0F12, 0xBCF0},    // 70003CDC 
{0x0F12, 0xBC08},    // 70003CDE 
{0x0F12, 0x4718},    // 70003CE0 
{0x0F12, 0xB510},    // 70003CE2 
{0x0F12, 0xF7FF},    // 70003CE4 
{0x0F12, 0xFF7D},    // 70003CE6 
{0x0F12, 0xF000},    // 70003CE8 
{0x0F12, 0xF8FA},    // 70003CEA 
{0x0F12, 0xE502},    // 70003CEC 
{0x0F12, 0x0000},    // 70003CEE 
{0x0F12, 0x3F88},    // 70003CF0 
{0x0F12, 0x7000},    // 70003CF2 
{0x0F12, 0x2A24},    // 70003CF4 
{0x0F12, 0x7000},    // 70003CF6 
{0x0F12, 0x31A0},    // 70003CF8 
{0x0F12, 0x7000},    // 70003CFA 
{0x0F12, 0x2A64},    // 70003CFC 
{0x0F12, 0x7000},    // 70003CFE 
{0x0F12, 0xA006},    // 70003D00 
{0x0F12, 0x0000},    // 70003D02 
{0x0F12, 0xA000},    // 70003D04 
{0x0F12, 0xD000},    // 70003D06 
{0x0F12, 0x064C},    // 70003D08 
{0x0F12, 0x7000},    // 70003D0A 
{0x0F12, 0x07C4},    // 70003D0C 
{0x0F12, 0x7000},    // 70003D0E 
{0x0F12, 0x07E8},    // 70003D10 
{0x0F12, 0x7000},    // 70003D12 
{0x0F12, 0x1FA0},    // 70003D14 
{0x0F12, 0x7000},    // 70003D16 
{0x0F12, 0x4778},    // 70003D18 
{0x0F12, 0x46C0},    // 70003D1A 
{0x0F12, 0xC000},    // 70003D1C 
{0x0F12, 0xE59F},    // 70003D1E 
{0x0F12, 0xFF1C},    // 70003D20 
{0x0F12, 0xE12F},    // 70003D22 
{0x0F12, 0x1F63},    // 70003D24 
{0x0F12, 0x0001},    // 70003D26 
{0x0F12, 0x4778},    // 70003D28 
{0x0F12, 0x46C0},    // 70003D2A 
{0x0F12, 0xC000},    // 70003D2C 
{0x0F12, 0xE59F},    // 70003D2E 
{0x0F12, 0xFF1C},    // 70003D30 
{0x0F12, 0xE12F},    // 70003D32 
{0x0F12, 0x1EDF},    // 70003D34 
{0x0F12, 0x0001},    // 70003D36 
{0x0F12, 0x4778},    // 70003D38 
{0x0F12, 0x46C0},    // 70003D3A 
{0x0F12, 0xC000},    // 70003D3C 
{0x0F12, 0xE59F},    // 70003D3E 
{0x0F12, 0xFF1C},    // 70003D40 
{0x0F12, 0xE12F},    // 70003D42 
{0x0F12, 0xFDAF},    // 70003D44 
{0x0F12, 0x0000},    // 70003D46 
{0x0F12, 0x4778},    // 70003D48 
{0x0F12, 0x46C0},    // 70003D4A 
{0x0F12, 0xF004},    // 70003D4C 
{0x0F12, 0xE51F},    // 70003D4E 
{0x0F12, 0x2328},    // 70003D50 
{0x0F12, 0x0001},    // 70003D52 
{0x0F12, 0x4778},    // 70003D54 
{0x0F12, 0x46C0},    // 70003D56 
{0x0F12, 0xC000},    // 70003D58 
{0x0F12, 0xE59F},    // 70003D5A 
{0x0F12, 0xFF1C},    // 70003D5C 
{0x0F12, 0xE12F},    // 70003D5E 
{0x0F12, 0x9E89},    // 70003D60 
{0x0F12, 0x0000},    // 70003D62 
{0x0F12, 0x4778},    // 70003D64 
{0x0F12, 0x46C0},    // 70003D66 
{0x0F12, 0xC000},    // 70003D68 
{0x0F12, 0xE59F},    // 70003D6A 
{0x0F12, 0xFF1C},    // 70003D6C 
{0x0F12, 0xE12F},    // 70003D6E 
{0x0F12, 0x495F},    // 70003D70 
{0x0F12, 0x0000},    // 70003D72 
{0x0F12, 0x4778},    // 70003D74 
{0x0F12, 0x46C0},    // 70003D76 
{0x0F12, 0xC000},    // 70003D78 
{0x0F12, 0xE59F},    // 70003D7A 
{0x0F12, 0xFF1C},    // 70003D7C 
{0x0F12, 0xE12F},    // 70003D7E 
{0x0F12, 0xE403},    // 70003D80 
{0x0F12, 0x0000},    // 70003D82 
{0x0F12, 0x4778},    // 70003D84 
{0x0F12, 0x46C0},    // 70003D86 
{0x0F12, 0xC000},    // 70003D88 
{0x0F12, 0xE59F},    // 70003D8A 
{0x0F12, 0xFF1C},    // 70003D8C 
{0x0F12, 0xE12F},    // 70003D8E 
{0x0F12, 0x24B3},    // 70003D90 
{0x0F12, 0x0001},    // 70003D92 
{0x0F12, 0x4778},    // 70003D94 
{0x0F12, 0x46C0},    // 70003D96 
{0x0F12, 0xC000},    // 70003D98 
{0x0F12, 0xE59F},    // 70003D9A 
{0x0F12, 0xFF1C},    // 70003D9C 
{0x0F12, 0xE12F},    // 70003D9E 
{0x0F12, 0xEECD},    // 70003DA0 
{0x0F12, 0x0000},    // 70003DA2 
{0x0F12, 0x4778},    // 70003DA4 
{0x0F12, 0x46C0},    // 70003DA6 
{0x0F12, 0xC000},    // 70003DA8 
{0x0F12, 0xE59F},    // 70003DAA 
{0x0F12, 0xFF1C},    // 70003DAC 
{0x0F12, 0xE12F},    // 70003DAE 
{0x0F12, 0xF049},    // 70003DB0 
{0x0F12, 0x0000},    // 70003DB2 
{0x0F12, 0x4778},    // 70003DB4 
{0x0F12, 0x46C0},    // 70003DB6 
{0x0F12, 0xC000},    // 70003DB8 
{0x0F12, 0xE59F},    // 70003DBA 
{0x0F12, 0xFF1C},    // 70003DBC 
{0x0F12, 0xE12F},    // 70003DBE 
{0x0F12, 0x12DF},    // 70003DC0 
{0x0F12, 0x0000},    // 70003DC2 
{0x0F12, 0x4778},    // 70003DC4 
{0x0F12, 0x46C0},    // 70003DC6 
{0x0F12, 0xC000},    // 70003DC8 
{0x0F12, 0xE59F},    // 70003DCA 
{0x0F12, 0xFF1C},    // 70003DCC 
{0x0F12, 0xE12F},    // 70003DCE 
{0x0F12, 0xF05B},    // 70003DD0 
{0x0F12, 0x0000},    // 70003DD2 
{0x0F12, 0x4778},    // 70003DD4 
{0x0F12, 0x46C0},    // 70003DD6 
{0x0F12, 0xC000},    // 70003DD8 
{0x0F12, 0xE59F},    // 70003DDA 
{0x0F12, 0xFF1C},    // 70003DDC 
{0x0F12, 0xE12F},    // 70003DDE 
{0x0F12, 0xF07B},    // 70003DE0 
{0x0F12, 0x0000},    // 70003DE2 
{0x0F12, 0x4778},    // 70003DE4 
{0x0F12, 0x46C0},    // 70003DE6 
{0x0F12, 0xC000},    // 70003DE8 
{0x0F12, 0xE59F},    // 70003DEA 
{0x0F12, 0xFF1C},    // 70003DEC 
{0x0F12, 0xE12F},    // 70003DEE 
{0x0F12, 0xFE6D},    // 70003DF0 
{0x0F12, 0x0000},    // 70003DF2 
{0x0F12, 0x4778},    // 70003DF4 
{0x0F12, 0x46C0},    // 70003DF6 
{0x0F12, 0xC000},    // 70003DF8 
{0x0F12, 0xE59F},    // 70003DFA 
{0x0F12, 0xFF1C},    // 70003DFC 
{0x0F12, 0xE12F},    // 70003DFE 
{0x0F12, 0x3295},    // 70003E00 
{0x0F12, 0x0000},    // 70003E02 
{0x0F12, 0x4778},    // 70003E04 
{0x0F12, 0x46C0},    // 70003E06 
{0x0F12, 0xC000},    // 70003E08 
{0x0F12, 0xE59F},    // 70003E0A 
{0x0F12, 0xFF1C},    // 70003E0C 
{0x0F12, 0xE12F},    // 70003E0E 
{0x0F12, 0x234F},    // 70003E10 
{0x0F12, 0x0000},    // 70003E12 
{0x0F12, 0x4778},    // 70003E14 
{0x0F12, 0x46C0},    // 70003E16 
{0x0F12, 0xC000},    // 70003E18 
{0x0F12, 0xE59F},    // 70003E1A 
{0x0F12, 0xFF1C},    // 70003E1C 
{0x0F12, 0xE12F},    // 70003E1E 
{0x0F12, 0x4521},    // 70003E20 
{0x0F12, 0x0000},    // 70003E22 
{0x0F12, 0x4778},    // 70003E24 
{0x0F12, 0x46C0},    // 70003E26 
{0x0F12, 0xC000},    // 70003E28 
{0x0F12, 0xE59F},    // 70003E2A 
{0x0F12, 0xFF1C},    // 70003E2C 
{0x0F12, 0xE12F},    // 70003E2E 
{0x0F12, 0x7C0D},    // 70003E30 
{0x0F12, 0x0000},    // 70003E32 
{0x0F12, 0x4778},    // 70003E34 
{0x0F12, 0x46C0},    // 70003E36 
{0x0F12, 0xC000},    // 70003E38 
{0x0F12, 0xE59F},    // 70003E3A 
{0x0F12, 0xFF1C},    // 70003E3C 
{0x0F12, 0xE12F},    // 70003E3E 
{0x0F12, 0x7C2B},    // 70003E40 
{0x0F12, 0x0000},    // 70003E42 
{0x0F12, 0x4778},    // 70003E44 
{0x0F12, 0x46C0},    // 70003E46 
{0x0F12, 0xF004},    // 70003E48 
{0x0F12, 0xE51F},    // 70003E4A 
{0x0F12, 0x24C4},    // 70003E4C 
{0x0F12, 0x0001},    // 70003E4E 
{0x0F12, 0x4778},    // 70003E50 
{0x0F12, 0x46C0},    // 70003E52 
{0x0F12, 0xC000},    // 70003E54 
{0x0F12, 0xE59F},    // 70003E56 
{0x0F12, 0xFF1C},    // 70003E58 
{0x0F12, 0xE12F},    // 70003E5A 
{0x0F12, 0x3183},    // 70003E5C 
{0x0F12, 0x0000},    // 70003E5E 
{0x0F12, 0x4778},    // 70003E60 
{0x0F12, 0x46C0},    // 70003E62 
{0x0F12, 0xC000},    // 70003E64 
{0x0F12, 0xE59F},    // 70003E66 
{0x0F12, 0xFF1C},    // 70003E68 
{0x0F12, 0xE12F},    // 70003E6A 
{0x0F12, 0x302F},    // 70003E6C 
{0x0F12, 0x0000},    // 70003E6E 
{0x0F12, 0x4778},    // 70003E70 
{0x0F12, 0x46C0},    // 70003E72 
{0x0F12, 0xC000},    // 70003E74 
{0x0F12, 0xE59F},    // 70003E76 
{0x0F12, 0xFF1C},    // 70003E78 
{0x0F12, 0xE12F},    // 70003E7A 
{0x0F12, 0xEF07},    // 70003E7C 
{0x0F12, 0x0000},    // 70003E7E 
{0x0F12, 0x4778},    // 70003E80 
{0x0F12, 0x46C0},    // 70003E82 
{0x0F12, 0xC000},    // 70003E84 
{0x0F12, 0xE59F},    // 70003E86 
{0x0F12, 0xFF1C},    // 70003E88 
{0x0F12, 0xE12F},    // 70003E8A 
{0x0F12, 0x48FB},    // 70003E8C 
{0x0F12, 0x0000},    // 70003E8E 
{0x0F12, 0x4778},    // 70003E90 
{0x0F12, 0x46C0},    // 70003E92 
{0x0F12, 0xC000},    // 70003E94 
{0x0F12, 0xE59F},    // 70003E96 
{0x0F12, 0xFF1C},    // 70003E98 
{0x0F12, 0xE12F},    // 70003E9A 
{0x0F12, 0xF0B1},    // 70003E9C 
{0x0F12, 0x0000},    // 70003E9E 
{0x0F12, 0x4778},    // 70003EA0 
{0x0F12, 0x46C0},    // 70003EA2 
{0x0F12, 0xC000},    // 70003EA4 
{0x0F12, 0xE59F},    // 70003EA6 
{0x0F12, 0xFF1C},    // 70003EA8 
{0x0F12, 0xE12F},    // 70003EAA 
{0x0F12, 0xEEDF},    // 70003EAC 
{0x0F12, 0x0000},    // 70003EAE 
{0x0F12, 0x4778},    // 70003EB0 
{0x0F12, 0x46C0},    // 70003EB2 
{0x0F12, 0xC000},    // 70003EB4 
{0x0F12, 0xE59F},    // 70003EB6 
{0x0F12, 0xFF1C},    // 70003EB8 
{0x0F12, 0xE12F},    // 70003EBA 
{0x0F12, 0xAEF1},    // 70003EBC 
{0x0F12, 0x0000},    // 70003EBE 
{0x0F12, 0x4778},    // 70003EC0 
{0x0F12, 0x46C0},    // 70003EC2 
{0x0F12, 0xC000},    // 70003EC4 
{0x0F12, 0xE59F},    // 70003EC6 
{0x0F12, 0xFF1C},    // 70003EC8 
{0x0F12, 0xE12F},    // 70003ECA 
{0x0F12, 0xFD21},    // 70003ECC 
{0x0F12, 0x0000},    // 70003ECE 
{0x0F12, 0x4778},    // 70003ED0 
{0x0F12, 0x46C0},    // 70003ED2 
{0x0F12, 0xC000},    // 70003ED4 
{0x0F12, 0xE59F},    // 70003ED6 
{0x0F12, 0xFF1C},    // 70003ED8 
{0x0F12, 0xE12F},    // 70003EDA 
{0x0F12, 0x5027},    // 70003EDC 
{0x0F12, 0x0000},    // 70003EDE 
{0x0F12, 0x4778},    // 70003EE0 
{0x0F12, 0x46C0},    // 70003EE2 
{0x0F12, 0xC000},    // 70003EE4 
{0x0F12, 0xE59F},    // 70003EE6 
{0x0F12, 0xFF1C},    // 70003EE8 
{0x0F12, 0xE12F},    // 70003EEA 
{0x0F12, 0x04C9},    // 70003EEC 
{0x0F12, 0x0000},    // 70003EEE 
// End of Patch Data(Last : 70003EEEh)
// Total Size 2500 (0x09C4)
// Addr : 352C , Size : 2498(9C2h) 
{0x1000, 0x0001},

// following TnP was included
// USERMBCV_CONTROL		0x00000001
// --> "M_oif_usELFMaxPacketSize" This register was used for MBCV TnP
// AWB_MODUL_COMP			0x00000002

// VE_GROUP  [[ // 
{0x0028, 0xD000},
{0x002A, 0x1082},
{0x0F12, 0x02AA}, // 6ma
{0x0F12, 0x02AA},
{0x002A, 0x1088},
{0x0F12, 0x0AAA},
// VE_GROUP ]]

//MBCV Control
{0x0028, 0x7000},
{0x002A, 0x04B4},
{0x0F12, 0x0064},

{0x002A, 0x04B6},  //single frame
{0x0F12, 0x0001},

// AFIT by Normalized Brightness Tuning parameter
{0x0028, 0x7000},
{0x002A, 0x3302},
{0x0F12, 0x0000},	// on/off AFIT by NB option

{0x0F12, 0x0005},	//0014	// NormBR[0]
{0x0F12, 0x0019},	//00D2	// NormBR[1]
{0x0F12, 0x0050},	//0384	// NormBR[2]
{0x0F12, 0x0300},	//07D0	// NormBR[3]
{0x0F12, 0x0375},	//1388	// NormBR[4]

// flash
{0x002A, 0x3F82},
{0x0F12, 0x0000},		// TNP_Regs_PreflashStart
{0x0F12, 0x0000},		// TNP_Regs_PreflashEnd
{0x0F12, 0x0260},		// TNP_Regs_PreWP_r
{0x0F12, 0x0240},		// TNP_Regs_PreWP_b   		0

{0x002A, 0x3F98},		// BR Tuning
{0x0F12, 0x0100},		// TNP_Regs_BrRatioIn_0_
{0x0F12, 0x0150},
{0x0F12, 0x0200},
{0x0F12, 0x0300},
{0x0F12, 0x0400},

{0x0F12, 0x0100},		// TNP_Regs_BrRatioOut_0_
{0x0F12, 0x00A0},
{0x0F12, 0x0080},
{0x0F12, 0x0040},
{0x0F12, 0x0020},

{0x0F12, 0x0030},		// WP Tuning
{0x0F12, 0x0040},		// TNP_Regs_WPThresTbl_0_
{0x0F12, 0x0048},
{0x0F12, 0x0050},
{0x0F12, 0x0060},

{0x0F12, 0x0100},		// TNP_Regs_WPWeightTbl_0_
{0x0F12, 0x00C0},
{0x0F12, 0x0080},
{0x0F12, 0x000A},
{0x0F12, 0x0000},

{0x0F12, 0x0120},		// T_BR tune	
{0x0F12, 0x0150},		// TNP_Regs_FlBRIn_0_
{0x0F12, 0x0200},

{0x0F12, 0x003C},		//TNP_Regs_FlBRInOut_0_
{0x0F12, 0x003B},
{0x0F12, 0x002C},               //0030

{0x002A, 0x0430},		//REG_TC_FLS_Mode
{0x0F12, 0x0002},		
{0x002A, 0x3F80},		//TNP_Regs_FastFlashAlg
{0x0F12, 0x0000},

{0x002A, 0x165E},
{0x0F12, 0x0245},  //AWB R point //0245 0258
{0x0F12, 0x0235},  //AWB B point //0245 0245


///////////////////////////////////////////////////////////////////////////////////
// Analog & APS settings///////////////////////////////////////////////////////////
// This register is for FACTORY ONLY. If you change it without prior notification//
// YOU are RESPONSIBLE for the FAILURE that will happen in the future//////////////
///////////////////////////////////////////////////////////////////////////////////

//========================================================================================
// 5CC EVT0 analog register setting
// '10.07.14. Initial Draft
// '10.07.24. 0xE404=0000 -> 1FC0 (Depedestal 0 -> -64d)
// '10.08.16. 0xF410=0001 -> 0000 (for SHBN)
// '10.08.25. 0xF438=0020 -> 0002 (VTGSL=2.96V) by APS
//            0xF43A=0020 -> 0001 (VRG=2.83V) by APS
// '10.09.28. 0xF402=1F02 -> 3F02 ([13]: pixel bias powerdown according to HADR) for Darkshading
//		    0xF416=0000 -> 0001 (AAC_EN enable) for Darkshading
//========================================================================================

//============================= Analog & APS Control =====================================
{0x0028, 0xD000},
{0x002A, 0xF2AC},
{0x0F12, 0x0100},	// analog gain; 0200 x16, 0100 x8, 0080 x4, 0040 x2, 0020 x1
{0x002A, 0xF400},
{0x0F12, 0x001D},	// ldb_en[4], ld_en[3], clp_en[2](N/A), smp_en[1], dshut_en[0]
{0x0F12, 0x3F02},	// cds_test[15:0]; refer to the ATOP_TEST_INFORMATION.

{0x002A, 0xF40A},
{0x0F12, 0x0054},	// adc_sat[7:0]=84d (500mV)
{0x0F12, 0x0002},	// ms[2:0]; 2h@Normal, 2h@PLA, 1h@CNT.AVG
{0x0F12, 0x0008},	// rmp_option[7:0]; [3]SL_Low_PWR_SAVE On
{0x0F12, 0x0000},	// msoff_en; No MS if gain gain is lower than x2
{0x0F12, 0x00A4},	// rmp_init[7:0]

{0x002A, 0xF416},
{0x0F12, 0x0001},	// dbs_option[11:4], dbs_mode[3:2], dbs_bist_en[1], aac_en[0]

{0x002A, 0xF41E},
{0x0F12, 0x0065},	// comp2_bias[7:4], comp1_bias[3:0]

{0x002A, 0xF422},
{0x0F12, 0x0005},	// pix_bias[3:0]

{0x002A, 0xF426},
{0x0F12, 0x00D4},	// clp_lvl[7:0]

{0x002A, 0xF42A},
{0x0F12, 0x0001},	// ref_option[7:0]; [4]OB_PIX monit en, [3]Clamp monit en, [2]Monit amp en, [1]Clamp power-down, [0]CDS power-down during SL=low

{0x002A, 0xF42E},
{0x0F12, 0x0406},	// fb_lv[11:10], pd_fblv[9], capa_ctrl_en[8], pd_inrush_ctrl[7], pd_reg_ntg[6], pd_reg_tgsl[5], pd_reg_rg[4], pd_reg_pix[3], pd_ncp_rosc[2], pd_cp_rosc[1], pd_cp[0]

{0x002A, 0xF434},
{0x0F12, 0x0003},	// dbr_clk_sel[1:0]; PLL_mode=3h, ROSC_mode=0h
{0x0F12, 0x0004},	// reg_tune_pix[7:0]
{0x0F12, 0x0002},	// reg_tune_tgsl[7:0] (2.96V)
{0x0F12, 0x0001},	// reg_tune_rg[7:0] (2.83V)
{0x0F12, 0x0004},	// reg_tune_ntg[7:0]

{0x002A, 0xF446},
{0x0F12, 0x0000},	// blst_en_cintr[15:0]

{0x002A, 0xF466},
{0x0F12, 0x0000},	// srx_en[0]

{0x002A, 0x0054},
{0x0F12, 0x0028},	// pll_pd[10](0:enable, 1:disable), div_clk_en[0](0:enable, 1:disable)
{0x0F12, 0x8888},	// div_dbr[7:4]

{0x002A, 0xF132},
{0x0F12, 0x0206},	// tgr_frame_decription 4
{0x002A, 0xF152},
{0x0F12, 0x0206},	// tgr_frame_decription 7
{0x002A, 0xF1A2},
{0x0F12, 0x0200},	// tgr_frame_params_descriptor_3
{0x002A, 0xF1B2},
{0x0F12, 0x0202},	// tgr_frame_params_descriptor_6
//===========================================================================================

//============================= Line-ADLC Tuning ============================================
{0x002A, 0xE42E},
{0x0F12, 0x0004},	// adlc_qec[2:0]
//===========================================================================================

//============================= Senor BPR ===================================================
{0x002A, 0xE304},
{0x0F12, 0x01CC},	// active_bpr_en[11:11], active_bpr_thresh[10:0]                         
//===================================================================

// AWB white locus setting - Have to be written after TnP
//===================================================================
{0x0028, 0x7000},
{0x002A, 0x1014},
{0x0F12, 0x0132},	//0138	//awbb_IntcR
{0x0F12, 0x010A},	//011C	//awbb_IntcB

//===================================================================
// AF
//===================================================================
//1. AF interface setting
{0x002A, 0x01A2},
{0x0F12, 0x0003}, //REG_TC_IPRM_CM_Init_AfModeType            // VCM_I2C actuator
{0x0F12, 0x0000}, //REG_TC_IPRM_CM_Init_PwmConfig1           // No PWM
{0x0F12, 0x0000}, //REG_TC_IPRM_CM_Init_PwmConfig2
{0x0F12, 0x0041}, //REG_TC_IPRM_CM_Init_GpioConfig1            // Use GPIO_4 for enable port
{0x0F12, 0x0000}, //REG_TC_IPRM_CM_Init_GpioConfig2
{0x0F12, 0x2A0C}, //REG_TC_IPRM_CM_Init_Mi2cBits            // Use GPIO_5 for SCL, GPIO_6 for SDA
{0x0F12, 0x0190}, //REG_TC_IPRM_CM_Init_Mi2cRateKhz            // MI2C Speed : 400KHz

//2. AF window setting
{0x002A, 0x022C},
{0x0F12, 0x0100},	//REG_TC_AF_FstWinStartX 
{0x0F12, 0x00E3},	//REG_TC_AF_FstWinStartY
{0x0F12, 0x0200},	//REG_TC_AF_FstWinSizeX 
{0x0F12, 0x0238},	//REG_TC_AF_FstWinSizeY
{0x0F12, 0x018C},	//REG_TC_AF_ScndWinStartX
{0x0F12, 0x0166},	//REG_TC_AF_ScndWinStartY
{0x0F12, 0x00E6},	//REG_TC_AF_ScndWinSizeX
{0x0F12, 0x0132},	//REG_TC_AF_ScndWinSizeY
{0x0F12, 0x0001},	//REG_TC_AF_WinSizesUpdated

//3. AF Fine Search Settings
{0x002A, 0x063A},
{0x0F12, 0x00C0}, //#skl_af_StatOvlpExpFactor
{0x002A, 0x064A},
{0x0F12, 0x0001}, //#skl_af_bAfStatOff
{0x002A, 0x1488},
{0x0F12, 0x0000}, //#af_search_usAeStable
{0x002A, 0x1494},
{0x0F12, 0x1002},	//#af_search_usSingleAfFlags, 1000- fine search disable, 1002- fine search on
{0x002A, 0x149E},
{0x0F12, 0x0003}, //#af_search_usFinePeakCount
{0x0F12, 0x0000}, //#af_search_usFineMaxScale
{0x002A, 0x142C},
{0x0F12, 0x0602},	//#af_pos_usFineStepNumSize

//4.  AF Peak Threshold Setting
{0x002A, 0x1498},
{0x0F12, 0x0003},	//#af_search_usMinPeakSamples
{0x002A, 0x148A},
{0x0F12, 0x00CC},	//#af_search_usPeakThr  for 80%
{0x0F12, 0x00A0}, //#af_search_usPeakThrLow

//5.  AF Default Position
{0x002A, 0x1420},
{0x0F12, 0x0000},	//#af_pos_usHomePos
{0x0F12, 0x9600},	//#af_pos_usLowConfPos

//6. AF statistics
{0x002A, 0x14B4},
{0x0F12, 0x0280}, //#af_search_usConfThr_4_  LowEdgeBoth GRAD
{0x002A, 0x14C0},
{0x0F12, 0x03A0}, //#af_search_usConfThr_10_ LowLight HPF
{0x0F12, 0x0320}, //#af_search_usConfThr_11_
{0x002A, 0x14F4},
{0x0F12, 0x0030}, //#af_stat_usMinStatVal
{0x002A, 0x1514},
{0x0F12, 0x0060}, //#af_scene_usSceneLowNormBrThr
// AF Scene Settings
{0x002A, 0x151E},
{0x0F12, 0x0003}, //#af_scene_usSaturatedScene

//7. AF Lens Position Table Settings
{0x002A, 0x1434},
{0x0F12, 0x0010},  //#af_pos_usTableLastInd, 10h + 1h = 17 Steps 

{0x0F12, 0x0030},  //#af_pos_usTable_0_  48  
{0x0F12, 0x0033},  //#af_pos_usTable_1_  51   
{0x0F12, 0x0036},  //#af_pos_usTable_2_  54	
{0x0F12, 0x0039},  //#af_pos_usTable_3_  57	
{0x0F12, 0x003D},  //#af_pos_usTable_4_  61   
{0x0F12, 0x0041},  //#af_pos_usTable_5_  65	
{0x0F12, 0x0045},  //#af_pos_usTable_6_  69	
{0x0F12, 0x0049},  //#af_pos_usTable_7_  73	
{0x0F12, 0x004F},  //#af_pos_usTable_8_  79	
{0x0F12, 0x0055},  //#af_pos_usTable_9_  85	
{0x0F12, 0x005D},  //#af_pos_usTable_10_ 93	
{0x0F12, 0x0065},  //#af_pos_usTable_11_ 101	
{0x0F12, 0x006D},  //#af_pos_usTable_12_ 109	
{0x0F12, 0x0077},  //#af_pos_usTable_13_ 119	
{0x0F12, 0x0083},  //#af_pos_usTable_14_ 131	
{0x0F12, 0x008F},  //#af_pos_usTable_15_ 143	
{0x0F12, 0x0096},  //#af_pos_usTable_16_ 150

//8. VCM AF driver with PWM/I2C 
{0x002A, 0x1558},
{0x0F12, 0x8000}, //#afd_usParam[0]  I2C power down command
{0x0F12, 0x0006}, //#afd_usParam[1]  Position Right Shift 
{0x0F12, 0x3FF0}, //#afd_usParam[2]  I2C Data Mask
{0x0F12, 0x03E8}, //#afd_usParam[3]  PWM Period
{0x0F12, 0x0000}, //#afd_usParam[4]  PWM Divider
{0x0F12, 0x0020}, //#afd_usParam[5]  SlowMotion Delay    reduce lens collision noise.
{0x0F12, 0x0010}, //#afd_usParam[6]  SlowMotion Threshold
{0x0F12, 0x0008}, //#afd_usParam[7]  Signal Shaping
{0x0F12, 0x0040}, //#afd_usParam[8]  Signal Shaping level 
{0x0F12, 0x0080}, //#afd_usParam[9]  Signal Shaping level
{0x0F12, 0x00C0}, //#afd_usParam[10] Signal Shaping level
{0x0F12, 0x00E0}, //#afd_usParam[11] Signal Shaping level

{0x002A, 0x0224},
{0x0F12, 0x0003},	//REG_TC_AF_AfCmd	//Initialize AF subsystem (AF driver, AF algorithm)

//===================================================================
// Flash setting
//===================================================================
{0x002A, 0x018C},
{0x0F12, 0x0001},	//REG_TC_IPRM_AuxConfig	// bit[0] : Flash is in use, bit[1] : Mechanical shutter is in use // 0 : do not use, 1 : use
{0x0F12, 0x0003},	//REG_TC_IPRM_AuxPolarity	// bit[0] : Flash polarity (1 is active high), bit[1] : Mechanical shutter polarity (1 is active high)
{0x0F12, 0x0003},	//REG_TC_IPRM_AuxGpios	//1-4 : Flash GPIO number, If GPIO number is overaped with AF GPIO, F/W could be stop.

//===================================================================
// 1-H timing setting
//===================================================================
{0x002A, 0x1686},
{0x0F12, 0x005C},	//senHal_uAddColsBin
{0x0F12, 0x005C},	//senHal_uAddColsNoBin
{0x0F12, 0x005C},	//senHal_uMinColsHorBin
{0x0F12, 0x005C},	//senHal_uMinColsNoHorBin
{0x0F12, 0x025A},	//senHal_uMinColsAddAnalogBin

//===================================================================
// Forbidden area setting
//===================================================================
{0x002A, 0x1844},
{0x0F12, 0x0000},	//senHal_bSRX    //SRX off

{0x002A, 0x1680},
{0x0F12, 0x0002},	//senHal_NExpLinesCheckFine	//0004    //extend Forbidden area line

//===================================================================
// Preview subsampling mode
//===================================================================
{0x002A, 0x18F8},
{0x0F12, 0x0001},	//senHal_bAACActiveWait2Start
{0x002A, 0x18F6},
{0x0F12, 0x0001},	//senHal_bAlwaysAAC
{0x002A, 0x182C},
{0x0F12, 0x0001},	//senHal_bSenAAC
{0x002A, 0x0EE4},
{0x0F12, 0x0001},	//setot_bUseDigitalHbin
{0x002A, 0x1674},
{0x0F12, 0x0002},	//senHal_SenBinFactor	// 2:2x2, 4:4x4
{0x0F12, 0x0002},	//senHal_SamplingType	// 0:Full, 1:digital, 2:PLA, 3:CA
{0x0F12, 0x0000},	//senHal_SamplingMode	// 0:2x2,	1:4x4

//===================================================================
// PLL setting for Max frequency (EVT0.1) 2010.08.05 - Do not remove
//===================================================================
{0x002A, 0x19AE},
{0x0F12, 0xEA60},	//pll_uMaxSysFreqKhz
{0x0F12, 0x7530},	//pll_uMaxPVIFreq4KH
{0x002A, 0x19C2},
{0x0F12, 0x7530},	//pll_uMaxMIPIFreq4KH
{0x002A, 0x0244},
{0x0F12, 0x7530},	//REG_0TC_PCFG_usMaxOut4KHzRate
{0x002A, 0x0336},
{0x0F12, 0x7530},	//REG_0TC_CCFG_usMaxOut4KHzRate

//===================================================================
// Init Parameters
//===================================================================
//MCLK
{0x002A, 0x0188},
{0x0F12, 0x5DC0},	//REG_TC_IPRM_InClockLSBs
{0x0F12, 0x0000},	//REG_TC_IPRM_InClockMSBs
{0x002A, 0x01B2},
{0x0F12, 0x0003},	//REG_TC_IPRM_UseNPviClocks
{0x0F12, 0x0000},	//REG_TC_IPRM_UseNMipiClocks
{0x002A, 0x01B8},
{0x0F12, 0x0000},	//REG_TC_IPRM_bBlockInternalPllCalc	//1:pll bypass

//SCLK & PCLK
{0x0F12, 0x32C8},	//REG_TC_IPRM_OpClk4KHz_0	//52Mhz	//2EE0	//48Mhz 1F40	//32MHz
{0x0F12, 0x34BC},	//REG_TC_IPRM_MinOutRate4KHz_0	//53.936Mhz	//32A8	//51.872MHz
{0x0F12, 0x34BC},	//REG_TC_IPRM_MaxOutRate4KHz_0	//54.064Mhz	//32E8	//52.128MHz

//SCLK & PCLK
{0x0F12, 0x1F40},	//REG_TC_IPRM_OpClk4KHz_1	//52Mhz	//2EE0	//48Mhz 1F40	//32MHz
{0x0F12, 0x34BC},	//REG_TC_IPRM_MinOutRate4KHz_1	//53.936Mhz	//32A8	//51.872MHz
{0x0F12, 0x34BC},	//REG_TC_IPRM_MaxOutRate4KHz_1	//54.064Mhz	//32E8	//52.128MHz

//SCLK & PCLK
{0x0F12, 0x32C8},	//REG_TC_IPRM_OpClk4KHz_0	//52Mhz	//2EE0	//48Mhz 1F40	//32MHz
{0x0F12, 0x5208},	//REG_TC_IPRM_MinOutRate4KHz_2	//83.936Mhz	//32A8	//51.872MHz
{0x0F12, 0x5208},	//REG_TC_IPRM_MaxOutRate4KHz_2	//84.064Mhz	//32E8	//52.128MHz

{0x002A, 0x01CC},
{0x0F12, 0x0001},	//REG_TC_IPRM_InitParamsUpdated

//===================================================================
// JPEG Thumbnail Setting
//===================================================================
{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive			Thumbnail Enable
{0x0F12, 0x0140},		//REG_TC_THUMB_Thumb_uWidth				Thumbnail Width //320
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight			Thumbnail Height //240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format				Thumbnail Output Format 5:YUV

//===================================================================
// Input Width & Height
//===================================================================
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1536
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 0
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1536
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 0

{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x043C},
{0x0F12, 0x0800},	//REG_TC_PZOOM_ZoomInputWidth
{0x0F12, 0x0600},	//REG_TC_PZOOM_ZoomInputHeight
{0x0F12, 0x0000},	//REG_TC_PZOOM_ZoomInputWidthOfs
{0x0F12, 0x0000},	//REG_TC_PZOOM_ZoomInputHeightOfs

//===================================================================
// Preview 0 1024x768 system 52M PCLK 54M
//===================================================================
{0x002A, 0x023E},
{0x0F12, 0x0400},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x0300},	//REG_0TC_PCFG_usHeight
{0x0F12, 0x0005},	//REG_0TC_PCFG_Format
{0x0F12, 0x34BC},	//REG_0TC_PCFG_usMaxOut4KHzRate
{0x0F12, 0x34BC},	//REG_0TC_PCFG_usMinOut4KHzRate

{0x002A, 0x024C},
{0x0F12, 0x0052},	//REG_0TC_PCFG_PVIMask
{0x0F12, 0x0010},	//REG_0TC_PCFG_OIFMask

{0x002A, 0x0254},
{0x0F12, 0x0000},	//REG_0TC_PCFG_uClockInd
{0x0F12, 0x0000},	//REG_0TC_PCFG_usFrTimeType
{0x0F12, 0x0001},	//REG_0TC_PCFG_FrRateQualityType
{0x0F12, 0x03E8},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x014E},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
{0x0F12, 0x0000},	//REG_0TC_PCFG_uPrevMirror
{0x0F12, 0x0000},	//REG_0TC_PCFG_uCaptureMirror
{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation

//===================================================================
// Preview 2 1024x768 system 52M PCLK 54M
//===================================================================
{0x002A, 0x029E},
{0x0F12, 0x0400},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x0300},	//REG_0TC_PCFG_usHeight
{0x0F12, 0x0005},	//REG_0TC_PCFG_Format
{0x0F12, 0x34BC},	//REG_0TC_PCFG_usMaxOut4KHzRate
{0x0F12, 0x34BC},	//REG_0TC_PCFG_usMinOut4KHzRate

{0x002A, 0x02AC},
{0x0F12, 0x0052},	//REG_0TC_PCFG_PVIMask
{0x0F12, 0x0010},	//REG_0TC_PCFG_OIFMask

{0x002A, 0x02B4},
{0x0F12, 0x0000},	//REG_0TC_PCFG_uClockInd
{0x0F12, 0x0000},	//REG_0TC_PCFG_usFrTimeType
{0x0F12, 0x0001},	//REG_0TC_PCFG_FrRateQualityType
{0x0F12, 0x09C4},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x014E},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
{0x0F12, 0x0000},	//REG_0TC_PCFG_uPrevMirror
{0x0F12, 0x0000},	//REG_0TC_PCFG_uCaptureMirror
{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation


//===================================================================
// Capture 0 2048x1536 system 52M PCLK 54M
//===================================================================

{0x002A, 0x032E},
{0x0F12, 0x0000},	//REG_0TC_CCFG_uCaptureMode

{0x0F12, 0x0800},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0600},	//REG_0TC_CCFG_usHeight
{0x0F12, 0x0009},	//REG_0TC_CCFG_Format
{0x0F12, 0x34BC},	//REG_0TC_CCFG_usMaxOut4KHzRate
{0x0F12, 0x34BC},	//REG_0TC_CCFG_usMinOut4KHzRate

{0x002A, 0x033E},
{0x0F12, 0x0052},	//REG_0TC_CCFG_PVIMask [1]:PCLK Inversion, [4]:UV First, [5]:V First
{0x0F12, 0x0010},	//REG_0TC_CCFG_OIFMask
{0x0F12, 0x03C0},	//REG_0TC_CCFG_usJpegPacketSize

{0x002A, 0x0346},
{0x0F12, 0x0000},	//REG_0TC_CCFG_uClockInd
{0x0F12, 0x0000},	//REG_0TC_CCFG_usFrTimeType
{0x0F12, 0x0002},	//REG_0TC_CCFG_FrRateQualityType
{0x0F12, 0x0535},	//REG_0TC_CCFG_usMaxFrTimeMsecMult10
{0x0F12, 0x029A},	//REG_0TC_CCFG_usMinFrTimeMsecMult10
{0x0F12, 0x0000},	//REG_0TC_CCFG_bSmearOutput
{0x0F12, 0x0000},	//REG_0TC_CCFG_sSaturation
{0x0F12, 0x0000},	//REG_0TC_CCFG_sSharpBlur
{0x0F12, 0x0000},	//REG_0TC_CCFG_sColorTemp
{0x0F12, 0x0000},	//REG_0TC_CCFG_uDeviceGammaIndex

//===================================================================
// Capture 2 2048x1536 system 52M PCLK 54M
//===================================================================

{0x002A, 0x0386},
{0x0F12, 0x0000},	//REG_0TC_CCFG_uCaptureMode

{0x0F12, 0x0800},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0600},	//REG_0TC_CCFG_usHeight
{0x0F12, 0x0009},	//REG_0TC_CCFG_Format
{0x0F12, 0x34BC},	//REG_0TC_CCFG_usMaxOut4KHzRate
{0x0F12, 0x34BC},	//REG_0TC_CCFG_usMinOut4KHzRate

{0x002A, 0x0396},
{0x0F12, 0x0052},	//REG_0TC_CCFG_PVIMask
{0x0F12, 0x0010},	//REG_0TC_CCFG_OIFMask
{0x0F12, 0x03C0},	//REG_0TC_CCFG_usJpegPacketSize

{0x002A, 0x039E},
{0x0F12, 0x0000},	//REG_0TC_CCFG_uClockInd
{0x0F12, 0x0000},	//REG_0TC_CCFG_usFrTimeType
{0x0F12, 0x0002},	//REG_0TC_CCFG_FrRateQualityType
{0x0F12, 0x0535},	//REG_0TC_CCFG_usMaxFrTimeMsecMult10
{0x0F12, 0x029A},	//REG_0TC_CCFG_usMinFrTimeMsecMult10
{0x0F12, 0x0000},	//REG_0TC_CCFG_bSmearOutput
{0x0F12, 0x0000},	//REG_0TC_CCFG_sSaturation
{0x0F12, 0x0000},	//REG_0TC_CCFG_sSharpBlur
{0x0F12, 0x0000},	//REG_0TC_CCFG_sColorTemp
{0x0F12, 0x0000},	//REG_0TC_CCFG_uDeviceGammaIndex

{0x002A, 0x0426},
{0x0F12, 0x005F},	//REG_TC_BRC_usCaptureQuality


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//PREVIEW
{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged

{0xffff, 0x0064}, //p100	//Delay 100ms


//===================================================================
// AFC
//===================================================================
//Auto
{0x002A, 0x0F08},
{0x0F12, 0x0001},	//AFC_Default60Hz   01:60hz 00:50Hz
{0x002A, 0x04A4},
{0x0F12, 0x067F},	//REG_TC_DBG_AutoAlgEnBits, 065f : Manual AFC on   067f : Manual AFC off

//===================================================================
// Shading (AF module)
//===================================================================
// TVAR_ash_pGAS_high
{0x002A, 0x0D22},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F0F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x0F00},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F0F},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0F00},
{0x0F12, 0x0000},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x000F},
{0x0F12, 0x0F00},

// TVAR_ash_pGAS_low
{0x0F12, 0x6CA9},
{0x0F12, 0xF136},
{0x0F12, 0x06A1},
{0x0F12, 0xFE54},
{0x0F12, 0x097B},
{0x0F12, 0xF52E},
{0x0F12, 0xF030},
{0x0F12, 0xF536},
{0x0F12, 0x0783},
{0x0F12, 0xFE44},
{0x0F12, 0x00E9},
{0x0F12, 0xFF3E},
{0x0F12, 0xFCE5},
{0x0F12, 0x0A39},
{0x0F12, 0xFAB6},
{0x0F12, 0xFC00},
{0x0F12, 0x057D},
{0x0F12, 0xFFB6},
{0x0F12, 0x12DB},
{0x0F12, 0xFB17},
{0x0F12, 0x04D5},
{0x0F12, 0x05C6},
{0x0F12, 0xF896},
{0x0F12, 0xFC4C},
{0x0F12, 0xFCCB},
{0x0F12, 0xFB96},
{0x0F12, 0xFBF8},
{0x0F12, 0x0044},
{0x0F12, 0xFCE8},
{0x0F12, 0x0F81},
{0x0F12, 0xF3F5},
{0x0F12, 0x0BE9},
{0x0F12, 0xFBD3},
{0x0F12, 0x00A1},
{0x0F12, 0x0787},
{0x0F12, 0xEE76},
{0x0F12, 0x814D},
{0x0F12, 0xEF4C},
{0x0F12, 0x037D},
{0x0F12, 0x059F},
{0x0F12, 0x047B},
{0x0F12, 0xF588},
{0x0F12, 0xF15C},
{0x0F12, 0xE1E7},
{0x0F12, 0x1C1D},
{0x0F12, 0xECFA},
{0x0F12, 0x0A8E},
{0x0F12, 0x00D5},
{0x0F12, 0xEDF1},
{0x0F12, 0x1FBC},
{0x0F12, 0xE844},
{0x0F12, 0x054F},
{0x0F12, 0x0951},
{0x0F12, 0xF14A},
{0x0F12, 0x2B75},
{0x0F12, 0xEAD0},
{0x0F12, 0x0FB1},
{0x0F12, 0x0296},
{0x0F12, 0xEACD},
{0x0F12, 0x1894},
{0x0F12, 0xEE1E},
{0x0F12, 0x0081},
{0x0F12, 0xF821},
{0x0F12, 0x094D},
{0x0F12, 0x014C},
{0x0F12, 0xF42F},
{0x0F12, 0xF2FD},
{0x0F12, 0x1124},
{0x0F12, 0xF9B2},
{0x0F12, 0xF6ED},
{0x0F12, 0x0D98},
{0x0F12, 0xF984},
{0x0F12, 0x6CF4},
{0x0F12, 0xEE3A},
{0x0F12, 0x095D},
{0x0F12, 0xFCE5},
{0x0F12, 0x0895},
{0x0F12, 0xF6C3},
{0x0F12, 0xF373},
{0x0F12, 0xF0DC},
{0x0F12, 0x0E81},
{0x0F12, 0xF5E9},
{0x0F12, 0x0527},
{0x0F12, 0x00DB},
{0x0F12, 0xF953},
{0x0F12, 0x14F2},
{0x0F12, 0xEC1B},
{0x0F12, 0x0CD5},
{0x0F12, 0xF9ED},
{0x0F12, 0xFFAB},
{0x0F12, 0x1BD3},
{0x0F12, 0xEC85},
{0x0F12, 0x1308},
{0x0F12, 0xF7AF},
{0x0F12, 0xFCE5},
{0x0F12, 0x04E6},
{0x0F12, 0xEF89},
{0x0F12, 0x0598},
{0x0F12, 0xF28A},
{0x0F12, 0x0900},
{0x0F12, 0x0165},
{0x0F12, 0x002B},
{0x0F12, 0xFBA3},
{0x0F12, 0x09E3},
{0x0F12, 0x0045},
{0x0F12, 0xFB3D},
{0x0F12, 0x04E4},
{0x0F12, 0xF70E},
{0x0F12, 0x702E},
{0x0F12, 0xEEA1},
{0x0F12, 0x083A},
{0x0F12, 0xFE09},
{0x0F12, 0x0957},
{0x0F12, 0xF4EA},
{0x0F12, 0xF15F},
{0x0F12, 0xF40B},
{0x0F12, 0x09A2},
{0x0F12, 0xFBC5},
{0x0F12, 0x0007},
{0x0F12, 0x0332},
{0x0F12, 0xFCCD},
{0x0F12, 0x0D2A},
{0x0F12, 0xF2CB},
{0x0F12, 0x072C},
{0x0F12, 0x009C},
{0x0F12, 0xFB04},
{0x0F12, 0x1948},
{0x0F12, 0xF545},
{0x0F12, 0x0CE1},
{0x0F12, 0xF7B3},
{0x0F12, 0x0043},
{0x0F12, 0x023A},
{0x0F12, 0xF177},
{0x0F12, 0x003D},
{0x0F12, 0xFC39},
{0x0F12, 0x040C},
{0x0F12, 0xFB2A},
{0x0F12, 0x08E8},
{0x0F12, 0xFA4B},
{0x0F12, 0x0BCE},
{0x0F12, 0xF4F6},
{0x0F12, 0x08C3},
{0x0F12, 0x0051},
{0x0F12, 0xF4BE},

{0x002A, 0x04A8},
{0x0F12, 0x0001},	//REG_TC_DBG_ReInitCmd

//===================================================================
// Shading - Alpha
//===================================================================
{0x002A, 0x07E8},
{0x0F12, 0x00BC},	//TVAR_ash_AwbAshCord_0_	//HOR
{0x0F12, 0x00ED},	//TVAR_ash_AwbAshCord_1_	//INCA
{0x0F12, 0x0101},	//TVAR_ash_AwbAshCord_2_	//WW
{0x0F12, 0x012D},	//TVAR_ash_AwbAshCord_3_	//CW
{0x0F12, 0x0166},	//TVAR_ash_AwbAshCord_4_	//D50
{0x0F12, 0x0184},	//TVAR_ash_AwbAshCord_5_	//D65
{0x0F12, 0x01A0},	//TVAR_ash_AwbAshCord_6_	//D75

{0x002A, 0x07FE},
{0x0F12, 0x3A00},	//4000	//TVAR_ash_GASAlpha_0__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_0__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_0__2_
{0x0F12, 0x3C00},	//4000	//TVAR_ash_GASAlpha_0__3_
{0x0F12, 0x3A00},	//4000	//TVAR_ash_GASAlpha_1__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_1__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_1__2_
{0x0F12, 0x3C00},	//4000	//TVAR_ash_GASAlpha_1__3_
{0x0F12, 0x3A00},	//4000	//TVAR_ash_GASAlpha_2__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_2__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_2__2_
{0x0F12, 0x3C00},	//4000	//TVAR_ash_GASAlpha_2__3_
{0x0F12, 0x3A00},	//TVAR_ash_GASAlpha_3__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_3__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_3__2_
{0x0F12, 0x3C00},	//4000	//TVAR_ash_GASAlpha_3__3_
{0x0F12, 0x3A00},	//TVAR_ash_GASAlpha_4__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_4__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_4__2_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_4__3_
{0x0F12, 0x3A00},	//TVAR_ash_GASAlpha_5__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_5__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_5__2_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_5__3_
{0x0F12, 0x3A00},	//TVAR_ash_GASAlpha_6__0_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_6__1_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_6__2_
{0x0F12, 0x4000},	//TVAR_ash_GASAlpha_6__3_

{0x002A, 0x0836},
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_0_
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_1_
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_2_
{0x0F12, 0x4000},	//TVAR_ash_GASOutdoorAlpha_3_

//===================================================================
// Gamma
//===================================================================
//	param_start	SARR_usGammaLutRGBIndoor
{0x002A, 0x0660},
{0x0F12, 0x0000}, //0000 //0000	//0000	//saRR_usDualGammaLutRGBIndoor[0][0]
{0x0F12, 0x0004}, //0008 //0002	//0008	//saRR_usDualGammaLutRGBIndoor[0][1]
{0x0F12, 0x0016}, //0013 //0003	//0013	//saRR_usDualGammaLutRGBIndoor[0][2]
{0x0F12, 0x0045}, //002C //0008	//002C	//saRR_usDualGammaLutRGBIndoor[0][3]
{0x0F12, 0x0088}, //0062 //0048	//0062	//saRR_usDualGammaLutRGBIndoor[0][4]
{0x0F12, 0x00E8}, //00CD //00CD	//00CD	//saRR_usDualGammaLutRGBIndoor[0][5]
{0x0F12, 0x0130}, //0129 //0129	//0129	//saRR_usDualGammaLutRGBIndoor[0][6]
{0x0F12, 0x0153}, //0151 //0151	//0151	//saRR_usDualGammaLutRGBIndoor[0][7]
{0x0F12, 0x0174}, //0174 //0174	//0174	//saRR_usDualGammaLutRGBIndoor[0][8]
{0x0F12, 0x01AA}, //01AA //01AA	//01AA	//saRR_usDualGammaLutRGBIndoor[0][9]
{0x0F12, 0x01D7}, //01D7 //01D7	//01D7	//saRR_usDualGammaLutRGBIndoor[0][10]
{0x0F12, 0x01FE}, //01FE //01FE	//01FE	//saRR_usDualGammaLutRGBIndoor[0][11]
{0x0F12, 0x0221}, //0221 //0221	//0221	//saRR_usDualGammaLutRGBIndoor[0][12]
{0x0F12, 0x0252}, //0252 //025D	//025D	//saRR_usDualGammaLutRGBIndoor[0][13]
{0x0F12, 0x0281}, //0281 //0291	//0291	//saRR_usDualGammaLutRGBIndoor[0][14]
{0x0F12, 0x02E1}, //02E1 //02EB	//02EB	//saRR_usDualGammaLutRGBIndoor[0][15]
{0x0F12, 0x0345}, //0345 //033A	//033A	//saRR_usDualGammaLutRGBIndoor[0][16]
{0x0F12, 0x039C}, //039C //0380	//0380	//saRR_usDualGammaLutRGBIndoor[0][17]
{0x0F12, 0x03D9}, //03D9 //03C2	//03C2	//saRR_usDualGammaLutRGBIndoor[0][18]
{0x0F12, 0x03FF}, //03FF //03FF	//03FF	//saRR_usDualGammaLutRGBIndoor[0][19]
{0x0F12, 0x0000}, //0000 //0000	//0000	//saRR_usDualGammaLutRGBIndoor[1][0]
{0x0F12, 0x0004}, //0008 //0002	//0008	//saRR_usDualGammaLutRGBIndoor[1][1]
{0x0F12, 0x0016}, //0013 //0003	//0013	//saRR_usDualGammaLutRGBIndoor[1][2]
{0x0F12, 0x0045}, //002C //0008	//002C	//saRR_usDualGammaLutRGBIndoor[1][3]
{0x0F12, 0x0088}, //0062 //0048	//0062	//saRR_usDualGammaLutRGBIndoor[1][4]
{0x0F12, 0x00E8}, //00CD //00CD	//00CD	//saRR_usDualGammaLutRGBIndoor[1][5]
{0x0F12, 0x0130}, //0129 //0129	//0129	//saRR_usDualGammaLutRGBIndoor[1][6]
{0x0F12, 0x0153}, //0151 //0151	//0151	//saRR_usDualGammaLutRGBIndoor[1][7]
{0x0F12, 0x0174}, //0174 //0174	//0174	//saRR_usDualGammaLutRGBIndoor[1][8]
{0x0F12, 0x01AA}, //01AA //01AA	//01AA	//saRR_usDualGammaLutRGBIndoor[1][9]
{0x0F12, 0x01D7}, //01D7 //01D7	//01D7	//saRR_usDualGammaLutRGBIndoor[1][10]
{0x0F12, 0x01FE}, //01FE //01FE	//01FE	//saRR_usDualGammaLutRGBIndoor[1][11]
{0x0F12, 0x0221}, //0221 //0221	//0221	//saRR_usDualGammaLutRGBIndoor[1][12]
{0x0F12, 0x0252}, //0252 //025D	//025D	//saRR_usDualGammaLutRGBIndoor[1][13]
{0x0F12, 0x0281}, //0281 //0291	//0291	//saRR_usDualGammaLutRGBIndoor[1][14]
{0x0F12, 0x02E1}, //02E1 //02EB	//02EB	//saRR_usDualGammaLutRGBIndoor[1][15]
{0x0F12, 0x0345}, //0345 //033A	//033A	//saRR_usDualGammaLutRGBIndoor[1][16]
{0x0F12, 0x039C}, //039C //0380	//0380	//saRR_usDualGammaLutRGBIndoor[1][17]
{0x0F12, 0x03D9}, //03D9 //03C2	//03C2	//saRR_usDualGammaLutRGBIndoor[1][18]
{0x0F12, 0x03FF}, //03FF //03FF	//03FF	//saRR_usDualGammaLutRGBIndoor[1][19]
{0x0F12, 0x0000}, //0000 //0000	//0000	//saRR_usDualGammaLutRGBIndoor[2][0]
{0x0F12, 0x0004}, //0008 //0002	//0008	//saRR_usDualGammaLutRGBIndoor[2][1]
{0x0F12, 0x0016}, //0013 //0003	//0013	//saRR_usDualGammaLutRGBIndoor[2][2]
{0x0F12, 0x0045}, //002C //0008	//002C	//saRR_usDualGammaLutRGBIndoor[2][3]
{0x0F12, 0x0088}, //0062 //0048	//0062	//saRR_usDualGammaLutRGBIndoor[2][4]
{0x0F12, 0x00E8}, //00CD //00CD	//00CD	//saRR_usDualGammaLutRGBIndoor[2][5]
{0x0F12, 0x0130}, //0129 //0129	//0129	//saRR_usDualGammaLutRGBIndoor[2][6]
{0x0F12, 0x0153}, //0151 //0151	//0151	//saRR_usDualGammaLutRGBIndoor[2][7]
{0x0F12, 0x0174}, //0174 //0174	//0174	//saRR_usDualGammaLutRGBIndoor[2][8]
{0x0F12, 0x01AA}, //01AA //01AA	//01AA	//saRR_usDualGammaLutRGBIndoor[2][9]
{0x0F12, 0x01D7}, //01D7 //01D7	//01D7	//saRR_usDualGammaLutRGBIndoor[2][10]
{0x0F12, 0x01FE}, //01FE //01FE	//01FE	//saRR_usDualGammaLutRGBIndoor[2][11]
{0x0F12, 0x0221}, //0221 //0221	//0221	//saRR_usDualGammaLutRGBIndoor[2][12]
{0x0F12, 0x0252}, //0252 //025D	//025D	//saRR_usDualGammaLutRGBIndoor[2][13]
{0x0F12, 0x0281}, //0281 //0291	//0291	//saRR_usDualGammaLutRGBIndoor[2][14]
{0x0F12, 0x02E1}, //02E1 //02EB	//02EB	//saRR_usDualGammaLutRGBIndoor[2][15]
{0x0F12, 0x0345}, //0345 //033A	//033A	//saRR_usDualGammaLutRGBIndoor[2][16]
{0x0F12, 0x039C}, //039C //0380	//0380	//saRR_usDualGammaLutRGBIndoor[2][17]
{0x0F12, 0x03D9}, //03D9 //03C2	//03C2	//saRR_usDualGammaLutRGBIndoor[2][18]
{0x0F12, 0x03FF}, //03FF //03FF	//03FF	//saRR_usDualGammaLutRGBIndoor[2][19]

//{0x002A, 0x06D8},
{0x0F12, 0x0000}, //0000	//saRR_usDualGammaLutRGBOutdoor[0][0]
{0x0F12, 0x0008}, //0008	//0008	//saRR_usDualGammaLutRGBOutdoor[0][1]
{0x0F12, 0x0020}, //0013	//0013	//saRR_usDualGammaLutRGBOutdoor[0][2]
{0x0F12, 0x004F}, //002C	//002C	//saRR_usDualGammaLutRGBOutdoor[0][3]
{0x0F12, 0x0094}, //0057	//0062	//saRR_usDualGammaLutRGBOutdoor[0][4]
{0x0F12, 0x00EE}, //00A2	//009A	//saRR_usDualGammaLutRGBOutdoor[0][5]
{0x0F12, 0x0130}, //00F0	//00FD	//saRR_usDualGammaLutRGBOutdoor[0][6]
{0x0F12, 0x014C}, //0118	//0129	//saRR_usDualGammaLutRGBOutdoor[0][7]
{0x0F12, 0x0165}, //013F	//014B	//saRR_usDualGammaLutRGBOutdoor[0][8]
{0x0F12, 0x0196}, //0184	//0184	//saRR_usDualGammaLutRGBOutdoor[0][9]
{0x0F12, 0x01C0}, //01B8	//01B8	//saRR_usDualGammaLutRGBOutdoor[0][10]
{0x0F12, 0x01EA}, //01EA	//01EA	//saRR_usDualGammaLutRGBOutdoor[0][11]
{0x0F12, 0x0213}, //0216	//0216	//saRR_usDualGammaLutRGBOutdoor[0][12]
{0x0F12, 0x025B}, //025E	//025E	//saRR_usDualGammaLutRGBOutdoor[0][13]
{0x0F12, 0x0296}, //0299	//0299	//saRR_usDualGammaLutRGBOutdoor[0][14]
{0x0F12, 0x02F9}, //02F9	//02F9	//saRR_usDualGammaLutRGBOutdoor[0][15]
{0x0F12, 0x0341}, //0341	//0341	//saRR_usDualGammaLutRGBOutdoor[0][16]
{0x0F12, 0x0379}, //0379	//0379	//saRR_usDualGammaLutRGBOutdoor[0][17]
{0x0F12, 0x03B8}, //03B8	//03B8	//saRR_usDualGammaLutRGBOutdoor[0][18]
{0x0F12, 0x03E9}, //03E9	//03E9	//saRR_usDualGammaLutRGBOutdoor[0][19]
{0x0F12, 0x0000}, //0000	//saRR_usDualGammaLutRGBOutdoor[1][0]
{0x0F12, 0x0008}, //0008	//0008	//saRR_usDualGammaLutRGBOutdoor[1][1]
{0x0F12, 0x0020}, //0013	//0013	//saRR_usDualGammaLutRGBOutdoor[1][2]
{0x0F12, 0x004F}, //002C	//002C	//saRR_usDualGammaLutRGBOutdoor[1][3]
{0x0F12, 0x0094}, //0057	//0062	//saRR_usDualGammaLutRGBOutdoor[1][4]
{0x0F12, 0x00EE}, //00A2	//009A	//saRR_usDualGammaLutRGBOutdoor[1][5]
{0x0F12, 0x0130}, //00F0	//00FD	//saRR_usDualGammaLutRGBOutdoor[1][6]
{0x0F12, 0x014C}, //0118	//0129	//saRR_usDualGammaLutRGBOutdoor[1][7]
{0x0F12, 0x0165}, //013F	//014B	//saRR_usDualGammaLutRGBOutdoor[1][8]
{0x0F12, 0x0196}, //0184	//0184	//saRR_usDualGammaLutRGBOutdoor[1][9]
{0x0F12, 0x01C0}, //01B8	//01B8	//saRR_usDualGammaLutRGBOutdoor[1][10]
{0x0F12, 0x01EA}, //01EA	//01EA	//saRR_usDualGammaLutRGBOutdoor[1][11]
{0x0F12, 0x0213}, //0216	//0216	//saRR_usDualGammaLutRGBOutdoor[1][12]
{0x0F12, 0x025B}, //025E	//025E	//saRR_usDualGammaLutRGBOutdoor[1][13]
{0x0F12, 0x0299}, //0299	//0299	//saRR_usDualGammaLutRGBOutdoor[1][14]
{0x0F12, 0x02F9}, //02F9	//02F9	//saRR_usDualGammaLutRGBOutdoor[1][15]
{0x0F12, 0x0341}, //0341	//0341	//saRR_usDualGammaLutRGBOutdoor[1][16]
{0x0F12, 0x0379}, //0379	//0379	//saRR_usDualGammaLutRGBOutdoor[1][17]
{0x0F12, 0x03B8}, //03B8	//03B8	//saRR_usDualGammaLutRGBOutdoor[1][18]
{0x0F12, 0x03E9}, //03E9	//03E9	//saRR_usDualGammaLutRGBOutdoor[1][19]
{0x0F12, 0x0000}, //0000	//saRR_usDualGammaLutRGBOutdoor[2][0]
{0x0F12, 0x0008}, //0008	//0008	//saRR_usDualGammaLutRGBOutdoor[2][1]
{0x0F12, 0x0020}, //0013	//0013	//saRR_usDualGammaLutRGBOutdoor[2][2]
{0x0F12, 0x004F}, //002C	//002C	//saRR_usDualGammaLutRGBOutdoor[2][3]
{0x0F12, 0x0094}, //0057	//0062	//saRR_usDualGammaLutRGBOutdoor[2][4]
{0x0F12, 0x00EE}, //00A2	//009A	//saRR_usDualGammaLutRGBOutdoor[2][5]
{0x0F12, 0x0130}, //00F0	//00FD	//saRR_usDualGammaLutRGBOutdoor[2][6]
{0x0F12, 0x014C}, //0118	//0129	//saRR_usDualGammaLutRGBOutdoor[2][7]
{0x0F12, 0x0165}, //013F	//014B	//saRR_usDualGammaLutRGBOutdoor[2][8]
{0x0F12, 0x0196}, //0184	//0184	//saRR_usDualGammaLutRGBOutdoor[2][9]
{0x0F12, 0x01C0}, //01B8	//01B8	//saRR_usDualGammaLutRGBOutdoor[2][10]
{0x0F12, 0x01EA}, //01EA	//01EA	//saRR_usDualGammaLutRGBOutdoor[2][11]
{0x0F12, 0x0213}, //0216	//0216	//saRR_usDualGammaLutRGBOutdoor[2][12]
{0x0F12, 0x025B}, //025E	//025E	//saRR_usDualGammaLutRGBOutdoor[2][13]
{0x0F12, 0x0299}, //0299	//0299	//saRR_usDualGammaLutRGBOutdoor[2][14]
{0x0F12, 0x02F9}, //02F9	//02F9	//saRR_usDualGammaLutRGBOutdoor[2][15]
{0x0F12, 0x0341}, //0341	//0341	//saRR_usDualGammaLutRGBOutdoor[2][16]
{0x0F12, 0x0379}, //0379	//0379	//saRR_usDualGammaLutRGBOutdoor[2][17]
{0x0F12, 0x03B8}, //03B8	//03B8	//saRR_usDualGammaLutRGBOutdoor[2][18]
{0x0F12, 0x03E9}, //03E9	//03E9	//saRR_usDualGammaLutRGBOutdoor[2][19]

//===================================================================
// AE - shutter
//===================================================================
//****************************************/
// AE 2009 03 08 - based on TN
//****************************************/
//============================================================
// Frame rate setting
//============================================================
// How to set
// 1. Exposure value
// dec2hex((1 / (frame rate you want(ms))) * 100d * 5d)
//
//
// 2. Analog Digital gain
// dec2hex((Analog gain you want) * 256d)
//              Ex1) Simple Caculation for x3.25?:   3.25x256 = 832[dec] = 0340[hex]
//============================================================
//MBR
{0x002A, 0x01DE},
{0x0F12, 0x0000},	//REG_TC_bUseMBR	//MBR off
//MBR off is needed to prevent a shorter integration time when the scene has blurring in Night shot

//AE_Target
{0x002A, 0x1308},
{0x0F12, 0x003E},	//TVAR_ae_BrAve
{0x002A, 0x130E},
{0x0F12, 0x000F},	//ae_StatMode
//ae_StatMode bit[3] BLC has to be bypassed to prevent AE weight change, especially backlight scene

//AE_state
{0x002A, 0x04EE},
{0x0F12, 0x010E},	//#lt_uLimitHigh
{0x0F12, 0x00F5},	//#lt_uLimitLow

//For 60Hz
{0x002A, 0x0504},
{0x0F12, 0x3415},	//#lt_uMaxExp1
{0x002A, 0x0508},
{0x0F12, 0x681F},	//#lt_uMaxExp2
{0x002A, 0x050C},
{0x0F12, 0x8227},	//#lt_uMaxExp3
{0x002A, 0x0510},
{0x0F12, 0xC350},	//#lt_uMaxExp4

{0x002A, 0x0514},
{0x0F12, 0x3415},	//#lt_uCapMaxExp1
{0x002A, 0x0518},
{0x0F12, 0x681F},	//#lt_uCapMaxExp2
{0x002A, 0x051C},
{0x0F12, 0x8227},	//#lt_uCapMaxExp3
{0x002A, 0x0520},
{0x0F12, 0xC350},	//#lt_uCapMaxExp4

{0x002A, 0x0524},
{0x0F12, 0x0230},	//#lt_uMaxAnGain1
{0x0F12, 0x0230},	//#lt_uMaxAnGain2
{0x0F12, 0x0300},	//#lt_uMaxAnGain3
{0x0F12, 0x0A00},	//#lt_uMaxAnGain4

{0x0F12, 0x0100},	//#lt_uMaxDigGain
{0x0F12, 0x8000},	//#lt_uMaxTotGain  Total-gain is limited by #lt_uMaxTotGain

{0x0F12, 0x0230},	//#lt_uCapMaxAnGain1
{0x0F12, 0x0230},	//#lt_uCapMaxAnGain2
{0x0F12, 0x0300},	//#lt_uCapMaxAnGain3
{0x0F12, 0x0710},	//#lt_uCapMaxAnGain4

{0x0F12, 0x0100},	//#lt_uCapMaxDigGain
{0x0F12, 0x8000},	//#lt_uCapMaxTotGain  Total-gain is limited by #lt_uMaxTotGain

//===================================================================
//AE - Weights
//===================================================================
{0x002A, 0x1316},
{0x0F12, 0x0000},	//0000	//ae_WeightTbl_16[0]
{0x0F12, 0x0000},	//0101	//ae_WeightTbl_16[1]
{0x0F12, 0x0000},	//0101	//ae_WeightTbl_16[2]
{0x0F12, 0x0000},	//0000	//ae_WeightTbl_16[3]
{0x0F12, 0x0101},	//0101	//ae_WeightTbl_16[4]
{0x0F12, 0x0101},	//0101	//ae_WeightTbl_16[5]
{0x0F12, 0x0101},	//0101	//ae_WeightTbl_16[6]
{0x0F12, 0x0101},	//0101	//ae_WeightTbl_16[7]
{0x0F12, 0x0101},	//0201	//ae_WeightTbl_16[8]
{0x0F12, 0x0201},	//0303	//ae_WeightTbl_16[9]
{0x0F12, 0x0102},	//0303	//ae_WeightTbl_16[10]
{0x0F12, 0x0101},	//0102	//ae_WeightTbl_16[11]
{0x0F12, 0x0101},	//0201	//ae_WeightTbl_16[12]
{0x0F12, 0x0202},	//0403	//ae_WeightTbl_16[13]
{0x0F12, 0x0202},	//0304	//ae_WeightTbl_16[14]
{0x0F12, 0x0101},	//0102	//ae_WeightTbl_16[15]
{0x0F12, 0x0101},	//0201	//ae_WeightTbl_16[16]
{0x0F12, 0x0202},	//0403	//ae_WeightTbl_16[17]
{0x0F12, 0x0202},	//0304	//ae_WeightTbl_16[18]
{0x0F12, 0x0101},	//0102	//ae_WeightTbl_16[19]
{0x0F12, 0x0201},	//0201	//ae_WeightTbl_16[20]
{0x0F12, 0x0202},	//0403	//ae_WeightTbl_16[21]
{0x0F12, 0x0202},	//0304	//ae_WeightTbl_16[22]
{0x0F12, 0x0102},	//0102	//ae_WeightTbl_16[23]
{0x0F12, 0x0201},	//0201	//ae_WeightTbl_16[24]
{0x0F12, 0x0202},	//0303	//ae_WeightTbl_16[25]
{0x0F12, 0x0202},	//0303	//ae_WeightTbl_16[26]
{0x0F12, 0x0102},	//0102	//ae_WeightTbl_16[27]
{0x0F12, 0x0101},	//0201	//ae_WeightTbl_16[28]
{0x0F12, 0x0101},	//0202	//ae_WeightTbl_16[29]
{0x0F12, 0x0101},	//0202	//ae_WeightTbl_16[30]
{0x0F12, 0x0101},	//0102	//ae_WeightTbl_16[31]

//===================================================================
//AWB-BASIC setting
//===================================================================
{0x002A, 0x1018},
{0x0F12, 0x02A7},	//awbb_GLocusR
{0x0F12, 0x0343},	//awbb_GLocusB
{0x002A, 0x0FFC},
{0x0F12, 0x036C},	//awbb_CrclLowT_R_c
{0x002A, 0x1000},
{0x0F12, 0x011D},	//awbb_CrclLowT_B_c
{0x002A, 0x1004},
{0x0F12, 0x62C1},	//awbb_CrclLowT_Rad_c
{0x002A, 0x1034},
{0x0F12, 0x05F0},	//awbb_GamutWidthThr1
{0x0F12, 0x01F4},	//awbb_GamutHeightThr1
{0x0F12, 0x006C},	//awbb_GamutWidthThr2
{0x0F12, 0x0038},	//awbb_GamutHeightThr2
{0x002A, 0x1020},
{0x0F12, 0x000C},	//awbb_MinNumOfFinalPatches
{0x0F12, 0x001E},	//awbb_MinNumOfLowBrFinalPatches
{0x0F12, 0x0046},	//awbb_MinNumOfLowBr0_FinalPatches
{0x002A, 0x291A},
{0x0F12, 0x0006},	// #Mon_AWB_ByPassMode // [0]Outdoor [1]LowBr [2]LowTemp

{0x002A, 0x11C2},
{0x0F12, 0x0000},	//awbb_RGainOff
{0x0F12, 0x0000},	//awbb_BGainOff
{0x0F12, 0x0000},	//awbb_GGainOff
{0x0F12, 0x00C2},	//awbb_Alpha_Comp_Mode
{0x0F12, 0x0002},	//awbb_Rpl_InvalidOutDoor
{0x0F12, 0x0001},	//awbb_UseGrThrCorr
{0x0F12, 0x00E4},	//awbb_Use_Filters
{0x0F12, 0x053C},	//awbb_GainsInit[0]
{0x0F12, 0x0400},	//awbb_GainsInit[1]
{0x0F12, 0x055C},	//awbb_GainsInit[2]
{0x0F12, 0x001E},	//awbb_WpFilterMinThr
{0x0F12, 0x0190},	//awbb_WpFilterMaxThr
{0x0F12, 0x00A0},	//awbb_WpFilterCoef
{0x0F12, 0x0004},	//awbb_WpFilterSize
{0x0F12, 0x0001},	//awbb_otp_disable

//===================================================================
//AWB-Zone
//===================================================================
//	param_start	awbb_IndoorGrZones_m_BGrid
{0x002A, 0x0F28},
{0x0F12, 0x03BA},	//awbb_IndoorGrZones_m_BGrid[7]
{0x0F12, 0x03E8},	//03E8	//0406	//0406	//awbb_IndoorGrZones_m_BGrid[1]
{0x0F12, 0x035C},	//awbb_IndoorGrZones_m_BGrid[11]
{0x0F12, 0x03F6},	//03F0	//03F0	//03F0	//awbb_IndoorGrZones_m_BGrid[3]
{0x0F12, 0x0328},	//0328	//033A	//033A	//awbb_IndoorGrZones_m_BGrid[4]
{0x0F12, 0x03E0},	//03D4	//03D4	//03D4	//awbb_IndoorGrZones_m_BGrid[5]
{0x0F12, 0x02FE},	//02FE	//0312	//0312	//awbb_IndoorGrZones_m_BGrid[6]
{0x0F12, 0x03AC},	//0374	//03BA	//03BA	//awbb_IndoorGrZones_m_BGrid[7]
{0x0F12, 0x02E2},	//02DA	//02E8	//02E8	//awbb_IndoorGrZones_m_BGrid[8]
{0x0F12, 0x0370},	//033E	//03A0	//03A0	//awbb_IndoorGrZones_m_BGrid[9]
{0x0F12, 0x02C4},	//02BA	//02C0	//02C0	//awbb_IndoorGrZones_m_BGrid[10]
{0x0F12, 0x032E},	//0320	//035C	//035C	//awbb_IndoorGrZones_m_BGrid[11]
{0x0F12, 0x029E},	//0296	//0296	//0296	//awbb_IndoorGrZones_m_BGrid[12]
{0x0F12, 0x030A},	//0306	//030E	//030E	//awbb_IndoorGrZones_m_BGrid[13]
{0x0F12, 0x0278},	//0278	//0270	//0270	//awbb_IndoorGrZones_m_BGrid[14]
{0x0F12, 0x02F2},	//02F2	//02E2	//02E6	//awbb_IndoorGrZones_m_BGrid[15]
{0x0F12, 0x0258},	//0258	//024E	//0256	//awbb_IndoorGrZones_m_BGrid[16]
{0x0F12, 0x02E2},	//02D6	//02BE	//02C2	//awbb_IndoorGrZones_m_BGrid[17]
{0x0F12, 0x0232},	//0240	//awbb_IndoorGrZones_m_BGrid[18]
{0x0F12, 0x02CA},	//02B2	//029A	//02A6	//awbb_IndoorGrZones_m_BGrid[19]
{0x0F12, 0x0218},	//0226	//awbb_IndoorGrZones_m_BGrid[20]
{0x0F12, 0x02BC},	//02B6	//awbb_IndoorGrZones_m_BGrid[21]
{0x0F12, 0x01FE},	//0210	//awbb_IndoorGrZones_m_BGrid[22]
{0x0F12, 0x02A6},	//029C	//awbb_IndoorGrZones_m_BGrid[23]
{0x0F12, 0x01EE},	//0202	//awbb_IndoorGrZones_m_BGrid[24]
{0x0F12, 0x0290},	//0280	//awbb_IndoorGrZones_m_BGrid[25]
{0x0F12, 0x01E0},	//01FA	//awbb_IndoorGrZones_m_BGrid[26]
{0x0F12, 0x0274},	//0264	//awbb_IndoorGrZones_m_BGrid[27]
{0x0F12, 0x01E6},	//0200	//awbb_IndoorGrZones_m_BGrid[28]
{0x0F12, 0x024E},	//022E	//awbb_IndoorGrZones_m_BGrid[29]
{0x0F12, 0x01F2},	//0000	//awbb_IndoorGrZones_m_BGrid[30]
{0x0F12, 0x0226},	//0000	//awbb_IndoorGrZones_m_BGrid[31]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[32]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[33]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[34]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[35]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[36]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[37]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[38]
{0x0F12, 0x0000},	//awbb_IndoorGrZones_m_BGrid[39]
//	param_end	awbb_IndoorGrZones_m_BGrid

{0x0F12, 0x0005},	//awbb_IndoorGrZones_m_Grid
{0x002A, 0x0F80},
{0x0F12, 0x00E6},	//awbb_IndoorGrZones_m_Boff
{0x002A, 0x0F7C},
{0x0F12, 0x0010},	//000F

//	param_start	awbb_OutdoorGrZones_m_BGrid
{0x002A, 0x0F84},
{0x0F12, 0x0286},	//02A2	//0288	//027A	//0282	//0282	//0282	//026A	//awbb_OutdoorGrZones_m_BGrid[0]
{0x0F12, 0x02A2},	//0294	//029A	//029A	//029A	//029A	//awbb_OutdoorGrZones_m_BGrid[1]
{0x0F12, 0x0268},	//028A	//0272	//0268	//0270	//0270	//0270	//024E	//awbb_OutdoorGrZones_m_BGrid[2]
{0x0F12, 0x02B0},	//02C4	//02AE	//02A2	//02A8	//02A0	//02A0	//02A6	//awbb_OutdoorGrZones_m_BGrid[3]
{0x0F12, 0x0254},	//0276	//0264	//025A	//0264	//0264	//0264	//023A	//awbb_OutdoorGrZones_m_BGrid[4]
{0x0F12, 0x02BE},	//02C6	//02AE	//02A2	//02A4	//0298	//02A8	//0290	//awbb_OutdoorGrZones_m_BGrid[5]
{0x0F12, 0x0240},	//0264	//0258	//024E	//025A	//025A	//025A	//021A	//awbb_OutdoorGrZones_m_BGrid[6]
{0x0F12, 0x02BE},	//02C0	//02A8	//029C	//0298	//028A	//0298	//0274	//awbb_OutdoorGrZones_m_BGrid[7]
{0x0F12, 0x0230},	//0254	//024C	//0244	//0254	//0254	//0254	//0216	//awbb_OutdoorGrZones_m_BGrid[8]
{0x0F12, 0x02B8},	//02B8	//02A0	//028E	//028A	//027C	//028A	//0260	//awbb_OutdoorGrZones_m_BGrid[9]
{0x0F12, 0x0224},	//0240	//0242	//023C	//0260	//0258	//0260	//0212	//awbb_OutdoorGrZones_m_BGrid[10]
{0x0F12, 0x02B0},	//02AA	//0294	//0280	//027A	//0268	//027A	//024C	//awbb_OutdoorGrZones_m_BGrid[11]
{0x0F12, 0x0218},	//0230	//023A	//0232	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[12]
{0x0F12, 0x02A4},	//029C	//0286	//0274	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[13]
{0x0F12, 0x0212},	//0224	//0234	//022E	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[14]
{0x0F12, 0x0296},	//028C	//027A	//0260	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[15]
{0x0F12, 0x020E},	//0222	//0230	//022E	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[16]
{0x0F12, 0x0286},	//0274	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[13]
{0x0F12, 0x0210},	//0220	//023C	//0232	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[18]
{0x0F12, 0x0278},	//0270	//0250	//0246	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[19]
{0x0F12, 0x021A},	//0230	//0000	//0000	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[20]
{0x0F12, 0x026A},	//0254	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[17]
{0x0F12, 0x0228},	//0000	//0000	//0000	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[22]
{0x0F12, 0x024A},	//0000	//0000	//0000	//0000	//0000	//0000	//0000	//awbb_OutdoorGrZones_m_BGrid[23]
//	param_end	awbb_OutdoorGrZones_m_BGrid

{0x0F12, 0x0004},	//0005	//awbb_OutdoorGrZones_m_Gri
{0x002A, 0x0FB8},
{0x0F12, 0x000C},	//000A	//000B	//awbb_OutdoorGrZones_ZInfo_m_GridSz
{0x002A, 0x0FBC},
{0x0F12, 0x01E4},	//0206	//01E2	//01D6	//01E0	//awbb_OutdoorGrZones_m_Bof

//	param_start	awbb_LowBrGrZones_m_BGrid
{0x002A, 0x0FC0},
{0x0F12, 0x03B2},	//awbb_LowBrGrZones_m_BGrid[0]
{0x0F12, 0x044E},	//awbb_LowBrGrZones_m_BGrid[1]
{0x0F12, 0x0330},	//awbb_LowBrGrZones_m_BGrid[2]
{0x0F12, 0x0454},	//awbb_LowBrGrZones_m_BGrid[3]
{0x0F12, 0x02CC},	//awbb_LowBrGrZones_m_BGrid[4]
{0x0F12, 0x0414},	//awbb_LowBrGrZones_m_BGrid[5]
{0x0F12, 0x026E},	//awbb_LowBrGrZones_m_BGrid[6]
{0x0F12, 0x03D0},	//awbb_LowBrGrZones_m_BGrid[7]
{0x0F12, 0x0226},	//awbb_LowBrGrZones_m_BGrid[8]
{0x0F12, 0x0362},	//awbb_LowBrGrZones_m_BGrid[9]
{0x0F12, 0x01F0},	//awbb_LowBrGrZones_m_BGrid[10]
{0x0F12, 0x0312},	//awbb_LowBrGrZones_m_BGrid[11]
{0x0F12, 0x01CE},	//awbb_LowBrGrZones_m_BGrid[12]
{0x0F12, 0x02CC},	//awbb_LowBrGrZones_m_BGrid[13]
{0x0F12, 0x01B2},	//awbb_LowBrGrZones_m_BGrid[14]
{0x0F12, 0x029E},	//awbb_LowBrGrZones_m_BGrid[15]
{0x0F12, 0x01AC},	//awbb_LowBrGrZones_m_BGrid[16]
{0x0F12, 0x0278},	//awbb_LowBrGrZones_m_BGrid[17]
{0x0F12, 0x01B6},	//awbb_LowBrGrZones_m_BGrid[18]
{0x0F12, 0x0248},	//awbb_LowBrGrZones_m_BGrid[19]
{0x0F12, 0x0000},	//awbb_LowBrGrZones_m_BGrid[20]
{0x0F12, 0x0000},	//awbb_LowBrGrZones_m_BGrid[21]
{0x0F12, 0x0000},	//awbb_LowBrGrZones_m_BGrid[22]
{0x0F12, 0x0000},	//awbb_LowBrGrZones_m_BGrid[23]

//	param_end	awbb_LowBrGrZones_m_BGrid
{0x0F12, 0x0006},	//awbb_LowBrGrZones_m_GridStep
{0x002A, 0x0FF4},
{0x0F12, 0x000A},	//awbb_LowBrGrZones_ZInfo_m_GridSz
{0x002A, 0x0FF8},
{0x0F12, 0x00C2},	//awbb_LowBrGrZones_m_Boffs

//===================================================================
//AWB Scene Detection
//===================================================================

{0x002A, 0x1098},
{0x0F12, 0xFE82},	//awbb_SCDetectionMap_SEC_StartR_B
{0x0F12, 0x001E},	//awbb_SCDetectionMap_SEC_StepR_B
{0x0F12, 0x0E74},	//awbb_SCDetectionMap_SEC_SunnyNB
{0x0F12, 0x0122},	//awbb_SCDetectionMap_SEC_StepNB
{0x0F12, 0x00E4},	//awbb_SCDetectionMap_SEC_LowTempR_B
{0x0F12, 0x0096},	//awbb_SCDetectionMap_SEC_SunnyNBZone
{0x0F12, 0x000E},	//awbb_SCDetectionMap_SEC_LowTempR_BZone

{0x002A, 0x105C},
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_0__0_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_0__2_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_0__4_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_1__1_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_1__3_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_2__0_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_2__2_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_2__4_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_3__1_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_3__3_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_4__0_
{0x0F12, 0x0000},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_4__2_
{0x0F12, 0x0500},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_4__4_
{0x0F12, 0x5555},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_5__1_
{0x0F12, 0x5455},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_5__3_
{0x0F12, 0xAA55},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_6__0_
{0x0F12, 0xAAAA},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_6__2_
{0x0F12, 0xBF54},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_6__4_
{0x0F12, 0xFFFF},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_7__1_
{0x0F12, 0x54FE},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_7__3_
{0x0F12, 0xFF6F},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_8__0_
{0x0F12, 0xFEFF},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_8__2_
{0x0F12, 0x1B54},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_8__4_
{0x0F12, 0xFFFF},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_9__1_
{0x0F12, 0x54FE},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_9__3_
{0x0F12, 0xFF06},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_10__0_
{0x0F12, 0xFEFF},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_10__2_
{0x0F12, 0x0154},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_10__4_
{0x0F12, 0xBFBF},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_11__1_
{0x0F12, 0x54BE},	//#awbb_SCDetectionMap_SEC_SceneDetectionMap_11__3_

//===================================================================
//AWB - GridCorrection
//===================================================================

{0x002A, 0x11E0},
{0x0F12, 0x0002},	//awbb_GridEnable

{0x002A, 0x11A8},
{0x0F12, 0x02C8},	//awbb_GridConst_1[0]
{0x0F12, 0x0325},	//awbb_GridConst_1[1]
{0x0F12, 0x038F},	//awbb_GridConst_1[2]

{0x0F12, 0x0F8E},	//0FDD	//awbb_GridConst_2[0]
{0x0F12, 0x10B3},	//awbb_GridConst_2[1]
{0x0F12, 0x1127},	//awbb_GridConst_2[2]
{0x0F12, 0x1138},	//awbb_GridConst_2[3]
{0x0F12, 0x118E},	//awbb_GridConst_2[4]
{0x0F12, 0x1213},	//awbb_GridConst_2[5]

{0x0F12, 0x00A7},	//awbb_GridCoeff_R_1
{0x0F12, 0x00C2},	//awbb_GridCoeff_B_1
{0x0F12, 0x00BD},	//awbb_GridCoeff_R_2
{0x0F12, 0x00AC},	//awbb_GridCoeff_B_2

{0x002A, 0x1118},
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[0][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[0][4]
{0x0F12, 0x0014},	//0000	//FFEC	//awbb_GridCorr_R[0][2]
{0x0F12, 0x0014},	//0000	//awbb_GridCorr_R[0][3]
{0x0F12, 0x0030},	//0000	//awbb_GridCorr_R[1][1]
{0x0F12, 0x00A0},	//0000	//awbb_GridCorr_R[0][5]
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[1][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[1][4]
{0x0F12, 0x0014},	//0000	//FFEC	//awbb_GridCorr_R[1][2]
{0x0F12, 0x0014},	//awbb_GridCorr_R[1][3]
{0x0F12, 0x0030},	//0000	//awbb_GridCorr_R[2][1]
{0x0F12, 0x00A0},	//awbb_GridCorr_R[1][5]
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[2][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[2][4]
{0x0F12, 0x0020},	//0000	//FFEC	//awbb_GridCorr_R[2][2]
{0x0F12, 0x0020},	//awbb_GridCorr_R[2][3]
{0x0F12, 0x0030},	//awbb_GridCorr_R[2][4]
{0x0F12, 0x00A0},	//awbb_GridCorr_R[2][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[0][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[0][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[0][2]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[0][3]
{0x0F12, 0xFF40},	//0000	//awbb_GridCorr_B[0][4]
{0x0F12, 0xFE00},	//FCE0	//awbb_GridCorr_B[0][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[1][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[1][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[1][2]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[1][3]
{0x0F12, 0xFF40},	//0000	//awbb_GridCorr_B[1][4]
{0x0F12, 0xFE00},	//FCE0	//awbb_GridCorr_B[1][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[2][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[2][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[2][2]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[2][3]
{0x0F12, 0xFF40},	//0000	//awbb_GridCorr_B[2][4]
{0x0F12, 0xFE00},	//FCE0	//awbb_GridCorr_B[2][5]
{0x002A, 0x1160},
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[0][4]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[0][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_Out[0][5]
{0x0F12, 0x0000},	//001E	//awbb_GridCorr_R_Out[1][0]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[1][2]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[1][3]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[1][4]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[1][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_Out[1][5]
{0x0F12, 0x0000},	//001E	//awbb_GridCorr_R_Out[2][0]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[2][2]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[2][3]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[2][4]
{0x0F12, 0x0000},	//awbb_GridCorr_R_Out[2][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_Out[2][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_Out[2][3]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_R_Out[2][4]
{0x0F12, 0x0000},	//awbb_GridCorr_B_Out[0][2]
{0x0F12, 0xFFBA},	//awbb_GridCorr_B_Out[0][0]
{0x0F12, 0xFFBA},	//awbb_GridCorr_B_Out[0][1]
{0x0F12, 0x0000},	//awbb_GridCorr_B_Out[0][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_Out[0][3]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_Out[0][4]
{0x0F12, 0x0000},	//awbb_GridCorr_B_Out[1][2]
{0x0F12, 0xFFBA},	//awbb_GridCorr_B_Out[1][0]
{0x0F12, 0xFFBA},	//awbb_GridCorr_B_Out[1][1]
{0x0F12, 0x0000},	//awbb_GridCorr_B_Out[1][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_Out[1][3]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_Out[1][4]
{0x0F12, 0x0000},	//awbb_GridCorr_B_Out[2][2]
{0x0F12, 0xFFBA},	//awbb_GridCorr_B_Out[2][0]
{0x0F12, 0xFFBA},	//awbb_GridCorr_B_Out[2][1]
{0x0F12, 0x0000},	//awbb_GridCorr_B_Out[2][5]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_Out[2][3]
{0x0F12, 0x0000},	//0000	//awbb_GridCorr_B_Out[2][4]
{0x0F12, 0x0000},	//0000	//0000	//0000	//awbb_GridCorr_B_Out[2][5]

//===================================================================
// CCM
//===================================================================
{0x002A, 0x07D2},
{0x0F12, 0x00C0},	//SARR_AwbCcmCord_0_
{0x0F12, 0x00E0},	//SARR_AwbCcmCord_1_
{0x0F12, 0x0110},	//SARR_AwbCcmCord_2_
{0x0F12, 0x0139},	//SARR_AwbCcmCord_3_
{0x0F12, 0x0166},	//SARR_AwbCcmCord_4_
{0x0F12, 0x019F},	//SARR_AwbCcmCord_5_

//	param_start	TVAR_wbt_pBaseCcms
{0x002A, 0x07C4},
{0x0F12, 0x4000},	//TVAR_wbt_pBaseCcms
{0x0F12, 0x7000},

{0x002A, 0x4000},
{0x0F12, 0x01ED},	//TVAR_wbt_pBaseCcms[0]
{0x0F12, 0xFF9E},	//TVAR_wbt_pBaseCcms[1]
{0x0F12, 0xFFD8},	//TVAR_wbt_pBaseCcms[2]
{0x0F12, 0xFF59},	//FF0F	//FF0F	//FF0F	//FEEA	//FF04	//FEBB	//FEBB	//FE87	//FE6B	//TVAR_wbt_pBaseCcms[3]
{0x0F12, 0x0140},	//01A8	//01A8	//01A8	//01BB	//01B6	//00B9	//00B9	//010D	//0106	//TVAR_wbt_pBaseCcms[4]
{0x0F12, 0xFF3B},	//FF1C	//FF1C	//FF1C	//FF2F	//FF2C	//FF08	//FF08	//FEE8	//FF0B	//TVAR_wbt_pBaseCcms[5]
{0x0F12, 0xFFC3},	//FF91	//FF91	//FF91	//FF8B	//FFE5	//FFDD	//FFDD	//FFDD	//FFDD	//TVAR_wbt_pBaseCcms[6]
{0x0F12, 0xFFD5},	//FFA9	//FFA9	//FFA9	//FFAF	//FFE7	//FFEE	//FFEE	//FFEE	//FFEE	//TVAR_wbt_pBaseCcms[7]
{0x0F12, 0x0173},	//01D0	//01D0	//01D0	//01D0	//01BC	//01CB	//01CB	//01CB	//01CB	//TVAR_wbt_pBaseCcms[8]
{0x0F12, 0x00B9},	//00AA	//0046	//0061	//0034	//0039	//015D	//015D	//015D	//0187	//TVAR_wbt_pBaseCcms[9]
{0x0F12, 0x00D2},	//00AB	//00FC	//00E4	//00DA	//00DE	//00DE	//00DE	//00DE	//00A6	//TVAR_wbt_pBaseCcms[10]
{0x0F12, 0xFF2F},	//FEE1	//FEF3	//FF49	//FF54	//FF18	//FEB0	//FEB0	//FEB0	//FEBE	//TVAR_wbt_pBaseCcms[11]
{0x0F12, 0x00C8},	//00C8	//00C8	//00C8	//00C8	//00D2	//021C	//021C	//021C	//021C	//TVAR_wbt_pBaseCcms[12]
{0x0F12, 0xFF49},	//FF49	//FF49	//FF49	//FF49	//FF99	//FF5F	//FF5F	//FF5F	//FF5F	//TVAR_wbt_pBaseCcms[13]
{0x0F12, 0x014B},	//014B	//014B	//014B	//014B	//012B	//0175	//0175	//0175	//0175	//TVAR_wbt_pBaseCcms[14]
{0x0F12, 0xFF68},	//FF68	//FF68	//FF68	//FF68	//FF5F	//FEE7	//FEE7	//FEE7	//FEE7	//TVAR_wbt_pBaseCcms[15]
{0x0F12, 0x0109},	//0109	//0109	//0109	//0109	//00FC	//0106	//0106	//0106	//0106	//TVAR_wbt_pBaseCcms[16]
{0x0F12, 0x00F4},	//00F4	//00F4	//00F4	//00F4	//00EF	//00F3	//00F3	//00F3	//00F3	//TVAR_wbt_pBaseCcms[17]

{0x0F12, 0x01ED},	//TVAR_wbt_pBaseCcms[18]
{0x0F12, 0xFF9E},	//TVAR_wbt_pBaseCcms[19]
{0x0F12, 0xFFD8},	//TVAR_wbt_pBaseCcms[20]
{0x0F12, 0xFF59},	//FF0F	//FF0F	//FF0F	//FEEA	//FF04	//FEBB	//FEBB	//FE87	//FE6B	//TVAR_wbt_pBaseCcms[21]
{0x0F12, 0x0140},	//01A8	//01A8	//01A8	//01BB	//01B6	//00B9	//00B9	//010D	//0106	//TVAR_wbt_pBaseCcms[22]
{0x0F12, 0xFF3B},	//FF1C	//FF1C	//FF1C	//FF2F	//FF2C	//FF08	//FF08	//FEE8	//FF0B	//TVAR_wbt_pBaseCcms[23]
{0x0F12, 0xFFC3},	//FF91	//FF91	//FF91	//FF8B	//FFE5	//FFDD	//FFDD	//FFDD	//FFDD	//TVAR_wbt_pBaseCcms[24]
{0x0F12, 0xFFD5},	//FFA9	//FFA9	//FFA9	//FFAF	//FFE7	//FFEE	//FFEE	//FFEE	//FFEE	//TVAR_wbt_pBaseCcms[25]
{0x0F12, 0x0173},	//01D0	//01D0	//01D0	//01D0	//01BC	//01CB	//01CB	//01CB	//01CB	//TVAR_wbt_pBaseCcms[26]
{0x0F12, 0x00B9},	//00AA	//0046	//0061	//0034	//0039	//015D	//015D	//015D	//0187	//TVAR_wbt_pBaseCcms[27]
{0x0F12, 0x00D2},	//00AB	//00FC	//00E4	//00DA	//00DE	//00DE	//00DE	//00DE	//00A6	//TVAR_wbt_pBaseCcms[28]
{0x0F12, 0xFF2F},	//FEE1	//FEF3	//FF49	//FF54	//FF18	//FEB0	//FEB0	//FEB0	//FEBE	//TVAR_wbt_pBaseCcms[29]
{0x0F12, 0x00C8},	//00C8	//00C8	//00C8	//00C8	//00D2	//021C	//021C	//021C	//021C	//TVAR_wbt_pBaseCcms[30]
{0x0F12, 0xFF49},	//FF49	//FF49	//FF49	//FF49	//FF99	//FF5F	//FF5F	//FF5F	//FF5F	//TVAR_wbt_pBaseCcms[31]
{0x0F12, 0x014B},	//014B	//014B	//014B	//014B	//012B	//0175	//0175	//0175	//0175	//TVAR_wbt_pBaseCcms[32]
{0x0F12, 0xFF68},	//FF68	//FF68	//FF68	//FF68	//FF5F	//FEE7	//FEE7	//FEE7	//FEE7	//TVAR_wbt_pBaseCcms[33]
{0x0F12, 0x0109},	//0109	//0109	//0109	//0109	//00FC	//0106	//0106	//0106	//0106	//TVAR_wbt_pBaseCcms[34]
{0x0F12, 0x00F4},	//00F4	//00F4	//00F4	//00F4	//00EF	//00F3	//00F3	//00F3	//00F3	//TVAR_wbt_pBaseCcms[35]

{0x0F12, 0x01ED},	//TVAR_wbt_pBaseCcms[36]
{0x0F12, 0xFF9E},	//TVAR_wbt_pBaseCcms[37]
{0x0F12, 0xFFD8},	//TVAR_wbt_pBaseCcms[38]
{0x0F12, 0xFF59},	//FF0F	//FF0F	//FF0F	//FEEA	//FF04	//FEBB	//FEBB	//FE87	//FE6B	//TVAR_wbt_pBaseCcms[39]
{0x0F12, 0x0140},	//01A8	//01A8	//01A8	//01BB	//01B6	//00B9	//00B9	//010D	//0106	//TVAR_wbt_pBaseCcms[40]
{0x0F12, 0xFF3B},	//FF1C	//FF1C	//FF1C	//FF2F	//FF2C	//FF08	//FF08	//FEE8	//FF0B	//TVAR_wbt_pBaseCcms[41]
{0x0F12, 0xFFC3},	//FF91	//FF91	//FF91	//FF8B	//FFE5	//FFDD	//FFDD	//FFDD	//FFDD	//TVAR_wbt_pBaseCcms[42]
{0x0F12, 0xFFD5},	//FFA9	//FFA9	//FFA9	//FFAF	//FFE7	//FFEE	//FFEE	//FFEE	//FFEE	//TVAR_wbt_pBaseCcms[43]
{0x0F12, 0x0173},	//01D0	//01D0	//01D0	//01D0	//01BC	//01CB	//01CB	//01CB	//01CB	//TVAR_wbt_pBaseCcms[44]
{0x0F12, 0x00B9},	//00AA	//0046	//0061	//0034	//0039	//015D	//015D	//015D	//0187	//TVAR_wbt_pBaseCcms[45]
{0x0F12, 0x00D2},	//00AB	//00FC	//00E4	//00DA	//00DE	//00DE	//00DE	//00DE	//00A6	//TVAR_wbt_pBaseCcms[46]
{0x0F12, 0xFF2F},	//FEE1	//FEF3	//FF49	//FF54	//FF18	//FEB0	//FEB0	//FEB0	//FEBE	//TVAR_wbt_pBaseCcms[47]
{0x0F12, 0x00C8},	//00C8	//00C8	//00C8	//00C8	//00D2	//021C	//021C	//021C	//021C	//TVAR_wbt_pBaseCcms[48]
{0x0F12, 0xFF49},	//FF49	//FF49	//FF49	//FF49	//FF99	//FF5F	//FF5F	//FF5F	//FF5F	//TVAR_wbt_pBaseCcms[49]
{0x0F12, 0x014B},	//014B	//014B	//014B	//014B	//012B	//0175	//0175	//0175	//0175	//TVAR_wbt_pBaseCcms[50]
{0x0F12, 0xFF68},	//FF68	//FF68	//FF68	//FF68	//FF5F	//FEE7	//FEE7	//FEE7	//FEE7	//TVAR_wbt_pBaseCcms[51]
{0x0F12, 0x0109},	//0109	//0109	//0109	//0109	//00FC	//0106	//0106	//0106	//0106	//TVAR_wbt_pBaseCcms[52]
{0x0F12, 0x00F4},	//00F4	//00F4	//00F4	//00F4	//00EF	//00F3	//00F3	//00F3	//00F3	//TVAR_wbt_pBaseCcms[53]

{0x0F12, 0x01ED},	//TVAR_wbt_pBaseCcms[54]
{0x0F12, 0xFF9E},	//TVAR_wbt_pBaseCcms[55]
{0x0F12, 0xFFD8},	//TVAR_wbt_pBaseCcms[56]
{0x0F12, 0xFF59},	//FF58	//FF0F	//FF0F	//FF0F	//FEEA	//FF04  //FEC4	//FEC4	//FECF	//FECF //FEB9	//TVAR_wbt_pBaseCcms[57]
{0x0F12, 0x0140},	//018C	//01A8	//01A8	//01A8	//01BB	//01B6  //00DE	//00DE	//00E7	//00E7 //0111	//TVAR_wbt_pBaseCcms[58]
{0x0F12, 0xFF3B},	//FF1C	//FF1C	//FF1C	//FF1C	//FF2F	//FF2C  //FEF7	//FEF7	//FEE3	//FEE3 //FECF	//TVAR_wbt_pBaseCcms[59]
{0x0F12, 0xFFC3},	//FFC3	//FF91	//FF91	//FF91	//FF8B	//FFE5  //FFFD	//FFFD	//FFC9	//FFC9 //FFC9	//TVAR_wbt_pBaseCcms[60]
{0x0F12, 0xFFD5},	//FFD5	//FFA9	//FFA9	//FFA9	//FFAF	//FFE7  //FFD0	//FFD0	//0004	//0004 //0004	//TVAR_wbt_pBaseCcms[61]
{0x0F12, 0x0173},	//0173	//01D0	//01D0	//01D0	//01D0	//01BC  //01BB	//01BB	//01BA	//01BA //01BA	//TVAR_wbt_pBaseCcms[62]
{0x0F12, 0x00B9},	//00AA	//00AA	//0046	//0061	//0034	//0039  //00DF	//00DF	//00DF	//00DF //00DF	//TVAR_wbt_pBaseCcms[63]
{0x0F12, 0x00D2},	//00AB	//00AB	//00FC	//00E4	//00DA	//00DE  //00A7	//00A7	//00A7	//00A7 //00A7	//TVAR_wbt_pBaseCcms[64]
{0x0F12, 0xFF2F},	//FEE1	//FEE1	//FEF3	//FF49	//FF54	//FF18  //FF38	//FF39	//FF39	//FF39 //FF39	//TVAR_wbt_pBaseCcms[65]
{0x0F12, 0x00C8},	//00C8	//00C8	//00C8	//00C8	//00C8	//00D2  //00CA	//00CA	//00CA	//00CA //00CA	//TVAR_wbt_pBaseCcms[66]
{0x0F12, 0xFF49},	//FF49	//FF49	//FF49	//FF49	//FF49	//FF99  //FF9B	//FF9B	//FF9B	//FF9B //FF9B	//TVAR_wbt_pBaseCcms[67]
{0x0F12, 0x014B},	//014B	//014B	//014B	//014B	//014B	//012B  //0131	//0131	//0131	//0131 //0131	//TVAR_wbt_pBaseCcms[68]
{0x0F12, 0xFF68},	//FF68	//FF68	//FF68	//FF68	//FF68	//FF5F  //FF6D	//FF6D	//FF6D	//FF6D //FF6D	//TVAR_wbt_pBaseCcms[69]
{0x0F12, 0x0109},	//0109	//0109	//0109	//0109	//0109	//00FC  //0139	//0139	//0139	//0139 //0139	//TVAR_wbt_pBaseCcms[70]
{0x0F12, 0x00F4},	//00F4	//00F4	//00F4	//00F4	//00F4	//00EF  //00A4	//00A4	//00A4	//00A4 //00A4	//TVAR_wbt_pBaseCcms[71]

{0x0F12, 0x011D}, //01BC	//01C7	//01AF	//01AF	//01AC  //01A5	//01AF	//01AF	//01AF //01D9	//TVAR_wbt_pBaseCcms[72]
{0x0F12, 0xFFA7},	//FF5E	//FF58	//FFC2	//FFC2	//FFBF  //FFE3	//FFC3	//FFC3	//FFC4 //FFAC	//TVAR_wbt_pBaseCcms[73]
{0x0F12, 0xFFEC},	//FF97	//FF93	//FFF1	//FFF1	//FFF7  //FFDA	//FFF2	//FFF2	//FFF1 //FFDF	//TVAR_wbt_pBaseCcms[74]
{0x0F12, 0xFF58},	//FF0F	//FF0F	//FF0F	//FEEA	//FF04  //FEC4	//FEC4	//FECF	//FECF //FEB9	//TVAR_wbt_pBaseCcms[75]
{0x0F12, 0x018C},	//01A8	//01A8	//01A8	//01BB	//01B6  //00DE	//00DE	//00E7	//00E7 //0111	//TVAR_wbt_pBaseCcms[76]
{0x0F12, 0xFF1C},	//FF1C	//FF1C	//FF1C	//FF2F	//FF2C  //FEF7	//FEF7	//FEE3	//FEE3 //FECF	//TVAR_wbt_pBaseCcms[77]
{0x0F12, 0xFFC3},	//FF91	//FF91	//FF91	//FF8B	//FFE5  //FFFD	//FFFD	//FFC9	//FFC9 //FFC9	//TVAR_wbt_pBaseCcms[78]
{0x0F12, 0xFFD5},	//FFA9	//FFA9	//FFA9	//FFAF	//FFE7  //FFD0	//FFD0	//0004	//0004 //0004	//TVAR_wbt_pBaseCcms[79]
{0x0F12, 0x0173},	//01D0	//01D0	//01D0	//01D0	//01BC  //01BB	//01BB	//01BA	//01BA //01BA	//TVAR_wbt_pBaseCcms[80]
{0x0F12, 0x00AA},	//00AA	//0046	//0061	//0034	//0039  //00DF	//00DF	//00DF	//00DF //00DF	//TVAR_wbt_pBaseCcms[81]
{0x0F12, 0x00AB},	//00AB	//00FC	//00E4	//00DA	//00DE  //00A7	//00A7	//00A7	//00A7 //00A7	//TVAR_wbt_pBaseCcms[82]
{0x0F12, 0xFEE1},	//FEE1	//FEF3	//FF49	//FF54	//FF18  //FF38	//FF39	//FF39	//FF39 //FF39	//TVAR_wbt_pBaseCcms[83]
{0x0F12, 0x00C8},	//00C8	//00C8	//00C8	//00C8	//00D2  //00CA	//00CA	//00CA	//00CA //00CA	//TVAR_wbt_pBaseCcms[84]
{0x0F12, 0xFF49},	//FF49	//FF49	//FF49	//FF49	//FF99  //FF9B	//FF9B	//FF9B	//FF9B //FF9B	//TVAR_wbt_pBaseCcms[85]
{0x0F12, 0x014B},	//014B	//014B	//014B	//014B	//012B  //0131	//0131	//0131	//0131 //0131	//TVAR_wbt_pBaseCcms[86]
{0x0F12, 0xFF68},	//FF68	//FF68	//FF68	//FF68	//FF5F  //FF6D	//FF6D	//FF6D	//FF6D //FF6D	//TVAR_wbt_pBaseCcms[87]
{0x0F12, 0x0109},	//0109	//0109	//0109	//0109	//00FC  //0139	//0139	//0139	//0139 //0139	//TVAR_wbt_pBaseCcms[88]
{0x0F12, 0x00F4},	//00F4	//00F4	//00F4	//00F4	//00EF  //00A4	//00A4	//00A4	//00A4 //00A4	//TVAR_wbt_pBaseCcms[89]

{0x0F12, 0x011D}, //01BC	//01AF	//01AC  //01A5	//01AF	//01AF	//01AF //01D9	//TVAR_wbt_pBaseCcms[90]
{0x0F12, 0xFFA7},	//FF5E	//FFC2	//FFC2	//FFBF  //FFE3	//FFC3	//FFC3	//FFC4 //FFAC	//TVAR_wbt_pBaseCcms[91]
{0x0F12, 0xFFEC},	//FF97	//FFF1	//FFF1	//FFF7  //FFDA	//FFF2	//FFF2	//FFF1 //FFDF	//TVAR_wbt_pBaseCcms[92]
{0x0F12, 0xFF58},	//FF0F	//FF0F	//FEEA	//FF04  //FEC4	//FEC4	//FECF	//FECF //FEB9	//TVAR_wbt_pBaseCcms[93]
{0x0F12, 0x018C},	//01A8	//01A8	//01BB	//01B6  //00DE	//00DE	//00E7	//00E7 //0111	//TVAR_wbt_pBaseCcms[94]
{0x0F12, 0xFF1C},	//FF1C	//FF1C	//FF2F	//FF2C  //FEF7	//FEF7	//FEE3	//FEE3 //FECF	//TVAR_wbt_pBaseCcms[95]
{0x0F12, 0xFFC3},	//FF91	//FF91	//FF8B	//FFE5  //FFFD	//FFFD	//FFC9	//FFC9 //FFC9	//TVAR_wbt_pBaseCcms[96]
{0x0F12, 0xFFD5},	//FFA9	//FFA9	//FFAF	//FFE7  //FFD0	//FFD0	//0004	//0004 //0004	//TVAR_wbt_pBaseCcms[97]
{0x0F12, 0x0173},	//01D0	//01D0	//01D0	//01BC  //01BB	//01BB	//01BA	//01BA //01BA	//TVAR_wbt_pBaseCcms[98]
{0x0F12, 0x00AA},	//00AA	//0061	//0034	//0039  //00DF	//00DF	//00DF	//00DF //00DF	//TVAR_wbt_pBaseCcms[99]
{0x0F12, 0x00AB},	//00AB	//00E4	//00DA	//00DE  //00A7	//00A7	//00A7	//00A7 //00A7	//TVAR_wbt_pBaseCcms[100]
{0x0F12, 0xFEE1},	//FEE1	//FF49	//FF54	//FF18  //FF38	//FF39	//FF39	//FF39 //FF39	//TVAR_wbt_pBaseCcms[101]
{0x0F12, 0x00C8},	//00C8	//00C8	//00C8	//00D2  //00CA	//00CA	//00CA	//00CA //00CA	//TVAR_wbt_pBaseCcms[102]
{0x0F12, 0xFF49},	//FF49	//FF49	//FF49	//FF99  //FF9B	//FF9B	//FF9B	//FF9B //FF9B	//TVAR_wbt_pBaseCcms[103]
{0x0F12, 0x014B},	//014B	//014B	//014B	//012B  //0131	//0131	//0131	//0131 //0131	//TVAR_wbt_pBaseCcms[104]
{0x0F12, 0xFF68},	//FF68	//FF68	//FF68	//FF5F  //FF6D	//FF6D	//FF6D	//FF6D //FF6D	//TVAR_wbt_pBaseCcms[105]
{0x0F12, 0x0109},	//0109	//0109	//0109	//00FC  //0139	//0139	//0139	//0139 //0139	//TVAR_wbt_pBaseCcms[106]
{0x0F12, 0x00F4},	//00F4	//00F4	//00F4	//00EF  //00A4	//00A4	//00A4	//00A4 //00A4	//TVAR_wbt_pBaseCcms[107]
//	param_end	TVAR_wbt_pBasecms


{0x002A, 0x07CC},
{0x0F12, 0x40D8},	//#TVAR_wbt_pOutdoorCcm
{0x0F12, 0x7000},

{0x002A, 0x40D8},
{0x0F12, 0x01FC},	//0212	//023C	//0226	//TVAR_wbt_pOutdoorCcm[0]
{0x0F12, 0xFFBF},	//FFA9	//FF8A	//FF98	//TVAR_wbt_pOutdoorCcm[1]
{0x0F12, 0xFFEB},	//FFE5	//FFDA	//FFDB	//TVAR_wbt_pOutdoorCcm[2]
{0x0F12, 0xFEF9},	//FEB8	//FEB8	//FECE	//TVAR_wbt_pOutdoorCcm[3]
{0x0F12, 0x0195},	//0187	//0187	//0173	//TVAR_wbt_pOutdoorCcm[4]
{0x0F12, 0xFEF9},	//FF41	//FF41	//FF38	//TVAR_wbt_pOutdoorCcm[5]
{0x0F12, 0xFFFD},	//FFF4	//FFF4	//FFF1	//TVAR_wbt_pOutdoorCcm[6]
{0x0F12, 0x0010},	//0007	//0007	//0016	//TVAR_wbt_pOutdoorCcm[7]
{0x0F12, 0x0206},	//0211	//0211	//01FD	//TVAR_wbt_pOutdoorCcm[8]
{0x0F12, 0x00F4},	//00E9	//00E9	//0106	//TVAR_wbt_pOutdoorCcm[9]
{0x0F12, 0x00DB},	//00FC	//00FC	//0108	//TVAR_wbt_pOutdoorCcm[10]
{0x0F12, 0xFF4A},	//FF2C	//FF2C	//FEFC	//TVAR_wbt_pOutdoorCcm[11]
{0x0F12, 0x01D4},	//023D	//023D	//0224	//TVAR_wbt_pOutdoorCcm[12]
{0x0F12, 0xFFCF},	//FFA4	//FFA4	//FFB2	//TVAR_wbt_pOutdoorCcm[13]
{0x0F12, 0x01D4},	//01BE	//01BE	//01C1	//TVAR_wbt_pOutdoorCcm[14]
{0x0F12, 0xFEC8},	//FEB6	//FEB6	//FECB	//TVAR_wbt_pOutdoorCcm[15]
{0x0F12, 0x017D},	//0190	//0190	//018C	//TVAR_wbt_pOutdoorCcm[16]
{0x0F12, 0x0142},	//0139	//0139	//0120	//TVAR_wbt_pOutdoorCcm[17]
//	param_end	TVAR_wbt_pOutdoorCcm


{0x002A, 0x2A64},
{0x0F12, 0x0001},	//#MVAR_AAIO_bFIT
{0x002A, 0x2A68},
{0x0F12, 0x0001},	//#MVAR_AAIO_bAutoCCMandASH
{0x002A, 0x2A3C},
{0x0F12, 0x01DD},	//#Mon_AAIO_PrevFrmData_NormBr



//===================================================================
// AFIT
//===================================================================

//	param_start	afit_uNoiseIndInDoor
{0x002A, 0x085C},
{0x0F12, 0x0041},	//004A	//#afit_uNoiseIndInDoor_0_
{0x0F12, 0x0046},	//005F	//#afit_uNoiseIndInDoor_1_
{0x0F12, 0x00CB},	//00CB	//#afit_uNoiseIndInDoor_2_
{0x0F12, 0x0190},	//01E0	//#afit_uNoiseIndInDoor_3_
{0x0F12, 0x01E0},	//0220	//#afit_uNoiseIndInDoor_4_
//	param_end	afit_uNoiseIndInDoor

{0x002A, 0x08C0},
{0x0F12, 0x0008},	//700008C0	//AFIT NoiseIndex0 Brightness
{0x0F12, 0x0000},	//700008C2        
{0x0F12, 0x0000},	//700008C4        
{0x0F12, 0x0000},	//700008C6
{0x0F12, 0x0000},	//700008C8        
{0x0F12, 0x00C1},	//700008CA        
{0x0F12, 0x0000},	//700008CC        
{0x0F12, 0x03FF},	//700008CE        
{0x0F12, 0x009C},	//700008D0        
{0x0F12, 0x017C},	//700008D2        
{0x0F12, 0x03FF},	//700008D4        
{0x0F12, 0x000C},	//700008D6        
{0x0F12, 0x0010},	//700008D8        
{0x0F12, 0x012C},	//700008DA        
{0x0F12, 0x03E8},	//700008DC        
{0x0F12, 0x0046},	//700008DE        
{0x0F12, 0x005A},	//700008E0        
{0x0F12, 0x0070},	//700008E2        
{0x0F12, 0x000C},	//700008E4	//[15:0]iHystThLow        
{0x0F12, 0x000C},	//700008E6	//[15:0]iHystThHigh        
{0x0F12, 0x01F4},	//700008E8	//[15:0]iHystCenter        
{0x0F12, 0x003C},	//700008EA        
{0x0F12, 0x0008},	//700008EC        
{0x0F12, 0x003C},	//700008EE        
{0x0F12, 0x001E},	//700008F0        
{0x0F12, 0x003C},	//700008F2        
{0x0F12, 0x001E},	//700008F4        
{0x0F12, 0x0A24},	//700008F6        
{0x0F12, 0x1701},	//700008F8        
{0x0F12, 0x0229},	//700008FA        
{0x0F12, 0x1403},	//700008FC        
{0x0F12, 0x0004},	//700008FE        
{0x0F12, 0x0300},	//70000900        
{0x0F12, 0x0000},	//70000902        
{0x0F12, 0x02FF},	//70000904        
{0x0F12, 0x09E8},	//70000906        
{0x0F12, 0x1414},	//70000908        
{0x0F12, 0x0301},	//7000090A        
{0x0F12, 0x0007},	//7000090C        
{0x0F12, 0x4000},	//7000090E        
{0x0F12, 0x7803},	//70000910        
{0x0F12, 0x3C50},	//70000912        
{0x0F12, 0x003C},	//70000914        
{0x0F12, 0x1E80},	//70000916        
{0x0F12, 0x1E08},	//70000918        
{0x0F12, 0x000A},	//7000091A        
{0x0F12, 0x0000},	//7000091C        
{0x0F12, 0x120A},	//7000091E        
{0x0F12, 0x0F00},	//70000920        
{0x0F12, 0x0200},	//70000922        
{0x0F12, 0xFF00},	//70000924        
{0x0F12, 0x0200},	//70000926        
{0x0F12, 0x1B11},	//70000928        
{0x0F12, 0x0000},	//7000092A        
{0x0F12, 0x0009},	//7000092C        
{0x0F12, 0x0406},	//7000092E        
{0x0F12, 0x0605},	//70000930        
{0x0F12, 0x0307},	//70000932        
{0x0F12, 0x0609},	//70000934        
{0x0F12, 0x2C07},	//70000936        
{0x0F12, 0x142C},	//70000938        
{0x0F12, 0x0518},	//7000093A	//[15:8]iUVNRStrengthL, [7:0]iMaxThreshH        
{0x0F12, 0x8005},	//7000093C	//[7:0]iUVNRStrengthH        
{0x0F12, 0x0A80},	//7000093E        
{0x0F12, 0x004B},	//70000940
{0x0F12, 0x0080},	//70000942        
{0x0F12, 0x0101},	//70000944        
{0x0F12, 0x0707},	//70000946        
{0x0F12, 0x4601},	//70000948        
{0x0F12, 0xFF44},	//7000094A        
{0x0F12, 0x50FF},	//7000094C        
{0x0F12, 0x0500},	//7000094E        
{0x0F12, 0x0003},	//70000950        
{0x0F12, 0x1C01},	//70000952        
{0x0F12, 0x0714},	//70000954        
{0x0F12, 0x1464},	//70000956        
{0x0F12, 0x3204},	//70000958        
{0x0F12, 0x3C1E},	//7000095A        
{0x0F12, 0x400F},	//7000095C        
{0x0F12, 0x0204},	//7000095E        
{0x0F12, 0x1403},	//70000960        
{0x0F12, 0x0114},	//70000962        
{0x0F12, 0x0101},	//70000964        
{0x0F12, 0x4446},	//70000966        
{0x0F12, 0x646E},	//70000968        
{0x0F12, 0x0028},	//7000096A        
{0x0F12, 0x030A},	//7000096C        
{0x0F12, 0x0000},	//7000096E        
{0x0F12, 0x141E},	//70000970        
{0x0F12, 0xFF07},	//70000972        
{0x0F12, 0x0432},	//70000974        
{0x0F12, 0x0000},	//70000976        
{0x0F12, 0x0F0F},	//70000978        
{0x0F12, 0x0440},	//7000097A        
{0x0F12, 0x0302},	//7000097C        
{0x0F12, 0x1414},	//7000097E        
{0x0F12, 0x0101},	//70000980        
{0x0F12, 0x4601},	//70000982        
{0x0F12, 0x6E44},	//70000984        
{0x0F12, 0x2864},	//70000986        
{0x0F12, 0x0A00},	//70000988        
{0x0F12, 0x0003},	//7000098A        
{0x0F12, 0x1E00},	//7000098C        
{0x0F12, 0x0714},	//7000098E        
{0x0F12, 0x32FF},	//70000990        
{0x0F12, 0x0004},	//70000992        
{0x0F12, 0x0F00},	//70000994        
{0x0F12, 0x400F},	//70000996        
{0x0F12, 0x0204},	//70000998        
{0x0F12, 0x0003},	//7000099A        
{0x0F12, 0x0001},	//7000099C        
{0x0F12, 0x0000},	//7000099E	//AFIT NoiseIndex1 Brightness       
{0x0F12, 0x0000},	//700009A0        
{0x0F12, 0x0000},	//700009A2        
{0x0F12, 0x0000},	//700009A4        
{0x0F12, 0x0000},	//700009A6        
{0x0F12, 0x00C1},	//700009A8        
{0x0F12, 0x0000},	//700009AA        
{0x0F12, 0x03FF},	//700009AC        
{0x0F12, 0x009C},	//700009AE        
{0x0F12, 0x017C},	//700009B0        
{0x0F12, 0x03FF},	//700009B2        
{0x0F12, 0x000C},	//700009B4        
{0x0F12, 0x0010},	//700009B6        
{0x0F12, 0x012C},	//700009B8        
{0x0F12, 0x03E8},	//700009BA        
{0x0F12, 0x0046},	//700009BC        
{0x0F12, 0x005A},	//700009BE        
{0x0F12, 0x0070},	//700009C0        
{0x0F12, 0x0000},	//700009C2	//[15:0]iHystThLow        
{0x0F12, 0x0000},	//700009C4	//[15:0]iHystThHigh        
{0x0F12, 0x0320},	//700009C6	//[15:0]iHystCenter        
{0x0F12, 0x006E},	//700009C8        
{0x0F12, 0x0014},	//700009CA        
{0x0F12, 0x003C},	//700009CC        
{0x0F12, 0x001E},	//700009CE        
{0x0F12, 0x003C},	//700009D0        
{0x0F12, 0x001E},	//700009D2        
{0x0F12, 0x0A24},	//700009D4        
{0x0F12, 0x1701},	//700009D6        
{0x0F12, 0x0229},	//700009D8        
{0x0F12, 0x1403},	//700009DA        
{0x0F12, 0x0004},	//700009DC        
{0x0F12, 0x0300},	//700009DE        
{0x0F12, 0x0000},	//700009E0        
{0x0F12, 0x02FF},	//700009E2        
{0x0F12, 0x05E8},	//700009E4        
{0x0F12, 0x1414},	//700009E6        
{0x0F12, 0x0301},	//700009E8        
{0x0F12, 0x0007},	//700009EA        
{0x0F12, 0x2000},	//700009EC        
{0x0F12, 0x5003},	//700009EE        
{0x0F12, 0x3228},	//700009F0        
{0x0F12, 0x0032},	//700009F2        
{0x0F12, 0x1E80},	//700009F4        
{0x0F12, 0x1E08},	//700009F6        
{0x0F12, 0x000A},	//700009F8        
{0x0F12, 0x0000},	//700009FA        
{0x0F12, 0x120A},	//700009FC        
{0x0F12, 0x1400},	//700009FE        
{0x0F12, 0x0200},	//70000A00        
{0x0F12, 0xFF00},	//70000A02        
{0x0F12, 0x0200},	//70000A04        
{0x0F12, 0x1B11},	//70000A06        
{0x0F12, 0x0000},	//70000A08        
{0x0F12, 0x0009},	//70000A0A        
{0x0F12, 0x0406},	//70000A0C        
{0x0F12, 0x0605},	//70000A0E        
{0x0F12, 0x0307},	//70000A10        
{0x0F12, 0x0609},	//70000A12        
{0x0F12, 0x2C07},	//70000A14        
{0x0F12, 0x142C},	//70000A16        
{0x0F12, 0x0518},	//70000A18	//[15:8]iUVNRStrengthL, [7:0]iMaxThreshH        
{0x0F12, 0x8005},	//70000A1A	//[7:0]iUVNRStrengthH        
{0x0F12, 0x0580},	//70000A1C        
{0x0F12, 0x0080},	//70000A1E        
{0x0F12, 0x0080},	//70000A20        
{0x0F12, 0x0101},	//70000A22        
{0x0F12, 0x0707},	//70000A24        
{0x0F12, 0x4B01},	//70000A26        
{0x0F12, 0x494B},	//70000A28        
{0x0F12, 0x5044},	//70000A2A        
{0x0F12, 0x0500},	//70000A2C        
{0x0F12, 0x0503},	//70000A2E        
{0x0F12, 0x0D02},	//70000A30        
{0x0F12, 0x071E},	//70000A32        
{0x0F12, 0x1432},	//70000A34        
{0x0F12, 0x3201},	//70000A36        
{0x0F12, 0x2814},	//70000A38        
{0x0F12, 0x200F},	//70000A3A        
{0x0F12, 0x0204},	//70000A3C        
{0x0F12, 0x1E03},	//70000A3E        
{0x0F12, 0x011E},	//70000A40        
{0x0F12, 0x0101},	//70000A42        
{0x0F12, 0x3A3C},	//70000A44        
{0x0F12, 0x585A},	//70000A46        
{0x0F12, 0x0028},	//70000A48        
{0x0F12, 0x030A},	//70000A4A        
{0x0F12, 0x0000},	//70000A4C        
{0x0F12, 0x141E},	//70000A4E        
{0x0F12, 0xFF07},	//70000A50        
{0x0F12, 0x0432},	//70000A52        
{0x0F12, 0x0000},	//70000A54        
{0x0F12, 0x0F0F},	//70000A56        
{0x0F12, 0x0440},	//70000A58        
{0x0F12, 0x0302},	//70000A5A        
{0x0F12, 0x1E1E},	//70000A5C        
{0x0F12, 0x0101},	//70000A5E        
{0x0F12, 0x3C01},	//70000A60        
{0x0F12, 0x5A3A},	//70000A62        
{0x0F12, 0x2858},	//70000A64        
{0x0F12, 0x0A00},	//70000A66        
{0x0F12, 0x0003},	//70000A68        
{0x0F12, 0x1E00},	//70000A6A        
{0x0F12, 0x0714},	//70000A6C        
{0x0F12, 0x32FF},	//70000A6E        
{0x0F12, 0x0004},	//70000A70        
{0x0F12, 0x0F00},	//70000A72        
{0x0F12, 0x400F},	//70000A74        
{0x0F12, 0x0204},	//70000A76        
{0x0F12, 0x0003},	//70000A78        
{0x0F12, 0x0001},	//70000A7A        
{0x0F12, 0x0000},	//70000A7C	//AFIT NoiseIndex2 Brightness               
{0x0F12, 0x0000},	//70000A7E        
{0x0F12, 0x0000},	//70000A80        
{0x0F12, 0x0000},	//70000A82        
{0x0F12, 0x0000},	//70000A84        
{0x0F12, 0x00C1},	//70000A86        
{0x0F12, 0x0000},	//70000A88        
{0x0F12, 0x03FF},	//70000A8A        
{0x0F12, 0x009E},	//70000A8C        
{0x0F12, 0x017C},	//70000A8E        
{0x0F12, 0x03FF},	//70000A90        
{0x0F12, 0x000C},	//70000A92        
{0x0F12, 0x0010},	//70000A94        
{0x0F12, 0x012C},	//70000A96        
{0x0F12, 0x03E8},	//70000A98        
{0x0F12, 0x0046},	//70000A9A        
{0x0F12, 0x005A},	//70000A9C        
{0x0F12, 0x0070},	//70000A9E        
{0x0F12, 0x0000},	//70000AA0	//[15:0]iHystThLow        
{0x0F12, 0x0000},	//70000AA2	//[15:0]iHystThHigh        
{0x0F12, 0x0320},	//70000AA4	//[15:0]iHystCenter        
{0x0F12, 0x008C},	//70000AA6        
{0x0F12, 0x0014},	//70000AA8        
{0x0F12, 0x003C},	//70000AAA        
{0x0F12, 0x001E},	//70000AAC        
{0x0F12, 0x003C},	//70000AAE        
{0x0F12, 0x001E},	//70000AB0        
{0x0F12, 0x0A24},	//70000AB2        
{0x0F12, 0x1701},	//70000AB4        
{0x0F12, 0x0229},	//70000AB6        
{0x0F12, 0x1403},	//70000AB8        
{0x0F12, 0x0004},	//70000ABA        
{0x0F12, 0x0300},	//70000ABC        
{0x0F12, 0x0000},	//70000ABE        
{0x0F12, 0x02FF},	//70000AC0        
{0x0F12, 0x05DE},	//70000AC2        
{0x0F12, 0x1414},	//70000AC4        
{0x0F12, 0x0301},	//70000AC6        
{0x0F12, 0x0007},	//70000AC8        
{0x0F12, 0x1000},	//70000ACA        
{0x0F12, 0x2803},	//70000ACC        
{0x0F12, 0x261E},	//70000ACE        
{0x0F12, 0x0026},	//70000AD0        
{0x0F12, 0x1E80},	//70000AD2        
{0x0F12, 0x1E08},	//70000AD4        
{0x0F12, 0x010A},	//70000AD6        
{0x0F12, 0x0001},	//70000AD8        
{0x0F12, 0x3C0A},	//70000ADA        
{0x0F12, 0x2300},	//70000ADC        
{0x0F12, 0x0200},	//70000ADE        
{0x0F12, 0xFF00},	//70000AE0        
{0x0F12, 0x0200},	//70000AE2        
{0x0F12, 0x1B11},	//70000AE4        
{0x0F12, 0x0000},	//70000AE6        
{0x0F12, 0x0009},	//70000AE8        
{0x0F12, 0x0406},	//70000AEA        
{0x0F12, 0x0605},	//70000AEC        
{0x0F12, 0x0307},	//70000AEE        
{0x0F12, 0x0609},	//70000AF0        
{0x0F12, 0x1C07},	//70000AF2        
{0x0F12, 0x1014},	//70000AF4        
{0x0F12, 0x0510},	//70000AF6	//[15:8]iUVNRStrengthL, [7:0]iMaxThreshH        
{0x0F12, 0x8005},	//70000AF8	//[7:0]iUVNRStrengthH        
{0x0F12, 0x0080},	//70000AFA        
{0x0F12, 0x0080},	//70000AFC        
{0x0F12, 0x0080},	//70000AFE        
{0x0F12, 0x0101},	//70000B00        
{0x0F12, 0x0707},	//70000B02        
{0x0F12, 0x4B01},	//70000B04        
{0x0F12, 0x2A4B},	//70000B06        
{0x0F12, 0x5020},	//70000B08        
{0x0F12, 0x0500},	//70000B0A        
{0x0F12, 0x1C03},	//70000B0C        
{0x0F12, 0x0D0C},	//70000B0E        
{0x0F12, 0x0823},	//70000B10        
{0x0F12, 0x1428},	//70000B12        
{0x0F12, 0x4101},	//70000B14        
{0x0F12, 0x282D},	//70000B16        
{0x0F12, 0x2012},	//70000B18        
{0x0F12, 0x0204},	//70000B1A        
{0x0F12, 0x2803},	//70000B1C        
{0x0F12, 0x0128},	//70000B1E        
{0x0F12, 0x0101},	//70000B20        
{0x0F12, 0x2224},	//70000B22        
{0x0F12, 0x3236},	//70000B24        
{0x0F12, 0x0028},	//70000B26        
{0x0F12, 0x030A},	//70000B28        
{0x0F12, 0x0410},	//70000B2A        
{0x0F12, 0x141E},	//70000B2C        
{0x0F12, 0xFF07},	//70000B2E        
{0x0F12, 0x0432},	//70000B30        
{0x0F12, 0x4050},	//70000B32        
{0x0F12, 0x0F0F},	//70000B34        
{0x0F12, 0x0440},	//70000B36        
{0x0F12, 0x0302},	//70000B38        
{0x0F12, 0x2828},	//70000B3A        
{0x0F12, 0x0101},	//70000B3C        
{0x0F12, 0x2401},	//70000B3E        
{0x0F12, 0x3622},	//70000B40        
{0x0F12, 0x2832},	//70000B42        
{0x0F12, 0x0A00},	//70000B44        
{0x0F12, 0x1003},	//70000B46        
{0x0F12, 0x1E04},	//70000B48        
{0x0F12, 0x0714},	//70000B4A        
{0x0F12, 0x32FF},	//70000B4C        
{0x0F12, 0x5004},	//70000B4E        
{0x0F12, 0x0F40},	//70000B50        
{0x0F12, 0x400F},	//70000B52        
{0x0F12, 0x0204},	//70000B54        
{0x0F12, 0x0003},	//70000B56        
{0x0F12, 0x0001},	//70000B58        
{0x0F12, 0x0000},	//70000B5A	//AFIT NoiseIndex3 Brightness
{0x0F12, 0x0000},	//70000B5C        
{0x0F12, 0x0000},	//70000B5E        
{0x0F12, 0x0000},	//70000B60        
{0x0F12, 0x0000},	//70000B62        
{0x0F12, 0x00C1},	//70000B64        
{0x0F12, 0x0000},	//70000B66        
{0x0F12, 0x03FF},	//70000B68        
{0x0F12, 0x009E},	//70000B6A        
{0x0F12, 0x017C},	//70000B6C        
{0x0F12, 0x03FF},	//70000B6E        
{0x0F12, 0x000C},	//70000B70        
{0x0F12, 0x0010},	//70000B72        
{0x0F12, 0x00C8},	//70000B74        
{0x0F12, 0x03E8},	//70000B76        
{0x0F12, 0x0046},	//70000B78        
{0x0F12, 0x0050},	//70000B7A        
{0x0F12, 0x0070},	//70000B7C        
{0x0F12, 0x0000},	//70000B7E	//[15:0]iHystThLow        
{0x0F12, 0x0000},	//70000B80	//[15:0]iHystThHigh        
{0x0F12, 0x0320},	//70000B82	//[15:0]iHystCenter        
{0x0F12, 0x008C},	//70000B84        
{0x0F12, 0x0014},	//70000B86        
{0x0F12, 0x002D},	//70000B88        
{0x0F12, 0x0019},	//70000B8A        
{0x0F12, 0x002D},	//70000B8C        
{0x0F12, 0x0019},	//70000B8E        
{0x0F12, 0x0A24},	//70000B90        
{0x0F12, 0x1701},	//70000B92        
{0x0F12, 0x0229},	//70000B94        
{0x0F12, 0x1403},	//70000B96        
{0x0F12, 0x0004},	//70000B98        
{0x0F12, 0x0300},	//70000B9A        
{0x0F12, 0x0000},	//70000B9C        
{0x0F12, 0x02FF},	//70000B9E        
{0x0F12, 0x05DE},	//70000BA0        
{0x0F12, 0x1414},	//70000BA2        
{0x0F12, 0x0301},	//70000BA4        
{0x0F12, 0x0007},	//70000BA6        
{0x0F12, 0x1000},	//70000BA8        
{0x0F12, 0x2303},	//70000BAA        
{0x0F12, 0x231A},	//70000BAC        
{0x0F12, 0x0023},	//70000BAE        
{0x0F12, 0x1E80},	//70000BB0        
{0x0F12, 0x1E08},	//70000BB2        
{0x0F12, 0x010A},	//70000BB4        
{0x0F12, 0x0001},	//70000BB6        
{0x0F12, 0x3C0A},	//70000BB8        
{0x0F12, 0x2300},	//70000BBA        
{0x0F12, 0x0200},	//70000BBC        
{0x0F12, 0xFF00},	//70000BBE        
{0x0F12, 0x0200},	//70000BC0        
{0x0F12, 0x1E10},	//70000BC2        
{0x0F12, 0x0000},	//70000BC4        
{0x0F12, 0x0009},	//70000BC6        
{0x0F12, 0x0406},	//70000BC8        
{0x0F12, 0x0705},	//70000BCA        
{0x0F12, 0x0306},	//70000BCC        
{0x0F12, 0x0509},	//70000BCE        
{0x0F12, 0x2806},	//70000BD0        
{0x0F12, 0x1428},	//70000BD2        
{0x0F12, 0x0518},	//70000BD4	//[15:8]iUVNRStrengthL, [7:0]iMaxThreshH        
{0x0F12, 0x8005},	//70000BD6	//[7:0]iUVNRStrengthH        
{0x0F12, 0x0080},	//70000BD8        
{0x0F12, 0x0080},	//70000BDA        
{0x0F12, 0x0080},	//70000BDC        
{0x0F12, 0x0101},	//70000BDE        
{0x0F12, 0x0707},	//70000BE0        
{0x0F12, 0x4B01},	//70000BE2        
{0x0F12, 0x2A4B},	//70000BE4        
{0x0F12, 0x5020},	//70000BE6        
{0x0F12, 0x0500},	//70000BE8        
{0x0F12, 0x1C03},	//70000BEA        
{0x0F12, 0x0D0C},	//70000BEC        
{0x0F12, 0x0823},	//70000BEE        
{0x0F12, 0x1428},	//70000BF0        
{0x0F12, 0x4101},	//70000BF2        
{0x0F12, 0x282D},	//70000BF4        
{0x0F12, 0x2012},	//70000BF6        
{0x0F12, 0x0204},	//70000BF8        
{0x0F12, 0x3C03},	//70000BFA        
{0x0F12, 0x013C},	//70000BFC        
{0x0F12, 0x0101},	//70000BFE        
{0x0F12, 0x1C1E},	//70000C00        
{0x0F12, 0x1E22},	//70000C02        
{0x0F12, 0x0028},	//70000C04        
{0x0F12, 0x030A},	//70000C06        
{0x0F12, 0x0214},	//70000C08        
{0x0F12, 0x0E14},	//70000C0A        
{0x0F12, 0xFF06},	//70000C0C        
{0x0F12, 0x0432},	//70000C0E        
{0x0F12, 0x4052},	//70000C10        
{0x0F12, 0x150C},	//70000C12        
{0x0F12, 0x0440},	//70000C14        
{0x0F12, 0x0302},	//70000C16        
{0x0F12, 0x3C3C},	//70000C18        
{0x0F12, 0x0101},	//70000C1A        
{0x0F12, 0x1E01},	//70000C1C        
{0x0F12, 0x221C},	//70000C1E        
{0x0F12, 0x281E},	//70000C20        
{0x0F12, 0x0A00},	//70000C22        
{0x0F12, 0x1403},	//70000C24        
{0x0F12, 0x1402},	//70000C26        
{0x0F12, 0x060E},	//70000C28        
{0x0F12, 0x32FF},	//70000C2A        
{0x0F12, 0x5204},	//70000C2C        
{0x0F12, 0x0C40},	//70000C2E        
{0x0F12, 0x4015},	//70000C30        
{0x0F12, 0x0204},	//70000C32        
{0x0F12, 0x0003},	//70000C34        
{0x0F12, 0x0001},	//70000C36        
{0x0F12, 0x0000},	//70000C38	//AFIT NoiseIndex4 Brightness        
{0x0F12, 0x0000},	//70000C3A        
{0x0F12, 0x0000},	//70000C3C        
{0x0F12, 0x0000},	//70000C3E        
{0x0F12, 0x0000},	//70000C40        
{0x0F12, 0x00C1},	//70000C42        
{0x0F12, 0x0000},	//70000C44        
{0x0F12, 0x03FF},	//70000C46        
{0x0F12, 0x0008},	//70000C48        
{0x0F12, 0x017C},	//70000C4A        
{0x0F12, 0x03FF},	//70000C4C        
{0x0F12, 0x000C},	//70000C4E        
{0x0F12, 0x0010},	//70000C50        
{0x0F12, 0x0032},	//70000C52        
{0x0F12, 0x028A},	//70000C54        
{0x0F12, 0x0032},	//70000C56        
{0x0F12, 0x01F4},	//70000C58        
{0x0F12, 0x0070},	//70000C5A        
{0x0F12, 0x0002},	//70000C5C	//[15:0]iHystThLow        
{0x0F12, 0x0000},	//70000C5E	//[15:0]iHystThHigh        
{0x0F12, 0x0320},	//70000C60	//[15:0]iHystCenter        
{0x0F12, 0x0070},	//70000C62	//AFIT NoiseIndex4 iLowSharpClamp        
{0x0F12, 0x0014},	//70000C64	//AFIT NoiseIndex4 iHighSharpClamp
{0x0F12, 0x0046},	//70000C66        
{0x0F12, 0x0019},	//70000C68        
{0x0F12, 0x0046},	//70000C6A        
{0x0F12, 0x0019},	//70000C6C        
{0x0F12, 0x0A24},	//70000C6E        
{0x0F12, 0x1701},	//70000C70        
{0x0F12, 0x0229},	//70000C72        
{0x0F12, 0x0503},	//70000C74        
{0x0F12, 0x0101},	//70000C76        
{0x0F12, 0x0101},	//70000C78        
{0x0F12, 0x0000},	//70000C7A        
{0x0F12, 0x02FF},	//70000C7C        
{0x0F12, 0x0496},	//70000C7E        
{0x0F12, 0x1414},	//70000C80        
{0x0F12, 0x0301},	//70000C82        
{0x0F12, 0x0007},	//70000C84        
{0x0F12, 0x1000},	//70000C86        
{0x0F12, 0x2003},	//70000C88        
{0x0F12, 0x1020},	//70000C8A        
{0x0F12, 0x0010},	//70000C8C        
{0x0F12, 0x1E80},	//70000C8E        
{0x0F12, 0x1E06},	//70000C90        
{0x0F12, 0x030C},	//70000C92	//[15:8] iGRDenoiseVal        
{0x0F12, 0x0103},	//70000C94	//[7:0] iGBDenoiseVal        
{0x0F12, 0x5A0A},	//70000C96	//AFIT NoiseIndex4 [15:8] iMSharpen
{0x0F12, 0x2D00},	//70000C98	//AFIT NoiseIndex4 [15:8] iWSharpen
{0x0F12, 0x0100},	//70000C9A        
{0x0F12, 0xFF00},	//70000C9C        
{0x0F12, 0x0200},	//70000C9E        
{0x0F12, 0x1E10},	//70000CA0        
{0x0F12, 0x0000},	//70000CA2        
{0x0F12, 0x0009},	//70000CA4        
{0x0F12, 0x0406},	//70000CA6        
{0x0F12, 0x0705},	//70000CA8        
{0x0F12, 0x0305},	//70000CAA        
{0x0F12, 0x0609},	//70000CAC        
{0x0F12, 0x2C07},	//70000CAE        
{0x0F12, 0x142C},	//70000CB0        
{0x0F12, 0x0718},	//70000CB2	//[15:8]iUVNRStrengthL, [7:0]iMaxThreshH        
{0x0F12, 0x8007},	//70000CB4	//[7:0]iUVNRStrengthH        
{0x0F12, 0x0F80},	//70000CB6        
{0x0F12, 0x0080},	//70000CB8        
{0x0F12, 0x0080},	//70000CBA        
{0x0F12, 0x0101},	//70000CBC        
{0x0F12, 0x0A0A},	//70000CBE        
{0x0F12, 0x3201},	//70000CC0        
{0x0F12, 0x1428},	//70000CC2	//[15:8]iDenThreshLow        
{0x0F12, 0x100C},	//70000CC4	//[7:0]iDenThreshHigh
{0x0F12, 0x0500},	//70000CC6        
{0x0F12, 0x1E02},	//70000CC8	//AFIT NoiseIndex4 [15:8] iDemSharpenLow
{0x0F12, 0x0414},	//70000CCA	//AFIT NoiseIndex4 [7:0] iDemSharpenHigh
{0x0F12, 0x0828},	//70000CCC        
{0x0F12, 0x5064},	//70000CCE        
{0x0F12, 0x4605},	//70000CD0	//4605	//AFIT NoiseIndex4 [15:8] iLowSharpPower
{0x0F12, 0x1EA0},	//70000CD2	//AFIT NoiseIndex4 [7:0] iHighSharpPower
{0x0F12, 0x201E},	//70000CD4
{0x0F12, 0x0604},	//70000CD6
{0x0F12, 0x4606},	//70000CD8
{0x0F12, 0x0146},	//70000CDA
{0x0F12, 0x0101},	//70000CDC
{0x0F12, 0x1C18},	//70000CDE
{0x0F12, 0x1819},	//70000CE0
{0x0F12, 0x0028},	//70000CE2
{0x0F12, 0x030A},	//70000CE4
{0x0F12, 0x0514},	//70000CE6
{0x0F12, 0x0C14},	//70000CE8
{0x0F12, 0xFF05},	//70000CEA
{0x0F12, 0x0432},	//70000CEC
{0x0F12, 0x4052},	//70000CEE
{0x0F12, 0x1514},	//70000CF0
{0x0F12, 0x0440},	//70000CF2
{0x0F12, 0x0302},	//70000CF4
{0x0F12, 0x4646},	//70000CF6
{0x0F12, 0x0101},	//70000CF8
{0x0F12, 0x1801},	//70000CFA
{0x0F12, 0x191C},	//70000CFC
{0x0F12, 0x2818},	//70000CFE
{0x0F12, 0x0A00},	//70000D00
{0x0F12, 0x1403},	//70000D02
{0x0F12, 0x1405},	//70000D04
{0x0F12, 0x050C},	//70000D06
{0x0F12, 0x32FF},	//70000D08
{0x0F12, 0x5204},	//70000D0A
{0x0F12, 0x1440},	//70000D0C
{0x0F12, 0x4015},	//70000D0E
{0x0F12, 0x0204},	//70000D10
{0x0F12, 0x0003},	//70000D12
{0x0F12, 0x0001},	//70000D14

{0x0F12, 0xBA7A},	//70000D16
{0x0F12, 0x4FDE},	//70000D18
{0x0F12, 0x137F},	//70000D1A
{0x0F12, 0x3BDE},	//70000D1C
{0x0F12, 0xBF02},	//70000D1E
{0x0F12, 0x00B5},	//70000D20


{0x0028, 0x7000},
{0x002A, 0x0150},
{0x0F12, 0xAAAA},       //ESD Check
};

#define S5K5CCGX_INIT_SET_INDEX (sizeof(S5K5CCGX_INIT_SET) / sizeof(S5K5CCGX_INIT_SET[0]))


//==========================================================
//  EFFECT(6)
//==========================================================
static const s5k5ccgx_short_t S5K5CCGX_CAM_EFFECT_OFF[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0648},
{0x0F12, 0x0001},		//skl_af_bPregmOff	Pre/Post Gamma Off (����)

{0x002A, 0x01E2},
{0x0F12, 0x0000},		//REG_TC_GP_SpecialEffects	00:Normal Mode
};

#define S5K5CCGX_CAM_EFFECT_OFF_INDEX (sizeof(S5K5CCGX_CAM_EFFECT_OFF) / sizeof(S5K5CCGX_CAM_EFFECT_OFF[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_EFFECT_MONO[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0001},		//REG_TC_GP_SpecialEffects	01:Mono Mode
};

#define S5K5CCGX_CAM_EFFECT_MONO_INDEX (sizeof(S5K5CCGX_CAM_EFFECT_MONO) / sizeof(S5K5CCGX_CAM_EFFECT_MONO[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_EFFECT_SEPIA[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0004},		//REG_TC_GP_SpecialEffects	04:Sepia Mode
};

#define S5K5CCGX_CAM_EFFECT_SEPIA_INDEX (sizeof(S5K5CCGX_CAM_EFFECT_SEPIA) / sizeof(S5K5CCGX_CAM_EFFECT_SEPIA[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_EFFECT_NEGATIVE[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0003},		//REG_TC_GP_SpecialEffects	03:Negative Mode
};

#define S5K5CCGX_CAM_EFFECT_NEGATIVE_INDEX (sizeof(S5K5CCGX_CAM_EFFECT_NEGATIVE) / sizeof(S5K5CCGX_CAM_EFFECT_NEGATIVE[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_EFFECT_AQUA[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01E2},
{0x0F12, 0x0005},		//REG_TC_GP_SpecialEffects	05:Aqua Mode
};

#define S5K5CCGX_CAM_EFFECT_AQUA_INDEX (sizeof(S5K5CCGX_CAM_EFFECT_AQUA) / sizeof(S5K5CCGX_CAM_EFFECT_AQUA[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_EFFECT_SKETCH[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0648},
{0x0F12, 0x0000},		//skl_af_bPregmOff	Pre/Post Gamma On

{0x002A, 0x01E2},
{0x0F12, 0x0006},		//REG_TC_GP_SpecialEffects	06:Sketch Mode
};

#define S5K5CCGX_CAM_EFFECT_SKETCH_INDEX (sizeof(S5K5CCGX_CAM_EFFECT_SKETCH) / sizeof(S5K5CCGX_CAM_EFFECT_SKETCH[0]))

//==========================================================
// MWB(5)
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_CAM_WB_AUTO[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0001},		//Mon_AAIO_bAWB		AWB ON
};

#define S5K5CCGX_CAM_WB_AUTO_INDEX (sizeof(S5K5CCGX_CAM_WB_AUTO) / sizeof(S5K5CCGX_CAM_WB_AUTO[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_WB_DAYLIGHT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x0600},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x0526},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_CAM_WB_DAYLIGHT_INDEX (sizeof(S5K5CCGX_CAM_WB_DAYLIGHT) / sizeof(S5K5CCGX_CAM_WB_DAYLIGHT[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_WB_CLOUDY[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x07D0},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x04A0},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_CAM_WB_CLOUDY_INDEX (sizeof(S5K5CCGX_CAM_WB_CLOUDY) / sizeof(S5K5CCGX_CAM_WB_CLOUDY[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_WB_FLUORESCENT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x0570},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x08E0},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_CAM_WB_FLUORESCENT_INDEX (sizeof(S5K5CCGX_CAM_WB_FLUORESCENT) / sizeof(S5K5CCGX_CAM_WB_FLUORESCENT[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_WB_INCANDESCENT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x03D0},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x0970},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_CAM_WB_INCANDESCENT_INDEX (sizeof(S5K5CCGX_CAM_WB_INCANDESCENT) / sizeof(S5K5CCGX_CAM_WB_INCANDESCENT[0]))

//==========================================================
// BRIGHTNES(9)
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_N_4[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x001C},		//REG_TC_UserBrightness 
//s002A1308	
//s0F12001E		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_N_4_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_N_4) / sizeof(S5K5CCGX_BRIGHTNESS_N_4[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_N_3[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x0020},		//REG_TC_UserBrightness 

//s002A1308
//s0F120027		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_N_3_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_N_3) / sizeof(S5K5CCGX_BRIGHTNESS_N_3[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_N_2[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x0028},		//REG_TC_UserBrightness 

//s002A1308
//s0F12002F		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_N_2_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_N_2) / sizeof(S5K5CCGX_BRIGHTNESS_N_2[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_N_1[]=
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x0032},		//REG_TC_UserBrightness 

//s002A1308
//s0F120038		//TVAR_ae_BrAve

};

#define S5K5CCGX_BRIGHTNESS_N_1_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_N_1) / sizeof(S5K5CCGX_BRIGHTNESS_N_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_0[]=
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x003E},		//REG_TC_UserBrightness 

//s002A1308
//s0F120040		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_0_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_0) / sizeof(S5K5CCGX_BRIGHTNESS_0[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_P_1[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x0058},		//REG_TC_UserBrightness 

//s002A1308
//s0F120043		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_P_1_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_P_1) / sizeof(S5K5CCGX_BRIGHTNESS_P_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_P_2[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1308},
{0x0F12, 0x006D},		//REG_TC_UserBrightness  

//s002A1308
//s0F120046		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_P_2_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_P_2) / sizeof(S5K5CCGX_BRIGHTNESS_P_2[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_P_3[]=
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x1308},
{0x0F12, 0x0080},		//REG_TC_UserBrightness

//s002A1308
//s0F120049		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_P_3_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_P_3) / sizeof(S5K5CCGX_BRIGHTNESS_P_3[0]))

static const s5k5ccgx_short_t S5K5CCGX_BRIGHTNESS_P_4[]=
{
{0xFCFC, 0xD000},                  
{0x0028, 0x7000},                  
{0x002A, 0x1308},                  
{0x0F12, 0x0090},		//REG_TC_UserBrightness

//s002A0F70                  
//s0F12004C		//TVAR_ae_BrAve
};

#define S5K5CCGX_BRIGHTNESS_P_4_INDEX (sizeof(S5K5CCGX_BRIGHTNESS_P_4) / sizeof(S5K5CCGX_BRIGHTNESS_P_4[0]))


//==========================================================
// contrast(5)
//==========================================================
static const s5k5ccgx_short_t S5K5CCGX_CONTRAST_M_2[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0xFF80},		//REG_TC_UserContrast
};

#define S5K5CCGX_CONTRAST_M_2_INDEX (sizeof(S5K5CCGX_CONTRAST_M_2) / sizeof(S5K5CCGX_CONTRAST_M_2[0]))

static const s5k5ccgx_short_t S5K5CCGX_CONTRAST_M_1[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0xFFC0},		//REG_TC_UserContrast
};

#define S5K5CCGX_CONTRAST_M_1_INDEX (sizeof(S5K5CCGX_CONTRAST_M_1) / sizeof(S5K5CCGX_CONTRAST_M_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_CONTRAST_0[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0x0000},		//REG_TC_UserContrast
};

#define S5K5CCGX_CONTRAST_0_INDEX (sizeof(S5K5CCGX_CONTRAST_0) / sizeof(S5K5CCGX_CONTRAST_0[0]))

static const s5k5ccgx_short_t S5K5CCGX_CONTRAST_P_1[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0x0040},		//REG_TC_UserContrast
};

#define S5K5CCGX_CONTRAST_P_1_INDEX (sizeof(S5K5CCGX_CONTRAST_P_1) / sizeof(S5K5CCGX_CONTRAST_P_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_CONTRAST_P_2[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D2},
{0x0F12, 0x0080},		//REG_TC_UserContrast
};

#define S5K5CCGX_CONTRAST_P_2_INDEX (sizeof(S5K5CCGX_CONTRAST_P_2) / sizeof(S5K5CCGX_CONTRAST_P_2[0]))

//==========================================================
// saturation(5)
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_SATURATION_M_2[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0xFF80},		//REG_TC_UserContrast
};

#define S5K5CCGX_SATURATION_M_2_INDEX (sizeof(S5K5CCGX_SATURATION_M_2) / sizeof(S5K5CCGX_SATURATION_M_2[0]))

static const s5k5ccgx_short_t S5K5CCGX_SATURATION_M_1[]= 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0xFFC0},		//REG_TC_UserContrast
};

#define S5K5CCGX_SATURATION_M_1_INDEX (sizeof(S5K5CCGX_SATURATION_M_1) / sizeof(S5K5CCGX_SATURATION_M_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_SATURATION_0[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0x0000},		//REG_TC_UserContrast
};

#define S5K5CCGX_SATURATION_0_INDEX (sizeof(S5K5CCGX_SATURATION_0) / sizeof(S5K5CCGX_SATURATION_0[0]))

static const s5k5ccgx_short_t S5K5CCGX_SATURATION_P_1[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0x0040},		//REG_TC_UserContrast
};

#define S5K5CCGX_SATURATION_P_1_INDEX (sizeof(S5K5CCGX_SATURATION_P_1) / sizeof(S5K5CCGX_SATURATION_P_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_SATURATION_P_2[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D4},
{0x0F12, 0x0080},		//REG_TC_UserContrast
};

#define S5K5CCGX_SATURATION_P_2_INDEX (sizeof(S5K5CCGX_SATURATION_P_2) / sizeof(S5K5CCGX_SATURATION_P_2[0]))

//==========================================================
// sharpness(5)
//==========================================================
static const s5k5ccgx_short_t S5K5CCGX_SHAPNESS_M_2[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0xFFC0},		//REG_TC_UserContrast
};

#define S5K5CCGX_SHAPNESS_M_2_INDEX (sizeof(S5K5CCGX_SHAPNESS_M_2) / sizeof(S5K5CCGX_SHAPNESS_M_2[0]))

static const s5k5ccgx_short_t S5K5CCGX_SHAPNESS_M_1[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0xFFE0},		//REG_TC_UserContrast
};

#define S5K5CCGX_SHAPNESS_M_1_INDEX (sizeof(S5K5CCGX_SHAPNESS_M_1) / sizeof(S5K5CCGX_SHAPNESS_M_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_SHAPNESS_0[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0x0000},		//REG_TC_UserContrast
};

#define S5K5CCGX_SHAPNESS_0_INDEX (sizeof(S5K5CCGX_SHAPNESS_0) / sizeof(S5K5CCGX_SHAPNESS_0[0]))

static const s5k5ccgx_short_t S5K5CCGX_SHAPNESS_P_1[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0x0020},		//REG_TC_UserContrast
};

#define S5K5CCGX_SHAPNESS_P_1_INDEX (sizeof(S5K5CCGX_SHAPNESS_P_1) / sizeof(S5K5CCGX_SHAPNESS_P_1[0]))

static const s5k5ccgx_short_t S5K5CCGX_SHAPNESS_P_2[] = 
{
{0xFCFC, 0xD000}, 
{0x0028, 0x7000}, 
{0x002A, 0x01D6},
{0x0F12, 0x0040},		//REG_TC_UserContrast
};

#define S5K5CCGX_SHAPNESS_P_2_INDEX (sizeof(S5K5CCGX_SHAPNESS_P_2) / sizeof(S5K5CCGX_SHAPNESS_P_2[0]))

//==========================================================
//SCENE()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_OFF[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0001},		//Mon_AAIO_bAWB		0: AWB OFF, 1: AWB ON

//	01. Portait / Landscape / Text / Fall Color Off

{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x0000},		//REG_TC_UserSaturation
{0x0F12, 0x0000},		//REG_TC_UserSharpBlur


//	02. Night / Firework Off

{0x0028, 0x7000},
{0x002A, 0x0504},
{0x0F12, 0x3415},		//lt_uMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0508},
{0x0F12, 0x681F},		//lt_uMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x050C},
{0x0F12, 0x8227},		//lt_uMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms
{0x0F12, 0x0000},

{0x002A, 0x0514},
{0x0F12, 0x3415},		//lt_uCapMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0518},
{0x0F12, 0x681F},		//lt_uCapMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x051C},
{0x0F12, 0x8227},		//lt_uCapMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms
{0x0F12, 0x0000},

{0x002A, 0x0524},
{0x0F12, 0x0230},		//lt_uMaxAnGain1		0180h	= 0384d	= x1.5000
{0x0F12, 0x0230},		//lt_uMaxAnGain2		0180h	= 0384d	= x1.5000
{0x0F12, 0x0300},		//lt_uMaxAnGain3		0250h	= 0592d	= x2.3125
{0x0F12, 0x0A00},		//lt_uMaxAnGain4		0710h	= 1808d	= x7.0625

{0x0F12, 0x0100},		//lt_uMaxDigGain
{0x0F12, 0x8000},		//lt_uMaxTotGain

{0x0F12, 0x0230},		//lt_uCapMaxAnGain1		0180h	= 0384d	= x1.5000
{0x0F12, 0x0230},		//lt_uCapMaxAnGain2		0180h	= 0384d	= x1.5000
{0x0F12, 0x0300},		//lt_uCapMaxAnGain3		0250h	= 0592d	= x2.3125
{0x0F12, 0x0710},		//lt_uCapMaxAnGain4		0710h	= 1808d	= x7.0625

{0x0F12, 0x0100},		//lt_uCapMaxDigGain
{0x0F12, 0x8000},		//lt_uCapMaxTotGain

//0x002A, 0x08E4
//0x0F12, 0x000C		//AFIT16_demsharpmix1_iHystThLow
//0x0F12, 0x000C		//AFIT16_demsharpmix1_iHystThHigh
//0x002A, 0x0940
//0x0F12, 0x084B		//[15:8] AFIT8_RGB2YUV_iYOffset, [7:0] AFIT8_ccm_oscar_iSaturation


//	03. ISO Auto




{0x002A, 0x04A4},
{0x0F12, 0x067F},		//REG_TC_DBG_AutoAlgEnBits		Auto Algorithm Enable




{0x002A, 0x0486},
{0x0F12, 0x0000},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x002A, 0x048A},
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

{0x002A, 0x3302},
{0x0F12, 0x0000},		//AFIT by Normalized Brightness Tunning Parameter

//Preview
{0x002A, 0x0208},
{0x0F12, 0x0000},		//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},		//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},		//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},		//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},		//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},		//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},		//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},		//REG_TC_GP_EnablePreviewChanged   
};

#define S5K5CCGX_CAM_SCENE_OFF_INDEX (sizeof(S5K5CCGX_CAM_SCENE_OFF) / sizeof(S5K5CCGX_CAM_SCENE_OFF[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_PORTRAIT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x0000},		//REG_TC_UserSaturation
{0x0F12, 0xFFF6},		//REG_TC_UserSharpBlur
};

#define S5K5CCGX_CAM_SCENE_PORTRAIT_INDEX (sizeof(S5K5CCGX_CAM_SCENE_PORTRAIT) / sizeof(S5K5CCGX_CAM_SCENE_PORTRAIT[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_LANDSCAPE[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x001E},		//REG_TC_UserSaturation
{0x0F12, 0x000A},		//REG_TC_UserSharpBlur
};

#define S5K5CCGX_CAM_SCENE_LANDSCAPE_INDEX (sizeof(S5K5CCGX_CAM_SCENE_LANDSCAPE) / sizeof(S5K5CCGX_CAM_SCENE_LANDSCAPE[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_SPORTS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x0504},
{0x0F12, 0x3415},		//lt_uMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0508},
{0x0F12, 0x3415},		//lt_uMaxExp2		3415h = 13333d =  33.3325ms
{0x002A, 0x050C},
{0x0F12, 0x3415},		//lt_uMaxExp3		3415h = 13333d =  33.3325ms
{0x002A, 0x0510},
{0x0F12, 0x3415},		//lt_uMaxExp4		3415h = 13333d =  33.3325ms

{0x002A, 0x0514},
{0x0F12, 0x3415},		//lt_uCapMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0518},
{0x0F12, 0x3415},		//lt_uCapMaxExp2		3415h = 13333d =  33.3325ms
{0x002A, 0x051C},
{0x0F12, 0x3415},		//lt_uCapMaxExp3		3415h = 13333d =  33.3325ms
{0x002A, 0x0520},
{0x0F12, 0x3415},		//lt_uCapMaxExp4		3415h = 13333d =  33.3325ms

//0x002A, 0x05EA},
//0x0F12, 0x0100},		//lt_bUseSecISODgain

//0x002A, 0x0486},
//0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
//0x0F12, 0x0200},		//REG_SF_USER_IsoVal		0200h = 512d = x2
//0x0F12, 0x0001},		//REG_SF_USER_IsoChanged
};

#define S5K5CCGX_CAM_SCENE_SPORTS_INDEX (sizeof(S5K5CCGX_CAM_SCENE_SPORTS) / sizeof(S5K5CCGX_CAM_SCENE_SPORTS[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_PARTY[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x001E},		//REG_TC_UserSaturation
{0x0F12, 0x0000},		//REG_TC_UserSharpBlur

//ISO 200
//0x002A, 0x167C},  
//0x0F12, 0x2000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

{0x002A, 0x0504},
{0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0508},
{0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x050C},
{0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0518},
{0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x051C},
{0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0400},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

//Preview
{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged 
};

#define S5K5CCGX_CAM_SCENE_PARTY_INDEX (sizeof(S5K5CCGX_CAM_SCENE_PARTY) / sizeof(S5K5CCGX_CAM_SCENE_PARTY[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_BEACH[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0014},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x001E},		//REG_TC_UserSaturation
{0x0F12, 0x0000},		//REG_TC_UserSharpBlur

//ISO 50
//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

{0x002A, 0x0504},
{0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0508},
{0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x050C},
{0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0518},
{0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x051C},
{0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0100},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

//Preview
{0x002A, 0x0208},
{0x0F12, 0x0000},		//REG_TC_GP_ActivePrevConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},		//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},		//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},		//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},		//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},		//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},		//REG_TC_GP_EnablePreviewChanged 
};

#define S5K5CCGX_CAM_SCENE_BEACH_INDEX (sizeof(S5K5CCGX_CAM_SCENE_BEACH) / sizeof(S5K5CCGX_CAM_SCENE_BEACH[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_SUNSET[]=
{
// Use MWB Daylight
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x0600},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x0526},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_CAM_SCENE_SUNSET_INDEX (sizeof(S5K5CCGX_CAM_SCENE_SUNSET) / sizeof(S5K5CCGX_CAM_SCENE_SUNSET[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_DAWN[]=
{
// Use MWB CWF
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x0555},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x07F6},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_CAM_SCENE_DAWN_INDEX (sizeof(S5K5CCGX_CAM_SCENE_DAWN) / sizeof(S5K5CCGX_CAM_SCENE_DAWN[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_FALL[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x0032},		//REG_TC_UserSaturation
{0x0F12, 0x000A},		//REG_TC_UserSharpBlur
};

#define S5K5CCGX_CAM_SCENE_FALL_INDEX (sizeof(S5K5CCGX_CAM_SCENE_FALL) / sizeof(S5K5CCGX_CAM_SCENE_FALL[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_NIGHT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},  
//s002A029E		//Preview Configuraion 2
//s0F120400		//REG_2TC_PCFG_usWidth										0400h = 1024d
//s0F120300   //REG_2TC_PCFG_usHeight										0300h =  768d
//s0F120005   //REG_2TC_PCFG_Format											5:YUV, 9:JPEG
//s0F1234CC   //REG_2TC_PCFG_usMaxOut4KHzRate						34CCh = 13516d = 54.064Mhz
//s0F1234AC   //REG_2TC_PCFG_usMinOut4KHzRate						34ACh = 13484d = 53.936Mhz
//s002A02AC
//s0F120042   //REG_2TC_PCFG_PVIMask										[1]:PCLK Inversion, [4]:UV First, [5]:V First
//s0F120010   //REG_2TC_PCFG_OIFMask										[4]:ITU656
//s002A02B4
//s0F120000   //REG_2TC_PCFG_uClockInd
//s0F120000   //REG_2TC_PCFG_usFrTimeType								0:Dynamic, 2:Fixed
//s0F120001   //REG_2TC_PCFG_FrRateQualityType					0:Dynamic, 1:FrameRate, 2:Quality
//s0F1209C4   //REG_2TC_PCFG_usMaxFrTimeMsecMult10			09C4h = 250.0ms =  4fps
//s0F12014E   //REG_2TC_PCFG_usMinFrTimeMsecMult10			014Dh =  33.3ms = 30fps 
//s0F120000   //REG_2TC_PCFG_bSmearOutput
//s0F120000   //REG_2TC_PCFG_sSaturation
//s0F120000   //REG_2TC_PCFG_sSharpBlur
//s0F120000   //REG_2TC_PCFG_sColorTemp
//s0F120000   //REG_2TC_PCFG_uDeviceGammaIndex
//s0F120000   //REG_2TC_PCFG_uPrevMirror
//s0F120000   //REG_2TC_PCFG_uCaptureMirror
//s0F120000   //REG_2TC_PCFG_uRotation

//s002A02BA
//s0F1209C4   //REG_2TC_PCFG_usMaxFrTimeMsecMult10			09C4h = 250.0ms =  4fps
//s0F12014E   //REG_2TC_PCFG_usMinFrTimeMsecMult10			014Dh =  33.3ms = 30fps 
             
//s002A0386		//Capture Configuration 2
//s0F120000		//REG_2TC_CCFG_uCaptureMode								0:Auto Off, 1: Auto On
//s0F120800   //REG_2TC_CCFG_usWidth										0800h = 2048d
//s0F120600   //REG_2TC_CCFG_usHeight										0600h = 1536d
//s0F120005   //REG_2TC_CCFG_Format											5:YUV, 9:JPEG
//s0F1234CC   //REG_2TC_CCFG_usMaxOut4KHzRate						34CCh = 13516d = 54.064Mhz
//s0F1234AC   //REG_2TC_CCFG_usMinOut4KHzRate						34ACh = 13484d = 53.936Mhz
//s002A0396
//s0F120042   //REG_2TC_CCFG_PVIMask										[1]:PCLK Inversion, [4]:UV First, [5]:V First
//s0F120010   //REG_2TC_CCFG_OIFMask										[4]:ITU656
//s0F1203C0		//REG_2TC_CCFG_usJpegPacketSize
//s002A039E
//s0F120000   //REG_2TC_CCFG_uClockInd
//s0F120000   //REG_2TC_CCFG_usFrTimeType								0:Dynamic, 2:Fixed
//s0F120002   //REG_2TC_CCFG_FrRateQualityType					0:Dynamic, 1:FrameRate, 2:Quality
//s0F121388   //REG_2TC_CCFG_usMaxFrTimeMsecMult10			1388h = 500.0ms =  2fps
//s0F121388   //REG_2TC_CCFG_usMinFrTimeMsecMult10			1388h = 500.0ms =  2fps
//s0F120000   //REG_2TC_CCFG_bSmearOutput
//s0F120000   //REG_2TC_CCFG_sSaturation
//s0F120000   //REG_2TC_CCFG_sSharpBlur
//s0F120000   //REG_2TC_CCFG_sColorTemp
//s0F120000   //REG_2TC_CCFG_uDeviceGammaIndex

{0x002A, 0x03A4},
{0x0F12, 0x1388},   //REG_2TC_CCFG_usMaxFrTimeMsecMult10			1388h = 500.0ms =  2fps
{0x0F12, 0x1388},   //REG_2TC_CCFG_usMinFrTimeMsecMult10			1388h = 500.0ms =  2fps

{0x002A, 0x01CC},
{0x0F12, 0x0001},   //REG_TC_IPRM_InitParamsUpdated

{0x002A, 0x0504},
{0x0F12, 0x3415},		//lt_uMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0508},
{0x0F12, 0x681F},		//lt_uMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x050C},
{0x0F12, 0x8227},		//lt_uMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0510},
{0x0F12, 0x1A80},		//lt_uMaxExp4		00061A80h = 400000d =  1000ms
{0x0F12, 0x0006},

{0x002A, 0x0514},
{0x0F12, 0x3415},		//lt_uCapMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0518},
{0x0F12, 0x681F},		//lt_uCapMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x051C},
{0x0F12, 0x8227},		//lt_uCapMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0520},
{0x0F12, 0x1A80},		//lt_uCapMaxExp4		00061A80h = 400000d =  1000ms
{0x0F12, 0x0006},

{0x002A, 0x0524},
{0x0F12, 0x01A0},		//lt_uMaxAnGain1		0180h	= 0384d	= x1.5000
{0x0F12, 0x01A0},		//lt_uMaxAnGain2		0180h	= 0384d	= x1.5000
{0x0F12, 0x02C0},		//lt_uMaxAnGain3		0250h	= 0592d	= x2.3125
{0x0F12, 0x0800},		//lt_uMaxAnGain4		0710h	= 1808d	= x7.0625

{0x0F12, 0x0100},		//lt_uMaxDigGain
{0x0F12, 0x8000},		//lt_uMaxTotGain

{0x0F12, 0x01A0},		//lt_uCapMaxAnGain1		0180h	= 0384d	= x1.5000
{0x0F12, 0x01A0},		//lt_uCapMaxAnGain2		0180h	= 0384d	= x1.5000
{0x0F12, 0x02C0},		//lt_uCapMaxAnGain3		0250h	= 0592d	= x2.3125
{0x0F12, 0x0800},		//lt_uCapMaxAnGain4		0710h	= 1808d	= x7.0625

{0x0F12, 0x0100},		//lt_uCapMaxDigGain
{0x0F12, 0x8000},		//lt_uCapMaxTotGain

//0x002A, 0x08E4
//0x0F12, 0x0000		//AFIT16_demsharpmix1_iHystThLow
//0x0F12, 0x0000		//AFIT16_demsharpmix1_iHystThHigh
//0x002A, 0x0940
//0x0F12, 0x0A3C		//[15:8] AFIT8_RGB2YUV_iYOffset, [7:0] AFIT8_ccm_oscar_iSaturation

{0x002A, 0x0208}, 
{0x0F12, 0x0002}, 	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210}, 
{0x0F12, 0x0002}, 	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C}, 
{0x0F12, 0x0001}, 	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4}, 
{0x0F12, 0x0001}, 	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A}, 
{0x0F12, 0x0001}, 	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212}, 
{0x0F12, 0x0001}, 	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4}, 
{0x0F12, 0x0001}, 	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001}, 	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_CAM_SCENE_NIGHT_INDEX (sizeof(S5K5CCGX_CAM_SCENE_NIGHT) / sizeof(S5K5CCGX_CAM_SCENE_NIGHT[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_BACKLIGHT[]=
{
//Cam_Cfg_Exposure_Spot
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1316},		//ae_WeightTbl_16

{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0f01}, 
{0x0F12, 0x010f}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0f01}, 
{0x0F12, 0x010f}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000},
};

#define S5K5CCGX_CAM_SCENE_BACKLIGHT_INDEX (sizeof(S5K5CCGX_CAM_SCENE_BACKLIGHT) / sizeof(S5K5CCGX_CAM_SCENE_BACKLIGHT[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_FIRE[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},  
//0x002A, 0x029E},		//Preview Configuraion 2
//0x0F12, 0x0400},		//REG_2TC_PCFG_usWidth										0400h = 1024d
//0x0F12, 0x0300},   //REG_2TC_PCFG_usHeight										0300h =  768d
//0x0F12, 0x0005},   //REG_2TC_PCFG_Format											5:YUV, 9:JPEG
//0x0F12, 0x34CC},   //REG_2TC_PCFG_usMaxOut4KHzRate						34CCh = 13516d = 54.064Mhz
//0x0F12, 0x34AC},   //REG_2TC_PCFG_usMinOut4KHzRate						34ACh = 13484d = 53.936Mhz
//0x002A, 0x02AC},
//0x0F12, 0x0042},   //REG_2TC_PCFG_PVIMask										[1]:PCLK Inversion, [4]:UV First, [5]:V First
//0x0F12, 0x0010},   //REG_2TC_PCFG_OIFMask										[4]:ITU656
//0x002A, 0x02B4},
//0x0F12, 0x0000},   //REG_2TC_PCFG_uClockInd
//0x0F12, 0x0000},   //REG_2TC_PCFG_usFrTimeType								0:Dynamic, 2:Fixed
//0x0F12, 0x0001},   //REG_2TC_PCFG_FrRateQualityType					0:Dynamic, 1:FrameRate, 2:Quality
//0x0F12, 0x09C4},   //REG_2TC_PCFG_usMaxFrTimeMsecMult10			09C4h = 250.0ms =  4fps
//0x0F12, 0x014E},   //REG_2TC_PCFG_usMinFrTimeMsecMult10			014Dh =  33.3ms = 30fps 
//0x0F12, 0x0000},   //REG_2TC_PCFG_bSmearOutput
//0x0F12, 0x0000},   //REG_2TC_PCFG_sSaturation
//0x0F12, 0x0000},   //REG_2TC_PCFG_sSharpBlur
//0x0F12, 0x0000},   //REG_2TC_PCFG_sColorTemp
//0x0F12, 0x0000},   //REG_2TC_PCFG_uDeviceGammaIndex
//0x0F12, 0x0000},   //REG_2TC_PCFG_uPrevMirror
//0x0F12, 0x0000},   //REG_2TC_PCFG_uCaptureMirror
//0x0F12, 0x0000},   //REG_2TC_PCFG_uRotation

//0x002A, 0x02BA},
//0x0F12, 0x09C4},   //REG_2TC_PCFG_usMaxFrTimeMsecMult10			09C4h = 250.0ms =  4fps
//0x0F12, 0x014E},   //REG_2TC_PCFG_usMinFrTimeMsecMult10			014Dh =  33.3ms = 30fps 
             
//0x002A, 0x0386},		//Capture Configuration 2
//0x0F12, 0x0000},		//REG_2TC_CCFG_uCaptureMode								0:Auto Off, 1: Auto On
//0x0F12, 0x0800},   //REG_2TC_CCFG_usWidth										0800h = 2048d
//0x0F12, 0x0600},   //REG_2TC_CCFG_usHeight										0600h = 1536d
//0x0F12, 0x0005},   //REG_2TC_CCFG_Format											5:YUV, 9:JPEG
//0x0F12, 0x34CC},   //REG_2TC_CCFG_usMaxOut4KHzRate						34CCh = 13516d = 54.064Mhz
//0x0F12, 0x34AC},   //REG_2TC_CCFG_usMinOut4KHzRate						34ACh = 13484d = 53.936Mhz
//0x002A, 0x0396},
//0x0F12, 0x0042},   //REG_2TC_CCFG_PVIMask										[1]:PCLK Inversion, [4]:UV First, [5]:V First
//0x0F12, 0x0010},   //REG_2TC_CCFG_OIFMask										[4]:ITU656
//0x0F12, 0x03C0},		//REG_2TC_CCFG_usJpegPacketSize
//0x002A, 0x039E},
//0x0F12, 0x0000},   //REG_2TC_CCFG_uClockInd
//0x0F12, 0x0000},   //REG_2TC_CCFG_usFrTimeType								0:Dynamic, 2:Fixed
//0x0F12, 0x0002},   //REG_2TC_CCFG_FrRateQualityType					0:Dynamic, 1:FrameRate, 2:Quality
//0x0F12, 0x2710},   //REG_2TC_CCFG_usMaxFrTimeMsecMult10			2710h = 1000.0ms =  1fps
//0x0F12, 0x2710},   //REG_2TC_CCFG_usMinFrTimeMsecMult10			2710h = 1000.0ms =  1fps
//0x0F12, 0x0000},   //REG_2TC_CCFG_bSmearOutput
//0x0F12, 0x0000},   //REG_2TC_CCFG_sSaturation
//0x0F12, 0x0000},   //REG_2TC_CCFG_sSharpBlur
//0x0F12, 0x0000},   //REG_2TC_CCFG_sColorTemp
//0x0F12, 0x0000},   //REG_2TC_CCFG_uDeviceGammaIndex

{0x002A, 0x03A4},
{0x0F12, 0x2710},   //REG_2TC_CCFG_usMaxFrTimeMsecMult10			2710h = 1000.0ms =  1fps
{0x0F12, 0x2710},   //REG_2TC_CCFG_usMinFrTimeMsecMult10			2710h = 1000.0ms =  1fps

{0x002A, 0x01CC},
{0x0F12, 0x0001},   //REG_TC_IPRM_InitParamsUpdated

{0x002A, 0x0504},
{0x0F12, 0x3415},		//lt_uMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0508},
{0x0F12, 0x681F},		//lt_uMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x050C},
{0x0F12, 0x8227},		//lt_uMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0510},
{0x0F12, 0x1A80},		//lt_uMaxExp4		00061A80h = 400000d =  1000ms
{0x0F12, 0x0006},

{0x002A, 0x0514},
{0x0F12, 0x3415},		//lt_uCapMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0518},
{0x0F12, 0x681F},		//lt_uCapMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x051C},
{0x0F12, 0x8227},		//lt_uCapMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0520},
{0x0F12, 0x1A80},		//lt_uCapMaxExp4		00061A80h = 400000d =  1000ms
{0x0F12, 0x0006},

//ISO 50
//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

//0x002A, 0x0504},
//0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
//0x002A, 0x0508},
//0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
//0x002A, 0x050C},
//0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
//0x002A, 0x0510},
//0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

//0x002A, 0x0514},
//0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
//0x002A, 0x0518},
//0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
//0x002A, 0x051C},
//0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
//0x002A, 0x0520},
//0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0100},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

//Firework Mode Preview
{0x002A, 0x0208},
{0x0F12, 0x0002},		//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0002},		//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},		//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},		//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},		//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},		//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},		//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},		//REG_TC_GP_EnablePreviewChanged 
};

#define S5K5CCGX_CAM_SCENE_FIRE_INDEX (sizeof(S5K5CCGX_CAM_SCENE_FIRE) / sizeof(S5K5CCGX_CAM_SCENE_FIRE[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_TEXT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01D0},
{0x0F12, 0x0000},		//REG_TC_UserBrightness
{0x002A, 0x01D4},
{0x0F12, 0x0000},		//REG_TC_UserSaturation
{0x0F12, 0x0014},		//REG_TC_UserSharpBlur
};

#define S5K5CCGX_CAM_SCENE_TEXT_INDEX (sizeof(S5K5CCGX_CAM_SCENE_TEXT) / sizeof(S5K5CCGX_CAM_SCENE_TEXT[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_SCENE_CANDLE[]=
{
// Use MWB Daylight
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0f12, 0x0000},		//Mon_AAIO_bAWB		AWB OFF

{0x002A, 0x0470},
{0x0f12, 0x0600},		//REG_SF_USER_Rgain
{0x0f12, 0x0001},		//REG_SF_USER_RgainChanged
{0x0f12, 0x0400},		//REG_SF_USER_Ggain
{0x0f12, 0x0001},		//REG_SF_USER_GgainChanged
{0x0f12, 0x0526},		//REG_SF_USER_Bgain
{0x0f12, 0x0001},		//REG_SF_USER_BgainChaged
};

#define S5K5CCGX_SCENE_CAM_CANDLE_INDEX (sizeof(S5K5CCGX_CAM_SCENE_CANDLE) / sizeof(S5K5CCGX_CAM_SCENE_CANDLE[0]))


//==========================================================
//METERING()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_METERING_NORMAL[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1316},		//ae_WeightTbl_16

{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101},
};

#define S5K5CCGX_METERING_NORMAL_INDEX (sizeof(S5K5CCGX_METERING_NORMAL) / sizeof(S5K5CCGX_METERING_NORMAL[0]))

static const s5k5ccgx_short_t S5K5CCGX_METERING_SPOT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1316},		//ae_WeightTbl_16

{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0f01}, 
{0x0F12, 0x010f}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0f01}, 
{0x0F12, 0x010f}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0101}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000}, 
{0x0F12, 0x0000},
};

#define S5K5CCGX_METERING_SPOT_INDEX (sizeof(S5K5CCGX_METERING_SPOT) / sizeof(S5K5CCGX_METERING_SPOT[0]))

static const s5k5ccgx_short_t S5K5CCGX_METERING_CENTER[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},             
{0x002A, 0x1316},		//ae_WeightTbl_16

{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
};

#define S5K5CCGX_METERING_CENTER_INDEX (sizeof(S5K5CCGX_METERING_CENTER) / sizeof(S5K5CCGX_METERING_CENTER[0]))

//==========================================================
//ISO()
//==========================================================
static const s5k5ccgx_short_t S5K5CCGX_CAM_ISO_AUTO[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x067F},		//REG_TC_DBG_AutoAlgEnBits		Auto Algorithm Enable


{0x002A, 0x0504},
{0x0F12, 0x3415},		//lt_uMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0508},
{0x0F12, 0x681F},		//lt_uMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x050C},
{0x0F12, 0x8227},		//lt_uMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0x3415},		//lt_uCapMaxExp1		3415h = 13333d =  33.3325ms
{0x002A, 0x0518},
{0x0F12, 0x681F},		//lt_uCapMaxExp2		681Fh = 26655d =  66.6375ms
{0x002A, 0x051C},
{0x0F12, 0x8227},		//lt_uCapMaxExp3		8227h = 33319d =  83.2975ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0486},
{0x0F12, 0x0000},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x002A, 0x048A},
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

{0x002A, 0x3302},
{0x0F12, 0x0000},		//AFIT by Normalized Brightness Tunning Parameter

};

#define S5K5CCGX_CAM_ISO_AUTO_INDEX (sizeof(S5K5CCGX_CAM_ISO_AUTO) / sizeof(S5K5CCGX_CAM_ISO_AUTO[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_ISO_50[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

{0x002A, 0x0504},
{0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0508},
{0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x050C},
{0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0518},
{0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x051C},
{0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0100},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

};

#define S5K5CCGX_CAM_ISO_50_INDEX (sizeof(S5K5CCGX_CAM_ISO_50) / sizeof(S5K5CCGX_CAM_ISO_50[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_ISO_100[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

{0x002A, 0x0504},
{0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0508},
{0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x050C},
{0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0518},
{0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x051C},
{0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0200},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

};

#define S5K5CCGX_CAM_ISO_100_INDEX (sizeof(S5K5CCGX_CAM_ISO_100) / sizeof(S5K5CCGX_CAM_ISO_100[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_ISO_200[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

{0x002A, 0x0504},
{0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0508},
{0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x050C},
{0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0518},
{0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x051C},
{0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0400},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

};

#define S5K5CCGX_CAM_ISO_200_INDEX (sizeof(S5K5CCGX_CAM_ISO_200) / sizeof(S5K5CCGX_CAM_ISO_200[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAM_ISO_400[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

//0x002A, 0x167C},  
//0x0F12, 0x0000},		//senHal_ExpMinPixels

{0x002A, 0x04A4},
{0x0F12, 0x065F},		//REG_TC_DBG_AutoAlgEnBits		Auto Flicker Off
{0x002A, 0x048C},
{0x0F12, 0x0000},		//REG_SF_USER_FlickerQuant		0:No AFC, 1:50Hz, 2:60Hz
{0x0F12, 0x0001},		//REG_SF_USER_FlickerQuantChanged

{0x002A, 0x0504},
{0x0F12, 0xC350},		//lt_uMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0508},
{0x0F12, 0xC350},		//lt_uMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x050C},
{0x0F12, 0xC350},		//lt_uMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0510},
{0x0F12, 0xC350},		//lt_uMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x0514},
{0x0F12, 0xC350},		//lt_uCapMaxExp1		C350h = 50000d = 125.0000ms
{0x002A, 0x0518},
{0x0F12, 0xC350},		//lt_uCapMaxExp2		C350h = 50000d = 125.0000ms
{0x002A, 0x051C},
{0x0F12, 0xC350},		//lt_uCapMaxExp3		C350h = 50000d = 125.0000ms
{0x002A, 0x0520},
{0x0F12, 0xC350},		//lt_uCapMaxExp4		C350h = 50000d = 125.0000ms

{0x002A, 0x05EA},
{0x0F12, 0x0100},		//lt_bUseSecISODgain

// ISO Gain
{0x002A, 0x0486},
{0x0F12, 0x0003},		//REG_SF_USER_IsoType		0:OFF 3:ISO
{0x0F12, 0x0800},		//REG_SF_USER_IsoVal
{0x0F12, 0x0001},		//REG_SF_USER_IsoChanged

// AFIT by Normalized Brightness Tuning parameter
{0x002A, 0x3302},
{0x0F12, 0x0001},		//AFIT by Normalized Brightness Tunning Parameter

};

#define S5K5CCGX_CAM_ISO_400_INDEX (sizeof(S5K5CCGX_CAM_ISO_400) / sizeof(S5K5CCGX_CAM_ISO_400[0]))

//==========================================================
//PREVIEW()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW[]=
{
//PREVIEW
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x08C0},
{0x0F12, 0x0008},	//AFIT NoiseIndex0 Brightness
{0x002A, 0x0940},
{0x0F12, 0x0080},	//AFIT NoiseIndex0 Y-offset
{0x002A, 0x099E},
{0x0F12, 0x0000},	//AFIT NoiseIndex1 Brightness
{0x002A, 0x0A1E},
{0x0F12, 0x0080},	//AFIT NoiseIndex1 Y-offset


{0x002A, 0x1118},
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[0][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[0][4]
{0x0F12, 0x0014},	//0000	//FFEC	//awbb_GridCorr_R[0][2]
{0x0F12, 0x0014},	//0000	//awbb_GridCorr_R[0][3]
{0x0F12, 0x0014},	//0000	//awbb_GridCorr_R[1][1]
{0x0F12, 0x0050},	//0000	//awbb_GridCorr_R[0][5]
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[1][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[1][4]
{0x0F12, 0x0014},	//0000	//FFEC	//awbb_GridCorr_R[1][2]
{0x0F12, 0x0014},	//awbb_GridCorr_R[1][3]
{0x0F12, 0x0014},	//0000	//awbb_GridCorr_R[2][1]
{0x0F12, 0x0050},	//awbb_GridCorr_R[1][5]
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[2][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[2][4]
{0x0F12, 0x0020},	//0000	//FFEC	//awbb_GridCorr_R[2][2]
{0x0F12, 0x0020},	//awbb_GridCorr_R[2][3]
{0x0F12, 0x0020},	//awbb_GridCorr_R[2][4]
{0x0F12, 0x0050},	//awbb_GridCorr_R[2][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[0][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[0][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[0][2]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[0][3]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[0][4]
{0x0F12, 0xFE48},	//FCE0	//awbb_GridCorr_B[0][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[1][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[1][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[1][2]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[1][3]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[1][4]
{0x0F12, 0xFE48},	//FCE0	//awbb_GridCorr_B[1][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[2][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[2][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[2][2]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[2][3]
{0x0F12, 0xFF9C},	//0000	//awbb_GridCorr_B[2][4]
{0x0F12, 0xFE48},	//FCE0	//awbb_GridCorr_B[2][5]

{0x002A, 0x08E4},
{0x0F12, 0x0000}, //000C
{0x0F12, 0x0000}, //000C
{0x002A, 0x09C2},
{0x0F12, 0x0000}, //0007
{0x002A, 0x0AA0},
{0x0F12, 0x0000}, //0007
{0x002A, 0x0B7E},
{0x0F12, 0x0000}, //0007
{0x002A, 0x0C5C},
{0x0F12, 0x0000}, //0007

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E8},
{0x0F12, 0x0000},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_PREVIEW_INDEX (sizeof(S5K5CCGX_PREVIEW) / sizeof(S5K5CCGX_PREVIEW[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_NIGHT[]=
{
//PREVIEW
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x08C0},
{0x0F12, 0x0008},	//AFIT NoiseIndex0 Brightness
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x002A, 0x0940},
{0x0F12, 0x0080},	//AFIT NoiseIndex0 Y-offset
{0x002A, 0x099E},
{0x0F12, 0x0000},	//AFIT NoiseIndex1 Brightness
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x002A, 0x0A1E},
{0x0F12, 0x0080},	//AFIT NoiseIndex1 Y-offset

{0x002A, 0x08E4},
{0x0F12, 0x0000}, //000C
{0x0F12, 0x0000}, //000C
{0x002A, 0x09C2},
{0x0F12, 0x0000}, //0007
{0x002A, 0x0AA0},
{0x0F12, 0x0000}, //0007
{0x002A, 0x0B7E},
{0x0F12, 0x0000}, //0007
{0x002A, 0x0C5C},
{0x0F12, 0x0000}, //0007

{0x002A, 0x0208},
{0x0F12, 0x0002},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0002},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E8},
{0x0F12, 0x0000},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged

};

#define S5K5CCGX_PREVIEW_NIGHT_INDEX (sizeof(S5K5CCGX_PREVIEW_NIGHT) / sizeof(S5K5CCGX_PREVIEW_NIGHT[0]))

//==========================================================
//SNAPSHOT()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_HIGH_SNAPSHOT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x08E4},
{0x0F12, 0x000C},
{0x0F12, 0x000C},
{0x002A, 0x09C2},
{0x0F12, 0x0007},
{0x002A, 0x0AA0},
{0x0F12, 0x0007},
{0x002A, 0x0B7E},
{0x0F12, 0x0007},
{0x002A, 0x0C5C},
{0x0F12, 0x0000},

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
{0x002A, 0x01E8},
{0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ���� 

{0xffff, 0x00A0}, //160ms// 
};

#define S5K5CCGX_HIGH_SNAPSHOT_INDEX (sizeof(S5K5CCGX_HIGH_SNAPSHOT) / sizeof(S5K5CCGX_HIGH_SNAPSHOT[0]))

static const s5k5ccgx_short_t S5K5CCGX_NORMAL_SNAPSHOT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x08E4},
{0x0F12, 0x000C},
{0x0F12, 0x000C},
{0x002A, 0x09C2},
{0x0F12, 0x0007},
{0x002A, 0x0AA0},
{0x0F12, 0x0007},
{0x002A, 0x0B7E},
{0x0F12, 0x0007},
{0x002A, 0x0C5C},
{0x0F12, 0x0000},

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
{0x002A, 0x01E8},
{0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����

{0xffff, 0x00A0}, //160ms// 
};

#define S5K5CCGX_NORMAL_SNAPSHOT_INDEX (sizeof(S5K5CCGX_NORMAL_SNAPSHOT) / sizeof(S5K5CCGX_NORMAL_SNAPSHOT[0]))

static const s5k5ccgx_short_t S5K5CCGX_LOWLIGHT_SNAPSHOT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x08C0},
{0x0F12, 0x0010},	//AFIT NoiseIndex0 Brightness
{0x002A, 0x0940},
{0x0F12, 0x0B80},	//AFIT NoiseIndex0 Y-offset
{0x002A, 0x099E},
{0x0F12, 0x0010},	//AFIT NoiseIndex1 Brightness
{0x002A, 0x0A1E},
{0x0F12, 0x0B80},	//AFIT NoiseIndex1 Y-offset


{0x002A, 0x1118},
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[0][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[0][4]
{0x0F12, 0x0014},	//0000	//FFEC	//awbb_GridCorr_R[0][2]
{0x0F12, 0x0096},	//0000	//awbb_GridCorr_R[0][3]
{0x0F12, 0x0096},	//0000	//awbb_GridCorr_R[1][1]
{0x0F12, 0x0050},	//0000	//awbb_GridCorr_R[0][5]
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[1][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[1][4]
{0x0F12, 0x0014},	//0000	//FFEC	//awbb_GridCorr_R[1][2]
{0x0F12, 0x0096},	//awbb_GridCorr_R[1][3]
{0x0F12, 0x0096},	//0000	//awbb_GridCorr_R[2][1]
{0x0F12, 0x0050},	//awbb_GridCorr_R[1][5]
{0x0F12, 0x0000},	//FFD8	//FFEC	//awbb_GridCorr_R[2][0]
{0x0F12, 0xFFEC},	//0000	//awbb_GridCorr_R[2][4]
{0x0F12, 0x0020},	//0000	//FFEC	//awbb_GridCorr_R[2][2]
{0x0F12, 0x0096},	//awbb_GridCorr_R[2][3]
{0x0F12, 0x0096},	//awbb_GridCorr_R[2][4]
{0x0F12, 0x0050},	//awbb_GridCorr_R[2][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[0][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[0][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[0][2]
{0x0F12, 0xFED4},	//0000	//awbb_GridCorr_B[0][3]
{0x0F12, 0xFED4},	//0000	//awbb_GridCorr_B[0][4]
{0x0F12, 0xFE48},	//FCE0	//awbb_GridCorr_B[0][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[1][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[1][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[1][2]
{0x0F12, 0xFED4},	//0000	//awbb_GridCorr_B[1][3]
{0x0F12, 0xFED4},	//0000	//awbb_GridCorr_B[1][4]
{0x0F12, 0xFE48},	//FCE0	//awbb_GridCorr_B[1][5]
{0x0F12, 0x0000},	//awbb_GridCorr_B[2][0]
{0x0F12, 0x0000},	//awbb_GridCorr_B[2][1]
{0x0F12, 0xFFEC},	//awbb_GridCorr_B[2][2]
{0x0F12, 0xFED4},	//0000	//awbb_GridCorr_B[2][3]
{0x0F12, 0xFED4},	//0000	//awbb_GridCorr_B[2][4]
{0x0F12, 0xFE48},	//FCE0	//awbb_GridCorr_B[2][5]

{0xffff, 0x012C}, //300ms// 

{0x002A, 0x08E4},
{0x0F12, 0x000C},
{0x0F12, 0x000C},
{0x002A, 0x09C2},
{0x0F12, 0x0007},
{0x002A, 0x0AA0},
{0x0F12, 0x0007},
{0x002A, 0x0B7E},
{0x0F12, 0x0007},
{0x002A, 0x0C5C},
{0x0F12, 0x0000},

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
{0x002A, 0x01E8},
{0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����

{0xffff, 0x012C}, //300ms// 
};

#define S5K5CCGX_LOWLIGHT_SNAPSHOT_INDEX (sizeof(S5K5CCGX_LOWLIGHT_SNAPSHOT) / sizeof(S5K5CCGX_LOWLIGHT_SNAPSHOT[0]))

static const s5k5ccgx_short_t S5K5CCGX_NIGHT_SNAPSHOT[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x08C0},
{0x0F12, 0x0010},	//AFIT NoiseIndex0 Brightness
{0x0F12, 0x0000},
{0x0F12, 0x0020},
{0x002A, 0x0940},
{0x0F12, 0x1680},	//AFIT NoiseIndex0 Y-offset
{0x002A, 0x099E},
{0x0F12, 0x0010},	//AFIT NoiseIndex1 Brightness
{0x0F12, 0x0000},
{0x0F12, 0x0020},
{0x002A, 0x0A1E},
{0x0F12, 0x1680},	//AFIT NoiseIndex1 Y-offset

{0x002A, 0x08E4},
{0x0F12, 0x000C},
{0x0F12, 0x000C},
{0x002A, 0x09C2},
{0x0F12, 0x0007},
{0x002A, 0x0AA0},
{0x0F12, 0x0007},
{0x002A, 0x0B7E},
{0x0F12, 0x0007},
{0x002A, 0x0C5C},
{0x0F12, 0x0000},

{0x002A, 0x0208},
{0x0F12, 0x0002},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0002},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
{0x002A, 0x01E8},
{0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
{0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ���� 

{0xffff, 0x012C}, //300ms//   
};

#define S5K5CCGX_NIGHT_SNAPSHOT_INDEX (sizeof(S5K5CCGX_NIGHT_SNAPSHOT) / sizeof(S5K5CCGX_NIGHT_SNAPSHOT[0]))



//==========================================================
//FPS()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_7_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x0535},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x0535},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_7_FPS_INDEX (sizeof(S5K5CCGX_7_FPS) / sizeof(S5K5CCGX_7_FPS[0]))

static const s5k5ccgx_short_t S5K5CCGX_10_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x03E8},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x03E8},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_10_FPS_INDEX (sizeof(S5K5CCGX_10_FPS) / sizeof(S5K5CCGX_10_FPS[0]))

static const s5k5ccgx_short_t S5K5CCGX_12_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x0341},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x0341},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged

};

#define S5K5CCGX_12_FPS_INDEX (sizeof(S5K5CCGX_12_FPS) / sizeof(S5K5CCGX_12_FPS[0]))

static const s5k5ccgx_short_t S5K5CCGX_15_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x029A},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x029A},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_15_FPS_INDEX (sizeof(S5K5CCGX_15_FPS) / sizeof(S5K5CCGX_15_FPS[0]))

static const s5k5ccgx_short_t S5K5CCGX_30_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x014E},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x014E},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_30_FPS_INDEX (sizeof(S5K5CCGX_30_FPS) / sizeof(S5K5CCGX_30_FPS[0]))

static const s5k5ccgx_short_t S5K5CCGX_AUTO15_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x0535},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x029A},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_AUTO15_FPS_INDEX (sizeof(S5K5CCGX_AUTO15_FPS) / sizeof(S5K5CCGX_AUTO15_FPS[0]))

static const s5k5ccgx_short_t S5K5CCGX_AUTO30_FPS[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x025A},
{0x0F12, 0x03E8},	//REG_0TC_PCFG_usMaxFrTimeMsecMult10	//max frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS
{0x0F12, 0x014E},	//REG_0TC_PCFG_usMinFrTimeMsecMult10	//min frame time : 30fps 014D 15fps 029a; a6a - 3.75 fps; 0535 - 7.5FPS

{0x002A, 0x0208},
{0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
{0x002A, 0x0210},
{0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
{0x002A, 0x020C},
{0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
{0x002A, 0x01F4},
{0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
{0x002A, 0x020A},
{0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
{0x002A, 0x0212},
{0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
{0x002A, 0x01E4},
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
{0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_AUTO30_FPS_INDEX (sizeof(S5K5CCGX_AUTO30_FPS) / sizeof(S5K5CCGX_AUTO30_FPS[0]))


//==========================================================
//AE_LOCK(2)
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_AE_LOCK[]=
{
//AE lock
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A5A},
{0x0F12, 0x0000},
};

#define S5K5CCGX_AE_LOCK_INDEX (sizeof(S5K5CCGX_AE_LOCK) / sizeof(S5K5CCGX_AE_LOCK[0]))

static const s5k5ccgx_short_t S5K5CCGX_AE_UNLOCK[]=
{
//AE lock
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A5A},
{0x0F12, 0x0001},
};

#define S5K5CCGX_AE_UNLOCK_INDEX (sizeof(S5K5CCGX_AE_UNLOCK) / sizeof(S5K5CCGX_AE_UNLOCK[0]))

//==========================================================
//AWE_LOCK(2)
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_AWE_LOCK[]=
{
//AWB lock
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0000}, //M AAIO bAWB	
};

#define S5K5CCGX_AWE_LOCK_INDEX (sizeof(S5K5CCGX_AWE_LOCK) / sizeof(S5K5CCGX_AWE_LOCK[0]))

static const s5k5ccgx_short_t S5K5CCGX_AWE_UNLOCK[]=
{
//AWB unlock
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x2A62},
{0x0F12, 0x0001}, //M AAIO bAWB	
};

#define S5K5CCGX_AWE_UNLOCK_INDEX (sizeof(S5K5CCGX_AWE_UNLOCK) / sizeof(S5K5CCGX_AWE_UNLOCK[0]))

//==========================================================
//AF()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_AF_DO[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0224},
{0x0F12, 0x0005},   // REG_TC_AF_AfCmd = 5, single AF

// 2frame �ڿ�  Read [26FE]  usStatus
};

#define S5K5CCGX_AF_DO_INDEX (sizeof(S5K5CCGX_AF_DO) / sizeof(S5K5CCGX_AF_DO[0]))

static const s5k5ccgx_short_t S5K5CCGX_AF_NORMAL_ON[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0224},
{0x0F12, 0x0004},   // write [7000 0224, REG_TC_AF_AfCmd] = 0004 , manual AF. 

{0x002A, 0x0226},   // write [7000 0226, REG_TC_AF_AfCmdParam]
{0x0F12, 0x0000},   // write lens position from 0000 to 00FF. 0000 means infinity and 00FF means macro.

//0xFFFF, 0x0096},   // 150ms Delay

//#af_search_usSingleAfFlags, 1040 : macro mode on,  2nd search off, 1042 : macro mode on, 2nd search(fine search) on
//                            1000 : macro mode off, 2nd search off, 1002 : macro mode off, 2nd search on
{0x002A, 0x1494},
{0x0F12, 0x1000},
};

#define S5K5CCGX_AF_NORMAL_ON_INDEX (sizeof(S5K5CCGX_AF_NORMAL_ON) / sizeof(S5K5CCGX_AF_NORMAL_ON[0]))

static const s5k5ccgx_short_t S5K5CCGX_AF_MACRO_ON[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0224},
{0x0F12, 0x0004},   // write [7000 0224, REG_TC_AF_AfCmd] = 0004 , manual AF. 

{0x002A, 0x0226},   // write [7000 0226, REG_TC_AF_AfCmdParam]
{0x0F12, 0x00D0},   // write lens position from 0000 to 00FF. 0000 means infinity and 00FF means macro.

//0xFFFF, 0x0096},   // 150ms Delay

//#af_search_usSingleAfFlags, 1040 : macro mode on,  2nd search off, 1042 : macro mode on, 2nd search(fine search) on
//                            1000 : macro mode off, 2nd search off, 1002 : macro mode off, 2nd search on
{0x002A, 0x1494},
{0x0F12, 0x1040},

// when user use lens position 16(10h) -> lens position 0(00h)
// MSB 10 b means user uses #af_pos_usTable_16_ as start position.
// LSB 00 b means user uses #af_pos_usTable_0_ as end position
// refer to 5.3 macro mode setting. 
// "#af_pos_usMacroStartEnd" is only used in macro AF condition.
//  (normal AF doesn't use "#af_pos_usMacroStartEnd")
{0x002A, 0x1426},
{0x0F12, 0x1000},   //#af_pos_usMacroStartEnd
};

#define S5K5CCGX_AF_MACRO_ON_INDEX (sizeof(S5K5CCGX_AF_MACRO_ON) / sizeof(S5K5CCGX_AF_MACRO_ON[0]))

static const s5k5ccgx_short_t S5K5CCGX_AF_OFF[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0224},
{0x0F12, 0x0004},   // REG_TC_AF_AfCmd = 4, Abort AF
};

#define S5K5CCGX_AF_OFF_INDEX (sizeof(S5K5CCGX_AF_OFF) / sizeof(S5K5CCGX_AF_OFF[0]))

//==========================================================
//DTP
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_DTP_OFF[]=
{
{0xFCFC, 0xD000},
{0x0028, 0xD000},
{0x002A, 0xB054},
{0x0F12, 0x0000},
};

#define S5K5CCGX_DTP_OFF_INDEX (sizeof(S5K5CCGX_DTP_OFF) / sizeof(S5K5CCGX_DTP_OFF[0]))

static const s5k5ccgx_short_t S5K5CCGX_DTP_ON[]=
{
{0xFCFC, 0xD000},
{0x0028, 0xD000},
{0x002A, 0xB054},
{0x0F12, 0x0001},
};

#define S5K5CCGX_DTP_ON_INDEX (sizeof(S5K5CCGX_DTP_ON) / sizeof(S5K5CCGX_DTP_ON[0]))

//==========================================================
//preivew size()
//==========================================================
static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_176[]= //176 x 144
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive	//Thumbnail Enable
{0x0F12, 0x0190},		//REG_TC_THUMB_Thumb_uWidth		//Thumbnail Width 400
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight	//Thumbnail Height 240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x00B0},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x0090},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x00B0},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x0090},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0320},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0320},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//PREVIEW
//0x002A, 0x0208},
//0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
//0x002A, 0x020C},
//0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
//0x002A, 0x020A},
//0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
//0x002A, 0x01E4},
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_PREVIEW_SIZE_176_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_176) / sizeof(S5K5CCGX_PREVIEW_SIZE_176[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_144[]= //144 x 176 
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive	//Thumbnail Enable
{0x0F12, 0x0190},		//REG_TC_THUMB_Thumb_uWidth		//Thumbnail Width 400
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight	//Thumbnail Height 240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x0090},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x00B0},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x0090},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x00B0},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0320},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0320},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//PREVIEW
//0x002A, 0x0208},
//0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
//0x002A, 0x020C},
//0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
//0x002A, 0x020A},
//0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
//0x002A, 0x01E4},
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_PREVIEW_SIZE_144_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_144) / sizeof(S5K5CCGX_PREVIEW_SIZE_144[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_240[]= //  240x320
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232 => 1536
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2 => 0
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232 => 1536
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2 => 0 
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive			Thumbnail Enable
{0x0F12, 0x00A0},		//REG_TC_THUMB_Thumb_uWidth				Thumbnail Width //160
{0x0F12, 0x0078},		//REG_TC_THUMB_Thumb_uHeight			Thumbnail Height //120
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x00F0},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x0140},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x00F0},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x0140},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x00F0},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0140},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x00F0},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0140},   //REG_2TC_CCFG_usHeight
};

#define S5K5CCGX_PREVIEW_SIZE_240_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_240) / sizeof(S5K5CCGX_PREVIEW_SIZE_240[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_320[]= //  320x240
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232 => 1536
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2 => 0
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232 => 1536
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2 => 0 
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive			Thumbnail Enable
{0x0F12, 0x00A0},		//REG_TC_THUMB_Thumb_uWidth				Thumbnail Width //160
{0x0F12, 0x0078},		//REG_TC_THUMB_Thumb_uHeight			Thumbnail Height //120
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x0140},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x00F0},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x0140},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x00F0},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0140},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x00F0},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0140},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x00F0},   //REG_2TC_CCFG_usHeight
};

#define S5K5CCGX_PREVIEW_SIZE_320_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_320) / sizeof(S5K5CCGX_PREVIEW_SIZE_320[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_720[]= //720X480
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0558},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1368
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0054},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 84=(1536-1368)/2
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0558},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1368
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0054},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 84=(1536-1368)/2
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive	//Thumbnail Enable
{0x0F12, 0x0168},		//REG_TC_THUMB_Thumb_uWidth		//Thumbnail Width 360
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight	//Thumbnail Height 240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x02D0},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x01E0},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x02D0},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x01E0},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0320},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0320},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//PREVIEW
//0x002A, 0x0208},
//0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
//0x002A, 0x020C},
//0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
//0x002A, 0x020A},
//0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
//0x002A, 0x01E4},
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged 
};

#define S5K5CCGX_PREVIEW_SIZE_720_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_720) / sizeof(S5K5CCGX_PREVIEW_SIZE_720[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_W480[]= //800X480
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive	//Thumbnail Enable
{0x0F12, 0x0190},		//REG_TC_THUMB_Thumb_uWidth		//Thumbnail Width 400
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight	//Thumbnail Height 240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV
           
//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x0320},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x01E0},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x0320},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x01E0},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0320},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x01E0},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0320},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x01E0},   //REG_2TC_CCFG_usHeight
};

#define S5K5CCGX_PREVIEW_SIZE_W480_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_W480) / sizeof(S5K5CCGX_PREVIEW_SIZE_W480[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_800[]= //800X600
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232 => 1536
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2 => 0
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x0600},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232 => 1536
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0000},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2 => 0 
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive	//Thumbnail Enable
{0x0F12, 0x0140},		//REG_TC_THUMB_Thumb_uWidth		//Thumbnail Width 400 -> 320
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight	//Thumbnail Height 240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x0320},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x0320},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0320},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0320},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//PREVIEW
//0x002A, 0x0208},
//0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
//0x002A, 0x020C},
//0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
//0x002A, 0x020A},
//0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
//0x002A, 0x01E4},
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_PREVIEW_SIZE_800_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_800) / sizeof(S5K5CCGX_PREVIEW_SIZE_800[0]))

static const s5k5ccgx_short_t S5K5CCGX_PREVIEW_SIZE_1024[]= //1024X600
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x01F6},
{0x0F12, 0x0800},	//REG_TC_GP_PrevReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_PrevReqInputHeight	//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_PrevInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_PrevInputHeightOfs	//Sensor VOffset 152=(1536-1232)/2
{0x0F12, 0x0800},	//REG_TC_GP_CapReqInputWidth		//Sensor Crop Width	2048
{0x0F12, 0x04D0},	//REG_TC_GP_CapReqInputHeight		//Sensor Crop Height 1232
{0x0F12, 0x0000},	//REG_TC_GP_CapInputWidthOfs		//Sensor HOffset 0
{0x0F12, 0x0098},	//REG_TC_GP_CapInputHeightOfs		//Sensor VOffset 152=(1536-1232)/2
{0x002A, 0x0216},
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInPre
{0x0F12, 0x0001},	//REG_TC_GP_bUseReqInputInCap

{0x002A, 0x0428},
{0x0F12, 0x0001},		//REG_TC_THUMB_Thumb_bActive	//Thumbnail Enable
{0x0F12, 0x0190},		//REG_TC_THUMB_Thumb_uWidth		//Thumbnail Width 400
{0x0F12, 0x00F0},		//REG_TC_THUMB_Thumb_uHeight	//Thumbnail Height 240
{0x0F12, 0x0005},		//REG_TC_THUMB_Thumb_Format		//Thumbnail Output Format 5:YUV


//WRITE #REG_TC_PZOOM_ZoomInputWidth		0800     //ISP	Input Width	2048                  
//WRITE #REG_TC_PZOOM_ZoomInputHeight		0480	//ISP	Input Height	1152                  
//WRITE #REG_TC_PZOOM_ZoomInputWidthOfs	0000     //ISP	Input HOffset	0                     
//WRITE #REG_TC_PZOOM_ZoomInputHeightOfs	0000     //ISP	Input VOffset	0                     

//Preview Size
{0x002A, 0x023E},
{0x0F12, 0x0400},	//REG_0TC_PCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_PCFG_usHeight

{0x002A, 0x029E},
{0x0F12, 0x0400},		//REG_2TC_PCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_PCFG_usHeight

//Capture Size
{0x002A, 0x0330},
{0x0F12, 0x0400},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0400},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
//PREVIEW
//0x002A, 0x0208},
//0x0F12, 0x0000},	//REG_TC_GP_ActivePrevConfig
//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig
//0x002A, 0x020C},
//0x0F12, 0x0001},	//REG_TC_GP_PrevOpenAfterChange
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync
//0x002A, 0x020A},
//0x0F12, 0x0001},	//REG_TC_GP_PrevConfigChanged
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged
//0x002A, 0x01E4},
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreview
//0x0F12, 0x0001},	//REG_TC_GP_EnablePreviewChanged
};

#define S5K5CCGX_PREVIEW_SIZE_1024_INDEX (sizeof(S5K5CCGX_PREVIEW_SIZE_1024) / sizeof(S5K5CCGX_PREVIEW_SIZE_1024[0]))

//==========================================================
//Capture size()
//==========================================================

static const s5k5ccgx_short_t S5K5CCGX_CAPTURE_SIZE_800[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0320},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0320},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
//0x002A, 0x01E8},
//0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
//0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����
};

#define S5K5CCGX_CAPTURE_SIZE_800_INDEX (sizeof(S5K5CCGX_CAPTURE_SIZE_800) / sizeof(S5K5CCGX_CAPTURE_SIZE_800[0]))


static const s5k5ccgx_short_t S5K5CCGX_CAPTURE_SIZE_1600[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0640},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x04B0},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0640},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x04B0},   //REG_2TC_CCFG_usHeight

//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
//0x002A, 0x01E8},
//0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
//0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����
 
};

#define S5K5CCGX_CAPTURE_SIZE_1600_INDEX (sizeof(S5K5CCGX_CAPTURE_SIZE_1600) / sizeof(S5K5CCGX_CAPTURE_SIZE_1600[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAPTURE_SIZE_2048[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0800},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0600},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0800},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0600},   //REG_2TC_CCFG_usHeight

//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
//0x002A, 0x01E8},
//0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
//0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����
};

#define S5K5CCGX_CAPTURE_SIZE_2048_INDEX (sizeof(S5K5CCGX_CAPTURE_SIZE_2048) / sizeof(S5K5CCGX_CAPTURE_SIZE_2048[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAPTURE_SIZE_1024W[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0400},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x0258},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0400},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x0258},   //REG_2TC_CCFG_usHeight

//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
//0x002A, 0x01E8},
//0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
//0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����
};

#define S5K5CCGX_CAPTURE_SIZE_1024W_INDEX (sizeof(S5K5CCGX_CAPTURE_SIZE_1024W) / sizeof(S5K5CCGX_CAPTURE_SIZE_1024W[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAPTURE_SIZE_1600W[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0640},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x03C0},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0640},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x03C0},   //REG_2TC_CCFG_usHeight

//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
//0x002A, 0x01E8},
//0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
//0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����
};

#define S5K5CCGX_CAPTURE_SIZE_1600W_INDEX (sizeof(S5K5CCGX_CAPTURE_SIZE_1600W) / sizeof(S5K5CCGX_CAPTURE_SIZE_1600W[0]))

static const s5k5ccgx_short_t S5K5CCGX_CAPTURE_SIZE_2048W[]=
{
{0xFCFC, 0xD000},
{0x0028, 0x7000},

{0x002A, 0x0330},
{0x0F12, 0x0800},	//REG_0TC_CCFG_usWidth
{0x0F12, 0x04D0},	//REG_0TC_CCFG_usHeight

{0x002A, 0x0388},
{0x0F12, 0x0800},   //REG_2TC_CCFG_usWidth
{0x0F12, 0x04D0},   //REG_2TC_CCFG_usHeight

//0x002A, 0x0210},
//0x0F12, 0x0000},	//REG_TC_GP_ActiveCapConfig 0000 : capture configuration �����Ͽ�  �Է��ϸ� ��.
//0x002A, 0x01F4},
//0x0F12, 0x0001},	//REG_TC_GP_NewConfigSync 0001 : update configuration
//0x002A, 0x0212},
//0x0F12, 0x0001},	//REG_TC_GP_CapConfigChanged 0001
//0x002A, 0x01E8},
//0x0F12, 0x0001},	//REG_TC_GP_EnableCapture 0001 : capture ����
//0x0F12, 0x0001},	//REG_TC_GP_EnableCaptureChanged 0001 : ����

};

#define S5K5CCGX_CAPTURE_SIZE_2048W_INDEX (sizeof(S5K5CCGX_CAPTURE_SIZE_2048W) / sizeof(S5K5CCGX_CAPTURE_SIZE_2048W[0]))

// VE_GROUP [[
static const s5k5ccgx_short_t S5K5CCGX_VHFLIP_ON[]=
{
	{0xFCFC, 0xD000},
	{0x0028, 0x7000},

	{0x002A, 0x025E},
	
	{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
	{0x0F12, 0x0003},	//REG_0TC_PCFG_uPrevMirror
	{0x0F12, 0x0003},	//REG_0TC_PCFG_uCaptureMirror
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation

	{0x002A, 0x02BE},	
	{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
	{0x0F12, 0x0003},	//REG_0TC_PCFG_uPrevMirror
	{0x0F12, 0x0003},	//REG_0TC_PCFG_uCaptureMirror
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation	
	
};

#define S5K5CCGX_VHFLIP_ON_INDEX (sizeof(S5K5CCGX_VHFLIP_ON) / sizeof(S5K5CCGX_VHFLIP_ON[0]))

static const s5k5ccgx_short_t S5K5CCGX_VHFLIP_OFF[]=
{
	{0xFCFC, 0xD000},
	{0x0028, 0x7000},

	{0x002A, 0x025E},

	{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uPrevMirror
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uCaptureMirror
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation
	
	{0x002A, 0x02BE},	
	{0x0F12, 0x0000},	//REG_0TC_PCFG_bSmearOutput
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSaturation
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sSharpBlur
	{0x0F12, 0x0000},	//REG_0TC_PCFG_sColorTemp
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uDeviceGammaIndex
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uPrevMirror
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uCaptureMirror
	{0x0F12, 0x0000},	//REG_0TC_PCFG_uRotation
		
};

#define S5K5CCGX_VHFLIP_OFF_INDEX (sizeof(S5K5CCGX_VHFLIP_OFF) / sizeof(S5K5CCGX_VHFLIP_OFF[0]))
static const s5k5ccgx_short_t S5K5CCGX_ZOOM_00[] ={
// Preview 640x480?? ???? Zoom 1??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0100},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_00_INDEX (sizeof(S5K5CCGX_ZOOM_00) / sizeof(S5K5CCGX_ZOOM_00[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_01[] ={
// Preview 640x480?? ???? Zoom 1.2??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0133}, // 1.2* 256
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_01_INDEX (sizeof(S5K5CCGX_ZOOM_01) / sizeof(S5K5CCGX_ZOOM_01[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_02[] ={
// Preview 640x480?? ???? Zoom 1.6??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0199},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_02_INDEX (sizeof(S5K5CCGX_ZOOM_02) / sizeof(S5K5CCGX_ZOOM_02[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_03[] ={
// Preview 640x480?? ???? Zoom 2??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0200},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_03_INDEX (sizeof(S5K5CCGX_ZOOM_03) / sizeof(S5K5CCGX_ZOOM_03[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_04[] ={
// Preview 640x480?? ???? Zoom 2.4??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0266},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_04_INDEX (sizeof(S5K5CCGX_ZOOM_04) / sizeof(S5K5CCGX_ZOOM_04[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_05[] ={
// Preview 640x480?? ???? Zoom 2.8??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x02CC},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_05_INDEX (sizeof(S5K5CCGX_ZOOM_05) / sizeof(S5K5CCGX_ZOOM_05[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_06[] ={
// Preview 640x480?? ???? Zoom 3.0??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0300},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_06_INDEX (sizeof(S5K5CCGX_ZOOM_06) / sizeof(S5K5CCGX_ZOOM_06[0]))

static const s5k5ccgx_short_t S5K5CCGX_ZOOM_07[] ={
// Preview 640x480?? ???? Zoom 3.2??
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0444},
{0x0F12, 0x0333},
{0x002A, 0x0436},
{0x0F12, 0x0002},
};
#define S5K5CCGX_ZOOM_07_INDEX (sizeof(S5K5CCGX_ZOOM_07) / sizeof(S5K5CCGX_ZOOM_07[0]))

// Flash concept
static const s5k5ccgx_short_t S5K5CCGX_FLASH_WAIT[] ={
{0xFFFF, 0x0064}, // 100ms
};
#define S5K5CCGX_FLASH_WAIT_INDEX (sizeof(S5K5CCGX_FLASH_WAIT) / sizeof(S5K5CCGX_FLASH_WAIT[0]))

static const s5k5ccgx_short_t S5K5CCGX_PRE_FLASH_START_EVT1[] ={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x3F82},
{0x0F12, 0x0001},
};
#define S5K5CCGX_PRE_FLASH_START_EVT1_INDEX (sizeof(S5K5CCGX_PRE_FLASH_START_EVT1) / sizeof(S5K5CCGX_PRE_FLASH_START_EVT1[0]))

static const s5k5ccgx_short_t S5K5CCGX_PRE_FLASH_END_EVT1[] ={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x3F84},
{0x0F12, 0x0001},
};
#define S5K5CCGX_PRE_FLASH_END_EVT1_INDEX (sizeof(S5K5CCGX_PRE_FLASH_END_EVT1) / sizeof(S5K5CCGX_PRE_FLASH_END_EVT1[0]))

static const s5k5ccgx_short_t S5K5CCGX_FLASH_START_EVT1[] ={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x3F80},
{0x0F12, 0x0001},
};
#define S5K5CCGX_FLASH_START_EVT1_INDEX (sizeof(S5K5CCGX_FLASH_START_EVT1) / sizeof(S5K5CCGX_FLASH_START_EVT1[0]))

static const s5k5ccgx_short_t S5K5CCGX_AE_SPEEDUP[] ={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0500},
{0x0F12, 0x0000},
};
#define S5K5CCGX_AE_SPEEDUP_INDEX (sizeof(S5K5CCGX_AE_SPEEDUP) / sizeof(S5K5CCGX_AE_SPEEDUP[0]))

static const s5k5ccgx_short_t S5K5CCGX_AE_SPEEDNORMAL[] ={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x0500},
{0x0F12, 0x0002},
};
#define S5K5CCGX_AE_SPEEDNORMAL_INDEX (sizeof(S5K5CCGX_AE_SPEEDNORMAL) / sizeof(S5K5CCGX_AE_SPEEDNORMAL[0]))

static const s5k5ccgx_short_t S5K5CCGX_AE_WEIGHT[] ={
{0xFCFC, 0xD000},
{0x0028, 0x7000},
{0x002A, 0x1316},

{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0000},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},

{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},

{0x0F12, 0x0101},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0101},
{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},

{0x0F12, 0x0201},
{0x0F12, 0x0202},
{0x0F12, 0x0202},
{0x0F12, 0x0102},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
{0x0F12, 0x0101},
};
#define S5K5CCGX_AE_WEIGHT_INDEX (sizeof(S5K5CCGX_AE_WEIGHT) / sizeof(S5K5CCGX_AE_WEIGHT[0]))

static const s5k5ccgx_short_t S5K5CCGX_ABNORMAL_CAPTURE[] ={
	{0xFCFC, 0xD000},
	{0x0028, 0x7000},

	{0x002A, 0x01F6},
	{0x0F12, 0x0800}, //REG_TC_GP_PrevReqInputWidth  //Sensor Crop Width 2048
	{0x0F12, 0x0600}, //REG_TC_GP_PrevReqInputHeight //Sensor Crop Height 1232 => 1536
	{0x0F12, 0x0000}, //REG_TC_GP_PrevInputWidthOfs  //Sensor HOffset 0
	{0x0F12, 0x0000}, //REG_TC_GP_PrevInputHeightOfs //Sensor VOffset 152=(1536-1232)/2 => 0
	{0x0F12, 0x0800}, //REG_TC_GP_CapReqInputWidth  //Sensor Crop Width 2048
	{0x0F12, 0x0600}, //REG_TC_GP_CapReqInputHeight  //Sensor Crop Height 1232 => 1536
	{0x0F12, 0x0000}, //REG_TC_GP_CapInputWidthOfs  //Sensor HOffset 0
	{0x0F12, 0x0000}, //REG_TC_GP_CapInputHeightOfs  //Sensor VOffset 152=(1536-1232)/2 => 0 
	{0x002A, 0x0216},
	{0x0F12, 0x0001}, //REG_TC_GP_bUseReqInputInPre
	{0x0F12, 0x0001}, //REG_TC_GP_bUseReqInputInCap

	{0x002A, 0x0428},
	{0x0F12, 0x0001},  //REG_TC_THUMB_Thumb_bActive   Thumbnail Enable
	{0x0F12, 0x0140},  //REG_TC_THUMB_Thumb_uWidth    Thumbnail Width //320
	{0x0F12, 0x00F0},  //REG_TC_THUMB_Thumb_uHeight   Thumbnail Height //240
	{0x0F12, 0x0005},  //REG_TC_THUMB_Thumb_Format    Thumbnail Output Format 5:YUV

	{0x002A, 0x0330},
	{0x0F12, 0x0800}, //REG_0TC_CCFG_usWidth
	{0x0F12, 0x0600}, //REG_0TC_CCFG_usHeight

	{0x002A, 0x0388},
	{0x0F12, 0x0800},   //REG_2TC_CCFG_usWidth
	{0x0F12, 0x0600},   //REG_2TC_CCFG_usHeight
};
#define S5K5CCGX_ABNORMAL_CAPTURE_INDEX (sizeof(S5K5CCGX_ABNORMAL_CAPTURE) / sizeof(S5K5CCGX_ABNORMAL_CAPTURE[0]))

// VE_GROUP ]]


/**************  enum for read commands ****************/
enum ReadCommad
{
  RCommandMIN = 0,
  RCommandFWVersionInfo,
  RCommandAFLibraryVersionInfo,
  RCommandBatchReflectionStatus,
  RCommandFWEndStatusChk,
  RCommandFWLoaderStatusChk,
  RCommandFWUpdateStatusChk,
  RCommandPreviewStatusCheck,
  RCommandCaptureStatusCheck,
  RCommandErrorStatus,
  RCommandDataOutputSetting,
  RCommandDataOutputRequest,
  RCommandDataTransmissionCheck,
  RCommandJPEGCompressionStatus,
  RCommandLumpStatus,
  RCommandAFIdleStatus,
  RCommandAFFinishStatus,

  RCommandZoomIdleStatus,
  RCommandZoomReleaseStatus,
  RCommandZoomFinishStatus,

  RCommandAFZoomTuningStatus,
  RCommandAFZoomTuningFinishStatus,

  RCommandDZoomFinishStatus,
  RCommandFlashStatusCheck,

  RCommandISStatusCheck,

  RCommandFlashCheck,
  RCommandSmileCheck,
  RCommandBlinkCheck,
  RCommandBlinkLocCheck,
  RCommandLatLongCheck,
  RCommandAltitudeCheck,
  RCommandFastAFCheck,
  RCommandMAX
};

struct s5k5ccgx_preview_size {
  unsigned int width;
  unsigned int height;
};

const static struct s5k5ccgx_preview_size s5k5ccgx_preview_sizes[] = {
  {1280,720},  
  {1024,600},    	
  {800,600},    
  {800,480},    
  {720,480},   
  {640,480},    
  {400,240},  
  {352,288},  
  {320,240},
  {240,320},
  {200,120},   
  {176,144},  
  {144,176},  // for vt
  {160,120},    
};

struct s5k5ccgx_capture_size {
  unsigned int width;
  unsigned int height;
};

/* Image sizes */
const static struct s5k5ccgx_capture_size s5k5ccgx_image_sizes[] = {
  {3264,2448},
  {2560,1920},     
  {2560,1536},
  {2560,1440},
  {2048,1536}, 
  {2048,1232},
  {1920,1080},
  {1600,1200},
  {1600,960},
  {1280,960},
  {1280,768},   
  {1280,720},
  {1024,768},
  {1024,600},  
  {800,600},
  {800,480}, 
  {720,480}, 
  {640,480},
  {400,240},
  {352,288},
  {320,240},
  {200,120},    
  {176,144},
  {160,120},
};

#endif /* ifndef S5K5CCGX_H */
