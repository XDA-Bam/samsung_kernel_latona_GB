/*
 * drivers/media/video/ce147.h
 *
 * Register definitions for the NEC CE147 CameraChip.
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
 * modules/camera/ce147.h
 *
 * CE147 sensor driver header file
 *
 * Modified by paladin in Samsung Electronics
 */

#include<linux/videodev2.h>




#ifndef CE147_H
#define CE147_H

#define CAM_CE147_DBG_MSG            0
#define CAM_CE147_I2C_DBG_MSG        0
#define CAM_CE147_TUNE               0
#define CE147_DRIVER_NAME            "ce147"
#define CE147_MOD_NAME               "CE147: "

#define IN_IMAGE                     0
#define IN_SDCARD                    1

#define CE147_FIRMWARE_F2U_MMC_NAME   "/mnt/sdcard/external_sd/CE147F00.bin" //F2U
#define CE147_FIRMWARE_F3U_MMC_NAME   "/mnt/sdcard/external_sd/CE147F01.bin" //F3U
#define CE147_FIRMWARE_F2_MMC_NAME    "/mnt/sdcard/external_sd/CE147F02.bin" //F2
#define CE147_FIRMWARE_F3_MMC_NAME    "/mnt/sdcard/external_sd/CE147F03.bin" //F3

#define CE147_FIRMWARE_F2U_NAME       "/system/firmware/CE147F00.bin" //F2U
#define CE147_FIRMWARE_F3U_NAME       "/system/firmware/CE147F01.bin" //F3U
#define CE147_FIRMWARE_F2_NAME        "/system/firmware/CE147F02.bin" //F2
#define CE147_FIRMWARE_F3_NAME        "/system/firmware/CE147F03.bin" //F3

#define CE147_NOT_FW_UPDATE_OPERATION 0xFF

/**********************************************************/
/* define ISP support company                             */
/**********************************************************/
#define SAMSUNG_ELECTRO 0
#define SAMSUNG_FIBER_OPTICS 1
#define SAMSUNG_TECHWIN 2

#define ISP_SUPPORT_COMPANY SAMSUNG_ELECTRO 
/**********************************************************/

#define CE147_FW_MINOR_VERSION    0x0F
#define CE147_FW_MAJOR_VERSION    0x05
#define CE147_PRM_MINOR_VERSION   0x38
#define CE147_PRM_MAJOR_VERSION   0x08

#define CE147_THUMBNAIL_OFFSET    0x271000
#define CE147_YUV_OFFSET          0x280A00

#define CE147_I2C_ADDR            0x78>>1
#define CE147_I2C_RETRY           10
#define CE147_XCLK                24000000      //have to be fixed

#define SENSOR_DETECTED           1
#define SENSOR_NOT_DETECTED       0

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

enum ce147_op_mode {
	CE147_MODE_VIDEO = 0,
	CE147_MODE_IMAGE = 1,
};

typedef enum {
	AF_LENS_UNFOCUSED_STOP     = 0x00, // 0x00(&0x0F) 
	AF_LENS_UNFOCUSED_MOVING   = 0x00, // 0x01(&0x0F)   
	AF_LENZ_FOCUSED_STOP       = 0x02, // 0x02(&0x0F)
	AF_LENZ_INVALID_STOP       = 0x04, // 0x04(&0x0F)
	AF_LENS_INVALID_MOVING     = 0x05, // 0x05(&0x0F)     
} AFZOOM_Status;


/**
 * struct ce147_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct ce147_platform_data {
	int (*power_set)(enum v4l2_power power);
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
};

struct ce147_version {
	unsigned int major;
	unsigned int minor;
};

struct ce147_date_info {
	unsigned int year;
	unsigned int month;
	unsigned int date;
};

struct ce147_sensor_maker{
	unsigned int maker;
	unsigned int optical;
};

struct ce147_version_af{
	unsigned int low;
	unsigned int high;
};

struct ce147_gamma{
	unsigned int rg_low;
	unsigned int rg_high;
	unsigned int bg_low;
	unsigned int bg_high;	
};

struct ce147_position {
	int x;
	int y;
} ; 


/**   
 * struct ce147_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: ce147 chip version
 * @fps: frames per second value   
 */
struct ce147_sensor {
	const struct ce147_platform_data *pdata;
	struct mutex ce147_previewlock;
	struct mutex ce147_capturelock;
	struct mutex ce147_setlock;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct ce147_version fw;
	struct ce147_version prm;
	struct ce147_date_info dateinfo;  
	struct ce147_sensor_maker sensor_info;
	struct ce147_version_af af_info;
	struct ce147_gamma gamma;  
	struct ce147_position position;
	int sensor_version;
	unsigned int fw_dump_size;  
	struct ce147_version main_sw_fw;
	struct ce147_version main_sw_prm;
	struct ce147_date_info main_sw_dateinfo;  
	int check_dataline;  
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
};

/* delay define */
#define WAIT_CAM_AEAWB        100

/* State */
#define CE147_STATE_PREVIEW	  0x0000	/*  preview state */
#define CE147_STATE_CAPTURE	  0x0001	/*  capture state */
#define CE147_STATE_INVALID	  0x0002	/*  invalid state */

/* Mode */
#define CE147_MODE_CAMERA     1
#define CE147_MODE_CAMCORDER  2
#define CE147_MODE_VT         3

/* Preview Size */
#define CE147_PREVIEW_SIZE_1280_720   0
#define CE147_PREVIEW_SIZE_800_480    1
#define CE147_PREVIEW_SIZE_720_480    2
#define CE147_PREVIEW_SIZE_640_480    3
#define CE147_PREVIEW_SIZE_400_240    4
#define CE147_PREVIEW_SIZE_320_240    5
#define CE147_PREVIEW_SIZE_352_288    6
#define CE147_PREVIEW_SIZE_200_120    7
#define CE147_PREVIEW_SIZE_176_144    8
#define CE147_PREVIEW_SIZE_144_176    9
#define CE147_PREVIEW_SIZE_160_120    10

/* Image Size */
#define CE147_IMAGE_SIZE_3264_2448  0
#define CE147_IMAGE_SIZE_2560_1920  1
#define CE147_IMAGE_SIZE_2560_1536  2
#define CE147_IMAGE_SIZE_2560_1440  3
#define CE147_IMAGE_SIZE_2048_1536  4
#define CE147_IMAGE_SIZE_2048_1232  5
#define CE147_IMAGE_SIZE_1920_1080  6
#define CE147_IMAGE_SIZE_1600_1200  7
#define CE147_IMAGE_SIZE_1600_960   8
#define CE147_IMAGE_SIZE_1280_960   9
#define CE147_IMAGE_SIZE_1280_768   10
#define CE147_IMAGE_SIZE_1280_720   11
#define CE147_IMAGE_SIZE_1024_768   12
#define CE147_IMAGE_SIZE_800_600    13
#define CE147_IMAGE_SIZE_800_480    14
#define CE147_IMAGE_SIZE_720_480    15
#define CE147_IMAGE_SIZE_640_480    16
#define CE147_IMAGE_SIZE_400_240    17
#define CE147_IMAGE_SIZE_352_288    18
#define CE147_IMAGE_SIZE_320_240    19
#define CE147_IMAGE_SIZE_200_120    20
#define CE147_IMAGE_SIZE_176_144    21
#define CE147_IMAGE_SIZE_160_120    22

/* Image Effect */
#define CE147_EFFECT_OFF      1
#define CE147_EFFECT_SHARPEN  2
#define CE147_EFFECT_PURPLE   3
#define CE147_EFFECT_NEGATIVE 4
#define CE147_EFFECT_SEPIA    5
#define CE147_EFFECT_AQUA     6
#define CE147_EFFECT_GREEN    7
#define CE147_EFFECT_BLUE     8
#define CE147_EFFECT_PINK     9
#define CE147_EFFECT_YELLOW   10
#define CE147_EFFECT_GREY     11
#define CE147_EFFECT_RED      12
#define CE147_EFFECT_BW       13
#define CE147_EFFECT_ANTIQUE  14

/* ISO */
#define CE147_ISO_AUTO        1
#define CE147_ISO_50          2
#define CE147_ISO_100         3
#define CE147_ISO_200         4
#define CE147_ISO_400         5
#define CE147_ISO_800         6
#define CE147_ISO_1600        7

/* Photometry */
#define CE147_PHOTOMETRY_MATRIX   1
#define CE147_PHOTOMETRY_CENTER   2
#define CE147_PHOTOMETRY_SPOT     3

/* EV */
#define CE147_EV_MINUS_2P0    1
#define CE147_EV_MINUS_1P5    2
#define CE147_EV_MINUS_1P0    3
#define CE147_EV_MINUS_0P5    4
#define CE147_EV_DEFAULT      5
#define CE147_EV_PLUS_0P5     6
#define CE147_EV_PLUS_1P0     7
#define CE147_EV_PLUS_1P5     8
#define CE147_EV_PLUS_2P0     9

/* WDR */
#define CE147_WDR_OFF         1
#define CE147_WDR_ON          2
#define CE147_WDR_AUTO        3

/* Contrast */
#define CE147_CONTRAST_MINUS_3      1
#define CE147_CONTRAST_MINUS_2      2
#define CE147_CONTRAST_MINUS_1      3
#define CE147_CONTRAST_DEFAULT      4
#define CE147_CONTRAST_PLUS_1       5
#define CE147_CONTRAST_PLUS_2       6
#define CE147_CONTRAST_PLUS_3       7

/* Saturation */
#define CE147_SATURATION_MINUS_3    1
#define CE147_SATURATION_MINUS_2    2
#define CE147_SATURATION_MINUS_1    3
#define CE147_SATURATION_DEFAULT    4
#define CE147_SATURATION_PLUS_1     5
#define CE147_SATURATION_PLUS_2     6
#define CE147_SATURATION_PLUS_3     7

/* Sharpness */
#define CE147_SHARPNESS_MINUS_3     1
#define CE147_SHARPNESS_MINUS_2     2
#define CE147_SHARPNESS_MINUS_1     3
#define CE147_SHARPNESS_DEFAULT     4
#define CE147_SHARPNESS_PLUS_1      5
#define CE147_SHARPNESS_PLUS_2      6
#define CE147_SHARPNESS_PLUS_3      7

/* White Balance */
#define CE147_WB_AUTO               1
#define CE147_WB_DAYLIGHT           2
#define CE147_WB_CLOUDY             3
#define CE147_WB_INCANDESCENT       4
#define CE147_WB_FLUORESCENT        5

/* Image Stabilization */
#define CE147_ISC_STILL_OFF         1
#define CE147_ISC_STILL_ON          2
#define CE147_ISC_STILL_AUTO        3
#define CE147_ISC_MOVIE_ON          4

/* Scene Mode */
#define CE147_SCENE_OFF             1
#define CE147_SCENE_ASD             2
#define CE147_SCENE_SUNSET          3
#define CE147_SCENE_DAWN            4
#define CE147_SCENE_CANDLELIGHT     5
#define CE147_SCENE_BEACH_SNOW      6
#define CE147_SCENE_AGAINST_LIGHT   7
#define CE147_SCENE_TEXT            8
#define CE147_SCENE_NIGHTSHOT       9
#define CE147_SCENE_LANDSCAPE       10
#define CE147_SCENE_FIREWORKS       11
#define CE147_SCENE_PORTRAIT        12
#define CE147_SCENE_FALLCOLOR       13
#define CE147_SCENE_INDOORS         14
#define CE147_SCENE_SPORTS          15

/* Auto Exposure & Auto White Balance */
#define CE147_AE_UNLOCK_AWB_UNLOCK  0
#define CE147_AE_LOCK_AWB_LOCK      1
#define CE147_AE_LOCK_AWB_UNLOCK    2
#define CE147_AE_UNLOCK_AWB_LOCK    3

/* Anti-Shake */
#define CE147_ANTI_SHAKE_OFF        1
#define CE147_ANTI_SHAKE_ON         2

/* Flash Setting */
#define CE147_FLASH_CAPTURE_OFF     1
#define CE147_FLASH_CAPTURE_ON      2
#define CE147_FLASH_CAPTURE_AUTO    3

#define CE147_FLASH_MOVIE_OFF       1
#define CE147_FLASH_MOVIE_ON        2

/* Focus Mode */
//#define FEATURE_TOUCH_AF
#define CE147_AF_INIT_NORMAL        1
#define CE147_AF_INIT_MACRO         2
#define CE147_AF_INIT_FACE          3
#define CE147_AF_INIT_FACE_NOLINE   4
#define CE147_AF_INIT_CONTINUOUS 	5

/* Focust start/stop */
#define CE147_AF_START              1
#define CE147_AF_STOP               2

/* Auto Focus Status */
#define CE147_AF_STATUS_PROGRESS    1
#define CE147_AF_STATUS_SUCCESS     2
#define CE147_AF_STATUS_FAIL        3

/* Digital Zoom */
#define CE147_ZOOM_DEFAULT          0
#define CE147_ZOOM_1P00X            1
#define CE147_ZOOM_1P25X            2
#define CE147_ZOOM_1P50X            3
#define CE147_ZOOM_1P75X            4
#define CE147_ZOOM_2P00X            5
#define CE147_ZOOM_2P25X            6
#define CE147_ZOOM_2P50X            7
#define CE147_ZOOM_2P75X            8
#define CE147_ZOOM_3P00X            9
#define CE147_ZOOM_3P25X            10
#define CE147_ZOOM_3P50X            11
#define CE147_ZOOM_3P75X            12
#define CE147_ZOOM_4P00X            13

/* JPEG Quality */
#define CE147_JPEG_SUPERFINE        1
#define CE147_JPEG_FINE             2
#define CE147_JPEG_NORMAL           3
#define CE147_JPEG_ECONOMY          4

/* FACE Lock */
#define FACE_LOCK_OFF				0
#define FACE_LOCK_ON				1
#define FIRST_FACE_TRACKING			2
#define FACE_LOCK_MAX				3



static const u8 BatchReflectionRequest_list[] = {0x01, 0x00};
static const u8 BatchReflectionCheck_list[]   = {0x02};

/* Change YUV order & Modify module current */
static const u8 ConfigSensorYCOsetting_list[] = {0x03,0x00};

/************** Saturation Settings ****************/
/* Modified to SEHF Variable */
static const u8 Saturation_Plus3_List[]   = {0x3D, 0x06, 0x06};
static const u8 Saturation_Plus2_List[]   = {0x3D, 0x06, 0x05};
static const u8 Saturation_Plus1_List[]   = {0x3D, 0x06, 0x04};
static const u8 Saturation_Default_List[] = {0x3D, 0x06, 0x03};
static const u8 Saturation_Minus1_List[]  = {0x3D, 0x06, 0x02};
static const u8 Saturation_Minus2_List[]  = {0x3D, 0x06, 0x01};
static const u8 Saturation_Minus3_List[]  = {0x3D, 0x06, 0x00};


/************** Contrast Settings ****************/
/* Modified to SEHF Variable */
static const u8 Contrast_Plus3_List[]     = {0x3D, 0x07, 0x06};
static const u8 Contrast_Plus2_List[]     = {0x3D, 0x07, 0x05};
static const u8 Contrast_Plus1_List[]     = {0x3D, 0x07, 0x04};
static const u8 Contrast_default_List[]   = {0x3D, 0x07, 0x03};
static const u8 Contrast_Minus1_List[]    = {0x3D, 0x07, 0x02};
static const u8 Contrast_Minus2_List[]    = {0x3D, 0x07, 0x01};
static const u8 Contrast_Minus3_List[]    = {0x3D, 0x07, 0x00};


/************** Sharpness Settings ****************/
/* Modified to SEHF Variable */
static const u8 SharpnessPlus3_list[]     = {0x3D, 0x02, 0x06};
static const u8 SharpnessPlus2_list[]     = {0x3D, 0x02, 0x05};
static const u8 SharpnessPlus1_list[]     = {0x3D, 0x02, 0x04};
static const u8 SharpnessDefault_list[]   = {0x3D, 0x02, 0x03};
static const u8 SharpnessMinus1_list[]    = {0x3D, 0x02, 0x02};
static const u8 SharpnessMinus2_list[]    = {0x3D, 0x02, 0x01};
static const u8 SharpnessMinus3_list[]    = {0x3D, 0x02, 0x00};


/************** White Balance Setting ******************/
/* Modified to SEHF Variable */
static const u8 WB_Auto_Ctrl_List[]         = {0x04, 0x11, 0x00};
static const u8 WB_Auto_List[]              = {0x1A, 0x00};
static const u8 WB_Daylight_Ctrl_List[]     = {0x04, 0x10, 0x00};
static const u8 WB_Daylight_List[]          = {0x1A, 0x01};
static const u8 WB_Incandescent_Ctrl_List[] = {0x04, 0x10, 0x02};
static const u8 WB_Incandescent_List[]      = {0x1A, 0x01};
static const u8 WB_Fluorescent_Ctrl_List[]  = {0x04, 0x10, 0x03};
static const u8 WB_Fluorescent_List[]       = {0x1A, 0x01};
static const u8 WB_Cloudy_Ctrl_List[]       = {0x04, 0x10, 0x01};
static const u8 WB_Cloudy_List[]            = {0x1A, 0x01};

/************** Wide Dynamic Range Setting ****************/
/* Modified to SEHF Variable */
static const u8 WDR_OFF_list[]  = {0x88,0x00};
static const u8 WDR_ON_list[]   = {0x88,0x01};

/************** Exposure Value Setting ****************/
/* Modified to SEHF Variable */
static const u8 EV_Minus_2P0_List[]  = {0x04, 0x02, 0x02};
static const u8 EV_Minus_1P5_List[]  = {0x04, 0x02, 0x03};
static const u8 EV_Minus_1P0_List[]  = {0x04, 0x02, 0x04};
static const u8 EV_Minus_0P5_List[]  = {0x04, 0x02, 0x05};
static const u8 EV_Default_List[]    = {0x04, 0x02, 0x06};
static const u8 EV_Plus_0P5_List[]   = {0x04, 0x02, 0x07};
static const u8 EV_Plus_1P0_List[]   = {0x04, 0x02, 0x08};
static const u8 EV_Plus_1P5_List[]   = {0x04, 0x02, 0x09};
static const u8 EV_Plus_2P0_List[]   = {0x04, 0x02, 0x0A};

static const u8 EV_Minus_2P0_HD_List[]  = {0x04, 0x02, 0x0F};
static const u8 EV_Minus_1P5_HD_List[]  = {0x04, 0x02, 0x10};
static const u8 EV_Minus_1P0_HD_List[]  = {0x04, 0x02, 0x11};
static const u8 EV_Minus_0P5_HD_List[]  = {0x04, 0x02, 0x12};
static const u8 EV_Default_HD_List[]    = {0x04, 0x02, 0x13};
static const u8 EV_Plus_0P5_HD_List[]   = {0x04, 0x02, 0x14};
static const u8 EV_Plus_1P0_HD_List[]   = {0x04, 0x02, 0x15};
static const u8 EV_Plus_1P5_HD_List[]   = {0x04, 0x02, 0x16};
static const u8 EV_Plus_2P0_HD_List[]   = {0x04, 0x02, 0x17};


/************** Effect Setting ********************/
/* Modified to SEHF Variable */
#if (ISP_SUPPORT_COMPANY == SAMSUNG_ELECTRO)
static const u8 Effect_None_List[]      = {0x3D, 0x05, 0x00};
static const u8 Effect_Purple_List[]    = {0x3D, 0x05, 0x0C};
static const u8 Effect_Grey_List[]      = {0x3D, 0x05, 0x02};
static const u8 Effect_Sepia_List[]     = {0x3D, 0x05, 0x03};
static const u8 Effect_Sharpen_List[]   = {0x3D, 0x05, 0x04};
static const u8 Effect_Negative_List[]  = {0x3D, 0x05, 0x05};
static const u8 Effect_Aqua_List[]      = {0x3D, 0x05, 0x0D}; // water color
static const u8 Effect_Red_List[]       = {0x3D, 0x05, 0x07};
static const u8 Effect_Pink_List[]      = {0x3D, 0x05, 0x08};
static const u8 Effect_Yellow_List[]    = {0x3D, 0x05, 0x09};
static const u8 Effect_Green_List[]     = {0x3D, 0x05, 0x0A};
static const u8 Effect_Blue_List[]      = {0x3D, 0x05, 0x0B};
static const u8 Effect_BW_List[]        = {0x3D, 0x05, 0x01};
static const u8 Effect_Antique_List[]   = {0x3D, 0x05, 0x06};
#else
static const u8 Effect_None_List[]      = {0x3D, 0x05, 0x00};
static const u8 Effect_Purple_List[]    = {0x3D, 0x05, 0x01};
static const u8 Effect_Grey_List[]      = {0x3D, 0x05, 0x02};
static const u8 Effect_Sepia_List[]     = {0x3D, 0x05, 0x03};
static const u8 Effect_Sharpen_List[]   = {0x3D, 0x05, 0x04};
static const u8 Effect_Negative_List[]  = {0x3D, 0x05, 0x05};
static const u8 Effect_Aqua_List[]      = {0x3D, 0x05, 0x06};
static const u8 Effect_Red_List[]       = {0x3D, 0x05, 0x07};
static const u8 Effect_Pink_List[]      = {0x3D, 0x05, 0x08};
static const u8 Effect_Yellow_List[]    = {0x3D, 0x05, 0x09};
static const u8 Effect_Green_List[]     = {0x3D, 0x05, 0x0A};
static const u8 Effect_Blue_List[]      = {0x3D, 0x05, 0x0B};
static const u8 Effect_BW_List[]        = {0x3D, 0x05, 0x0C};
static const u8 Effect_Antique_List[]   = {0x3D, 0x05, 0x0D};
#endif


/************** Photometry Setting ********************/
/* Modified to SEHF Variable */
static const u8 Photometry_Center_List[]  = {0x04, 0x00, 0x00};
static const u8 Photometry_Spot_List[]    = {0x04, 0x00, 0x01};
static const u8 Photometry_Matrix_List[]  = {0x04, 0x00, 0x02};
static const u8 Photometry_MatrixHD_List[]  = {0x04, 0x00, 0x03};


/************** ISO Settings ****************/
/* Modified to SEHF Variable */
static const u8 ISOAuto_list[] = {0x04, 0x01, 0x06};
static const u8 ISO50_list[]   = {0x04, 0x01, 0x07};
static const u8 ISO100_list[]  = {0x04, 0x01, 0x08};
static const u8 ISO200_list[]  = {0x04, 0x01, 0x09};
static const u8 ISO400_list[]  = {0x04, 0x01, 0x0A};
static const u8 ISO800_list[]  = {0x04, 0x01, 0x0B};
static const u8 ISO1600_list[] = {0x04, 0x01, 0x0C};  // added for 8M(HALO)
static const u8 ISOCamcorder_list[]   = {0x04, 0x01, 0x03};
static const u8 ISOCamcorderHD_list[] = {0x04, 0x01, 0x02}; //03 : 30fps fix , 02 : 22fps fix


/************** Anti-Shake Settings ****************/
/* Modified to SEHF Variable */
static const u8 AntiShake_OFF_list[] = {0x5B, 0x00};
static const u8 AntiShake_ON_list[]  = {0x5B, 0x01};


/**************** Camera Preview Mode *************************/
/* Modified to SEHF Variable */
static const u8 ConfigSensorfliker_list_60Hz[]     = {0x14, 0x03};
static const u8 ConfigSensorfliker_list_50Hz[]     = {0x14, 0x02};
static const u8 ConfigSensorPreviewStart_list[]    = {0x6B, 0x01}; // Preview Start
static const u8 ConfigSensorPreviewStop_list[]     = {0x6B, 0x00}; // Preview Stop

/* Normal */
static const u8 ConfigSensorPreview[11][3] = {{0x54, 0x16, 0x02},  //1280X720
	{0x54, 0x13, 0x01},  //800X480
	{0x54, 0x20, 0x01},  //720X480
	{0x54, 0x04, 0x01},  //640X480
	{0x54, 0x12, 0x01},  //400X240
	{0x54, 0x1F, 0x01},  //352X288
	{0x54, 0x02, 0x01},  //320X240
	{0x54, 0x10, 0x01},  //200X120
	{0x54, 0x1E, 0x01},  //176X144
	{0x54, 0x26, 0x01},  //144X176 
	{0x54, 0x01, 0x01}}; //160X120

/* Slow Motion */
static const u8 ConfigSensorPreview_Fast[5][3] = {{0x54, 0x02, 0x04},  //320X240
	{0x54, 0x10, 0x04},  //200X120
	{0x54, 0x1E, 0x04},  //176X144
	{0x54, 0x26, 0x04},  //144X176
	{0x54, 0x01, 0x04}}; //160X120

/* 720P */
static const u8 ConfigSensorPreview720P_on_list[]  = {0x03, 0x01}; 
static const u8 ConfigSensorPreview720P_off_list[] = {0x03, 0x00}; 
static const u8 ConfigSensorPreview720P_chk_list[] = {0x05, 0x00, 0x00}; 

/* Flip */
static const u8 ConfigSensorPreviewFlipMirror_list[] = {0x69, 0x11}; // Preview Flip & Mirror


/*********** Buffering Capture Settings for different Sizes ******/
static const u8 ConfigSensorBufferingCapture[23][5] = {{0x73,0x0E,0x00,0x01,0x00},  //3264X2448
	{0x73,0x0B,0x00,0x01,0x00},  //2560X1920
	{0x73,0x15,0x00,0x01,0x00},  //2560X1536
	{0x73,0x21,0x00,0x01,0x00},  //2560X1440
	{0x73,0x09,0x00,0x01,0x00},  //2048X1536
	{0x73,0x23,0x00,0x01,0x00},  //2048X1232
	{0x73,0x17,0x00,0x01,0x00},  //1920X1080
	{0x73,0x08,0x00,0x01,0x00},  //1600X1200
	{0x73,0x0E,0x00,0x01,0x00},  //1600X960
	{0x73,0x07,0x00,0x01,0x00},  //1280X960
	{0x73,0x14,0x00,0x01,0x00},  //1280X768
	{0x73,0x16,0x00,0x01,0x00},  //1280X720
	{0x73,0x06,0x00,0x01,0x00},  //1024X768
	{0x73,0x05,0x00,0x01,0x00},  //800X600
	{0x73,0x13,0x00,0x01,0x00},  //800X480
	{0x73,0x20,0x00,0x01,0x00},  //720X480
	{0x73,0x04,0x00,0x01,0x00},  //640X480
	{0x73,0x12,0x00,0x01,0x00},  //400X240
	{0x73,0x1F,0x00,0x01,0x00},  //352X288
	{0x73,0x02,0x00,0x01,0x00},  //320X240
	{0x73,0x10,0x00,0x01,0x00},  //200X120
	{0x73,0x1E,0x00,0x01,0x00},  //176X144
	{0x73,0x01,0x00,0x01,0x00}}; //160X120                                                    

/* Start Buffering Capture */
static const u8 BufferingCaptureStart_list[] = {0x74, 00}; 


/**************** Make images in lump settings ********************/
#if (ISP_SUPPORT_COMPANY == SAMSUNG_ELECTRO)
static const u8 MakeImagesInLump[23][3] = {{0x8F, 0x00, 0x04},  //3264X2448
	{0x8F, 0x00, 0x04},  //2560X1920
	{0x8F, 0x00, 0x13},  //2560X1536
	{0x8F, 0x00, 0x21},  //2560X1440
	{0x8F, 0x00, 0x04},  //2048X1536
	{0x8F, 0x00, 0x13},  //2048X1232
	{0x8F, 0x00, 0x17},  //1920X1080
	{0x8F, 0x00, 0x04},  //1600X1200
	{0x8F, 0x00, 0x13},  //1600X960
	{0x8F, 0x00, 0x13},  //1280X960
	{0x8F, 0x00, 0x14},  //1280X768
	{0x8F, 0x00, 0x13},  //1280X720
	{0x8F, 0x00, 0x06},  //1024X768
	{0x8F, 0x00, 0x05},  //800X600
	{0x8F, 0x00, 0x13},  //800X480
	{0x8F, 0x00, 0x20},  //720X480
	{0x8F, 0x00, 0x04},  //640X480
	{0x8F, 0x00, 0x0F},  //400X240
	{0x8F, 0x00, 0x02},  //352X288
	{0x8F, 0x00, 0x02},  //320X240
	{0x8F, 0x00, 0x0E},  //200X120
	{0x8F, 0x00, 0x1E},  //176X144
	{0x8F, 0x00, 0x01}}; //160X120
#else
static const u8 MakeImagesInLump[23][3] = {{0x8F, 0x00, 0x0E},  //3264X2448
	{0x8F, 0x00, 0x0B},  //2560X1920
	{0x8F, 0x00, 0x15},  //2560X1536
	{0x8F, 0x00, 0x21},  //2560X1440
	{0x8F, 0x00, 0x09},  //2048X1536
	{0x8F, 0x00, 0x23},  //2048X1232
	{0x8F, 0x00, 0x17},  //1920X1080
	{0x8F, 0x00, 0x08},  //1600X1200
	{0x8F, 0x00, 0x0F},  //1600X960
	{0x8F, 0x00, 0x07},  //1280X960
	{0x8F, 0x00, 0x14},  //1280X768
	{0x8F, 0x00, 0x16},  //1280X720
	{0x8F, 0x00, 0x06},  //1024X768
	{0x8F, 0x00, 0x05},  //800X600
	{0x8F, 0x00, 0x13},  //800X480
	{0x8F, 0x00, 0x20},  //720X480
	{0x8F, 0x00, 0x04},  //640X480
	{0x8F, 0x00, 0x0F},  //400X240
	{0x8F, 0x00, 0x02},  //352X288
	{0x8F, 0x00, 0x10},  //320X240
	{0x8F, 0x00, 0x0E},  //200X120
	{0x8F, 0x00, 0x01},  //176X144
	{0x8F, 0x00, 0x01}}; //160X120                                       
#endif

static const u8 MakeImagesInLump_Status[]  = {0x8E, 0x00};
static const u8 MakeImagesInLump_Status1[] = {0x8E, 0x01};

static const u8 YUVDataOutputSetting[]    = {0x65,0x01,0x00};
static const u8 JPEGDataOutputSetting[]   = {0x65,0x02,0x00};
static const u8 DataOutputRequest[]       = {0x66};

static const u8 DataTransmissionCheck[]   = {0x61};


/************** AF & Zoom Settings ****************/
static const u8 Lense_MotorInit_list[]   = {0x30, 0x01};
static const u8 Lense_MotorDeinit_list[] = {0x30, 0x00};

static const u8 Lense_BackLashAndLocus_List[] = {0x20, 0x04};
static const u8 Lense_ZoomBackLash_List[]     = {0x20, 0x06};
static const u8 Lense_AFBackLash_List[]       = {0x20, 0x05};
static const u8 Lense_Locus_List[]            = {0x20, 0x03};

static const u8 Lense_AFStandardMode_list[]   = {0x20, 0x00};
static const u8 Lense_AFMacroMode_list[]      = {0x20, 0x01};
static const u8 Lense_AFContinuousMode_list[]	= {0x20, 0x02};
static const u8 Lense_AFMacroOff_list[]		  = {0x33, 0x02, 0x00, 0x00};

static const u8 Lense_AFStart_List[]  = {0x23};
static const u8 Lense_AFCancel_List[] = {0x35};
static const u8 Lense_TouchAFOff_List[]  = {0x4D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static const u8 Lense_CheckStatus_List[]                 = {0x24};
static const u8 Lense_CheckTuningStatus_List[]           = {0xE8, 0x08, 0x01};
static const u8 Lense_CheckMaxZoomLevelStatus_List[]     = {0xE8, 0x08, 0x10};
static const u8 Lense_CheckCurrentZoomLevelStatus_List[] = {0xE8, 0x08, 0x0F};

static const u8 Lense_SetMaxZoomLevelStatus_List[] = {0xE9, 0x08, 0x10, 0x09};
static const u8 Lense_FROMWriteFinishStatus_List[] = {0xE3, 0x00};

static const u8 Lense_SetZoomLevel_Up_List[]   = {0x31, 0x11, 0x00, 0x00};
static const u8 Lense_SetZoomLevel_Down_List[] = {0x31, 0x01, 0x00, 0x00};
static const u8 Lense_SetZoomRelease_List[]    = {0x20, 0x02};

static const u8 Lense_SetZoomLevel_MIN_List[] = {0x31, 0x00, 0x00, 0x00};
static const u8 Lense_SetZoomLevel_MAX_List[] = {0x31, 0x10, 0x00, 0x00};

static const u8 Lense_SetZoomLevel_0_List[] = {0x31, 0x02, 0x00, 0x00};
static const u8 Lense_SetZoomLevel_1_List[] = {0x31, 0x02, 0x01, 0x00};
static const u8 Lense_SetZoomLevel_2_List[] = {0x31, 0x02, 0x02, 0x00};
static const u8 Lense_SetZoomLevel_3_List[] = {0x31, 0x02, 0x03, 0x00};
static const u8 Lense_SetZoomLevel_4_List[] = {0x31, 0x02, 0x04, 0x00};
static const u8 Lense_SetZoomLevel_5_List[] = {0x31, 0x02, 0x05, 0x00};
static const u8 Lense_SetZoomLevel_6_List[] = {0x31, 0x02, 0x06, 0x00};
static const u8 Lense_SetZoomLevel_7_List[] = {0x31, 0x02, 0x07, 0x00};
static const u8 Lense_SetZoomLevel_8_List[] = {0x31, 0x02, 0x08, 0x00};

#if (ISP_SUPPORT_COMPANY == SAMSUNG_ELECTRO)
static const u8 Lense_SetDZoom_1p00x_List[]  = {0xB9, 0xFF};
static const u8 Lense_SetDZoom_1p25x_List[]  = {0xB9, 0xCC};
static const u8 Lense_SetDZoom_1p50x_List[]  = {0xB9, 0xAA};
static const u8 Lense_SetDZoom_1p75x_List[]  = {0xB9, 0x91};
static const u8 Lense_SetDZoom_2p00x_List[]  = {0xB9, 0x7F};
static const u8 Lense_SetDZoom_2p25x_List[]  = {0xB9, 0x71};
static const u8 Lense_SetDZoom_2p50x_List[]  = {0xB9, 0x65};
static const u8 Lense_SetDZoom_2p75x_List[]  = {0xB9, 0x5C};
static const u8 Lense_SetDZoom_3p00x_List[]  = {0xB9, 0x54};
static const u8 Lense_SetDZoom_3p25x_List[]  = {0xB9, 0x4E};
static const u8 Lense_SetDZoom_3p50x_List[]  = {0xB9, 0x48};
static const u8 Lense_SetDZoom_3p75x_List[]  = {0xB9, 0x43};
static const u8 Lense_SetDZoom_4p00x_List[]  = {0xB9, 0x3F};
#else
static const u8 Lense_SetDZoom_1p00x_List[]  = {0xB9, 0xFF};
static const u8 Lense_SetDZoom_1p25x_List[]  = {0xB9, 0xCB};
static const u8 Lense_SetDZoom_1p50x_List[]  = {0xB9, 0xA9};
static const u8 Lense_SetDZoom_1p75x_List[]  = {0xB9, 0x91};
static const u8 Lense_SetDZoom_2p00x_List[]  = {0xB9, 0x7F};
static const u8 Lense_SetDZoom_2p25x_List[]  = {0xB9, 0x71};
static const u8 Lense_SetDZoom_2p50x_List[]  = {0xB9, 0x66};
static const u8 Lense_SetDZoom_2p75x_List[]  = {0xB9, 0x5D};
static const u8 Lense_SetDZoom_3p00x_List[]  = {0xB9, 0x55};
static const u8 Lense_SetDZoom_3p25x_List[]  = {0xB9, 0x4E};
static const u8 Lense_SetDZoom_3p50x_List[]  = {0xB9, 0x48};
static const u8 Lense_SetDZoom_3p75x_List[]  = {0xB9, 0x43};
static const u8 Lense_SetDZoom_4p00x_List[]  = {0xB9, 0x3F};
#endif

static const u8 ce147_buf_set_dzoom[31] = {0xff,0xe7,0xd3,0xc2,0xb4,0xa7, \
	0x9c,0x93,0x8b,0x83,0x7c,0x76, \
		0x71,0x6c,0x67,0x63,0x5f,0x5b, \
		0x58,0x55,0x52,0x4f,0x4d,0x4a, \
		0x48,0x46,0x44,0x42,0x41,0x40,0x3f};

static const u8 Lense_CheckDZoomStatus_List[] = {0xBA};
static const u8 Lense_CheckFlashStatus_List[] = {0x16};

/* quality tuning batch on/off */
static const u8 ImageTuningBatch_On_List[]  = {0x3A, 0x00, 0x10, 0x00, 0x00};
static const u8 ImageTuningBatch_Off_List[] = {0x3B, 0x00, 0x10, 0x00, 0x00};


/*************  JPEG Compression Tables **********/
/*set the mode as jpeg compression.*/
#if (ISP_SUPPORT_COMPANY == SAMSUNG_ELECTRO)
static const u8 JpegCompression_SuperFine_List[] = {0x90, 0x00, 0xA4, 0x06, 0x78, 0x05, 0x03, 0x00};
static const u8 JpegCompression_Fine_List[]      = {0x90, 0x00, 0x78, 0x05, 0x4C, 0x04, 0x03, 0x00};
static const u8 JpegCompression_Normal_List[]    = {0x90, 0x00, 0x4C, 0x04, 0x20, 0x03, 0x03, 0x00};
static const u8 JpegCompression_Economy_List[]   = {0x90, 0x00, 0x20, 0x03, 0xF4, 0x01, 0x03, 0x00};
#else
static const u8 JpegCompression_SuperFine_List[] = {0x90, 0x00, 0xDC, 0x05, 0xB0, 0x04, 0x03, 0x00};
static const u8 JpegCompression_Fine_List[]      = {0x90, 0x00, 0xB0, 0x04, 0xE8, 0x03, 0x03, 0x00};
static const u8 JpegCompression_Normal_List[]    = {0x90, 0x00, 0xE8, 0x03, 0xBC, 0x02, 0x03, 0x00};
static const u8 JpegCompression_Economy_List[]   = {0x90, 0x00, 0xBC, 0x02, 0x90, 0x01, 0x03, 0x00};
#endif

static const u8 Exif_List1[] = {0xCB, 0X07, 0X01, 0X01, 0X00, 0X00, 0X00};
static const u8 Exif_List2[] = {0xCC, 0X01, 0X20, 0X20, 0X20, 0X20, 0X20,
	0X20, 0X20, 0X20, 0X20, 0X20, 0X20, 0X20, 
	0X20, 0X20, 0X20, 0X20, 0X20, 0X20, 0X20, 
	0X20};
static const u8 Exif_List3[] = {0xCA, 0X00};

static const u8 JpegCompression1[] = {0x76, 0x04, 0x00, 0x00};
static const u8 JpegCompression2[] = {0x65, 0x02, 0x00};
static const u8 JpegCompression3[] = {0x66};


// jpeg compression start
static const u8 JpegCompression_Start_List[]  = {0x92, 0x00};
static const u8 JpegCompression_Start_List2[] = {0x92, 0x01};
static const u8 ExifCompression_Start_List[]  = {0xCA, 0x00};

static const u8 CameraReadCommand_JPEGCompressionStatus[]= {0x93};


/**************** Thumbnail settings ********************/
static const u8 Thumbnail_ImageQVGAConvertion_List[]  = {0x76, 0x06, 0x00, 0x00};
static const u8 Thumbnail_ImageVGAConvertion_List[]   = {0x76, 0x08, 0x00, 0x00};


/************* Scene Mode Settings **********************/
static const u8 ASD_CheckStatus_List[] = {0x83};

/*Off*/
static const u8 SceneModeOff_List1[] = {0x82, 0x00};
static const u8 SceneModeOff_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeOff_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeOff_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeOff_List5[] = {0x1A, 0x00};
static const u8 SceneModeOff_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeOff_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeOff_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeOff_List9[] = {0x20, 0x00};

/*ASD*/
static const u8 SceneModeASD_List1[] = {0x82, 0x01};
static const u8 SceneModeASD_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeASD_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeASD_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeASD_List5[] = {0x1A, 0x00};
static const u8 SceneModeASD_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeASD_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeASD_List8[] = {0x9A, 0x03, 0x01, 0x03};
static const u8 SceneModeASD_List9[] = {0x20, 0x00};

/*Portrait*/
static const u8 SceneModePortrait_List1[] = {0x82, 0x00};
static const u8 SceneModePortrait_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModePortrait_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModePortrait_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModePortrait_List5[] = {0x1A, 0x00};
static const u8 SceneModePortrait_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModePortrait_List7[] = {0x3D, 0x02, 0x02};
static const u8 SceneModePortrait_List8[] = {0x9A, 0x03, 0x01, 0x03};
static const u8 SceneModePortrait_List9[] = {0x20, 0x00};

/*Lanscape*/
static const u8 SceneModeLanscape_List1[] = {0x82, 0x00};
static const u8 SceneModeLanscape_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeLanscape_List3[] = {0x04, 0x00, 0x02};
static const u8 SceneModeLanscape_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeLanscape_List5[] = {0x1A, 0x00};
static const u8 SceneModeLanscape_List6[] = {0x3D, 0x06, 0x04};
static const u8 SceneModeLanscape_List7[] = {0x3D, 0x02, 0x04};
static const u8 SceneModeLanscape_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeLanscape_List9[] = {0x20, 0x00};

/*Sunset*/
static const u8 SceneModeSunset_List1[] = {0x82, 0x00};
static const u8 SceneModeSunset_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeSunset_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeSunset_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeSunset_List5[] = {0x04, 0x10, 0x00};
static const u8 SceneModeSunset_List6[] = {0x1A, 0x01};
static const u8 SceneModeSunset_List7[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeSunset_List8[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeSunset_List9[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeSunset_List10[] = {0x20, 0x00};

/*Dawn*/
static const u8 SceneModeDawn_List1[] = {0x82, 0x00};
static const u8 SceneModeDawn_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeDawn_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeDawn_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeDawn_List5[] = {0x04, 0x10, 0x03};
static const u8 SceneModeDawn_List6[] = {0x1A, 0x01};
static const u8 SceneModeDawn_List7[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeDawn_List8[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeDawn_List9[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeDawn_List10[] = {0x20, 0x00};

/*Nightshot*/
static const u8 SceneModeNightShot_List1[] = {0x82, 0x00};
static const u8 SceneModeNightShot_List2[] = {0x04, 0x01, 0x17};
static const u8 SceneModeNightShot_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeNightShot_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeNightShot_List5[] = {0x1A, 0x00};
static const u8 SceneModeNightShot_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeNightShot_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeNightShot_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeNightShot_List9[] = {0x20, 0x00};

/*Text */
static const u8 SceneModeText_List1[] = {0x82, 0x00};
static const u8 SceneModeText_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeText_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeText_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeText_List5[] = {0x1A, 0x00};
static const u8 SceneModeText_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeText_List7[] = {0x3D, 0x02, 0x05};
static const u8 SceneModeText_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeText_List9[] = {0x20, 0x01};

/*Sports*/
static const u8 SceneModeSports_List1[] = {0x82, 0x00};
static const u8 SceneModeSports_List2[] = {0x04, 0x01, 0x12};
static const u8 SceneModeSports_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeSports_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeSports_List5[] = {0x1A, 0x00};
static const u8 SceneModeSports_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeSports_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeSports_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeSports_List9[] = {0x20, 0x00};

/*AgainstLight */
static const u8 SceneModeAgainstLight_List1[] = {0x82, 0x00};
static const u8 SceneModeAgainstLight_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeAgainstLight_List3_0[] = {0x04, 0x00, 0x00};
static const u8 SceneModeAgainstLight_List3_1[] = {0x04, 0x00, 0x01};
static const u8 SceneModeAgainstLight_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeAgainstLight_List5[] = {0x1A, 0x00};
static const u8 SceneModeAgainstLight_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeAgainstLight_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeAgainstLight_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeAgainstLight_List9[] = {0x20, 0x00};

/*Indoors*/
static const u8 SceneModeIndoor_List1[] = {0x82, 0x00};
static const u8 SceneModeIndoor_List2[] = {0x04, 0x01, 0x09};
static const u8 SceneModeIndoor_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeIndoor_List4[] = {0x04, 0x02, 0x06};
static const u8 SceneModeIndoor_List5[] = {0x1A, 0x00};
static const u8 SceneModeIndoor_List6[] = {0x3D, 0x06, 0x04};
static const u8 SceneModeIndoor_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeIndoor_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeIndoor_List9[] = {0x20, 0x00};

/*Beach&Snow*/
static const u8 SceneModeBeachSnow_List1[] = {0x82, 0x00};
static const u8 SceneModeBeachSnow_List2[] = {0x04, 0x01, 0x07};
static const u8 SceneModeBeachSnow_List3[] = {0x04, 0x00, 0x00};              
static const u8 SceneModeBeachSnow_List4[] = {0x04, 0x02, 0x08};             
static const u8 SceneModeBeachSnow_List5[] = {0x1A, 0x00};               
static const u8 SceneModeBeachSnow_List6[] = {0x3D, 0x06, 0x04};
static const u8 SceneModeBeachSnow_List7[] = {0x3D, 0x02, 0x03}; 
static const u8 SceneModeBeachSnow_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeBeachSnow_List9[] = {0x20, 0x00};

/* Fall Color */
static const u8 SceneModeFallColor_List1[] = {0x82, 0x00};
static const u8 SceneModeFallColor_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeFallColor_List3[] = {0x04, 0x00, 0x00};
static const u8 SceneModeFallColor_List4[] = {0x04, 0x02, 0x06};              
static const u8 SceneModeFallColor_List5[] = {0x1A, 0x00};              
static const u8 SceneModeFallColor_List6[] = {0x3D, 0x06, 0x05};             
static const u8 SceneModeFallColor_List7[] = {0x3D, 0x02, 0x03};   
static const u8 SceneModeFallColor_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeFallColor_List9[] = {0x20, 0x00};

/*Fireworks */
static const u8 SceneModeFireWorks_List1[] = {0x82, 0x00};
#if (ISP_SUPPORT_COMPANY == SAMSUNG_ELECTRO)
static const u8 SceneModeFireWorks_List2[] = {0x04, 0x01, 0x07};
#else
static const u8 SceneModeFireWorks_List2[] = {0x04, 0x01, 0x11};
#endif
static const u8 SceneModeFireWorks_List3[] = {0x04, 0x00, 0x00};             
static const u8 SceneModeFireWorks_List4[] = {0x04, 0x02, 0x06};             
static const u8 SceneModeFireWorks_List5[] = {0x1A, 0x00};              
static const u8 SceneModeFireWorks_List6[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeFireWorks_List7[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeFireWorks_List8[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeFireWorks_List9[] = {0x20, 0x00};

/*Candlelight */
static const u8 SceneModeCandlelight_List1[] = {0x82, 0x00};
static const u8 SceneModeCandlelight_List2[] = {0x04, 0x01, 0x06};
static const u8 SceneModeCandlelight_List3[] = {0x04, 0x00, 0x00};             
static const u8 SceneModeCandlelight_List4[] = {0x04, 0x02, 0x06};     
static const u8 SceneModeCandlelight_List5[] = {0x04, 0x10, 0x00}; 
static const u8 SceneModeCandlelight_List6[] = {0x1A, 0x01};              
static const u8 SceneModeCandlelight_List7[] = {0x3D, 0x06, 0x03};
static const u8 SceneModeCandlelight_List8[] = {0x3D, 0x02, 0x03};
static const u8 SceneModeCandlelight_List9[] = {0x9A, 0x00, 0x00, 0x00};
static const u8 SceneModeCandlelight_List10[] = {0x20, 0x00};


/*************** FW read command ****************/
static const u8 ConfigSensorFirmWareInit_list[]    = {0xF0};      // FW Start
static const u8 ConfigSensorFirmWareEnd_list[]     = {0x95,0x00}; // FW end

static const u8 CameraReadCommand_FWVersion_List[] = {0x00, 0x00};
static const u8 CameraReadCommand_FWDate_List[] = {0x00, 0x01};
static const u8 CameraReadCommand_AFVersion_List[] = {0x00, 0x05};
static const u8 CameraReadCommand_CaptureStatus[]  = {0x6C};
static const u8 CameraReadCommand_CommandStatus[]  = {0xA1};
static const u8 CameraRead_FWEnd_Status_List[]     = {0x96};


/*************** FW Update Setting *****************/
static const u8 CameraRead_FWUpdater_Version_List[] = {0xF1};
static const u8 CameraRead_FWUpdate_Status_List[]   = {0xF5};
static const u8 CameraRead_FWUpdate_Dummy_List[]    = {0xF3};


/**************** Image stabilization settings ****************/
/* Modified to SEHF Variable */
static const u8 IS_Still_Off_list[]  = {0x5B, 1};
static const u8 IS_Still_On_list[]   = {0x5B, 2};
static const u8 IS_Still_Auto_list[] = {0x5B, 3};
static const u8 IS_Movie_On_list[]   = {0x5B, 18};


static const u8 IS_Command_list[] = {0x5C, 0x01, 0x01}; // IS setting command
static const u8 IS_Check_list[]   = {0x5D}; // IS status check


/**************** AE /WB Lock settings ****************/
static const u8 AEWB_UnLock_list[]        = {0x11, 0x00};
static const u8 AE_Lock_AWB_Unlock_list[] = {0x11, 0x01};
static const u8 AE_Unlock_AWB_Lock_list[] = {0x11, 0x10};
static const u8 AEWB_Lock_list[]          = {0x11, 0x11};


/**************** Flash settings ****************/
static const u8 FlashPreAF_Ctrl_list[]     = {0xB3, 0x01, 0x01, 0x00, 0x00};
static const u8 FlashCaptureAF_Ctrl_list[] = {0xB3, 0x03, 0x01, 0x00, 0x00};
static const u8 FlashMovieAF_Ctrl_list[]   = {0xB3, 0x00, 0x00, 0x0A, 0x01};

static const u8 FlashStatusCheck[]        = {0xC3};

static const u8 FlashAFOff_list[]         = {0xB2, 0x01, 0x00};
static const u8 FlashAFOn_list[]          = {0xB2, 0x01, 0x01};
static const u8 FlashAFAuto_list[]        = {0xB2, 0x01, 0x02};
static const u8 FlashCaptureOff_list[]    = {0xB2, 0x03, 0x00};
static const u8 FlashCaptureOn_list[]     = {0xB2, 0x03, 0x01};
static const u8 FlashCaptureAuto_list[]   = {0xB2, 0x03, 0x02};
static const u8 FlashMovieOff_list[]      = {0x06, 0x00, 0x00};
static const u8 FlashMovieOn_list[]       = {0x06, 0x00, 0x01};


/**************** FaceDetection Settings ****************/
static const u8 FaceDetection_On_list[]     = {0x9A, 0x03, 0x01, 0x0A};
static const u8 FaceDetection_On_NoLine_list[]     = {0x9A, 0x03, 0x00, 0x0A};
static const u8 FaceDetection_Off_list[]    = {0x9A, 0x00, 0x00, 0x00};
static const u8 FaceDetection_box_On_list[]  = {0x9A, 0x03, 0x01, 0x0A};
static const u8 FaceDetection_box_Off_list[] = {0x9A, 0x03, 0x00, 0x0A};
static const u8 FaceDetection_Check_list[]  = {0x9B};
static const u8 FaceDetection_Lock_list[]   = {0x9C, 0x01};
static const u8 FaceDetection_UnLock_list[] = {0x9C, 0x00};
static const u8 FaceDetection_Tracking_list[] = {0x9C, 0x02};


/**************** SmileShot Settings ****************/
static const u8 SmileShutterOn_list[]  = {0x43, 0x01};
static const u8 SmileShutterOff_list[] = {0x43, 0x00};
static const u8 SmileValue_list[]      = {0x44};


/**************** BlinkShot Settings ****************/
static const u8 Blink_FaceDetect_list[] = {0x9A, 0x43,0x01,0x03};
static const u8 BlinkValue_list[]       = {0x46};
static const u8 BlinkLocation_list[]    = {0x9B};

/**************** AE Speed Control Settings ****************/
static const u8 AE_Speed_Ctrl_Normal[]  = {0x04, 0x03, 0x00};
static const u8 AE_Speed_Ctrl_2sec[]    = {0x04, 0x03, 0x01};
static const u8 AE_Speed_Ctrl_2sec_HD[] = {0x04, 0x03, 0x02};

/**************** Gammal Settings ****************/
static const u8 Gamma_Normal[] = {0x3D, 0x01, 0x00};
static const u8 Gammal_HD[]    = {0x3D, 0x01, 0x01};
static const u8 Preview_UV[]   = {0x3D, 0x03, 0x00};
static const u8 Preview_UV_HD[]= {0x3D, 0x03, 0x01};

static const u8 Lense_AFoff_list[] = {0x33, 0x02, 0x64, 0x00};

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

struct ce147_preview_size {
	unsigned int width;
	unsigned int height;
};

const static struct ce147_preview_size ce147_preview_sizes[] = {
	{1280,720},  
	{800,480},    
	{720,480},   
	{640,480},    
	{400,240},  
	{352,288},  
	{320,240},  
	{200,120},   
	{176,144},  
	{144,176},  // for vt
	{160,120},    
};

struct ce147_capture_size {
	unsigned int width;
	unsigned int height;
};

/* Image sizes */
const static struct ce147_capture_size ce147_image_sizes[] = {
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

#endif /* ifndef CE147_H */
