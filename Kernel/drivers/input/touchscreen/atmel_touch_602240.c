/*
 *  Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#define __CONFIG_SAMSUNG__
#define __CONFIG_SAMSUNG_DEBUG__
#define __VER_1_4__
#define __VER_1_5__

#ifndef __CONFIG_SAMSUNG__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#else	// __CONFIG_SAMSUNG__
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>

#include <plat/gpio.h>	//ryun
#include <plat/mux.h>	//ryun 

#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/mutex.h>

#include "atmel_touch.h"

//#define TEST_TOUCH_KEY_IN_ATMEL

#define  U16 	unsigned short int 
#define U8	__u8
#define u8	__u8
#define S16	signed short int
#define U16	unsigned short int
#define S32	signed long int
#define U32	unsigned long int
#define S64	signed long long int
#define U64	unsigned long long int
#define F32	float
#define F64	double

#define PRINT_FUNCTION_ENTER ;//printk("[TSP] %s() - start\n", __FUNCTION__);
#define PRINT_FUNCTION_EXIT ;//printk("[TSP] %s() - end\n", __FUNCTION__);

extern int i2c_tsp_sensor_init(void);
extern int i2c_tsp_sensor_read(u16 reg, u8 *read_val, unsigned int len );
extern int i2c_tsp_sensor_write(u16 reg, u8 *read_val, unsigned int len);
extern	void set_touch_i2c_gpio_init(void);
extern void set_touch_irq_gpio_init(void);
extern void set_touch_irq_gpio_disable(void);		// ryun 20091202
extern int  enable_irq_handler(void);
extern unsigned char get_touch_irq_gpio_value(void);
extern void clear_touch_history(void);

extern uint8_t read_boot_state(u8 *data);
extern uint8_t boot_unlock(void);	
extern uint8_t boot_write_mem(uint16_t ByteCount, unsigned char * Data );

extern void disable_tsp_irq(void);
extern void enable_tsp_irq(void);

unsigned int g_i2c_debugging_enable=0;

extern const unsigned char fw_bin_version;
extern const unsigned char fw_bin_build;
extern int fh_err_count;

uint8_t touch_state;

#endif	// __CONFIG_SAMSUNG__

/*------------------------------ definition block -----------------------------------*/

/* \brief Defines CHANGE line active mode. */
#define CHANGELINE_ASSERTED 0

/* This sets the I2C frequency to 400kHz (it's a feature in I2C driver that the
 * actual value needs to be double that). */
/*! \brief I2C bus speed. */
#define I2C_SPEED                   800000u

#define CONNECT_OK                  1u
#define CONNECT_ERROR               2u

#define READ_MEM_OK                 1u
#define READ_MEM_FAILED             2u

#define MESSAGE_READ_OK             1u
#define MESSAGE_READ_FAILED         2u

#define WRITE_MEM_OK                1u
#define WRITE_MEM_FAILED            2u

#define CFG_WRITE_OK                1u
#define CFG_WRITE_FAILED            2u

#define I2C_INIT_OK                 1u
#define I2C_INIT_FAIL               2u

#define CRC_CALCULATION_OK          1u
#define CRC_CALCULATION_FAILED      2u

#define ID_MAPPING_OK               1u
#define ID_MAPPING_FAILED           2u

#define ID_DATA_OK                  1u
#define ID_DATA_NOT_AVAILABLE       2u

/*! \brief Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   0u

/*! Address where object table starts at touch IC memory map. */
#define OBJECT_TABLE_START_ADDRESS      7U

/*! Size of one object table element in touch IC memory map. */
#define OBJECT_TABLE_ELEMENT_SIZE     	6U

/*! Offset to RESET register from the beginning of command processor. */
#define RESET_OFFSET		            0u

/*! Offset to BACKUP register from the beginning of command processor. */
#define BACKUP_OFFSET		1u

/*! Offset to CALIBRATE register from the beginning of command processor. */
#define CALIBRATE_OFFSET	2u

/*! Offset to REPORTALL register from the beginning of command processor. */
#define REPORTATLL_OFFSET	3u

/*! Offset to DEBUG_CTRL register from the beginning of command processor. */
#define DEBUG_CTRL_OFFSET	4u

#define DIAGNOSTIC_OFFSET	5u

enum driver_setup_t {DRIVER_SETUP_OK, DRIVER_SETUP_INCOMPLETE};



#define OBJECT_LIST__VERSION_NUMBER	0x10

#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMCONFIG_T18                         18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23	                      23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u

#define DEBUG_DIAGNOSTIC_T37	37u

#define Enable_global_interrupt()        
#define Disable_global_interrupt()        

/*------------------------------ type define block -----------------------------------*/
#ifndef __CONFIG_SAMSUNG__

typedef unsigned char           Bool; //!< Boolean.
typedef signed char             S8 ;  //!< 8-bit signed integer.
typedef unsigned char           U8 ;  //!< 8-bit unsigned integer.
typedef signed short int        S16;  //!< 16-bit signed integer.
typedef unsigned short int      U16;  //!< 16-bit unsigned integer.
typedef signed long int         S32;  //!< 32-bit signed integer.
typedef unsigned long int       U32;  //!< 32-bit unsigned integer.
typedef signed long long int    S64;  //!< 64-bit signed integer.
typedef unsigned long long int  U64;  //!< 64-bit unsigned integer.
typedef float                   F32;  //!< 32-bit floating-point number.
typedef double                  F64;  //!< 64-bit floating-point number.

typedef U8 uint8_t;
typedef S8 int8_t;

typedef U16 uint16_t;
typedef S16 int16_t;

typedef U32 uint32_t;
typedef S32 int32_t;

#else


#endif
/* Array of I2C addresses where we are trying to find the chip. */
uint8_t QT_i2c_address = 0x4A;


/*! ==GEN_COMMANDPROCESSOR_T6==
 The T6 CommandProcessor allows commands to be issued to the chip
 by writing a non-zero vale to the appropriate register.
*/
/*! \brief
 This structure is used to configure the commandprocessor and should be made
 accessible to the host controller.
*/
typedef struct
{
   uint8_t reset;       /*!< Force chip reset             */
   uint8_t backupnv;    /*!< Force backup to eeprom/flash */
   uint8_t calibrate;   /*!< Force recalibration          */
   uint8_t reportall;   /*!< Force all objects to report  */
   uint8_t debugctrl;   /*!< Turn on output of debug data */

   uint8_t diagnostic;  /*!< Controls the diagnostic object */
} __packed gen_commandprocessor_t6_config_t;

/*! ==GEN_POWERCONFIG_T7==
 The T7 Powerconfig controls the power consumption by setting the
 acquisition frequency.
*/
/*! \brief
 This structure is used to configure the powerconfig and should be made
 accessible to the host controller.
*/
typedef struct
{
   uint8_t idleacqint;    /*!< Idle power mode sleep length in ms           */
   uint8_t actvacqint;    /*!< Active power mode sleep length in ms         */
   uint8_t actv2idleto;   /*!< Active to idle power mode delay length in ms */
} __packed gen_powerconfig_t7_config_t;


/*! ==GEN_ACQUISITIONCONFIG_T8==
 The T8 AcquisitionConfig object controls how the device takes each
 capacitive measurement.
*/
/*! \brief
 This structure is used to configure the acquisitionconfig and should be made
 accessible to the host controller.
*/
typedef struct
{
   uint8_t chrgtime;          /*!< Burst charge time                      */
   uint8_t reserved;         /*!< Anti-touch drift compensation period   */
   uint8_t tchdrift;          /*!< Touch drift compensation period        */
   uint8_t driftst;           /*!< Drift suspend time                     */
   uint8_t tchautocal;        /*!< Touch automatic calibration delay in ms*/
   uint8_t sync;              /*!< Measurement synchronisation control    */

   uint8_t atchcalst;         /*!< recalibration suspend time after last detection */
   uint8_t atchcalsthr;       /*!< Anti-touch calibration suspend threshold */

} __packed gen_acquisitionconfig_t8_config_t;



/*! ==TOUCH_MULTITOUCHSCREEN_T9==
 The T9 Multitouchscreen is a multi-touch screen capable of tracking upto 8
 independent touches.
*/

/*! \brief
 This structure is used to configure the multitouchscreen and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Screen Configuration */
   uint8_t ctrl;            /*!< Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;         /*!< Object x start position on matrix  */
   uint8_t yorigin;         /*!< Object y start position on matrix  */
   uint8_t xsize;           /*!< Object x size (i.e. width)         */
   uint8_t ysize;           /*!< Object y size (i.e. height)        */

   /* Detection Configuration */
   uint8_t akscfg;          /*!< Adjacent key suppression config     */
   uint8_t blen;            /*!< Burst length for all object channels*/
   uint8_t tchthr;          /*!< Threshold for all object channels   */
   uint8_t tchdi;           /*!< Detect integration config           */

   uint8_t orientate;  /*!< Controls flipping and rotating of touchscreen
                        *   object */
   uint8_t mrgtimeout; /*!< Timeout on how long a touch might ever stay
                        *   merged - units of 0.2s, used to tradeoff power
                        *   consumption against being able to detect a touch
                        *   de-merging early */

   /* Position Filter Configuration */
   uint8_t movhysti;   /*!< Movement hysteresis setting used after touchdown */
   uint8_t movhystn;   /*!< Movement hysteresis setting used once dragging   */
   uint8_t movfilter;  /*!< Position filter setting controlling the rate of  */

   /* Multitouch Configuration */
   uint8_t numtouch;   /*!< The number of touches that the screen will attempt
                        *   to track */
   uint8_t mrghyst;    /*!< The hystersis applied on top of the merge threshold
                        *   to stop oscillation */
   uint8_t mrgthr;     /*!< The threshold for the point when two peaks are
                        *   considered one touch */

   uint8_t tchamphyst;    	/*!< TBD */

   /* Resolution Controls */
   uint16_t xrange;
   uint16_t yrange;
   uint8_t xloclip;
   uint8_t xhiclip;
   uint8_t yloclip;
   uint8_t yhiclip;

  /* edge correction controls */
  uint8_t xedgectrl;     /*!< LCMASK */
  uint8_t xedgedist;     /*!< LCMASK */
  uint8_t yedgectrl;     /*!< LCMASK */
  uint8_t yedgedist;     /*!< LCMASK */

  uint8_t jumplimit;

} __packed touch_multitouchscreen_t9_config_t;


/*! \brief
 This structure is used to configure a slider or wheel and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Key Array Configuration */
   uint8_t ctrl;           /*!< Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;        /*!< Object x start position on matrix  */
   uint8_t yorigin;        /*!< Object y start position on matrix  */
   uint8_t size;           /*!< Object x size (i.e. width)         */

   /* Detection Configuration */
   uint8_t akscfg;         /*!< Adjacent key suppression config     */
   uint8_t blen;           /*!< Burst length for all object channels*/
   uint8_t tchthr;         /*!< Threshold for all object channels   */
   uint8_t tchdi;          /*!< Detect integration config           */
   uint8_t reserved[2];    /*!< Spare x2 */
   uint8_t movhysti;       /*!< Movement hysteresis setting used after touchdown */
   uint8_t movhystn;       /*!< Movement hysteresis setting used once dragging */
   uint8_t movfilter;      /*!< Position filter setting controlling the rate of  */
} __packed touch_slider_wheel_t11_t12_t13_t14_config_t;


/*! ==TOUCH_KEYARRAY_T15==
 The T15 Keyarray is a key array of upto 32 keys
*/
/*! \brief
 This structure is used to configure the keyarry and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Key Array Configuration */
   uint8_t ctrl;               /*!< Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;           /*!< Object x start position on matrix  */
   uint8_t yorigin;           /*!< Object y start position on matrix  */
   uint8_t xsize;             /*!< Object x size (i.e. width)         */
   uint8_t ysize;             /*!< Object y size (i.e. height)        */

   /* Detection Configuration */
   uint8_t akscfg;             /*!< Adjacent key suppression config     */
   uint8_t blen;               /*!< Burst length for all object channels*/
   uint8_t tchthr;             /*!< Threshold for all object channels   */
   uint8_t tchdi;              /*!< Detect integration config           */
   uint8_t reserved[2];        /*!< Spare x2 */

} __packed touch_keyarray_t15_config_t;


/*! ==PROCI_LINEARIZATIONTABLE_T17==
The LinearizationTable operates upon either a multi touch screen and/or a single touch screen.
The linearization table is setup to remove non-linear touch sensor properties.
*/
/*! \brief
 This structure is used to configure the linearizationtable and should be made
 accessible to the host controller.
*/
typedef struct
{
   uint8_t  ctrl;
   uint16_t xoffset;
   uint8_t  xsegment[16];
   uint16_t yoffset;
   uint8_t  ysegment[16];

} __packed proci_linearizationtable_t17_config_t;

/*! ==SPT_COMCCONFIG_T18==
The communication configuration object adjust the communication behaviour of
the device.

*/
/*! \brief
This structure is used to configure the COMCCONFIG object and should be made
accessible to the host controller.
*/

typedef struct
{
    uint8_t  ctrl;
    uint8_t  cmd;
} __packed spt_comcconfig_t18_config_t;

/*! ==SPT_GPIOPWM_T19==
 The T19 GPIOPWM object provides input, output, wake on change and PWM support
for one port.
*/
/*! \brief
 This structure is used to configure the GPIOPWM object and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* GPIOPWM Configuration */
   uint8_t ctrl;             /*!< Main configuration field           */
   uint8_t reportmask;       /*!< Event mask for generating messages to
                              *   the host */
   uint8_t dir;              /*!< Port DIR register   */
   uint8_t pullup;           /*!< Port pull-up per pin enable register */
   uint8_t out;              /*!< Port OUT register*/
   uint8_t wake;             /*!< Port wake on change enable register  */
   uint8_t pwm;              /*!< Port pwm enable register    */
   uint8_t per;              /*!< PWM period (min-max) percentage*/
   uint8_t duty[4];          /*!< PWM duty cycles percentage */

} __packed spt_gpiopwm_t19_config_t;


/*! ==PROCI_GRIPFACESUPPRESSION_T20==
 The T20 GRIPFACESUPPRESSION object provides grip bounday suppression and
 face suppression
*/
/*! \brief
 This structure is used to configure the GRIPFACESUPPRESSION object and
 should be made accessible to the host controller.
*/
typedef struct
{
   uint8_t ctrl;
   uint8_t xlogrip;
   uint8_t xhigrip;
   uint8_t ylogrip;
   uint8_t yhigrip;
   uint8_t maxtchs;
   uint8_t reserved;
   uint8_t szthr1;
   uint8_t szthr2;
   uint8_t shpthr1;
   uint8_t shpthr2;

   uint8_t supextto;

} __packed proci_gripfacesuppression_t20_config_t;


/*! ==PROCG_NOISESUPPRESSION_T22==
 The T22 NOISESUPPRESSION object provides ferquency hopping acquire control,
 outlier filtering and grass cut filtering.
*/
/*! \brief
 This structure is used to configure the NOISESUPPRESSION object and
 should be made accessible to the host controller.
*/
typedef struct
{
   uint8_t ctrl;


   uint8_t reserved;

   uint8_t reserved1;
   int16_t gcaful;
   int16_t gcafll;


   uint8_t actvgcafvalid;        /* LCMASK */

   uint8_t noisethr;
   uint8_t reserved2;
   uint8_t freqhopscale;


   uint8_t freq[5u];             /* LCMASK ACMASK */
   uint8_t idlegcafvalid;        /* LCMASK */

} __packed procg_noisesuppression_t22_config_t;

/*! ==TOUCH_PROXIMITY_T23==
 The T23 Proximity is a proximity key made of upto 16 channels
*/
/*! \brief
 This structure is used to configure the prox object and should be made
 accessible to the host controller.
*/
typedef struct
{
   /* Prox Configuration */
   uint8_t ctrl;               /*!< ACENABLE LCENABLE Main configuration field           */

   /* Physical Configuration */
   uint8_t xorigin;           /*!< ACMASK LCMASK Object x start position on matrix  */
   uint8_t yorigin;           /*!< ACMASK LCMASK Object y start position on matrix  */
   uint8_t xsize;             /*!< ACMASK LCMASK Object x size (i.e. width)         */
   uint8_t ysize;             /*!< ACMASK LCMASK Object y size (i.e. height)        */
   uint8_t reserved_for_future_aks_usage;
   /* Detection Configuration */
   uint8_t blen;               /*!< ACMASK Burst length for all object channels*/
   uint16_t tchthr;             /*!< LCMASK Threshold    */
   uint8_t tchdi;              /*!< Detect integration config           */
   uint8_t average;            /*!< LCMASK Sets the filter length on the prox signal */
   uint16_t rate;               /*!< Sets the rate that prox signal must exceed */
   uint16_t mvdthr           /*!< movement detection threshold */
} __packed touch_proximity_t23_config_t;

/*! ==PROCI_ONETOUCHGESTUREPROCESSOR_T24==
 The T24 ONETOUCHGESTUREPROCESSOR object provides gesture recognition from
 touchscreen touches.
*/
/*! \brief
 This structure is used to configure the ONETOUCHGESTUREPROCESSOR object and
 should be made accessible to the host controller.
*/
typedef struct
{
   uint8_t  ctrl;
   uint8_t  numgest;
   uint16_t gesten;
   uint8_t  pressproc;
   uint8_t  tapto;
   uint8_t  flickto;
   uint8_t  dragto;
   uint8_t  spressto;
   uint8_t  lpressto;
   uint8_t  rptpressto;
   uint16_t flickthr;
   uint16_t dragthr;
   uint16_t tapthr;
   uint16_t throwthr;
} __packed proci_onetouchgestureprocessor_t24_config_t;


/*! ==SPT_SELFTEST_T25==
 The T25 SEFLTEST object provides self testing routines
*/
/*! \brief
 This structure is used to configure the SELFTEST object and
 should be made accessible to the host controller.
*/

/*! = signal limit test structure = */

typedef struct
{
   uint16_t upsiglim;              /* LCMASK */
   uint16_t losiglim;              /* LCMASK */

} __packed siglim_t;

typedef struct
{
   uint8_t  ctrl;
   uint8_t  cmd;
   uint16_t upsiglim;
   uint16_t losiglim;
} __packed spt_selftest_t25_config_t;


/*! ==PROCI_TWOTOUCHGESTUREPROCESSOR_T27==
 The T27 TWOTOUCHGESTUREPROCESSOR object provides gesture recognition from
 touchscreen touches.
*/
/*! \brief
 This structure is used to configure the TWOTOUCHGESTUREPROCESSOR object and
 should be made accessible to the host controller.
*/
typedef struct
{
   uint8_t  ctrl;          /*< Bit 0 = object enable, bit 1 = report enable */
   uint8_t  numgest;       /*< Runtime control for how many two touch gestures
                            *  to process */
    uint8_t reserved2;
    
    uint8_t gesten;        /*!< Control for turning particular gestures on or
                            *   off */
   uint8_t  rotatethr;
   uint16_t zoomthr;
//   uint8_t tcheventto;
//   uint8_t reserved2;
} __packed proci_twotouchgestureprocessor_t27_config_t;


/*! ==SPT_CTECONFIG_T28==
 The T28 CTECONFIG object provides controls for the CTE.
*/
/*! \brief
 This structure is used to configure the CTECONFIG object and
 should be made accessible to the host controller.
*/
typedef struct
{
   uint8_t ctrl;          /*!< Ctrl field reserved for future expansion */
   uint8_t cmd;           /*!< Cmd field for sending CTE commands */
   uint8_t mode;          /*!< CTE mode configuration field */
   uint8_t idlegcafdepth; /*!< If non-zero, this acts as the a global gcaf
                           *   number of averages when idle */
   uint8_t actvgcafdepth; /*!< LCMASK The global gcaf number of averages when active */					   	
   int8_t voltage; 
} __packed spt_cteconfig_t28_config_t;


typedef struct
{
	char * config_name;

	gen_powerconfig_t7_config_t power_config;
	gen_acquisitionconfig_t8_config_t acquisition_config;
	touch_multitouchscreen_t9_config_t touchscreen_config;
	touch_keyarray_t15_config_t keyarray_config;
	proci_gripfacesuppression_t20_config_t gripfacesuppression_config;
	proci_linearizationtable_t17_config_t linearization_config;
	spt_comcconfig_t18_config_t comc_config;
	spt_gpiopwm_t19_config_t gpiopwm_config;
	spt_selftest_t25_config_t selftest_config;
	procg_noisesuppression_t22_config_t noise_suppression_config;
	proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config;
	proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config;
	spt_cteconfig_t28_config_t cte_config;
	touch_proximity_t23_config_t proximity_config;
} all_config_setting;



/*! \brief Object table element struct. */
typedef struct
{
   uint8_t object_type;     /*!< Object type ID. */
   uint16_t i2c_address;    /*!< Start address of the obj config structure. */
   uint8_t size;            /*!< Byte length of the obj config structure -1.*/
   uint8_t instances;       /*!< Number of objects of this obj. type -1. */
   uint8_t num_report_ids;  /*!< The max number of touches in a screen,
                             *  max number of sliders in a slider array, etc.*/
} __packed object_t;



/*! \brief Info ID struct. */
typedef struct
{
   uint8_t family_id;            /* address 0 */
   uint8_t variant_id;           /* address 1 */

   uint8_t version;              /* address 2 */
   uint8_t build;                /* address 3 */

   uint8_t matrix_x_size;        /* address 4 */
   uint8_t matrix_y_size;        /* address 5 */

   /*! Number of entries in the object table. The actual number of objects 
    * can be different if any object has more than one instance. */
   uint8_t num_declared_objects; /* address 6 */
} __packed info_id_t;


/*! \brief Info block struct holding ID and object table data and their CRC sum.
 * 
 * Info block struct. Similar to one in info_block.h, but since 
 * the size of object table is not known beforehand, it's pointer to an
 * array instead of an array. This is not defined in info_block.h unless
 * we are compiling with IAR AVR or AVR32 compiler (__ICCAVR__ or __ICCAVR32__
 * is defined). If this driver is compiled with those compilers, the
 * info_block.h needs to be edited to not include that struct definition.
 * 
 * CRC is 24 bits, consisting of CRC and CRC_hi; CRC is the lower 16 bytes and
 * CRC_hi the upper 8. 
 * 
 */
typedef struct
{
    /*! Info ID struct. */
    info_id_t info_id;

    /*! Pointer to an array of objects. */
    object_t *objects;

    /*! CRC field, low bytes. */
    uint16_t CRC;

    /*! CRC field, high byte. */
    uint8_t CRC_hi;
} __packed info_block_t;


/*! \brief Struct holding the object type / instance info.
 * 
 * Struct holding the object type / instance info. An array of these maps
 * report id's to object type / instance (array index = report id).  Note
 * that the report ID number 0 is reserved.
 * 
 */
typedef struct
{
   uint8_t object_type; 	/*!< Object type. */
   uint8_t instance;   	    /*!< Instance number. */
} __packed report_id_map_t;

/*! Pointer to info block structure. */
static info_block_t *info_block;

/*! Pointer to report_id - > object type/instance map structure. */
static report_id_map_t *report_id_map;

/*! \brief Flag indicating whether there's a message read pending.
 * 
 * When the CHANGE line is asserted during the message read, the state change 
 * is possibly missed by the host monitoring the CHANGE line. The host 
 * should check this flag after a read operation, and if set, do a read.
 * 
 */
volatile uint8_t read_pending;

/*! Maximum report id number. */
static int max_report_id = 0;

/*! Maximum message length. */
uint8_t max_message_length;

/*! Message processor address. */
uint16_t message_processor_address;

/*! Command processor address. */
static uint16_t command_processor_address;


/*! Flag indicating if driver setup is OK. */
static enum driver_setup_t driver_setup = DRIVER_SETUP_INCOMPLETE;

/*! Pointer to message handler provided by main app. */
static void (*application_message_handler)(void);

/*! Message buffer holding the latest message received. */
uint8_t *atmel_msg;

/*! \brief The current address pointer. */
static U16 address_pointer;


unsigned char g_version=0, g_build = 0, qt60224_notfound_flag=1;
Atmel_model_type g_model = DEFAULT_MODEL;

all_config_setting config_normal = { "config_normal" , 0 };

uint8_t cal_check_flag = 0u;//20100208

#ifdef ENABLE_NOISE_TEST_MODE

#define TEST_POINT_NUM      7
typedef enum 
{
    QT_PAGE_UP         = 0x01,
    QT_PAGE_DOWN       = 0x02,
    QT_DELTA_MODE      = 0x10,
    QT_REFERENCE_MODE  = 0x11,
    QT_CTE_MODE        = 0x31 
}diagnostic_debug_command;

                                       //botton_right    botton_left  center  top_right  top_left  menu_key  back_key
unsigned char test_node[TEST_POINT_NUM] = {13,           21,          113,    193,       201,      203,      215}; //19 by 11

unsigned int return_refer_0, return_refer_1, return_refer_2, return_refer_3, return_refer_4;
unsigned int return_delta_0, return_delta_1, return_delta_2, return_delta_3, return_delta_4;
uint16_t diagnostic_addr;
#endif

static DECLARE_MUTEX(g_tsp_mutex);

/*------------------------------ functions prototype -----------------------------------*/
uint8_t init_touch_driver(uint8_t I2C_address, void (*handler)(void));
void message_handler(U8 *msg, U8 length);
uint8_t get_version(uint8_t *version);
uint8_t get_family_id(uint8_t *family_id);
uint8_t get_build_number(uint8_t *build);
uint8_t get_variant_id(uint8_t *variant);
void disable_changeline_int(void);
void enable_changeline_int(void);
uint8_t calculate_infoblock_crc(uint32_t *crc_pointer);
uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);
uint8_t type_to_report_id(uint8_t object_type, uint8_t instance);
uint8_t write_power_config(gen_powerconfig_t7_config_t cfg);
uint8_t get_max_report_id(void);
uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t cfg);
uint8_t write_multitouchscreen_config(uint8_t instance, touch_multitouchscreen_t9_config_t cfg);
uint16_t get_object_address(uint8_t object_type, uint8_t instance);
uint8_t write_linearization_config(uint8_t instance, proci_linearizationtable_t17_config_t cfg);
uint8_t write_twotouchgesture_config(uint8_t instance, proci_twotouchgestureprocessor_t27_config_t cfg);
uint8_t write_onetouchgesture_config(uint8_t instance, proci_onetouchgestureprocessor_t24_config_t cfg);
uint8_t write_gripsuppression_config(uint8_t instance, proci_gripfacesuppression_t20_config_t cfg);
uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg);
uint8_t write_selftest_config(uint8_t instance, spt_selftest_t25_config_t cfg);
uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg);
uint8_t write_comc_config(spt_comcconfig_t18_config_t cfg);
uint8_t write_gpiopwm_config(spt_gpiopwm_t19_config_t cfg);
uint8_t write_proximity_config(touch_proximity_t23_config_t cfg);
uint8_t diagnostic_chip(uint8_t mode);
uint8_t backup_config(void);
uint8_t reset_chip(void);
uint8_t calibrate_chip(void);
U8 read_changeline(void);
uint8_t write_keyarray_config(uint8_t instance, touch_keyarray_t15_config_t cfg);
uint8_t get_message(void);
U8 init_I2C(U8 I2C_address_arg);
uint8_t read_id_block(info_id_t *id);
U8 read_mem(U16 start, U8 size, U8 *mem);
U8 read_U16(U16 start, U16 *mem);
void init_changeline(void);
U8 write_mem(U16 start, U8 size, U8 *mem);
uint8_t get_object_size(uint8_t object_type);
uint8_t write_simple_config(uint8_t object_type, uint8_t instance, void *cfg);
uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2);
uint8_t read_simple_config(uint8_t object_type, uint8_t instance, void *cfg);	// ryun 20091202
uint8_t QT_Boot(uint8_t qt_force_update);


void read_all_register(void);

void get_default_power_config(gen_powerconfig_t7_config_t *power_config);
void get_default_acquisition_config(gen_acquisitionconfig_t8_config_t *acquisition_config);
void get_default_touchscreen_config(touch_multitouchscreen_t9_config_t *touchscreen_config);
void get_default_keyarray_config(touch_keyarray_t15_config_t *keyarray_config);
void get_default_linearization_config(proci_linearizationtable_t17_config_t *linearization_config);
void get_default_twotouch_gesture_config(proci_twotouchgestureprocessor_t27_config_t *twotouch_gesture_config );
void get_default_onetouch_gesture_config(proci_onetouchgestureprocessor_t24_config_t * onetouch_gesture_config);
void get_default_noise_suppression_config(procg_noisesuppression_t22_config_t * noise_suppression_config);
void get_default_selftest_config(spt_selftest_t25_config_t *selftest_config);
void get_default_gripfacesuppression_config(proci_gripfacesuppression_t20_config_t *gripfacesuppression_config);
void get_default_cte_config(spt_cteconfig_t28_config_t *cte_config);

void set_specific_power_config(gen_powerconfig_t7_config_t *power_config, Atmel_model_type model_type );
void set_specific_acquisition_config(gen_acquisitionconfig_t8_config_t *acquisition_config, Atmel_model_type model_type);
void set_specific_touchscreen_config(touch_multitouchscreen_t9_config_t *touchscreen_config, Atmel_model_type model_type);
void set_specific_keyarray_config(touch_keyarray_t15_config_t *keyarray_config, Atmel_model_type model_type);
void set_specific_linearization_config(proci_linearizationtable_t17_config_t *linearization_config, Atmel_model_type model_type);
void set_specific_twotouch_gesture_config(proci_twotouchgestureprocessor_t27_config_t *twotouch_gesture_config, Atmel_model_type model_type );
void set_specific_onetouch_gesture_config(proci_onetouchgestureprocessor_t24_config_t * onetouch_gesture_config, Atmel_model_type model_type);
void set_specific_noise_suppression_config(procg_noisesuppression_t22_config_t * noise_suppression_config, Atmel_model_type model_type);
void set_specific_selftest_config(spt_selftest_t25_config_t *selftest_config, Atmel_model_type model_type);
void set_specific_gripfacesuppression_config(proci_gripfacesuppression_t20_config_t *gripfacesuppression_config, Atmel_model_type model_type);
void set_specific_cte_config(spt_cteconfig_t28_config_t *cte_config, Atmel_model_type model_type);

static int firmware_ret_val = -1;
unsigned char firmware_602240[] = {
#include "Firmware16.h"
};

int is_suspend_state = 0; // to check suspend mode
int is_boot_state = 1; // to check boot mode
int ta_state = 0;
int set_tsp_for_ta_detect(int state);

extern void handle_multi_touch(uint8_t *atmel_msg);
extern void handle_keyarray(uint8_t * atmel_msg);
extern int touchscreen_get_tsp_int_num(void);
/*------------------------------ main block -----------------------------------*/

unsigned int qt_time_point=0;
unsigned int qt_time_diff=0;
unsigned int qt_timer_state =0;

static int check_abs_time(void)
{
	if (!qt_time_point)
		return 0;

	qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
	if(qt_time_diff > 0)
		return 1;
	else
		return 0;
	
}

void set_autocal(uint8_t value)
{
	static int g_value =0;
	if (down_interruptible(&g_tsp_mutex))
		return;

	if(g_value != value) {
		g_value = value;
		config_normal.acquisition_config.tchautocal = value;
		if (write_acquisition_config(config_normal.acquisition_config) != CFG_WRITE_OK)
		{
			/* "Acquisition config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			// MUST be fixed
		}
		if(value)
			printk(KERN_DEBUG "[TSP] autocalibration enabled\n");
		else
			printk(KERN_DEBUG "[TSP] autocalibration disabled\n");			
	}
	up(&g_tsp_mutex);
}

void force_disable_tsp_autocal()
{
	if((!is_boot_state && !is_suspend_state) && !qt60224_notfound_flag)
	{
		if (down_interruptible(&g_tsp_mutex))
			return;

		config_normal.acquisition_config.tchautocal = 0;
		if (write_acquisition_config(config_normal.acquisition_config) != CFG_WRITE_OK)
		{
			/* "Acquisition config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			// MUST be fixed
		}
		printk(KERN_DEBUG "[TSP] autocalibration force disabled\n");

		up(&g_tsp_mutex);
	}
}
EXPORT_SYMBOL(force_disable_tsp_autocal);

void bootcomplete(void)
{
	if(qt60224_notfound_flag == 1) return;
	is_boot_state = 0;
	set_tsp_for_ta_detect(ta_state);
	set_autocal(0);
	printk(KERN_DEBUG "[TSP] set boot threshold = %d\n", config_normal.touchscreen_config.tchthr);
}

struct work_struct autocal_work;
struct timer_list autocal_timer;

void disable_autocal(struct work_struct *work)
{
	if((!is_boot_state && !is_suspend_state) && !qt60224_notfound_flag)
	{
		set_autocal(0);
	}
}

static void autocal_timer_out (unsigned long v)
{
	schedule_work(&autocal_work);
	return;
}

void enable_autocal_timer(unsigned int value)
{
	if((!is_boot_state && !is_suspend_state) && !qt60224_notfound_flag)
	{
		set_autocal(value);
		mod_timer(&autocal_timer, jiffies + (3 * HZ));
	}
}

// ryun 20100208 
void check_chip_calibration(unsigned char one_touch_input_flag)
{

	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint16_t diag_address;
	uint8_t tch_ch = 0, atch_ch = 0;
	uint8_t check_mask;
	uint8_t i;
	uint8_t j;
	uint8_t x_line_limit;
	if (down_interruptible(&g_tsp_mutex))
		return;
	
	if(g_version >=0x16)
	{

		/* we have had the first touchscreen or face suppression message 
		 * after a calibration - check the sensor state and try to confirm if
		 * cal was good or bad */

		/* get touch flags from the chip using the diagnostic object */
		/* write command to command processor to get touch flags - 0xF3 Command required to do this */
		write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);
		/* get the address of the diagnostic object so we can get the data we need */
		diag_address = get_object_address(DEBUG_DIAGNOSTIC_T37,0);

		msleep(10); 

		/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
		memset( data_buffer , 0xFF, sizeof( data_buffer ) );

		/* wait for diagnostic object to update */
		while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
		{
			/* wait for data to be valid  */
			if(try_ctr > 10)
			{
				qt_timer_state = 0;
				/* Failed! */
				printk("[TSP] Diagnostic Data did not update!!\n");
				break;
			}
			msleep(2); 
			try_ctr++; /* timeout counter */
			read_mem(diag_address, 2,data_buffer);
			//printk("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
		}


		/* data is ready - read the detection flags */
		read_mem(diag_address, 82,data_buffer);

		/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

		/* count up the channels/bits if we recived the data properly */
		if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
		{

			/* mode 0 : 16 x line, mode 1 : 17 etc etc upto mode 4.*/
			x_line_limit = 16 + config_normal.cte_config.mode;
			if(x_line_limit > 20)
			{
				/* hard limit at 20 so we don't over-index the array */
				x_line_limit = 20;
			}

			/* double the limit as the array is in bytes not words */
			x_line_limit = x_line_limit << 1;

			/* count the channels and print the flags to the log */
			for(i = 0; i < x_line_limit; i+=2) /* check X lines - data is in words so increment 2 at a time */
			{
				/* print the flags to the log - only really needed for debugging */
				//printk("[TSP] Detect Flags X%d, %x%x, %x%x \n", i>>1,data_buffer[3+i],data_buffer[2+i],data_buffer[43+i],data_buffer[42+i]);

				/* count how many bits set for this row */
				for(j = 0; j < 8; j++)
				{
					/* create a bit mask to check against */
					check_mask = 1 << j;

					/* check detect flags */
					if(data_buffer[2+i] & check_mask)
					{
						tch_ch++;
					}
					if(data_buffer[3+i] & check_mask)
					{
						tch_ch++;
					}

					/* check anti-detect flags */
					if(data_buffer[42+i] & check_mask)
					{
						atch_ch++;
					}
					if(data_buffer[43+i] & check_mask)
					{
						atch_ch++;
					}
				}
			}


			/* print how many channels we counted */
			printk(KERN_DEBUG "[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);

			/* send page up command so we can detect when data updates next time,
			 * page byte will sit at 1 until we next send F3 command */
			data_byte = 0x01;
			write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &data_byte);

			/* process counters and decide if we must re-calibrate or if cal was good */      
			if((tch_ch) && (atch_ch == 0))
			{
				if(!check_abs_time())
				{
					qt_time_diff=501;
				}

				if(qt_timer_state == 1)
				{
					if(qt_time_diff > 500)
					{
						printk(KERN_DEBUG "[TSP] calibration was good\n");
						/* cal was good - don't need to check any more */
						cal_check_flag = 0;
						qt_timer_state = 0;
						qt_time_point = 0;

						printk(KERN_DEBUG "[TSP] reset acq atchcalst=%d, atchcalsthr=%d\n", config_normal.acquisition_config.atchcalst, config_normal.acquisition_config.atchcalsthr );
							/* Write normal acquisition config back to the chip. */
							if (write_acquisition_config(config_normal.acquisition_config) != CFG_WRITE_OK)
						{
							/* "Acquisition config write failed!\n" */
							printk("\n[TSP][ERROR] line : %d\n", __LINE__);
							// MUST be fixed
						}
					}
					else 
					{
						cal_check_flag = 1;
					}
				}
				else
				{
					qt_timer_state = 1;
					qt_time_point = jiffies_to_msecs(jiffies);
					cal_check_flag = 1;
				}
			}
			else if(atch_ch >= 6)
			{
				printk("[TSP] calibration was bad\n");
				/* cal was bad - must recalibrate and check afterwards */
				calibrate_chip();
				qt_timer_state = 0;
				qt_time_point = 0;
			}
			else
			{
				printk(KERN_DEBUG "[TSP] calibration was not decided yet\n");
				/* we cannot confirm if good or bad - we must wait for next touch 
				 * message to confirm */
				cal_check_flag = 1;
				qt_timer_state= 0;
				qt_time_point = 0;
			}
		}
	}
	up(&g_tsp_mutex);
}


void restore_power_config(void)
{
	//printk("[ATMEL] restore_power_config \n");

	gen_powerconfig_t7_config_t power_config = {0};
		/* Set power config. */
	/* Set Idle Acquisition Interval to 32 ms. */
	power_config.idleacqint = config_normal.power_config.idleacqint;
	/* Set Active Acquisition Interval to 16 ms. */
	power_config.actvacqint = config_normal.power_config.actvacqint; 
	/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
	power_config.actv2idleto = config_normal.power_config.actv2idleto; 

		/* Write power config to chip. */
	if (write_power_config(power_config) != CFG_WRITE_OK)
	{
		/* "Power config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return;
	}

}

void restore_acquisition_config(void)
{
	gen_acquisitionconfig_t8_config_t acquisition_config={0};
	acquisition_config.chrgtime = config_normal.acquisition_config.chrgtime; 
	acquisition_config.tchdrift = config_normal.acquisition_config.tchdrift; 
	acquisition_config.driftst = config_normal.acquisition_config.driftst; 
	acquisition_config.tchautocal   = config_normal.acquisition_config.tchautocal; 
      	acquisition_config.atchcalst = config_normal.acquisition_config.atchcalst; 
      acquisition_config.atchcalsthr = config_normal.acquisition_config.atchcalsthr; 

	/* Write acquisition config to chip. */
	if (write_acquisition_config(acquisition_config) != CFG_WRITE_OK)
	{
		/* "Acquisition config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return;
	}    	

}
int atmel_suspend(void)
{
	if (down_interruptible(&g_tsp_mutex))
		return -ERESTARTSYS;

	is_suspend_state = 1; // set suspend state
	
	gen_powerconfig_t7_config_t power_config = {0};
		/* Set power config. */
	/* Set Idle Acquisition Interval to 32 ms. */
	power_config.idleacqint = 0;
	/* Set Active Acquisition Interval to 16 ms. */
	power_config.actvacqint = 0;
	/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
	power_config.actv2idleto = 20;

		/* Write power config to chip. */
	if (write_power_config(power_config) != CFG_WRITE_OK)
	{
		/* "Power config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
		up(&g_tsp_mutex);
		return -1;
	}
	up(&g_tsp_mutex);

	msleep(100);		
	disable_tsp_irq();
	clear_touch_history();
	qt_timer_state = 0;
	qt_time_point = 0;
	return 0;
}

int atmel_resume(void)
{
	restore_power_config();
	calibrate_chip();

	if(is_suspend_state==2) {
		set_tsp_for_ta_detect(ta_state);
	}

	is_suspend_state = 0;
	return 0;
}

int set_all_config(all_config_setting config)
    {

	/* Write power config to chip. */
	if (write_power_config(config.power_config) != CFG_WRITE_OK)
	{
		/* "Power config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
	}

	/* Write acquisition config to chip. */
	if (write_acquisition_config(config.acquisition_config) != CFG_WRITE_OK)
	{
		/* "Acquisition config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
	}

	/* Write touchscreen (1st instance) config to chip. */
	if (write_multitouchscreen_config(0, config.touchscreen_config) != CFG_WRITE_OK)
	{
		/* "Multitouch screen config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
	}

	/* Write key array (1st instance) config to chip. */
	if (write_keyarray_config(0, config.keyarray_config) != CFG_WRITE_OK)
	{
		/* "Key array config write failed!\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
	}

	/* Write linearization table config to chip. */
	if (get_object_address(PROCI_LINEARIZATIONTABLE_T17, 0) != OBJECT_NOT_FOUND)
	{
		if (write_linearization_config(0, config.linearization_config) != CFG_WRITE_OK)
		{
			/* "Linearization config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write grip suppression config to chip. */
	if (get_object_address(PROCI_GRIPFACESUPPRESSION_T20, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gripsuppression_config(0, config.gripfacesuppression_config) != CFG_WRITE_OK)
		{
			/* "Grip suppression config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

    /* Write Noise suppression config to chip. */
    if (get_object_address(PROCG_NOISESUPPRESSION_T22, 0) != OBJECT_NOT_FOUND)
    {
        if (write_noisesuppression_config(0,config.noise_suppression_config) != CFG_WRITE_OK)
        {
            printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
        }
    }

	/* Write one touch gesture config to chip. */
	if (get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 0) != OBJECT_NOT_FOUND)
	{
		if (write_onetouchgesture_config(0, config.onetouch_gesture_config) != CFG_WRITE_OK)
		{
			/* "One touch gesture config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write twotouch gesture config to chip. */
	if (get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 0) != OBJECT_NOT_FOUND)
	{
		if (write_twotouchgesture_config(0, config.twotouch_gesture_config) != CFG_WRITE_OK)
		{
			/* "Two touch gesture config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write CTE config to chip. */
	if (get_object_address(SPT_CTECONFIG_T28, 0) != OBJECT_NOT_FOUND)
	{
		if (write_CTE_config(config.cte_config) != CFG_WRITE_OK)
		{
			/* "CTE config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write comc config to chip. */
	if (get_object_address(SPT_COMCONFIG_T18, 0) != OBJECT_NOT_FOUND)
	{
		if (write_comc_config(config.comc_config) != CFG_WRITE_OK)
		{
			/* "comc config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write gpiopwm config to chip. */
	if (get_object_address(SPT_GPIOPWM_T19, 0) != OBJECT_NOT_FOUND)
	{
		if (write_gpiopwm_config(config.gpiopwm_config) != CFG_WRITE_OK)
		{
			/* "gpiopwm config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write proximity config to chip. */
	if (get_object_address(TOUCH_PROXIMITY_T23, 0) != OBJECT_NOT_FOUND)
	{
		if (write_proximity_config(config.proximity_config) != CFG_WRITE_OK)
		{
			/* "comc proximity write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	/* Write selftest config to chip. */
	if (get_object_address(SPT_SELFTEST_T25, 0) != OBJECT_NOT_FOUND)
	{
		if (write_selftest_config(0, config.selftest_config) != CFG_WRITE_OK)
		{
			/* "selftest config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return -1;
		}
	}

	//mdelay(200);
// ]] ryun 20091202 for reset
// [[ ryun 20091202 for cfg error start


// This routine is dependent on QC Driver IC version.
// And this is for additional setting configuration value after setting CTE mode,
// bacuase CTE mode and xsize and ysize value can't be set at the same time.
// so please don't change place this function is called
    if(g_version < 0x15)
    {
        // default value
    	config.touchscreen_config.xsize = 18;
    	config.touchscreen_config.ysize = 12;

        // model specific value
        switch(g_model)
        {
            case LATONA:
            {
            }
            break;
            break;
        }    
        
    	/* Write touchscreen (1st instance) config to chip. */
    	if (write_multitouchscreen_config(0, config.touchscreen_config) != CFG_WRITE_OK)
    	{
    		/* "Multitouch screen config write failed!\n" */
    		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
    			return -1;
    	}

    	/* Backup settings to NVM. */
//    	if (backup_config() != WRITE_MEM_OK)
//    	{
//    		/* "Failed to backup, exiting...\n" */
//    		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
//    		return ;
//    	}
//    	mdelay(100);
    //	read_simple_config(28u, 0, temp);	// ryun 20091202

    	/* Calibrate the touch IC. */
//    	if (calibrate_chip() != WRITE_MEM_OK)
 //   	{
 //   		/* "Failed to calibrate, exiting...\n" */
//    		return ;
//    	}
//    	mdelay(100);
    }
	return 1;
	
}
void atmel_touch_probe(void)
{
   U8 touch_chip_found = 0;
   U8 report_id, MAX_report_ID;
   U8 object_type, instance;
   U32 crc;//, stored_crc;
   uint8_t chip_reset;
   int count;
   int i;
   U8 version, family_id, variant, build;
#if 0 
	/* Power config settings. */
	gen_powerconfig_t7_config_t power_config = {0};

	/* Acquisition config. */
	gen_acquisitionconfig_t8_config_t acquisition_config = {0};

	/* Multitouch screen config. */
	touch_multitouchscreen_t9_config_t touchscreen_config = {0};

	/* Key array config. */
	touch_keyarray_t15_config_t keyarray_config = {0};

	/* Grip / face suppression config. */
	proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};

	/* Grip / face suppression config. */
	proci_linearizationtable_t17_config_t linearization_config = {0};

	/* Selftest config. */
	spt_selftest_t25_config_t selftest_config = {0};

	/* Noise suppression config. */
	procg_noisesuppression_t22_config_t noise_suppression_config = {0};

	/* One-touch gesture config. */
	proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config = {0};

	/* Two-touch gesture config. */
	proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config = {0};

	/* CTE config. */
	spt_cteconfig_t28_config_t cte_config = {0};
#endif 	
#ifdef __CONFIG_SAMSUNG__
	set_touch_irq_gpio_init();
#endif

	if (init_touch_driver(QT_i2c_address, (void *) &message_handler) == DRIVER_SETUP_OK)
	{
		/* "\nTouch device found\n" */
		printk(KERN_DEBUG "[TSP] Touch device found\n");
    }
    else
    {
		/* "\nTouch device NOT found\n" */
		printk("[TSP][ERROR] Touch device NOT found\n");
		return ;
    }
    
	/* Get and show the version information. */
	get_family_id(&family_id);
	get_variant_id(&variant);

	get_version(&g_version);

	get_build_number(&g_build);
	build = g_build;

   /* Disable interrupts from touch chip during config. We might miss initial
    * calibration/reset messages, but the text we are ouputting here doesn't
    * get cluttered with touch chip messages. */
	disable_changeline_int();	// ryun !!! ???
#ifndef	__CONFIG_SAMSUNG__
	Enable_global_interrupt(); 	
#endif
//	printk("\n[TSP] a-2\n");

	if(calculate_infoblock_crc(&crc) != CRC_CALCULATION_OK)
	{
		/* "Calculating CRC failed, skipping check!\n" */
		printk("[TSP][ERROR] line : %d\n", __LINE__);
			return;
	}
#ifndef	__CONFIG_SAMSUNG__
	Disable_global_interrupt();
#endif
//	printk("\n[TSP] b\n");


	/* Test the report id to object type / instance mapping: get the maximum
    * report id and print the report id map. */
   MAX_report_ID = get_max_report_id();
   for (report_id = 1; report_id <= MAX_report_ID; report_id++)
   {
      object_type = report_id_to_type(report_id, &instance);
   }

   /* Find the report ID of the first key array (instance 0). */
   report_id = type_to_report_id(TOUCH_KEYARRAY_T15, 0);
//   printk("\n[TSP] c\n");


    get_default_power_config(&config_normal.power_config);
    set_specific_power_config(&config_normal.power_config, g_model);

    get_default_acquisition_config(&config_normal.acquisition_config);
    set_specific_acquisition_config(&config_normal.acquisition_config, g_model);

    get_default_touchscreen_config(&config_normal.touchscreen_config);
    set_specific_touchscreen_config(&config_normal.touchscreen_config, g_model);

    get_default_keyarray_config(&config_normal.keyarray_config);
    set_specific_keyarray_config(&config_normal.keyarray_config, g_model);

    get_default_linearization_config(&config_normal.linearization_config);
    set_specific_linearization_config(&config_normal.linearization_config, g_model);

    get_default_twotouch_gesture_config(&config_normal.twotouch_gesture_config);
    set_specific_twotouch_gesture_config(&config_normal.twotouch_gesture_config, g_model);

    get_default_onetouch_gesture_config(&config_normal.onetouch_gesture_config);
    set_specific_onetouch_gesture_config(&config_normal.onetouch_gesture_config, g_model);

    get_default_noise_suppression_config(&config_normal.noise_suppression_config);
    set_specific_noise_suppression_config(&config_normal.noise_suppression_config, g_model);

    get_default_selftest_config(&config_normal.selftest_config);
    set_specific_selftest_config(&config_normal.selftest_config, g_model);

    get_default_gripfacesuppression_config(&config_normal.gripfacesuppression_config);
    set_specific_gripfacesuppression_config(&config_normal.gripfacesuppression_config, g_model);

    get_default_cte_config(&config_normal.cte_config);
    set_specific_cte_config(&config_normal.cte_config, g_model);

	set_all_config(config_normal);

	msleep(100);

	/* Backup settings to NVM. */
	if (backup_config() != WRITE_MEM_OK)
	{
		/* "Failed to backup, exiting...\n" */
		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
		return ;
	}
	msleep(100);

	chip_reset = 0;

	for(i = 0; i < max_report_id; i++)
	{
		touch_state = 0;
		if(read_changeline() == CHANGELINE_ASSERTED) 
			get_message();
		
		if(touch_state & 0x08)
		{
			chip_reset = 1;
		}
	}
	
	if(chip_reset == 1)
	{
		if (reset_chip() != WRITE_MEM_OK)
		{
			/* "Failed to reset, exiting...\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return ;
		}
		msleep(100);
	}
	else 
	{
		if(calibrate_chip() != WRITE_MEM_OK)
		{
			/* "Failed to calibrate, exiting...\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
			return ;
		}
	}

   /* Make sure that if the CHANGE line was asserted at the beginning, 
    * before the interrupt handler was monitoring it, we do a read. */



/*
   
	if (read_changeline() == CHANGELINE_ASSERTED)
	{
        count = 0;
    	while(count < 100 && (read_changeline() == CHANGELINE_ASSERTED) )
    	{
            mdelay(1);
		touch_state = 0;
            printk(" irq line is asserted\n");
    		get_message();
    		count++;
    	}

        if( count < 100 )
        {
            printk("[TSP] initialization is finishded normally, count = %d\n",count);
        }
        else
        {
            printk("[TSP] initialization is finishded wronglly, count = %d\n", count);        
        }
    }
    */
	enable_changeline_int();	// ryun 20091203 
	qt60224_notfound_flag = 0;

	if(is_boot_state) {
		set_tsp_for_ta_detect(1);
		set_autocal(10); // holding 2 seconds on touch point makes calibration automatically
		INIT_WORK(&autocal_work, disable_autocal);
	  	init_timer(&autocal_timer);
	  	autocal_timer.data = 0; 
  		autocal_timer.function = autocal_timer_out;
	}
}
/*------------------------------ Sub functions -----------------------------------*/
/*!
  \brief Initializes touch driver.
  
  This function initializes the touch driver: tries to connect to given 
  address, sets the message handler pointer, reads the info block and object
  table, sets the message processor address etc.
  
  @param I2C_address is the address where to connect.
  @param (*handler) is a pointer to message handler function.
  @return DRIVER_SETUP_OK if init successful, DRIVER_SETUP_FAILED 
          otherwise.
  */
uint8_t init_touch_driver(uint8_t I2C_address, void (*handler)(void))
{
	printk(KERN_DEBUG "[TSP] %s() - start\n", __FUNCTION__);
   int i;
   
   int current_report_id = 0;
   
   uint8_t tmp;
   uint16_t current_address;
   uint16_t crc_address;
   object_t *object_table;
   info_id_t *id;
   
   uint32_t *CRC;
   
   uint8_t status;
//   U8 temp_data8;
//   U16 temp_data16;

#if 0 // __CONFIG_SAMSUNG_DEBUG__
	uint16_t readdate;
#endif

   /* Set the message handler function pointer. */ 
   application_message_handler = handler;
   
   /* Try to connect. */
 PRINT_FUNCTION_ENTER;
   if(init_I2C(I2C_address) != CONNECT_OK)
   {
   	printk("[TSP][ERROR] 1\n");
	   return(DRIVER_SETUP_INCOMPLETE);
   }
   
   
   /* Read the info block data. */
   id = (info_id_t *) kmalloc(sizeof(info_id_t), GFP_KERNEL | GFP_ATOMIC);
   if (id == NULL)
   {
      return(DRIVER_SETUP_INCOMPLETE);
   }

#if 0 // def __CONFIG_SAMSUNG_DEBUG__
	   printk("[TSP] u16 size = %d u8 size = %d \n", sizeof(u16), sizeof(u8) );

	   status = read_mem(374, 1, (void *) &id->family_id);
	   if (status != READ_MEM_OK)
	   {
		   return(status);
	   }else
	   {
		   printk("[TSP] family_id = 0x%x\n\n", id->family_id);
	   }
	   return(DRIVER_SETUP_INCOMPLETE);
#endif

#if 0 //def __CONFIG_SAMSUNG_DEBUG__
	read_U16(4, &readdate);
	printk("[TSP] readdate = 0x%x \n", readdate);
	 return(DRIVER_SETUP_INCOMPLETE);
#endif

   if (read_id_block(id) != 1)
   {
   	printk("[TSP][ERROR] 2\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }
   
   /* Read object table. */

   object_table = (object_t *) kmalloc(id->num_declared_objects * sizeof(object_t), GFP_KERNEL | GFP_ATOMIC);
   if (object_table == NULL)
   {
   	printk("[TSP][ERROR] 3\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }


   /* Reading the whole object table block to memory directly doesn't work cause sizeof object_t
      isn't necessarily the same on every compiler/platform due to alignment issues. Endianness
      can also cause trouble. */
   
   current_address = OBJECT_TABLE_START_ADDRESS;
   
   
   for (i = 0; i < id->num_declared_objects; i++)
   {
//   	printk("[TSP] === try get object[%d] current_address = 0x%x ===\n", i, current_address);

//	  printk("[TSP] 1- &object_table[i].object_type = 0x%p 0x%x\n ", &(object_table[i].object_type), &(object_table[i].object_type));
      status = read_mem(current_address, 1, &(object_table[i]).object_type);
//	  printk("[TSP] 2- &object_table[i].object_type = 0x%p 0x%x\n ", &(object_table[i].object_type), &(object_table[i].object_type));
      if (status != READ_MEM_OK)
      {
      	printk("[TSP][ERROR] 4\n");
         return(DRIVER_SETUP_INCOMPLETE);
      }else
      	{
//      		printk("[TSP] 4 : object_table[i].object_type = %d\n", object_table[i].object_type);
      	}
      current_address++;
      
      status = read_U16(current_address, &object_table[i].i2c_address);
      if (status != READ_MEM_OK)
      {
	  printk("[TSP][ERROR] 5\n");
         return(DRIVER_SETUP_INCOMPLETE);
      }else
      	{
//      		printk("[TSP] 5 : object_table[i].i2c_address=%d(0x%x)\n", object_table[i].i2c_address, object_table[i].i2c_address);
      	}
      current_address += 2;

      status = read_mem(current_address, 1, (U8*)&object_table[i].size);
//	printk("[TSP] object_table[i].size=%d \n", object_table[i].size);
      if (status != READ_MEM_OK)
      {
      	printk("[TSP][ERROR] 6\n");
         return(DRIVER_SETUP_INCOMPLETE);
      }else
      	{
//      		printk("[TSP] 6 : object_table[i].size = %d\n", object_table[i].size);
      	}
      current_address++;
      
      status = read_mem(current_address, 1, &object_table[i].instances);
      if (status != READ_MEM_OK)
      {
      	printk("[TSP][ERROR] 7\n");
         return(DRIVER_SETUP_INCOMPLETE);
      }else
      	{
//      		printk("[TSP] 7 : object_table[i].instances = %d\n", object_table[i].instances);
      	}
      current_address++;
      
      status = read_mem(current_address, 1, &object_table[i].num_report_ids);
      if (status != READ_MEM_OK)
      {
      	printk("[TSP][ERROR] 8\n");
         return(DRIVER_SETUP_INCOMPLETE);
      }else
      	{
//      		printk("[TSP] 8 : object_table[i].num_report_ids = %d\n", object_table[i].num_report_ids);
      	}
      current_address++;
      
      max_report_id += object_table[i].num_report_ids;
      
      /* Find out the maximum message length. */
      if (object_table[i].object_type == GEN_MESSAGEPROCESSOR_T5)
      {
         max_message_length = object_table[i].size + 1;
      }
//	  printk("\n");
   }//  for (i = 0; i < id->num_declared_objects; i++)
   
   /* Check that message processor was found. */
   if (max_message_length == 0)
   {
   	printk("[TSP][ERROR] 9\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }
   
   /* Read CRC. */

   CRC = (uint32_t *) kmalloc(sizeof(info_id_t), GFP_KERNEL | GFP_ATOMIC);
   if (CRC == NULL)
   {
   	printk("[TSP][ERROR] 10\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }

   
  
   info_block = kmalloc(sizeof(info_block_t), GFP_KERNEL | GFP_ATOMIC);
   if (info_block == NULL)
   {
	   printk("err\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }
   

   info_block->info_id = *id;
   
   info_block->objects = object_table;

   crc_address = OBJECT_TABLE_START_ADDRESS + 
                 id->num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE;
   
   status = read_mem(crc_address, 1u, &tmp);
   if (status != READ_MEM_OK)
   {
   	printk("[TSP][ERROR] 11\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }
   info_block->CRC = tmp;
   
   status = read_mem(crc_address + 1u, 1u, &tmp);
   if (status != READ_MEM_OK)
   {
   	printk("[TSP][ERROR] 12\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }
   info_block->CRC |= (tmp << 8u);
   
   status = read_mem(crc_address + 2, 1, &info_block->CRC_hi);
   if (status != READ_MEM_OK)
   {
   	printk("[TSP][ERROR] 13\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }

   /* Store message processor address, it is needed often on message reads. */
   message_processor_address = get_object_address(GEN_MESSAGEPROCESSOR_T5, 0);
   if (message_processor_address == 0)
   {
   	printk("[TSP][ERROR] 14 !!\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }

   /* Store command processor address. */
   command_processor_address = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
   if (command_processor_address == 0)
   {
   	printk("[TSP][ERROR] 15\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }

   atmel_msg = kmalloc(max_message_length, GFP_KERNEL | GFP_ATOMIC);
   if (atmel_msg == NULL)
   {
   	printk("[TSP][ERROR] 16\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }
 
   /* Allocate memory for report id map now that the number of report id's 
    * is known. */

   report_id_map = kmalloc(sizeof(report_id_map_t) * max_report_id, GFP_KERNEL | GFP_ATOMIC);
   
   if (report_id_map == NULL)
   {
   	printk("[TSP][ERROR] 17\n");
      return(DRIVER_SETUP_INCOMPLETE);
   }

   
   /* Report ID 0 is reserved, so start from 1. */
      
   report_id_map[0].instance = 0;
   report_id_map[0].object_type = 0;
   current_report_id = 1;
   
   for (i = 0; i < id->num_declared_objects; i++)
   {
       if (object_table[i].num_report_ids != 0)
       {
          int instance;
          for (instance = 0; 
               instance <= object_table[i].instances; 
               instance++)
          {
             int start_report_id = current_report_id;
             for (; 
                  current_report_id < 
                     (start_report_id + object_table[i].num_report_ids);
                  current_report_id++)
             {
                report_id_map[current_report_id].instance = instance;
                report_id_map[current_report_id].object_type = 
                   object_table[i].object_type;
             }
          }
       }
   }
   driver_setup = DRIVER_SETUP_OK;
   
   /* Initialize the pin connected to touch ic pin CHANGELINE to catch the
    * falling edge of signal on that line. */
   init_changeline();
   
   return(DRIVER_SETUP_OK);
}

void read_all_register(void)
{
	U16 addr = 0;
	U8 msg;
	U16 calc_addr = 0;

//	printk("[TSP] read all register start !! \n\n%2d : ", addr+1);
	for(addr = 0 ; addr < 1273 ; addr++)
	{
//		calc_addr = (addr & 0x00ff) << 8 | (addr  & 0xff00) >> 8;
		calc_addr = addr;


		if(read_mem(addr, 1, &msg) == READ_MEM_OK)
		{
			printk("(0x%2x)", msg);
			if( (addr+1) % 10 == 0)
			{
				printk("\n");
				printk("%2d : ", addr+1);
			}

//			if(get_touch_irq_gpio_value() == 1)
//				printk("\n\n[TSP] find !!! 0x%x \n\n", (int)addr);
		}else
		{
			printk("\n\n[TSP][ERROR] %s() read fail !! \n", __FUNCTION__);
		}
	}
//	printk("\n\n[TSP] %s() all register read done !! \n", __FUNCTION__);
}

/*!
 * \brief Reads message from the message processor.
 * 
 * This function reads a message from the message processor and calls
 * the message handler function provided by application earlier.
 *
 * @return MESSAGE_READ_OK if driver setup correctly and message can be read 
 * without problems, or MESSAGE_READ_FAILED if setup not done or incorrect, 
 * or if there's a previous message read still ongoing (this can happen if
 * we are both polling the CHANGE line and monitoring it with interrupt that
 * is triggered by falling edge).
 * 
 */

uint8_t get_message(void)
{
  static volatile uint8_t read_in_progress;
  //unsigned long x, y;
  //unsigned int x480, y800, press;
  uint8_t ret_val = MESSAGE_READ_FAILED;
  //U8 mydata;


    uint8_t object_type, instance;

   
   if (read_in_progress == 0)
   {
      read_in_progress = 1;
      
      if (driver_setup == DRIVER_SETUP_OK)
      {

//	g_i2c_debugging_enable = 1;
         if(read_mem(message_processor_address, max_message_length, atmel_msg) == READ_MEM_OK)
         {
	      object_type = report_id_to_type(atmel_msg[0], &instance);


		switch(object_type)
			{
			case GEN_COMMANDPROCESSOR_T6:
				touch_state = atmel_msg[1];
			case TOUCH_MULTITOUCHSCREEN_T9:
			case PROCI_GRIPFACESUPPRESSION_T20:
			case TOUCH_KEYARRAY_T15:
			case SPT_GPIOPWM_T19:
			case PROCI_ONETOUCHGESTUREPROCESSOR_T24:
			case PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
			case SPT_SELFTEST_T25:
			case SPT_CTECONFIG_T28:
			case PROCG_NOISESUPPRESSION_T22:
			default:
				printk(KERN_DEBUG "[TSP] 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", atmel_msg[0], atmel_msg[1],atmel_msg[2], 
					    atmel_msg[3], atmel_msg[4],atmel_msg[5], atmel_msg[6], atmel_msg[7], atmel_msg[8]);
				break;
		};
            ret_val = MESSAGE_READ_OK;
         }
//	g_i2c_debugging_enable = 0;
      }
      read_in_progress = 0;
   }
   return (ret_val);
}

/*!
 * \brief Resets the chip.
 * 
 *  This function will send a reset command to touch chip.
 *
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */
uint8_t reset_chip(void)
{
   	uint8_t data = 1u;
	int ret = WRITE_MEM_OK;

	write_mem(command_processor_address + RESET_OFFSET, 1, &data);
	/* set flag for calibration lockup recovery if cal command was successful */
	if(ret == WRITE_MEM_OK)
	{ 
		/* set flag to show we must still confirm if calibration was good or bad */
		cal_check_flag = 1u;
	}
	//		msleep(65);
	return ret;
}


/*!
 * \brief Calibrates the chip.
 * 
 * This function will send a calibrate command to touch chip.
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */
uint8_t calibrate_chip(void)
{
   uint8_t data = 1u;
#if 1
	int ret = WRITE_MEM_OK;
	uint8_t atchcalst, atchcalsthr;
   
   if((g_version >= 0x16)&&(cal_check_flag == 0))
   {
	    /* change calibration suspend settings to zero until calibration confirmed good */
	    /* store normal settings */
	    atchcalst = config_normal.acquisition_config.atchcalst;
	    atchcalsthr = config_normal.acquisition_config.atchcalsthr;
	        
	    /* resume calibration must be performed with zero settings */
	    config_normal.acquisition_config.atchcalst = 0;
	    config_normal.acquisition_config.atchcalsthr = 0; 
	          
		printk(KERN_DEBUG "[TSP] reset acq atchcalst=%d, atchcalsthr=%d\n", config_normal.acquisition_config.atchcalst, config_normal.acquisition_config.atchcalsthr );
	        
		/* Write temporary acquisition config to chip. */
		if (write_acquisition_config(config_normal.acquisition_config) != CFG_WRITE_OK)
		{
			/* "Acquisition config write failed!\n" */
			printk("\n[TSP][ERROR] line : %d\n", __LINE__);
	        ret = WRITE_MEM_FAILED; /* calling function should retry calibration call */
		}

	    /* restore settings to the local structure so that when we confirm the 
	     * cal is good we can correct them in the chip */
	    /* this must be done before returning */
	    config_normal.acquisition_config.atchcalst = atchcalst;
	    config_normal.acquisition_config.atchcalsthr = atchcalsthr;
	        
	}
   
	/* send calibration command to the chip */
	if(ret == WRITE_MEM_OK)
	{
		/* change calibration suspend settings to zero until calibration confirmed good */
        	ret = write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data);
		if(ret == WRITE_MEM_OK)
		{
			/* set flag to show we must still confirm if calibration was good or bad */
			cal_check_flag = 1u;
		 }
	}
	return ret;
#else   
	/* set flag to show we must still confirm if calibration was good or bad */
	cal_check_flag = 1u;

   return(write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data));
#endif
}

/*!
 * \brief Diagonostic chip.
 * 
 * This function will send a command to diagnostic chip
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */
uint8_t diagnostic_chip(uint8_t mode)
{
    uint8_t status;
    status = write_mem(command_processor_address + DIAGNOSTIC_OFFSET, 1, &mode);
    return(status);
}

/*!
 * \brief Backups config area.
 * 
 * This function will send a command to backup the configuration to
 * non-volatile memory.
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */
uint8_t backup_config(void)
{
   /* Write 0x55 to BACKUPNV register to initiate the backup. */
   uint8_t data = 0x55u;
   
   return(write_mem(command_processor_address + BACKUP_OFFSET, 1, &data));
}

/*!
 * \brief Reads the id part of info block.
 * 
 * Reads the id part of the info block (7 bytes) from touch IC to 
 * info_block struct.
 *
 */
uint8_t read_id_block(info_id_t *id)
{
    uint8_t status;
     PRINT_FUNCTION_ENTER;
    status = read_mem(0, 1, (void *) &id->family_id);
    if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] family_id = 0x%x\n", id->family_id);
	}
    
    status = read_mem(1, 1, (void *) &id->variant_id);
    if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] variant_id = 0x%x\n", id->variant_id);
	}
        
    status = read_mem(2, 1, (void *) &id->version);
    if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] version = 0x%x\n", id->version);
	}
        
    status = read_mem(3, 1, (void *) &id->build);
    if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] build = 0x%x\n", id->build);
	}
        
    status = read_mem(4, 1, (void *) &id->matrix_x_size);
    if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] matrix_x_size = 0x%x\n", id->matrix_x_size);
	}
        
    status = read_mem(5, 1, (void *) &id->matrix_y_size);
    if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] matrix_y_size = 0x%x\n", id->matrix_y_size);
	}
        
    status = read_mem(6, 1, (void *) &id->num_declared_objects);
	if (status != READ_MEM_OK)
    {
        return(status);
    }else
	{
		printk(KERN_DEBUG "[TSP] num_declared_objects = 0x%x\n", id->num_declared_objects);
	}
	
	PRINT_FUNCTION_EXIT;
    return(status);
}

 /*!
  * \brief  This function retrieves the FW version number.
  * 
  * This function retrieves the FW version number.
  * 
  * @param *version pointer where to save version data.
  * @return ID_DATA_OK if driver is correctly initialized and 
  * id data can be read, ID_DATA_NOT_AVAILABLE otherwise.
  * 
  */
uint8_t get_version(uint8_t *version)
{
   if (info_block)
   {
       *version = info_block->info_id.version;
   }
   else
   {
      return(ID_DATA_NOT_AVAILABLE);
   }
   
   return (ID_DATA_OK);
}

/*!
 * \brief  This function retrieves the FW family id number.
 * 
 * This function retrieves the FW family id number.
 * 
 * @param *family_id pointer where to save family id data.
 * @return ID_DATA_OK if driver is correctly initialized and 
 * id data can be read, ID_DATA_NOT_AVAILABLE otherwise.
 * 
 */
uint8_t get_family_id(uint8_t *family_id)
{
  if (info_block)
  {
      *family_id = info_block->info_id.family_id;
  }
  else
  {
     return(ID_DATA_NOT_AVAILABLE);
  }
  
  return (ID_DATA_OK);
}

/*!
 * \brief  This function retrieves the FW build number.
 * 
 * This function retrieves the FW build number.
 * 
 * @param *build pointer where to save the build number data.
 * @return ID_DATA_OK if driver is correctly initialized and 
 * id data can be read, ID_DATA_NOT_AVAILABLE otherwise.
 * 
 */
uint8_t get_build_number(uint8_t *build)
{
  if (info_block)
  {
      *build = info_block->info_id.build;
  }
  else
  {
     return(ID_DATA_NOT_AVAILABLE);
  }
  
  return (ID_DATA_OK);
}

/*!
 * \brief  This function retrieves the FW variant number.
 * 
 * This function retrieves the FW variant id number.
 * 
 * @param *variant pointer where to save the variant id number data.
 * @return ID_DATA_OK if driver is correctly initialized and 
 * id data can be read, ID_DATA_NOT_AVAILABLE otherwise.
 * 
 */
uint8_t get_variant_id(uint8_t *variant)
{
  if (info_block)
  {
      *variant = info_block->info_id.variant_id;
  }
  else
  {
     return(ID_DATA_NOT_AVAILABLE);
  }
  
  return (ID_DATA_OK);
}

/*!
 * \brief Writes multitouchscreen config. 
 * 
 * 
 * This function will write the given configuration to given multitouchscreen
 * instance number.
 * 
 * @param instance the instance number of the multitouchscreen.
 * @param cfg multitouchscreen config struct.
 * @return 1 if successful.
 * 
 */
uint8_t write_multitouchscreen_config(uint8_t instance, 
                                      touch_multitouchscreen_t9_config_t cfg)
{
   uint16_t object_address;
   uint8_t *tmp;
   uint8_t status;
   uint8_t object_size;
   PRINT_FUNCTION_ENTER;
   object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
   if (object_size == 0)
   {
       return(CFG_WRITE_FAILED);
   }

   tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
   if (tmp == NULL)
   {
	   return(CFG_WRITE_FAILED);
   }

	memset(tmp,0,object_size); // ryun 20091201 
   
   /* 18 elements at beginning are 1 byte. */
   memcpy(tmp, &cfg, 18);
   
   /* Next two are 2 bytes. */
   
   *(tmp + 18) = (uint8_t) (cfg.xrange &  0xFF);
   *(tmp + 19) = (uint8_t) (cfg.xrange >> 8);

   *(tmp + 20) = (uint8_t) (cfg.yrange &  0xFF);
   *(tmp + 21) = (uint8_t) (cfg.yrange >> 8);

   /* And the last 4 1 bytes each again. */

   *(tmp + 22) = cfg.xloclip;
   *(tmp + 23) = cfg.xhiclip;
   *(tmp + 24) = cfg.yloclip;
   *(tmp + 25) = cfg.yhiclip;


    *(tmp + 26) = cfg.xedgectrl;
    *(tmp + 27) = cfg.xedgedist;
    *(tmp + 28) = cfg.yedgectrl;
    *(tmp + 29) = cfg.yedgedist;

    if(g_version >= 0x16)
    {
        *(tmp + 30) = cfg.jumplimit;
    }
   
   object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, 
                                       instance);
   
   if (object_address == 0)
   {
      return(CFG_WRITE_FAILED);
   }

//	printk("[TSP] %s() write_mem(object_address=0x%x, object_size=%d, ... )", __FUNCTION__, object_address,object_size);
   status = write_mem(object_address, object_size, tmp);
//   read_simple_config(TOUCH_MULTITOUCHSCREEN_T9, instance,  tmp);	// ryun 20091202
   kfree(tmp);	// ryun 20091201 
      PRINT_FUNCTION_EXIT;
   return(status);
}


/*!
 * \brief Writes GPIO/PWM config. 
 * 
 * @param instance the instance number which config to write.
 * @param cfg GPIOPWM config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_gpio_config(uint8_t instance, 
                          spt_gpiopwm_t19_config_t cfg)
{
   return(write_simple_config(SPT_GPIOPWM_T19, instance, (void *) &cfg));
}

/*!
 * \brief Writes GPIO/PWM config. 
 * 
 * @param instance the instance number which config to write.
 * @param cfg GPIOPWM config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_linearization_config(uint8_t instance, 
                                   proci_linearizationtable_t17_config_t cfg)
{
   uint16_t object_address;
   uint8_t *tmp;
   uint8_t status;   
   uint8_t object_size;
   
   object_size = get_object_size(PROCI_LINEARIZATIONTABLE_T17);
   if (object_size == 0)
   {
      return(CFG_WRITE_FAILED);
   }

   tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
   
   if (tmp == NULL)
   {
      return(CFG_WRITE_FAILED);
   }

	memset(tmp,0,object_size); // ryun 20091201 
    
   *(tmp + 0) = cfg.ctrl;
   *(tmp + 1) = (uint8_t) (cfg.xoffset & 0x00FF);
   *(tmp + 2) = (uint8_t) (cfg.xoffset >> 8);

   memcpy((tmp+3), &cfg.xsegment, 16);

   *(tmp + 19) = (uint8_t) (cfg.yoffset & 0x00FF);
   *(tmp + 20) = (uint8_t) (cfg.yoffset >> 8);

   memcpy((tmp+21), &cfg.ysegment, 16);
   
   object_address = get_object_address(PROCI_LINEARIZATIONTABLE_T17, 
                                       instance);
   
   if (object_address == 0)
   {
      return(CFG_WRITE_FAILED);
   }
   
   status = write_mem(object_address, object_size, tmp);
//   read_simple_config(PROCI_LINEARIZATIONTABLE_T17, instance,  tmp);	// ryun 20091202
   kfree(tmp);	// ryun 20091201 
   return(status);
}

/*!
 * \brief Writes two touch gesture processor config. 
 * 
 * @param instance the instance number which config to write.
 * @param cfg two touch gesture config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_twotouchgesture_config(uint8_t instance, 
   proci_twotouchgestureprocessor_t27_config_t cfg)
{
   uint16_t object_address;
   uint8_t *tmp;
   uint8_t status;   
   uint8_t object_size;
          
   object_size = get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
   if (object_size == 0)
   {
      return(CFG_WRITE_FAILED);
   }

   tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
       
   if (tmp == NULL)
   {
      return(CFG_WRITE_FAILED);
   }

	memset(tmp,0,object_size); // ryun 20091201 
    
   *(tmp + 0) = cfg.ctrl;
   *(tmp + 1) = cfg.numgest;
/*   	ryun 20091201 
   *(tmp + 2) = (uint8_t) (cfg.gesten & 0x00FF);
   *(tmp + 3) = (uint8_t) (cfg.gesten >> 8);
*/   
   *(tmp + 2) = 0;	// ryun 20091201 !!! ???
   *(tmp + 3) = cfg.gesten ;	// ryun 20091201

   *(tmp + 4) = cfg.rotatethr;
   
   *(tmp + 5) = (uint8_t) (cfg.zoomthr & 0x00FF);
   *(tmp + 6) = (uint8_t) (cfg.zoomthr >> 8);
      
   object_address = get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 
                                       instance);
   
   if (object_address == 0)
   {
      return(CFG_WRITE_FAILED);
   }

   status = write_mem(object_address, object_size, tmp);
//   read_simple_config(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, instance,  tmp);	// ryun 20091202

   kfree(tmp);	// ryun 20091201 
   return(status);
}


/*!
 * \brief Writes one touch gesture processor config. 
 * 
 * @param instance the instance number which config to write.
 * @param cfg one touch gesture config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_onetouchgesture_config(uint8_t instance, 
   proci_onetouchgestureprocessor_t24_config_t cfg)
{
   uint16_t object_address;
   uint8_t *tmp;
   uint8_t status;   
    uint8_t object_size;
   
   object_size = get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24);
   if (object_size == 0)
   {
     return(CFG_WRITE_FAILED);
   }

   tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
   if (tmp == NULL)
   {
      return(CFG_WRITE_FAILED);
   }

	memset(tmp,0,object_size); // ryun 20091201 
    
   *(tmp + 0) = cfg.ctrl;
   *(tmp + 1) = cfg.numgest;
   
   *(tmp + 2) = (uint8_t) (cfg.gesten & 0x00FF);
   *(tmp + 3) = (uint8_t) (cfg.gesten >> 8);
   
   memcpy((tmp+4), &cfg.pressproc, 7);
   
   *(tmp + 11) = (uint8_t) (cfg.flickthr & 0x00FF);
   *(tmp + 12) = (uint8_t) (cfg.flickthr >> 8);
   
   *(tmp + 13) = (uint8_t) (cfg.dragthr & 0x00FF);
   *(tmp + 14) = (uint8_t) (cfg.dragthr >> 8);
   
   *(tmp + 15) = (uint8_t) (cfg.tapthr & 0x00FF);
   *(tmp + 16) = (uint8_t) (cfg.tapthr >> 8);
   
   *(tmp + 17) = (uint8_t) (cfg.throwthr & 0x00FF);
   *(tmp + 18) = (uint8_t) (cfg.throwthr >> 8);

   object_address = get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 
                                       instance);
   
   if (object_address == 0)
   {
      return(CFG_WRITE_FAILED);
   }

   status = write_mem(object_address, object_size, tmp);
//      read_simple_config(PROCI_ONETOUCHGESTUREPROCESSOR_T24, instance,  tmp);	// ryun 20091202
   kfree(tmp);		// ryun 20091201 
   return(status);
}

/*!
 * \brief Writes selftest config. 
 * 
 * @param instance the instance number which config to write.
 * @param cfg selftest config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_selftest_config(uint8_t instance, 
                              spt_selftest_t25_config_t cfg)
{
   uint16_t object_address;
   uint8_t *tmp;
   uint8_t status;   
   uint8_t object_size;
   
   object_size = get_object_size(SPT_SELFTEST_T25);
   if (object_size == 0)
   {
      return(CFG_WRITE_FAILED);
   }

   tmp = (uint8_t *) kmalloc(object_size, GFP_KERNEL | GFP_ATOMIC);
      
   
   if (tmp == NULL)
   {
      return(CFG_WRITE_FAILED);
   }

	memset(tmp,0,object_size); // ryun 20091201 
	
   *(tmp + 0) = cfg.ctrl;
   *(tmp + 1) = cfg.cmd;
   
   *(tmp + 2) = (uint8_t) (cfg.upsiglim & 0x00FF);
   *(tmp + 3) = (uint8_t) (cfg.upsiglim >> 8);
   
   *(tmp + 2) = (uint8_t) (cfg.losiglim & 0x00FF);
   *(tmp + 3) = (uint8_t) (cfg.losiglim >> 8);
   
   object_address = get_object_address(SPT_SELFTEST_T25, 
                                       instance);
   
   if (object_address == 0)
   {
      return(CFG_WRITE_FAILED);
   }

   status = write_mem(object_address, object_size, tmp);
//        read_simple_config(SPT_SELFTEST_T25, instance,  tmp);		// ryun 20091202
   
   kfree(tmp);	// ryun 20091201 
   return(status);
}

/*!
 * \brief Writes key array config. 
 * 
 * @param instance the instance number which config to write.
 * @param cfg key array config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_keyarray_config(uint8_t instance, 
                              touch_keyarray_t15_config_t cfg)
{
   return(write_simple_config(TOUCH_KEYARRAY_T15, instance, (void *) &cfg));
}

/*!
 * \brief Writes grip suppression config. 
 * 
 * @param instance the instance number indicating which config to write.
 * @param cfg grip suppression config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_gripsuppression_config(uint8_t instance, 
   proci_gripfacesuppression_t20_config_t cfg)
{
   return(write_simple_config(PROCI_GRIPFACESUPPRESSION_T20, instance, 
                              (void *) &cfg));
}

/*!
 * \brief Writes a simple config struct to touch chip.
 *
 * Writes a simple config struct to touch chip. Does not necessarily 
 * (depending on how structs are stored on host platform) work for 
 * configs that have multibyte variables, so those are handled separately.
 * 
 * @param object_type object id number.
 * @param instance object instance number.
 * @param cfg pointer to config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_simple_config(uint8_t object_type, 
                            uint8_t instance, void *cfg)
{
   uint16_t object_address;
   uint8_t object_size;
   
   object_address = get_object_address(object_type, instance);
   object_size = get_object_size(object_type);
//   printk("[TSP] write_simple_config() : object_type=%d, object_address=%x, object_size=%d \n", object_type, object_address, object_size);
   if ((object_size == 0) || (object_address == 0))
   {
      return(CFG_WRITE_FAILED);
   }
   
   return (write_mem(object_address, object_size, cfg));
//	write_mem(object_address, object_size, cfg);	// ryun 20091202
//	read_simple_config(object_type, instance, cfg);	// ryun 20091202
//	return 1;	// ryun 20091202
}

uint8_t write_noisesuppression_config(uint8_t instance, procg_noisesuppression_t22_config_t cfg)
{

    return(write_simple_config(PROCG_NOISESUPPRESSION_T22, instance,
        (void *) &cfg));
}

// [[ ryun 20091202
uint8_t read_simple_config(uint8_t object_type, 
                            uint8_t instance, void *cfg)
{
   uint16_t object_address;
   uint8_t object_size;
   uint8_t buf[30];
   
   object_address = get_object_address(object_type, instance);
   object_size = get_object_size(object_type);
   printk("========[TSP] read_simple_config() : object_type=%d, object_address=%x, object_size=%d \n", object_type, object_address, object_size);
   if ((object_size == 0) || (object_address == 0))
   {
      return(CFG_WRITE_FAILED);
   }
   read_mem(object_address, object_size, buf);
   int i;
   for( i=0; i<30 ; i++){
	printk("======== [TSP] buf[%d] = %d \n", i, buf[i]);
   	}
   	return 0;
}
// ]] ryun 20091202

/*!
 * \brief Writes power config. 
 * 
 * @param cfg power config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_power_config(gen_powerconfig_t7_config_t cfg)
{
   return(write_simple_config(GEN_POWERCONFIG_T7, 0, (void *) &cfg));
}

/*!
 * \brief Writes CTE config. 
 * 
 * Writes CTE config, assumes that there is only one instance of 
 * CTE config objects.
 *
 * @param cfg CTE config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_CTE_config(spt_cteconfig_t28_config_t cfg)
{
   return(write_simple_config(SPT_CTECONFIG_T28, 0, (void *) &cfg));
}


/*!
 * \brief Writes acquisition config. 
 * 
 * @param cfg acquisition config struct.
 * 
 * @return 1 if successful.
 * 
 */
uint8_t write_acquisition_config(gen_acquisitionconfig_t8_config_t cfg)
{
	return(write_simple_config(GEN_ACQUISITIONCONFIG_T8, 0, (void *) &cfg));
}

// writing default 0s to avoid backup flash corruption
uint8_t write_comc_config(spt_comcconfig_t18_config_t cfg)
{
	return(write_simple_config(SPT_COMCONFIG_T18, 0, (void *) &cfg));
}

uint8_t write_gpiopwm_config(spt_gpiopwm_t19_config_t cfg)
{
	return(write_simple_config(SPT_GPIOPWM_T19, 0, (void *) &cfg));
}

uint8_t write_proximity_config(touch_proximity_t23_config_t cfg)
{
	return(write_simple_config(TOUCH_PROXIMITY_T23, 0, (void *) &cfg));
}

/*!
 * \brief Returns the start address of the selected object.
 * 
 * Returns the start address of the selected object and instance 
 * number in the touch chip memory map.  
 *
 * @param object_type the object ID number.
 * @param instance the instance number of the object.
 * 
 * @return object address, or OBJECT_NOT_FOUND if object/instance not found.
 * 
 */
uint16_t get_object_address(uint8_t object_type, uint8_t instance)
{
   object_t obj;
   uint8_t object_table_index = 0;
   uint8_t address_found = 0;
   uint16_t address = OBJECT_NOT_FOUND;
   
   object_t *object_table;
   if(info_block == NULL) {
		return address;
   }
   object_table = info_block->objects;
//   PRINT_FUNCTION_ENTER;

  #if 0 //def __CONFIG_SAMSUNG_DEBUG__
	printk("[TSP] object_table->object_type = 0x%x\n", object_table->object_type);
	printk("[TSP] object_table->i2c_address  = 0x%x\n", object_table->i2c_address );
	printk("[TSP] object_table->size  = 0x%x\n", object_table->size );
	printk("[TSP] object_table->instances  = 0x%x\n", object_table->instances );
	printk("[TSP] object_table->num_report_ids  = 0x%x\n\n", object_table->num_report_ids );
  #endif

   while ((object_table_index < info_block->info_id.num_declared_objects) && !address_found)
   {
      obj = object_table[object_table_index];
#if 0 //def __CONFIG_SAMSUNG_DEBUG__
	printk("[TSP] object_table_index  = 0x%x\n", object_table_index );
	printk("[TSP] info_block->info_id.num_declared_objects  = 0x%x\n", info_block->info_id.num_declared_objects );
	printk("[TSP] address_found  = 0x%x\n", address_found );
#endif

//	printk("[TSP] obj.object_type  = %d, object_type=%d\n", obj.object_type , object_type);
      /* Does object type match? */
      if (obj.object_type == object_type)
      {

         address_found = 1;

         /* Are there enough instances defined in the FW? */
         if (obj.instances >= instance)
         {
//         	printk("[TSP] find !!! obj.i2c_address  = 0x%x\n", obj.i2c_address);
            address = obj.i2c_address + (obj.size + 1) * instance;
         }
      }
      object_table_index++;
   }
//   PRINT_FUNCTION_EXIT;

   return(address);
}


/*!
 * \brief Returns the size of the selected object in the touch chip memory map.
 * 
 * Returns the size of the selected object in the touch chip memory map.
 *
 * @param object_type the object ID number.
 * 
 * @return object size, or OBJECT_NOT_FOUND if object not found.
 * 
 */
uint8_t get_object_size(uint8_t object_type)
{
   object_t obj;
   uint8_t object_table_index = 0;
   uint8_t object_found = 0;
   //uint16_t 
   uint8_t size = OBJECT_NOT_FOUND;
   
   object_t *object_table;
   if(info_block == NULL) {
		return size;
   }
   object_table = info_block->objects;

   while ((object_table_index < info_block->info_id.num_declared_objects) &&
          !object_found)
   {
      obj = object_table[object_table_index];
      /* Does object type match? */
      if (obj.object_type == object_type)
      {
         object_found = 1;
         size = obj.size + 1;
      }
      object_table_index++;
   }

   return(size);
}


/*!
 * \brief Return report id of given object/instance.
 * 
 *  This function will return a report id corresponding to given object type
 *  and instance, or 
 * 
 * @param object_type the object type identifier.
 * @param instance the instance of object.
 * 
 * @return report id, or 255 if the given object type does not have report id,
 * of if the firmware does not support given object type / instance.
 * 
 */
uint8_t type_to_report_id(uint8_t object_type, uint8_t instance)
{
   uint8_t report_id = 1;
   int8_t report_id_found = 0;
   
   while((report_id <= max_report_id) && (report_id_found == 0))
   {
      if((report_id_map[report_id].object_type == object_type) &&
         (report_id_map[report_id].instance == instance))
      {
         report_id_found = 1;
      }
      else
      {
         report_id++;	
      }
   }
   if (report_id_found)
   {
      return(report_id);
   }
   else
   {
      return(ID_MAPPING_FAILED);
   }
}

/*!
 * \brief Maps report id to object type and instance.
 * 
 *  This function will return an object type id and instance corresponding
 *  to given report id.
 * 
 * @param report_id the report id.
 * @param *instance pointer to instance variable. This function will set this
 *        to instance number corresponding to given report id.
 * @return the object type id, or 255 if the given report id is not used
 * at all.
 * 
 */
uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance)
{
   if (report_id <= max_report_id)
   {
      *instance = report_id_map[report_id].instance;
      return(report_id_map[report_id].object_type);
   }
   else
   {
      return(ID_MAPPING_FAILED);
   }
}

/*! 
 * \brief Return the maximum report id in use in the touch chip.
 * 
 * @return maximum_report_id 
 * 
 */
uint8_t get_max_report_id(void)
{
	return(max_report_id);
}



/*------------------------------ I2C Driver block -----------------------------------*/

/*!
 * \brief Information concerning the data transmission
 */
typedef struct
{
  //! TWI chip address to communicate with.
  char chip;
  //! TWI address/commands to issue to the other chip (node).
  unsigned int addr;
  //! Length of the TWI data address segment (1-3 bytes).
  int addr_length;
  //! Where to find the data to be written.
  void *buffer;
  //! How many bytes do we want to write.
  unsigned int length;
} twi_package_t;

#ifdef _I2C_USED_GPIO_PORT_
/* The following code is the I2C driver */
/* ------------ I2C Driver Defines --------------- */
#define ACK         0
#define NACK        1
#define I2C_OK      1
#define I2C_FAIL    0
#define READ_FLAG   0x01
#define BUS_DELAY   23

/*------------ I2C Code Macros ------------------- */
#define I2C_DELAY   for (i = 0; i < BUS_DELAY; i++)
#define SET_HI_SCL   {SCL = 1; while (!SCL); I2C_DELAY;}
#define SET_LO_SCL   {SCL = 0; I2C_DELAY;}
#define SET_HI_SDA   {SDA = 1; while (!SDA); I2C_DELAY;}
#define SET_LO_SDA   {SDA = 0; I2C_DELAY;}
#define FLOAT_SDA   {SDA = 1;}
#define SEND_START  {SET_LO_SDA; SET_LO_SCL;}
#define SEND_STOP   {SET_LO_SDA; SET_HI_SCL; SET_HI_SDA;}
#define SEND_CLOCK  {SET_HI_SCL; SET_LO_SCL;}
/*----------------------------------------------- */


/*============================================================================
  Name : send_i2c_byte()
------------------------------------------------------------------------------
Purpose: Write a single byte to the I2C bus and read the received ACK bit
Input  : Byte to send
Output : I2C_OK if device ACKs, I2C_FAIL if device NACKs
Notes  : 
============================================================================*/
uint8_t send_i2c_byte ( uint8_t tx_byte )
{
   uint8_t i,b, result = I2C_OK;
   
   for (b = 0; b < 8; b++)
   {
      if ( tx_byte & 0x80 )
         SET_HI_SDA
      else
         SET_LO_SDA
      SEND_CLOCK
      tx_byte <<= 1;	/* shift out data byte */
   }

   FLOAT_SDA	/* prepare to receive ACK bit */
   SET_HI_SCL	/* read ACK bit */
   if ( SDA == 1 )
      result = I2C_FAIL;
   SET_LO_SCL

   return result;
}

/*============================================================================
  Name : get_i2c_byte()
------------------------------------------------------------------------------
Purpose: Read a single byte from the I2C bus and terminate with ACK or NACK
Input  : State of ACK bit to send
Output : Received byte
Notes  : 
============================================================================*/
uint8_t get_i2c_byte ( uint8_t ack_bit )
{
   uint8_t i,b, rx_byte = 0;

   FLOAT_SDA

   for (b = 0; b < 8; b++)
   {
      rx_byte <<= 1;	/* shift in data byte */
      SET_HI_SCL;
      if ( SDA )
         rx_byte |= 1;
      SET_LO_SCL;
   }

   if ( ack_bit )	/* send ACK bit */
      SET_LO_SDA
   else
      SET_HI_SDA
   SEND_CLOCK

   return rx_byte;
}
#endif  //#ifdef _I2C_USED_GPIO_PORT_

/*!
 * \brief Initializes the I2C interface.
 *
 * @param I2C_address_arg the touch chip I2C address.
 *
 */
U8 init_I2C(U8 I2C_address_arg)
{
   //int status = 0;

   /* Set I2C Master options. */

   /* Check init result. */
#ifdef __CONFIG_SAMSUNG__
	PRINT_FUNCTION_ENTER;
	set_touch_i2c_gpio_init();
	i2c_tsp_sensor_init();
#endif


   return (I2C_INIT_OK);
}

void init_changeline(void) // ???????????????
{
   /* Enable pin. */

   /* Enable pin interrupts on falling edge. */
//   set_touch_irq_gpio_init();

   /* Enable interrupts, register interrupt handler. */

   /* Register GPIO interrupt at priority level 1 (2nd lowest). */
#ifdef __CONFIG_SAMSUNG__
	//	readly init in samsung probe
#endif
}

/*! \brief Disables pin change interrupts on falling edge. */
void disable_changeline_int(void)
{
#ifdef __CONFIG_SAMSUNG__
	set_touch_irq_gpio_disable();	// ryun 20091202
#endif
}

/*! \brief Enables pin change interrupts on falling edge. */
void enable_changeline_int(void)
{
#ifdef __CONFIG_SAMSUNG__
	set_touch_irq_gpio_init();
	enable_irq_handler();

#endif
}

/*! \brief Returns the changeline state. */
U8 read_changeline(void)
{
    //return(gpio_get_pin_value(CHANGELINE));
#ifdef __CONFIG_SAMSUNG__
    return get_touch_irq_gpio_value();
#endif

}

/*! \brief Maxtouch Memory read by I2C bus */
U8 read_mem(U16 start, U8 size, U8 *mem)
{
#ifdef __CONFIG_SAMSUNG__
	int status;
	unsigned int count = 0;
	
	while(1)
	{
		status = i2c_tsp_sensor_read(start, mem, size);
		if(status != -EREMOTEIO || count++ == RETRY_COUNT) break;
		printk("[TSP][ERROR] I2C NAK signal is received [count = %d]\n", count);
		msleep(10);
	}

	if(is_boot_state == 1 && count > RETRY_COUNT)
		return READ_MEM_FAILED;

	if(status < 0)
	{
		printk("[TSP][ERROR] i2c_tsp_sensor_read error !!\n");
		disable_irq(touchscreen_get_tsp_int_num());
		int repeat_count=0;
		unsigned char do_reset=1;
		unsigned char resume_success=0;

		do {
			if( do_reset )
			{
				printk("[TSP][ERROR] Reset TSP IC on I2C fail\n");
				gpio_direction_output(OMAP_GPIO_TOUCH_INT, 0);
				gpio_set_value(OMAP_GPIO_TOUCH_EN, 0);
				msleep(200);
				gpio_direction_output(OMAP_GPIO_TOUCH_INT, 1);
				gpio_direction_input(OMAP_GPIO_TOUCH_INT);
				gpio_set_value(OMAP_GPIO_TOUCH_EN, 1);
				msleep(80); // recommended value
				//calibrate_chip(); // Removed. When i2c error occur, this function call read_mem() as Recursion Function and then goes into infinite loop.
			}

			touch_state = 0;
			get_message();

			if( (touch_state & 0x10) == 0x10 )
			{
				printk(KERN_DEBUG "[TSP] reset and calibration success\n");
				resume_success = 1;
			}
			else
			{
				printk("[TSP] retry to reset\n");
				resume_success = 0;
				do_reset = 1;
			}
	 	} while (resume_success != 1 && repeat_count++ < RETRY_COUNT);
		clear_touch_history();
		enable_irq(touchscreen_get_tsp_int_num());
		return READ_MEM_FAILED;
	}
#endif
#if 0
   static uint8_t first_read = 1;

   int status = 0;
   twi_package_t packet, packet_received;
   U8 tmp;

   if (first_read){
      /* Make sure that when making the first read, address pointer
       * is always written. */
      address_pointer = start + 1;
      first_read = 0;
   }

   /* Set the read address pointer, if needed - consecutive reads from
    * message processor address can be done without updating the pointer.
    */

   if ((address_pointer != start) || (start != message_processor_address))
   {

      address_pointer = start;

      /* Swap nibbles since MSB is first but touch IC wants LSB first. */
      tmp = (U8) (start & 0xFF);
      start = (start >> 8) | (tmp << 8);


      /* TWI chip address to communicate with. */
      packet.chip = QT_I2C_address;
      /* TWI address/commands to issue to the other chip (node). */
      packet.addr = 0;
      /* Length of the TWI data address segment (1-3 bytes). */
      packet.addr_length = 0;
      /* Where to find the data to be written */
      packet.buffer = (void*) &start;
      /* How many bytes do we want to write. */
      packet.length = 2;

      //status = twi_master_write(&AVR32_TWI, &packet);

      if (status != TWI_SUCCESS)
      {
          return(READ_MEM_FAILED);
      }
   }

   packet_received.addr = 0;
   packet_received.addr_length = 0;
   packet_received.chip = I2C_address;
   packet_received.buffer = mem;
   packet_received.length = size;

   status = twi_master_read(&AVR32_TWI, &packet_received);

   if (status != TWI_SUCCESS)
   {
      return(READ_MEM_FAILED);
   }
#endif

   return(READ_MEM_OK);
}

U8 read_U16(U16 start, U16 *mem)
{
   //U8 tmp;
   U8 status;

   status = read_mem(start, 2, (U8 *) mem);

   /* Do the byte swap (touch chip uses LSB first byte order,
    * UC3B MSB first). */
/*
   if (status == READ_MEM_OK)
   {
      tmp = (uint8_t) (*mem & 0x00FF);
      *mem >>= 8;
      *mem |= (uint16_t) (tmp << 8);
   }
*/
   return (status);
}

/*! \brief Maxtouch Memory write by I2C bus */
U8 write_mem(U16 start, U8 size, U8 *mem)
{
#ifdef __CONFIG_SAMSUNG__
//	int i2c_tsp_sensor_write(u8 reg, u8 *val, unsigned int len)
	if(i2c_tsp_sensor_write(start, mem, size) < 0)
	{
		printk("[TSP][ERROR] i2c_tsp_sensor_write error !!\n");
		disable_irq(touchscreen_get_tsp_int_num());
		int repeat_count=0;
		unsigned char do_reset=1;
		unsigned char resume_success=0;

		do {
			if( do_reset )
			{
				printk("[TSP][ERROR] Reset TSP IC on I2C fail\n");
				gpio_direction_output(OMAP_GPIO_TOUCH_INT, 0);
				gpio_set_value(OMAP_GPIO_TOUCH_EN, 0);
				msleep(200);
				gpio_direction_output(OMAP_GPIO_TOUCH_INT, 1);
				gpio_direction_input(OMAP_GPIO_TOUCH_INT);
				gpio_set_value(OMAP_GPIO_TOUCH_EN, 1);
				msleep(80); // recommended value
				//calibrate_chip(); // Removed. When i2c error occur, this function call read_mem() as Recursion Function and then goes into infinite loop.
			}

			touch_state = 0;
			get_message();

			if( (touch_state & 0x10) == 0x10 )
			{
				printk(KERN_DEBUG "[TSP] reset and calibration success\n");
				resume_success = 1;
			}
			else
			{
				printk("[TSP] retry to reset\n");
				resume_success = 0;
				do_reset = 1;
			}
	 	} while (resume_success != 1 && repeat_count++ < RETRY_COUNT);
		clear_touch_history();
		enable_irq(touchscreen_get_tsp_int_num());
		return WRITE_MEM_FAILED;
	}
#endif

#if 0
   int status = 0;
   int i = 0;
   twi_package_t packet;
   U8 *tmp;

   address_pointer = start;

   tmp = kmalloc(2 + size, GFP_KERNEL);
   if (tmp == NULL)
   {
      return(WRITE_MEM_FAILED);
   }

   /* Swap start address nibbles since MSB is first but touch IC wants
    * LSB first. */
   *tmp = (U8) (start & 0xFF);
   *(tmp + 1) = (U8) (start >> 8);

   memcpy((tmp + 2), mem, size);

   disable_changeline_int();

   /* TWI chip address to communicate with. */
   packet.chip = I2C_address;
   /* TWI address/commands to issue to the other chip (node). */
   packet.addr = 0;
   /* Length of the TWI data address segment (1-3 bytes). */
   packet.addr_length = 0;
   /* Where to find the data to be written. */
   packet.buffer = (void*) tmp;
   /* How many bytes do we want to write. */
   packet.length = 2 + size;

   status = twi_master_write(&AVR32_TWI, &packet);

   /* Try the write several times if unsuccessful at first. */
   i = 0;
   while ((status != TWI_SUCCESS) & (i < 10))
   {
      status = twi_master_write(&AVR32_TWI, &packet);
      i++;
   }

   enable_changeline_int();

   kfree(tmp);

   if (status != TWI_SUCCESS)
   {
      return(WRITE_MEM_FAILED);
   }
#endif

   return(WRITE_MEM_OK);
}


/*
 * Message handler that is called by the touch chip driver when messages
 * are received.
 * 
 * This example message handler simply prints the messages as they are
 * received to USART1 port of EVK1011 board.
 */
void message_handler(U8 *msg, U8 length)
{  
   //usart_write_line(&AVR32_USART1, "Touch IC message: ");
   //write_message_to_usart(msg, length);
   //usart_write_line(&AVR32_USART1, "\n");
}

/*------------------------------ CRC calculation  block -----------------------------------*/
/*!
 * \brief Returns the stored CRC sum for the info block & object table area.
 *
 * @return the stored CRC sum for the info block & object table.
 *
 */
uint32_t get_stored_infoblock_crc(void)
{
   uint32_t crc;
   
   crc = (uint32_t) (((uint32_t) info_block->CRC_hi) << 16);
   crc = crc | info_block->CRC;
   return(crc);
}

/*!
 * \brief Calculates the CRC sum for the info block & object table area,
 * and checks it matches the stored CRC.
 *
 * Global interrupts need to be on when this function is called
 * since it reads the info block & object table area from the touch chip.
 *
 * @param *crc_pointer Pointer to memory location where
 *        the calculated CRC sum for the info block & object
 *        will be stored.
 * @return the CRC_CALCULATION_FAILED if calculation doesn't succeed, of
 *         CRC_CALCULATION_OK otherwise.
 *
 */
uint8_t calculate_infoblock_crc(uint32_t *crc_pointer)
{
   uint32_t crc = 0;
   //uint16_t 
   uint8_t crc_area_size;
   uint8_t *mem;
   uint8_t i;
   uint8_t status;
PRINT_FUNCTION_ENTER;
   /* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
   crc_area_size = 7 + info_block->info_id.num_declared_objects * 6;

   mem = (uint8_t *) kmalloc(crc_area_size, GFP_KERNEL | GFP_ATOMIC);
   if (mem == NULL)
   {
      return(CRC_CALCULATION_FAILED);
   }

   status = read_mem(0, crc_area_size, mem);

   if (status != READ_MEM_OK)
   {
      return(CRC_CALCULATION_FAILED);
   }

   i = 0;
   while (i < (crc_area_size - 1))
   {
      crc = CRC_24(crc, *(mem + i), *(mem + i + 1));
      i += 2;
   }

   crc = CRC_24(crc, *(mem + i), 0);

//   kfree(mem);

   /* Return only 24 bit CRC. */
   *crc_pointer = (crc & 0x00FFFFFF);
   PRINT_FUNCTION_EXIT;
   return(CRC_CALCULATION_OK);
}

/*!
 * \brief CRC calculation routine.
 *
 * @param crc the current CRC sum.
 * @param byte1 1st byte of new data to add to CRC sum.
 * @param byte2 2nd byte of new data to add to CRC sum.
 * @return crc the new CRC sum.
 *
 */
uint32_t CRC_24(uint32_t crc, uint8_t byte1, uint8_t byte2)
{
   static const uint32_t crcpoly = 0x80001B;
   uint32_t result;
   uint16_t data_word;

   data_word = (uint16_t) ((uint16_t) (byte2 << 8u) | byte1);
   result = ((crc << 1u) ^ (uint32_t) data_word);

   if (result & 0x1000000)
   {
      result ^= crcpoly;
   }

   return(result);
}


void get_default_power_config(gen_powerconfig_t7_config_t *power_config)
{
	/* Set power config. */
	/* Set Idle Acquisition Interval to 32 ms. */
	power_config->idleacqint = 48;
	/* Set Active Acquisition Interval to 16 ms. */
	power_config->actvacqint = 255;
	/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
	power_config->actv2idleto = 20;    
}
void get_default_acquisition_config(gen_acquisitionconfig_t8_config_t *acquisition_config)
{
	/* Set acquisition config. */
	acquisition_config->chrgtime = 8;
	acquisition_config->reserved = 0;
	acquisition_config->tchdrift = 20;
	acquisition_config->driftst = 20;
	acquisition_config->tchautocal = 0;
	acquisition_config->sync = 0;

	acquisition_config->atchcalst = 0;
	acquisition_config->atchcalsthr = 0;
  
}
void get_default_touchscreen_config(touch_multitouchscreen_t9_config_t *touchscreen_config)
{
	/* Set touchscreen config. */
	touchscreen_config->ctrl = 143;
	touchscreen_config->xorigin = 0;
	touchscreen_config->yorigin = 0;
	touchscreen_config->xsize = 0;   
	touchscreen_config->ysize = 0;
	touchscreen_config->akscfg = 0;
	touchscreen_config->blen = 0x21;
	touchscreen_config->tchthr = 50;
	touchscreen_config->tchdi = 2; 
	touchscreen_config->orientate = 1;
	touchscreen_config->mrgtimeout = 0;
	touchscreen_config->movhysti = 5;
	touchscreen_config->movhystn = 1;
	touchscreen_config->movfilter = 0;
	touchscreen_config->numtouch= 1;
	touchscreen_config->mrghyst = 10;
	touchscreen_config->mrgthr = 5;	// ryun 20100113 for multi touch
	touchscreen_config->tchamphyst = 10;
	touchscreen_config->xrange = 799;
	touchscreen_config->yrange = 479;
	touchscreen_config->xloclip = 0;
	touchscreen_config->xhiclip = 0;
	touchscreen_config->yloclip = 0;
	touchscreen_config->yhiclip = 0;

	touchscreen_config->xedgectrl = 0;
	touchscreen_config->xedgedist = 0;
	touchscreen_config->yedgectrl = 0;
	touchscreen_config->yedgedist = 0;
	touchscreen_config->jumplimit = 0;
    
}
void get_default_keyarray_config(touch_keyarray_t15_config_t *keyarray_config)
{
	/* Set key array config. */
	keyarray_config->ctrl = 0;
	keyarray_config->xorigin = 0;
	keyarray_config->xsize = 0;
	keyarray_config->yorigin = 0;
	keyarray_config->ysize = 0;
	keyarray_config->akscfg = 0;
	keyarray_config->blen = 0;
	keyarray_config->tchthr = 0;
	keyarray_config->tchdi = 0;    
}
void get_default_linearization_config(proci_linearizationtable_t17_config_t *linearization_config)
{
	linearization_config->ctrl = 0;    
}
void get_default_twotouch_gesture_config(proci_twotouchgestureprocessor_t27_config_t *twotouch_gesture_config )
{
	twotouch_gesture_config->ctrl = 0;
	twotouch_gesture_config->numgest = 0;
	twotouch_gesture_config->reserved2 = 0;
	twotouch_gesture_config->gesten = 0;
	twotouch_gesture_config->rotatethr = 0;
	twotouch_gesture_config->zoomthr = 0;
}
void get_default_onetouch_gesture_config(proci_onetouchgestureprocessor_t24_config_t * onetouch_gesture_config)
{
	onetouch_gesture_config->ctrl = 0;
	onetouch_gesture_config->numgest = 0;
	onetouch_gesture_config->gesten = 0;
	onetouch_gesture_config->pressproc = 0;
	onetouch_gesture_config->tapto = 0;
	onetouch_gesture_config->flickto = 0;
	onetouch_gesture_config->dragto = 0;
	onetouch_gesture_config->spressto = 0;
	onetouch_gesture_config->lpressto = 0;
	onetouch_gesture_config->rptpressto = 0;
	onetouch_gesture_config->flickthr = 0;
	onetouch_gesture_config->dragthr = 0;
	onetouch_gesture_config->tapthr = 0;
	onetouch_gesture_config->throwthr = 0;    
}

void get_default_noise_suppression_config(procg_noisesuppression_t22_config_t * noise_suppression_config)
{
	noise_suppression_config->ctrl = 5;//0; // ryun 20100113 for TA noise   

	noise_suppression_config->reserved1 = 0;
	noise_suppression_config->gcaful = 0;
	noise_suppression_config->gcafll = 0;


    noise_suppression_config->noisethr = 0;
    noise_suppression_config->reserved2 = 0;
    noise_suppression_config->freqhopscale = 0;

	noise_suppression_config->reserved = 0;
	noise_suppression_config->actvgcafvalid = 0;
	noise_suppression_config->freq[0] = 10;
	noise_suppression_config->freq[1] = 15;
	noise_suppression_config->freq[2] = 20;
	noise_suppression_config->freq[3] = 25;
	noise_suppression_config->freq[4] = 30;
	noise_suppression_config->idlegcafvalid = 0;

}
void get_default_selftest_config(spt_selftest_t25_config_t *selftest_config)
{
	selftest_config->ctrl = 0;
	selftest_config->cmd = 0;
}
void get_default_gripfacesuppression_config(proci_gripfacesuppression_t20_config_t *gripfacesuppression_config)
{
	gripfacesuppression_config->ctrl = 0;
	gripfacesuppression_config->xlogrip = 0;
	gripfacesuppression_config->xhigrip = 0;
	gripfacesuppression_config->ylogrip = 0;
	gripfacesuppression_config->yhigrip = 0;
	gripfacesuppression_config->maxtchs = 0;
	gripfacesuppression_config->reserved = 0;
	gripfacesuppression_config->szthr1 = 0;
	gripfacesuppression_config->szthr2 = 0;
	gripfacesuppression_config->shpthr1 = 0;
	gripfacesuppression_config->shpthr2 = 0;

	gripfacesuppression_config->supextto = 0;

}
void get_default_cte_config(spt_cteconfig_t28_config_t *cte_config)
{
	/* Disable CTE. */
	cte_config->ctrl = 1;
	cte_config->cmd = 0;
	cte_config->mode = 3; 
	cte_config->idlegcafdepth = 4;
	cte_config->actvgcafdepth = 4;

	cte_config->voltage= 0xA; //jihyon82.kim for 1.5 ver

}

void set_specific_power_config(
    gen_powerconfig_t7_config_t *power_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
        case LATONA:
        {
        	power_config->idleacqint = 64;
        	power_config->actvacqint = 255;//16;
        	power_config->actv2idleto = 50;
        }
        break;
        default:
        {
        }
    }    
}
void set_specific_acquisition_config(
    gen_acquisitionconfig_t8_config_t *acquisition_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
		case LATONA:
		{
			acquisition_config->chrgtime = 10;//8;
			acquisition_config->atchcalst = 0;
			acquisition_config->tchdrift = 5;
			acquisition_config->driftst = 1;
			acquisition_config->atchcalst = 9;
			acquisition_config->atchcalsthr = 27;//15;
		}
		break;
	}    
}
void set_specific_touchscreen_config(
    touch_multitouchscreen_t9_config_t *touchscreen_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
        case LATONA:
        {
		touchscreen_config->xsize = 18;
		touchscreen_config->ysize = 11;
		touchscreen_config->akscfg = 1;
		touchscreen_config->blen = 16; // TODO : 0x10 or 0x20
		touchscreen_config->tchthr = 32;//35;
		touchscreen_config->orientate = 1;
		touchscreen_config->movhysti = 3;
		touchscreen_config->mrghyst = 5;
		touchscreen_config->movfilter = 46;//31;
		touchscreen_config->numtouch = MAX_TOUCH_NUM;
		touchscreen_config->mrghyst = 5;
		touchscreen_config->mrgthr = 40;//10;
		touchscreen_config->jumplimit = 18;
        }
	break;
    }    
}
void set_specific_keyarray_config(
    touch_keyarray_t15_config_t *keyarray_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
        case LATONA:
        {
		keyarray_config->ctrl = 131;
		keyarray_config->xorigin = 16;
		keyarray_config->yorigin = 11;
		keyarray_config->xsize = 2;
		keyarray_config->ysize = 1;
		keyarray_config->akscfg = 1;
		keyarray_config->blen = 0; // TODO : 0x10 or 0x20
		keyarray_config->tchthr = 45; //40;
		keyarray_config->tchdi = 4; // 4;
        }
        break;
    }    
}
void set_specific_linearization_config(
    proci_linearizationtable_t17_config_t *linearization_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
        case LATONA:
        {
        }
        break;
    }    
}
void set_specific_twotouch_gesture_config(
    proci_twotouchgestureprocessor_t27_config_t *twotouch_gesture_config, 
    Atmel_model_type model_type  )
{
    switch(model_type)
    {
	case LATONA:
        {
        }
        break;
    }    
}
void set_specific_onetouch_gesture_config(
    proci_onetouchgestureprocessor_t24_config_t * onetouch_gesture_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
	case LATONA:
        {
        }
        break;
    }    
}

void set_specific_noise_suppression_config(
    procg_noisesuppression_t22_config_t * noise_suppression_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
	case LATONA:
        {
        	noise_suppression_config->ctrl = 135;
        	noise_suppression_config->gcaful = 0;
        	noise_suppression_config->gcafll = 0;
        	noise_suppression_config->actvgcafvalid = 3;
        	noise_suppression_config->noisethr = 27;
        	noise_suppression_config->freqhopscale = 0;
        	noise_suppression_config->freq[0] = 29;//6;
        	noise_suppression_config->freq[1] = 34;//11;
        	noise_suppression_config->freq[2] = 39;//16;
        	noise_suppression_config->freq[3] = 49;//19;
        	noise_suppression_config->freq[4] = 58;//21;
        	noise_suppression_config->idlegcafvalid = 3;
        }
        break;
    }    
}
void set_specific_selftest_config(
    spt_selftest_t25_config_t *selftest_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
	case LATONA:
        {
        }
        break;
    }    
}
void set_specific_gripfacesuppression_config(
    proci_gripfacesuppression_t20_config_t *gripfacesuppression_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
	case LATONA:
        {
       	gripfacesuppression_config->ctrl = 19;

       	gripfacesuppression_config->xlogrip = 0;
       	gripfacesuppression_config->xhigrip = 0;
       	gripfacesuppression_config->ylogrip = 5;
       	gripfacesuppression_config->yhigrip = 5;
       	gripfacesuppression_config->maxtchs = 0;
       	gripfacesuppression_config->reserved = 0;
		gripfacesuppression_config->szthr1 = 30;
		gripfacesuppression_config->szthr2 = 20;
		gripfacesuppression_config->shpthr1 = 4;
		gripfacesuppression_config->shpthr2 = 15;//15;
		gripfacesuppression_config->supextto = 10;	
        }
        break;
    }    
}
void set_specific_cte_config(
    spt_cteconfig_t28_config_t *cte_config, 
    Atmel_model_type model_type )
{
    switch(model_type)
    {
		case LATONA:
        {
            cte_config->ctrl = 0;
            cte_config->cmd = 0;
            cte_config->mode = 2;
            cte_config->idlegcafdepth = 16;
            cte_config->actvgcafdepth = 63;//32;
            cte_config->voltage= 60;
        }
        break;
    }    
}


/*------------------------------ for tunning ATmel - start ----------------------------*/
ssize_t set_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);
	cmd_no = (int) (value / 1000);
	config_value = ( int ) (value % 1000 );

	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , power_config.idleacqint = %d\n", __func__,config_value );
		config_normal.power_config.idleacqint = config_value;
	}		
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , power_config.actvacqint = %d\n", __func__, config_value);
		config_normal.power_config.actvacqint = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , power_config.actv2idleto= %d\n", __func__, config_value);
		config_normal.power_config.actv2idleto = config_value;
	}
	else 
	{
		printk("[%s] unknown CMD\n", __func__);
	}

	return size;
}


ssize_t set_acquisition_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_acquisition_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);
	cmd_no = (int) (value / 1000);
	config_value = ( int ) (value % 1000 );

	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , acquisition_config.chrgtime = %d\n", __func__, config_value );
		config_normal.acquisition_config.chrgtime = config_value;
	}		
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , acquisition_config.reserved = %d\n", __func__, config_value );
		config_normal.acquisition_config.reserved = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , acquisition_config.tchdrift = %d\n", __func__, config_value );
		config_normal.acquisition_config.tchdrift = config_value;
	}
	else if(cmd_no == 3)
	{
		printk("[%s] CMD 3 , acquisition_config.driftst= %d\n", __func__, config_value );
		config_normal.acquisition_config.driftst = config_value;
	}else if(cmd_no == 4)
	{
		printk("[%s] CMD 4 , acquisition_config.tchautocal = %d\n", __func__, config_value );
		config_normal.acquisition_config.tchautocal= config_value;
	}
	else if(cmd_no == 5)
	{
		printk("[%s] CMD 5 , acquisition_config.sync = %d\n", __func__, config_value );
		config_normal.acquisition_config.sync = config_value;
	}
	else if(cmd_no == 6)
	{
		printk("[%s] CMD 6 , acquisition_config.atchcalst = %d\n", __func__, config_value );
		config_normal.acquisition_config.atchcalst = config_value;
	}
	else if(cmd_no == 7)
	{
		printk("[%s] CMD 7 , acquisition_config.atchcalsthr = %d\n", __func__, config_value );
		config_normal.acquisition_config.atchcalsthr= config_value;
	}
	else 
	{
		printk("[%s] unknown CMD\n", __func__);
	}


	return size;
}


ssize_t set_touchscreen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_touchscreen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);
	cmd_no = (int) (value / 1000);
	config_value = ( int ) (value % 1000 );

	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , touchscreen_config.ctrl = %d\n", __func__, config_value );
		config_normal.touchscreen_config.ctrl = config_value;
	}		
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , touchscreen_config.xorigin  = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xorigin = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , touchscreen_config.yorigin = %d\n", __func__, config_value );
		config_normal.touchscreen_config.yorigin  = config_value;
	}
	else if(cmd_no == 3)
	{
		printk("[%s] CMD 3 , touchscreen_config.xsize = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xsize =  config_value;
	}
	else if(cmd_no == 4)
	{
		printk("[%s] CMD 4 , touchscreen_config.ysize = %d\n", __func__, config_value );
		config_normal.touchscreen_config.ysize =  config_value;
	}
	else if(cmd_no == 5)
	{
		printk("[%s] CMD 5 , touchscreen_config.akscfg = %d\n", __func__, config_value );
		config_normal.touchscreen_config.akscfg = config_value;
	}
	else if(cmd_no == 6)
	{
		printk("[%s] CMD 6 , touchscreen_config.blen = %d\n", __func__, config_value );
		config_normal.touchscreen_config.blen = config_value;
	}
	else if(cmd_no == 7)
	{
		printk("[%s] CMD 7 , touchscreen_config.tchthr = %d\n", __func__, config_value );
		config_normal.touchscreen_config.tchthr = config_value;
	}
	else if(cmd_no == 8)
	{
		printk("[%s] CMD 8 , touchscreen_config.tchdi = %d\n", __func__, config_value );
		config_normal.touchscreen_config.tchdi= config_value;
	}
	else if(cmd_no == 9)
	{
		printk("[%s] CMD 9 , touchscreen_config.orientate = %d\n", __func__, config_value );
		config_normal.touchscreen_config.orientate = config_value;
	}
	else if(cmd_no == 10)
	{
		printk("[%s] CMD 10 , touchscreen_config.mrgtimeout = %d\n", __func__, config_value );
		config_normal.touchscreen_config.mrgtimeout = config_value;
	}
	else if(cmd_no == 11)
	{
		printk("[%s] CMD 11 , touchscreen_config.movhysti = %d\n", __func__, config_value );
		config_normal.touchscreen_config.movhysti = config_value;
	}
	else if(cmd_no == 12)
	{
		printk("[%s] CMD 12 , touchscreen_config.movhystn = %d\n", __func__, config_value );
		config_normal.touchscreen_config.movhystn = config_value;
	}
	else if(cmd_no == 13)
	{
		printk("[%s] CMD 13 , touchscreen_config.movfilter = %d\n", __func__, config_value );
		config_normal.touchscreen_config.movfilter = config_value;
	}
	else if(cmd_no == 14)
	{
		printk("[%s] CMD 14 , touchscreen_config.numtouch = %d\n", __func__, config_value );
		config_normal.touchscreen_config.numtouch = config_value;
	}
	else if(cmd_no == 15)
	{
		printk("[%s] CMD 15 , touchscreen_config.mrghyst = %d\n", __func__, config_value );
		config_normal.touchscreen_config.mrghyst = config_value;
	}
	else if(cmd_no == 16)
	{
		printk("[%s] CMD 16 , touchscreen_config.mrgthr = %d\n", __func__, config_value );
		config_normal.touchscreen_config.mrgthr = config_value;
	}
	else if(cmd_no == 17)
	{
		printk("[%s] CMD 17 , touchscreen_config.tchamphyst = %d\n", __func__, config_value );
		config_normal.touchscreen_config.tchamphyst = config_value;
	}
	else if(cmd_no == 18)
	{
		printk("[%s] CMD 18 , touchscreen_config.xrange = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xrange= config_value;
	}
	else if(cmd_no == 19)
	{
		printk("[%s] CMD 19 , touchscreen_config.yrange = %d\n", __func__, config_value );
		config_normal.touchscreen_config.yrange = config_value;
	}
	else if(cmd_no == 20)
	{
		printk("[%s] CMD 20 , touchscreen_config.xloclip = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xloclip = config_value;
	}
	else if(cmd_no == 21)
	{
		printk("[%s] CMD 21 , touchscreen_config.xhiclip = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xhiclip = config_value;
	}
	else if(cmd_no == 22)
	{
		printk("[%s] CMD 22 , touchscreen_config.yloclip = %d\n", __func__, config_value );
		config_normal.touchscreen_config.yloclip = config_value;
	}
	else if(cmd_no == 23)
	{
		printk("[%s] CMD 23 , touchscreen_config.yhiclip = %d\n", __func__, config_value );
		config_normal.touchscreen_config.yhiclip = config_value;
	}
	else if(cmd_no == 24)
	{
		printk("[%s] CMD 24 , touchscreen_config.xedgectrl = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xedgectrl = config_value;
	}
	else if(cmd_no == 25)
	{
		printk("[%s] CMD 25 , touchscreen_config.xedgedist = %d\n", __func__, config_value );
		config_normal.touchscreen_config.xedgedist = config_value;
	}
	else if(cmd_no == 26)
	{
		printk("[%s] CMD 26 , touchscreen_config.yedgectrl = %d\n", __func__, config_value );
		config_normal.touchscreen_config.yedgectrl = config_value;
	}
	else if(cmd_no == 27)
	{
		printk("[%s] CMD 27 , touchscreen_config.yedgedist = %d\n", __func__, config_value );
		config_normal.touchscreen_config.yedgedist = config_value;
	}
	else if(cmd_no == 28)
	{
		printk("[%s] CMD 28 , touchscreen_config.jumplimit = %d\n", __func__, config_value );
		config_normal.touchscreen_config.jumplimit = config_value;
	}
	else 
	{
		printk("[%s] unknown CMD\n", __func__);
	}

	return size;
}

ssize_t set_keyarray_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_keyarray_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);
	cmd_no = (int) (value / 1000);
	config_value = ( int ) (value % 1000 );

	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , keyarray_config.ctrl = %d\n", __func__, config_value );
		config_normal.keyarray_config.ctrl = config_value;
	}		
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , keyarray_config.xorigin = %d\n", __func__, config_value );
		config_normal.keyarray_config.xorigin = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , keyarray_config.yorigin = %d\n", __func__, config_value );
		config_normal.keyarray_config.yorigin = config_value;
	}
	else if(cmd_no == 3)
	{
		printk("[%s] CMD 3 , keyarray_config.xsize = %d\n", __func__, config_value );
		config_normal.keyarray_config.xsize = config_value;
	}
	else if(cmd_no == 4)
	{
		printk("[%s] CMD 4 , keyarray_config.ysize = %d\n", __func__, config_value );
		config_normal.keyarray_config.ysize  = config_value;
	}
	else if(cmd_no == 5)
	{
		printk("[%s] CMD 5 , keyarray_config.akscfg = %d\n", __func__, config_value );
		config_normal.keyarray_config.akscfg  = config_value;
	}
	else if(cmd_no == 6)
	{
		printk("[%s] CMD 6 , keyarray_config.blen = %d\n", __func__, config_value );
		config_normal.keyarray_config.blen = config_value;
	}
	else if(cmd_no == 7)
	{
		printk("[%s] CMD 7 , keyarray_config.tchthr = %d\n", __func__, config_value );
		config_normal.keyarray_config.tchthr = config_value;
	}
	else if(cmd_no == 8)
	{
		printk("[%s] CMD 8 , keyarray_config.tchdi = %d\n", __func__, config_value );
		config_normal.keyarray_config.tchdi = config_value;
	}
	else 
	{
		printk("[%s] unknown CMD\n", __func__);
	}

	return size;
}

ssize_t set_grip_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_grip_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);
	cmd_no = (int) (value / 1000);
	config_value = ( int ) (value % 1000 );

	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , gripfacesuppression_config.ctrl = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.ctrl =  config_value;
	}
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , gripfacesuppression_config.xlogrip = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.xlogrip = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , gripfacesuppression_config.xhigrip = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.xhigrip =  config_value;
	}
	else if(cmd_no == 3)
	{
		printk("[%s] CMD 3 , gripfacesuppression_config.ylogrip = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.ylogrip =  config_value;
	}
	else if(cmd_no == 4 )
	{
		printk("[%s] CMD 4 , gripfacesuppression_config.yhigrip = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.yhigrip =  config_value;
	}
	else if(cmd_no == 5 )
	{
		printk("[%s] CMD 5 , gripfacesuppression_config.maxtchs= %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.maxtchs =  config_value;
	}
	else if(cmd_no == 6)
	{
		printk("[%s] CMD 6 , gripfacesuppression_config.reserved = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.reserved=  config_value;
	}
	else if(cmd_no == 7)
	{
		printk("[%s] CMD 7 , gripfacesuppression_config.szthr1 = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.szthr1=  config_value;
	}
	else if(cmd_no == 8)
	{
		printk("[%s] CMD 8 , gripfacesuppression_config.szthr2 = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.szthr2=  config_value;
	}
	else if(cmd_no == 9)
	{
		printk("[%s] CMD 9 , gripfacesuppression_config.shpthr1 = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.shpthr1=  config_value;
	}
	else if(cmd_no == 10)
	{
		printk("[%s] CMD 10 , gripfacesuppression_config.shpthr2 = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.shpthr2 =  config_value;
	}
	else if(cmd_no == 11)
	{
		printk("[%s] CMD 11 , gripfacesuppression_config.supextto = %d\n", __func__, config_value );
		config_normal.gripfacesuppression_config.supextto =  config_value;
	}
	else 
	{
		printk("[%s] unknown CMD\n", __func__);
	}

	return size;
}

ssize_t set_noise_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_noise_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);

	if ( value > 100000 ){ 
		cmd_no = (int) (value / 100000 );
		config_value = ( int ) (value % 100000 );

		if( !( (cmd_no==1) || (cmd_no==2) ) ){   // this can be only gcaful, gcafll // CMD1, CMD2
			printk("[%s] unknown CMD\n", __func__);
			return size;	
		}
	}	
	else 
	{
		cmd_no = (int) (value / 1000);
		config_value = ( int ) (value % 1000 );

		if( ( (cmd_no==1) || (cmd_no==2) ) ){   // this can't be only gcaful, gcafll // CMD1, CMD2
			printk("[%s] unknown CMD\n", __func__);
			return size;	
		}
	}


	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , noise_suppression_config.ctrl = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.ctrl = config_value;
	}
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , noise_suppression_config.gcaful = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.gcaful = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , noise_suppression_config.gcafll= %d\n", __func__, config_value );
		config_normal.noise_suppression_config.gcafll = config_value;
	}
	else if(cmd_no == 3)
	{
		printk("[%s] CMD 3 , noise_suppression_config.actvgcafvalid = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.actvgcafvalid = config_value;
	}
	else if(cmd_no == 4)
	{
		printk("[%s] CMD 4 , noise_suppression_config.noisethr = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.noisethr = config_value;
	}
	else if(cmd_no == 5)
	{
		printk("[%s] CMD 5 , noise_suppression_config.freqhopscale = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.freqhopscale = config_value;
	}
	else if(cmd_no == 6)
	{
		printk("[%s] CMD 6 , noise_suppression_config.freq[0] = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.freq[0] = config_value;
	}
	else if(cmd_no == 7)
	{	printk("[%s] CMD 7 , noise_suppression_config.freq[1] = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.freq[1] = config_value;
	}
	else if(cmd_no == 8)
	{
		printk("[%s] CMD 8 , noise_suppression_config.freq[2] = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.freq[2] = config_value;
	}
	else if(cmd_no == 9)
	{		
		printk("[%s] CMD 9 , noise_suppression_config.freq[3] = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.freq[3] = config_value;
	}
	else if(cmd_no == 10)
	{		
		printk("[%s] CMD 10 , noise_suppression_config.freq[4] = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.freq[4] = config_value;
	}
	else if(cmd_no == 11)
	{
		printk("[%s] CMD 11 , noise_suppression_config.idlegcafvalid = %d\n", __func__, config_value );
		config_normal.noise_suppression_config.idlegcafvalid = config_value;
	}
	else 
	{
		printk("[%s] unknown CMD\n", __func__);
	}

	return size;
}

ssize_t set_total_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int value;
	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
ssize_t set_total_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//	int ret;
  int cmd_no,config_value = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);
	cmd_no = (int) (value / 1000);
	config_value = ( int ) (value % 1000 );

	if (cmd_no == 0)
	{
		printk("[%s] CMD 0 , linearization_config.ctrl = %d\n", __func__, config_value );
		config_normal.linearization_config.ctrl = config_value;
	}		
	else if(cmd_no == 1)
	{
		printk("[%s] CMD 1 , twotouch_gesture_config.ctrl = %d\n", __func__, config_value );
		config_normal.twotouch_gesture_config.ctrl = config_value;
	}
	else if(cmd_no == 2)
	{
		printk("[%s] CMD 2 , onetouch_gesture_config.ctrl = %d\n", __func__, config_value );
		config_normal.onetouch_gesture_config.ctrl = config_value;
	}
	else if(cmd_no == 3)
	{
		printk("[%s] CMD 3 , selftest_config.ctrl = %d\n", __func__, config_value );
		config_normal.selftest_config.ctrl = config_value;
	}
	else if(cmd_no == 4)
	{
		printk("[%s] CMD 4 , cte_config.ctrl = %d\n", __func__, config_value );
		config_normal.cte_config.ctrl = config_value;
	}
	else if(cmd_no == 5)
	{
		printk("[%s] CMD 5 , cte_config.cmd = %d\n", __func__, config_value );
		config_normal.cte_config.cmd = config_value;
	}
	else if(cmd_no == 6)
	{
		printk("[%s] CMD 6 , cte_config.mode = %d\n", __func__, config_value );
		config_normal.cte_config.mode = config_value;
	}
	else if(cmd_no == 7)
	{
		printk("[%s] CMD 7 , cte_config.idlegcafdepth = %d\n", __func__, config_value );
		config_normal.cte_config.idlegcafdepth = config_value;
	}
	else if(cmd_no == 8)
	{
		printk("[%s] CMD 8 , cte_config.actvgcafdepth = %d\n", __func__, config_value );
		config_normal.cte_config.actvgcafdepth = config_value;
	}
	else if(cmd_no == 9)
	{
		printk("[%s] CMD 9 , cte_config.voltage = %d\n", __func__, config_value );
		config_normal.cte_config.voltage = config_value;
	}
/*	else if(cmd_no == 10)
	{
		printk("[%s] CMD 9 , DELAY_TIME = %d\n", __func__, config_value );
		DELAY_TIME = config_value;
	}*/
	else
	{
		printk("[%s] unknown CMD\n", __func__);
	}

	return size;
}

ssize_t set_write_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  int tmp=-1;

	printk("\n=========== [TSP] Configure SET for normal ============\n");
	printk("=== set_power - GEN_POWERCONFIG_T7 ===\n");
	printk("0. idleacqint=%3d, 1. actvacqint=%3d, 2. actv2idleto=%3d\n", config_normal.power_config.idleacqint, config_normal.power_config.actvacqint, config_normal.power_config.actv2idleto );  

	printk("=== set_acquisition - GEN_ACQUIRECONFIG_T8 ===\n");
	printk("0. chrgtime=%3d,   1. reserved=%3d,   2. tchdrift=%3d\n",config_normal.acquisition_config.chrgtime,config_normal.acquisition_config.reserved,config_normal.acquisition_config.tchdrift); 
	printk("3. driftst=%3d,    4. tchautocal=%3d, 5. sync=%3d\n", config_normal.acquisition_config.driftst,config_normal.acquisition_config.tchautocal,config_normal.acquisition_config.sync);
	printk("6. atchcalst=%3d,  7. atchcalsthr=%3d\n", config_normal.acquisition_config.atchcalst , config_normal.acquisition_config.atchcalsthr); 

	printk("=== set_touchscreen - TOUCH_MULTITOUCHSCREEN_T9 ===\n");
	printk("0. ctrl=%3d,       1. xorigin=%3d,    2. yorigin=%3d\n",  config_normal.touchscreen_config.ctrl, config_normal.touchscreen_config.xorigin, config_normal.touchscreen_config.yorigin  );
	printk("3. xsize=%3d,      4. ysize=%3d,      5. akscfg=%3d\n", config_normal.touchscreen_config.xsize, config_normal.touchscreen_config.ysize , config_normal.touchscreen_config.akscfg  );
	printk("6. blen=%3d,       7. tchthr=%3d,     8. tchdi=%3d\n", config_normal.touchscreen_config.blen, config_normal.touchscreen_config.tchthr, config_normal.touchscreen_config.tchdi );
	printk("9. orientate=%3d,  10.mrgtimeout=%3d, 11.movhysti=%3d\n",config_normal.touchscreen_config.orientate,config_normal.touchscreen_config.mrgtimeout, config_normal.touchscreen_config.movhysti);
	printk("12.movhystn=%3d,   13.movfilter=%3d,  14.numtouch=%3d\n",config_normal.touchscreen_config.movhystn, config_normal.touchscreen_config.movfilter ,config_normal.touchscreen_config.numtouch );
	printk("15.mrghyst=%3d,    16.mrgthr=%3d,     17.tchamphyst=%3d\n",config_normal.touchscreen_config.mrghyst,config_normal.touchscreen_config.mrgthr,config_normal.touchscreen_config.tchamphyst );
	printk("18.xrange=%3d,       19.yrange=%3d,       20.xloclip=%3d\n",config_normal.touchscreen_config.xrange, config_normal.touchscreen_config.yrange, config_normal.touchscreen_config.xloclip );
	printk("21.xhiclip=%3d,    22.yloclip=%3d,    23.yhiclip=%3d\n", config_normal.touchscreen_config.xhiclip, config_normal.touchscreen_config.yloclip ,config_normal.touchscreen_config.yhiclip );
	printk("24.xedgectrl=%3d,  25.xedgedist=%3d,  26.yedgectrl=%3d\n",config_normal.touchscreen_config.xedgectrl,config_normal.touchscreen_config.xedgedist,config_normal.touchscreen_config.yedgectrl);
	printk("27.yedgedist=%3d,  28.jumplimit=%3d\n", config_normal.touchscreen_config.yedgedist, config_normal.touchscreen_config.jumplimit );

	printk("=== set_keyarray - TOUCH_KEYARRAY_T15 ===\n");
	printk("0. ctrl=%3d,       1. xorigin=%3d,    2. yorigin=%3d\n", config_normal.keyarray_config.ctrl, config_normal.keyarray_config.xorigin, config_normal.keyarray_config.yorigin); 
	printk("3. xsize=%3d,      4. ysize=%3d,      5. akscfg=%3d\n", config_normal.keyarray_config.xsize, config_normal.keyarray_config.ysize, config_normal.keyarray_config.akscfg );
	printk("6. blen=%3d,       7. tchthr=%3d,     8. tchdi=%3d\n", config_normal.keyarray_config.blen, config_normal.keyarray_config.tchthr, config_normal.keyarray_config.tchdi  );

	printk("=== set_grip - PROCI_GRIPFACESUPRESSION_T20 ===\n");
	printk("0. ctrl=%3d,       1. xlogrip=%3d,    2. xhigrip=%3d\n",config_normal.gripfacesuppression_config.ctrl, config_normal.gripfacesuppression_config.xlogrip, config_normal.gripfacesuppression_config.xhigrip );
	printk("3. ylogrip=%3d     4. yhigrip=%3d,    5. maxtchs=%3d\n",config_normal.gripfacesuppression_config.ylogrip,config_normal.gripfacesuppression_config.yhigrip , config_normal.gripfacesuppression_config.maxtchs );
	printk("6. reserved=%3d,  7. szthr1=%3d,     8. szthr2=%3d\n"  , config_normal.gripfacesuppression_config.reserved , config_normal.gripfacesuppression_config.szthr1, config_normal.gripfacesuppression_config.szthr2 );
	printk("9. shpthr1=%3d     10.shpthr2=%3d,    11.supextto=%3d\n",config_normal.gripfacesuppression_config.shpthr1 , config_normal.gripfacesuppression_config.shpthr2 , config_normal.gripfacesuppression_config.supextto );

	printk("=== set_noise ===\n");
	printk("0. ctrl = %3d,         1. gcaful(2bts)=%d\n", config_normal.noise_suppression_config.ctrl, config_normal.noise_suppression_config.gcaful); 
	printk("2. gcafll(2bts)=%5d, 3. actvgcafvalid =%d\n", config_normal.noise_suppression_config.gcafll ,  config_normal.noise_suppression_config.actvgcafvalid); 
	printk("4. noisethr=%3d,   5.freqhopscale=%3d,6. freq[0]=%3d\n", config_normal.noise_suppression_config.noisethr, config_normal.noise_suppression_config.freqhopscale, config_normal.noise_suppression_config.freq[0] ); 
	printk("7. freq[1]=%3d,    8. freq[2]=%3d,    9. freq[3]=%3d\n", config_normal.noise_suppression_config.freq[1],  config_normal.noise_suppression_config.freq[2] , config_normal.noise_suppression_config.freq[3]); 
	printk("10.freq[4]=%3d,    11.idlegcafvalid=%3d\n", config_normal.noise_suppression_config.freq[4] , config_normal.noise_suppression_config.idlegcafvalid); 

	printk("=== set_total ===\n");
	printk("0 , linearization_config.ctrl = %d\n", config_normal.linearization_config.ctrl );
	printk("1 , twotouch_gesture_config.ctrl = %d\n", config_normal.twotouch_gesture_config.ctrl );
	printk("2 , onetouch_gesture_config.ctrl = %d\n",config_normal.onetouch_gesture_config.ctrl );
	printk("3 , selftest_config.ctrl = %d\n", config_normal.selftest_config.ctrl);
	printk("4. cte_config.ctrl=%3d,  5. cte_config.cmd=%3d\n", config_normal.cte_config.ctrl, config_normal.cte_config.cmd );
	printk("6. cte_config.mode=%3d,  7. cte_config.idlegcafdepth=%3d\n", config_normal.cte_config.mode,  config_normal.cte_config.idlegcafdepth );
	printk("8. cte_config.actvgcafdepth=%3d,  9.cte_config.voltage=%3d\n", config_normal.cte_config.actvgcafdepth, config_normal.cte_config.voltage );
//	printk("9 , DELAY_TIME = %d\n", DELAY_TIME );
	printk("================= end ======================\n");

	/* set all configuration */
	tmp = set_all_config(config_normal);
	if( tmp == 1 )
		printk("[TSP] set all configs success : %d\n", __LINE__);
	else
		printk("[TSP] set all configs fail : error : %d\n", __LINE__);


    	if (backup_config() != WRITE_MEM_OK)
    	{
    		/* "Failed to backup, exiting...\n" */
    		printk("\n[TSP][ERROR] line : %d\n", __LINE__);
    		return -EIO;
    	}
    	mdelay(100);
    	
/*	if( config_set_mode )       // 0 for battery, 1 for TA, 2 for USB
		tmp = set_all_config( config_TA_charging );
	else 
		tmp = set_all_config( config_normal );

	if( tmp == 1 )
		printk("[TSP] set all configs success : %d\n", __LINE__);
	else
		printk("[TSP] set all configs fail : error : %d\n", __LINE__);
*/
	return 0;
}
ssize_t set_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret, cmd_no,config_value = 0;
	char *after;

	printk(KERN_INFO "[TSP] %s : operate nothing\n", __FUNCTION__);

	return size;
}
/*------------------------------ for tunning ATmel - end ----------------------------*/


/* firmware 2009.09.24 CHJ - start 2/2 */

uint8_t boot_reset(void)
{
	uint8_t data = 0xA5u;
	return(write_mem(command_processor_address + RESET_OFFSET, 1, &data));
}

uint8_t QT_Boot(uint8_t qt_force_update)
{
	unsigned char boot_status = 0;
	unsigned char boot_ver = 0;
	unsigned long f_size = 0;
	unsigned long int character_position;
	unsigned int frame_size = 0;
	unsigned int next_frame = 0;
	unsigned int crc_error_count = 0;
	unsigned char rd_msg = 0;
	unsigned int retry_cnt = 0;
	unsigned int size1 = 0, size2 = 0;
	unsigned int i = 0, j = 0, read_status_flag = 0;
  
	// pointer 
	unsigned char *firmware_data ;

	firmware_ret_val = -1;

	// disable_irq for firmware update
	//disable_irq(tsp.irq);
	qt60224_notfound_flag = 1;
	disable_tsp_irq();

	firmware_data = firmware_602240;
	//    firmware_data = &my_code;

	f_size = sizeof(firmware_602240);
	//f_size = sizeof(my_code);
	if(qt_force_update == 0)
	{
	//Step 1 : Boot Reset
	if(boot_reset() == WRITE_MEM_OK)
		printk("[TSP] boot_reset is complete!\n");
	else 	
	{
		printk("[TSP] boot_reset is not complete!\n");
		return 0;	
	}

	}
	i=0;

	for(retry_cnt = 0; retry_cnt < 30; retry_cnt++)
	{
		printk("[TSP] %s,retry = %d\n", __func__, retry_cnt );

		msleep(60);

		if( (read_boot_state(&boot_status) == READ_MEM_OK) && (boot_status & 0xC0) == 0xC0) 
		{
			boot_ver = boot_status & 0x3F;
			crc_error_count = 0;
			character_position = 0;
			next_frame = 0;

			while(1)
			{
			        for(j=0;j<5;j++)
			        {
//				msleep(60);
				if(read_boot_state(&boot_status) == READ_MEM_OK)
				{
        				        read_status_flag = 1;
        				        break;
        				}
        				else
        				{
        				        read_status_flag = 0;
        				}
				}
				
				if(read_status_flag == 1)
				{
					if((boot_status & QT_WAITING_BOOTLOAD_COMMAND) == QT_WAITING_BOOTLOAD_COMMAND)
					{
						msleep(60);
						if(boot_unlock() == WRITE_MEM_OK)
						{
							msleep(60);
						}
						else
						{
							printk("[TSP] %s - boot_unlock fail!!%d\n", __func__, __LINE__ );
						}
					}
					else if((boot_status & 0xC0) == QT_WAITING_FRAME_DATA)
					{
						printk("[TSP] %s, sending packet %dth\n", __func__,i);
						/* Add 2 to frame size, as the CRC bytes are not included */
						size1 =  *(firmware_data+character_position);
						size2 =  *(firmware_data+character_position+1)+2;
						frame_size = (size1<<8) + size2;
//						msleep(20);

						/* Exit if frame data size is zero */
						next_frame = 1;
						boot_write_mem(frame_size, (firmware_data +character_position));
						i++;
//						msleep(10);

					}
					else if(boot_status == QT_FRAME_CRC_CHECK)
					{
//						msleep(60);
					}
					else if(boot_status == QT_FRAME_CRC_PASS)
					{
						if( next_frame == 1)
						{
							/* Ensure that frames aren't skipped */
							character_position += frame_size;
							next_frame = 0;
						}
					}
					else if(boot_status  == QT_FRAME_CRC_FAIL)
					{
						printk("[TSP] %s, crc fail %d times\n", __func__, crc_error_count);
						crc_error_count++;
					}
					if(crc_error_count > 10)
					{
						printk("[TSP] %s, crc fail %d times! firmware update fail!!!!\n", __func__, crc_error_count );
						return 0;
						//return QT_FRAME_CRC_FAIL;
					}
				}
				else
				{
					/* Complete Firmware Update!!!!! */
					printk("[TSP] %s, Complete update, Check the new Version!\n", __func__);
					/* save new slave address */
					//QT_i2c_address = I2C_address;

					/* Poll device */
					for(j=0;j<3;j++)
					{
					        msleep(60);
					if(read_mem(0, 1, &rd_msg) == READ_MEM_OK)
					{
						atmel_touch_probe();
					             printk("[TSP] %s, Complete update, Check the new Version!\n", __func__);
							return 1;
						}
					printk("[TSP] %s, After firmware update, read_mem fail - line %d\n", __func__, __LINE__ );
					}
					return 0;
				}
			}
		}
		else
		{
			printk("[TSP] read_boot_state() or (boot_status & 0xC0) == 0xC0) is fail!!!\n");
//			return 0;
		}

	}
	return 0;
}

int set_tsp_for_ta_detect(int state)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;
	uint8_t object_size;

	if(qt60224_notfound_flag == 1)
	{
		return -1;
	}

	if(is_boot_state == 2)
	{
		printk(KERN_DEBUG "[TSP] Postpone TA detection setting during boot\n");
		ta_state = state;
		return 1;
	}

	if(is_boot_state == 1)
	{
		is_boot_state = 2;
	}
	else
	{
		ta_state = state;
	}

	if(is_suspend_state == 1)
	{
		printk(KERN_DEBUG "[TSP] Postpone processing wakeup signal from TA\n");
		is_suspend_state = 2;
		return 1;
	}

	if(is_suspend_state != 2)
	{
		disable_tsp_irq();
	}

	if (down_interruptible(&g_tsp_mutex)) 
		return -ERESTARTSYS;

	if(state)
	{
		config_normal.touchscreen_config.tchthr = 60; //touchscreen_config.tchthr = 70;
		config_normal.noise_suppression_config.noisethr = 20; //noise_suppression_config.noisethr = 20;		   

		printk(KERN_DEBUG "[TSP] high noise mode\n");

		object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, 0);

		if (object_address == 0)
		{
	    	printk("\n[TSP][ERROR] TOUCH_MULTITOUCHSCREEN_T9 object_address : %d\n", __LINE__);
			enable_changeline_int();//enable_irq(qt602240->client->irq);
			up(&g_tsp_mutex); 
			return -1;
		}
		tmp= &config_normal.touchscreen_config.tchthr; // &touchscreen_config.tchthr;
		status = write_mem(object_address+7, 1, tmp);	
		
		if (status == WRITE_MEM_FAILED)
		{
		    printk("\n[TSP][ERROR] TOUCH_MULTITOUCHSCREEN_T9 write_mem : %d\n", __LINE__);
		}

		object_address = get_object_address(PROCG_NOISESUPPRESSION_T22, 0);

		if (object_address == 0)
		{
			printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T22 object_address : %d\n", __LINE__);
			enable_changeline_int();//enable_irq(qt602240->client->irq);
			up(&g_tsp_mutex); 
			return -1;
		}
		tmp= &config_normal.noise_suppression_config.noisethr; //&noise_suppression_config.noisethr ;
		status = write_mem(object_address+8, 1, tmp);	
		
		if (status == WRITE_MEM_FAILED)
		{
			printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T22 write_mem : %d\n", __LINE__);
		}	
		
    }
	else
	{
		fh_err_count = 0;
		config_normal.touchscreen_config.blen = 16;
		config_normal.touchscreen_config.tchthr = 32;//touchscreen_config.tchthr = 40;
		config_normal.noise_suppression_config.noisethr = 27; //noise_suppression_config.noisethr = 30;  
        	config_normal.noise_suppression_config.freq[0] = 29;
        	config_normal.noise_suppression_config.freq[1] = 34;
        	config_normal.noise_suppression_config.freq[2] = 39;
        	config_normal.noise_suppression_config.freq[3] = 49;
        	config_normal.noise_suppression_config.freq[4] = 58;
		config_normal.power_config.idleacqint = 64;

		printk(KERN_DEBUG "[TSP] normal mode\n");

		object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, 0);

		if (object_address == 0)
		{
			printk("\n[TSP][ERROR] TOUCH_MULTITOUCHSCREEN_T9 object_address : %d\n", __LINE__);
			enable_changeline_int();//enable_irq(qt602240->client->irq);
			up(&g_tsp_mutex); 
			return -1;
		}

		tmp= &config_normal.touchscreen_config.blen;
		status = write_mem(object_address+6, 1, tmp);

		tmp= &config_normal.touchscreen_config.tchthr; //&touchscreen_config.tchthr;
		status |= write_mem(object_address+7, 1, tmp);	

		if (status & WRITE_MEM_FAILED)
		{
			printk("\n[TSP][ERROR] TOUCH_MULTITOUCHSCREEN_T9 write_mem : %d\n", __LINE__);
		}

		object_address = get_object_address(PROCG_NOISESUPPRESSION_T22, 0);

		if (object_address == 0)
		{
			printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T22 object_address : %d\n", __LINE__);
			enable_changeline_int();//enable_irq(qt602240->client->irq);
			up(&g_tsp_mutex); 
			return -1;
		}
		tmp= &config_normal.noise_suppression_config.noisethr;//&noise_suppression_config.noisethr ;
		status = write_mem(object_address+8, 1, tmp);	
		
		tmp= &config_normal.noise_suppression_config.freq[0];
		status |= write_mem(object_address+11, 1, tmp);
		tmp= &config_normal.noise_suppression_config.freq[1];
		status |= write_mem(object_address+12, 1, tmp);	
		tmp= &config_normal.noise_suppression_config.freq[2];
		status |= write_mem(object_address+13, 1, tmp);	
		tmp= &config_normal.noise_suppression_config.freq[3];
		status |= write_mem(object_address+14, 1, tmp);	
		tmp= &config_normal.noise_suppression_config.freq[4];
		status |= write_mem(object_address+15, 1, tmp);

		if (status & WRITE_MEM_FAILED)
		{
			printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T22 write_mem : %d\n", __LINE__);
		}

		object_address = get_object_address(GEN_POWERCONFIG_T7, 0);

		if (object_address == 0)
		{
			printk("\n[TSP][ERROR] GEN_POWERCONFIG_T7 object_address : %d\n", __LINE__);
			enable_changeline_int();//enable_irq(qt602240->client->irq);
			up(&g_tsp_mutex); 
			return -1;
		}

		tmp = &config_normal.power_config.idleacqint;
		status |= write_mem(object_address, 1, tmp);

		if (status & WRITE_MEM_FAILED)
		{
			printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T7 write_mem : %d\n", __LINE__);
		}
	}

	if(is_suspend_state != 2)
	{
		enable_tsp_irq();
	}

	up(&g_tsp_mutex); 
	return 1;
}
EXPORT_SYMBOL(set_tsp_for_ta_detect);

void set_frequency_hopping_table(int mode)
{
	uint16_t object_address;
	uint8_t *tmp;
	uint8_t status;

	config_normal.touchscreen_config.blen = 0;
	config_normal.touchscreen_config.tchthr = 40;
	config_normal.noise_suppression_config.noisethr = 35;
	config_normal.power_config.idleacqint = 255;

	if(mode == 0) {
        	config_normal.noise_suppression_config.freq[0] = 29;
        	config_normal.noise_suppression_config.freq[1] = 34;
        	config_normal.noise_suppression_config.freq[2] = 39;
        	config_normal.noise_suppression_config.freq[3] = 49;
        	config_normal.noise_suppression_config.freq[4] = 58;
	}
	else if(mode == 1) {
        	config_normal.noise_suppression_config.freq[0] = 45;
        	config_normal.noise_suppression_config.freq[1] = 49;
        	config_normal.noise_suppression_config.freq[2] = 55;
        	config_normal.noise_suppression_config.freq[3] = 59;
        	config_normal.noise_suppression_config.freq[4] = 63;
	}
	else if(mode == 2) {
        	config_normal.noise_suppression_config.freq[0] = 10;
        	config_normal.noise_suppression_config.freq[1] = 12;
        	config_normal.noise_suppression_config.freq[2] = 18;
        	config_normal.noise_suppression_config.freq[3] = 40;
        	config_normal.noise_suppression_config.freq[4] = 72;
	}
	else if(mode == 3) {
        	config_normal.noise_suppression_config.freq[0] = 7;
        	config_normal.noise_suppression_config.freq[1] = 33;
        	config_normal.noise_suppression_config.freq[2] = 39;
        	config_normal.noise_suppression_config.freq[3] = 52;
        	config_normal.noise_suppression_config.freq[4] = 64;
	}

	object_address = get_object_address(GEN_POWERCONFIG_T7, 0);

	if (object_address == 0)
	{
		printk("\n[TSP][ERROR] GEN_POWERCONFIG_T7 object_address : %d\n", __LINE__);
		enable_changeline_int();//enable_irq(qt602240->client->irq);
		up(&g_tsp_mutex); 
		return -1;
	}

	tmp = &config_normal.power_config.idleacqint;
	status |= write_mem(object_address, 1, tmp);

	if (status & WRITE_MEM_FAILED)
	{
		printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T7 write_mem : %d\n", __LINE__);
	}

	object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, 0);

	if (object_address == 0)
	{
		printk("\n[TSP][ERROR] TOUCH_MULTITOUCHSCREEN_T9 object_address : %d\n", __LINE__);
		enable_changeline_int();//enable_irq(qt602240->client->irq);
		up(&g_tsp_mutex); 
		return -1;
	}

	tmp= &config_normal.touchscreen_config.blen;
	status = write_mem(object_address+6, 1, tmp);

	tmp= &config_normal.touchscreen_config.tchthr; //&touchscreen_config.tchthr;
	status |= write_mem(object_address+7, 1, tmp);	

	if (status & WRITE_MEM_FAILED)
	{
		printk("\n[TSP][ERROR] TOUCH_MULTITOUCHSCREEN_T9 write_mem : %d\n", __LINE__);
	}

	object_address = get_object_address(PROCG_NOISESUPPRESSION_T22, 0);

	if (object_address == 0)
	{
		printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T22 object_address : %d\n", __LINE__);
		enable_changeline_int();//enable_irq(qt602240->client->irq);
		up(&g_tsp_mutex); 
		return -1;
	}
	tmp= &config_normal.noise_suppression_config.noisethr;//&noise_suppression_config.noisethr ;
	status = write_mem(object_address+8, 1, tmp);	
	
	tmp= &config_normal.noise_suppression_config.freq[0];
	status |= write_mem(object_address+11, 1, tmp);
	tmp= &config_normal.noise_suppression_config.freq[1];
	status |= write_mem(object_address+12, 1, tmp);	
	tmp= &config_normal.noise_suppression_config.freq[2];
	status |= write_mem(object_address+13, 1, tmp);	
	tmp= &config_normal.noise_suppression_config.freq[3];
	status |= write_mem(object_address+14, 1, tmp);	
	tmp= &config_normal.noise_suppression_config.freq[4];
	status |= write_mem(object_address+15, 1, tmp);

	if (status & WRITE_MEM_FAILED)
	{
		printk("\n[TSP][ERROR] PROCG_NOISESUPPRESSION_T22 write_mem : %d\n", __LINE__);
	}
}

/* firmware 2009.09.24 CHJ - end 2/2 */

#ifdef ENABLE_NOISE_TEST_MODE
uint8_t read_uint16_t( uint16_t Address, uint16_t *Data )
{
	uint8_t status;
	uint8_t temp[2];

	status = read_mem(Address, 2, temp);
	*Data= ((uint16_t)temp[1]<<8)+ (uint16_t)temp[0];

	return (status);
}

uint8_t read_dbg_data(uint8_t dbg_mode , uint8_t node, uint16_t * dbg_data)
{
	uint8_t status;
	uint8_t mode,page;
	uint8_t read_page,read_point;

	if (down_interruptible(&g_tsp_mutex)) 
		return 0;
	if(is_suspend_state) {
		up(&g_tsp_mutex);
		return 0;
	}
  
	diagnostic_addr= get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
    
	read_page = node / 64;
	node %= 64;
	read_point = (node *2) + 2;
    
	//Page Num Clear
	status = diagnostic_chip(QT_CTE_MODE);
	msleep(10);

	do {
		status = read_mem(diagnostic_addr,1, &mode);
		if(status != READ_MEM_OK)
		{
			printk(KERN_INFO "[TSP] READ_MEM_FAILED \n");
			up(&g_tsp_mutex);		
			return READ_MEM_FAILED;
		}
	} while(mode != QT_CTE_MODE);
	 
	status = diagnostic_chip(dbg_mode);
	msleep(10);    
	
	do {
		status = read_mem(diagnostic_addr,1, &mode);
		if(status != READ_MEM_OK)
		{
			printk(KERN_INFO "[TSP] READ_MEM_FAILED \n");
			up(&g_tsp_mutex);
			return READ_MEM_FAILED;
		}
	} while(mode != dbg_mode);

    for(page = 1; page <= read_page;page++)
	{
		status = diagnostic_chip(QT_PAGE_UP);
		msleep(10);
		do {
			status = read_mem(diagnostic_addr+1,1, &mode);
			if(status != READ_MEM_OK)
			{
				printk(KERN_INFO "[TSP] READ_MEM_FAILED \n");			
				up(&g_tsp_mutex);
				return READ_MEM_FAILED;
			}
		} while(mode != page);
	}
	status = read_uint16_t(diagnostic_addr + read_point,dbg_data);  
	if(status != READ_MEM_OK)
	{
		printk(KERN_INFO "[TSP] READ_MEM_FAILED \n");			
		up(&g_tsp_mutex);
		return READ_MEM_FAILED;
	}

	msleep(10); 
	up(&g_tsp_mutex);    
	return (status);
}

ssize_t set_refer0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[0],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}

ssize_t set_refer1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[1],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}

ssize_t set_refer2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[2],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}


ssize_t set_refer3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[3],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}


ssize_t set_refer4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[4],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}

ssize_t set_refer5_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[5],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}


ssize_t set_refer6_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_refrence=0;

	status = read_dbg_data(QT_REFERENCE_MODE, test_node[6],&qt_refrence);
	return sprintf(buf, "%u\n", qt_refrence);	
}

ssize_t set_delta0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[0],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t set_delta1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[1],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t set_delta2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[2],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t set_delta3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[3],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t set_delta4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[4],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t set_delta5_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[5],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t set_delta6_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char status;
	uint16_t qt_delta=0;

	status = read_dbg_data(QT_DELTA_MODE, test_node[6],&qt_delta);
	if(qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);	
	}
	else {
		qt_delta = 65535 - qt_delta;
		if(qt_delta) return sprintf(buf, "-%u\n", qt_delta);
		else return sprintf(buf, "%u\n", qt_delta);
	}
}

ssize_t threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", config_normal.touchscreen_config.tchthr);
}
#endif
