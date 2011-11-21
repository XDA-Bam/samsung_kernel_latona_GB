
#ifndef _P1_KEYBOARD_H_
#define _P1_KEYBOARD_H_

#include <linux/input.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/earlysuspend.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/gpio.h>
#include <plat/gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define KEYBOARD_SIZE   128
#define US_KEYBOARD     0xeb
#define UK_KEYBOARD     0xec

#define KEYBOARD_MIN   0x4
#define KEYBOARD_MAX   0x7f

#define MAX_BUF     255

//for the remap key
#define REMAPKEY_RELEASED    0x0
#define REMAPKEY_PRESSED      0x1

#define ACC_EN_KBD

#define IRQ_ACCESSORY_INT      OMAP_GPIO_IRQ(OMAP_GPIO_ACCESSORY_INT)

//#define ACC_INT_KBD
//#define RESUME_KBD

enum KEY_LAYOUT
{
    UNKOWN_KEYLAYOUT = 0,
    US_KEYLAYOUT,
    UK_KEYLAYOUT,
};

  /* Each client has this additional data */
struct dock_keyboard_data
{
    struct input_dev *input_dev;
    struct work_struct work_msg;
    struct work_struct work_led;
    struct early_suspend	early_suspend;
    struct timer_list key_timer;
    bool led_on;
    int gpio;
    unsigned int kl;
};

struct key_table
{
    int keycode;
    bool pressed;
};

struct key_table dock_keycodes[KEYBOARD_SIZE] =
{
    // keycode              , pressed           decimal     hex
    {	KEY_RESERVED,       	 false},	//	0		0
    {	KEY_RESERVED,       	 false},	//	1		1
    {	KEY_RESERVED,       	 false},	//	2		2
    {	KEY_RESERVED,       	 false},	//	3		3
    {	KEY_A,              	 false},	//	4		4
    {	KEY_B,              	 false},	//	5		5
    {	KEY_C,              	 false},	//	6		6
    {	KEY_D,              	 false},	//	7		7
    {	KEY_E,              	 false},	//	8		8
    {	KEY_F,              	 false},	//	9		9
    {	KEY_G,              	 false},	//	10		0A
    {	KEY_H,              	 false},	//	11		0B
    {	KEY_I,              	 false},	//	12		0C
    {	KEY_J,              	 false},	//	13		0D
    {	KEY_K,              	 false},	//	14		0E
    {	KEY_L,              	 false},	//	15		0F
    {	KEY_M,              	 false},	//	16		10
    {	KEY_N,              	 false},	//	17		11
    {	KEY_O,              	 false},	//	18		12
    {	KEY_P,              	 false},	//	19		13
    {	KEY_Q,              	 false},	//	20		14
    {	KEY_R,              	 false},	//	21		15
    {	KEY_S,              	 false},	//	22		16
    {	KEY_T,              	 false},	//	23		17
    {	KEY_U,              	 false},	//	24		18
    {	KEY_V,              	 false},	//	25		19
    {	KEY_W,              	 false},	//	26		1A
    {	KEY_X,              	 false},	//	27		1B
    {	KEY_Y,              	 false},	//	28		1C
    {	KEY_Z,              	 false},	//	29		1D
    {	KEY_1,              	 false},	//	30		1E
    {	KEY_2,              	 false},	//	31		1F
    {	KEY_3,              	 false},	//	32		20
    {	KEY_4,              	 false},	//	33		21
    {	KEY_5,              	 false},	//	34		22
    {	KEY_6,              	 false},	//	35		23
    {	KEY_7,              	 false},	//	36		24
    {	KEY_8,              	 false},	//	37		25
    {	KEY_9,              	 false},	//	38		26
    {	KEY_0,              	 false},	//	39		27
    {	KEY_ENTER,          	 false},	//	40		28
    {	KEY_SCREENLOCK,          	 false},	//	41		29
    {	KEY_BACKSPACE,      	 false},	//	42		2A
    {	KEY_TAB,            	 false},	//	43		2B
    {	KEY_SPACE,          	 false},	//	44		2C
    {	KEY_MINUS,          	 false},	//	45		2D
    {	KEY_EQUAL,          	 false},	//	46		2E
    {	KEY_LEFTBRACE,      	 false},	//	47		2F
    {	KEY_RIGHTBRACE,     	 false},	//	48		30
    {	KEY_BACKSLASH,      	 false},	//	49		31
    {	KEY_RESERVED,       	 false},	//	50		32
    {	KEY_SEMICOLON,      	 false},	//	51		33
    {	KEY_APOSTROPHE,     	 false},	//	52		34
    {	KEY_GRAVE,          	 false},	//	53		35
    {	KEY_COMMA,          	 false},	//	54		36
    {	KEY_DOT,            	 false},	//	55		37
    {	KEY_SLASH,          	 false},	//	56		38
    {	KEY_CAPSLOCK,       	 false},	//	57		39
    {	KEY_TIME,           	 false},	//	58		3A
    {	KEY_BRIGHTNESSDOWN, 	 false},	//	59		3B
    {	KEY_BRIGHTNESSUP,   	 false},	//	60		3C
    {	KEY_WWW,            	 false},	//	61		3D
    {	KEY_MENU,           	 false},	//	62		3E
    {	KEY_HOME,           	 false},	//	63		3F
    {	KEY_BACK,           	 false},	//	64		40
    {	KEY_SEARCH,         	 false},	//	65		41
    {	KEY_MP3,            	 false},	//	66		42
    {	KEY_VIDEO,          	 false},	//	67		43
    {	KEY_PLAY,           	 false},	//	68		44
    {	KEY_REWIND,         	 false},	//	69		45
    {	KEY_MUTE,           	 false},	//	70		46
    {	KEY_RESERVED,       	 false},	//	71		47
    {	KEY_FASTFORWARD,    	 false},	//	72		48
    {	KEY_VOLUMEDOWN,     	 false},	//	73		49
    {	KEY_RESERVED,       	 false},	//	4A
    {	KEY_RESERVED,       	 false},	//	75		4B
    {	KEY_VOLUMEUP,       	 false},	//	76		4C
    {	KEY_RESERVED,       	 false},	//	77		4D
    {	KEY_RESERVED,       	 false},	//	78		4E
    {	KEY_RIGHT,          	 false},	//	79		4F
    {	KEY_LEFT,           	 false},	//	80		50
    {	KEY_DOWN,  	 false},	//	81		51
    {	KEY_UP,       	 false},	//	82		52
    {	KEY_NUMLOCK,        	 false},	//	83		53
    {	KEY_KPSLASH,        	 false},	//	84		54
    {	KEY_APOSTROPHE,     	 false},	//	85		55
    {	KEY_KPMINUS,        	 false},	//	86		56
    {	KEY_KPPLUS,         	 false},	//	87		57
    {	KEY_KPENTER,        	 false},	//	88		58
    {	KEY_KP1,            	 false},	//	89		59
    {	KEY_KP2,            	 false},	//	90		5A
    {	KEY_KP3,            	 false},	//	91		5B
    {	KEY_KP4,            	 false},	//	92		5C
    {	KEY_KP5,            	 false},	//	93		5D
    {	KEY_KP6,            	 false},	//	94		5E
    {	KEY_KP7,            	 false},	//	95		5F
    {	KEY_KP8,            	 false},	//	96		60
    {	KEY_KP9,            	 false},	//	97		61
    {	KEY_KPDOT,          	 false},	//	98		62
    {	KEY_RESERVED,       	 false},	//	99		63
    {	KEY_BACKSLASH,      	 false},	//	100		64      //For the UK keyboard
    {	KEY_F18,           	 false},	//	101		65
    {	KEY_RESERVED,       	 false},	//	102		66
    {	KEY_RESERVED,       	 false},	//	103		67
    {	KEY_RESERVED,       	 false},	//	104		68
    {	KEY_RESERVED,       	 false},	//	105		69
    {	KEY_RESERVED,       	 false},	//	106		6A
    {	KEY_RESERVED,       	 false},	//	107		6B
    {	KEY_RESERVED,       	 false},	//	108		6C
    {	KEY_RESERVED,       	 false},	//	109		6D
    {	KEY_RESERVED,       	 false},	//	110		6E
    {	KEY_RESERVED,       	 false},	//	111		6F
    {	KEY_HANGEUL,        	 false},	//	112		70
    {	KEY_HANJA,          	 false},	//	113		71
    {	KEY_F13,            	 false},	//	114		72
    {	KEY_LEFTSHIFT,      	 false},	//	115		73
    {	KEY_F16,            	 false},	//	116		74
    {	KEY_F17,            	 false},	//	75		Left GUI (Windows Key)
    {	KEY_F19,            	 false},	//	118		76
    {	KEY_RIGHTSHIFT,     	 false},	//	119		77
    {	KEY_F15,            	 false},	//	120		78
    {	KEY_RESERVED,       	 false},	//	121		79		Right GUI (Windows Key)
    {	KEY_RESERVED,       	 false},	//	122		7A
    {	KEY_RESERVED,       	 false},	//	123		7B
    {	KEY_RESERVED,       	 false},	//	124		7C
    {	KEY_RESERVED,       	 false},	//	125		7D
    {	KEY_RESERVED,       	 false},	//	126		7E
    {	KEY_F14,            	 false},	//	127		7F

};

#endif  //_P1_KEYBOARD_H_
