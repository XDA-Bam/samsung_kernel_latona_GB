#define IMPROVE_SOURCE // KMJ_DA18

typedef enum
{
    DEFAULT_MODEL,
    LATONA,
    MODEL_TYPE_MAX
} Atmel_model_type;

typedef enum
{
    VERSION_1_2,
    VERSION_1_4,
    VERSION_1_5,
} Atmel_fw_version_type;


// [[ ryun
#define ATEML_TOUCH_DEBUG 0
#if ATEML_TOUCH_DEBUG
#define dprintk(flag, fmt, args...) printk( "[ATMEL]%s: " fmt, __func__ , ## args)	// ryun !!!  ???
static bool en_touch_log = 1;
#else
#define dprintk(flag, fmt, args...) /* */
static bool en_touch_log = 0;
#endif
#define LONG(x) ((x)/BITS_PER_LONG)
// ]] ryun

/* firmware 2009.09.24 CHJ - start 1/2 */
#define QT602240_I2C_BOOT_ADDR 0x24
#define QT_WAITING_BOOTLOAD_COMMAND 0xC0
#define QT_WAITING_FRAME_DATA       0x80
#define QT_FRAME_CRC_CHECK          0x02
#define QT_FRAME_CRC_PASS           0x04
#define QT_FRAME_CRC_FAIL           0x03

#define WRITE_MEM_OK                1u
#define WRITE_MEM_FAILED            2u
#define READ_MEM_OK                 1u
#define READ_MEM_FAILED             2u

/* firmware 2009.09.24 CHJ - end 1/2 */

#define ENABLE_NOISE_TEST_MODE
#define MAX_TOUCH_NUM	5
#define MAX_TRACKING_ID	MAX_TOUCH_NUM
#define RETRY_COUNT 10

