

#include <linux/slab.h>
#include "p1_keyboard.h"

extern u32 hw_revision;

struct dock_keyboard_data *g_data;
static bool handshaking = false;
static bool remap_state = false;
static unsigned int pre_kl;
static unsigned long connected_time=0;
static int buf_front = 0;
static int buf_rear = 0;
static unsigned char key_buf[MAX_BUF] = {0,};
static int en_gpio;

/* to contol the gpio in sleep mode */
bool keyboard_enable = false;
EXPORT_SYMBOL(keyboard_enable);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void keyboard_early_suspend(struct early_suspend *);
static void keyboard_late_resume(struct early_suspend *);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

extern struct class *sec_class;
extern void dock_keyboard_tx(u8 val);
extern int change_console_baud_rate(int baud);
extern void set_uart_suspend(int value);

void remapkey_timer(unsigned long data)
{
    unsigned int keycode = 0;
    if((dock_keycodes[0x45].pressed)||(dock_keycodes[0x48].pressed))
    {
        remap_state = REMAPKEY_PRESSED;
        keycode = dock_keycodes[data].keycode;
        input_report_key(g_data->input_dev,keycode, 1);
        input_sync(g_data->input_dev);
    }
    else
    {
        remap_state = REMAPKEY_RELEASED;

        if(data == 0x48)
        {
            keycode = KEY_NEXTSONG;
        }
        else
        {
            keycode = KEY_PREVIOUSSONG;
        }

#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
        printk(KERN_DEBUG "[Keyboard] remap key %d\n", keycode);
#endif

        input_report_key(g_data->input_dev, keycode, 1);
        input_report_key(g_data->input_dev, keycode, 0);
        input_sync(g_data->input_dev);
    }
}

void release_all_keys(void)
{
    int i;
    //printk(KERN_DEBUG "[Keyboard] Release the pressed keys.\n");
    for(i = 0; i < KEYBOARD_MAX; i++)
    {
        if(dock_keycodes[i].pressed)
        {
            input_report_key(g_data->input_dev, dock_keycodes[i].keycode, 0);
            dock_keycodes[i].pressed = false;
        }
    }
}

static void key_event_work(struct work_struct *work)
{
    bool press;
    static int release_cnt = 0;
    unsigned int keycode;
    unsigned char scan_code;
    struct dock_keyboard_data *data = container_of(work,
                struct dock_keyboard_data, work_msg);

    while(buf_front != buf_rear)
    {
        buf_front = (1+buf_front)%MAX_BUF;
        scan_code = key_buf[buf_front];

        /* keyboard driver need the contry code*/
        if(data->kl == UNKOWN_KEYLAYOUT)
        {
            switch(scan_code)
            {
                case US_KEYBOARD:
                    data->kl = US_KEYLAYOUT;
                    dock_keycodes[49].keycode = KEY_BACKSLASH;
                    /* for the wakeup state*/
                    pre_kl = data->kl;
                    printk(KERN_DEBUG "[Keyboard] US keyboard is attacted.\n");
                    break;

                case UK_KEYBOARD:
                    data->kl = UK_KEYLAYOUT;
                    dock_keycodes[49].keycode = KEY_NUMERIC_POUND;
                    /* for the wakeup state*/
                    pre_kl = data->kl;
                    printk(KERN_DEBUG "[Keyboard] UK keyboard is attacted.\n");
                    break;

                default:
                    printk(KERN_DEBUG "[Keyboard] Unkown key layout : %x\n", scan_code);
                    break;
            }
        }
        else
        {
            /* Do not send the key_event durring the handshake time */
            if(handshaking)
            {
                /* Caps lock led on/off */
                if(scan_code == 0xca || scan_code == 0xcb)
                {
                    // Ignore
                    //dock_keyboard_tx(scan_code);
                }
                else if(scan_code == 0x0)
                {
                    release_all_keys();
                    release_cnt = 16;
                }
                else
                {
                    press = ((scan_code & 0x80) != 0x80);
                    keycode = (scan_code & 0x7f);

                    if(keycode >= KEYBOARD_MIN || keycode <= KEYBOARD_MAX)
                    {
                        if(press)
                        {
                            // workaround keyboard issue
                            if(dock_keycodes[keycode].pressed)
                            {
                                input_report_key(data->input_dev, dock_keycodes[keycode].keycode, 0);
                                msleep(1);
                            }
                            dock_keycodes[keycode].pressed = true;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
                            printk(KERN_DEBUG "[Keyboard] %d key is pressed.\n", dock_keycodes[keycode].keycode);
#endif
                        }
                        else
                        {
                            dock_keycodes[keycode].pressed = false;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
                            printk(KERN_DEBUG "[Keyboard] %d key is released.\n", dock_keycodes[keycode].keycode);
#endif
                        }

                        /* for the remap keys*/
                        if(keycode == 0x45 || keycode == 0x48)
                        {
                            if(press)
                            {
                                data->key_timer.data = (unsigned long) keycode;
                                mod_timer(&data->key_timer, jiffies + HZ/3);
                            }
                            else
                            {
                                if(remap_state == REMAPKEY_PRESSED)
                                {
                                    remap_state = REMAPKEY_RELEASED;
                                    input_report_key(data->input_dev, dock_keycodes[keycode].keycode, press);
                                    input_sync(data->input_dev);
                                }
                            }
                        }
                        else
                        {
                            input_report_key(data->input_dev, dock_keycodes[keycode].keycode, press);
                            input_sync(data->input_dev);
                        }
                    }
/* This is not working */
#if 0
                    else
                    {
                        /* request last scancode again*/
                        dock_keyboard_tx(0xaa);
                        printk(KERN_DEBUG "[Keyboard] wrong key_code : 0x%x\n", scan_code);
                    }
#endif
                    if(release_cnt >0)
                    {
                        if(press)
                        {
                            input_report_key(data->input_dev, dock_keycodes[keycode].keycode, 0);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
                            printk(KERN_DEBUG "[Keyboard] forced release %d key.\n", dock_keycodes[keycode].keycode);
#endif
                        }
                        release_cnt--;
                    }
                }
            }
            else
            {
                /* device don't send the key_event durring the handshaking */
                if((jiffies - connected_time) >= HZ/2)
                {
                    handshaking =true;
                }
            }
        }
    }

}

static void led_work(struct work_struct *work)
{
    struct dock_keyboard_data *data = container_of(work,
                struct dock_keyboard_data, work_led);

    if(data->led_on)
    {
        //caps lock led on
        dock_keyboard_tx(0xca);
        msleep(100);
        dock_keyboard_tx(0xca);
    }
    else
    {
        //caps lock led off
        dock_keyboard_tx(0xcb);
        msleep(100);
        dock_keyboard_tx(0xcb);
    }
}

int check_keyboard_dock(void)
{
    static bool dockconnected = false;
    static bool pre_connected =false;
    static bool pre_uart_path =false;
    static unsigned long disconnected_time=0;
    int try_cnt = 0;
    int error = 0;
    int max_cnt = 10;

    if(gpio_get_value(g_data->gpio))
    {
        dockconnected = false;
    }
    else
    {
        pre_connected = true;

        /*for checking handshake*/
        connected_time = jiffies;

        if((connected_time - disconnected_time) < HZ)
        {
            g_data->kl = pre_kl;
//            dockconnected = true;
            printk(KERN_DEBUG "[Keyboard] kl : %d\n", pre_kl);
       }
        else
        {
            pre_kl = UNKOWN_KEYLAYOUT;
        }

// TEMP BLOCK        set_uart_suspend(0);
        /* if the uart is set as cp path, the path should be switched to ap path.*/
        pre_uart_path = gpio_get_value(OMAP_GPIO_UART_SEL);
        if(!pre_uart_path)
        {
            gpio_set_value(OMAP_GPIO_UART_SEL, 1);
            printk(KERN_DEBUG "[Keyboard] console uart path is switched to AP.\n");
        }

        /* Set baud rate for the keyboard uart */
        error = change_console_baud_rate(9600);
        if(error<0)
        {
            printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");
            goto con_err;
        }

        if(!keyboard_enable)
        {
	printk("[Keyboard] check_keyboard_dock 6");
           gpio_set_value(en_gpio, 1);
            keyboard_enable = true;
        }

        if(!dockconnected)
        {
            /* try to get handshake data */
            for(try_cnt=0; try_cnt<max_cnt; try_cnt++)
            {

                msleep(100);
                if(g_data->kl != UNKOWN_KEYLAYOUT)
                {
                    dockconnected = true;
                    break;
                }

                /* the accessory is dettached. */
                if(gpio_get_value(g_data->gpio))
                {
                    dockconnected = false;
                    break;
                }
            }
        }
    }

    if(dockconnected)
    {
        // if the led is on, the led should be off.
        g_data->led_on= false;
        if (!work_pending(&g_data->work_led))
        {
            schedule_work(&g_data->work_led);
        }
        return 1;
    }
    else
    {
        if(pre_connected)
        {
            gpio_set_value(en_gpio, 0);
            keyboard_enable = false;

            error = change_console_baud_rate(115200);
            if(error<0)
            {
                printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");
            }
con_err:
            dockconnected = false;
            gpio_set_value(OMAP_GPIO_UART_SEL, pre_uart_path);

            g_data->kl = UNKOWN_KEYLAYOUT;
            pre_connected = false;
            handshaking = false;
            disconnected_time = jiffies;
            release_all_keys();
        }
// TEMP BLOCK        set_uart_suspend(1);
        return 0;
    }
}

EXPORT_SYMBOL(check_keyboard_dock);

void send_keyevent(unsigned int key_code)
{
    buf_rear = (1+buf_rear)%MAX_BUF;
    if( buf_front == buf_rear )
    {
        if(buf_rear == 0)
        {
            buf_rear = MAX_BUF;
        }
        else
        {
            buf_rear--;
        }
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
        printk(KERN_DEBUG "[Keyboard] miss the key_code : %x\n", key_code);
#endif
    }
    else
    {
        key_buf[buf_rear]  = (unsigned char)key_code;
    }
//    printk(KERN_DEBUG "[Keyboard] key_code : %x\n", key_code);
    if (!work_pending(&g_data->work_msg))
    {
        schedule_work(&g_data->work_msg);
    }
}

EXPORT_SYMBOL(send_keyevent);

static ssize_t caps_lock_led(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
//    struct dock_keyboard_data *data = dev->platform_data;
    int i=0;
    //printk(KERN_DEBUG "[Keyboard] Caps lock led : %d.\n", g_data->led_on);
    if(sscanf(buf,"%d",&i)==1)
    {
        if(i == 1)
        {
            g_data->led_on = true;
        }
        else
        {
            g_data->led_on = false;
        }
    }
    else
    {
        printk(KERN_ERR "[Keyboard] Couldn't get led state.\n");
    }

    if (!work_pending(&g_data->work_led))
    {
        schedule_work(&g_data->work_led);
    }

    return size;
}
static DEVICE_ATTR(keyboard_led, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, NULL, caps_lock_led);

static int __devinit dock_keyboard_probe(struct platform_device *pdev)
{
//    struct dock_keyboard_data *data = pdev->dev.platform_data;
    struct dock_keyboard_data *data;
    struct input_dev *input;
    int i, error;
#if defined(ACC_INT_KBD)
    int gpio, irq;
#endif
    struct device *keyboard_dev;

    data = kzalloc(sizeof(struct dock_keyboard_data), GFP_KERNEL);
    if(NULL == data)
    {
        error = -ENOMEM;
        goto err_free_mem;
    }

    INIT_WORK(&data->work_msg, key_event_work);
    INIT_WORK(&data->work_led, led_work);

    input = input_allocate_device();
    if (!input)
    {
        printk(KERN_ERR "[Keyboard] Fail to allocate input device.\n");
        error = -ENOMEM;
        goto err_free_mem;
    }

    data->input_dev = input;
    data->kl = UNKOWN_KEYLAYOUT;

    input->name = pdev->name;
    input->dev.parent = &pdev->dev;
    input->id.bustype = BUS_RS232;

    set_bit(EV_SYN, input->evbit);
//    set_bit(EV_REP, input->evbit);
    set_bit(EV_KEY, input->evbit);

    for(i = 0; i < KEYBOARD_SIZE; i++)
    {
        if( KEY_RESERVED != dock_keycodes[i].keycode)
        {
            input_set_capability(input, EV_KEY, dock_keycodes[i].keycode);
        }
    }

    /* for the UK keyboard */
    input_set_capability(input, EV_KEY, KEY_NUMERIC_POUND);

    /* for the remaped keys */
    input_set_capability(input, EV_KEY, KEY_NEXTSONG);
    input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);

    error = input_register_device(data->input_dev);
    if(error<0)
    {
        printk(KERN_ERR "[Keyboard] Fail to register input device.\n");
        error = -ENOMEM;
        goto err_free_mem;
    }

	printk("[Keyboard] hw_revisiion = %d.. \n", hw_revision);

#if ( (defined( CONFIG_MACH_SAMSUNG_P1LITE ) || defined( CONFIG_MACH_SAMSUNG_P1WIFI )) && ( CONFIG_SAMSUNG_REL_HW_REV >= 3 ) )
    if(hw_revision < 4)
        en_gpio = OMAP_GPIO_KEY_LED_EN;
    else
#endif
        en_gpio = OMAP_GPIO_ACCESSORY_EN;

    /* Accessory detect pin is used by dock accessory driver. */
#if defined(ACC_INT_KBD)
    gpio = OMAP_GPIO_ACCESSORY_INT;
    if(gpio_is_valid(gpio))
    {
        if(gpio_request(gpio, NULL))
        {
            printk("[TSP] Can't request ENABLE gpio\n");
        }
        gpio_direction_input(gpio);
    }
    irq = IRQ_ACCESSORY_INT;

    error = request_irq(irq, accessory_interrupt,
                    IRQF_SAMPLE_RANDOM|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			"p1_keyboard", data);
    if(error)
    {
        printk(KERN_ERR "[Keyboard] Fail to request irq : %d\n", error);
        error = -EINTR;
        goto err_free_mem;
    }
    data->gpio = gpio;
#else
    data->gpio = OMAP_GPIO_ACCESSORY_INT;
#endif
    g_data = data;

    keyboard_dev = device_create(sec_class, NULL, 0, NULL, "keyboard");
    if (IS_ERR(keyboard_dev))
        pr_err("Failed to create device(ts)!\n");

    if (device_create_file(keyboard_dev, &dev_attr_keyboard_led) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_keyboard_led.attr.name);

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = keyboard_early_suspend;
	data->early_suspend.resume = keyboard_late_resume;
	register_early_suspend(&data->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

    init_timer(&data->key_timer);
    data->key_timer.expires = jiffies + HZ/2;
    data->key_timer.function = remapkey_timer;

    return 0;

err_free_mem:
    input_free_device(input);
    kfree(data);
    return error;

}

static int __devexit dock_keyboard_remove(struct platform_device *pdev)
{
//    struct dock_keyboard_data *pdata = pdev->dev.platform_data;
    input_unregister_device(g_data->input_dev);
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void keyboard_early_suspend(struct early_suspend *early_sus)
{
    //caps lock led off
    dock_keyboard_tx(0xcb);
    msleep(100);
    dock_keyboard_tx(0x10);
}

static void keyboard_late_resume(struct early_suspend *early_sus)
{
}
#else // Do not use below until OMAP uart driver is fixed
static int dock_keyboard_suspend(struct platform_device *pdev, pm_message_t state)
{
    //caps lock led off
    dock_keyboard_tx(0xcb);
    msleep(100);
    dock_keyboard_tx(0x10);
    return 0;
}

static int dock_keyboard_resume(struct platform_device *pdev)
{
}
#endif

static struct platform_driver dock_keyboard_device_driver =
{
    .probe		= dock_keyboard_probe,
    .remove	= __devexit_p(dock_keyboard_remove),
#if !defined(CONFIG_HAS_EARLYSUSPEND)
    .suspend	= dock_keyboard_suspend,
    .resume	= dock_keyboard_resume,
#endif
    .driver		=
    {
    	.name	= "p1_keyboard",
    	.owner	= THIS_MODULE,
    }
};

static int __init dock_keyboard_init(void)
{
    return platform_driver_register(&dock_keyboard_device_driver);
}

static void __exit dock_keyboard_exit(void)
{
    platform_driver_unregister(&dock_keyboard_device_driver);
}

late_initcall(dock_keyboard_init);
module_exit(dock_keyboard_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEC P series Dock Keyboard driver");
