
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

// System and identification registers map
#define REG_ID_VER			0x02
#define REG_SYS_CTRL1		0x03
#define REG_SYS_CTRL2		0x04
#define REG_SPI_CFG			0x08
#define REG_INT_CTRL		0x09
#define REG_INT_EN			0x0A
#define REG_INT_STA			0x0B

// Analog to Digital Converter
#define REG_ADC_CTRL1		0x20
#define REG_ADC_CTRL2		0x21
#define REG_ADC_CAPT		0x22
#define REG_ADC_DATA_CH0	0x30
#define REG_ADC_DATA_CH1	0x32
#define REG_ADC_DATA_CH2	0x34
#define REG_ADC_DATA_CH3	0x36
#define REG_ADC_DATA_CH4	0x38
#define REG_ADC_DATA_CH5	0x3A
#define REG_ADC_DATA_CH6	0x3C
#define REG_ADC_DATA_CH7	0x3E

#define REG_ADC_INT_EN		0x0E
#define REG_ADC_INT_STA		0x0F


#define STMPE811_SLAVE_ADDR 0x41 // 44

#if defined(CONFIG_USE_GPIO_I2C)
    #include <plat/i2c-omap-gpio.h>

    static OMAP_GPIO_I2C_CLIENT * stmpe811_i2c_client;
#endif    


struct stmpe811_data {
	struct device		*dev;
	struct mutex		lock;
	struct work_struct	ws;
	int isr;
};


struct stmpe811_adc_platform_data {
	int		irq_line;
};

static struct stmpe811_data *the_madc;


static u8 stmpe811_i2c_read(unsigned char reg_addr)
{
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
    unsigned char buf;

	if(stmpe811_i2c_client == NULL)
	{
		printk("stmpe811_i2c_write i2c is not ready!!");
		return -1;
	}

    i2c_rd_param.reg_len = 1;
    i2c_rd_param.reg_addr = &reg_addr;
    i2c_rd_param.rdata_len = 1;
    i2c_rd_param.rdata = &buf;
    omap_gpio_i2c_read(stmpe811_i2c_client, &i2c_rd_param);

	return buf;
}

static u16 stmpe811_i2c_read_2byte(unsigned char reg_addr)
{
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
    unsigned char buf[2];
	u16 ret;

	if(stmpe811_i2c_client == NULL)
	{
		printk("stmpe811_i2c_write i2c is not ready!!");
		return -1;
	}

    i2c_rd_param.reg_len = 1;
    i2c_rd_param.reg_addr = &reg_addr;
    i2c_rd_param.rdata_len = 2;
    i2c_rd_param.rdata = buf;
    omap_gpio_i2c_read(stmpe811_i2c_client, &i2c_rd_param);

	ret = (buf[0] << 8) | buf[1];

	return ret;
}

static void stmpe811_i2c_write(u8 addr, u8 val)
{
	OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
	unsigned char buf[1] = { val };
	int capt_count = 0;

	if(stmpe811_i2c_client == NULL)
	{
		printk("stmpe811_i2c_write i2c is not ready!!");
		return;
	}

    i2c_wr_param.reg_addr = &addr;
    i2c_wr_param.reg_len = 1;
    i2c_wr_param.wdata_len = 1;
    i2c_wr_param.wdata = buf;
    omap_gpio_i2c_write(stmpe811_i2c_client, &i2c_wr_param);
}

int get_adc_conversion(int ch)
{
	int ret;
	int capt_count = 0;
	u16 ret_val;

	mutex_lock(&the_madc->lock);

	stmpe811_i2c_write(REG_ADC_CAPT, (0x1 << ch));
	while(!(stmpe811_i2c_read(REG_ADC_CAPT) & (0x1 << ch)))
	{
		printk("get_adc_conversion : wait capture adc \n");
		if(capt_count++ > 20)
			goto adc_timeout;
	}
	ret_val = stmpe811_i2c_read_2byte(REG_ADC_DATA_CH0 + (ch * 2));
	ret_val = ret_val & 0xFFF;

	mutex_unlock(&the_madc->lock);

	// Conversion : ADC data = Vin / Vcc * 4095
	// Vin = ADC data/4095*3000 (mV)
	ret = ret_val * 3000 / 4095;

	return ret;
adc_timeout:
	mutex_unlock(&the_madc->lock);
	printk("[stmpe811] adc timeout!!\n");
	ret = -1;
	return ret;
}
EXPORT_SYMBOL(get_adc_conversion);

int get_id_ver(void)
{
	return stmpe811_i2c_read(REG_ID_VER);
}
EXPORT_SYMBOL(get_id_ver);

static int __init stmpe811_probe(struct platform_device *pdev)
{
	struct stmpe811_data *madc;
	
	int ret = 0;
	u8 regval = 0;
	unsigned char buf[3];

	printk("stmpe811_probe 1\n");
	madc = kzalloc(sizeof *madc, GFP_KERNEL);
	if (!madc)
		return -ENOMEM;

	platform_set_drvdata(pdev, madc);
	mutex_init(&madc->lock);


	/* Enabling the Soft Reset in the System Control Register (0x03)*/
	stmpe811_i2c_write(REG_SYS_CTRL1, 0x02);
	/* Enabling the ADC clock in the System Control Register (0x04)*/
	stmpe811_i2c_write(REG_SYS_CTRL2, 0x0a);
	
	/* Setting up the ADC - Sampling Time = 36, 10 bit ADC, Internal Referenced */
	stmpe811_i2c_write(REG_ADC_CTRL1, 0x08);
	/* Setting up the ADC Clock Speed = 1.6Mhz*/
	stmpe811_i2c_write(REG_ADC_CTRL2, 0x00);
	/* Enable the ADC Interrupt for IN0, IN2, IN3 (DISABLE - Don't use interrupt) */
	stmpe811_i2c_write(REG_ADC_INT_EN, 0x00);
	/* Enabling the Global Interrupt in the Interrupt Control Register (0x09) (DISABLE - Don't use interrupt)*/
	stmpe811_i2c_write(REG_INT_CTRL, 0x00);
	/* Enabling the interrupt for the ADC (DISABLE - Don't use interrupt) */
	stmpe811_i2c_write(REG_INT_EN, 0x00);

	/* Clear all interrupts */
	//stmpe811_i2c_write(REG_INT_STA, 0xFF);
	/* Initiate Data Acquisition on IN0, IN2, IN3 */
	//stmpe811_i2c_write(REG_ADC_CAPT, 0x0B);

	the_madc = madc;

	return 0;

err_pdata:
	kfree(madc);

	return ret;
}

static int __exit stmpe811_remove(struct platform_device *pdev)
{
	//struct stmpe811_data *madc = platform_get_drvdata(pdev);
	return 0;
}

static struct platform_driver stmpe811_driver = {
	.probe		= stmpe811_probe,
	.remove		= __exit_p(stmpe811_remove),
	.driver		= {
		.name	= "stmpe811",
		.owner	= THIS_MODULE,
	},
};

static int __init stmpe811_init(void)
{
    stmpe811_i2c_client = omap_gpio_i2c_init(OMAP_GPIO_LEVEL_SDA, OMAP_GPIO_LEVEL_SCL, STMPE811_SLAVE_ADDR, 100);

	printk("[ADC IC] STMPE811 Init. \n");
    if(stmpe811_i2c_client == NULL)
    {
        printk(KERN_ERR "[ADC IC] omap_gpio_i2c_init failed!\n");
    }
	
	return platform_driver_register(&stmpe811_driver);
}
module_init(stmpe811_init);

static void __exit stmpe811_exit(void)
{
    printk("[FG] Fuelgauge Exit.\n");

    omap_gpio_i2c_deinit(stmpe811_i2c_client);
	platform_driver_unregister(&stmpe811_driver);
}
module_exit(stmpe811_exit);

MODULE_ALIAS("platform:STMPE811");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("STMPE811 driver");
MODULE_LICENSE("GPL");

