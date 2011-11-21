#include <asm/io.h>
#include <linux/clocksource.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <plat/mux.h>

#define PHY_VINTANA2_DEDICATED      0xA1
#define T2_VINTANA2_2V75            0x1
#define T2_VINTANA2_OFF             1<<6

#define CARKIT_ANA_CTRL 0xBB
#define SEL_MADC_MCPC   0x08

#define REG_VCELL	0x2
#define REG_SOC     0x4
#define REG_MODE	0x6
#define REG_VERSION 0x8
#define REG_RCOMP	0xC
#define REG_COMMAND	0xFE

/*MADC*/
#define CTRL1               0x00
#define SW1SELECT_LSB       0x06
#define SW1SELECT_MSB       0x07
#define SW1AVERAGE_LSB      0x08
#define SW1AVERAGE_MSB      0x09
#define CTRL_SW1            0x12
#define GPCH0_LSB           0x37
#define GPCH0_MSB           0x38

#define CTRL1_MADC_ON       0x01
#define CTRL_SW1_SW1        0x20


#define ADCIN0  0	/* BCI : Battery Type */
#define ADCIN1  1		/* BCI : Battery Temperature */
#define ADCIN2  2		/* General-purpose analog input */
#define ADCIN3  3	/* General-purpose analog input */
#define ADCIN4  4	/* General-purpose analog input */
#define ADCIN5  5		/* General-purpose analog input */
#define ADCIN6  6	/* General-purpose analog input */
#define ADCIN7  7	/* General-purpose analog input */
#define ADCIN8  8	/* BCI : VBUS voltage */
#define ADCIN9  9	/* BCI : Backup Battery Voltage */
#define ADCIN10 10  /* BCI : Battery Charger Current */
#define ADCIN11 11  /* BCI : Battery Charger Voltage */
#define ADCIN12 12  /* BCI : Main Battery Volatge */
#define ADCIN13 13  /* Reserved */
#define ADCIN14 14  /* Reserved */
#define ADCIN15 15  /* VRBUS supply or speaker right or
        	         * speaker left polarization level
		             */

struct madc_chrequest {
	int channel;
	unsigned int data;
	int is_average;
};

struct madc_chrequest adc_request[6]={
	{
	.channel=0,
	.is_average=0,
	},
	{
	.channel=1,
	.is_average=0,
	},
	{
	.channel=2,
	.is_average=0,
	},
	{
	.channel=3,
	.is_average=0,
	},
	{
	.channel=4,
	.is_average=0,
	},
	{
	.channel=5,
	.is_average=0,
	},
};

s32 t2_write(u8 devaddr, u8 value, u8 regoffset);
s32 t2_read(u8 devaddr, u8 *value, u8 regoffset);
s32 normal_i2c_read_word(u8 devaddr, u8 regoffset, u8 *value);


s32 clear_set(u8 mod_no, u8 clear, u8 set, u8 reg);
s32 t2_madc_convert(struct madc_chrequest *ch_request, u32 count);
s32 t2_adc_data( u8 channel);
int sleep_get_max17040_battery_adc(void);
int sleep_get_max17040_battery_soc(void);
int sleep_max17040_read(u8 reg);
int get_average_adc_value(unsigned int * data, int count);


s32 clear_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	s32 ret;
	u8 data = 0;

	/* Gets the initial register value */
	ret = t2_read(mod_no, &data, reg);
	if (ret)
		return ret;
	/* Clearing all those bits to clear */
	data &= ~(clear);
	/* Setting all those bits to set */
	data |= set;
	ret = t2_write(mod_no, data, reg);
	/* Update the register */
	if (ret)
		return ret;
	return 0;
}

#define NO_MADC_POWER_CTRL
s32 t2_madc_convert(struct madc_chrequest *ch_request, u32 count)
{
	s32 ret = 0;
	u32 i=0;
	u32 rd_val;
	u8 data, val;
	u32 msb, lsb;

#ifndef NO_MADC_POWER_CTRL
	ret = t2_write(0x4A, CTRL1_MADC_ON, CTRL1);
	if (ret) {
		printk("Could Not turn on MADC module\n");
		return ret;
	}

	ret = t2_read(0x4A, &data, CTRL1);
	if (ret) {
		printk("Could Not turn on MADC module\n");
		return ret;
	}
#endif

	for (i = 0; i < count; i++) { 
		if (ch_request[i].channel < ADCIN8) {
			ret = clear_set(0x4A, 0, 1 << ch_request[i].channel, SW1SELECT_LSB);
			if (ret)
			{
				goto out;
			}
			if (ch_request[i].is_average) {
				ret = clear_set(0x4A, 0, 1 << ch_request[i].channel, SW1AVERAGE_LSB);
				if (ret)
					goto out;
			}
		} else {
			ret = clear_set(0x4A, 0, 1 << (ch_request[i].channel - 8), SW1SELECT_MSB);
			if (ret)
				goto out;
			if (ch_request[i].is_average) {
				ret = clear_set(0x4A, 0, 1 << (ch_request[i].channel - 8), SW1AVERAGE_MSB);
				if (ret)
					goto out;
			}
		}
	}

	/* Start the conversion process*/

	ret = clear_set(0x4A, 0, CTRL_SW1_SW1, CTRL_SW1);
	if (ret)
		goto out;

	i=0;
	t2_read(0x4A, &val, 0x12/*CTRL_SW1*/);
	while(1)
	{
		i++;
		if( !(val & 0x1/*TWL4030_MADC_BUSY*/) && (val & 0x2/*TWL4030_MADC_EOC_SW*/) )
			break;

		t2_read(0x4A, &val, 0x12/*CTRL_SW1*/);
	}

	for (i = 0; i < count; i++) {
		ret = t2_read(0x4A, &data, GPCH0_MSB + 2 * ch_request[i].channel);
		if (ret)
			goto out;
		msb = data;
		//rd_val = data << 2;

		ret = t2_read(0x4A, &data, GPCH0_LSB + 2 * ch_request[i].channel);
		if (ret)
			goto out;

		lsb = data;
		//rd_val = rd_val | (data >> 6);
		rd_val = (((msb << 8) | lsb) >> 6);
		//printk("The read value is %x\n", rd_val);
		//printk("rd_val: %d, ch: %d\n", rd_val, ch_request[i].channel);
		//rd_val = CONV_VOLTAGE(rd_val, 5/*ch_request[i].channel*/);
		ch_request[i].data = rd_val;
		//printk("The SW1 conv value from channel %d is %d\n", ch_request[i].channel, ch_request[i].data);
	}
    
out:
	/*
	 * Clear the channel select and averaging registers so that
	 * next time when a request comes the current channels are
	 * not converted unnecessarily
	 */
	t2_write(0x4A, 0, SW1SELECT_LSB);
	t2_write(0x4A, 0, SW1SELECT_MSB);
	t2_write(0x4A, 0, SW1AVERAGE_LSB);
	t2_write(0x4A, 0, SW1AVERAGE_MSB);
	/* The module is no longer busy doing any cconversion */

#ifndef NO_MADC_POWER_CTRL
	ret = clear_set(0x4A, CTRL1_MADC_ON, 0, CTRL1);
	if (ret)
		printk("Could Not turn off MADC module\n");
#endif
	return ret;
}

s32 t2_adc_data( u8 channel)
{
	s32 ret=0;

	int i;
	unsigned int data[5];

#ifdef CONFIG_MACH_SAMSUNG_LATONA
    // To control thermal sensor power
   	omap_set_gpio_dataout_in_sleep(OMAP_GPIO_EN_TEMP_VDD, 1);
#endif
	t2_write(0x48, SEL_MADC_MCPC, CARKIT_ANA_CTRL); 

    #if 0
    if(channel == 0) {
        t2_read(0x4A,&regval,0x97);
        regval |= 0x10;	
        t2_write(0x4A, regval, 0x97);
    }
    mdelay(10);
    #endif

	for(i=0 ; i<5 ; i++)
	{
        ret = t2_madc_convert(&adc_request[channel], 1);
		if(ret)
			printk("[TA] Fail to get ADC data\n");

		data[i]=adc_request[channel].data;

      	//printk("ADC : %d\n", data[i]);
	}

    ret = get_average_adc_value(data, 5);

#ifdef CONFIG_MACH_SAMSUNG_LATONA
    // To control thermal sensor power
    omap_set_gpio_dataout_in_sleep(OMAP_GPIO_EN_TEMP_VDD, 0);
#endif

    #if 0
    // PM_RECEIVER [
    printk("## pm start ##\n");
    for (i = 0x5B; i <= 0xF1; i ++)
    {
    	t2_read(0x4b, &val, i);
        printk("%x: %x\n", i, val);
    }
    printk("## pm end ##\n");
    // PM_RECEIVER ]

    // USB [
    printk("## usb start ##\n");
    for (i = 0; i <= 0xFF; i ++)
    {
    	t2_read(0x48, &val, i);
        printk("%x: %x\n", val);
    }
    printk("## usb end ##\n");
    // USB ]

    // MADC [
    #if 1
    printk("## madc start ##\n");
    for (i = 0x0; i <= 0x67; i ++)
    {
    	t2_read(0x4a, &val, i);
        printk("%x: %x\n", i, val);
    }
    printk("## madc end ##\n");
    #endif
    // MADC ]
    #endif

	return ret;
}

int get_average_adc_value(unsigned int * data, int count)
{
	int i=0, average, min=0xFFFFFFFF, max=0, total=0;
	for(i=0 ; i<count ; i++)
	{
		if(data[i] < min)
			min=data[i];
		if(data[i] > max)
			max=data[i];

		total+=data[i];
	}

	average = (total - min -max)/(count -2);
	return average;
}
