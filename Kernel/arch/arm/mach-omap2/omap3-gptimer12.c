#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <plat/omap3-gptimer12.h>
#include <plat/dmtimer.h>
#include <plat/prcm.h>
#include "prm.h"
#include "prcm-common.h"
#include "cm.h"
#include <asm/string.h>

#define MAX_GPTIMER12_INSTANCE 5
#define PRCM_GPT12 OMAP3430_EN_GPT12_MASK
#define PRCM_ENABLE 1
#define PRCM_DISABLE 0

static struct omap_dm_timer *battery_timer;
//extern strcmp(char* , char *);
#define GP_TIMER12_SEC_TO_TICK(secs)   (clk_get_rate(omap_dm_timer_get_fclk(battery_timer)))*(secs)

#define GP_TIMER12_TICK_TO_SEC(ticks)  (clk_get_rate(omap_dm_timer_get_fclk(battery_timer)))/(ticks)

struct gptimer12_manager timer_manager[MAX_GPTIMER12_INSTANCE];
unsigned int gptimer12_count;
static volatile u32 cm_val=0;
#if 0
int init_gptimer12 (void);
EXPORT_SYMBOL(init_gptimer12);

int finish_gptimer12 (void);
EXPORT_SYMBOL(finish_gptimer12);

int request_gptimer12(struct gptimer12_timer *timer);
EXPORT_SYMBOL(request_gptimer12);

int release_gptimer12(struct gptimer12_timer *timer);
EXPORT_SYMBOL(release_gptimer12);

int expire_gptier12(void);
#endif
static int prcm_wakeup_event_control ( u32 reg_bit, u8 operation )
{
	volatile u32 prcm_val=0;

	prcm_val = prm_read_mod_reg( WKUP_MOD, PM_WKEN1 );
	if ( operation )
		prcm_val = prcm_val | reg_bit;
	else
		prcm_val = prcm_val & ~reg_bit;

	prm_write_mod_reg( prcm_val, WKUP_MOD, PM_WKEN1 );

	return 0;
}

//static irqreturn_t timer_interrupt(int irq, void *dev_id); DELETE this

int init_gptimer12 ( void )
{
	gptimer12_count = 0;
	memset( &timer_manager, 0, sizeof( timer_manager ) );
	battery_timer = omap_dm_timer_request_specific(12);
	BUG_ON( battery_timer == NULL );

	omap_dm_timer_set_source( battery_timer, OMAP_TIMER_SRC_32_KHZ );
	//battery_timer_irq.dev_id = (void *)battery_timer;
	//setup_irq(omap_dm_timer_get_irq(battery_timer), &battery_timer_irq);
	//omap_dm_timer_set_int_enable(battery_timer, OMAP_TIMER_INT_OVERFLOW);
	//omap_dm_timer_set_prescaler(battery_timer, GP_TIMER12_PRESCALAR);
	omap_dm_timer_write_status( battery_timer, OMAP_TIMER_INT_OVERFLOW | OMAP_TIMER_INT_CAPTURE
	                                                | OMAP_TIMER_INT_MATCH );

	omap_dm_timer_disable( battery_timer );

    return 0;
}
EXPORT_SYMBOL(init_gptimer12);

int finish_gptimer12 ( void )
{
	omap_dm_timer_stop( battery_timer );
	omap_dm_timer_write_status( battery_timer, OMAP_TIMER_INT_OVERFLOW );
	omap_dm_timer_set_int_enable( battery_timer, 0 );

	return 0;

}
EXPORT_SYMBOL(finish_gptimer12);

int request_gptimer12( struct gptimer12_timer *timer )
{
	int loop_count;
	int empty_slot = -1;
	unsigned int next_time = 0;
	unsigned int current_time = 0;
	//unsigned int temp; 

	unsigned char update_flag = 0;
	// printk("request_gptimer12 called \n");

	if ( gptimer12_count >= MAX_GPTIMER12_INSTANCE ) 
	{
		printk( "Error... max gptimer12 instance" );
		return -1;
	}
	//CM_ICLKEN_WKUP |= 1 << 1;
	cm_val = cm_read_mod_reg( WKUP_MOD, CM_ICLKEN1 );
	cm_val = cm_val | ( 1 << 1 );
	cm_write_mod_reg( cm_val, WKUP_MOD, CM_ICLKEN1 );

	//modify exist entry 
	for ( loop_count = 0; loop_count < MAX_GPTIMER12_INSTANCE; loop_count++ )
	{
		if ( timer_manager[loop_count].timer.name != NULL ) 
		{
			if( strcmp( timer_manager[loop_count].timer.name, timer->name ) == 0 ) 
			{
				//printk("timer update \n");
				memcpy( &( timer_manager[loop_count].timer ), timer, sizeof( struct gptimer12_timer ) );
				timer_manager[loop_count].remain_time = timer->expire_time;
				update_flag = 1;
			}
		}
	}

	next_time = timer->expire_time;
	if ( update_flag == 0 ) 
	{
		//add new entry 
		for ( loop_count = 0; loop_count < MAX_GPTIMER12_INSTANCE; loop_count++ )
		{
			if ( ( timer_manager[loop_count].timer.name ) == NULL )
			{
				empty_slot = loop_count;
				break;
			}
		}
		//printk("empty_slot : %d, loop_count : %d\n",empty_slot,loop_count);

		if ( empty_slot == -1 )
		{
			printk("Error.. No empty slot ");
			return -1;
		}

		gptimer12_count++;

		memcpy(&(timer_manager[empty_slot].timer),timer, sizeof(struct gptimer12_timer));
		timer_manager[empty_slot].remain_time = timer->expire_time;

		// printk("test3 : gptimer12_count : %d\n",gptimer12_count);

		if ( gptimer12_count == 1 )
		{
			omap_dm_timer_enable( battery_timer );
			prcm_wakeup_event_control( PRCM_GPT12, PRCM_ENABLE );
			omap_dm_timer_write_status( battery_timer, OMAP_TIMER_INT_OVERFLOW );
			omap_dm_timer_set_int_enable( battery_timer, OMAP_TIMER_INT_OVERFLOW );

			for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
			{
				if ( timer_manager[loop_count].timer.name != NULL )
					timer_manager[loop_count].remain_time -= next_time;
			}
			omap_dm_timer_set_load_start( battery_timer, 0, 0xffffffff - GP_TIMER12_SEC_TO_TICK(next_time) );
			//CM_ICLKEN_WKUP &= ~(1 << 1);
#if 0
			cm_val = cm_read_mod_reg(WKUP_MOD,CM_ICLKEN1);
			cm_val = cm_val&~(1<<1);
			cm_write_mod_reg(cm_val,WKUP_MOD,CM_ICLKEN1);
#endif

			timer->active = true;

			return 1;
		}
	}

	omap_dm_timer_stop( battery_timer ); //:commented this

#if 1
	current_time = GP_TIMER12_TICK_TO_SEC( 0xffffffff - omap_dm_timer_read_counter(battery_timer) );
#endif
	for ( loop_count = 0; loop_count < MAX_GPTIMER12_INSTANCE; loop_count++ )
	{
		timer_manager[loop_count].remain_time += current_time;
#if 0 
		if((timer_manager[loop_count].timer.name) != 0)
		{
			if((timer_manager[loop_count].remain_time) == 0)
			timer_manager[loop_count].remain_time = current_time; 
		}
#endif
	}

	for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
	{
		if ( timer_manager[loop_count].timer.name != NULL )
		{
			next_time = timer_manager[loop_count].remain_time;
			break;
		}
	}

	for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
	{
		if ( timer_manager[loop_count].timer.name != NULL )
		{
			if ( next_time > timer_manager[loop_count].remain_time )
				next_time = timer_manager[loop_count].remain_time;
		}
	}

	for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
	{
		if ( timer_manager[loop_count].timer.name != NULL )
			timer_manager[loop_count].remain_time -= next_time;
	}

	// timer
	prcm_wakeup_event_control( PRCM_GPT12, PRCM_ENABLE );
	omap_dm_timer_set_int_enable( battery_timer, OMAP_TIMER_INT_OVERFLOW );
	omap_dm_timer_set_load_start( battery_timer, 0, 0xffffffff - GP_TIMER12_SEC_TO_TICK(next_time) );
	//CM_ICLKEN_WKUP &= ~(1 << 1);
#if 0
	cm_val = cm_read_mod_reg(WKUP_MOD,CM_ICLKEN1);
	cm_val = cm_val&~(1<<1);
	cm_write_mod_reg(cm_val,WKUP_MOD,CM_ICLKEN1);
	// printk("requested gptimer12_count : %d \n",gptimer12_count);
#endif

	timer->active = true;

	return 1;

}
EXPORT_SYMBOL(request_gptimer12);

int release_gptimer12(struct gptimer12_timer *timer)
{
	int loop_count;
	int temp_count;
	int slot = -1;
	unsigned int next_time=0;
	int current_time;

	if ( gptimer12_count == 0 )
		return -1;

	//printk ("\ntest 1 timer_instance_count : %d \n",gptimer12_count);

	for ( loop_count = 0; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ ) 
	{
		if ( timer_manager[loop_count].timer.name != NULL)
		{
			if( strcmp( timer->name, timer_manager[loop_count].timer.name ) == 0 )
			{
				slot = loop_count;
				break;
			}
		}
	}

	if ( slot == -1 )
		return -1;

	// case : delete current working timer 
	//CM_ICLKEN_WKUP |= 1 << 1;
	cm_val = cm_read_mod_reg( WKUP_MOD, CM_ICLKEN1 );
	cm_val = cm_val | ( 1 << 1 );
	cm_write_mod_reg( cm_val, WKUP_MOD, CM_ICLKEN1 );

	if ( timer_manager[slot].remain_time == 0 )
	{
		//current timer ..
		omap_dm_timer_stop( battery_timer );
		current_time = GP_TIMER12_TICK_TO_SEC( 0xffffffff - omap_dm_timer_read_counter( battery_timer ) );
		//printk("test1 current_time : %d\n",current_time);
		for ( loop_count = 0; loop_count < MAX_GPTIMER12_INSTANCE; loop_count++ )
		{
			timer_manager[loop_count].remain_time += current_time;
#if 0 
			if((timer_manager[loop_count].timer.name) != 0)
			{
				if((timer_manager[loop_count].remain_time) == 0)
					timer_manager[loop_count].remain_time = current_time; 

			}
#endif
		}
	}

	memset ( &( timer_manager[slot] ), 0, sizeof( struct gptimer12_manager ) );
	gptimer12_count--;
	
	if( gptimer12_count < 0 )
		gptimer12_count = 0;

	//printk("released : timer_instance_count : %d\n",gptimer12_count);
	if ( gptimer12_count == 0 )
	{
		//printk("\n\n M        timer empty .. \n"); 
		prcm_wakeup_event_control( PRCM_GPT12, PRCM_DISABLE ); //egkim
		finish_gptimer12(); //egkim
		omap_dm_timer_disable( battery_timer );

		//CM_ICLKEN_WKUP &= ~(1 << 1);
		cm_val = cm_read_mod_reg( WKUP_MOD, CM_ICLKEN1 );
		cm_val = cm_val & ~( 1 << 1 );
		cm_write_mod_reg( cm_val, WKUP_MOD,CM_ICLKEN1 );


		return 0;
	}

	for ( temp_count = 0 ; temp_count < MAX_GPTIMER12_INSTANCE ; temp_count++ )
	{
		if( timer_manager[temp_count].timer.name != NULL )
		{
			next_time = timer_manager[temp_count].remain_time;
			break;
		}
	}

	for ( temp_count = 0 ; temp_count < MAX_GPTIMER12_INSTANCE ; temp_count++ )
	{
		if ( timer_manager[temp_count].timer.name != NULL )
		{
			next_time = timer_manager[temp_count].remain_time;
			break;
		}
	}

	for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
	{
		if( timer_manager[loop_count].timer.name != NULL )
		{
			if ( next_time > timer_manager[loop_count].remain_time )
				next_time = timer_manager[loop_count].remain_time;
		}
	}

	for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
	{
		if( timer_manager[loop_count].timer.name != NULL )
			timer_manager[loop_count].remain_time -= next_time;
	}
	printk( "\n\n\n next timeout : %d\n",next_time );
	prcm_wakeup_event_control( PRCM_GPT12, PRCM_ENABLE );
	omap_dm_timer_set_load_start( battery_timer, 0, 0xffffffff - GP_TIMER12_SEC_TO_TICK(next_time) );

	//CM_ICLKEN_WKUP &= ~(1 << 1);
	cm_val = cm_read_mod_reg( WKUP_MOD, CM_ICLKEN1 );
	cm_val = cm_val & ~( 1 << 1 );
	cm_write_mod_reg( cm_val, WKUP_MOD,CM_ICLKEN1 );

	timer->active = false;

	return 0;
}
EXPORT_SYMBOL(release_gptimer12);

int expire_gptimer12( void )
{
	int loop_count;
	int callback_result = 0;
	//int result;

// egkim [
	//omap_dm_timer_write_status( battery_timer, OMAP_TIMER_INT_OVERFLOW );
	//omap_dm_timer_set_int_enable( battery_timer, 0 );
	//finish_gptimer12(); // Commented on Latona GB
// ]
	omap_dm_timer_disable( battery_timer );
	//printk("expire_gptimer12 called \n");
	for ( loop_count = 0 ; loop_count < MAX_GPTIMER12_INSTANCE ; loop_count++ )
	{
		if ( timer_manager[loop_count].timer.name != NULL ) 
		{
			if ( timer_manager[loop_count].remain_time == 0 )
			{
				//printk(KERN_ERR "call callback function[%s] \n",timer_manager[loop_count].timer.name);
				//callback_result = timer_manager[loop_count].timer.expire_callback(&(timer_manager[loop_count].timer));
				
				callback_result = timer_manager[loop_count].timer.expire_callback( timer_manager[loop_count].timer.data );

				//result = release_gptimer12(&(timer_manager[loop_count].timer));
				//printk("timer release result : %d\n",result); 
			}
		}
	}
	return callback_result;
}
EXPORT_SYMBOL(expire_gptimer12);



