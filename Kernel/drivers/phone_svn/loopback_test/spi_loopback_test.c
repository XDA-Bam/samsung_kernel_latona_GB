 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <plat/gpio.h>
#include <plat/hardware.h>
#include <plat/mux.h>
#include <plat/control.h>
#include <plat/display.h>

#include <asm/mach-types.h>

#include <linux/leds.h>


#if defined( __DEBUG )
#define dprintk( s, args... )			printk( "[SPI_LOOP] %s:%d - " s, __func__, __LINE__,  ##args )
#else
#define dprintk( s, args... )			do {} while( 0 );
#endif // _DEBUG


#define GPIO_CP_RST			43
#define GPIO_SRDY				181
#define GPIO_MRDY				182
#define GPIO_PDA_ACTVE		111
#define GPIO_PHONE_ACTIVE	154

#define DEF_BUF_SIZE			2044


static struct spi_device *ipc_spi_test_spi;

typedef struct spi_protocol_header_rec {
	u8 nDSRDTR;			// 1 bit
	u8 nRTSCTS;			// 1 bit
	u8 nDCD;				// 1 bit
	u8 nRI;					// 1 bit
	u16 nNext_Data_Size;		// 10 bit
	u8 nReserved;			// 2 bit
	u8 nPacket_Id;			// 2 bit
	u8 nRx_Error;			// 1 bit
	u8 nMore;				// 1 bit
	u16 nCurrent_Data_Size;	// 12 bit

	u32 spi_header;
} spi_protocol_header;

struct pdp_header {
	u32 len;		//Data length
	u8 id;		//Channel ID
	u8 control;	//Control field
} __attribute__ ( ( packed ) );


static u8 bof = 0x7F;
static u8 eof = 0x7E;

static u8 tx_buf[ 2048 ];
static u8 tx_save_buf[ 2048 ];
static u8 rx_buf[ 2048 ];

static int spi_setup_done = 0;

static u32 rx_data_count = 0;

static struct timer_list check_speed_timer;
static struct timer_list srdy_timeout_timer;

static int srdy_timeout_flag = 0;


static void spi_loopback_modem_power_on( void )
{
	gpio_set_value( GPIO_CP_RST, 1 );

	gpio_set_value( GPIO_PDA_ACTVE, 1 );
	
	printk( "[SPI_LOOP] Modem Power On.\n" );
}

static void spi_loopback_gpio_init( void )
{
	if( gpio_request( GPIO_SRDY, "SRDY" ) < 0 ) {
		printk( "[SPI_LOOP] SRDY gpio error.\n" );
	}
	gpio_direction_input( GPIO_SRDY );
	
	if( gpio_request( GPIO_MRDY, "MRDY" ) < 0 ) {
		printk( "[SPI_LOOP] MRDY gpio error.\n" );
	}
	gpio_direction_output( GPIO_MRDY, 0 );

	if( gpio_request( GPIO_CP_RST, "CP_RST" ) < 0 ) {
		printk( "[SPI_LOOP] CP_RST gpio error.\n" );
	}
	gpio_direction_output( GPIO_CP_RST, 0 );

	if( gpio_request( GPIO_PDA_ACTVE, "PDA_ACTIVE" ) < 0 ) {
		printk( "[SPI_LOOP] PDA_ACTIVE gpio error.\n" );
	}
	gpio_direction_output( GPIO_PDA_ACTVE, 1 );

	if( gpio_request( GPIO_PHONE_ACTIVE, "PHONE_ACTIVE" ) < 0 ) {
		printk( "[SPI_LOOP] PHONE_ACTIVE gpio error.\n" );
	}
	gpio_direction_input( GPIO_PHONE_ACTIVE );

	printk( "[SPI_LOOP] spi_loopback_gpio_init Done.\n" );
}

static void spi_loopback_test_make_spi_header( spi_protocol_header *header )
{
	header->spi_header = 0x00000000;

	header->spi_header = ( header->nDSRDTR << 31 ) & 0x80000000;
	header->spi_header |= ( header->nRTSCTS << 30 ) & 0x40000000;
	header->spi_header |= ( header->nDCD << 29 ) & 0x20000000;
	header->spi_header |= ( header->nRI << 28 ) & 0x10000000;
	header->spi_header |= ( header->nNext_Data_Size << 18 ) & 0x0FFC0000;
	header->spi_header |= ( header->nReserved << 16 ) & 0x00030000;
	header->spi_header |= ( header->nPacket_Id << 14 ) & 0x0000C000;
	header->spi_header |= ( header->nRx_Error << 13 ) & 0x00002000;
	header->spi_header |= ( header->nMore << 12 ) & 0x00001000;
	header->spi_header |= header->nCurrent_Data_Size & 0x00000FFF;

	dprintk( "spi_loopback_test_make_spi_header Done : 0x%08x\n", header->spi_header );
}

static void spi_loopback_test_make_init_cmd_tx_data( void )
{
	int i;
	spi_protocol_header write_header;
	u16 write_ipc_header;
	
	memset( tx_buf, 0, 2048 );

	write_header.nDSRDTR = 0;
	write_header.nRTSCTS = 0;
	write_header.nDCD = 0;
	write_header.nRI = 0;
	write_header.nNext_Data_Size = DEF_BUF_SIZE >> 2;
	write_header.nReserved = 0;
	write_header.nPacket_Id = 0;
	write_header.nRx_Error = 0;
	write_header.nMore = 0;
	write_header.nCurrent_Data_Size = 3;
	spi_loopback_test_make_spi_header( &write_header );

	tx_buf[ 0 ] = ( u8 )( write_header.spi_header >> 24 );
	tx_buf[ 1 ] = ( u8 )( write_header.spi_header >> 16 );
	tx_buf[ 2 ] = ( u8 )( write_header.spi_header >> 8 );
	tx_buf[ 3 ] = ( u8 )write_header.spi_header;

	//memcpy( tx_buf, &write_header.spi_header, sizeof( write_header.spi_header ) );

	write_ipc_header = 0x0004;
	memcpy( ( tx_buf + sizeof( write_header.spi_header ) ), &write_ipc_header, sizeof( write_ipc_header ) );

	tx_buf[ 6 ] = 0xC2;

	//printk( "[SPI_LOOP] TX : " );
	//for( i = 0 ; i < 20 ; i++ ) {
	//	printk( "%02x ", tx_buf[ i ] );
	//}
	//printk( "\n" );

	dprintk( "Prepare Tx Data Done.\n" );
}

static void spi_loopback_test_make_loopback_tx_data( void )
{
	int i;
	spi_protocol_header write_header;
	u16 write_ipc_header;
	struct pdp_header write_pdp_header;
	unsigned char test_data[ 10 ] = "0123456789";
	u32 loopback_data_size = 1500;
	
	memset( tx_buf, 0, 2048 );

	write_header.nDSRDTR = 0;
	write_header.nRTSCTS = 0;
	write_header.nDCD = 0;
	write_header.nRI = 0;
	write_header.nNext_Data_Size = DEF_BUF_SIZE >> 2;
	write_header.nReserved = 0;
	write_header.nPacket_Id = 0;
	write_header.nRx_Error = 0;
	write_header.nMore = 0;
	write_header.nCurrent_Data_Size = loopback_data_size + 10;
	spi_loopback_test_make_spi_header( &write_header );

	tx_buf[ 0 ] = ( u8 )( write_header.spi_header >> 24 );
	tx_buf[ 1 ] = ( u8 )( write_header.spi_header >> 16 );
	tx_buf[ 2 ] = ( u8 )( write_header.spi_header >> 8 );
	tx_buf[ 3 ] = ( u8 )write_header.spi_header;

	//memcpy( tx_buf, &write_header.spi_header, sizeof( write_header.spi_header ) );

	write_ipc_header = 0x0002;
	memcpy( ( tx_buf + sizeof( write_header.spi_header ) ), &write_ipc_header, sizeof( write_ipc_header ) );

	memcpy( ( tx_buf + sizeof( write_header.spi_header ) + sizeof( write_ipc_header ) ), &bof, sizeof( bof ) );

	write_pdp_header.control = 0;
	write_pdp_header.id = 31;
	write_pdp_header.len = sizeof( write_pdp_header ) + loopback_data_size;
	memcpy( ( tx_buf + sizeof( write_header.spi_header ) + sizeof( write_ipc_header ) + sizeof( bof ) ), &write_pdp_header, sizeof( write_pdp_header ) );

	for( i = 0 ; i < ( loopback_data_size / 10 ) ; i++ ) {
		memcpy( ( tx_buf + sizeof( write_header.spi_header ) + sizeof( write_ipc_header ) + sizeof( bof ) + sizeof( write_pdp_header ) + ( i * 10 ) ), &test_data, sizeof( test_data ) );	
	}

	memcpy( ( tx_buf + sizeof( write_header.spi_header ) + sizeof( write_ipc_header ) + sizeof( bof ) + sizeof( write_pdp_header ) + loopback_data_size ), &eof, sizeof( eof ) );	

	memcpy( tx_save_buf, tx_buf, sizeof( tx_save_buf ) );

	//printk( "[SPI_LOOP] TX : " );
	//for( i = 0 ; i < 20 ; i++ ) {
	//	printk( "%02x ", tx_buf[ i ] );
	//}
	//printk( "\n" );
	
	dprintk( "Prepare Tx Data Done.\n" );
}

static int spi_loopback_test_write_read( u8 * tx_d, u8 * rx_d )
{
	struct spi_transfer t;
	struct spi_message msg;
	
	memset( &t, 0, sizeof t );
	
	t.len = 2048;

	t.tx_buf = tx_d;
	t.rx_buf = rx_d;

	t.cs_change = 0;
	t.bits_per_word = 32;
	t.speed_hz = 24000000;

	spi_message_init( &msg );
	spi_message_add_tail( &t, &msg );

	return spi_sync( ipc_spi_test_spi, &msg );
}

static void spi_loopback_test_handshake( void )
{
	int retval = 0, i = 0;
	
	spi_loopback_test_make_init_cmd_tx_data();
	memset( rx_buf, 0, 2048 );

	gpio_set_value( GPIO_MRDY, 1 );
	dprintk( "MRDY set HIGH.\n" );

	dprintk( "Wait SRDY set HIGH...." );
	while( !gpio_get_value( GPIO_SRDY ) );
	dprintk( "SRDY set HIGH.\n" );

	retval = spi_loopback_test_write_read( tx_buf, rx_buf );
	if( retval != 0 ) {
		printk( "[SPI_LOOP] spi_loopback_test_write_read error : %d\n", retval );
	}
	else {
		dprintk( "[SPI_LOOP] spi_loopback_test_write_read Done.\n" );
	}

	//printk( "[SPI_LOOP] RX : " );
	//for( i = 0 ; i < 20 ; i++ ) {
	//	printk( "%02x ", rx_buf[ i ] );
	//}
	//printk( "\n" );

	dprintk( "Wait SRDY set LOW...." );
	while( gpio_get_value( GPIO_SRDY ) );
	dprintk( "SRDY set LOW.\n" );

	gpio_set_value( GPIO_MRDY, 0 );
	dprintk( "[SPI_LOOP] MRDY set LOW.\n" );

	printk( "[SPI_LOOP] spi_loopback_test_handshake Done.\n" );
}

static void spi_loopback_test_take_spi_header( spi_protocol_header * spi_header, u8 * spi_data )
{
	memset( spi_header, 0, sizeof( spi_header ) );
	
	spi_header->spi_header = spi_data[ 0 ] << 24;
	spi_header->spi_header |= spi_data[ 1 ] << 16;
	spi_header->spi_header |= spi_data[ 2 ] << 8;
	spi_header->spi_header |= spi_data[ 3 ];
	dprintk( "Read Spi Header : 0x%08x\n", spi_header->spi_header );

	spi_header->nDSRDTR = spi_header->spi_header >> 31;
	dprintk( "nDSRDTR : 0x%01x\n", spi_header->nDSRDTR );

	spi_header->nRTSCTS = ( spi_header->spi_header >> 30 ) & 0x01;
	dprintk( "nRTSCTS : 0x%01x\n", spi_header->nRTSCTS );

	spi_header->nDCD = ( spi_header->spi_header >> 29 ) & 0x01;
	dprintk( "nDCD : 0x%01x\n", spi_header->nDCD );

	spi_header->nRI = ( spi_header->spi_header >> 28 ) & 0x01;
	dprintk( "nRI : 0x%01x\n", spi_header->nRI );

	spi_header->nNext_Data_Size = ( spi_header->spi_header >> 18 ) & 0x03FF;
	dprintk( "nNext_Data_Size : 0x%04x\n", spi_header->nNext_Data_Size );

	spi_header->nReserved = ( spi_header->spi_header >> 16 ) & 0x03;
	dprintk( "nReserved : 0x%01x\n", spi_header->nReserved );

	spi_header->nPacket_Id = ( spi_header->spi_header >> 14 ) & 0x03;
	dprintk( "nPacket_Id : 0x%01x\n", spi_header->nPacket_Id );

	spi_header->nRx_Error = ( spi_header->spi_header >> 13 ) & 0x01;
	dprintk( "nRx_Error : 0x%01x\n", spi_header->nRx_Error );

	spi_header->nMore = ( spi_header->spi_header >> 12 ) & 0x01;
	dprintk( "nMore : 0x%01x\n", spi_header->nMore );

	spi_header->nCurrent_Data_Size = spi_header->spi_header & 0x0FFF;
	dprintk( "nCurrent_Data_Size : 0x%04x\n", spi_header->nCurrent_Data_Size );
}

static int spi_loopback_test_is_same_rx_tx( u8 * rcv_data )
{
	int retval = 0;
	
	if( memcmp( ( void * )( tx_save_buf + 13 ), ( void * )( rcv_data + 13 ), 1499 ) ) {
		retval = 0;
	}
	else {
		retval = 1;
	}

	return retval;
}

static int spi_loopback_test_thread( void *data )
{
	int retval = 0, i = 0;
	spi_protocol_header read_header;
	
	daemonize( "spi_loopback_test_thread" );

	printk( "[SPI_LOOP] Thread start.\n" );

	msleep( 3000 );

	// Send First Data( Handshake )
	spi_loopback_test_handshake();

	mod_timer( &check_speed_timer, jiffies + HZ );

	while( 1 ) {

LOOP :

		spi_loopback_test_make_loopback_tx_data();
		memset( rx_buf, 0, 2048 );

		gpio_set_value( GPIO_MRDY, 1 );
		dprintk( "MRDY set HIGH.\n" );
	
		srdy_timeout_flag = 0;

AGAIN :
		
		dprintk( "Wait SRDY set HIGH...." );
		while( !gpio_get_value( GPIO_SRDY ) ) {
			if( !timer_pending( &srdy_timeout_timer ) )
				mod_timer( &srdy_timeout_timer, jiffies + ( HZ / 20 ) ); // 50ms
				
			if( srdy_timeout_flag ) {
				dprintk( "SRDY TIME OUT.\n" );

				srdy_timeout_flag = 0;

				gpio_set_value( GPIO_MRDY, 0 );
				dprintk( "MRDY set LOW.\n" );

				goto LOOP;
			}
		}
		
		if( timer_pending( &srdy_timeout_timer ) )
			del_timer( &srdy_timeout_timer );
		
		dprintk( "SRDY set HIGH.\n" );

		retval = spi_loopback_test_write_read( tx_buf, rx_buf );
		if( retval != 0 ) {
			printk( "[SPI_LOOP] spi_loopback_test_write_read error : %d\n", retval );
		}
		else {
			dprintk( "spi_loopback_test_write_read Done.\n" );
		}

		//printk( "[SPI_LOOP] RX : " );
		//for( i = 0 ; i < 20 ; i++ ) {
		//	printk( "%02x ", rx_buf[ i ] );
		//}
		//printk( "\n" );

		spi_loopback_test_take_spi_header( &read_header, rx_buf );
		if( read_header.nCurrent_Data_Size != 0xFFF ) {			
			if( spi_loopback_test_is_same_rx_tx( rx_buf ) ) {
				dprintk( "tx-rx is same.\n" );

				rx_data_count += 1500;
			}
			else {
				dprintk( "tx-rx is NOT same.\n" );

				if( read_header.nMore ) {
					printk( "[SPI_LOOP] nMore is 1.\n" );
					
					memset( tx_buf, 0, 2048 );

					goto AGAIN;
				}
			}

			if( read_header.nRTSCTS ) {
				printk( "[SPI_LOOP] nRTSCTS is 1.\n" );
			}

			if( read_header.nDSRDTR ) {
				printk( "[SPI_LOOP] nDSRDTR is 1.\n" );
			}
		}

		srdy_timeout_flag = 0;
		
		dprintk( "Wait SRDY set LOW...." );
		while( gpio_get_value( GPIO_SRDY ) ) {
			if( !timer_pending( &srdy_timeout_timer ) )
				mod_timer( &srdy_timeout_timer, jiffies + ( HZ / 128 ) );
				
			if( srdy_timeout_flag ) {
				dprintk( "SRDY TIME OUT : %x.\n", read_header.nCurrent_Data_Size );

				srdy_timeout_flag = 0;
				
				break;
			}
		}
		
		if( timer_pending( &srdy_timeout_timer ) )
			del_timer( &srdy_timeout_timer );
		
		dprintk( "SRDY set LOW.\n" );

		gpio_set_value( GPIO_MRDY, 0 );
		dprintk( "MRDY set LOW.\n" );
		
	}
}

static void spi_loopback_test_check_speed_timer_func( unsigned long data )
{
	printk( "[SPI_LOOP] SPEED : %lu BytesPerSec.\n", rx_data_count );

	rx_data_count = 0;
	
	mod_timer( &check_speed_timer, jiffies + HZ );
}

static void spi_loopback_test_srdy_timeout_timer_func( unsigned long data )
{
	dprintk( "spi_loopback_test_srdy_timeout_timer_func\n" );
	
	srdy_timeout_flag = 1;
}

static int spi_loopback_test_probe( struct spi_device *spi )
{
	int retval = 0;

	ipc_spi_test_spi = spi;
	ipc_spi_test_spi->mode = SPI_MODE_1;
	ipc_spi_test_spi->bits_per_word = 32;

	retval = spi_setup( ipc_spi_test_spi );
	if( retval != 0 ) {
		printk( "[SPI_LOOP] spi_setup error : %d\n", retval );
	}
	else {
		printk( "[SPI_LOOP] spi_setup Done.\n" );
	}

	spi_loopback_gpio_init();

	spi_loopback_modem_power_on() ;

	spi_setup_done = 1;
	
	setup_timer( &check_speed_timer, spi_loopback_test_check_speed_timer_func, 0 );
	setup_timer( &srdy_timeout_timer, spi_loopback_test_srdy_timeout_timer_func, 0 );

	printk( "[SPI_LOOP] spi_loopback_test_probe Done.\n" );

	return 0;
}

static int spi_loopback_test_remove( struct spi_device *spi )
{
	return 0;
}
static void spi_loopback_test_shutdown( struct spi_device *spi )
{
}

static int spi_loopback_test_suspend( struct spi_device *spi, pm_message_t mesg )
{
	return 0;
}

static int spi_loopback_test_resume( struct spi_device *spi )
{
	return 0;
}

static struct spi_driver spi_loopback_test_spi_driver = {
	.probe = spi_loopback_test_probe,
	.remove = spi_loopback_test_remove,
	.shutdown = spi_loopback_test_shutdown,
	.suspend = spi_loopback_test_suspend,
	.resume = spi_loopback_test_resume,
	.driver = {
		.name = "spi_loopback_test",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
};

static int __init spi_loopback_test_init( void )
{
	int retval = 0;

	retval = kernel_thread( spi_loopback_test_thread, NULL, 0 );
	if( retval < 0 ) {
		printk( "[SPI_LOOP] kernel_thread() failed.\n" );
		
		return retval;
	}
	
	return spi_register_driver( &spi_loopback_test_spi_driver );
}

static void __exit spi_loopback_test_exit( void )
{
	return spi_unregister_driver( &spi_loopback_test_spi_driver );
}

module_init( spi_loopback_test_init );
module_exit( spi_loopback_test_exit );
MODULE_LICENSE( "GPL" );

