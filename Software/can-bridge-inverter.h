#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stddef.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/eeprom.h>
#include "Descriptors.h"
#include "LUFA/Platform/Platform.h"
#include "LUFA/Drivers/USB/USB.h"
#include "sp_driver.h"
#include "mcp25xx.h"
#include "helper_functions.h"

//function prototypes
void hw_init(void);
void print(char * str, uint8_t len);
void uint32_to_str(char * str, uint32_t num);
void SID_to_str(char * str, uint32_t num);
//void blink(void);
void canframe_to_str(char * str, can_frame_t frame);
void send_can(uint8_t can_bus, can_frame_t frame);
void send_can1(can_frame_t frame);
void send_can2(can_frame_t frame);
void send_can3(can_frame_t frame);
void can_handler(uint8_t can_bus);
void ProcessCDCCommand(void);
void check_can1(void);
void check_can2(void);
void check_can3(void);


//defines
#define TC0_CLKSEL_DIV1_gc		0b0001
#define TC0_CLKSEL_DIV256_gc	0b0110
#define TC0_CLKSEL_DIV1024_gc	0b0111
#define TC0_OVFINTLVL_HI_gc		0b11
#define TC0_OVFINTLVL_LO_gc		0b01
#define TC0_CCAINTLVL_HI_gc		0x03
#define TC0_CCBINTLVL_HI_gc		0x0C
#define TC0_CCCINTLVL_HI_gc		0x30
#define TC0_WGMODE_SINGLESLOPE_bm	0x03

#define OUTBUF_SIZE				2048
#define CURVOL_AVERAGES			100

#define		SHIFT_P		0x00
#define		SHIFT_D		0x40 //Modified, no longer following mux standard
#define		SHIFT_N		0x30
#define		SHIFT_R		0x20

#define TXBUFFER_SIZE			16

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
	.Config = {
		.ControlInterfaceNumber   = 0,
		.DataINEndpoint           = {
			.Address          = CDC_TX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.DataOUTEndpoint =	{
			.Address          = CDC_RX_EPADDR,
			.Size             = CDC_TXRX_EPSIZE,
			.Banks            = 1,
		},
		.NotificationEndpoint =	{
			.Address          = CDC_NOTIFICATION_EPADDR,
			.Size             = CDC_NOTIFICATION_EPSIZE,
			.Banks            = 1,
		},
	},
};
