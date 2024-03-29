
/*
 INVERTER UPGRADE FIRMWARE
When fitted between the inverter and VCM [EV-CAN], this CAN-bridge firmware allows for higher power output. For this, we need to:
 - Modify the commanded torque up
 - Return a lower reported motor torque
 - Send a bunch of 2018+ CAN messages that the inverter expects to keep it error free
 - Optionally, protect the fuse by limiting output at low SOC%, for instance if you run a 24kWh pack (SMALLBATTERY_IN_USE)
*/

/* Choose your inverter */
#define LEAF_110kW
//#define LEAF_160kW

/* Choose to enable fuse safeguard features*/
//#define SMALLBATTERY_IN_USE //Use with 24 kWh packs

/* Choose to enable regen tuning*/
#define REGEN_TUNING_ENABLED

#define TORQUE_MULTIPLIER_110 1.32 									//(1.28 is too small, results in 118kW and 1.37 results in 122kW)
#define TORQUE_MULTIPLIER_160 1.40									//For a more aggressive pedal feel, multipliers between 1.4 and 1.6 can be used

#define REGEN_MULTIPLIER 1.10

/* Do not make any changes to the rows below unless you are sure what you are doing */

#include "can-bridge-inverter.h"

volatile	uint16_t	torqueDemand			= 0; //NM * 4
volatile	uint16_t	torqueResponse			= 0; //NM * 2
volatile	uint16_t	VCMtorqueDemand			= 0; //NM * 4
volatile	uint16_t	current					= 0;
volatile	uint8_t		battery_soc				= 0;
volatile	uint8_t		shift_state				= 0;
volatile	uint8_t		charging_state			= 0;
volatile	uint8_t		eco_screen				= 0;

#define SHIFT_DRIVE		4
#define SHIFT_ECO		5
#define SHIFT_REVERSE	2
#define SHIFT_NEUTRAL	3
#define SHIFT_PARK		0

#define ECO_OFF			1
#define ECO_ON			2

#define CHARGING_QUICK_START	0x40
#define CHARGING_QUICK			0xC0
#define CHARGING_QUICK_END		0xE0
#define CHARGING_SLOW			0x20
#define CHARGING_IDLE			0x60

// CAN messages used for deleting P3197 [EV/HEV] & P318E [MOTOR CONTROL] DTC (Send 4B9)
volatile	can_frame_t	inv_4B9_message	= {.can_id = 0x4B9, .can_dlc = 1, .data = {0x40}};
volatile	uint8_t		content_4B9			= 0x40;

// Sending 355 does not fix any DTC (but probably good to send it anyway)
// Sending 625 removes U215B [HV BATTERY]
// Sending 5C5 (355 at 40ms) removes U214E [HV BATTERY] and U1000 [MOTOR CONTROL] 
// Sending 3B8 and 5EB removes U1000 and P318E [HV BATTERY]
	volatile	uint8_t		PRUN10MS			= 0;
	
	static		can_frame_t	inv_605_message	= {.can_id = 0x605, .can_dlc = 1, .data = {0x00}};
	static		can_frame_t	inv_607_message	= {.can_id = 0x607, .can_dlc = 1, .data = {0x00}};

	//This message is 40kWh specific (maybe on
	volatile	can_frame_t	inv_1C2_message	= {.can_id = 0x1C2, .can_dlc = 1, .data = {0x50}};
	volatile	uint8_t		content_1C2			= 0x50;

	volatile	can_frame_t	inv_108_message	= {.can_id = 0x108, .can_dlc = 3, .data = {0x00,0x00,0x00}};
	volatile	uint8_t		content_108_1			= 0x00;
	volatile	uint8_t		content_108_2			= 0x00;
	static		uint8_t		lookuptable_crc_108[16] = {0x00,0x85,0x8F,0x0A,0x9B,0x1e,0x14,0x91,0xb3,0x36,0x3c,0xb9,0x28,0xad,0xa7,0x22};

	//volatile	can_frame_t	inv_1CB_message	= {.can_id = 0x1cb, .can_dlc = 7, .data = {0x00,0x00,0x00,0x02,0x60,0x00,0x62}}; //Actual content
	volatile	can_frame_t	inv_1CB_message	= {.can_id = 0x1cb, .can_dlc = 7, .data = {0x00,0x09,0xFF,0xCE,0x10,0x8b,0xe7}}; //Startup sequence

	//This message is not needed if you have a 62kWh pack (1ED), but probably good to send it towards the 160kW inverter
	volatile	can_frame_t	inv_1ED_message	= {.can_id = 0x1ED, .can_dlc = 3, .data = {0xFF,0xe0,0x68}};
	volatile	uint8_t		content_1ED_1		= 0xe0;
	volatile	uint8_t		content_1ED_2		= 0x68;

	//Static for now, content unknown and changes. Snapshot from power run
	static		can_frame_t	inv_355_message	= {.can_id = 0x355, .can_dlc = 8, .data = {0x14,0x0a,0x13,0x97,0x10,0x00,0x40,0x00}};
	volatile	uint8_t		ticker40ms		= 0;
	volatile	can_frame_t	inv_5CD_message	= {.can_id = 0x5CD, .can_dlc = 5, .data = {0x7a,0x06,0xf5,0x1F,0xC0}}; 
	volatile	uint8_t		content_5CD		= 0;
	volatile	uint8_t		flipFlop		= 0;										
	volatile	uint8_t		ticker100ms		= 0;
	volatile	can_frame_t	inv_3B8_message	= {.can_id = 0x3B8, .can_dlc = 5, .data = {0x7F,0xE8,0x01,0x07,0xFF}}; //Data from idle
	volatile	uint8_t		content_3B8		= 0;
	volatile	uint8_t		flip_3B8		= 0;
	
	//Content does not change
	static		can_frame_t	inv_625_message	= {.can_id = 0x625, .can_dlc = 6, .data = {0x02,0x00,0xff,0x1d,0x20,0x00}};
	static		can_frame_t	inv_5EC_message	= {.can_id = 0x5EC, .can_dlc = 1, .data = {0x00}};
	static		can_frame_t	inv_5C5_message	= {.can_id = 0x5C5, .can_dlc = 8, .data = {0x40,0x01,0x2F,0x5E,0x00,0x00,0x00,0x00}};
	static		can_frame_t	inv_5EB_message	= {.can_id = 0x5EB, .can_dlc = 8, .data = {0xE0,0x0F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}};
		
//General variables
volatile	uint8_t		can_busy			= 0;		//tracks whether the can_handler() subroutine is running

//Because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t		tx0_buffer_pos		= 0;
uint8_t		tx0_buffer_end		= 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t		tx2_buffer_pos		= 0;
uint8_t		tx2_buffer_end		= 0;

can_frame_t tx3_buffer[5];
uint8_t		tx3_buffer_pos		= 0;
uint8_t		tx3_buffer_end		= 0;

void hw_init(void){
	uint8_t caninit;

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);		
	
	//turn off everything we don' t use
	PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	PR.PRPA			= PR_ADC_bm | PR_AC_bm;
	PR.PRPC			= PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
	PR.PRPD			= PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPE			= PR_TWI_bm | PR_USART0_bm;
	
	//blink output
	PORTB.DIRSET	= 3;
	
	//start 16MHz crystal and PLL it up to 48MHz
	OSC.XOSCCTRL	= OSC_FRQRANGE_12TO16_gc |		//16MHz crystal
	OSC_XOSCSEL_XTAL_16KCLK_gc;						//16kclk startup
	OSC.CTRL	   |= OSC_XOSCEN_bm;				//enable crystal
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));			//wait until ready
	OSC.PLLCTRL		= OSC_PLLSRC_XOSC_gc | 2;		//XTAL->PLL, 2x multiplier (32MHz)
	OSC.CTRL	   |= OSC_PLLEN_bm;					//start PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm));			//wait until ready
	CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
	CLK.CTRL		= CLK_SCLKSEL_PLL_gc;			//use PLL output as system clock	
	
	//output 16MHz clock to MCP25625 chips (PE0)
	//next iteration: put this on some other port, pin  4 or 7, so we can use the event system
	TCE0.CTRLA		= TC0_CLKSEL_DIV1_gc;						//clkper/1
	TCE0.CTRLB		= TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;	//enable CCA, single-slope PWM
	TCE0.CCA		= 1;										//compare value
	TCE0.PER		= 1;										//period of 1, generates 24MHz output
	
	PORTE.DIRSET	= PIN0_bm;									//set CLKOUT pin to output
	
	//setup CAN pin interrupts
	PORTC.INTCTRL	= PORT_INT0LVL_HI_gc;
	PORTD.INTCTRL	= PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;	
	
	PORTD.INT0MASK	= PIN0_bm;						//PORTD0 has can1 interrupt
	PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	PORTD.INT1MASK	= PIN5_bm;						//PORTD5 has can2 interrupt
	PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	#ifdef ENABLE_CAN3
	PORTC.INT0MASK	= PIN2_bm;						//PORTC2 has can3 interrupt
	PORTC.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#endif
	
	//buffer checking interrupt
	TCC1.CTRLA		= TC0_CLKSEL_DIV1_gc;			//32M/1/3200 ~ 100usec
	TCC1.PER		= 3200;
	TCC1.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//same priority as can interrupts
	
	//we want to optimize performance, so we're going to time stuff
	//48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
	//frame time timer
	TCC0.CTRLA		= TC0_CLKSEL_DIV1_gc;
	TCC0.PER		= 32000;						//32MHz/32000=1ms
	TCC0.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//interrupt on overflow
	
	PORTB.OUTCLR	= (1 << 0);
	
	can_system_init:
			
	//Init SPI and CAN interface:
	if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
		caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
	} else {
		caninit = can_init(MCP_OPMOD_NORMAL, 1);
	}
	
	if(caninit){		
		//PORTB.OUTSET |= (1 << 0);					//green LED, uncommented to save power
	} else {		
		//PORTB.OUTSET |= (1 << 1);					//red LED
		_delay_ms(10);
		goto can_system_init;
	}
	
	//Set and enable interrupts with round-robin
	XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
	
	USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW);
	
	wdt_enable(WDTO_15MS);
	
	sei();
}

int main(void){
	
	hw_init();

	while(1){
		//Setup complete, wait for can messages to trigger interrupts
		}
}

// Event handler for the LUFA library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void){}

void EVENT_USB_Device_Connect(void){}

//fires every 1ms
ISR(TCC0_OVF_vect){	
	wdt_reset(); //Reset the watchdog
}

//fires approx. every 100us
ISR(TCC1_OVF_vect){
	check_can1();
	check_can2();
	check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
	can_busy = 1;
	can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
	can_busy = 1;
	can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
	can_busy = 1;
	can_handler(3);
}

//Main function for handling CAN messages
void can_handler(uint8_t can_bus){
	can_frame_t frame;
	uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
		
	if (flag & (MCP_RX0IF | MCP_RX1IF)){

		if(flag & MCP_RX1IF){ //prioritize the rollover buffer
			can_read_rx_buf(MCP_RX_1, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
			} else {
			can_read_rx_buf(MCP_RX_0, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
		}
		
		switch(frame.can_id){
			case 0x5A9:
				eco_screen = (frame.data[0] & 0x03);
			break;
			case 0x11A: //store shifter status
			switch(frame.data[0] & 0xF0){
				case 0x20:
				shift_state = SHIFT_REVERSE;
				break;
				case 0x30:
				shift_state = SHIFT_NEUTRAL;
				break;
				case 0x40:
				shift_state = SHIFT_DRIVE;
				break;
				case 0x00:
				shift_state = SHIFT_PARK;
				break;
				default:
				shift_state = SHIFT_PARK;
				break;
			}
			break;
// 			case 0x1DB: //Current signal here from BMS going towards VCM
// 				current = (int16_t) ((frame.data[0] << 8) | frame.data[1]);
// 				current = (current & 0xFF70) >> 5; //take out only 11 bits (remove 5)
// 				
// 				temp = (current & 0b0000010000000000); //extract the 11th bit, this contains signed info.
// 				if(temp > 0) //if message is signed, we are pulling power FROM battery
// 				 {	//time for shenanigans
// 					frame.data[0] = 0xF6; //hardcodes -40A pulled from battery
// 					frame.data[1] = 0x00; //wonder if this caused DTC?
// 					calc_crc8(&frame);
// 				 }
// 			break;
			case 0x1D4: //VCM request signal

				torqueDemand = ((frame.data[2] << 8) | (frame.data[3])); //Resides in frame2 and frame3
				VCMtorqueDemand = (torqueDemand >> 4); //Store the original VCM demand value (ignoring sign, just the raw NM demand)
				
				#ifdef SMALLBATTERY_IN_USE
				if(battery_soc < 70){
					break; //abort all message modifications, SOC is too low and we risk blowing the fuse
				}
				#endif
				
				if (shift_state != SHIFT_DRIVE || eco_screen == ECO_ON) break; //Stop modifying message if: Not in drive OR ECO mode is ON
				
				if((frame.data[2] & 0x80)){ //Message is signed, we are requesting regen
					#ifndef REGEN_TUNING_ENABLED
					break; //We are demanding regen and regen tuning is not on, abort modification!
					#endif
					torqueDemand = ~torqueDemand; //2S complement
					torqueDemand = (torqueDemand >> 4);
					torqueDemand = (torqueDemand * REGEN_MULTIPLIER);
					torqueDemand = (torqueDemand << 4);
					torqueDemand = ~torqueDemand; //2S complement
					        
					frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
					frame.data[3] = (torqueDemand & 0x00F0);		        
				}
				else{ //Message is not signed, we are requesting power
					torqueDemand = (torqueDemand >> 4);
					#ifdef LEAF_110kW
					torqueDemand = (torqueDemand * TORQUE_MULTIPLIER_110); //Increase torque by the specified multiplier
					#endif
					#ifdef LEAF_160kW
					torqueDemand = (torqueDemand * TORQUE_MULTIPLIER_160); //Increase torque by the specified multiplier
					#endif
					torqueDemand = (torqueDemand << 4);
					frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
					frame.data[3] = (torqueDemand & 0x00F0);
				}

				calc_crc8(&frame);
				break;
			case 0x1DA: //motor response also needs to be modified
				torqueResponse = (((frame.data[2] & 0x07) << 8) | frame.data[3]);
				torqueResponse = (torqueResponse & 0x7FF); //only take out 11bits, no need to shift
				
				#ifdef SMALLBATTERY_IN_USE
				if(battery_soc < 70){
					break; //abort all message modifications, SOC is too low and we risk blowing the fuse
				}
				#endif
				
				if (shift_state != SHIFT_DRIVE || eco_screen == ECO_ON) break; //Stop modifying message if: Not in drive OR ECO mode is ON

				if (frame.data[2] & 0x04){ //We are Regen braking
					#ifndef REGEN_TUNING_ENABLED
					break; //We are demanding regen and regen tuning is not on, abort modification!
					#endif
					torqueResponse = (VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand
					frame.data[2] = ((frame.data[2] & 0xF8) | (torqueResponse >> 8));
					frame.data[3] = (torqueResponse & 0xFF);
				}
				else //We are requesting power
				{
					torqueResponse = (VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand				
					frame.data[2] = ((frame.data[2] & 0xF8) | (torqueResponse >> 8));
					frame.data[3] = (torqueResponse & 0xFF);
				}

				calc_crc8(&frame);
				break;
			case 0x284: //Hacky way of generating missing inverter message 
				if(charging_state == CHARGING_SLOW){
					break; //abort all message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
				}
			
				//Upon reading VCM originating 0x284 every 20ms, send the missing message(s) to the inverter
				ticker40ms++;
				if(ticker40ms > 1)
				{
					ticker40ms = 0;
					
					if(can_bus == 1)
					{
						send_can2(inv_355_message); //40ms
					}
					else
					{
						send_can1(inv_355_message); //40ms
					}
				}
				break;
			case 0x50C: //Hacky way of generating missing inverter message 
				//Eliminate the CheckEV light first
				content_4B9++;
				if(content_4B9 > 79)
				{
					content_4B9 = 64;
				}
				inv_4B9_message.data[0] = content_4B9; //64 - 79 (0x40 - 0x4F)
				
				if(can_bus == 1)
				{
					send_can2(inv_4B9_message); //100ms
				}
				else
				{
					send_can1(inv_4B9_message); //100ms
				}
				
				if(charging_state == CHARGING_SLOW){
					break; //abort all further message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
				}
			
				//Upon reading VCM originating 0x50C every 100ms, send the missing message(s) to the inverter
				if(can_bus == 1)
				{
					send_can2(inv_625_message); //100ms
					send_can2(inv_5C5_message); //100ms
					send_can2(inv_3B8_message); //100ms
				} 
				else 
				{
					send_can1(inv_625_message); //100ms
					send_can1(inv_5C5_message); //100ms
					send_can1(inv_3B8_message); //100ms
				}
				
				content_3B8++;
				if(content_3B8 > 14)
				{
					content_3B8 = 0;
				}
				inv_3B8_message.data[2] = content_3B8; //0 - 14 (0x00 - 0x0E)
				
				if(flip_3B8)
				{
					flip_3B8 = 0;
					inv_3B8_message.data[1] = 0xC8;
				}
				else
				{
					flip_3B8 = 1;
					inv_3B8_message.data[1] = 0xE8;
				}
			
				
				ticker100ms++; //500ms messages go here
				if(ticker100ms > 4)
				{
					ticker100ms = 0;
					if(can_bus == 1)
					{
						send_can2(inv_5EC_message); //500ms
						send_can2(inv_5EB_message); //500ms
					}
					else
					{
						send_can1(inv_5EC_message); //500ms
						send_can1(inv_5EB_message); //500ms
					}
					
					
					if(flipFlop == 0)
					{
						flipFlop = 1;
						inv_5CD_message.data[1] = content_5CD;
						if(can_bus == 1)//1000ms messages alternating times
						{
							send_can2(inv_5CD_message); //1000ms
						}
						else
						{
							send_can1(inv_5CD_message); //1000ms
						}
						content_5CD = (content_5CD + 4);
						if(content_5CD > 238)
						{
							content_5CD = 2;
						}
					}
					else
					{
						flipFlop = 0;
					}
				}
				break;
			case 0x1F2: //Hacky way of generating missing inverter message
				charging_state = frame.data[2];
				
				if(charging_state == CHARGING_SLOW){
					break; //abort all message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
				}
				//Upon reading VCM originating 0x1F2 every 10ms, send the missing message(s) to the inverter
				if(can_bus == 1)
				{
					send_can2(inv_1C2_message);
					send_can2(inv_108_message);
					send_can2(inv_1CB_message);
					send_can2(inv_1ED_message);
				}
				else
				{
					send_can1(inv_1C2_message);
					send_can1(inv_108_message);
					send_can1(inv_1CB_message);
					send_can1(inv_1ED_message);
				}
			
				PRUN10MS++;
				if (PRUN10MS > 3){
					PRUN10MS = 0;
				}
			
				if (PRUN10MS == 0)
				{
					inv_1CB_message.data[5] = 0x88;
					inv_1CB_message.data[6] = 0xED;
					inv_1ED_message.data[1] = 0xE0;
					inv_1ED_message.data[2] = 0x68;
				}
				else if(PRUN10MS == 1)
				{
					inv_1CB_message.data[5] = 0x89;
					inv_1CB_message.data[6] = 0x68;
					inv_1ED_message.data[1] = 0xE1;
					inv_1ED_message.data[2] = 0xED;
				}
				else if(PRUN10MS == 2)
				{
					inv_1CB_message.data[5] = 0x8A;
					inv_1CB_message.data[6] = 0x62;
					inv_1ED_message.data[1] = 0xE2;
					inv_1ED_message.data[2] = 0xE7;
				}
				else if(PRUN10MS == 3)
				{
					inv_1CB_message.data[5] = 0x8B;
					inv_1CB_message.data[6] = 0xE7;
					inv_1ED_message.data[1] = 0xE3;
					inv_1ED_message.data[2] = 0x62;
				}
			
						
				content_1C2++;
				if(content_1C2 > 95)
				{
					content_1C2 = 80;
				}
				inv_1C2_message.data[0] = content_1C2; //80 - 95 (0x50 - 0x5F)
			
				content_108_1++;
				if(content_108_1 > 0x0F)
				{
					content_108_1 = 0;
				}
			
				content_108_2 = lookuptable_crc_108[content_108_1];
			
				inv_108_message.data[1] = content_108_1;
				inv_108_message.data[2] = content_108_2;

				break;
			case 0x55B:
				//Collect SOC%
				battery_soc = ((frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6)) / 10;

			break;
			case 0x603:
				//Send new ZE1 wakeup messages, why not
				if(can_bus == 1)
				{
					send_can2(inv_605_message);
					send_can2(inv_607_message);
				}
				else
				{
					send_can1(inv_605_message);
					send_can1(inv_607_message);
				}
				break;
			default:
			break;
			}
		
			if(can_bus == 1){send_can2(frame);} else {send_can1(frame);
	}
}		
			
	
	if(flag & 0xA0){
		uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
		if(flag2 & 0xC0){
			can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
		}
		if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
		if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
	}
	can_busy = 0;
}

void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	else if(can_bus == 2) send_can2(frame);
	else if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){	
	//put in the buffer
	memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
	
	if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx0_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can1();
}

void check_can1(void){
	uint8_t reg;
	
	if(tx0_buffer_end != tx0_buffer_pos){
		//check if TXB0 is free use
		reg = can1_read(MCP_REG_TXB0CTRL);
	
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
			can1_rts(0);
			if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
	}
}

void send_can2(can_frame_t frame){
	//put in the buffer
	memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
	
	if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx2_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can2();
}

void check_can2(void){
	uint8_t reg;
	
	if(tx2_buffer_end != tx2_buffer_pos){
		//check if TXB0 is free use
		reg = can2_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
			can2_rts(0);
			if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
				tx2_buffer_end = 0;
				tx2_buffer_pos = 0;
			}
		}
	}
}

void send_can3(can_frame_t frame){
	//put in the buffer
	memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
	
	if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx3_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can3();
}

void check_can3(void){
	uint8_t reg;
	
	if(tx3_buffer_end != tx3_buffer_pos){
		//check if TXB0 is free use
		reg = can3_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
			can3_rts(0);
			if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
				tx3_buffer_end = 0;
				tx3_buffer_pos = 0;
			}
		}
	}
}


