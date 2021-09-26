
/*
 INVERTER UPGRADE FIRMWARE
When fitted between the inverter and VCM [EV-CAN], this CAN-bridge firmware allows for higher power output. For this, we need to:
 - Modify the commanded torque up
 - Return a lower reported motor torque
 
The firmware should also be possible to do the following at your own risk
 - Disable creep mode ( motor commands below 40Nm are ignored )
*/


/* Choose your inverter */
#define LEAF_110kW
//#define LEAF_160kW

/* Optional functionality */
//#define DISABLE_CREEP
//#define USB_SERIAL
//#define ENABLE_CAN3


#include "can-bridge-inverter.h"

volatile	int16_t		torqueDemand			= 0; //NM * 4
volatile	int16_t		torqueResponse			= 0; //NM * 2
volatile	int16_t		VCMtorqueDemand			= 0; //NM * 4
volatile	uint8_t		shift_state				= 0;

#define SHIFT_DRIVE		4
#define SHIFT_ECO		5
#define SHIFT_REVERSE	2
#define SHIFT_NEUTRAL	3
#define SHIFT_PARK		0

// Variables used for deleting P3197 DTC
volatile	can_frame_t	inv_4B9_message	= {.can_id = 0x4B9, .can_dlc = 1, .data = {0x40}};
volatile	uint8_t		content_4B9			= 0x40;

//General variables
volatile	uint8_t		can_busy			= 0;		//tracks whether the can_handler() subroutine is running
volatile	uint16_t	sec_timer			= 1;		//counts down from 1000
volatile	uint16_t	ms_timer_100		= 1;		//increments on every TCC0 overflow (ever ms)

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




#ifdef USB_SERIAL
#include "usb-hub-sensor.h"
uint8_t ReadCalibrationByte( uint8_t index );
void ProcessCDCCommand(void);

uint8_t		configSuccess				= false;		//tracks whether device successfully enumerated
static FILE USBSerialStream;							//fwrite target for CDC
uint8_t		signature[11];								//signature bytes
//print variables
volatile	uint8_t		print_char_limit		= 0;
#endif

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
	#ifdef USB_SERIAL
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	#endif
	
	wdt_enable(WDTO_15MS);
	
	sei();
}

int main(void){
	#ifdef USB_SERIAL
	char * str = "";
	#endif
	
	hw_init();

	while(1){
		//Setup complete, wait for can messages to trigger interrupts
		#ifdef USB_SERIAL
		//when USB is essentially unused, we output general status info
		if(!output_can_to_serial){
			if(sec_interrupt){
				sec_interrupt = 0;
			
				/*//current shifter state
				str = "Shift: 00\n";
				int_to_hex((char *) (str + 7), shifter_state);
				print(str, 10);*/
			}
		}
		#endif
	}
}
#ifdef USB_SERIAL
/* services commands received over the virtual serial port */
void ProcessCDCCommand(void)
{
	uint16_t	ReportStringLength = 0;
	char *		ReportString = "";
	int16_t cmd = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	
	if(cmd > -1){
		switch(cmd){
			case 48: //0
			break;
			
			case 0: //reset
			case 90: //'Z'
			_delay_ms(1000);
			CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
			RST.CTRL		= RST_SWRST_bm;
			break;
			
			case 255: //send identity
			ReportString = "DALA CAN bridge - v2.5 Leaf\n"; ReportStringLength = 28;
			break;
			
			default: //when all else fails
			ReportString = "Unrecognized Command:   \n"; ReportStringLength = 25;
			ReportString[22] = cmd;
			break;
		}
		
		if(ReportStringLength){
			print(ReportString, ReportStringLength);
		}
		
	}
}
#endif
// Event handler for the LUFA library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void){}

void EVENT_USB_Device_Connect(void){}
#ifdef USB_SERIAL
// Event handler for the LUFA library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void){ configSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface); }

// Event handler for the LUFA library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void){	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface); }

//appends string to ring buffer and initiates transmission
void print(char * str, uint8_t len){
	if((print_char_limit + len) <= 120){
		fwrite(str, len, 1, &USBSerialStream);
		print_char_limit += len;
	} else {
		fwrite("X\n",2,1,&USBSerialStream);
	}
}
#endif

//fires every 1ms
ISR(TCC0_OVF_vect){	
	wdt_reset(); //Reset the watchdog
	sec_timer--; //Increment the 1000ms timer
	
	#ifdef USB_SERIAL
	if(!can_busy) ProcessCDCCommand();
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
	//handle second print buffer
	if(print_char_limit <= 64) { print_char_limit = 0; }
	else { print_char_limit -= 64; }
	#endif
	
	//fires every 100ms
	if(ms_timer_100 == 100) //Task that need to be performed each 100ms go here
	{
		ms_timer_100 = 0; //reset the timer
	}
	
	//fires every second (1000ms tasks go here)
	if(sec_timer == 0){
		PORTB.OUTCLR = (1 << 1);
		
	}
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

//VCM side of the CAN bus (in Muxsan)
void can_handler(uint8_t can_bus){
	can_frame_t frame;
	int16_t temp = 0;
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
			case 0x11A: //store shifter status
			switch(frame.data[0] & 0xF0){
				case 0x20:
				shift_state = SHIFT_REVERSE;
				break;
				case 0x30:
				shift_state = SHIFT_NEUTRAL;
				break;
				case 0x40:
				if(frame.data[1] & 0x10){ shift_state = SHIFT_ECO; }
				else { shift_state = SHIFT_DRIVE; }
				break;
				case 0x00:
				shift_state = SHIFT_PARK;
				break;
				default:
				shift_state = SHIFT_PARK;
				break;
			}
			break;
			case 0x1D4: //VCM request signal
				
				torqueDemand = (int16_t) ((frame.data[2] << 8) | frame.data[3]); //Requested torque is 12-bit long signed.
				torqueDemand = (torqueDemand & 0xFFF0) >> 4; //take out only 12 bits (remove 4)
				VCMtorqueDemand = torqueDemand; //Store the original VCM demand value
							
				temp = (torqueDemand & 0b0000100000000000); //extract the 12th bit, this contains signed info.
				if (temp > 0){//check if message is signed
					break; //We are demanding regen, abort modification of message TODO, make better handling of this
				}
				
				if(shift_state == SHIFT_DRIVE){ // Increase power only when in drive AND
					if(torqueDemand > 180){ // When outside creep area (180*0.25 = 45Nm)
						#ifdef LEAF_110kW
						torqueDemand = torqueDemand*1.375; //Add a multiplier of 1.375 to torque (250NM*1.3=343NM)
						#endif
						#ifdef LEAF_160kW
						torqueDemand = torqueDemand*1.6; //Add a multiplier of 1.6 to torque (250NM*1.3=400NM)
						#endif
						
						torqueDemand = (torqueDemand << 4); //Shift back the 4 removed bits	
						frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
						frame.data[3] = torqueDemand & 0xF0; //Only high nibble on 3rd frame (0xFF might work if no colliding data)
						calc_crc8(&frame);
					}
					else{
						//Inside creep area
						#ifdef DISABLE_CREEP
						torqueDemand = 0;
						frame.data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
						frame.data[3] = torqueDemand & 0xF0; //Only high nibble on 3rd frame (0xFF might work if no colliding data)
						#endif
					}				
				}	


				break;
			case 0x1DA: //motor response also needs to be modified

				torqueResponse = (int16_t) ((frame.data[2] << 8) | frame.data[3]);
				torqueResponse = (torqueResponse & 0b0000011111111111); //only take out 11bits, no need to shift
				
				temp = (torqueResponse & 0b0000010000000000); //extract the 11th bit, this contains signed info.
				if (temp > 0){ //message is signed
					break;//We are getting regen, abort modification of message
				} 

				if(shift_state == SHIFT_DRIVE){ //modify power response message only when in drive
					if(torqueResponse > 90){ //(90*0.5=45Nm)
						//torqueResponse = torqueResponse*0.77; //Fool VCM that the response is smaller (OLD STRATEGY)
						
						torqueResponse = (VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand (remove 
						
						frame.data[2] = (torqueResponse >> 8);
						frame.data[3] = (torqueResponse & 0xFF);
						calc_crc8(&frame);
					}
				}

				break;
			case 0x50C: //Hacky way of generating missing inverter message
				//Upon reading VCM originating 0x50C every 100ms, send the missing message to the inverter
				if(can_bus == 1)
				{
					send_can2(inv_4B9_message);
				} 
				else 
				{
					send_can1(inv_4B9_message);
				}
				
				content_4B9++;
				if(content_4B9 > 79)
				{
					content_4B9 = 64;
				}
				
				inv_4B9_message.data[0] = content_4B9; //64 - 79 (0x40 - 0x4F)
				
				break;
			default:
			break;
			}
		
		
		//block unwanted messages
			uint8_t block = 0;
			switch(frame.can_id){
				case 0xABC:
					//block = 1;
					break;
				default:
					block = 0;
					break;
			}
			if(!block){
				if(can_bus == 1){send_can2(frame);} else {send_can1(frame);
				}
			}
		}		
			
	
	if(flag & 0xA0){
		uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
		if(flag2 & 0xC0){
			can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
			//ReportString = "CANX RX OVF\n";
			//ReportString[3] = 48 + can_bus;
			//print(ReportString,12);
		}
		if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
		if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
	}
	can_busy = 0;
}


void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	if(can_bus == 2) send_can2(frame);
	if(can_bus == 3) send_can3(frame);
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


