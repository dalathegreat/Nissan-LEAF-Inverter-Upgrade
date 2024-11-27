
/*
 INVERTER UPGRADE FIRMWARE
When fitted between the inverter and VCM [EV-CAN], this CAN-bridge firmware allows for higher power output. For this, we need to:
 - Modify the commanded torque up
 - Return a lower reported motor torque
 - Send a bunch of 2018+ CAN messages that the inverter expects to keep it error free
 - Optionally, protect the fuse by limiting output at low SOC%, for instance if you run a 24kWh pack (SMALLBATTERY_IN_USE)
*/

/* Choose your inverter */
//#define LEAF_110kW
#define LEAF_160kW

/* Choose to enable fuse safeguard features*/
//#define SMALLBATTERY_IN_USE //Use with 24 kWh packs

/* Choose to enable regen tuning*/
#define REGEN_TUNING_ENABLED

#define TORQUE_MULTIPLIER_110 1.32 									//(1.28 is too small, results in 118kW and 1.37 results in 122kW)
#define TORQUE_MULTIPLIER_160 1.40									//For a more aggressive pedal feel, multipliers between 1.4 and 1.6 can be used

#define REGEN_MULTIPLIER 1.10

#include "can.h"

#include "can-bridge-firmware.h"
#include "nissan_can_structs.h"

#include <stdio.h>
#include <string.h>
static uint8_t	crctable[256] = {0,133,143,10,155,30,20,145,179,54,60,185,40,173,167,34,227,102,108,233,120,253,247,114,80,213,223,90,203,78,68,193,67,198,204,73,216,93,87,210,240,117,127,250,107,238,228,97,160,37,47,170,59,190,180,49,19,150,156,25,136,13,7,130,134,3,9,140,29,152,146,23,53,176,186,63,174,43,33,164,101,224,234,111,254,123,113,244,214,83,89,220,77,200,194,71,197,64,74,207,94,219,209,84,118,243,249,124,237,104,98,231,38,163,169,44,189,56,50,183,149,16,26,159,14,139,129,4,137,12,6,131,18,151,157,24,58,191,181,48,161,36,46,171,106,239,229,96,241,116,126,251,217,92,86,211,66,199,205,72,202,79,69,192,81,212,222,91,121,252,246,115,226,103,109,232,41,172,166,35,178,55,61,184,154,31,21,144,1,132,142,11,15,138,128,5,148,17,27,158,188,57,51,182,39,162,168,45,236,105,99,230,119,242,248,125,95,218,208,85,196,65,75,206,76,201,195,70,215,82,88,221,255,122,112,245,100,225,235,110,175,42,32,165,52,177,187,62,28,153,147,22,135,2,8,141};

static volatile	uint16_t		torqueDemand			= 0; //NM * 4
static volatile	uint16_t		torqueResponse			= 0; //NM * 2
static volatile	uint16_t		VCMtorqueDemand			= 0; //NM * 4
//static volatile	uint16_t		current					= 0;
static volatile	uint16_t	main_battery_soc		= 0;
static volatile	uint8_t		shift_state				= 0;
static volatile	uint8_t		charging_state			= 0;

#define SHIFT_DRIVE		4
#define SHIFT_ECO		5
#define SHIFT_REVERSE	2
#define SHIFT_NEUTRAL	3
#define SHIFT_PARK		0

#define ECO_OFF			1
#define ECO_ON			2
	
#define CHARGING_QUICK_START	0x40
#define CHARGING_QUICK				0xC0
#define CHARGING_QUICK_END		0xE0
#define CHARGING_SLOW					0x20
#define CHARGING_IDLE					0x60

// CAN messages used for deleting P3197 [EV/HEV] & P318E [MOTOR CONTROL] DTC (Send 4B9)
static CAN_FRAME	inv_4B9_message	= {.ID =  0x4B9, .dlc = 1, .data = {0x40}};
static uint8_t		content_4B9			= 0x40;

// Sending 355 does not fix any DTC (but probably good to send it anyway)
// Sending 625 removes U215B [HV BATTERY]
// Sending 5C5 (355 at 40ms) removes U214E [HV BATTERY] and U1000 [MOTOR CONTROL] 
// Sending 3B8 and 5EB removes U1000 and P318E [HV BATTERY]
	static volatile	uint8_t		PRUN10MS			= 0;
	
	static		CAN_FRAME	inv_605_message	= {.ID =  0x605, .dlc = 1, .data = {0x00}};
	static		CAN_FRAME	inv_607_message	= {.ID =  0x607, .dlc = 1, .data = {0x00}};

	//This message is 40kWh specific (maybe on
	static	CAN_FRAME	inv_1C2_message	= {.ID =  0x1C2, .dlc = 1, .data = {0x50}};
	static 	uint8_t		content_1C2			= 0x50;

	static CAN_FRAME	inv_108_message	= {.ID =  0x108, .dlc = 3, .data = {0x00,0x00,0x00}};
	static volatile	uint8_t		content_108_1			= 0x00;
	static volatile	uint8_t		content_108_2			= 0x00;
	static		uint8_t		lookuptable_crc_108[16] = {0x00,0x85,0x8F,0x0A,0x9B,0x1e,0x14,0x91,0xb3,0x36,0x3c,0xb9,0x28,0xad,0xa7,0x22};

	//volatile	can_frame_t	inv_1CB_message	= {.ID =  0x1cb, .dlc = 7, .data = {0x00,0x00,0x00,0x02,0x60,0x00,0x62}}; //Actual content
	static CAN_FRAME	inv_1CB_message	= {.ID =  0x1cb, .dlc = 7, .data = {0x00,0x09,0xFF,0xCE,0x10,0x8b,0xe7}}; //Startup sequence

	//This message is not needed if you have a 62kWh pack (1ED), but probably good to send it towards the 160kW inverter
	static CAN_FRAME	inv_1ED_message	= {.ID =  0x1ED, .dlc = 3, .data = {0xFF,0xe0,0x68}};
	//static volatile	uint8_t		content_1ED_1		= 0xe0;
	//static volatile	uint8_t		content_1ED_2		= 0x68;

	//Static for now, content unknown and changes. Snapshot from power run
	static		CAN_FRAME	inv_355_message	= {.ID =  0x355, .dlc = 8, .data = {0x14,0x0a,0x13,0x97,0x10,0x00,0x40,0x00}};
	static volatile	uint8_t		ticker40ms		= 0;
	static CAN_FRAME	inv_5CD_message	= {.ID =  0x5CD, .dlc = 5, .data = {0x7a,0x06,0xf5,0x1F,0xC0}}; 
	static volatile	uint8_t		content_5CD		= 0;
	static volatile	uint8_t		flipFlop		= 0;										
	static volatile	uint8_t		ticker100ms		= 0;
	static CAN_FRAME	inv_3B8_message	= {.ID =  0x3B8, .dlc = 5, .data = {0x7F,0xE8,0x01,0x07,0xFF}}; //Data from idle
	static volatile	uint8_t		content_3B8		= 0;
	static volatile	uint8_t		flip_3B8		= 0;
	
	//Content does not change
	static		CAN_FRAME	inv_625_message	= {.ID =  0x625, .dlc = 6, .data = {0x02,0x00,0xff,0x1d,0x20,0x00}};
	static		CAN_FRAME	inv_5EC_message	= {.ID =  0x5EC, .dlc = 1, .data = {0x00}};
	static		CAN_FRAME	inv_5C5_message	= {.ID =  0x5C5, .dlc = 8, .data = {0x40,0x01,0x2F,0x5E,0x00,0x00,0x00,0x00}};
	static		CAN_FRAME	inv_5EB_message	= {.ID =  0x5EB, .dlc = 8, .data = {0xE0,0x0F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}};

void calc_crc8(CAN_FRAME *frame);

void calc_sum4(CAN_FRAME *frame);

	void one_second_ping( void )
{
}

//recalculates the CRC-8 with 0x85 poly
void calc_crc8(CAN_FRAME *frame)
{
    uint8_t crc = 0;
    
	for(uint8_t j = 0; j < 7; j++)
    {
        crc = crctable[(crc ^ ((int) frame->data[j])) % 256];
    }
    
    frame->data[7] = crc;
}



void can_handler(uint8_t can_bus, CAN_FRAME *frame)
{
    if (1)
    {
        switch (frame->ID)
        {
case 0x5A9:
			break;
case 0x11A: //store shifter status
			switch(frame->data[0] & 0xF0){
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
case 0x1D4: //VCM request signal

				torqueDemand = (uint16_t)((frame->data[2] << 8) | (frame->data[3])); //Resides in frame2 and frame3
				VCMtorqueDemand = (torqueDemand >> 4); //Store the original VCM demand value (ignoring sign, just the raw NM demand)
				
				#ifdef SMALLBATTERY_IN_USE
				if(battery_soc < 70){
					break; //abort all message modifications, SOC is too low and we risk blowing the fuse
				}
				#endif
				
				if (shift_state != SHIFT_DRIVE) break; //Stop modifying message if: Not in drive OR ECO mode is ON
				
				if((frame->data[2] & 0x80)){ //Message is signed, we are requesting regen
					#ifndef REGEN_TUNING_ENABLED
					break; //We are demanding regen and regen tuning is not on, abort modification!
					#endif
					torqueDemand = ~torqueDemand; //2S complement
					torqueDemand = (torqueDemand >> 4);
					torqueDemand = (uint16_t)(torqueDemand * REGEN_MULTIPLIER);
					torqueDemand = (uint16_t)(torqueDemand << 4);
					torqueDemand = ~torqueDemand; //2S complement
					        
					frame->data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
					frame->data[3] = (torqueDemand & 0x00F0);		        
				}
				else{ //Message is not signed, we are requesting power
					torqueDemand = (torqueDemand >> 4);
					#ifdef LEAF_110kW
					torqueDemand = (uint16_t)(torqueDemand * TORQUE_MULTIPLIER_110); //Increase torque by the specified multiplier
					#endif
					#ifdef LEAF_160kW
					torqueDemand = (uint16_t)(torqueDemand * TORQUE_MULTIPLIER_160); //Increase torque by the specified multiplier
					#endif
					torqueDemand = (uint16_t)(torqueDemand << 4);
					frame->data[2] = torqueDemand >> 8; //Slap it back into whole 2nd frame
					frame->data[3] = (torqueDemand & 0x00F0);
				}

				calc_crc8(frame);
				break;
			case 0x1DA: //motor response also needs to be modified
				torqueResponse = (uint16_t)(((frame->data[2] & 0x07) << 8) | frame->data[3]);
				torqueResponse = (torqueResponse & 0x7FF); //only take out 11bits, no need to shift
				
				#ifdef SMALLBATTERY_IN_USE
				if(battery_soc < 70){
					break; //abort all message modifications, SOC is too low and we risk blowing the fuse
				}
				#endif
				
				if (shift_state != SHIFT_DRIVE) break; //Stop modifying message if: Not in drive OR ECO mode is ON

				if (frame->data[2] & 0x04){ //We are Regen braking
					#ifndef REGEN_TUNING_ENABLED
					break; //We are demanding regen and regen tuning is not on, abort modification!
					#endif
					torqueResponse = (uint16_t)(VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand
					frame->data[2] = ((frame->data[2] & 0xF8) | (torqueResponse >> 8));
					frame->data[3] = (torqueResponse & 0xFF);
				}
				else //We are requesting power
				{
					torqueResponse = (uint16_t)(VCMtorqueDemand*0.5); //Fool VCM that response is exactly the same as demand				
					frame->data[2] = ((frame->data[2] & 0xF8) | (torqueResponse >> 8));
					frame->data[3] = (torqueResponse & 0xFF);
				}

				calc_crc8(frame);
				break;
			case 0x284: //Hacky way of generating missing inverter message 
				//Upon reading VCM originating 0x284 every 20ms, send the missing message(s) to the inverter
			
				if(charging_state == CHARGING_SLOW){
					break; //abort all message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
				}

				ticker40ms++;
				if(ticker40ms > 1)
				{
					ticker40ms = 0;
					
					if(can_bus == 0)
					{
						PushCan(1, CAN_TX, &inv_355_message); //40ms
					}
					else
					{
						PushCan(0, CAN_TX, &inv_355_message); //40ms
					}
				}
				break;
			case 0x50C: //Hacky way of generating missing inverter message 
				//Upon reading VCM originating 0x50C every 100ms, send the missing message(s) to the inverter
				content_4B9++;
				if(content_4B9 > 79)
				{
					content_4B9 = 64;
				}
				inv_4B9_message.data[0] = content_4B9; //64 - 79 (0x40 - 0x4F)
			
				if(can_bus == 0)
				{
					PushCan(1, CAN_TX, &inv_4B9_message); //100ms
				} 
				else 
				{
					PushCan(0, CAN_TX, &inv_4B9_message); //100ms
				}
				
				if(charging_state == CHARGING_SLOW){
					break; //abort all further message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
				}
				
				if(can_bus == 0)
				{
					PushCan(1, CAN_TX, &inv_4B9_message); //100ms
					PushCan(1, CAN_TX, &inv_625_message); //100ms
					PushCan(1, CAN_TX, &inv_5C5_message); //100ms
					PushCan(1, CAN_TX, &inv_3B8_message); //100ms
				} 
				else 
				{
					PushCan(0, CAN_TX, &inv_4B9_message); //100ms
					PushCan(0, CAN_TX, &inv_625_message); //100ms
					PushCan(0, CAN_TX, &inv_5C5_message); //100ms
					PushCan(0, CAN_TX, &inv_3B8_message); //100ms
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
					if(can_bus == 0)
					{
						PushCan(1,CAN_TX, &inv_5EC_message); //500ms
						PushCan(1,CAN_TX, &inv_5EB_message); //500ms
					}
					else
					{
						PushCan(2,CAN_TX, &inv_5EC_message); //500ms
						PushCan(2,CAN_TX, &inv_5EB_message); //500ms
					}
					
					
					if(flipFlop == 0)
					{
						flipFlop = 1;
						inv_5CD_message.data[1] = content_5CD;
						if(can_bus == 0)//1000ms messages alternating times
						{
							PushCan(1, CAN_TX, &inv_5CD_message); //1000ms
						}
						else
						{
							PushCan(2, CAN_TX, &inv_5CD_message); //1000ms
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
				charging_state = frame->data[2];
				
				if(charging_state == CHARGING_SLOW){
					break; //abort all message modifications, otherwise we interrupt AC charging on 62kWh LEAFs
				}
				
				//Upon reading VCM originating 0x1F2 every 10ms, send the missing message(s) to the inverter
				if(can_bus == 0)
				{
					PushCan(1, CAN_TX, &inv_1C2_message);
					PushCan(1, CAN_TX, &inv_108_message);
					PushCan(1, CAN_TX, &inv_1CB_message);
					PushCan(1, CAN_TX, &inv_1ED_message);
				}
				else
				{
					PushCan(0, CAN_TX, &inv_1C2_message);
					PushCan(0, CAN_TX, &inv_108_message);
					PushCan(0, CAN_TX, &inv_1CB_message);
					PushCan(0, CAN_TX, &inv_1ED_message);
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
				main_battery_soc = (uint16_t) (frame->data[0] << 2) | ((frame->data[1] & 0xC0) >> 6); 
				main_battery_soc /= 10; //Remove decimals, 0-100 instead of 0-100.0
			break;
			case 0x603:
				//Send new ZE1 wakeup messages, why not
				if(can_bus == 0)
				{
					PushCan(1, CAN_TX, &inv_605_message);
					PushCan(1, CAN_TX, &inv_607_message);
				}
				else
				{
					PushCan(0, CAN_TX, &inv_605_message);
					PushCan(0, CAN_TX, &inv_607_message);
				}
				break;
			default:
			break;

    }
				if (can_bus == 0)
				{
						PushCan(1, CAN_TX, frame);
				}
				else
				{
						PushCan(0, CAN_TX, frame);
				}

    }
}
