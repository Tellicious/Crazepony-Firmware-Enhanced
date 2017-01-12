/*    
      ____                       ____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
*/

#include "Led.h"
#include "config.h"
#include "imu.h"
#include "FailSafe.h"

extern int LostRCFlag;

uint8_t LED_status;
uint8_t LED_Current_Pattern = 0;

/********************************************
Led1-->PA11 Rear Left
Led2-->PA8	Rear Right
Led3-->PB1	Front Right
Led4-->PB3	Front Left
********************************************/

/* LED Patterns*/
/* LED Pattern 0: all LEDs OFF */
#define LED_PATTERN_0 LED_status = 0x00; LED_Current_Pattern = 0;
/* LED Pattern 1: all LEDs ON */
#define LED_PATTERN_1 LED_status = 0x0F; LED_Current_Pattern = 1;
/* LED Pattern 2: all LEDs blinking */
#define LED_PATTERN_2 (LED_Current_Pattern != 2) ? (LED_status = 0x0F) : (LED_status ^= 0x0F); LED_Current_Pattern = 2;
/* LED Pattern 3: front LEDs on */
#define LED_PATTERN_3 LED_status = 0x0C; LED_Current_Pattern = 3;
/* LED Pattern 4: rear LEDs on */
#define LED_PATTERN_4 LED_status = 0x03; LED_Current_Pattern = 4;
/* LED Pattern 5: front LEDs blinking */
#define LED_PATTERN_5 (LED_Current_Pattern != 5) ? (LED_status = 0x0C) : (LED_status ^= 0x0C); LED_Current_Pattern = 5;
/* LED Pattern 6: rear LEDs blinking */
#define LED_PATTERN_6 (LED_Current_Pattern != 6) ? (LED_status = 0x03) : (LED_status ^= 0x03); LED_Current_Pattern = 6;
/* LED Pattern 7: round loop CCW*/
#define LED_PATTERN_7 (LED_Current_Pattern != 7) ? (LED_status = 0x08) : (LED_status = ((LED_status << 1) | (LED_status >> 3)) & 0x0F); LED_Current_Pattern = 7;
/* LED Pattern 8: round loop CW*/
#define LED_PATTERN_8 (LED_Current_Pattern != 8) ? (LED_status = 0x10) : (LED_status = ((LED_status << 3) | (LED_status >> 1)) & 0x0F); LED_Current_Pattern = 8;
/* LED Pattern 9: front/rear */
#define LED_PATTERN_9 (LED_Current_Pattern != 9) ? (LED_status = 0x03) : (LED_status ^= 0x0F); LED_Current_Pattern = 9;
/* LED Pattern 10: diagonals */
#define LED_PATTERN_10 (LED_Current_Pattern != 10) ? (LED_status = 0x05) : (LED_status ^= 0x0F); LED_Current_Pattern = 10;

void LEDCmd(void){
	GPIO_WriteBit(GPIOB, GPIO_Pin_3, (BitAction) ((LED_status >> 3) & 0x01)); // Front Left LED 4
	GPIO_WriteBit(GPIOB, GPIO_Pin_1, (BitAction) ((LED_status >> 2) & 0x01)); // Front Right LED 3
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction) ((LED_status >> 1) & 0x01));	// Rear Right LED 2
	GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction) (LED_status & 0x01));	// Rear Left LED 1
}

void LEDFSM(void)
{
	if(LANDING == altCtrlMode){
		LED_PATTERN_1;
	}
	else if((warnings.BATTERY_CHARGING)){	
		LED_PATTERN_0;
	}
	else if(imuCaliFlag){
		LED_PATTERN_4;
	}
	else if(warnings.LOW_BATTERY){
		LED_PATTERN_2;
	}
	else if(!imu.caliPass){
		LED_PATTERN_9;
	}
	else if(1 == LostRCFlag){
		LED_PATTERN_7;
	}
	else if(!imu.ready){
		LED_PATTERN_4;
	}
	else {
		LED_PATTERN_6;
	}
	LEDCmd();
}

void PowerOn(void){
  uint8_t i;
	for(i=0;i<16;i++){
		LED_PATTERN_7;
		LEDCmd();
		delay_ms(100);
	}

	for(i=0;i<6;i++)
	{
		LED_PATTERN_2;
		LEDCmd();
		delay_ms(100);
	}
}
