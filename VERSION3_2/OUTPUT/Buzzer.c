/* Private includes ----------------------------------------------------------*/
#include "buzzer.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private define ------------------------------------------------------------*/
#define Bass_Do ( 262)
#define Bass_Re ( 294)
#define Bass_Mi ( 330)
#define Bass_Fa ( 349)
#define Bass_So ( 392)
#define Bass_La ( 440)
#define Bass_Si ( 494)

#define Alto_Do ( 523)
#define Alto_Re ( 587)
#define Alto_Mi ( 659)
#define Alto_Fa ( 698)
#define Alto_So ( 784)
#define Alto_La ( 880)
#define Alto_Si ( 988)

#define Treb_Do (1046)
#define Treb_Re (1175)
#define Treb_Mi (1318)
#define Treb_Fa (1397)
#define Treb_So (1568)
#define Treb_La (1760)
#define Treb_Si (1976)

#define Buzzer_Tim        htim4
#define Buzzer_Channel    TIM_CHANNEL_3
#define Buzzer_Delay(x)   osDelay(x)

//#define Buzzer_Delay(x)   vTaskDelay(x)

/* Pubulic variables ----------------------------------------------------------*/
uint8_t Buzzer_Busy;

const uint16_t Musical_Scale[3][8] =
{
	{0, Bass_Do, Bass_Re, Bass_Mi, Bass_Fa, Bass_So, Bass_La, Bass_Si},
	{0, Alto_Do, Alto_Re, Alto_Mi, Alto_Fa, Alto_So, Alto_La, Alto_Si},
	{0, Treb_Do, Treb_Re, Treb_Mi, Treb_Fa, Treb_So, Treb_La, Treb_Si},
};

const uint32_t Music_Galayou[] =
{
          13,         83,          2,         41,
	0x40100103, 0x40100104, 0x40000105, 0x20100105, 0x20100103, 
	0x20100105, 0x10100103, 0x10000105, 0x20100105, 0x20000107, 
	0x20100107, 0x20000201, 0x40100201, 
};
const uint32_t Music_HaoYunLai[1];
const uint32_t Music_MiXueBingCheng[] =
{
	        30,         83,          6,         18,
	0x08010103, 0x08010105, 0x0c010105, 0x04010106, 0x08010105, 0x08010103, 0x08010101,
	0x04010101, 0x04010102, 0x08010103, 0x08010103, 0x08010102, 0x08010101, 0x10010102,
	0x10010000, 
	0x08010103, 0x08010105, 0x0c010105, 0x04010106, 0x08010105, 0x08010103, 0x08010101,
	0x04010101, 0x04010102, 0x08010103, 0x08010103, 0x08010102, 0x08010102, 0x10010101,
	0x10010000, 
};
const uint32_t Music_Dayu[] = {
            14,        83,         12,        23,
	0x08000103, 0x08010102, 0x08000103, 0x08010106, 0x08000103, 0x08010102, 0x08000103, 0x08010107, 
	0x08000103, 0x08010102, 0x08000103, 0x08010201, 0x10010107, 0x10010105, 
	0x08000103, 0x08010102, 0x08000103, 0x08010106, 0x08000103, 0x08010102, 0x08000103, 0x08010107, 
	0x10010105, 0x10000102, 0x10000007, 0x10000007, 
	0x08000103, 0x08010102, 0x08000103, 0x08010106, 0x08000103, 0x08010102, 0x08000103, 0x08010107, 
	0x08000103, 0x08010102, 0x08000103, 0x08010201, 0x10010107, 0x10010105,  
};

/* Functions -----------------------------------------------------------------*/
void Buzzer_On(uint16_t Prescaler, uint16_t Period, uint16_t Compare)
{
	__HAL_TIM_SET_PRESCALER(&Buzzer_Tim, Prescaler);
	__HAL_TIM_SET_AUTORELOAD(&Buzzer_Tim, Period);
	__HAL_TIM_SET_COMPARE(&Buzzer_Tim, Buzzer_Channel, Compare);
}

void Buzzer_Off(void)
{
  __HAL_TIM_SET_COMPARE(&Buzzer_Tim, Buzzer_Channel, 0);
}

void Buzzer_PlayMusic(const uint32_t *Music)
{
	uint16_t Music_Pointer = 4; 
	uint16_t Delay_Counter = 1; 
	
	uint8_t Buzzer_Off_DelayTime;
	uint8_t Buzzer_Onn_DelayTime;
	uint8_t Music_Level;
	uint8_t Music_Scale;
	
	Buzzer_Busy = 1;
	while(Music_Pointer < Music[0] + 4)
	{
		Buzzer_Onn_DelayTime = (Music[Music_Pointer] & 0xFF000000) >> 24;
		Buzzer_Off_DelayTime = (Music[Music_Pointer] & 0x00FF0000) >> 16;
		Music_Level          = (Music[Music_Pointer] & 0x0000FF00) >>  8;
		Music_Scale          = (Music[Music_Pointer] & 0x000000FF) >>  0;
		Delay_Counter        = 1;
		
		while(Delay_Counter <= Buzzer_Onn_DelayTime + Buzzer_Off_DelayTime)
		{
			if(Delay_Counter <= Buzzer_Onn_DelayTime)
			{
				Buzzer_On(Music[1], 1000000 / Musical_Scale[Music_Level][Music_Scale] - 1, 500000 / Musical_Scale[Music_Level][Music_Scale]);
				Delay_Counter ++;
			}
			else if(Delay_Counter <= Buzzer_Onn_DelayTime + Buzzer_Off_DelayTime)
			{
				Buzzer_Off();
				Delay_Counter ++;
			}
			Buzzer_Delay(100 * Music[2] / Music[3]);
		}
		
		Music_Pointer ++;
	}
	
	Buzzer_Off();
	Buzzer_Busy = 0;
}
