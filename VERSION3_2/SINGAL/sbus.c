#include "usart.h"
#include "sbus.h"
#include "tim.h"

#define sbus_led
uint8_t rec_sbus_data[25];
uint8_t sbus_erroflag;
uint16_t sbus_channel[16];
/*
1900->200
ch3 ch4 ch2 ch1
*/
void sbus_calc(void)
{
  if(rec_sbus_data[23]==0x03)
	{
		sbus_channel[0]  = ((rec_sbus_data[1]|rec_sbus_data[2]<<8) & 0x07FF);
		sbus_channel[1]  = ((rec_sbus_data[2]>>3 |rec_sbus_data[3]<<5) & 0x07FF);
		sbus_channel[2]  = ((rec_sbus_data[3]>>6 |rec_sbus_data[4]<<2 |rec_sbus_data[5]<<10) & 0x07FF);
		sbus_channel[3]  = ((rec_sbus_data[5]>>1 |rec_sbus_data[6]<<7) & 0x07FF);
		sbus_channel[4]  = ((rec_sbus_data[6]>>4 |rec_sbus_data[7]<<4) & 0x07FF);
		sbus_channel[5]  = ((rec_sbus_data[7]>>7 |rec_sbus_data[8]<<1 |rec_sbus_data[9]<<9) & 0x07FF);
		sbus_channel[6]  = ((rec_sbus_data[9]>>2 |rec_sbus_data[10]<<6) & 0x07FF);
		sbus_channel[7]  = ((rec_sbus_data[10]>>5|rec_sbus_data[11]<<3) & 0x07FF);
		sbus_channel[8]  = ((rec_sbus_data[12]   |rec_sbus_data[13]<<8) & 0x07FF);
		sbus_channel[9]  = ((rec_sbus_data[13]>>3|rec_sbus_data[14]<<5) & 0x07FF);
		sbus_channel[10] = ((rec_sbus_data[14]>>6|rec_sbus_data[15]<<2|rec_sbus_data[16]<<10) & 0x07FF);
		sbus_channel[11] = ((rec_sbus_data[16]>>1|rec_sbus_data[17]<<7) & 0x07FF);
		sbus_channel[12] = ((rec_sbus_data[17]>>4|rec_sbus_data[18]<<4) & 0x07FF);
		sbus_channel[13] = ((rec_sbus_data[18]>>7|rec_sbus_data[19]<<1|rec_sbus_data[20]<<9)& 0x07FF);
		sbus_channel[14] = ((rec_sbus_data[20]>>2|rec_sbus_data[21]<<6) & 0x07FF);
		sbus_channel[15] = ((rec_sbus_data[21]>>5|rec_sbus_data[22]<<3) & 0x07FF);
    sbus_erroflag=0;
		#ifdef sbus_led
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,1000);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,0);
		#endif
	}
	else
	{
		#ifdef sbus_led
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,500);
		__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,0);
		#endif
		sbus_erroflag=1;
	}
		
}
