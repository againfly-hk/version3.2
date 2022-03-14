/**
  ******************************************************************************
  * @file    buzzer.h
  * @author  Karolance Future
  * @version V1.0.0
  * @date    2022/03/01
  * @brief   Header file of buzzer.c
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

extern uint8_t Buzzer_Busy;
extern const uint16_t Musical_Scale[3][8];
extern const uint32_t Music_Galayou[];
extern const uint32_t Music_HaoYunLai[];
extern const uint32_t Music_MiXueBingCheng[];
extern const uint32_t Music_Dayu[];

void Buzzer_On(uint16_t Prescaler, uint16_t Period, uint16_t Compare);
void Buzzer_Off(void);
void Buzzer_PlayMusic(const uint32_t *Music);

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H__ */
