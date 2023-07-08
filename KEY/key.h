#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
#include"stm32l4xx.h"
/*********************************************************************************
	KEY0 	- PD10
	KEY1 	- PD9
	KEY2 	- PD8
	WK_UP 	- PC13
*/
#define KEY0        HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)
#define KEY1        HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)
#define KEY2        HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)
#define WK_UP       HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)

#define KEY0_PRES 		1
#define KEY1_PRES		2
#define KEY2_PRES		3
#define WKUP_PRES   	4

void KEY_Init(void);
u8 KEY_Scan(u8 mode);
#endif
