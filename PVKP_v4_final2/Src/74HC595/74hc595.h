#ifndef __74HC595_H__
#define	__74HC595_H__

//#include "main.h"

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
/*#include "stm32f10xx_gpio.h"
#include "stm32f10xx_misc.h"
#include "stm32f10xx_exti.h"
#include "stm32f10xx_rcc.h"
#include "stm32f10xx_it.h"*/

void TimingDelay_Decrement(void);

#define M74HC595_PORT		GPIOB

#define M74HC595_SCLR_NOT	GPIO_PIN_8  //MR
#define M74HC595_SCK		GPIO_PIN_15 //SCLK
#define M74HC595_RCK		GPIO_PIN_14 //ST
#define M74HC595_G_NOT		GPIO_PIN_13 //OE
#define M74HC595_SI			GPIO_PIN_12 //OUT

void M74HC595_Init(void);
void M74HC595_ClearData(void);
void M74HC595_EnableData(void);
void M74HC595_WriteData(const unsigned char data0, const unsigned char data1, const unsigned char data2);

#endif
