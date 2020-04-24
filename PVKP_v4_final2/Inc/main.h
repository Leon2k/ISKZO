/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "../Src/modbus/include/mb.h"
#include "../Src/modbus/port/port.h"
#include "../Src/one_wire/one_wire.h"
//#include "../Src/modbus/include/mbconfig.h"
//#include "../Src/modbus/include/mbframe.h"
//#include "../Src/modbus/include/mbutils.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ONEWire_Pin GPIO_PIN_13
#define ONEWire_GPIO_Port GPIOC
#define ROW4_Pin GPIO_PIN_0
#define ROW4_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_1
#define ROW3_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_2
#define ROW2_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_3
#define ROW1_GPIO_Port GPIOA
#define ROW0_Pin GPIO_PIN_4
#define ROW0_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_5
#define COL2_GPIO_Port GPIOA
#define COL2_EXTI_IRQn EXTI9_5_IRQn
#define COL1_Pin GPIO_PIN_6
#define COL1_GPIO_Port GPIOA
#define COL1_EXTI_IRQn EXTI9_5_IRQn
#define COL0_Pin GPIO_PIN_7
#define COL0_GPIO_Port GPIOA
#define COL0_EXTI_IRQn EXTI9_5_IRQn
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOB
#define LED_CS_Pin GPIO_PIN_12
#define LED_CS_GPIO_Port GPIOB
#define SPI2_SCK___LED_SCK_Pin GPIO_PIN_13
#define SPI2_SCK___LED_SCK_GPIO_Port GPIOB
#define SPI2_MOSI___LED_SDI_Pin GPIO_PIN_15
#define SPI2_MOSI___LED_SDI_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_11
#define RS485_DE_GPIO_Port GPIOA
#define RS485_RE_Pin GPIO_PIN_12
#define RS485_RE_GPIO_Port GPIOA
#define RGB_BLUE_Pin GPIO_PIN_3
#define RGB_BLUE_GPIO_Port GPIOB
#define RGB_GREEN_Pin GPIO_PIN_4
#define RGB_GREEN_GPIO_Port GPIOB
#define RGB_RED_Pin GPIO_PIN_5
#define RGB_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//uint16_t ModBusRegistersData[8];
//uint16_t usSRegHoldBuf[8];
//uint8_t CardID[8];


/* Светодиоды */
enum LEDs {
	LED_PIT,
	LED_KAR,
	LED_OBR,
	LED_PRS,
	LED_REM,
	LED_1,
	LED_2,
	LED_3,
	LED_4,
	LED_5,
	LED_6,
	LED_7,
	LED_8,
	LED_9,
	LED_10,
	LED_11,
	LED_12,
	LED_13,
	LED_14,
	LED_15
};


one_wire_t OneWireDevice;
one_wire_rom_id_t NFCCard;

#define OutputRegister		0						// Номер исходящего регистра
#define InputModeRegister 	1						// Номер входящего регистра режима
#define InputLightRegister	2						// Номер входящего регистра светофора
#define CardRegister1		3
#define CardRegister2		4
//#define CardRegister3		5
//#define CardRegister4		6
#define DownTimeRegister	5
#define RepairRegister		6
#define VersionRegister		7						// Номер регистра версии ПО

#define NumOfRegs			10						// Кол-во регистров

/* Порты и адреса */
#define ModBusSlaveID  			149					// номер устройства в сети Modbus RTU
#define RS485Speed 				115200				// Скорость

/* Макросы для сдвиговых регистров */
#define cs_set() HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_SET)
#define cs_strob() cs_reset();cs_set()

/* 1 байт входящего регистра (Состояние системы) */
enum InputRegisterSystemState {
	UnknownInputState	= 0,
	AuthorizedOperator 	= 1,
	AuthorizedMechanic	= 2,
	AuthorizedAdmin		= 3,
	AuthorizeError		= 10
};

/* Состояния ПВКП */
typedef enum _PVKPMode {
	UnknownMode					= 0,		// Неизвестное состояние (Перед проверкой связи)
	WaitForCard 				= 1,		// Ожидание карты !ВЫСШИЙ ПРИОРИТЕТ!
	CardLevelRequest			= 2,		// Запрос уровня доступа
	Authorized					= 3,		// Авторизован оператор
	Working						= 4,		// Рабочее состояние (ЗЕЛЕНЫЙ сигнал)
	Downtime_Started			= 5,		// Начало ПРОСТОЯ/РЕМОНТА (Красный, Желтый сигнал) Вкл таймер
	Downtime_RepairChoice		= 6,		// Выбор причины РЕМОНТА (Механик)
	Downtime_DowntimeChoice		= 7,		// Выбор причины ПРОСТОЯ (Оператор)
	Downtime_ReparSelected		= 8,		// Выбрана причина РЕМОНТА (Механик)
	Downtime_DowntimeSelected	= 9,		// Выбрана причина ПРОСТОЯ (Оператор)
	DownTime_FailChoice			= 10,		// Тайм-аут выбора
	AdminModeRequest			= 11,		// Режим АДМИНА (Запрос пароля)
	AdminMode					= 12		// Режим АДМИНА
} PVKPMode;

typedef enum _Lights {
	RedLight		= 0b1000, 	//8,
	YellowLight		= 0b100, 	//4,
	BlueLight		= 0b10, 	//2,
	GreenLight		= 0b1,		//1,
	UnknownLight	= 0
} Lights;

uint8_t* ReadModBusIDFromFlash (void);
void WriteModBusIDToFlash (uint8_t data);

void RGBBlink (void);
void SendToShiftRegisters(uint8_t* LEDs);
void SetLEDPin (int led, bool state);
void PlayOKTones (void);
void PlayBadTones(void);
void PlaySwitchTones(void);
void LoopPlayer(void);


uint8_t HighByteOfWord (uint16_t Register);
uint8_t LowByteOfWord (uint16_t Register);
void AnnulateCurrentCard (void);
void SendCurrentCard (void);
void SwitchCurrentPVKPMode (PVKPMode State);
void CheckLink (void);
void ProcessCurrentMode (void);
void ProcessCurrentLEDs (void);
void ProcessKeyPad (void);
void ProcessCurrentLights (void);
void GetSystemInputStateFromRegister (void);	//
void GetLightsInputStateFromRegister (void);	//
void ResetInputRegister (void);					//
void ResetKeypadLEDs (void);					//
void WriteCurrentCardToCardRegisters (void);
Lights GetCurrentLights (void);
void ReadNFCCard (void);
bool IsNewCard (void);
bool CardMatches (uint8_t Card1[8], uint8_t Card2[8]);

/* REG #0: ИСХОДЯЩИЙ
 * 1й байт - Состояние ПВКП (1 = Ожидание авторизации, 2 = Запрос уровня доступа карты, 3 = Введен код простоя)
 * 2й байт - Код простоя

 * REG #1: ВХОДЯЩИЙ
 * 1й байт - Состояние Системы (Уровень авторизации: 0 = Нет данных, 1 = Оператор, 2 = Механик, 3 = Настройщик??, 10 = Ошибка авторизации)
 * 2й байт -

 * REG #2: ВХОДЯЩИЙ
 * 1й байт - Данные светофора (биты 0..2)
 * 2й байт -

 * REG #3:
 * 1й байт - Текущая карта(байт 1)     ---1й байт - Текущая карта(байт 1) (обнулять при смене)
 * 2й байт - Текущая карта(байт 2) (обнулять при смене)
 * REG #4:
 * 1й байт - Текущая карта(байт 3) (обнулять при смене)
 * ---2й байт - Текущая карта(байт 4) (обнулять при смене)
 * REG #5
 * 1й байт - Причина простоя
 *
 * REG #6
 * 1й байт - Причина ремонта

 * REG #7:
 * 1й байт - Версия прошивки
 *
 */
uint16_t ModbusRegistersData16bit[NumOfRegs];


/*
 *  M O D B U S   R T U
 */

#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   1
#define S_COIL_START                  0
#define S_COIL_NCOILS                 1

// Использумые регистры
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             10

#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           8

/* slave mode: holding register's all address */
#define          S_HD_RESERVE                     0
/* slave mode: input register's all address */
#define          S_IN_RESERVE                     0
/* slave mode: coil's all address */
#define          S_CO_RESERVE                     0
/* slave mode: discrete's all address */
#define          S_DI_RESERVE                     0

USHORT usSRegInBuf[S_REG_INPUT_NREGS];
USHORT usSRegHoldBuf[S_REG_HOLDING_NREGS];

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs );
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode);
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode);
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete );
void __critical_enter(void);
void __critical_exit(void);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
