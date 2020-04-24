/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdbool.h"
#include "buzzer/buzzer.h"
#include "one_wire/one_wire.h"
#include "modbus/include/mb.h"
#include "modbus/port/port.h"
#include "modbus/include/mbutils.h"	// ?? без этого есть предупреждалки
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t CurrentKeyPressed = 0;
bool KeyPressProcessed = false;

uint8_t CardID[8] = 		{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};			// Буфер приема карты
uint8_t CurrentCardID[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};			// Текущая считанная карта

uint32_t CurrentTime = 0;

uint32_t lastRGBBlinkTime = 0;

uint8_t LEDs[3] = {0xFF, 0xFF, 0xFF};

bool StartPlay = false;					// Запуск воспроизведения
uint32_t StartPlayTime = 0;				// Время старта воспроизведения
uint8_t CurrentPlayTone = 0;			//
uint16_t CurrentTonePlayTime = 0;		// Время проигрывания текущего звука
uint16_t CurrentTone = 0;				// Частота проигрывания текущего звука

uint32_t LastCardReadTime = 0;														// Время последнего считывания карты

uint8_t CurrentRepairCode = 0;			// Текущий код ремонта
uint8_t CurrentDowntimeCode = 0;		// Текущий код простоя

// Переменные для мигания
uint32_t PrevBlinkTime = 0;
bool BlinkState = false;

uint16_t lastLightsState = 0;			// Последнее состояние светофора


/* Глобальные переменные регистров */
uint8_t SystemStateInput = 0;			// Переменная Состояния Системы
uint8_t LightsStateInput = 0;			// Переменная Светофора

PVKPMode CurrentMode = UnknownMode;		// Переменная текущего глобального состояния ПВКП

eMBErrorCode eStatus = 0;				// Переменная ошибки устройства OneWire

one_wire_t ow;							// Переменная устройства OneWire
one_wire_rom_id_t NFCCard;				// Переменная

uint32_t LastKeyPressTime = 0;			// Переменная последнего времени нажатия кнопки
/*
uint8_t password[3] = {8, 4, 3};		// Пароль админа
uint8_t passwordPos = 0;				// Позиция ввода пароля

uint8_t ModBusID[3] = {0,0,0};			// Введенный ID
uint8_t ModBusIDPos = 0;				// Позиция ввода ID

uint32_t FLASH_MBID_START_ADDR = 0x08000000 + 63 * FLASH_PAGE_SIZE;
uint32_t FLASH_MBID_END_ADDR = 0x08000000 + 64 * FLASH_PAGE_SIZE;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
/* TIM2 канал 4 - ШИМ пищалки
 * TIM3 -
 * TIM4 канал 1 - для Modbus RTU (RS485)
 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  	//WriteModBusIDToFlash(143);
  	/*
    uint8_t* ModBusIDFromFlash = ReadModBusIDFromFlash();

	if (*ModBusIDFromFlash == 0 || *ModBusIDFromFlash > 250) {
		eStatus = eMBInit(MB_RTU, ModBusSlaveID, 0, 115200, MB_PAR_NONE);
	} else eStatus = eMBInit(MB_RTU, *ModBusIDFromFlash, 0, 115200, MB_PAR_NONE);
	*/

    eMBInit(MB_RTU, ModBusSlaveID, 0, 115200, MB_PAR_NONE);
	eStatus = eMBEnable();

	ow.GPIO = ONEWire_GPIO_Port;
	ow.GPIO_Pin = ONEWire_Pin;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	for (int i = 0; i < NumOfRegs; ++i) {									// Обнуление регистров, на всякий
		ModbusRegistersData16bit[i] = 0x0000;
	}
	ModbusRegistersData16bit[VersionRegister] = 0x4 * 256 + 0x03;		// Версия ПО (Для проверки связи)

	for (int i = LED_PIT; i <= LED_15; ++i) {
		SetLEDPin(i, 1);
		//SetLEDPin(i-1, 0);
		SendToShiftRegisters(LEDs);
		//HAL_Delay(500);
	 }
	HAL_Delay(500);

	SendToShiftRegisters(LEDs);
	//PlayOKTones();

	//WriteToFlash(0x5678);

	//uint16_t* fls = ReadModBusIDFromFlash();
	SwitchCurrentPVKPMode(WaitForCard);

	//FLASH->KEYR = ((uint32_t)0x45670123);
	//HAL_FLASH_Program()

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  eMBPoll();
	  LoopPlayer();
	  ProcessCurrentLEDs();
	  ProcessCurrentLights();
	  ProcessCurrentMode();
	  ProcessKeyPad();

	  // Проброс регистров
	  usSRegHoldBuf[OutputRegister] = ModbusRegistersData16bit[OutputRegister];
	  ModbusRegistersData16bit[InputModeRegister] = usSRegHoldBuf[InputModeRegister];
	  ModbusRegistersData16bit[InputLightRegister] = usSRegHoldBuf[InputLightRegister];
	  for (int i = 3; i <= 9; ++i) {						// Переброс переменной в буфер библиотеки Modbus RTU
		  usSRegHoldBuf[i] = ModbusRegistersData16bit[i];
	  }

	  //RGBBlink();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/*
uint8_t* ReadModBusIDFromFlash (void) {
	return (volatile uint8_t *)FLASH_MBID_START_ADDR;
}
void WriteModBusIDToFlash (uint8_t data) {
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError = 0;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_MBID_START_ADDR;
	EraseInitStruct.NbPages     = (FLASH_MBID_END_ADDR - FLASH_MBID_START_ADDR) / FLASH_PAGE_SIZE;

	HAL_FLASH_Unlock();                     		// Разблокируем флеш память
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) Error_Handler();
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_MBID_START_ADDR, data));
	HAL_FLASH_Lock();                        // Блокируем флеш память
}
*/

void ReadNFCCard (void) {										// Считывание NFC карты
	uint32_t CurrentTime = HAL_GetTick();
	if (CurrentTime - LastCardReadTime > 2000) {						// Если прошло не меньше 2 секунд
		if (one_wire_reset(&ow)) { 										// если обнаружена карта
			one_wire_read_rom(&ow, &NFCCard);							// отправляем команду "считать ROM"
			if (NFCCard.family_code > 0) {								// Если считана карта

				CardID[0] = NFCCard.family_code;
				CardID[1] = NFCCard.serial[0];
				CardID[2] = NFCCard.serial[1];	// байт 3
				CardID[3] = NFCCard.serial[2];	// байт 2
				CardID[4] = NFCCard.serial[3];	// байт 1
				CardID[5] = NFCCard.serial[4];
				CardID[6] = NFCCard.serial[5];
				CardID[7] = NFCCard.crc;

				// Обнуление карты в переменной
				NFCCard.family_code = 0;
				NFCCard.serial[0] = 0;
				NFCCard.serial[1] = 0;
				NFCCard.serial[2] = 0;
				NFCCard.serial[3] = 0;
				NFCCard.serial[4] = 0;
				NFCCard.serial[5] = 0;
				NFCCard.crc = 0;
			}

			if (!CardMatches(CardID, CurrentCardID)) {
				LastCardReadTime = CurrentTime;
				for (int i=0; i<8; i++) CurrentCardID[i] = CardID[i];	// Записываем карту как последнюю прочитанную
			}
		}
	}
}

bool CardMatches (uint8_t Card1[8], uint8_t Card2[8]) {			// Возвращает совпаденают ли карты
	int k = 0;
	for (int i=0; i < 8; i++) if (Card1[i] == Card2[i]) k++;
	if (k >= 8) return true; else return false;
}

bool IsNewCurrentCard (void) {
	bool NewCard = false;
	for (int i = 0; i < 8; i++) {
		if (CurrentCardID[i] > 0x00) NewCard = true;
	}
	return NewCard;
}

void WriteCurrentCardToCardRegisters (void) {
	/* Запись карты в 3-6 регистры для проверки */
	ModbusRegistersData16bit[CardRegister1] = CurrentCardID[3] * 256 + CurrentCardID[2];
	ModbusRegistersData16bit[CardRegister2] = CurrentCardID[1] * 256;

	/*
	ModbusRegistersData16bit[CardRegister1] = CurrentCardID[0] * 256 + CurrentCardID[1];
	ModbusRegistersData16bit[CardRegister2] = CurrentCardID[2] * 256 + CurrentCardID[3];
	ModbusRegistersData16bit[CardRegister3] = CurrentCardID[4] * 256 + CurrentCardID[5];
	ModbusRegistersData16bit[CardRegister4] = CurrentCardID[6] * 256 + CurrentCardID[7];
	*/
}

void ProcessCurrentMode (void) {								// Автомат обработки переключения режимов
	if (CurrentMode == WaitForCard) {							// <<=== Если Режим = Ожидание авторизации картой (!ВЫСШИЙ ПРИОРИТЕТ!)
		ReadNFCCard();											// Считываем карту
		if (IsNewCurrentCard()) {										// Если есть считанная карта
			PlayOKTones();
			WriteCurrentCardToCardRegisters();
			SwitchCurrentPVKPMode(CardLevelRequest);			// Режим = Запрос уровня доступа
		}
	} else if (CurrentMode == CardLevelRequest) {				// <<=== Если Режим = запрос уровня доступа и есть ответ от Системы
		GetSystemInputStateFromRegister();								// Проверяем ответ от Системы
		switch (SystemStateInput) {
			case AuthorizedOperator:							// Авторизован Оператор (код 0x1)
				SwitchCurrentPVKPMode(Authorized);
				//TODO PlayOperatorCardTones();
				break;
			case AuthorizedMechanic:							// Авторизован Механик (код 0x2)
				SwitchCurrentPVKPMode(Authorized);
				//TODO PlayMechanicCardTones();
				break;

			/*case AuthorizedAdmin:								// Авторизован Админ (не реализовано)
				//TODO Моргать по-другому, вход в режим админа
				break;*/
			case AuthorizeError:								// Система прислала ошибку авторизации (код 0xA)
				PlayBadTones();
				SwitchCurrentPVKPMode(WaitForCard);
				break;
			default:
				break;
		}
	} else if (CurrentMode == Downtime_Started) {				// <<=== Если Режим = Начало простоя
		//TODO Старт таймера ?
	}
}

void ResetKeypadLEDs (void) {
	for (int i = LED_OBR; i <= LED_15; i++) SetLEDPin(i, 0);	// Перебор всех LED и отключение
}

void GetSystemInputStateFromRegister (void) {					// Чтение всех регистров в глобальные переменные
	SystemStateInput = ModbusRegistersData16bit[InputModeRegister];//HighByteOfWord(
}

void GetLightsInputStateFromRegister (void) {
	LightsStateInput = ModbusRegistersData16bit[InputLightRegister];//LowByteOfWord(
}

void ResetInputRegisters (void) {								// Обнуление входящего регистра
	ModbusRegistersData16bit[InputModeRegister] = 0x0000;
	ModbusRegistersData16bit[InputLightRegister] = 0x0000;
	SystemStateInput = 0;
	LightsStateInput = 0;
}

void ProcessKeyPad (void) {										// Обработка нажатий клавиатуры
	uint8_t key = CurrentKeyPressed;
	if (!KeyPressProcessed && key > 0) {
		switch (CurrentMode) {
			case WaitForCard:											// Если РЕЖИМ = Ожидание карты
				/*if (key == 13) {										// Вход в Сервисный режим ??
					SwitchCurrentPVKPMode(AdminModeRequest);			// Запрос пароля админа
				} else PlayBadTones();
				KeyPressProcessed = true;*/
				break;
			/*case AdminModeRequest:
				if (key == password[passwordPos]) {
					passwordPos++;
					PlayOKTones();
				} else if (key == 15) {
					passwordPos = 0;
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} //else PlayBadTones();
				if (passwordPos == 3) {
					passwordPos = 0;
					SwitchCurrentPVKPMode(AdminMode);
				}
				KeyPressProcessed = true;
				break;
			case AdminMode:												// Ввод ModBus ID
				if (key >= 1 && key <= 9) {	// 1-9
					ModBusID[ModBusIDPos] = key;
					ModBusIDPos++;
					PlayOKTones();
				} else if (key == 10) {		// 0
					ModBusID[ModBusIDPos] = 0;
					ModBusIDPos++;
					PlayOKTones();
				} else if (key == 13 && ModBusIDPos == 3) {				// Если нажать 13 после 3й цифры, сохраняем
					uint8_t EnteredID = ModBusID[0] * 100 + ModBusID[1] * 10 + ModBusID[2];
					WriteModBusIDToFlash(EnteredID);
					ModBusIDPos = 0;
					uint8_t* ModBusIDFromFlash = ReadModBusIDFromFlash();
					eStatus = eMBInit(MB_RTU, ModBusSlaveID, 0, 115200, MB_PAR_NONE);
					PlayOKTones();
				} else if (key == 15) {										// СДАЧА СМЕНЫ (15)
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				if (ModBusIDPos >= 4) {									// Если больше 3х цифр, выходим
					ModBusIDPos = 0;
					SwitchCurrentPVKPMode(WaitForCard);
					PlayBadTones();
				}
				KeyPressProcessed = true;
				break;*/
			case CardLevelRequest:										// Если РЕЖИМ = Запрос уровня доступа
				if (key == 15) {										// СДАЧА СМЕНЫ ('15')
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Authorized:											// Если РЕЖИМ = Авторизован
				if (key == 15) {										// СДАЧА СМЕНЫ ('15')
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Working:												// Если РЕЖИМ = "Обработка" - клавиатура отключена
				if (key == 15) {										// СДАЧА СМЕНЫ ('15')
					PlayOKTones();
					ResetKeypadLEDs();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Downtime_Started:										// Если РЕЖИМ = Начало простоя (таймер?)
				if (key == 15) {										// СДАЧА СМЕНЫ ('15')
					PlayOKTones();
					ResetKeypadLEDs();
					SwitchCurrentPVKPMode(WaitForCard);
				} else if (key == 13 && SystemStateInput == AuthorizedOperator) {	// Если ОПЕРАТОР нажимает ПРОСТОЙ
					PlayOKTones();
					SwitchCurrentPVKPMode(Downtime_DowntimeChoice);		// Режим = Выбор ПРОСТОЯ
				} else if (key == 14 && SystemStateInput == AuthorizedMechanic) {	// Если МЕХАНИК нажимает РЕМОНТ
					PlayOKTones();
					SwitchCurrentPVKPMode(Downtime_RepairChoice);		// Режим = Выбор РЕМОНТА
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Downtime_DowntimeChoice:								// Если РЕЖИМ = Выбор причины ПРОСТОЯ
				if (key >= 1 && key <= 12) {
					PlayOKTones();
					CurrentDowntimeCode = key;
					SwitchCurrentPVKPMode(Downtime_DowntimeSelected);	// Режим = Выбрана причина простоя
					ModbusRegistersData16bit[DownTimeRegister] = key;
				} else if (key == 15) {								// СДАЧА СМЕНЫ (15)
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Downtime_DowntimeSelected:								// Если РЕЖИМ = Выбрана причина ПРОСТОЯ
				if (key >= 1 && key <= 12) {							// ПЕРЕвыбор причины
					PlayOKTones();
					CurrentDowntimeCode = key;
					SwitchCurrentPVKPMode(Downtime_DowntimeSelected);	// Режим = Выбрана причина простоя
					ModbusRegistersData16bit[DownTimeRegister] = key;
				} else if (key == 15) {										// СДАЧА СМЕНЫ (15)
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Downtime_RepairChoice:									// Если РЕЖИМ = Выбор причины РЕМОНТА
				if (key >= 1 && key <= 8) {
					PlayOKTones();
					CurrentRepairCode = key;
					SwitchCurrentPVKPMode(Downtime_ReparSelected);
					ModbusRegistersData16bit[RepairRegister] = key;
				} else if (key == 15 || key == 9) {					// СДАЧА СМЕНЫ (15) или "ВЫВОД ИЗ РЕМОНТА"
					PlayOKTones();
					SwitchCurrentPVKPMode(Downtime_DowntimeChoice);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			case Downtime_ReparSelected:								// Выбрана причина РЕМОНТА
				if (key >= 1 && key <= 8) {								// ПЕРЕвыбор причины
					PlayOKTones();
					CurrentRepairCode = key;
					SwitchCurrentPVKPMode(Downtime_ReparSelected);
					ModbusRegistersData16bit[RepairRegister] = key;
				} else if (key == 15 || key == 9) {							// СДАЧА СМЕНЫ (15) / Вывод из ремонта
					PlayOKTones();
					SwitchCurrentPVKPMode(WaitForCard);
				} else PlayBadTones();
				KeyPressProcessed = true;
				break;
			default:
				KeyPressProcessed = true;
				break;
		}
	}
}

void ProcessCurrentLEDs (void) {								// Обработка отображения текущего состояния
	uint32_t CurrentTime = HAL_GetTick();
	if (CurrentMode >= WaitForCard) SetLEDPin(LED_PIT, 1);
	if (CurrentMode >= Authorized) SetLEDPin(LED_KAR, 1);		// Если есть авторизация, "КАР" горит постоянно
	GetSystemInputStateFromRegister();							// Проверяем ответ от Системы
	switch (CurrentMode) {
		case WaitForCard:										// Если РЕЖИМ = ОЖИДАНИЕ КАРТЫ
			if (CurrentTime - PrevBlinkTime > 300) {			// Моргаем LED "КАР"
				PrevBlinkTime = CurrentTime;
				BlinkState = !BlinkState;
				SetLEDPin(LED_KAR, BlinkState);
			}
			break;
		case CardLevelRequest:									// Если РЕЖИМ = ЗАПРОС УРОВНЯ ДОСТУПА
			switch (SystemStateInput) {
				case UnknownInputState:								// Если в регистре "0"
					if (CurrentTime - PrevBlinkTime > 150) {		// Моргаем LED "КАР"
						PrevBlinkTime = CurrentTime;
						BlinkState = !BlinkState;
						SetLEDPin(LED_KAR, BlinkState);
					}
					break;
				case AuthorizedOperator:							// Авторизован Оператор (код 0x1)
					SetLEDPin(LED_KAR, 1);
					break;
				case AuthorizedMechanic:							// Авторизован Механик (код 0x2)
					SetLEDPin(LED_KAR, 1);
					break;
				/*case AuthorizedAdmin:								// Авторизован Админ (не реализовано)
					//TODO Моргать по-другому, вход в режим админа
					break;*/
			}
			break;
		case Working:											// Если РЕЖИМ = ОБРАБОТКА
			if (CurrentTime - PrevBlinkTime > 1000) {				// Моргаем LED "ОБР"
				PrevBlinkTime = CurrentTime;
				BlinkState = !BlinkState;
				SetLEDPin(LED_OBR, BlinkState);
			}
			break;
		case Downtime_Started:									// Если РЕЖИМ = НАЧАЛО ПРОСТОЯ
			if (CurrentTime - PrevBlinkTime > 300) {
				PrevBlinkTime = CurrentTime;
				BlinkState = !BlinkState;
				if (SystemStateInput == AuthorizedOperator) {			// Если Оператор,
					SetLEDPin(LED_PRS, BlinkState);						// Моргаем "Простой"
					SetLEDPin(LED_13, BlinkState);
					SetLEDPin(LED_REM, 0);
				} else if (SystemStateInput == AuthorizedMechanic) {	// Если Механик,
					SetLEDPin(LED_REM, BlinkState);						// Моргаем "Ремонт"
					SetLEDPin(LED_14, BlinkState);
					SetLEDPin(LED_PRS, 0);
				} else {
					//Serial.println("No access level!");
				}
			}
			break;
		case Downtime_DowntimeChoice:							// Если РЕЖИМ = ВЫБОР ПРИЧИНЫ ПРОСТОЯ
			SetLEDPin(LED_REM, 0);
			SetLEDPin(LED_PRS, 1);
			SetLEDPin(LED_13, 1);
			if (CurrentTime - PrevBlinkTime > 300) {
				PrevBlinkTime = CurrentTime;
				BlinkState = !BlinkState;
				for (int i = LED_1; i <= LED_12; i++) SetLEDPin(i, BlinkState);
			}
			break;
		case Downtime_RepairChoice:								// Если РЕЖИМ = ВЫБОР ПРИЧИНЫ РЕМОНТА
			SetLEDPin(LED_REM, 1);
			SetLEDPin(LED_PRS, 0);
			SetLEDPin(LED_14, 1);
			if (CurrentTime - PrevBlinkTime > 300) {
				PrevBlinkTime = CurrentTime;
				BlinkState = !BlinkState;
				for (int i = LED_1; i <= LED_9; i++) SetLEDPin(i, BlinkState);
			}
			break;
		case Downtime_DowntimeSelected:							// Если РЕЖИМ = ПРИЧИНА ПРОСТОЯ ВЫБРАНА
			SetLEDPin(LED_REM, 0);
			SetLEDPin(LED_PRS, 1);
			SetLEDPin(LED_13, 1);
			SetLEDPin(CurrentDowntimeCode + 4, 1);
			break;
		case Downtime_ReparSelected:							// Если РЕЖИМ = ПРИЧИНА РЕМОНТА ВЫБРАНА
			SetLEDPin(LED_REM, 1);
			SetLEDPin(LED_PRS, 0);
			SetLEDPin(LED_14, 1);
			SetLEDPin(CurrentRepairCode + 4, 1);
			break;
		/*
		case AdminModeRequest:									// Если РЕЖИМ = Админ (запрос)
			if (CurrentTime - PrevBlinkTime > 300) {
				PrevBlinkTime = CurrentTime;
				BlinkState = !BlinkState;
				SetLEDPin(LED_OBR, BlinkState);
				SetLEDPin(LED_PRS, BlinkState);
				SetLEDPin(LED_REM, BlinkState);
			}
			break;
		case AdminMode:											// Если РЕЖИМ = Админ
			SetLEDPin(LED_OBR, 1);
			SetLEDPin(LED_PRS, 1);
			SetLEDPin(LED_REM, 1);
			break;*/
		default:
			break;
	}
}

void ProcessCurrentLights (void) {
	GetLightsInputStateFromRegister();
	GetSystemInputStateFromRegister();
	if (CurrentMode >= Authorized && CurrentMode < Downtime_RepairChoice) {
		if (LightsStateInput == RedLight) {
			if (SystemStateInput == 1 || SystemStateInput == 2)
				if (CurrentMode != Downtime_Started) SwitchCurrentPVKPMode(Downtime_Started);
		} else if (LightsStateInput == GreenLight) {
			if (CurrentMode != Working) SwitchCurrentPVKPMode(Working);
		} else {											// Если нет сигнала светофора
			//ShowQuestionChar();
		}
	} else if (CurrentMode >= Downtime_ReparSelected) {
		if (LightsStateInput == GreenLight) {
			SwitchCurrentPVKPMode(Working);
		} else {											// Если нет сигнала светофора
			//ShowQuestionChar();
		}
	}
}

void AnnulateCurrentCard (void) {								// Аннулирование текущей карты (стирает CurrentCardID)
	for (int i = 0; i < 8; i++) CurrentCardID[i] = 0;
	ModbusRegistersData16bit[InputModeRegister] = 0x00;
	ModbusRegistersData16bit[CardRegister1] = 0x00;
	ModbusRegistersData16bit[CardRegister2] = 0x00;
}


void SwitchCurrentPVKPMode (PVKPMode Mode) {
	if (Mode == WaitForCard) {
		AnnulateCurrentCard();					// Аннулирование карты
		ResetInputRegisters();
	/*
	} else if (Mode == AdminMode) {
		//PlayOKTones();
		//PlayBadTones();*/
	}
	ResetKeypadLEDs();
	CurrentMode = Mode;								// Переключение режима
	ModbusRegistersData16bit[OutputRegister] = (Mode << 8);			// Запись состояния в верхний байт 0-го регистра

	ModbusRegistersData16bit[DownTimeRegister] = 0x0000;			// Сброс регистров причин простоя
	ModbusRegistersData16bit[RepairRegister] = 0x0000;
	PlaySwitchTones();
}

void PlayOKTones (void) {
	StartPlay = true;
	StartPlayTime = HAL_GetTick();
	CurrentTone = 1000;
	CurrentTonePlayTime = 100;
}

void PlayBadTones(void) {
	StartPlay = true;
	StartPlayTime = HAL_GetTick();
	CurrentTone = 300;
	CurrentTonePlayTime = 200;
}

void PlaySwitchTones(void) {
	StartPlay = true;
	StartPlayTime = HAL_GetTick();
	CurrentTone = 5000;
	CurrentTonePlayTime = 100;
}

void LoopPlayer(void) {
	if (StartPlay) {
		if (HAL_GetTick() < StartPlayTime + CurrentTonePlayTime) {
			BuzzerSetFreq(CurrentTone);
			BuzzerSetVolume(10);
		} else {
			StartPlay = false;
			BuzzerSetVolume(0);
		}
	} else {
		StartPlay = false;
		BuzzerSetVolume(0);
	}
}

void SendToShiftRegisters(uint8_t* LEDs) {
  uint8_t *aTxBuffer;
  cs_reset();
  aTxBuffer = &LEDs[2];
  HAL_SPI_Transmit(&hspi2, (uint8_t*)aTxBuffer, 1, 50);
  //cs_strob();
  aTxBuffer = &LEDs[1];
  HAL_SPI_Transmit(&hspi2, (uint8_t*)aTxBuffer, 1, 50);
  //cs_strob();
  aTxBuffer = &LEDs[0];
  HAL_SPI_Transmit(&hspi2, (uint8_t*)aTxBuffer, 1, 50);
  //cs_strob();
  cs_set();
  cs_reset();
}

/* Вкл и Выкл светодиодов */
void SetLEDPin (int led, bool state) {
	switch (led) {
		case LED_PIT:
			if (state) LEDs[0] &= ~(1 << 0);
				else LEDs[0] |= (1 << 0);
			SendToShiftRegisters(LEDs);
			break;
		case LED_KAR:
			if (state) LEDs[0] &= ~(1 << 1);
				else LEDs[0] |= (1 << 1);
			SendToShiftRegisters(LEDs);
			break;
		case LED_OBR:
			if (state) LEDs[0] &= ~(1 << 2);
				else LEDs[0] |= (1 << 2);
			SendToShiftRegisters(LEDs);
			break;
		case LED_REM:
			if (state) LEDs[1] &= ~(1 << 7);
				else LEDs[1] |= (1 << 7);
			SendToShiftRegisters(LEDs);
			break;
		case LED_PRS:
			if (state) LEDs[1] &= ~(1 << 6);
				else LEDs[1] |= (1 << 6);
			SendToShiftRegisters(LEDs);
			break;
		case LED_1:
			if (state) LEDs[0] &= ~(1 << 3);
				else LEDs[0] |= (1 << 3);
			SendToShiftRegisters(LEDs);
			break;
		case LED_2:
			if (state) LEDs[1] &= ~(1 << 4);
				else LEDs[1] |= (1 << 4);
			SendToShiftRegisters(LEDs);
			break;
		case LED_3:
			if (state) LEDs[1] &= ~(1 << 5);
				else LEDs[1] |= (1 << 5);
			SendToShiftRegisters(LEDs);
			break;
		case LED_4:
			if (state) LEDs[0] &= ~(1 << 4);
				else LEDs[0] |= (1 << 4);
			SendToShiftRegisters(LEDs);
			break;
		case LED_5:
			if (state) LEDs[1] &= ~(1 << 2);
				else LEDs[1] |= (1 << 2);
			SendToShiftRegisters(LEDs);
			break;
		case LED_6:
			if (state) LEDs[1] &= ~(1 << 3);
				else LEDs[1] |= (1 << 3);
			SendToShiftRegisters(LEDs);
			break;
		case LED_7:
			if (state) LEDs[0] &= ~(1 << 5);
				else LEDs[0] |= (1 << 5);
			SendToShiftRegisters(LEDs);
			break;
		case LED_8:
			if (state) LEDs[1] &= ~(1 << 0);
				else LEDs[1] |= (1 << 0);
			SendToShiftRegisters(LEDs);
			break;
		case LED_9:
			if (state) LEDs[1] &= ~(1 << 1);
				else LEDs[1] |= (1 << 1);
			SendToShiftRegisters(LEDs);
			break;
		case LED_10:
			if (state) LEDs[0] &= ~(1 << 6);
				else LEDs[0] |= (1 << 6);
			SendToShiftRegisters(LEDs);
			break;
		case LED_11:
			if (state) LEDs[2] &= ~(1 << 2);
				else LEDs[2] |= (1 << 2);
			SendToShiftRegisters(LEDs);
			break;
		case LED_12:
			if (state) LEDs[2] &= ~(1 << 3);
				else LEDs[2] |= (1 << 3);
			SendToShiftRegisters(LEDs);
			break;
		case LED_13:
			if (state) LEDs[0] &= ~(1 << 7);
				else LEDs[0] |= (1 << 7);
			SendToShiftRegisters(LEDs);
			break;
		case LED_14:
			if (state) LEDs[2] &= ~(1 << 0);
				else LEDs[2] |= (1 << 0);
			SendToShiftRegisters(LEDs);
			break;
		case LED_15:
			if (state) LEDs[2] &= ~(1 << 1);
				else LEDs[2] |= (1 << 1);
			SendToShiftRegisters(LEDs);
			break;
	}
}

/*
void ReadNFCCard(void) {
	if (one_wire_reset(&OneWireDevice)) {
		one_wire_read_rom(&OneWireDevice, &NFCCard);
		if (NFCCard.family_code > 0) {
		  CurrentCardID[0] = NFCCard.family_code;
		  for (int i = 1; i < 5; ++i) CurrentCardID[i] = NFCCard.serial[i-1];
		  CurrentCardID[7] = NFCCard.crc;
		}
	}
}
*/

void RGBBlink (void) {
	CurrentTime = HAL_GetTick();
	uint32_t TargetTime = CurrentTime - lastRGBBlinkTime;
	uint8_t interval = 100;
	if (TargetTime > interval * 1 && TargetTime < interval * 1 + 5) {
		HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 0);
	} else if (TargetTime > interval * 2 && TargetTime < interval * 2 + 5) {
		HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 1);
	} else if (TargetTime > interval * 3 && TargetTime < interval * 3 + 5) {
		HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 0);
	} else if (TargetTime > interval * 4 && TargetTime < interval * 4 + 5) {
		HAL_GPIO_WritePin(RGB_BLUE_GPIO_Port, RGB_BLUE_Pin, 1);

	} else if (TargetTime > interval * 5 && TargetTime < interval * 5 + 5) {
		HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 0);
	} else if (TargetTime > interval * 6 && TargetTime < interval * 6 + 5) {
		HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 1);
	} else if (TargetTime > interval * 7 && TargetTime < interval * 7 + 5) {
		HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 0);
	} else if (TargetTime > interval * 8 && TargetTime < interval * 8 + 5) {
		HAL_GPIO_WritePin(RGB_RED_GPIO_Port, RGB_RED_Pin, 1);
	} else if (TargetTime > interval * 15 && TargetTime < interval * 15 + 5) {
		lastRGBBlinkTime = CurrentTime;
	}
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
	uint32_t CurrentTime = HAL_GetTick();
	if (CurrentTime - LastKeyPressTime > 500) {
		LastKeyPressTime = CurrentTime;
		  uint8_t Row = 0;
		  if (HAL_GPIO_ReadPin(GPIOA, ROW0_Pin) == 1) Row = 0;
		  else if (HAL_GPIO_ReadPin(GPIOA, ROW1_Pin) == 1) Row = 1;
		  else if (HAL_GPIO_ReadPin(GPIOA, ROW2_Pin) == 1) Row = 2;
		  else if (HAL_GPIO_ReadPin(GPIOA, ROW3_Pin) == 1) Row = 3;
		  else if (HAL_GPIO_ReadPin(GPIOA, ROW4_Pin) == 1) Row = 4;
		  switch (GPIO_Pin) {
			case COL0_Pin:
				CurrentKeyPressed = (Row * 3) + 1;
				KeyPressProcessed = false;
				break;
			case COL1_Pin:
				CurrentKeyPressed = (Row * 3) + 2;
				KeyPressProcessed = false;
				break;
			case COL2_Pin:
				CurrentKeyPressed = (Row * 3) + 3;
				KeyPressProcessed = false;
				break;
			default:
				CurrentKeyPressed = 0;
				KeyPressProcessed = true;
				break;
		}
	}
}

// M O D B U S   U S E R   F U N C T I O N S

USHORT   usSDiscInStart                               = S_DISCRETE_INPUT_START;
#if S_DISCRETE_INPUT_NDISCRETES%8
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
#else
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8]  ;
#endif
//Slave mode:Coils variables
USHORT   usSCoilStart                                 = S_COIL_START;
#if S_COIL_NCOILS%8
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]                ;
#else
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8]                  ;
#endif

//Slave mode:InputRegister variables
USHORT   usSRegInStart                                = S_REG_INPUT_START;
//USHORT   usSRegInBuf[S_REG_INPUT_NREGS]               ;

//Slave mode:HoldingRegister variables
USHORT   usSRegHoldStart                              = S_REG_HOLDING_START;
//USHORT   usSRegHoldBuf[S_REG_HOLDING_NREGS]           ;

/**
 * Modbus slave input register callback function.
 *
 * @param pucRegBuffer input register buffer
 * @param usAddress input register address
 * @param usNRegs input register number
 *
 * @return result
 */
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ) {
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    USHORT *        pusRegInputBuf;
    USHORT          REG_INPUT_START;
    USHORT          REG_INPUT_NREGS;
    USHORT          usRegInStart;

    pusRegInputBuf = usSRegInBuf;
    REG_INPUT_START = S_REG_INPUT_START;
    REG_INPUT_NREGS = S_REG_INPUT_NREGS;
    usRegInStart = usSRegInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_INPUT_START)
            && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = usAddress - usRegInStart;
        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (UCHAR) (pusRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR) (pusRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode) {
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    USHORT *        pusRegHoldingBuf;
    USHORT          REG_HOLDING_START;
    USHORT          REG_HOLDING_NREGS;
    USHORT          usRegHoldStart;

    pusRegHoldingBuf = usSRegHoldBuf;
    //pusRegHoldingBuf = ModbusRegistersData16bit;
    REG_HOLDING_START = S_REG_HOLDING_START;
    REG_HOLDING_NREGS = S_REG_HOLDING_NREGS;
    usRegHoldStart = usSRegHoldStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_HOLDING_START)
            && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - usRegHoldStart;
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode) {
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucCoilBuf;
    USHORT          COIL_START;
    USHORT          COIL_NCOILS;
    USHORT          usCoilStart;
    iNReg =  usNCoils / 8 + 1;

    pucCoilBuf = ucSCoilBuf;
    COIL_START = S_COIL_START;
    COIL_NCOILS = S_COIL_NCOILS;
    usCoilStart = usSCoilStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= COIL_START ) &&
        ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (usAddress - usCoilStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
        switch ( eMode )
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++],
                        iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
                        *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
                        *pucRegBuffer++);
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/**
 * Modbus slave discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete ) {
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          DISCRETE_INPUT_START;
    USHORT          DISCRETE_INPUT_NDISCRETES;
    USHORT          usDiscreteInputStart;
    iNReg =  usNDiscrete / 8 + 1;

    pucDiscreteInputBuf = ucSDiscInBuf;
    DISCRETE_INPUT_START = S_DISCRETE_INPUT_START;
    DISCRETE_INPUT_NDISCRETES = S_DISCRETE_INPUT_NDISCRETES;
    usDiscreteInputStart = usSDiscInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= DISCRETE_INPUT_START)
            && (usAddress + usNDiscrete    <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES))
    {
        iRegIndex = (USHORT) (usAddress - usDiscreteInputStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usDiscreteInputStart) % 8;

        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++],
                    iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}


static uint32_t lock_nesting_count = 0;
void __critical_enter(void)
{
    __disable_irq();
    ++lock_nesting_count;
}

void __critical_exit(void)
{
    /* Unlock interrupts only when we are exiting the outermost nested call. */
    --lock_nesting_count;
    if (lock_nesting_count == 0) {
        __enable_irq();
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
