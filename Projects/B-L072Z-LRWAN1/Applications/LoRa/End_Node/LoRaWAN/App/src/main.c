/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * 		 Adrien FRAVAL
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/*PA10 -> RX
 * PB6 -> TX
 * PB15 -> CS
 * PA0 -> Battery
 * PA4 -> Tore
 * PA5 -> HUMIDITE
 * PB14 -> TOR
 * PB13 -> TOR
 */


/* Includes ------------------------------------------------------------------*/

#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "settings.h"

/* Private typedef -----------------------------------------------------------*/
#define CapteurON	1
#define CapteurOFF	0
/* Private define ------------------------------------------------------------*/



/*!
 * CAYENNE_LPP is myDevices Application server.
 */
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_DATATYPE_CONCENTRATION_PM1  0x5D
#define LPP_DATATYPE_CONCENTRATION_PM25  0x6D
#define LPP_DATATYPE_CONCENTRATION_PM10  0x7D
#define LPP_DATATYPE_POWER_MEASUREMENT  0x69
#define LPP_DATATYPE_CURRENT 0x70
#define LPP_DATATYPE_SWITCH 0x71
#define LPP_DATATYPE_HUMIDITE 0x73
#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            1500
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_C
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_CONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];
static uint8_t AppDataBuffRx[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppDataRx={ AppDataBuffRx,  0 ,0 };

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/* Get the PM (particulate matters) concentration in ug/m3 */
void GetParticule(void);

/*Try to active the PM sensor  */
void GetSensorState(void);

/* Init PM sensor before getting data */
void InitPMSensor(void);

/* Active the PM sensor */
void ParticuleSensorON(void);

/* Deactivates the PM sensor */
void ParticuleSensorOFF(void);

/*Active or deactivate the PM sensor */
void SensorControl(uint8_t EtatCapteur);

/*Decode the request receive by LoRa */
void DecodeRequest(void);

/*Check if all necessary data are collected and call the function */
void CheckBeforeSend(void);

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* LoRa endNode send request*/
static void Send( void* context );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void* context );

/* tx timer callback function*/
static void LoraMacProcessNotify( void );

/* Private variables ---------------------------------------------------------*/

/* Variables from settings.h */
const uint8_t BATTERY_ID;
const uint8_t PMSENSOR1_ID;
const uint8_t PMSENSOR25_ID;
const uint8_t PMSENSOR10_ID;
const uint8_t TEMPERATURE_ID;
const uint8_t SWITCH_ID;
const uint8_t HUMIDITE_ID;
uint8_t DevEui[8];
uint8_t JoinEui[8];
uint8_t AppKey[16];
uint8_t NwkKey[16];

/* Analog sensors */
volatile uint16_t BatteryValue = 0;
volatile uint16_t TEMPERATUREValue = 0;
volatile uint8_t SwitchValue = 0;
volatile uint16_t HUMIDITEValue = 0;

volatile uint8_t BatteryValueLow;
volatile uint8_t BatteryValueHigh;
volatile uint8_t TEMPERATUREValueLow;
volatile uint8_t TEMPERATUREValueHigh;
volatile uint8_t HUMIDITEValueLow;
volatile uint8_t HUMIDITEValueHigh;

volatile char GetBatteryLevel = 0;
volatile char GetTEMPERATURE = 0;
volatile char GetSwitch = 0;
volatile char GetHUMIDITE = 0;

volatile char BatteryLevelObtain = 0;
volatile char TEMPERATUREObtain = 0;
volatile char SwitchObtain = 0;
volatile char HUMIDITEObtain = 0;

volatile char BatteryLevelAsked = 0;
volatile char TEMPERATUREAsked = 0;
volatile char SwitchAsked = 0;
volatile char HUMIDITEAsked = 0;

/* Switch specifications */
volatile int cptSwitch = 20;
volatile char SwitchChanged = 0;
volatile char SwitchWarning = 0;

/*Numeric sensor */
volatile char UARTReceived = 0;
uint8_t TransmitUART[3];
uint8_t ReceiveUART[16];
uint8_t DataParticule[6];
volatile uint8_t DataErrorSensor= 0x00;
volatile char ParticuleDataReceive = 0;
volatile char SensorStateReceive = 0;
volatile char SensorErrorReceive = 0;
volatile uint8_t SensorTimerCpt = 0;

volatile char GetPM = 0;
volatile char PMObtain = 0;
volatile char PMAsked = 0;

enum PSensorState {
	State0, State1, State2
};
volatile enum PSensorState PSensorState = State0;

/* LoRa Transmission variables */

char TxReady = 0;
volatile int cpt = 0;
volatile char TxProcessing = 0;



/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify};
LoraFlagStatus LoraMacProcessRequest=LORA_RESET;
LoraFlagStatus AppProcessRequest=LORA_RESET;
/*!
 * Specifies the state of the application LED
 */
//static uint8_t AppLedStateOn = RESET;
                                               
static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void* context );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init();
  
  /* Configure the system clock*/
  SystemClock_Config();
  
  /* Configure the debug mode*/
  DBG_Init();
  
  /* Configure the hardware*/
  HW_Init();
  
  /* USER CODE BEGIN 1 */
  /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_USART1_UART_Init();


    HAL_TIM_Base_Start_IT(&htim6);
    ParticuleSensorOFF();

  /* USER CODE END 1 */
  
  /*Disable Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  LORA_Join();

  LoraStartTx( TX_ON_TIMER) ;
  
  while( 1 )
  {
    if (AppProcessRequest==LORA_SET)
    {
      /*reset notification flag*/
      AppProcessRequest=LORA_RESET;

      GetBatteryLevel = 1;
      BatteryLevelAsked = 1;

      GetPM = 1;
      PMAsked = 1;
      InitPMSensor();


      GetHUMIDITE = 1;
      HUMIDITEAsked = 1;

      GetSwitch = 1;
      SwitchAsked = 1;

      GetTEMPERATURE = 1;
      TEMPERATUREAsked = 1;
    }
	if (LoraMacProcessRequest==LORA_SET)
    {
      /*reset notification flag*/
      LoraMacProcessRequest=LORA_RESET;
      LoRaMacProcess( );
    }
    /*If a flag is set at this point, mcu must not enter low power and must loop*/
    DISABLE_IRQ( );
    
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */
    if ((LoraMacProcessRequest!=LORA_SET) && (AppProcessRequest!=LORA_SET))
    {
#ifndef LOW_POWER_DISABLE
      LPM_EnterLowPower( );
#endif
    }

    ENABLE_IRQ();
    
	if(SwitchWarning)
	{
		Send(NULL);
	}
	if (GetBatteryLevel || GetTEMPERATURE || GetHUMIDITE)
	{
		GetBatteryLevel = 0;
		GetTEMPERATURE = 0;
		GetHUMIDITE = 0;

		HAL_ADC_Start(&hadc);

		HAL_ADC_PollForConversion(&hadc, 100);
		BatteryValue = HAL_ADC_GetValue(&hadc);

		HAL_ADC_PollForConversion(&hadc, 100);
		TEMPERATUREValue = HAL_ADC_GetValue(&hadc);

		HAL_ADC_PollForConversion(&hadc, 100);
		HUMIDITEValue = HAL_ADC_GetValue(&hadc);

		HAL_ADC_Stop (&hadc);
		if(BatteryLevelAsked)
		{
			BatteryLevelObtain = 1;
			GetBatteryLevel = 0;
		}
		if(HUMIDITEAsked)
		{
			HUMIDITEObtain = 1;
			GetHUMIDITE = 0;
		}
		if(TEMPERATUREAsked)
		{
			TEMPERATUREObtain = 1;
			GetTEMPERATURE = 0;
		}

	}
	if(GetPM)
	{
		switch (PSensorState) {
			case State0 : {				//wait 20 sec for initialisation
				if(SensorTimerCpt == 2)
				{
					GetSensorState();
					PSensorState = State1;
					DataErrorSensor = 1;
					SensorStateReceive = 0;
				}
				break;
			}
			case State1 : {			//get sensor state (ON or ERROR) if error, wait for 20s
				if(UARTReceived == 1)
				{
					if(SensorStateReceive)
					{
						SensorStateReceive = 0;
						if(DataErrorSensor%2==0)
						{
							DataErrorSensor = 1;
							HAL_TIM_Base_Stop(&htim7);
							HAL_UART_AbortReceive_IT(&huart1);
							HAL_UART_AbortTransmit_IT(&huart1);
							HAL_TIM_Base_Start(&htim7);
							PSensorState = State2;
							GetParticule();
						}
						else
						{
							GetSensorState();
						}
					}
					else if(SensorTimerCpt == 3)
					{
						GetSensorState();
					}
				}
				else if(SensorTimerCpt == 4)
				{
					HAL_TIM_Base_Stop(&htim7);
					HAL_UART_AbortReceive_IT(&huart1);
					HAL_UART_AbortTransmit_IT(&huart1);
					//Get State failed
					HAL_TIM_Base_Start(&htim7);
					GetParticule();
					PSensorState = State2;
				}
				break;
			}
			case State2 : {				// get data
				if(SensorTimerCpt == 6)
				{
					HAL_TIM_Base_Stop(&htim7);
					HAL_UART_AbortReceive_IT(&huart1);
					HAL_UART_AbortTransmit_IT(&huart1);
					//transmission failed
					ParticuleSensorOFF();
					GetPM = 0;
					PMObtain = 1;
				}
				if(UARTReceived == 1 && ParticuleDataReceive == 1)
				{
					HAL_TIM_Base_Stop(&htim7);
					HAL_UART_AbortReceive_IT(&huart1);
					HAL_UART_AbortTransmit_IT(&huart1);
					ParticuleSensorOFF();
					GetPM = 0;
					PMObtain = 1;
				}
				break;
			}
		}
	}
	if(GetSwitch)
	{
		SwitchValue = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
		GetSwitch = 0;
		SwitchObtain = 1;
	}
    CheckBeforeSend();

  } // End while
}

void GetParticule(void)
{
	HAL_UART_Receive_IT(&huart1, ReceiveUART, 16);
	TransmitUART[0] = 0x81;
	TransmitUART[1] = 0x11;
	TransmitUART[2] = 0x6e;
	UARTReceived = 0;
	HAL_UART_Transmit_IT(&huart1, TransmitUART,3);
}

void GetSensorState(void)
{
	HAL_UART_Receive_IT(&huart1, ReceiveUART, 4);
	TransmitUART[0] = 0x81;
	TransmitUART[1] = 0x16;
	TransmitUART[2] = 0x69;
	UARTReceived = 0;
	HAL_UART_Transmit_IT(&huart1, TransmitUART,3);
}

void InitPMSensor(void)
{
    ParticuleSensorON();
    HAL_TIM_Base_Start_IT(&htim7);
    SensorTimerCpt = 0;
    PSensorState = State0;
}

void ParticuleSensorON(void)
{
	SensorControl(CapteurON);
}

void ParticuleSensorOFF(void)
{
	SensorControl(CapteurOFF);
}

void SensorControl(uint8_t EtatCapteur)
{
	if(EtatCapteur == CapteurON)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, 0);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, 1);
	}
}

void DecodeRequest(void)
{
	if( AppDataRx.BuffSize > 0 && TxProcessing == 0)
	{
		uint8_t BuffSize = AppDataRx.BuffSize;
		for (int i = 0 ; i < BuffSize ; i++)
		{
			uint8_t data = AppDataRx.Buff[i];
			if(data == BATTERY_ID) //Battery level
			{
				BatteryLevelAsked = 1;
				GetBatteryLevel = 1;
				TxProcessing = 1;
			}
			else if(data == PMSENSOR1_ID ) //PM
			{
				PMAsked = 1;
				GetPM = 1;
				TxProcessing = 1;
			}
			else if(data == TEMPERATURE_ID ) //DC Sensor
			{
				TEMPERATUREAsked = 1;
				GetTEMPERATURE = 1;
				TxProcessing = 1;
			}
			else if(data == SWITCH_ID) 	//Switch Sensor
			{
				SwitchAsked = 1;
				GetSwitch = 1;
				TxProcessing = 1;
			}
			else if(data ==HUMIDITE_ID) //HUMIDITE Sensor
			{
				HUMIDITEAsked = 1;
				GetHUMIDITE = 1;
				TxProcessing = 1;
			}
		}
	}
}

void CheckBeforeSend(void)
{
	if(TxProcessing)
	{
		if( BatteryLevelObtain == BatteryLevelAsked && PMObtain == PMAsked && TEMPERATUREObtain == TEMPERATUREAsked && SwitchObtain == SwitchAsked && HUMIDITEObtain == HUMIDITEAsked)
		{
			Send(NULL);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_14)
	{
		SwitchValue = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
		if(cptSwitch > 10)	//if switch hasn't been triggered for a long time, it now triggers
		{
			cptSwitch = 0;
			SwitchWarning = 1;
		}
		else			//wait a moment before send a new alarm
		{
			SwitchChanged = 1;
		}
		//cptSwitch++;
	}
	else
	{
		HW_GPIO_IrqHandler( GPIO_Pin );
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if ( htim->Instance == TIM6)
	{
		GetBatteryLevel = 1;
	}
	if ( htim->Instance == TIM7)
	{
		SensorTimerCpt++;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
	  TxCpltCallback();
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ( huart->Instance == USART1)
  	{
	  UARTReceived = 1;
	  if(ReceiveUART[0] == 0x81)		//Sensor has respond whitout shift
	  {
		  if(ReceiveUART[1] == 0x11 || ReceiveUART[1] == 0x12 || ReceiveUART[1] == 0x13 )	//data receive
		  {
			 ParticuleDataReceive = 1;
			 PMObtain = 1;
			 DataParticule[0] = ReceiveUART[9];
			 DataParticule[1] = ReceiveUART[10];
			 DataParticule[2] = ReceiveUART[11];
			 DataParticule[3] = ReceiveUART[12];
			 DataParticule[4] = ReceiveUART[13];
			 DataParticule[5] = ReceiveUART[14];
		  }
		  else if(ReceiveUART[1] == 0x16)			//state receive
		  {
			  SensorStateReceive = 1;
			  DataErrorSensor = ReceiveUART[2];
		  }
		  else										//other cases
		  {
			  SensorErrorReceive = 1;
		  }
	  }
	  else if(ReceiveUART[1] == 0x81) //Sensor has respond whit a shift
	  {
		  if(ReceiveUART[2] == 0x11 || ReceiveUART[2] == 0x12 || ReceiveUART[2] == 0x13 )	//data receive
		  {
			 ParticuleDataReceive = 1;
			 PMObtain = 1;
			 DataParticule[0] = ReceiveUART[10];
			 DataParticule[1] = ReceiveUART[11];
			 DataParticule[2] = ReceiveUART[12];
			 DataParticule[3] = ReceiveUART[13];
			 DataParticule[4] = ReceiveUART[14];
			 DataParticule[5] = ReceiveUART[15];
		  }
		  else if(ReceiveUART[2] == 0x16)		//state receive
		  {
			  SensorStateReceive = 1;
			  DataErrorSensor = ReceiveUART[3];
		  }
		  else						//other cases
		  {
			  SensorErrorReceive = 1;
		  }
	  }
  	}
}




void LoraMacProcessNotify(void)
{
  LoraMacProcessRequest=LORA_SET;
}


static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Send( void* context )
{
  /* USER CODE BEGIN 3 */
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }
  
  TVL1(PRINTF("SEND REQUEST\n\r");)
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif
  
#ifdef USE_B_L072Z_LRWAN1
  TimerInit( &TxLedTimer, OnTimerLedEvent );
  
  TimerSetValue(  &TxLedTimer, 200);
  
  LED_On( LED_RED1 ) ; 
  
  TimerStart( &TxLedTimer );  
#endif

  uint32_t i = 0;
  AppData.Port = LPP_APP_PORT;

  if(SwitchWarning)			//if switch triggered
  {
	  SwitchWarning = 0;
	  AppData.Buff[i++] = SWITCH_ID;
	  AppData.Buff[i++] = SwitchValue;
  }
  else
  {
	if(BatteryLevelObtain && BatteryLevelAsked)
	{
	  BatteryLevelAsked = 0;
	  BatteryLevelObtain = 0;
	  BatteryValueLow = BatteryValue & 0xff;
	  BatteryValueHigh = (BatteryValue >> 8);
	  AppData.Buff[i++] = BATTERY_ID;
	  AppData.Buff[i++] = BatteryValueHigh;
	  AppData.Buff[i++] = BatteryValueLow;
	}
	if(TEMPERATUREObtain && TEMPERATUREAsked)
	{
	  TEMPERATUREAsked = 0;
	  TEMPERATUREObtain = 0;
	  TEMPERATUREValueLow = TEMPERATUREValue & 0xff;
	  TEMPERATUREValueHigh = (TEMPERATUREValue >> 8);
	  AppData.Buff[i++] = TEMPERATURE_ID;
	  AppData.Buff[i++] = TEMPERATUREValueHigh;
	  AppData.Buff[i++] = TEMPERATUREValueLow;
	}
	if(PMObtain && PMAsked)
	{
		PMAsked = 0;
		PMObtain = 0;

		AppData.Buff[i++] = PMSENSOR1_ID;
		AppData.Buff[i++] = DataParticule[0];
		AppData.Buff[i++] = DataParticule[1];

		AppData.Buff[i++] = PMSENSOR25_ID;
		AppData.Buff[i++] = DataParticule[2];
		AppData.Buff[i++] = DataParticule[3];

		AppData.Buff[i++] = PMSENSOR10_ID;
		AppData.Buff[i++] = DataParticule[4];
		AppData.Buff[i++] = DataParticule[5];
	}
	if(SwitchObtain && SwitchAsked)
	{
		SwitchObtain = 0;
		SwitchAsked = 0;

		AppData.Buff[i++] = SWITCH_ID;
		AppData.Buff[i++] = SwitchValue;
	}
	if(HUMIDITEObtain && HUMIDITEAsked)
	{
		HUMIDITEObtain = 0;
		HUMIDITEAsked = 0;
		HUMIDITEValueLow = HUMIDITEValue & 0xff;
		HUMIDITEValueHigh = (HUMIDITEValue >> 8);
		AppData.Buff[i++] = HUMIDITE_ID;
		AppData.Buff[i++] = HUMIDITEValueHigh;
		AppData.Buff[i++] = HUMIDITEValueLow;
	}
	TxProcessing = 0;
  }

  AppData.BuffSize = i;

  uint8_t cptTrySend = 0;
  bool send = false;
  while(!send && cptTrySend < 3)	//ir transmit error, try to resend 3 times
  {
	  send = LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	  cptTrySend++;
  }


  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppDa )
{
	AppDataRx = *AppDa;
	DecodeRequest();
}

static void OnTxTimerEvent( void* context )
{
	/*Wait for next tx slot*/
	TimerStart( &TxTimer);
	cpt++;
	cptSwitch++;
	if(SwitchChanged)
	{
		if(cptSwitch > 10)
		{
			SwitchChanged = 0;
			cptSwitch = 0;
			SwitchWarning = 1;
		}
	}
	if(cptSwitch >200)
	{	//in case of overflow
		cptSwitch = 10;
	}
	if(cpt >= 120 && TxProcessing == 0 )
	{
		cpt = 0;
		TxProcessing = 1;
		AppProcessRequest=LORA_SET;
	}

  
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime cpt reaches goal */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent( NULL );
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct={0};
  
    initStruct.Mode =GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void* context )
{
  LED_Off( LED_RED1 ) ; 
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
