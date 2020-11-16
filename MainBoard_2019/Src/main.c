
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include "string.h"

/* USER CODE BEGIN Includes */

#define GPS_ON 1
#define XBEE_ON 1
#define BEACON_ON 1
#define CAN_ON 1

/*Código GPS*/
#ifdef GPS_ON
#include "GPS_adafruit.h"
#endif


/* Para enviar como string, defina SENDNUMBER como 0. Para enviar um buffer de bytes, defina como 1*/
#define SENDNUMBER 0

/* Define endereï¿½os dos dados (measureID) de acordo com o protocolo FTCAN2.0*/
#define TPS_ADDR 0x0002
#define RPM_ADDR 0x0084
#define BAT_V_ADDR 0x0012
#define TPM_MOT_ADDR 0x0008
#define OILP_ADDR 0x000A
#define SPEEDFR_ADDR 0x001A
#define FUELP_ADDR 0x000C
#define MAP_ADDR 0x0004
#define DRSFRPM 0x0020
#define SPEEDRL_ADDR 0x001C //Velocidade traseira


/* Tamanho máximo dos buffers enviados pelo Xbee*/
#define BUF60_MAX 37
#define BUF30_MAX 113
#define BUF10_MAX 154
#define BUF_STRING_MAX 550
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

GPS_data hgps;     		//objeto GPS
char gpsData[100]; 		//dado recebido do GPS
char beaconData[10]; 	//dado recebido do Beacon

/*Variaveis globais dos dados*/
uint32_t TIMERCOUNT=0;

/*Flag do Beacon*/
uint8_t BEACON_FLAG = 0;

/*Variáveis do Pacote 1 -> 60Hz*/
uint16_t acelX_DE = 0, //Aceleração no eixo X da placa dianteira esquerda
		 acelY_DE = 0, //Aceleração no eixo Y da placa dianteira esquerda
		 acelZ_DE = 0, //Aceleração no eixo Z da placa dianteira esquerda
		 acelX_DD = 0, //Aceleração no eixo X da placa dianteira direita
		 acelY_DD = 0, //Aceleração no eixo Y da placa dianteira direita
		 acelZ_DD = 0, //Aceleração no eixo Z da placa dianteira direita
		 acelX_TE = 0, //Aceleração no eixo X da placa traseira esquerda
		 acelY_TE = 0, //Aceleração no eixo Y da placa traseira esquerda
		 acelZ_TE = 0, //Aceleração no eixo Z da placa traseira esquerda
		 acelX_TD = 0, //Aceleração no eixo X da placa traseira dianteira
		 acelY_TD = 0, //Aceleração no eixo Y da placa traseira dianteira
		 acelZ_TD = 0, //Aceleração no eixo Z da placa traseira dianteira
		 vel_DE = 0,   //Velocidade da roda dianteira esquerda
		 vel_DD = 0,   //Velocidade da roda dianteira direita
		 vel_TE = 0,   //Velocidade da roda traseira esquerda
		 vel_TD = 0,   //Velocidade da roda traseira direita
		 RPM = 0,
		 Beacon = 0;

/*Variáveis do Pacote 2 -> 30Hz*/
unsigned long ext1_DE = 0, //Extensômetro 1 da placa dianteira esquerda
			 ext2_DE = 0, //Extensômetro 2 da placa dianteira esquerda
			 ext3_DE = 0, //Extensômetro 3 da placa dianteira esquerda
			 ext1_DD = 0, //Extensômetro 1 da placa dianteira direita
			 ext2_DD = 0, //Extensômetro 2 da placa dianteira direita
			 ext3_DD = 0, //Extensômetro 3 da placa dianteira direita
			 ext1_TE = 0, //Extensômetro 1 da placa traseira esquerda
			 ext2_TE = 0, //Extensômetro 2 da placa traseira esquerda
			 ext3_TE = 0, //Extensômetro 3 da placa traseira esquerda
			 ext1_TD = 0, //Extensômetro 1 da placa traseira dianteira
			 ext2_TD = 0, //Extensômetro 2 da placa traseira dianteira
			 ext3_TD = 0; //Extensômetro 3 da placa traseira dianteira

/*Variáveis do Pacote 3 -> 30Hz*/
uint16_t TPS = 0, 			//TPS
		 OILP = 0, 			//Pressão de Óleo
		 FUELP = 0, 		//Pressão de Combustível
		 VAZBICOS = 0, 		//Vazão dos Bicos
		 PoSus_DE = 0, 		//Posição da suspensão da placa dianteira esquerda
		 PoSus_DD = 0, 		//Posição da suspensão da placa dianteira direita
		 PoSus_TE = 0, 		//Posição da suspensão da placa traseira esquerda
		 PoSus_TD = 0, 		//Posição da suspensão da placa traseira dianteira
		 PosVolante = 0,	//Posição do volante
		 BatCor = 0, 		//Corrente da Bateria
		 VentCor = 0, 		//Corrente da Ventoinha
		 BombCor = 0, 		//Corrente da Bomba
		 PresFreio_D = 0, 	//Pressão do Freio Dianteiro
		 PresFreio_T = 0; 	//Pressão do Freio Traseiro

/*Variáveis do Pacote 4 -> 10Hz*/
uint16_t BAT = 0,
		 ECT = 0,
		 OILT = 0,
		 Tempdisco_DE = 0,
		 Tempdisco_DD = 0,
		 Tempdisco_TE = 0,
		 Tempdisco_TD = 0,
		 TempVentoinha = 0,
		 TempBomba = 0,
		 PosRunners = 0,
		 AcioVent = 0,
		 AcioBomba = 0,
		 AcioMata = 0;
uint32_t GPS_Lat = 0, //GPS.lat*100;
		 GPS_Long = 0; //GPS.long*100;
char	 GPS_NS = 'N',
		 GPS_EW = 'E';

/* UART- Xbee - Pacotes enviados pelo Xbee */
uint8_t buffer_60[BUF60_MAX],
		buffer_30[BUF30_MAX],
		buffer_10[BUF10_MAX];

/*Buffer to Write*/
char bufferToWrite[BUF_STRING_MAX];
char x;
/* UART  */
char tx_buffer[100];
char tx_buffer2[80];
char tx_buffer3[80];
char tx_buffer4[100];
uint8_t pack3_cnt = 0, pack2_cnt = 0, PACKNO;
//uint8_t buff1[16], buff2[22], buff3[15] ,buff4[30];

/* Recepcao de pacotes CAN */
uint32_t canID =0, callbackCnt = 0;
uint8_t datafieldID;
uint16_t payloadLength, payloadCnt = 0;
uint8_t segPackNo = 0;
uint8_t payloadData[80];
uint8_t canData[8];

/* Acelerometro e Giroscópio*/
int16_t g_x, g_y, g_z, a_x, a_y, a_z;

/* ADC */
uint32_t adc_buffer[600], ADC[6]= {0,0,0,0,0,0};

/* Extensometros */
uint8_t dados_ext[8], RECEIVED_ETX = 0;
uint8_t EXT_ARRAY[8];
unsigned long EXT_DATA[8] = {0,0,0,0,0,0,0,0};
uint8_t CANTX = 0, CANRX = 0;

/* Sensor de temperatura PWM */
uint32_t PWMHighCounter = 0, PWMPeriodCounter = 0, dutyCycle = 0;
uint16_t TEMPBREAK_1=0, TEMPBREAK_2=0;

/*Variaveis para gravação de dados*/
char buffer[512];
static FATFS g_sFatFs;
FRESULT fresult;
FIL file;
int len;
UINT bytes_written;
uint8_t buffer1[37];
uint8_t buffer2[34];
uint8_t buffer3[42];
uint8_t buffer4[42];
long counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*Monta o pacote para gravar no cartão SD*/
void assemblePackageToWrite(uint8_t n);
/*Monta o pacote para enviar por telemetria*/
void assemblePackage(uint8_t n);

void assemblePackage1(uint8_t n);

/*Função para gravação no arquivo*/
void writeFile(char *buff, int leng);

#ifdef CAN_ON
/* Declara funcoes proprias do usuario */
static void CAN_Config(void);
void getMeasure(uint8_t address, uint16_t value);
void getMesureDAM(uint16_t idDAM, uint8_t*data);
#endif

#ifdef GPS_ON
void Transmit_GPS(char *send);
#endif

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)

{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	/*LED C13 pisca para sinalizar que o progrma comeï¿½ou*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(500);


#ifdef GPS_ON

	 // Inicialização do GPS
	 common_init(&hgps);
	 //Configuração do GPS
	 Transmit_GPS(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	 Transmit_GPS(PMTK_SET_NMEA_UPDATE_10HZ);
	 Transmit_GPS(PGCMD_ANTENNA);

	 if(HAL_UART_Receive_DMA(&huart2, (uint8_t*)gpsData, (unsigned)sizeof(gpsData))!= HAL_OK){
		 Error_Handler();
	 }

#endif

#ifdef CAN_ON

	/* Funcoes de inicialicao das mensagens CAN */
	CAN_Config();
	/* Chama funcao de recepcao de frames CAN  */
	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
#endif

	/* Sinaliza para timer comecar a contagem para transmissão do Xbee*/
//	HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(BEACON_FLAG){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_Delay(200);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		  BEACON_FLAG = 0;
	  }

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);

		if(counter%2 == 0 && counter !=6){//Envio na frequência de 30 Hz
//			assemblePackageToWrite(2);
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(1);
			HAL_UART_Transmit_DMA(&huart1, buffer1, sizeof(buffer1));
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(2);
			HAL_UART_Transmit_DMA(&huart1, buffer2, sizeof(buffer2));
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(4);
			HAL_UART_Transmit_DMA(&huart1, buffer4, sizeof(buffer4));
		}
		else if(counter==6){ //Envio na frequência de 10 Hz
//			assemblePackageToWrite(3);
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(1);
			HAL_UART_Transmit_DMA(&huart1, buffer1, sizeof(buffer1));
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(2);
			HAL_UART_Transmit_DMA(&huart1, buffer2, sizeof(buffer2));
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(4);
			HAL_UART_Transmit_DMA(&huart1, buffer4, sizeof(buffer4));
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			assemblePackage1(3);
			HAL_UART_Transmit_DMA(&huart1, buffer3, sizeof(buffer3));
			//Escreve no arquivo
//			writeFile(bufferToWrite, sizeof(bufferToWrite));
			counter = 0;
		}
		else{ //Envio na frequência de 60 Hz
			if(counter == 1){
				strcpy(bufferToWrite, "");
			}
//			assemblePackageToWrite(1);
			assemblePackage1(1);
			while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
			HAL_UART_Transmit_DMA(&huart1, buffer1, sizeof(buffer1));
		}
		counter++;
		TIMERCOUNT++;

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_6TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_SD_CARD_Pin|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_SD_CARD_Pin PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = CS_SD_CARD_Pin|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef GPS_ON
//Funções do GPS
void Transmit_GPS(char *send){
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)send, (unsigned)sizeof(send));
}
#endif


/*
 * Função que faz o tratamento dos dados que chegam tanto do GPS quanto do Beacon
 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){

	/*IF do GPS*/
//	if(huart->Instance == USART2){
//		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//
//		parse((char*)huart->pRxBuffPtr, &hgps);
//		//HAL_UART_Transmit_IT(&huart1, huart->pRxBuffPtr, (unsigned) huart->RxXferSize);
//		// Habilita leitura do GPS+
//		if(HAL_UART_Receive_DMA(&huart2, (uint8_t*)gpsData, (unsigned)sizeof(gpsData))!= HAL_OK){
//		  Error_Handler();
//		}
//	}
	/*IF do Beacon*/
//	else if(huart->Instance == USART3){
//		//Habilita leitura do Beacon
//		BEACON_FLAG = 1;
//		if(HAL_UART_Receive_DMA(&huart3, (uint8_t*)beaconData, (unsigned)sizeof(beaconData))!= HAL_OK){
//		  Error_Handler();
//		}
//	}
}


static void CAN_Config(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;
 static CanTxMsgTypeDef        TxMessage;     //--
 static CanRxMsgTypeDef        RxMessage;     //-- codigo original

  /*##-1- Configure the CAN peripheral #######################################*/
   hcan.pTxMsg = &TxMessage;     //-- codigo original
   hcan.pRxMsg = &RxMessage;	  // --



  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Configure Transmission process #####################################*/
  hcan.pTxMsg->StdId = 0x321;
  hcan.pTxMsg->ExtId = 0x300<<19;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  /*configurando como extended id (29 bits)*/
  hcan.pTxMsg->IDE = CAN_ID_EXT;
  hcan.pTxMsg->DLC = 2;
}

/* Recebe endereï¿½o do measureID do dado recebido pelo barramento e seu valor e atualiza a variavel global correspondente*/
void getMeasure(uint8_t address, uint16_t value){

	switch(address){
	case TPS_ADDR:
		TPS = value; //TPS = value*0.1;
		break;
	case RPM_ADDR:
		RPM = value;
		break;
	case  OILP_ADDR:
		OILP = value; //OILP = 0.001*value;
		break;
	case  FUELP_ADDR:
		FUELP = value;
		break;
	case SPEEDFR_ADDR:
		vel_DD = value;
		break;
	case BAT_V_ADDR:
		BAT = value; //BAT = 0.01*value;
		break;
	case TPM_MOT_ADDR:
		ECT = value; //ECT = 0.1*value;
		break;
	case DRSFRPM:
		RPM = value;
		break;
	case SPEEDRL_ADDR:
		vel_TD = value;
		break;
	default:
		break;
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	//For CAN Rx frames received in FIFO number 0
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0);
	HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{

	uint8_t i=0, dataLength, add=0;
	uint16_t value=0;

	//Armazena a quantidade de bytes do frame na variavel dataLength e a ID em canID
	dataLength = hcan->pRxMsg->DLC;
	canID = hcan->pRxMsg->ExtId;

	//Armazena dados recebidos em array canData
	for (i=0; i<dataLength; i++){
		canData[i] = hcan->pRxMsg->Data[i];
	}

	//Se o ProductID for relativo a FT550 (0x281)
	if ((((hcan->pRxMsg->ExtId) >> 19) == 0x281) && (hcan->pRxMsg->IDE == CAN_ID_EXT)){

		//LED 6A muda de estado toda vez que ï¿½ chamada a funcï¿½o callback, ou seja, toda vez que recebe um pacote CAN
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

		datafieldID = (canID>>11)& 0x7;

		if (datafieldID != 0x2){
		}

		//Testa se pacote e do tipo Single Packet
		if (canData[0] == 0xff){
		}

		//Testa se pacote e do tipo segmented packet
		//Testa se e o primeiro pacote do segmented packet (valor 0 no primeiro byte)
		if ( canData[0] == 0){

			// Zera variavel que conta o indice da payload
			payloadCnt = 0;

			// Seta segPackNo como 1, indicando que o proximo pacote a ser recebido tem valor do primeiro byte como 1
			segPackNo = 1;

			// Armazena tamanho da payload na variavel payloadLength. Tamanho da payload sao o segundo e terceiro bytes do primeiro pacote
			payloadLength = ((canData[1] & 0x7) << 8) | canData[2];

			// Nos 5 bytes restantes, armazena payload no array
			for (i=3;i<dataLength;i++){
				payloadData[payloadCnt] = canData[i];
				if(payloadCnt % 4 == 3){
					add = payloadData[payloadCnt-2];
					value = (payloadData[payloadCnt-1]<<8)|payloadData[payloadCnt]; // Junta os 2 bytes correspondentes ao valor do dado
					getMeasure(add, value);
				}
				// Chegou no final da payload
				if((payloadCnt == payloadLength-1)){
					break;
				}
				payloadCnt++;
			}
		}
		// Caso o pacote tenha o primeiro byte diferente de 0
		else{
			// Caso seja o pacote com o numero certo
			if(canData[0] == segPackNo){
				segPackNo++;
				// Armazena os dados recebidos na variavel payloadData, comecando do byte 1 e indo ate o ultimo byte recebido
				for (i=1;i<dataLength;i++){
					payloadData[payloadCnt] = canData[i];
					// Dados sao enviados de 4 em 4 bytes, sendo os 2 primeiro relativos ao endereco especifico de cada ume os 2 ultimos o dado. Faz divisao por 4 e pega o resto.
					if(payloadCnt % 4 == 3){
						add = payloadData[payloadCnt-2];
						value = (payloadData[payloadCnt-1]<<8)|payloadData[payloadCnt]; // Junta os 2 bytes correspondentes ao valor do dado
						getMeasure(add, value);
					}

					// Chegou no final da payload
					if((payloadCnt == payloadLength-1)){
						break;
					}

					payloadCnt++;
				}
			}
		}
	}
	else{
		getMesureDAM(hcan->pRxMsg->ExtId>>19, hcan->pRxMsg->Data);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	}

	callbackCnt++; //Contador de quantos callbacks foram chamados

	// Chama funcao para receber novo pacote
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK){
		Error_Handler();
	}
}

/*
 * Timer 3 -> clock vem de APB1 = 72Mhz
 * Prescaler de 36000 -> 72Mhz/12k = 6000Hz frequencia do timer. Prescaler divide o clock do timer
 * Counter Period = 99
 * Frequencia de interrupts = 6000/( 99 + 1 ) = 60Hz
 * Assim como o nome diz, HAL_TIM_PeriodElapsedCallback ï¿½ o callback da interrupï¿½ï¿½o gerada quando a contagem do timer
 * chega no limite, o couter period. Nesse caso Counter Period foi setado no Cube como 49, pois comeï¿½ando do 0, isso
 * corresponde a 50 periodos do timer. Dessa forma, com uma frequencia do timer configurada para 2000Hz, temos
 * uma interrupï¿½ï¿½o a cada 20ms
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);

	if(counter%2 == 0 && counter !=6){//Envio na frequência de 30 Hz
		assemblePackageToWrite(2);
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(1);
		HAL_UART_Transmit_DMA(&huart1, buffer1, sizeof(buffer1));
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(2);
		HAL_UART_Transmit_DMA(&huart1, buffer2, sizeof(buffer2));
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(4);
		HAL_UART_Transmit_DMA(&huart1, buffer4, sizeof(buffer4));
	}
	else if(counter==6){ //Envio na frequência de 10 Hz
		assemblePackageToWrite(3);
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(1);
		HAL_UART_Transmit_DMA(&huart1, buffer1, sizeof(buffer1));
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(2);
		HAL_UART_Transmit_DMA(&huart1, buffer2, sizeof(buffer2));
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(4);
		HAL_UART_Transmit_DMA(&huart1, buffer4, sizeof(buffer4));
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		assemblePackage1(3);
		HAL_UART_Transmit_DMA(&huart1, buffer3, sizeof(buffer3));
		//Escreve no arquivo
		writeFile(bufferToWrite, sizeof(bufferToWrite));
		counter = 0;
	}
	else{ //Envio na frequência de 60 Hz
		if(counter == 1){
			strcpy(bufferToWrite, "");
		}
		assemblePackageToWrite(1);
		assemblePackage1(1);
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY){}
		HAL_UART_Transmit_DMA(&huart1, buffer1, sizeof(buffer1));
	}
	counter++;
	TIMERCOUNT++;

}


void getMesureDAM(uint16_t idDAM, uint8_t*data){
	uint8_t idMsg = data[0];
	if(idDAM==0x0){ //Placa de aquisição dianteira direita
		switch(idMsg){
			case 1:
				acelX_DD = data[1]<<8 |data[2];
				acelY_DD = data[3]<<8 |data[4];
				ext1_DD = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 2:
				acelZ_DD = data[1]<<8 |data[2];
				PoSus_DD = data[3]<<8 |data[4];
				ext2_DD = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 3:
				PresFreio_D = data[1]<<8 |data[2];
				PresFreio_T = data[3]<<8 |data[4];
				ext3_DD = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 4:
				Tempdisco_DD = data[1]<<8 |data[2];
				break;
			default:
				break;
		}
	}
	else if(idDAM==0x302){ //Placa de aquisição dianteira esquerda
		switch(idMsg){
			case 1:
				acelX_DE = data[1]<<8 |data[2];
				acelY_DE = data[3]<<8 |data[4];
				ext1_DE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 2:
				acelZ_DE = data[1]<<8 |data[2];
				vel_DE = data[3]<<8 |data[4];
				ext2_DE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 3:
				PoSus_DE = data[1]<<8 |data[2];
				PosVolante = data[3]<<8 |data[4];
				ext3_DE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 4:
				Tempdisco_DE = data[1]<<8 |data[2];
				break;
			default:
				break;
		}
	}
	else if(idDAM==0x303){  //Placa de aquisição traseira direita
		switch(idMsg){
			case 1:
				acelX_TD = data[1]<<8 |data[2];
				acelY_TD = data[3]<<8 |data[4];
				ext1_TD = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 2:
				acelZ_TD = data[1]<<8 |data[2];
				PoSus_TD = data[3]<<8 |data[4];
				ext2_TD = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 3:
				BatCor = data[1]<<8 |data[2];
				Tempdisco_TD = data[3]<<8 |data[4];
				ext3_DE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			default:
				break;
		}
	}
	else if(idDAM==0x304){ //Placa de aquisição traseira esquerda
		switch(idMsg){
			case 1:
				acelX_TE = data[1]<<8 |data[2];
				acelY_TE = data[3]<<8 |data[4];
				ext1_TE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 2:
				acelZ_TE = data[1]<<8 |data[2];
				vel_TE = data[3]<<8 |data[4];
				ext2_TE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 3:
				PoSus_TE = data[1]<<8 |data[2];
				PosRunners = data[3]<<8 |data[4];
				ext3_DE = data[5]<<16 | data[6]<<8 | data[7];
				break;
			case 4:
				Tempdisco_TE = data[1]<<8 |data[2];
				break;
			default:
				break;
		}
	}
	else if(idDAM==0x100){ //PDU
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		VentCor = (data[0] & 127)<<8 | data[1];
		AcioVent = data[0]>>7;
		BombCor = (data[2] & 127)<<8 | data[3];
		AcioBomba = data[2]>>7;
		TempVentoinha = (data[4] & 127)<<8 | data[5];
		AcioMata = data[4]>>7;
		TempBomba = data[6]<<8 | data[7];
	}
}

void assemblePackageToWrite(uint8_t n){
	char aux[BUF_STRING_MAX];
	switch(n){
		case 1:
			len = sprintf(aux, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %ld\n", 1,
					 acelX_DD,
					 acelY_DD,
					 acelZ_DD,
					 acelX_DE,
					 acelY_DE,
					 acelZ_DE,
					 acelX_TD,
					 acelY_TD,
					 acelZ_TD,
					 acelX_TE,
					 acelY_TE,
					 acelZ_TE,
					 vel_DE,
					 vel_DD,
					 vel_TE,
					 vel_TD,
					 RPM,
					 Beacon,
					 TIMERCOUNT);
			strcat(bufferToWrite, aux);
			break;
		case 2:
			len = sprintf(aux, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
					 1,
					 acelX_DD,
					 acelY_DD,
					 acelZ_DD,
					 acelX_DE,
					 acelY_DE,
					 acelZ_DE,
					 acelX_TD,
					 acelY_TD,
					 acelZ_TD,
					 acelX_TE,
					 acelY_TE,
					 acelZ_TE,
					 vel_DE,
					 vel_DD,
					 vel_TE,
					 vel_TD,
					 RPM,
					 Beacon,
					 TIMERCOUNT,
					 4,
					 ext1_DD,
					 ext2_DD,
					 ext3_DD,
					 ext1_DE,
					 ext2_DE,
					 ext3_DE,
					 ext1_TD,
					 ext2_TD,
					 ext3_TD,
					 ext1_TE,
					 ext2_TE,
					 ext3_TE,
					 TIMERCOUNT,
					 3,
					 TPS,
					 OILP,
					 FUELP,
					 VAZBICOS,
					 PoSus_DD,
					 PoSus_DE,
					 PoSus_TD,
					 PoSus_TE,
					 PosVolante,
					 BatCor,
					 VentCor,
					 BombCor,
					 PresFreio_D,
					 PresFreio_T,
					 TIMERCOUNT
					 );
			strcat(bufferToWrite, aux);
			break;
		case 3:
			len = sprintf(aux, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d % %d %d %d %d %d %d %d %d %d %d %d %d\n",
			1,
			acelX_DD,
			acelY_DD,
			acelZ_DD,
			acelX_DE,
			acelY_DE,
			acelZ_DE,
			acelX_TD,
			acelY_TD,
			acelZ_TD,
			acelX_TE,
			acelY_TE,
			acelZ_TE,
			vel_DE,
			vel_DD,
			vel_TE,
			vel_TD,
			RPM,
			Beacon,
			TIMERCOUNT,
			4,
			ext1_DD,
			ext2_DD,
			ext3_DD,
			ext1_DE,
			ext2_DE,
			ext3_DE,
			ext1_TD,
			ext2_TD,
			ext3_TD,
			ext1_TE,
			ext2_TE,
			ext3_TE,
			TIMERCOUNT,
			3,
			TPS,
			OILP,
			FUELP,
			VAZBICOS,
			PoSus_DD,
			PoSus_DE,
			PoSus_TD,
			PoSus_TE,
			PosVolante,
			BatCor,
			VentCor,
			BombCor,
			PresFreio_D,
			PresFreio_T,
			TIMERCOUNT,
			2,
			BAT,
			ECT,
			OILT,
			Tempdisco_DD,
			Tempdisco_DE,
			Tempdisco_TD,
			Tempdisco_TE,
			TempVentoinha,
			TempBomba,
			PosRunners,
			AcioVent,
			AcioBomba,
			AcioMata,
			hgps.latitude,
			hgps.lat,
			hgps.longitude,
			hgps.lon,
			hgps.day,
			hgps.month,
			hgps.year,
			hgps.hour,
			hgps.minute,
			hgps.seconds,
			hgps.milliseconds,
			TIMERCOUNT
			);
			strcat(bufferToWrite, aux);
			break;
		default:
			break;
	}
}
void assemblePackage1(uint8_t n){

	switch(n){
	case 1:

			buffer1[0]= 1;
		    buffer1[1]= 5;
		    buffer1[2]= acelX_DD >> 8;
		    buffer1[3]= acelX_DD;
		    buffer1[4]= acelY_DD >> 8;
		    buffer1[5]= acelY_DD;
		    buffer1[6]= acelZ_DD >> 8;
		    buffer1[7]= acelZ_DD;
		    buffer1[8]= acelX_DE >> 8;
		    buffer1[9]= acelX_DE;
		    buffer1[10]= acelY_DE >> 8;
		    buffer1[11]= acelY_DE;
		    buffer1[12]= acelZ_DE >> 8;
		    buffer1[13]= acelZ_DE;
		    buffer1[14]= acelX_TD >> 8;
		    buffer1[15]= acelX_TD;
		    buffer1[16]= acelY_TD >> 8;
		    buffer1[17]= acelY_TD;
		    buffer1[18]= acelZ_TD >> 8;
		    buffer1[19]= acelZ_TD;
		    buffer1[20]= acelX_TE >> 8;
		    buffer1[21]= acelX_TE;
		    buffer1[22]= acelY_TE >> 8;
		    buffer1[23]= acelY_TE;
		    buffer1[24]= acelZ_TE >> 8;
		    buffer1[25]= acelZ_TE;
		    buffer1[26]= vel_DD;
		    buffer1[27]= vel_DE;
		    buffer1[28]= vel_TD;
		    buffer1[29]= vel_TE;
		    buffer1[30]= RPM>>8;
		    buffer1[31]= RPM;
		    buffer1[32]= Beacon;
		    buffer1[33] = TIMERCOUNT>>8;
		    buffer1[34] = TIMERCOUNT;
		    buffer1[35] = 9;
		    buffer1[36] = '\n';
		    break;
	case 2:
		    buffer2[0] = 2;
		    buffer2[1] = 5;
		    buffer2[2] = TPS>>8; // de 0 a 1000, 10bits
		    buffer2[3] = TPS;
		    buffer2[4] = OILP>>8;
		    buffer2[5] = OILP;
		    buffer2[6] = FUELP>>8;
		    buffer2[7] = FUELP;
		    buffer2[8] = VAZBICOS>>8;
		    buffer2[9] = VAZBICOS;
		    buffer2[10] = PoSus_DE>>8;
		    buffer2[11] = PoSus_DE;
		    buffer2[12] = PoSus_DD>>8;
		    buffer2[13] = PoSus_DD;
		    buffer2[14] = PoSus_TE>>8;
		    buffer2[15] = PoSus_TE;
		    buffer2[16] = PoSus_TD>>8;
		    buffer2[17] = PoSus_TD;
		    buffer2[18] = PosVolante>>8;
		    buffer2[19] = PosVolante;
		    buffer2[20] = BatCor>>8;
		    buffer2[21] = BatCor;
		    buffer2[22] = VentCor>>8;
		    buffer2[23] = VentCor;
		    buffer2[24] = BombCor>>8;
		    buffer2[25] = BombCor;
		    buffer2[26]= PresFreio_D>>8;
		    buffer2[27] = PresFreio_D;
		    buffer2[28]= PresFreio_T>>8;
		    buffer2[29] = PresFreio_T;
		    buffer2[30] = TIMERCOUNT>>8;
		    buffer2[31] = TIMERCOUNT;
		    buffer2[32] = 9;
		    buffer2[33] = '\n';
		    break;
	case 3:
		    buffer3[0] = 3;
		    buffer3[1] = 5;
		    buffer3[2] = BAT>>8; //max1500
		    buffer3[3] = BAT;
		    buffer3[4] = ECT>>8;
		    buffer3[5] = ECT;
		    buffer3[6] = OILT>>8;
		    buffer3[7] = OILT;
		    buffer3[8] = Tempdisco_DE>>8;
		    buffer3[9] = Tempdisco_DE;
		    buffer3[10] = Tempdisco_DD>>8;
		    buffer3[11] = Tempdisco_DD;
		    buffer3[12] = Tempdisco_TE>>8;
		    buffer3[13] = Tempdisco_TE;
		    buffer3[14] = Tempdisco_TD>>8;
		    buffer3[15] = Tempdisco_TD;
		    buffer3[16] = TempVentoinha>>8;
		    buffer3[17] = TempVentoinha;
		    buffer3[18] = TempBomba>>8;
		    buffer3[19] = TempBomba;
		    buffer3[20] = PosRunners>>8;
		    buffer3[21] = PosRunners;
		    buffer3[22] = (AcioVent<<3) | (AcioBomba<<7) | (AcioMata<<5);
		    buffer3[23] = GPS_Lat>>16;
		    buffer3[24] = GPS_Lat>>8;
		    buffer3[25] = GPS_Lat;
		    buffer3[26] = GPS_Long>>16;
		    buffer3[27] = GPS_Long>>8;
		    buffer3[28] = GPS_Long;
		    buffer3[29] = GPS_NS;
		    buffer3[30] = GPS_EW;
		    buffer3[31] = hgps.hour;
		    buffer3[32] = hgps.minute;
		    buffer3[33] = hgps.seconds;
		    buffer3[34] = hgps.milliseconds;
		    buffer3[35] = hgps.year;
		    buffer3[36] = hgps.month;
		    buffer3[37] = hgps.day;
		    buffer3[38] = TIMERCOUNT>>8;
		    buffer3[39] = TIMERCOUNT;
		    buffer3[40] = 9;
		    buffer3[41] = '\n';
		    break;
	case 4:
		    buffer4[0] = 4;
		    buffer4[1] = 5;
		    buffer4[2] = ext1_DE >> 16;
		    buffer4[3] = ext1_DE >> 8;
		    buffer4[4] = ext1_DE;
		    buffer4[5] = ext2_DE >> 16;
		    buffer4[6] = ext2_DE >> 8;
		    buffer4[7] = ext2_DE;
		    buffer4[8] = ext3_DE >> 16;
		    buffer4[9] = ext3_DE >> 8;
		    buffer4[10] = ext3_DE;
		    buffer4[11] = ext1_DD >> 16;
		    buffer4[12] = ext1_DD >> 8;
		    buffer4[13] = ext1_DD;
		    buffer4[14] = ext2_DD >> 16;
		    buffer4[15] = ext2_DD >> 8;
		    buffer4[16] = ext2_DD;
		    buffer4[17] = ext3_DD >> 16;
		    buffer4[18] = ext3_DD >> 8;
		    buffer4[19] = ext3_DD;
		    buffer4[20] = ext1_TE >> 16;
		    buffer4[21] = ext1_TE >> 8;
		    buffer4[22] = ext1_TE;
		    buffer4[23] = ext2_TE >> 16;
		    buffer4[24] = ext2_TE >> 8;
		    buffer4[25] = ext2_TE;
		    buffer4[26] = ext3_TE >> 16;
		    buffer4[27] = ext3_TE >> 8;
		    buffer4[28] = ext3_TE;
		    buffer4[29] = ext1_TD >> 16;
		    buffer4[30] = ext1_TD >> 8;
		    buffer4[31] = ext1_TD;
		    buffer4[32] = ext2_TD >> 16;
		    buffer4[33] = ext2_TD >> 8;
		    buffer4[34] = ext2_TD;
		    buffer4[35] = ext3_TD >> 16;
		    buffer4[36] = ext3_TD >> 8;
		    buffer4[37] = ext3_TD;
		    buffer4[38] = TIMERCOUNT>>8;
		    buffer4[39] = TIMERCOUNT;
		    buffer4[40] = 9;
		    buffer4[41] = 10;
		    break;
	}
}

void assemblePackage(uint8_t n){

	switch(n){

	case 1:
		buffer_60[0]= 1;
		buffer_60[1]= 5;
		buffer_60[2]= acelX_DD >> 8;
		buffer_60[3]= acelX_DD;
		buffer_60[4]= acelY_DD >> 8;
		buffer_60[5]= acelY_DD;
		buffer_60[6]= acelZ_DD >> 8;
		buffer_60[7]= acelZ_DD;
		buffer_60[8]= acelX_DE >> 8;
		buffer_60[9]= acelX_DE;
		buffer_60[10]= acelY_DE >> 8;
		buffer_60[11]= acelY_DE;
		buffer_60[12]= acelZ_DE >> 8;
		buffer_60[13]= acelZ_DE;
		buffer_60[14]= acelX_TD >> 8;
		buffer_60[15]= acelX_TD;
		buffer_60[16]= acelY_TD >> 8;
		buffer_60[17]= acelY_TD;
		buffer_60[18]= acelZ_TD >> 8;
		buffer_60[19]= acelZ_TD;
		buffer_60[20]= acelX_TE >> 8;
		buffer_60[21]= acelX_TE;
		buffer_60[22]= acelY_TE >> 8;
		buffer_60[23]= acelY_TE;
		buffer_60[24]= acelZ_TE >> 8;
		buffer_60[25]= acelZ_TE;
		buffer_60[26]= vel_DD;
		buffer_60[27]= vel_DE;
		buffer_60[28]= vel_TD;
		buffer_60[29]= vel_TE;
		buffer_60[30]= RPM>>8;
		buffer_60[31]= RPM;
		buffer_60[32]= Beacon;
		buffer_60[33]= TIMERCOUNT>>8;
		buffer_60[34] = TIMERCOUNT;
		buffer_60[35] = 9;
		buffer_60[36] = '\n';
		break;
	case 2:
		buffer_30[0]= 1;
		buffer_30[1]= 5;
		buffer_30[2]= acelX_DD >> 8;
		buffer_30[3]= acelX_DD;
		buffer_30[4]= acelY_DD >> 8;
		buffer_30[5]= acelY_DD;
		buffer_30[6]= acelZ_DD >> 8;
		buffer_30[7]= acelZ_DD;
		buffer_30[8]= acelX_DE >> 8;
		buffer_30[9]= acelX_DE;
		buffer_30[10]= acelY_DE >> 8;
		buffer_30[11]= acelY_DE;
		buffer_30[12]= acelZ_DE >> 8;
		buffer_30[13]= acelZ_DE;
		buffer_30[14]= acelX_TD >> 8;
		buffer_30[15]= acelX_TD;
		buffer_30[16]= acelY_TD >> 8;
		buffer_30[17]= acelY_TD;
		buffer_30[18]= acelZ_TD >> 8;
		buffer_30[19]= acelZ_TD;
		buffer_30[20]= acelX_TE >> 8;
		buffer_30[21]= acelX_TE;
		buffer_30[22]= acelY_TE >> 8;
		buffer_30[23]= acelY_TE;
		buffer_30[24]= acelZ_TE >> 8;
		buffer_30[25]= acelZ_TE;
		buffer_30[26]= vel_DD;
		buffer_30[27]= vel_DE;
		buffer_30[28]= vel_TD;
		buffer_30[29]= vel_TE;
		buffer_30[30]= RPM>>8;
		buffer_30[31]= RPM;
		buffer_30[32]= Beacon;
		buffer_30[33] = TIMERCOUNT>>8;
		buffer_30[34] = TIMERCOUNT;
		buffer_30[35] = 9;
		buffer_30[36] = '\n';

		buffer_30[37] = 2;
		buffer_30[38] = 5;
		buffer_30[39] = TPS>>8; // de 0 a 1000, 10bits
		buffer_30[40] = TPS;
		buffer_30[41] = OILP>>8;
		buffer_30[42] = OILP;
		buffer_30[43] = FUELP>>8;
		buffer_30[44] = FUELP;
		buffer_30[45] = VAZBICOS>>8;
		buffer_30[46] = VAZBICOS;
		buffer_30[47] = PoSus_DE>>8;
		buffer_30[48] = PoSus_DE;
		buffer_30[49] = PoSus_DD>>8;
		buffer_30[50] = PoSus_DD;
		buffer_30[51] = PoSus_TE>>8;
		buffer_30[52] = PoSus_TE;
		buffer_30[53] = PoSus_TD>>8;
		buffer_30[54] = PoSus_TD;
		buffer_30[55] = PosVolante>>8;
		buffer_30[56] = PosVolante;
		buffer_30[57] = BatCor>>8;
		buffer_30[58] = BatCor;
		buffer_30[59] = VentCor>>8;
		buffer_30[60] = VentCor;
		buffer_30[61] = BombCor>>8;
		buffer_30[62] = BombCor;
		buffer_30[63]= PresFreio_D>>8;
		buffer_30[64] = PresFreio_D;
		buffer_30[65]= PresFreio_T>>8;
		buffer_30[66] = PresFreio_T;
		buffer_30[67] = TIMERCOUNT>>8;
		buffer_30[68] = TIMERCOUNT;
		buffer_30[69] = 9;
		buffer_30[70] = '\n';

		buffer_30[71] = 4;
		buffer_30[72] = 5;
		buffer_30[73] = ext1_DE >> 16;
		buffer_30[74] = ext1_DE >> 8;
		buffer_30[75] = ext1_DE;
		buffer_30[76] = ext2_DE >> 16;
		buffer_30[77] = ext2_DE >> 8;
		buffer_30[78] = ext2_DE;
		buffer_30[79] = ext3_DE >> 16;
		buffer_30[80] = ext3_DE >> 8;
		buffer_30[81] = ext3_DE;
		buffer_30[82] = ext1_DD >> 16;
		buffer_30[83] = ext1_DD >> 8;
		buffer_30[84] = ext1_DD;
		buffer_30[85] = ext2_DD >> 16;
		buffer_30[86] = ext2_DD >> 8;
		buffer_30[87] = ext2_DD;
		buffer_30[88] = ext3_DD >> 16;
		buffer_30[89] = ext3_DD >> 8;
		buffer_30[90] = ext3_DD;
		buffer_30[91] = ext1_TE >> 16;
		buffer_30[92] = ext1_TE >> 8;
		buffer_30[93] = ext1_TE;
		buffer_30[94] = ext2_TE >> 16;
		buffer_30[95] = ext2_TE >> 8;
		buffer_30[96] = ext2_TE;
		buffer_30[97] = ext3_TE >> 16;
		buffer_30[98] = ext3_TE >> 8;
		buffer_30[99] = ext3_TE;
		buffer_30[100] = ext1_TD >> 16;
		buffer_30[101] = ext1_TD >> 8;
		buffer_30[102] = ext1_TD;
		buffer_30[103] = ext2_TD >> 16;
		buffer_30[104] = ext2_TD >> 8;
		buffer_30[105] = ext2_TD;
		buffer_30[106] = ext3_TD >> 16;
		buffer_30[107] = ext3_TD >> 8;
		buffer_30[108] = ext3_TD;
		buffer_30[109] = TIMERCOUNT>>8;
		buffer_30[110] = TIMERCOUNT;
		buffer_30[111] = 9;
		buffer_30[112] = '\n';

//		buffer_30[37] = 4;
//		buffer_30[38] = 5;
//		buffer_30[39] = ext1_DE >> 16;
//		buffer_30[40] = ext1_DE >> 8;
//		buffer_30[41] = ext1_DE;
//		buffer_30[42] = ext2_DE >> 16;
//		buffer_30[43] = ext2_DE >> 8;
//		buffer_30[44] = ext2_DE;
//		buffer_30[45] = ext3_DE >> 16;
//		buffer_30[46] = ext3_DE >> 8;
//		buffer_30[47] = ext3_DE;
//		buffer_30[48] = ext1_DD >> 16;
//		buffer_30[49] = ext1_DD >> 8;
//		buffer_30[50] = ext1_DD;
//		buffer_30[51] = ext2_DD >> 16;
//		buffer_30[52] = ext2_DD >> 8;
//		buffer_30[53] = ext2_DD;
//		buffer_30[54] = ext3_DD >> 16;
//		buffer_30[55] = ext3_DD >> 8;
//		buffer_30[56] = ext3_DD;
//		buffer_30[57] = ext1_TE >> 16;
//		buffer_30[58] = ext1_TE >> 8;
//		buffer_30[59] = ext1_TE;
//		buffer_30[60] = ext2_TE >> 16;
//		buffer_30[61] = ext2_TE >> 8;
//		buffer_30[62] = ext2_TE;
//		buffer_30[63] = ext3_TE >> 16;
//		buffer_30[64] = ext3_TE >> 8;
//		buffer_30[65] = ext3_TE;
//		buffer_30[66] = ext1_TD >> 16;
//		buffer_30[67] = ext1_TD >> 8;
//		buffer_30[68] = ext1_TD;
//		buffer_30[69] = ext2_TD >> 16;
//		buffer_30[70] = ext2_TD >> 8;
//		buffer_30[71] = ext2_TD;
//		buffer_30[72] = ext3_TD >> 16;
//		buffer_30[73] = ext3_TD >> 8;
//		buffer_30[74] = ext3_TD;
//		buffer_30[75] = TIMERCOUNT>>8;
//		buffer_30[76] = TIMERCOUNT;
//		buffer_30[77] = 9;
//		buffer_30[78] = '\n';
//
//		buffer_30[79] = 2;
//		buffer_30[80] = 5;
//		buffer_30[81] = TPS>>8; // de 0 a 1000, 10bits
//		buffer_30[82] = TPS;
//		buffer_30[83] = OILP>>8;
//		buffer_30[84] = OILP;
//		buffer_30[85] = FUELP>>8;
//		buffer_30[86] = FUELP;
//		buffer_30[87] = VAZBICOS>>8;
//		buffer_30[88] = VAZBICOS;
//		buffer_30[89] = PoSus_DE>>8;
//		buffer_30[90] = PoSus_DE;
//		buffer_30[91] = PoSus_DD>>8;
//		buffer_30[92] = PoSus_DD;
//		buffer_30[93] = PoSus_TE>>8;
//		buffer_30[94] = PoSus_TE;
//		buffer_30[95] = PoSus_TD>>8;
//		buffer_30[96] = PoSus_TD;
//		buffer_30[97] = PosVolante>>8;
//		buffer_30[98] = PosVolante;
//		buffer_30[99] = BatCor>>8;
//		buffer_30[100] = BatCor;
//		buffer_30[101] = VentCor>>8;
//		buffer_30[102] = VentCor;
//		buffer_30[103] = BombCor>>8;
//		buffer_30[104] = BombCor;
//		buffer_30[105]= PresFreio_D>>8;
//		buffer_30[106] = PresFreio_D;
//		buffer_30[107]= PresFreio_T>>8;
//		buffer_30[108] = PresFreio_T;
//		buffer_30[109] = TIMERCOUNT>>8;
//		buffer_30[110] = TIMERCOUNT;
//		buffer_30[111] = 9;
//		buffer_30[112] = '\n';
		break;
	case 3:
		buffer_10[0]= 1;
		buffer_10[1]= 5;
		buffer_10[2]= acelX_DD >> 8;
		buffer_10[3]= acelX_DD;
		buffer_10[4]= acelY_DD >> 8;
		buffer_10[5]= acelY_DD;
		buffer_10[6]= acelZ_DD >> 8;
		buffer_10[7]= acelZ_DD;
		buffer_10[8]= acelX_DE >> 8;
		buffer_10[9]= acelX_DE;
		buffer_10[10]= acelY_DE >> 8;
		buffer_10[11]= acelY_DE;
		buffer_10[12]= acelZ_DE >> 8;
		buffer_10[13]= acelZ_DE;
		buffer_10[14]= acelX_TD >> 8;
		buffer_10[15]= acelX_TD;
		buffer_10[16]= acelY_TD >> 8;
		buffer_10[17]= acelY_TD;
		buffer_10[18]= acelZ_TD >> 8;
		buffer_10[19]= acelZ_TD;
		buffer_10[20]= acelX_TE >> 8;
		buffer_10[21]= acelX_TE;
		buffer_10[22]= acelY_TE >> 8;
		buffer_10[23]= acelY_TE;
		buffer_10[24]= acelZ_TE >> 8;
		buffer_10[25]= acelZ_TE;
		buffer_10[26]= vel_DD;
		buffer_10[27]= vel_DE;
		buffer_10[28]= vel_TD;
		buffer_10[29]= vel_TE;
		buffer_10[30]= RPM>>8;
		buffer_10[31]= RPM;
		buffer_10[32]= Beacon;
		buffer_10[33] = TIMERCOUNT>>8;
		buffer_10[34] = TIMERCOUNT;
		buffer_10[35] = 9;
		buffer_10[36] = '\n';

//		buffer_10[37] = 4;
//		buffer_10[38] = 5;
//		buffer_10[39] = ext1_DE >> 16;
//		buffer_10[40] = ext1_DE >> 8;
//		buffer_10[41] = ext1_DE;
//		buffer_10[42] = ext2_DE >> 16;
//		buffer_10[43] = ext2_DE >> 8;
//		buffer_10[44] = ext2_DE;
//		buffer_10[45] = ext3_DE >> 16;
//		buffer_10[46] = ext3_DE >> 8;
//		buffer_10[47] = ext3_DE;
//		buffer_10[48] = ext1_DD >> 16;
//		buffer_10[49] = ext1_DD >> 8;
//		buffer_10[50] = ext1_DD;
//		buffer_10[51] = ext2_DD >> 16;
//		buffer_10[52] = ext2_DD >> 8;
//		buffer_10[53] = ext2_DD;
//		buffer_10[54] = ext3_DD >> 16;
//		buffer_10[55] = ext3_DD >> 8;
//		buffer_10[56] = ext3_DD;
//		buffer_10[57] = ext1_TE >> 16;
//		buffer_10[58] = ext1_TE >> 8;
//		buffer_10[59] = ext1_TE;
//		buffer_10[60] = ext2_TE >> 16;
//		buffer_10[61] = ext2_TE >> 8;
//		buffer_10[62] = ext2_TE;
//		buffer_10[63] = ext3_TE >> 16;
//		buffer_10[64] = ext3_TE >> 8;
//		buffer_10[65] = ext3_TE;
//		buffer_10[66] = ext1_TD >> 16;
//		buffer_10[67] = ext1_TD >> 8;
//		buffer_10[68] = ext1_TD;
//		buffer_10[69] = ext2_TD >> 16;
//		buffer_10[70] = ext2_TD >> 8;
//		buffer_10[71] = ext2_TD;
//		buffer_10[72] = ext3_TD >> 16;
//		buffer_10[73] = ext3_TD >> 8;
//		buffer_10[74] = ext3_TD;
//		buffer_10[75] = TIMERCOUNT>>8;
//		buffer_10[76] = TIMERCOUNT;
//		buffer_10[77] = 9;
//		buffer_10[78] = '\n';
//
//		buffer_10[79] = 2;
//		buffer_10[80] = 5;
//		buffer_10[81] = TPS>>8; // de 0 a 1000, 10bits
//		buffer_10[82] = TPS;
//		buffer_10[83] = OILP>>8;
//		buffer_10[84] = OILP;
//		buffer_10[85] = FUELP>>8;
//		buffer_10[86] = FUELP;
//		buffer_10[87] = VAZBICOS>>8;
//		buffer_10[88] = VAZBICOS;
//		buffer_10[89] = PoSus_DE>>8;
//		buffer_10[90] = PoSus_DE;
//		buffer_10[91] = PoSus_DD>>8;
//		buffer_10[92] = PoSus_DD;
//		buffer_10[93] = PoSus_TE>>8;
//		buffer_10[94] = PoSus_TE;
//		buffer_10[95] = PoSus_TD>>8;
//		buffer_10[96] = PoSus_TD;
//		buffer_10[97] = PosVolante>>8;
//		buffer_10[98] = PosVolante;
//		buffer_10[99] = BatCor>>8;
//		buffer_10[100] = BatCor;
//		buffer_10[101] = VentCor>>8;
//		buffer_10[102] = VentCor;
//		buffer_10[103] = BombCor>>8;
//		buffer_10[104] = BombCor;
//		buffer_10[105]= PresFreio_D>>8;
//		buffer_10[106] = PresFreio_D;
//		buffer_10[107]= PresFreio_T>>8;
//		buffer_10[108] = PresFreio_T;
//		buffer_10[109] = TIMERCOUNT>>8;
//		buffer_10[110] = TIMERCOUNT;
//		buffer_10[111] = 9;
//		buffer_10[112] = '\n';

		buffer_10[37] = 2;
		buffer_10[38] = 5;
		buffer_10[39] = TPS>>8; // de 0 a 1000, 10bits
		buffer_10[40] = TPS;
		buffer_10[41] = OILP>>8;
		buffer_10[42] = OILP;
		buffer_10[43] = FUELP>>8;
		buffer_10[44] = FUELP;
		buffer_10[45] = VAZBICOS>>8;
		buffer_10[46] = VAZBICOS;
		buffer_10[47] = PoSus_DE>>8;
		buffer_10[48] = PoSus_DE;
		buffer_10[49] = PoSus_DD>>8;
		buffer_10[50] = PoSus_DD;
		buffer_10[51] = PoSus_TE>>8;
		buffer_10[52] = PoSus_TE;
		buffer_10[53] = PoSus_TD>>8;
		buffer_10[54] = PoSus_TD;
		buffer_10[55] = PosVolante>>8;
		buffer_10[56] = PosVolante;
		buffer_10[57] = BatCor>>8;
		buffer_10[58] = BatCor;
		buffer_10[59] = VentCor>>8;
		buffer_10[60] = VentCor;
		buffer_10[61] = BombCor>>8;
		buffer_10[62] = BombCor;
		buffer_10[63]= PresFreio_D>>8;
		buffer_10[64] = PresFreio_D;
		buffer_10[65]= PresFreio_T>>8;
		buffer_10[66] = PresFreio_T;
		buffer_10[67] = TIMERCOUNT>>8;
		buffer_10[68] = TIMERCOUNT;
		buffer_10[69] = 9;
		buffer_10[70] = '\n';

		buffer_10[71] = 4;
		buffer_10[72] = 5;
		buffer_10[73] = ext1_DE >> 16;
		buffer_10[74] = ext1_DE >> 8;
		buffer_10[75] = ext1_DE;
		buffer_10[76] = ext2_DE >> 16;
		buffer_10[77] = ext2_DE >> 8;
		buffer_10[78] = ext2_DE;
		buffer_10[79] = ext3_DE >> 16;
		buffer_10[80] = ext3_DE >> 8;
		buffer_10[81] = ext3_DE;
		buffer_10[82] = ext1_DD >> 16;
		buffer_10[83] = ext1_DD >> 8;
		buffer_10[84] = ext1_DD;
		buffer_10[85] = ext2_DD >> 16;
		buffer_10[86] = ext2_DD >> 8;
		buffer_10[87] = ext2_DD;
		buffer_10[88] = ext3_DD >> 16;
		buffer_10[89] = ext3_DD >> 8;
		buffer_10[90] = ext3_DD;
		buffer_10[91] = ext1_TE >> 16;
		buffer_10[92] = ext1_TE >> 8;
		buffer_10[93] = ext1_TE;
		buffer_10[94] = ext2_TE >> 16;
		buffer_10[95] = ext2_TE >> 8;
		buffer_10[96] = ext2_TE;
		buffer_10[97] = ext3_TE >> 16;
		buffer_10[98] = ext3_TE >> 8;
		buffer_10[99] = ext3_TE;
		buffer_10[100] = ext1_TD >> 16;
		buffer_10[101] = ext1_TD >> 8;
		buffer_10[102] = ext1_TD;
		buffer_10[103] = ext2_TD >> 16;
		buffer_10[104] = ext2_TD >> 8;
		buffer_10[105] = ext2_TD;
		buffer_10[106] = ext3_TD >> 16;
		buffer_10[107] = ext3_TD >> 8;
		buffer_10[108] = ext3_TD;
		buffer_10[109] = TIMERCOUNT>>8;
		buffer_10[110] = TIMERCOUNT;
		buffer_10[111] = 9;
		buffer_10[112] = '\n';

		buffer_10[113] = 3;
		buffer_10[114] = 5;
		buffer_10[115] = BAT>>8; //max1500
		buffer_10[116] = BAT;
		buffer_10[117] = ECT>>8;
		buffer_10[118] = ECT;
		buffer_10[119] = OILT>>8;
		buffer_10[120] = OILT;
		buffer_10[121] = Tempdisco_DE>>8;
		buffer_10[122] = Tempdisco_DE;
		buffer_10[123] = Tempdisco_DD>>8;
		buffer_10[124] = Tempdisco_DD;
		buffer_10[125] = Tempdisco_TE>>8;
		buffer_10[126] = Tempdisco_TE;
		buffer_10[127] = Tempdisco_TD>>8;
		buffer_10[128] = Tempdisco_TD;
		buffer_10[129] = TempVentoinha>>8;
		buffer_10[130] = TempVentoinha;
		buffer_10[131] = TempBomba>>8;
		buffer_10[132] = TempBomba;
		buffer_10[133] = PosRunners>>8;
		buffer_10[134] = PosRunners;
		buffer_10[135] = (AcioVent<<3) | (AcioBomba<<7) | (AcioMata<<5);
		buffer_10[136] = GPS_Lat>>16;
		buffer_10[137] = GPS_Lat>>8;
		buffer_10[138] = GPS_Lat;
		buffer_10[139] = GPS_Long>>16;
		buffer_10[140] = GPS_Long>>8;
		buffer_10[141] = GPS_Long;
		buffer_10[142] = GPS_NS;
		buffer_10[143] = GPS_EW;
		buffer_10[144] = hgps.hour;
		buffer_10[145] = hgps.minute;
		buffer_10[146] = hgps.seconds;
		buffer_10[147] = hgps.milliseconds;
		buffer_10[148] = hgps.year;
		buffer_10[149] = hgps.month;
		buffer_10[150] = hgps.day;
		buffer_10[151] = TIMERCOUNT>>8;
		buffer_10[152] = TIMERCOUNT;
		buffer_10[153] = 9;
		buffer_10[154] = '\n';
		break;

	default:
			break;

	}
}


/*
 *  Função para o arquivo.
 */
void writeFile(char *buff, int leng){
	/* Writing a File*/
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	// Mount SD card
	if(f_mount(&g_sFatFs, "0:", 0) != FR_OK){
		// Mount Error
		//Error_Handler();
	}

	/* Open file on SD card */
	if(f_open(&file, "CANTest.txt", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK){
		/* Open File Error */
		//Error_Handler();
	}

	/*Go to the end of the file*/
	if(f_lseek(&file, file.fsize) != FR_OK){
		/* Open File Error */
		//Error_Handler();
	}

	/*Write data to the file*/
	if(f_write(&file, buff, leng, &bytes_written)){
		/* Write data Error */
		//Error_Handler();
	}

	/* Close file on SD card */
	if(f_close(&file) != FR_OK){
		/* Close file Error */
		//Error_Handler();
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  break;
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**   \
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
