/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
enum trx_t
{
	CHIP_RX,
	CHIP_TX
};

enum tp_t
{
	TP1,
	TP2
};

enum transf_t
{
	TX_SINGLE,
	TX_BURST,
	RX_SINGLE,
	RX_BURST
};

enum strobe_t
{
	STR_SRES = 0x30,		//reset
	STR_SCAL = 0x33,		//calibrate
	STR_SRX,				//set state to "receive"
	STR_STX,				//set state to "transmit"
	STR_IDLE,				//set state to "idle"
	STR_SNOP = 0x3D			//no operation
};

struct trx_data_t
{
	uint8_t name[8];		//chip's name (CC1200, CC1201, unknown)
	uint32_t frequency;		//frequency in hertz
	uint8_t pwr;			//power setting (3..63)
	int16_t fcorr;			//frequency correction
	uint8_t pll_locked;		//PLL locked flag
}trx_data[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dbg_print(const char* fmt, ...)
{
	char str[64];
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
}

//set rf power output setpoint
//0 - lowest power, 0xFFF - max power
void set_rf_pwr_setpoint(uint16_t pwr)
{
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pwr>0xFFF?0xFFF:pwr);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

float get_ref_pwr(void)
{
	//TODO

	return 0.0f;
}

float get_fwd_pwr(void)
{
	//TODO

	return 0.0f;
}

//calculate SWR based on fwd and ref power
//returns 0.0 when input data is invalid
float calc_swr(float fwd, float ref)
{
	float s=sqrtf(ref/fwd);
	float swr=(1.0f+s)/(1.0f-s);

	return swr>=1.0f?swr:0.0f;
}

//enable RF PA (assert one of two enable signals)
void rf_pa_en(uint8_t en)
{
	if(en)
		PA_EN_GPIO_Port->BSRR=(uint32_t)PA_EN_Pin;
	else
		PA_EN_GPIO_Port->BSRR=(uint32_t)PA_EN_Pin<<16;
}

void set_TP(enum tp_t tp, uint8_t state)
{
	if(tp==TP1)
	{
		if(state)
			DBG_TP1_GPIO_Port->BSRR=(uint32_t)DBG_TP1_Pin;
		else
			DBG_TP1_GPIO_Port->BSRR=(uint32_t)DBG_TP1_Pin<<16;
	}
	else if(tp==TP2)
	{
		if(state)
			DBG_TP2_GPIO_Port->BSRR=(uint32_t)DBG_TP2_Pin;
		else
			DBG_TP2_GPIO_Port->BSRR=(uint32_t)DBG_TP2_Pin<<16;
	}
}

void set_CS(enum trx_t trx, uint8_t state)
{
	if(trx==CHIP_RX)
	{
		if(state)
			RX_nCS_GPIO_Port->BSRR=(uint32_t)RX_nCS_Pin;
		else
			RX_nCS_GPIO_Port->BSRR=(uint32_t)RX_nCS_Pin<<16;
	}
	else if(trx==CHIP_TX)
	{
		if(state)
			TX_nCS_GPIO_Port->BSRR=(uint32_t)TX_nCS_Pin;
		else
			TX_nCS_GPIO_Port->BSRR=(uint32_t)TX_nCS_Pin<<16;
	}
}

uint8_t trx_readreg(enum trx_t trx, uint16_t addr)
{
	uint8_t txd[3]={0, 0, 0};
	uint8_t rxd[3]={0, 0, 0};

	set_CS(trx, 0);
	if((addr>>8)==0)
	{
		txd[0]=(addr&0xFF)|0x80;
		txd[1]=0;
		HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 2, 10);
		return rxd[1];
	}
	else
	{
		txd[0]=((addr>>8)&0xFF)|0x80;
		txd[1]=addr&0xFF;
		txd[2]=0;
		HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 3, 10);
		return rxd[2];
	}
	set_CS(trx, 1);
}

void trx_writereg(enum trx_t trx, uint16_t addr, uint8_t val)
{
	uint8_t txd[3]={addr>>8, addr&0xFF, val};

	set_CS(trx, 0);
	if((addr>>8)==0)
	{
		txd[0]=addr&0xFF;
		txd[1]=val;
		HAL_SPI_Transmit(&hspi1, txd, 2, 10);
	}
	else
	{
		txd[0]=(addr>>8)&0xFF;
		txd[1]=addr&0xFF;
		txd[2]=val;
		HAL_SPI_Transmit(&hspi1, txd, 3, 10);
	}
	set_CS(trx, 1);
}

void trx_writecmd(enum trx_t trx, uint8_t addr)
{
	uint8_t txd=addr;

	set_CS(trx, 0);
	HAL_SPI_Transmit(&hspi1, &txd, 1, 10);
	set_CS(trx, 1);
}

uint8_t read_pn(enum trx_t trx)
{
	return trx_readreg(trx, 0x2F8F);
}

uint8_t read_status(enum trx_t trx)
{
	uint8_t txd=STR_SNOP; //no operation strobe
	uint8_t rxd=0;

	set_CS(trx, 0);
	HAL_SPI_TransmitReceive(&hspi1, &txd, &rxd, 1, 10);
	set_CS(trx, 1);

	return rxd;
}

void detect_ic(uint8_t* rx, uint8_t* tx)
{
	uint8_t rxid=read_pn(CHIP_RX);
	uint8_t txid=read_pn(CHIP_TX);

	if(rxid==0x20)
		sprintf((char*)rx, "CC1200");
	else if(rxid==0x21)
		sprintf((char*)rx, "CC1201");
	else
		sprintf((char*)rx, "unknown");

	if(txid==0x20)
		sprintf((char*)tx, "CC1200");
	else if(txid==0x21)
		sprintf((char*)tx, "CC1201");
	else
		sprintf((char*)tx, "unknown");
}

void config_ic(enum trx_t trx, uint8_t* settings)
{
	for(uint8_t i=0; i<51; i++)
	{
		//dbg_print("[%03d] 0x%02X 0x%02X 0x%02X\n",
		//		i*3, settings[i*3], settings[i*3+1], settings[i*3+2]);
		set_CS(trx, 0);
		if(settings[i*3])
			HAL_SPI_Transmit(&hspi1, &settings[i*3], 3, 10);
		else
			HAL_SPI_Transmit(&hspi1, &settings[i*3+1], 2, 10);
		set_CS(trx, 1);
		HAL_Delay(10);
	}
}

void config_rf(enum trx_t trx, uint32_t frequency, uint8_t tx_pwr)
{
	uint32_t freq_word=0;

	static uint8_t cc1200_rx_settings[51*3] =
	{
		0x00, 0x01, 0x08,
		0x00, 0x03, 0x09,
		0x00, 0x08, 0x1F,
		0x00, 0x0A, 0x9F, //RX bw
		0x00, 0x0B, 0x00,
		0x00, 0x0C, 0x5D,
		0x00, 0x0D, 0x00,
		0x00, 0x0E, 0x8A,
		0x00, 0x0F, 0xCB,
		0x00, 0x10, 0xAC,
		0x00, 0x11, 0x00,
		0x00, 0x12, 0x45,
		0x00, 0x13, 0x3F, //symbol rate 2
		0x00, 0x14, 0x75, //symbol rate 1
		0x00, 0x15, 0x10, //symbol rate 0
		0x00, 0x16, 0x37,
		0x00, 0x17, 0xEC,
		0x00, 0x19, 0x11,
		0x00, 0x1B, 0x51,
		0x00, 0x1C, 0x87,
		0x00, 0x1D, 0x00,
		0x00, 0x20, 0x14,
		0x00, 0x26, 0x03,
		0x00, 0x27, 0x00,
		0x00, 0x28, 0x20,
		0x00, 0x2B, 0x03,
		0x00, 0x2E, 0xFF,
		0x2F, 0x00, 0x1C,
		0x2F, 0x01, 0x02, //AFC, 0x22 - on, 0x02 - off
		0x2F, 0x04, 0x0C, //oscillator frequency is 40 MHz
		0x2F, 0x05, 0x0D,
		0x2F, 0x0C, 0x57, //freq round((float)435000000/5000000*(1<<16))
		0x2F, 0x0D, 0x00, //freq
		0x2F, 0x0E, 0x00, //freq
		0x2F, 0x10, 0xEE,
		0x2F, 0x11, 0x10,
		0x2F, 0x12, 0x07,
		0x2F, 0x13, 0xAF,
		0x2F, 0x16, 0x40,
		0x2F, 0x17, 0x0E,
		0x2F, 0x19, 0x03,
		0x2F, 0x1B, 0x33,
		0x2F, 0x1D, 0x17,
		0x2F, 0x1F, 0x00,
		0x2F, 0x20, 0x6E,
		0x2F, 0x21, 0x1C,
		0x2F, 0x22, 0xAC,
		0x2F, 0x27, 0xB5,
		0x2F, 0x32, 0x0E,
		0x2F, 0x36, 0x03,
		0x2F, 0x91, 0x08,
	};

	static uint8_t cc1200_tx_settings[51*3] =
	{
		0x00, 0x01, 0x08,
		0x00, 0x03, 0x09,
		0x00, 0x08, 0x1F,
		0x00, 0x0A, 0x06, //deviation
		0x00, 0x0B, 0x01, //deviation, LSB - exponent
		0x00, 0x0C, 0x5D,
		0x00, 0x0D, 0x00,
		0x00, 0x0E, 0x8A,
		0x00, 0x0F, 0xCB,
		0x00, 0x10, 0xAC,
		0x00, 0x11, 0x00,
		0x00, 0x12, 0x45,
		0x00, 0x13, 0x83, //symbol rate 2 - 24kSa/s
		0x00, 0x14, 0xA9, //symbol rate 1
		0x00, 0x15, 0x2A, //symbol rate 0
		0x00, 0x16, 0x37,
		0x00, 0x17, 0xEC,
		0x00, 0x19, 0x11,
		0x00, 0x1B, 0x51,
		0x00, 0x1C, 0x87,
		0x00, 0x1D, 0x00,
		0x00, 0x20, 0x14,
		0x00, 0x26, 0x03,
		0x00, 0x27, 0x00,
		0x00, 0x28, 0x20,
		0x00, 0x2B, 0x09, //power (0x03..0x3F)
		0x00, 0x2E, 0xFF,
		0x2F, 0x00, 0x1C,
		0x2F, 0x01, 0x22,
		0x2F, 0x04, 0x0C, //oscillator frequency is 40 MHz
		0x2F, 0x05, 0x09, //16x upsampler, CFM enable
		0x2F, 0x0C, 0x57, //freq 435M = 0x570000
		0x2F, 0x0D, 0x00, //freq
		0x2F, 0x0E, 0x00, //freq
		0x2F, 0x10, 0xEE,
		0x2F, 0x11, 0x10,
		0x2F, 0x12, 0x07,
		0x2F, 0x13, 0xAF,
		0x2F, 0x16, 0x40,
		0x2F, 0x17, 0x0E,
		0x2F, 0x19, 0x03,
		0x2F, 0x1B, 0x33,
		0x2F, 0x1D, 0x17,
		0x2F, 0x1F, 0x00,
		0x2F, 0x20, 0x6E,
		0x2F, 0x21, 0x1C,
		0x2F, 0x22, 0xAC,
		0x2F, 0x27, 0xB5,
		0x2F, 0x32, 0x0E,
		0x2F, 0x36, 0x03,
		0x2F, 0x91, 0x08,
	};

	freq_word=roundf((float)frequency/5000000.0*((uint32_t)1<<16));
	//dbg_print("freq_word=%04lX\n", freq_word);

	if(trx==CHIP_RX)
	{
		cc1200_rx_settings[32*3-1]=(freq_word>>16)&0xFF;
		cc1200_rx_settings[33*3-1]=(freq_word>>8)&0xFF;
		cc1200_rx_settings[34*3-1]=freq_word&0xFF;
		config_ic(trx, cc1200_rx_settings);
	}
	else if(trx==CHIP_TX)
	{
		if(tx_pwr>0x3F) tx_pwr=0x3F;
		if(tx_pwr<0x03) tx_pwr=0x03;
		cc1200_tx_settings[26*3-1]=tx_pwr;
		cc1200_tx_settings[32*3-1]=(freq_word>>16)&0xFF;
		cc1200_tx_settings[33*3-1]=(freq_word>>8)&0xFF;
		cc1200_tx_settings[34*3-1]=freq_word&0xFF;
		config_ic(trx, cc1200_tx_settings);
	}
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  set_rf_pwr_setpoint(0);
  rf_pa_en(0);
  set_CS(CHIP_RX, 1);
  set_CS(CHIP_TX, 1);
  set_TP(TP1, 0);
  set_TP(TP2, 0);

  HAL_Delay(100);
  trx_writecmd(CHIP_RX, STR_SRES);
  trx_writecmd(CHIP_TX, STR_SRES);
  dbg_print("Hello world, this is your rru-rf board\n");

  HAL_Delay(100);
  detect_ic(trx_data[CHIP_RX].name, trx_data[CHIP_TX].name);
  dbg_print("Detected RF ICs:\nRX - %s\nTX - %s\n", trx_data[CHIP_RX].name, trx_data[CHIP_TX].name);

  trx_data[CHIP_RX].frequency=435000000-7600000;
  trx_data[CHIP_TX].frequency=435000000;
  trx_data[CHIP_RX].fcorr=-12;
  trx_data[CHIP_TX].fcorr=-12;
  trx_data[CHIP_TX].pwr=0x3F; //0x03 to 0x3F

  dbg_print("Starting TRX config...\n");

  //config_rf(CHIP_RX, trx_data[CHIP_RX].frequency, trx_data[CHIP_RX].pwr);
  config_rf(CHIP_TX, trx_data[CHIP_TX].frequency, trx_data[CHIP_TX].pwr);
  trx_writereg(CHIP_RX, 0x2F0A, (uint16_t)trx_data[CHIP_RX].fcorr>>8);
  trx_writereg(CHIP_RX, 0x2F0B, (uint16_t)trx_data[CHIP_RX].fcorr&0xFF);
  trx_writereg(CHIP_TX, 0x2F0A, (uint16_t)trx_data[CHIP_TX].fcorr>>8);
  trx_writereg(CHIP_TX, 0x2F0B, (uint16_t)trx_data[CHIP_TX].fcorr&0xFF);

  dbg_print("Done\n");

  HAL_Delay(50);
  trx_writecmd(CHIP_RX, STR_SRX);
  trx_writecmd(CHIP_TX, STR_STX);

  HAL_Delay(50);
  trx_data[CHIP_RX].pll_locked = trx_readreg(CHIP_RX, 0x2F8D)%2; //FSCAL_CTRL
  trx_data[CHIP_TX].pll_locked = trx_readreg(CHIP_TX, 0x2F8D)%2;
  trx_data[CHIP_RX].pll_locked ? dbg_print("RX PLL locked\n") : dbg_print("RX PLL unlocked\n");
  trx_data[CHIP_TX].pll_locked ? dbg_print("TX PLL locked\n") : dbg_print("TX PLL unlocked\n");

  if(!trx_data[CHIP_RX].pll_locked || !trx_data[CHIP_TX].pll_locked)
  {
	  dbg_print("ERROR: At least one PLL didn't lock\nHalting\n");
	  while(1);
  }

  rf_pa_en(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //dbg_print("Status %01X\n", trx_readreg(CHIP_TX, STR_SNOP)>>4);
	  set_TP(TP1, 1);
	  set_TP(TP2, 0);
	  set_rf_pwr_setpoint(0);
	  HAL_Delay(1000);
	  set_TP(TP1, 0);
	  set_TP(TP2, 1);
	  set_rf_pwr_setpoint(3040);
	  HAL_Delay(5000);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DBG_TP1_Pin|DBG_TP2_Pin|RX_nCS_Pin|TX_nCS_Pin
                          |PA_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DBG_TP1_Pin DBG_TP2_Pin RX_nCS_Pin TX_nCS_Pin
                           PA_EN_Pin */
  GPIO_InitStruct.Pin = DBG_TP1_Pin|DBG_TP2_Pin|RX_nCS_Pin|TX_nCS_Pin
                          |PA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
