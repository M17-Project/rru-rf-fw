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
#include "m17_rrc.h"
#include "term.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IDENT_STR		"RRU-rf-board-v1.0.0 40.0000MHz CC1200 FW by SP5WWP"
#define VDDA			(3.24f)					//measured VDDA voltage
#define M17_SPS			5						//samples per symbol
#define M17_FLT_LEN		FLT_LEN_5
#define M17_BUFLEN		12						//M17 buffer depth in frames
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

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

enum tx_state_t
{
	TX_IDLE,
	TX_ACTIVE
};

enum mmdvm_comm_t
{
	COMM_IDLE,
	COMM_RDY,
	COMM_TOT,
	COMM_OVF
};

//PA
uint16_t alc_set=0;									//automatic level control setting (nonlinear)

//MMDVM stuff
volatile uint8_t rxb[100]={0};						//rx buffer for MMDVM data
volatile uint8_t rx_bc=0;							//UART1 rx byte counter
uint8_t m17_buf[M17_BUFLEN][48]={0};				//M17 frame buffer
uint8_t m17_buf_idx_wr=1;							//current frame buffer index (for writing)
uint8_t m17_buf_idx_rd=0;							//current frame buffer index (for reading)
uint32_t m17_symbols=192;							//how many symbols (frames*192) have been received so far (starts at 192)
uint32_t m17_sym_ctr=0;								//consumed symbols counter
uint8_t m17_samples=0;								//modulo M17_SPS counter for baseband sampling
enum tx_state_t tx_state=TX_IDLE;					//transmitter state
volatile uint8_t bsb_tx_pend=0;						//do we need to transmit another baseband sample?
volatile enum mmdvm_comm_t mmdvm_comm=COMM_IDLE;	//MMDVM comm status
float m17_bsb_buff[M17_FLT_LEN]={0};				//delay line for the baseband filter

//ADC stuff
uint32_t adc_vals[3]={0};						//raw values read from ADC channels 0, 1, and 2

//AD8318 RF power sensor cal data (consts for now)
//based on 2-point linear approximation
//P(U) = a*U + b + cal_atten + cal_corr
//U[V]		P[dBm]
//0.64		0.0
//1.85		-50.0
const float cal_a		= -41.3223140495868f;	//dBm/V
const float cal_b		= 26.4462809917355f;	//dBm
const float cal_offs	= 62.5f;				//dB
const float cal_corr	= -3.6f;				//dB
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dbg_print(const char* color_code, const char* fmt, ...)
{
	char str[100];
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if(color_code!=NULL)
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)color_code, strlen(color_code), 10);
		HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
		HAL_UART_Transmit(&huart3, (uint8_t*)TERM_DEFAULT, strlen(TERM_DEFAULT), 10);
	}
	else
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
	}
}

//get RF power in dBm
//and temperature in degs C, based on LMT87's output voltage
void get_adc_vals(float* pwr_fwd, float* pwr_ref, float* temp)
{
	uint32_t values[3];
	HAL_ADC_Start_DMA(&hadc1, values, 3);
	HAL_ADC_PollForConversion(&hadc1, 20);

	float u=values[2]/4096.0f*VDDA;
	HAL_Delay(1); //TODO: why is this delay needed to make this func work?

	//dbg_print(TERM_YELLOW, "[DBG] %ld, %ld, %ld\n", values[0], values[1], values[2]);
	//dbg_print(TERM_YELLOW, "[DBG] %1.4f\n", u);

	*pwr_fwd=cal_a*(values[0]/4096.0f*VDDA)+cal_b + cal_offs+cal_corr; //take coupling and attenuation into account
	*pwr_ref=cal_a*(values[1]/4096.0f*VDDA)+cal_b + cal_offs+cal_corr;
	*temp=(-13.582f+sqrtf(184.470724f+0.01732f*(2.2308f-u)*1000.0f))/0.00866f + 30.0f;
}

//set rf power output setpoint
//0 - lowest power, 0xFFF - max power
void set_rf_pwr_setpoint(uint16_t pwr)
{
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pwr>0xFFF?0xFFF:pwr);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

void set_dac_ch2(uint16_t val)
{
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, val>0xFFF?0xFFF:val);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
}

//calculate SWR based on fwd and ref power
//returns 0.0 when input data is invalid
float calc_swr(float fwd, float ref)
{
	//in an ideal world this would be physically impossible
	//but there's always some measurement error that could cause this
	if(ref>=fwd)
		return INFINITY;

	//dBm to watts
	fwd=powf(10.0f, fwd/10.0f-3.0f);
	ref=powf(10.0f, ref/10.0f-3.0f);

	float s=sqrtf(ref/fwd);
	float swr=(1.0f+s)/(1.0f-s);

	return swr;
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
		set_CS(trx, 1);
		return rxd[1];
	}
	else
	{
		txd[0]=((addr>>8)&0xFF)|0x80;
		txd[1]=addr&0xFF;
		txd[2]=0;
		HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 3, 10);
		set_CS(trx, 1);
		return rxd[2];
	}
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
		//dbg_print(TERM_YELLOW, "[%03d] 0x%02X 0x%02X 0x%02X\n",
		//		i*3, settings[i*3], settings[i*3+1], settings[i*3+2]);
		set_CS(trx, 0);
		if(settings[i*3])
			HAL_SPI_Transmit(&hspi1, &settings[i*3], 3, 10);
		else
			HAL_SPI_Transmit(&hspi1, &settings[i*3+1], 2, 10);
		set_CS(trx, 1);
		//HAL_Delay(10);
	}
}

void config_rf(enum trx_t trx, struct trx_data_t trx_data)
{
	uint32_t freq_word=0;

	static uint8_t cc1200_rx_settings[51*3] =
	{
		0x00, 0x01, 0x08,
		0x00, 0x03, 0x09,
		0x00, 0x08, 0x1F,
		0x00, 0x0A, 0x9F, //deviation - about 3kHz full scale
		0x00, 0x0B, 0x00, //deviation
		0x00, 0x0C, 0x5D,
		0x00, 0x0D, 0x00,
		0x00, 0x0E, 0x8A,
		0x00, 0x0F, 0xCB,
		0x00, 0x10, 0xAC, //RX filter BW - 9.5kHz
		0x00, 0x11, 0x00,
		0x00, 0x12, 0x45,
		0x00, 0x13, 0x3F, //symbol rate 2 - 1.2k sym/s
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
		0x00, 0x2B, 0x03, //output power - 0x03..0x3F (doesn't matter for RX)
		0x00, 0x2E, 0xFF,
		0x2F, 0x00, 0x1C,
		0x2F, 0x01, 0x02, //AFC, 0x22 - on, 0x02 - off
		0x2F, 0x04, 0x0C, //external oscillator's frequency is 40 MHz
		0x2F, 0x05, 0x09, //16x upsampler, CFM enable
		0x2F, 0x0C, 0x57, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
		0x2F, 0x0D, 0x00, //frequency
		0x2F, 0x0E, 0x00, //frequency
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
		0x2F, 0x91, 0x08
	};

	static uint8_t cc1200_tx_settings[51*3] =
	{
		0x00, 0x01, 0x08,
		0x00, 0x03, 0x09,
		0x00, 0x08, 0x1F,
		0x00, 0x0A, 0x06, //deviation - 5kHz full scale
		0x00, 0x0B, 0x01, //deviation
		0x00, 0x0C, 0x5D,
		0x00, 0x0D, 0x00,
		0x00, 0x0E, 0x8A,
		0x00, 0x0F, 0xCB,
		0x00, 0x10, 0xAC, //RX filter BW - 9.5kHz (doesn't matter for TX)
		0x00, 0x11, 0x00,
		0x00, 0x12, 0x45,
		0x00, 0x13, 0x83, //symbol rate 2 - 24k symb/s
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
		0x00, 0x2B, 0x03, //output power - 0x03..0x3F
		0x00, 0x2E, 0xFF,
		0x2F, 0x00, 0x1C,
		0x2F, 0x01, 0x22,
		0x2F, 0x04, 0x0C, //external oscillator's frequency is 40 MHz
		0x2F, 0x05, 0x09, //16x upsampler, CFM enable
		0x2F, 0x0C, 0x57, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
		0x2F, 0x0D, 0x00, //frequency
		0x2F, 0x0E, 0x00, //frequency
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
		0x2F, 0x91, 0x08
	};

	freq_word=roundf((float)trx_data.frequency/5000000.0*((uint32_t)1<<16));
	//dbg_print(0, "freq_word=%04lX\n", freq_word);

	if(trx==CHIP_RX)
	{
		cc1200_rx_settings[32*3-1]=(freq_word>>16)&0xFF;
		cc1200_rx_settings[33*3-1]=(freq_word>>8)&0xFF;
		cc1200_rx_settings[34*3-1]=freq_word&0xFF;
		config_ic(trx, cc1200_rx_settings);
	}
	else if(trx==CHIP_TX)
	{
		uint8_t tx_pwr=trx_data.pwr;
		if(tx_pwr>0x3F) tx_pwr=0x3F;
		if(tx_pwr<0x03) tx_pwr=0x03;
		cc1200_tx_settings[26*3-1]=tx_pwr;
		cc1200_tx_settings[32*3-1]=(freq_word>>16)&0xFF;
		cc1200_tx_settings[33*3-1]=(freq_word>>8)&0xFF;
		cc1200_tx_settings[34*3-1]=freq_word&0xFF;
		config_ic(trx, cc1200_tx_settings);
	}

	//frequency correction
	trx_writereg(trx, 0x2F0A, (uint16_t)trx_data.fcorr>>8);
	trx_writereg(trx, 0x2F0B, (uint16_t)trx_data.fcorr&0xFF);
	//disable address autoincrement in burst mode (default - enabled)
	trx_writereg(trx, 0x2F06, 0);
}

float m17_map_symbol(uint8_t dibit)
{
	switch(dibit)
	{
		case 0b00:
			return +1.0f;
		break;

		case 0b01:
			return +3.0f;
		break;

		case 0b10:
			return -1.0f;
		break;

		case 0b11:
			return -3.0f;
		break;
	}

	return 0.0f;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_TypeDef* tim = htim->Instance;

	//baseband sampler timer
	if(tim==TIM7)
	{
		bsb_tx_pend=1;
	}

	//USART1 timeout timer
	else if(tim==TIM6)
	{
		HAL_TIM_Base_Stop_IT(&htim6);

		if(mmdvm_comm==COMM_IDLE)
		{
			mmdvm_comm=COMM_TOT; //set the TOT flag
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	USART_TypeDef* iface = huart->Instance;

	if(iface==USART1)
	{
		//check frame's validity
		if(rxb[0]==0xE0 && rxb[1]==rx_bc+1)
		{
			HAL_TIM_Base_Stop_IT(&htim6);
			rx_bc=0;
			mmdvm_comm=COMM_RDY;
			return;
		}

		//handle overflow
		if(rx_bc<sizeof(rxb)) //all normal - proceed
		{
			rx_bc++;
			HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxb[rx_bc], 1);
			//reset timeout timer
			TIM6->CNT=0;
			FIX_TIMER_TRIGGER(&htim6);
			HAL_TIM_Base_Start_IT(&htim6);
		}
		else //overflow
		{
			mmdvm_comm=COMM_OVF; //set overflow flag, shouldn't normally happen
		}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  set_rf_pwr_setpoint(0);
  set_dac_ch2(0); //can be used for debugging
  rf_pa_en(0);
  set_CS(CHIP_RX, 1);
  set_CS(CHIP_TX, 1);
  set_TP(TP1, 0);
  set_TP(TP2, 0);

  HAL_Delay(100);
  trx_writecmd(CHIP_RX, STR_SRES);
  trx_writecmd(CHIP_TX, STR_SRES);
  dbg_print(0, TERM_CLR); //clear console and print out ident string
  dbg_print(TERM_GREEN, IDENT_STR); dbg_print(0,  "\n");

  HAL_Delay(100);
  detect_ic(trx_data[CHIP_RX].name, trx_data[CHIP_TX].name);
  dbg_print(0, "RX IC: %s\nTX IC: %s\n", trx_data[CHIP_RX].name, trx_data[CHIP_TX].name);

  trx_data[CHIP_RX].frequency=435000000; //default
  trx_data[CHIP_TX].frequency=435000000;
  trx_data[CHIP_RX].fcorr=trx_data[CHIP_TX].fcorr=-9; //shared clock source, thus the same corr
  trx_data[CHIP_TX].pwr=3; //3 to 63

  dbg_print(0, "Starting TRX config...");
  config_rf(CHIP_RX, trx_data[CHIP_RX]);
  config_rf(CHIP_TX, trx_data[CHIP_TX]);
  dbg_print(TERM_GREEN, " done\n");

  HAL_Delay(50);
  trx_writecmd(CHIP_RX, STR_SRX);
  trx_writecmd(CHIP_TX, STR_STX);

  HAL_Delay(50);
  trx_data[CHIP_RX].pll_locked = trx_readreg(CHIP_RX, 0x2F8D)%2; //FSCAL_CTRL
  trx_data[CHIP_TX].pll_locked = trx_readreg(CHIP_TX, 0x2F8D)%2;
  dbg_print(0, "RX PLL");
  trx_data[CHIP_RX].pll_locked ? dbg_print(TERM_GREEN, " locked\n") : dbg_print(TERM_RED, " unlocked\n");
  dbg_print(0, "TX PLL");
  trx_data[CHIP_TX].pll_locked ? dbg_print(TERM_GREEN, " locked\n") : dbg_print(TERM_RED, " unlocked\n");

  if(!trx_data[CHIP_RX].pll_locked || !trx_data[CHIP_TX].pll_locked)
  {
	  dbg_print(TERM_RED, "ERROR: At least one PLL didn't lock\nHalting\n");
	  while(1);
  }

  //dbg_print(TERM_YELLOW, "[DBG] TX status %01X\n", trx_readreg(CHIP_TX, STR_SNOP)>>4);
  rf_pa_en(1);
  dbg_print(TERM_YELLOW, "[DBG] RF_PA enabled\n");

  //enable MMDVM comms over UART1
  //memset((uint8_t*)rxb, 0, sizeof(rxb));
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(0) //PA test
  {
	  float fwd=0.0, ref=0.0, temp=0.0, swr=0.0;

	  set_rf_pwr_setpoint(0);
	  HAL_Delay(1000);
	  //get_rf_pwr_dbm(&fwd, &ref);
	  //swr=calc_swr(fwd, ref);
	  //dbg_print(0, "FWD=%2.1fdBm REF=%2.1fdBm, SWR=%-.--f\n", fwd, ref);

	  set_rf_pwr_setpoint(2500);
	  HAL_Delay(5000);
	  get_adc_vals(&fwd, &ref, &temp);
	  swr=calc_swr(fwd, ref);
	  if(fwd>=27.0)
		  dbg_print(0, "FWD=%2.1fdBm REF=%2.1fdBm SWR=%2.2f T=%3.1f\n", fwd, ref, swr, temp);
	  else
		  dbg_print(0, "FWD=--.-dBm REF=--.-fdBm SWR=--.- T=%3.1f\n", temp);
  }

  while(0) //temp test
  {
	  float temp=0.0;

	  get_adc_vals(NULL, NULL, &temp);
	  dbg_print(0, "T=%3.1f\n", temp);
	  HAL_Delay(500);
  }

  while(1) //MMDVM test
  {
	  if(mmdvm_comm==COMM_RDY) //if a valid MMDVM frame is detected
	  {
		  set_TP(TP1, 1);
		  HAL_UART_AbortReceive_IT(&huart1);
		  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);
		  mmdvm_comm=COMM_IDLE;
		  set_TP(TP1, 0);

		  //dbg_print(0, "type 0x%02X\tlen %02d\n", rxb[2], rxb[1]);
		  uint8_t cmd=rxb[2];

		  if(cmd==0x00) //"Get Version"
		  {
			  //reply with RRU ident string
			  uint8_t ident[70];
			  sprintf((char*)&ident[4], IDENT_STR);
			  ident[0]=0xE0; //header
			  ident[2]=0x00; //a reply to "Get Version" command
			  ident[3]=0x01; //protocol version
			  ident[1]=strlen((char*)&ident[4])+4; //total length
			  HAL_UART_Transmit_IT(&huart1, ident, ident[1]);
		  }
		  else if(cmd==0x04) //"???" - set RX/TX frequencies etc. TODO: I'm not sure if 0x04 command sets power output
		  {
			  uint32_t rx_freq, tx_freq;
			  memcpy((uint8_t*)&rx_freq, (uint8_t*)&rxb[4], sizeof(uint32_t));
			  memcpy((uint8_t*)&tx_freq, (uint8_t*)&rxb[8], sizeof(uint32_t));
			  dbg_print(0, "[MMDVM_CMD] RX %ld Hz\n[MMDVM_CMD] TX %ld Hz\n", rx_freq, tx_freq);
			  //reconfig TRXs
			  trx_data[CHIP_RX].frequency=rx_freq;
			  trx_data[CHIP_TX].frequency=tx_freq;
			  config_rf(CHIP_RX, trx_data[CHIP_RX]);
			  config_rf(CHIP_TX, trx_data[CHIP_TX]);
			  alc_set=2000; //0 - no output, 3000 - about 47.8dBm (60W)
			  //ACK it
			  uint8_t ack[4]={0xE0, 0x04, 0x70, cmd};
			  HAL_UART_Transmit_IT(&huart1, ack, 4);
		  }
		  else if(cmd==0x02) //"Set Config"
		  {
			  //for now just reply with an ACK
			  uint8_t ack[4]={0xE0, 0x04, 0x70, cmd};
			  HAL_UART_Transmit_IT(&huart1, ack, 4);
		  }
		  else if(cmd==0x03) //"Set Mode"
		  {
			  //for now just reply with an ACK
			  uint8_t ack[4]={0xE0, 0x04, 0x70, cmd};
			  HAL_UART_Transmit_IT(&huart1, ack, 4);
		  }
		  else if(cmd==0x01) //"Get Status"
		  {
			  //reply with a hardcoded sequence
			  uint8_t reply[14]={0xE0, 0x0E, 0x01, 0x80,
			  	  	  	  	  	0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00,
			  	  	  	  	  	0x00, 0x1F};
			  HAL_UART_Transmit_IT(&huart1, reply, 14);
		  }
		  else if(cmd==0x45 || cmd==0x46) //M17 LSF frame data or stream frame data
		  {
			  //HAL_UART_Transmit(&huart3, (uint8_t*)&rxb[4], 48, 6); //debug data dump
			  memcpy((uint8_t*)&m17_buf[m17_buf_idx_wr][0], (uint8_t*)&rxb[4], 48);
			  m17_symbols+=192;
			  m17_buf_idx_wr++;
			  m17_buf_idx_wr%=M17_BUFLEN;
			  //start transmitting only after receiving some frames
			  //to avoid possible buffer underflows
			  if(tx_state==TX_IDLE && m17_buf_idx_wr>2)
			  {
				  tx_state=TX_ACTIVE;
				  trx_data[CHIP_TX].pwr=63;
				  trx_writereg(CHIP_TX, 0x002B, trx_data[CHIP_TX].pwr);
				  set_rf_pwr_setpoint(alc_set);
				  dbg_print(0, "TX -> start\n");
				  //fill the preamble
				  memset((uint8_t*)&m17_buf[0][0], 0b01110111, 48);
				  //initiate baseband SPI transfer to the transmitter
				  uint8_t header[2]={0x2F|0x40, 0x7E}; //CFM_TX_DATA_IN, burst access
				  set_CS(CHIP_TX, 0); //CS low
				  HAL_SPI_Transmit(&hspi1, header, 2, 10); //send 3-byte header
				  TIM7->CNT=0;
				  //FIX_TIMER_TRIGGER(&htim7);
				  HAL_TIM_Base_Start_IT(&htim7); //baseband sample timer
				  //set_TP(TP2, 1); //debug
			  }
		  }
	  }
	  else if(mmdvm_comm==COMM_TOT || mmdvm_comm==COMM_OVF)
	  {
		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_UART_AbortReceive_IT(&huart1);
		  rx_bc=0;
		  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);
		  mmdvm_comm=COMM_IDLE;
	  }

	  if(bsb_tx_pend==1)
	  {
		  set_TP(TP2, 1); //debug
		  m17_samples++;

		  //push buffer
		  for(uint8_t i=M17_FLT_LEN-1; i>0; i--)
		  {
			  m17_bsb_buff[i]=m17_bsb_buff[i-1];
		  }

		  //fetch next sample
		  if(m17_samples==M17_SPS)
		  {
			  uint8_t dibit=0;
			  //take another dibit from the buffer
			  dibit=(m17_buf[m17_buf_idx_rd][(m17_sym_ctr/4)%48]>>(6-(m17_sym_ctr%4)*2)) & 0b11;

			  //map to a symbol and push to buffer
			  m17_bsb_buff[0]=m17_map_symbol(dibit);

			  m17_samples=0;
			  m17_sym_ctr++;
			  m17_buf_idx_rd=(m17_sym_ctr/192)%M17_BUFLEN;
		  }
		  else
		  {
			  m17_bsb_buff[0]=0.0f;
		  }

		  //calculate next baseband sample
		  float f_bsb_sample=0.0f;
		  for(uint8_t i=0; i<M17_FLT_LEN; i++)
		  	  f_bsb_sample+=rrc_taps_5[i]*m17_bsb_buff[i];

		  //scaling factor required to get +2.4k deviation for +3 symbol
		  int8_t bsb_sample=roundf(f_bsb_sample*23.08f);
		  set_dac_ch2(bsb_sample*32+2048); //check if we don't overflow int8_t
		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&bsb_sample, 1, 2); //send baseband sample

		  //nothing else to transmit
		  if(m17_sym_ctr==m17_symbols)
		  {
			  HAL_TIM_Base_Stop_IT(&htim7); //baseband sample timer
			  trx_data[CHIP_TX].pwr=3;
			  trx_writereg(CHIP_TX, 0x002B, trx_data[CHIP_TX].pwr);
			  set_rf_pwr_setpoint(0);
			  set_CS(CHIP_TX, 1); //CS high
			  tx_state=TX_IDLE;
			  m17_buf_idx_wr=1;
			  m17_buf_idx_rd=0;
			  m17_samples=0;
			  m17_symbols=192;
			  m17_sym_ctr=0;
			  //set_TP(TP2, 0); //debug
			  dbg_print(0, "TX -> end\n");
		  }

		  set_TP(TP2, 0); //debug
		  bsb_tx_pend=0;
	  }
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4200-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3500-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
