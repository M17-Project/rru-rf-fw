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
#include "cc1200.h"
#include "term.h"
#include "interface_cmds.h"
#include "enums.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IDENT_STR				"Remote Radio Unit (RRU) 420-450 MHz\nFW v2.0.0 by Wojciech SP5WWP"

#define DBG_MSGS											//enable debug messages over DBG_UART
#define VDDA					3.24f						//measured VDDA voltage
#define CC1200_REG_NUM			51							//number of regs used to initialize CC1200s
#define FCORR_STEP				(40.0e6f/262144.0f/8.0f)	//unit value for frequency correction (262144=2^18)
#define UART_LONG_BUF_SIZE		1024
#define UART_SHORT_BUF_SIZE		128							//buffer length for UART
#define SHORT_TXQ_SIZE			8							//TX queue length (short messages)
#define LONG_TXQ_SIZE   		2							//TX queue length (baseband transfers)
#define BSB_SIZE				960							//size of baseband chunks
#define BSB_TX_BUFF_SIZE		(10*BSB_SIZE)				//large buffer for baseband
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))
#define DBG_HALT while(1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
/* USER CODE BEGIN PV */
//aliases
UART_HandleTypeDef *ctrl_uart = &huart1;
UART_HandleTypeDef *dbg_uart = &huart3;
TIM_HandleTypeDef *ctrl_uart_tot = &htim6;
TIM_HandleTypeDef *adc_timebase = &htim8;

//errors
const char *errstrings[5] =
{
	"OK",
	"RX PLL did not lock",
	"TX PLL did not lock",
	"RX SPI communication error",
	"TX SPI communication error"
};


//settings
struct rru_settings_t
{
	uint32_t tx_freq;
	uint32_t rx_freq;
	float tx_pwr_dbm;
} rru_settings;

trx_data_t trx_data[2];

uint32_t dev_err; //default state - no error

//ADC stuff
volatile uint16_t adc_vals[3]; //raw values read from ADC channels 0, 1, and 2

//AD8318 RF power sensor cal data (consts for now)
//based on 2-point linear approximation
//P(U) = a*U + b + cal_atten + cal_corr
//U[V]		P[dBm]
//0.64		0.0
//1.85		-50.0
const float cal_a		= -41.3223140495868f;	//dBm/V
const float cal_b		=  26.4462809917355f;	//dBm
const float cal_offs	=  62.5f;				//dB
const float cal_corr	=  -3.6f;				//dB

//UART control and baseband transfer
volatile uint8_t uart_rx[UART_LONG_BUF_SIZE];	//DMA UART RX buffer
volatile uint16_t uart_rx_tail;
enum uart_state_t uart_state = WAIT_ID;		//idle state

uint8_t cmd_id;
uint16_t cmd_len;
uint16_t payload_pos;
uint8_t payload[1024];

typedef struct											//TX queue - short packets
{
	uint8_t  data[UART_SHORT_BUF_SIZE];
	uint16_t len;
} txq_queue_t;
txq_queue_t txq[SHORT_TXQ_SIZE];						//TX queue - short packets

volatile uint8_t txq_head;
volatile uint8_t txq_tail;
volatile uint8_t txq_busy;   							//ongoing DMA transfer?

typedef struct											//TX queue - long packets
{
    uint8_t data[UART_LONG_BUF_SIZE];
    uint16_t len;
} bsbq_queue_t;
bsbq_queue_t bsbq[LONG_TXQ_SIZE];						//TX queue - long packets

volatile uint8_t bsbq_head;
volatile uint8_t bsbq_tail;
volatile uint8_t bsbq_busy;

/*uint8_t bsb_rx[2*BSB_SIZE];								//rx samples
volatile uint8_t rx_pend;								//pending rx sample read?
uint16_t rx_num_wr;*/

uint8_t circ_bsb_tx_buff[BSB_TX_BUFF_SIZE];
volatile uint16_t circ_bsb_buff_tail;
volatile uint16_t circ_bsb_buff_head;
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
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dbg_print(const char* color_code, const char* fmt, ...)
{
	#ifdef DBG_MSGS
	char str[100];
	va_list ap;

	va_start(ap, fmt);
	vsprintf(str, fmt, ap);
	va_end(ap);

	if(color_code!=NULL)
	{
		//TODO: replace this with queued DMA
		HAL_UART_Transmit(&huart3, (uint8_t*)color_code, strlen(color_code), 10);
		HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
		HAL_UART_Transmit(&huart3, (uint8_t*)TERM_DEFAULT, strlen(TERM_DEFAULT), 10);
	}
	else
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 100);
	}
	#endif
}

uint16_t dbm_to_alc(float dbm)
{
	//TODO: magic numbers
	return roundf(43.8033381164565f*dbm + 564.564609923722f);
}

//get RF power in dBm
//and temperature in degs C, based on LMT87's output voltage
void get_meas_vals(float* pwr_fwd, float* pwr_ref, float* temp)
{
	*pwr_fwd = cal_a * (adc_vals[0]/4096.0f*VDDA) + cal_b + cal_offs+cal_corr; //take coupling and attenuation into account
	*pwr_ref = cal_a * (adc_vals[1]/4096.0f*VDDA) + cal_b + cal_offs+cal_corr;
	*temp = (-13.582f+sqrtf(184.470724f+0.01732f*(2.2308f-(adc_vals[2]/4096.0f*VDDA))*1000.0f))/0.00866f + 30.0f;
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
	if(ref >= fwd)
		return INFINITY;

	//dBm to watts
	fwd = powf(10.0f, fwd/10.0f - 3.0f);
	ref = powf(10.0f, ref/10.0f - 3.0f);

	float s = sqrtf(ref/fwd);

	return (1.0f+s)/(1.0f-s);
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

void set_uart_ctrl_timeout(uint8_t ms)
{
	ctrl_uart_tot->Instance->ARR = (uint32_t)ms*10 - 1;
}

void platform_init(void)
{
	//enable FPU
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

	//default settings
	set_rf_pwr_setpoint(0);
	set_dac_ch2(0); //can be used for debugging
	rf_pa_en(0);
	trx_set_CS(CHIP_RX, 1);
	trx_set_CS(CHIP_TX, 1);
	set_TP(TP1, 0);
	set_TP(TP2, 0);

	HAL_Delay(500);
	trx_reset(CHIP_RX);
	trx_reset(CHIP_TX);

	HAL_Delay(100);
	trx_write_cmd(CHIP_RX, STR_IDLE);
	trx_write_cmd(CHIP_TX, STR_IDLE);

	dbg_print(0, TERM_CLR); //clear console and print out ident string
	dbg_print(TERM_GREEN, IDENT_STR); dbg_print(0,  "\n");

	HAL_Delay(10);
	trx_detect(CHIP_RX, trx_data[CHIP_RX].name);
	trx_detect(CHIP_TX, trx_data[CHIP_TX].name);
	dbg_print(0, "RX IC: %s\nTX IC: %s\n", trx_data[CHIP_RX].name, trx_data[CHIP_TX].name);

	trx_data[CHIP_RX].freq = 433475000;							//default
	trx_data[CHIP_TX].freq = 435000000;
	trx_data[CHIP_RX].fcorr = 0;
	trx_data[CHIP_TX].fcorr = 0;
	trx_data[CHIP_TX].pwr = 27;									//3 to 63
	rru_settings.tx_pwr_dbm = 30.00f;							//30dBm (1W) default
	set_rf_pwr_setpoint(dbm_to_alc(rru_settings.tx_pwr_dbm));	//set the DAC for RF power setpoint

	dbg_print(0, "Starting TRX config...");
	trx_config(CHIP_RX, trx_data[CHIP_RX]);
	trx_config(CHIP_TX, trx_data[CHIP_TX]);
	trx_write_reg(CHIP_TX, 0x2F7E, 0); //zero frequency deviation at TX idle
	dbg_print(TERM_GREEN, " done\n");

	HAL_Delay(50);
	trx_write_cmd(CHIP_RX, STR_SCAL);
	trx_write_cmd(CHIP_TX, STR_SCAL);

	HAL_Delay(50);
	trx_data[CHIP_RX].pll_locked = ((trx_read_reg(CHIP_RX, 0x2F8D)^0x80)&0x81)==0x81; //FSCAL_CTRL=1 and FSCAL_CTRL_NOT_USED=0
	trx_data[CHIP_TX].pll_locked = ((trx_read_reg(CHIP_TX, 0x2F8D)^0x80)&0x81)==0x81;
	dbg_print(0, "RX PLL");
	trx_data[CHIP_RX].pll_locked ? dbg_print(TERM_GREEN, " locked\n") : dbg_print(TERM_RED, " unlocked\n");
	dbg_print(0, "TX PLL");
	trx_data[CHIP_TX].pll_locked ? dbg_print(TERM_GREEN, " locked\n") : dbg_print(TERM_RED, " unlocked\n");

	if(!trx_data[CHIP_RX].pll_locked)
	{
		dbg_print(TERM_RED, "ERROR %d: %s\n", ERR_RX_PLL, errstrings[ERR_RX_PLL]);
		dev_err|=(1UL<<ERR_RX_PLL);
	}
	if(!trx_data[CHIP_TX].pll_locked)
	{
		dbg_print(TERM_RED, "ERROR %d: %s\n", ERR_TX_PLL, errstrings[ERR_TX_PLL]);
		dev_err|=(1UL<<ERR_TX_PLL);
	}
	if(strstr((char*)trx_data[CHIP_RX].name, "unknown")!=NULL)
	{
		dbg_print(TERM_RED, "ERROR %d: %s\n", ERR_RX_SPI, errstrings[ERR_RX_SPI]);
		dev_err|=(1UL<<ERR_RX_SPI);
	}
	if(strstr((char*)trx_data[CHIP_TX].name, "unknown")!=NULL)
	{
		dbg_print(TERM_RED, "ERROR %d: %s\n", ERR_TX_SPI, errstrings[ERR_TX_SPI]);
		dev_err|=(1UL<<ERR_TX_SPI);
	}

	trx_write_cmd(CHIP_RX, STR_SRX);
	trx_write_cmd(CHIP_TX, STR_STX);

	//dbg_print(TERM_YELLOW, "[DBG] TX status %01X\n", trx_read_reg(CHIP_TX, STR_SNOP)>>4);
	//dbg_print(TERM_YELLOW, "[DBG] UART1 BRR: %08X\n", huart1.Instance->BRR);

	//start ADC conversions
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_vals, 3);
	HAL_TIM_Base_Start(adc_timebase);

	//enable interface comms over UART1
	set_uart_ctrl_timeout(10); //10ms timeout
	HAL_UART_Receive_DMA(ctrl_uart, (uint8_t*)uart_rx, sizeof(uart_rx));
}

uint16_t uart_ctrl_data_avbl(void)
{
    uint16_t head = UART_LONG_BUF_SIZE - __HAL_DMA_GET_COUNTER(ctrl_uart->hdmarx);

    if (head >= uart_rx_tail)
        return head - uart_rx_tail;
    else
        return UART_LONG_BUF_SIZE - uart_rx_tail + head;
}

int uart_ctrl_get_byte(uint8_t *b)
{
    if (uart_ctrl_data_avbl() == 0)
        return 0;

    *b = uart_rx[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_LONG_BUF_SIZE;

    //start the timeout timer when the data is detected, not when it arrives
    FIX_TIMER_TRIGGER(ctrl_uart_tot);
    ctrl_uart_tot->Instance->CNT = 0;
    HAL_TIM_Base_Start_IT(ctrl_uart_tot);

    return 1;
}

void interface_resp_short(uint8_t cid, const uint8_t *resp, uint16_t pld_len)
{
	uint16_t len = pld_len + 3;

	//drop oversized
    if (len > UART_SHORT_BUF_SIZE)
    	return;

    //next queue entry
    uint8_t next = (txq_head+1) % SHORT_TXQ_SIZE;

    //queue full, drop packet
    if (next == txq_tail)
    	return;

    txq[txq_head].data[0] = cid;
    memcpy(&txq[txq_head].data[1], (uint8_t*)&len, sizeof(uint16_t));
    memcpy(&txq[txq_head].data[3], resp, pld_len);
    txq[txq_head].len = len;
    txq_head = next;

    //if UART TX is idle, start transmission
    if (!txq_busy && !bsbq_busy)
    {
        txq_busy = 1;
        HAL_UART_Transmit_DMA(ctrl_uart, txq[txq_tail].data, txq[txq_tail].len);
    }
}

void interface_resp_long(uint8_t cid, const uint8_t *resp, uint16_t pld_len)
{
	uint16_t len = pld_len+3;

	//drop oversized
    if (len > UART_LONG_BUF_SIZE)
    	return;

    //next queue entry
    uint8_t next = (bsbq_head+1) % LONG_TXQ_SIZE;

    //queue full, drop packet
    if (next == bsbq_tail)
    	return;

    bsbq[bsbq_head].data[0] = cid;
    memcpy(&bsbq[bsbq_head].data[1], (uint8_t*)&len, sizeof(uint16_t));
    memcpy(&bsbq[bsbq_head].data[3], resp, pld_len);
    bsbq[bsbq_head].len = len;
    bsbq_head = next;

    //if UART TX is idle, start transmission
    if (!txq_busy && !bsbq_busy)
    {
    	bsbq_busy = 1;
        HAL_UART_Transmit_DMA(ctrl_uart, bsbq[bsbq_tail].data, bsbq[bsbq_tail].len);
    }
}

void interface_resp_byte(enum cmd_t cmd, uint8_t resp)
{
	//single-byte response (4 bytes total)
	interface_resp_short(cmd, &resp, 1);
}

void handle_command(uint8_t cid, uint8_t *pld, uint16_t pld_len)
{
	//float f_val;
	//int8_t i8_val;
	uint8_t u8_val;
	int16_t i16_val;
	uint32_t u32_val;

	switch (cid)
	{
		case CMD_PING:
			interface_resp_short(cid, (uint8_t*)&dev_err, sizeof(dev_err));
		break;

		case CMD_SET_RX_FREQ:
		{
			if (pld_len == sizeof(uint32_t))
			{
				memcpy((uint8_t*)&u32_val, pld, sizeof(uint32_t));
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(u32_val>=420e6 && u32_val<=450e6)
			{
				trx_data[CHIP_RX].freq = u32_val;
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		}
		break;

		case CMD_SET_TX_FREQ:
			if (pld_len == sizeof(uint32_t))
			{
				memcpy((uint8_t*)&u32_val, pld, sizeof(uint32_t));
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(u32_val>=420e6 && u32_val<=450e6)
			{
				trx_data[CHIP_TX].freq = u32_val;
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

		case CMD_SET_TX_POWER:
			if (pld_len == 1)
			{
				u8_val = pld[0]; //value in dBm
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(u8_val>=30 && u8_val<=47) //30 to 47 dBm
			{
				rru_settings.tx_pwr_dbm = (float)u8_val;
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

	  	case CMD_SET_RX_FREQ_CORR:
	  		if (pld_len == sizeof(int16_t))
	  		{
	  			memcpy((uint8_t*)&i16_val, pld, sizeof(int16_t));
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

			if(i16_val>=-200 && i16_val<=200) //keep it within a sane range
			{
				trx_data[CHIP_RX].fcorr = i16_val;
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

	  	case CMD_SET_TX_FREQ_CORR:
	  		if (pld_len == sizeof(int16_t))
	  		{
	  			memcpy((uint8_t*)&i16_val, pld, sizeof(int16_t));
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

			if(i16_val>=-200 && i16_val<=200) //keep it within a sane range
			{
				trx_data[CHIP_TX].fcorr = i16_val;
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

	  	case CMD_SET_AFC:
	  		if (pld_len == 1)
	  		{
	  			u8_val = *pld;
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

	  		//the change will be applied at the next TX->RX switch
			trx_data[CHIP_RX].afc = u8_val==0 ? 0 : 1;
			interface_resp_byte(cid, ERR_OK);
		break;

	  	case CMD_TX_START:
	  		if (pld_len == 1)
	  		{
	  			u8_val = *pld;
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

	  		if(u8_val) //start
	  		{
	  			trx_config(CHIP_TX, trx_data[CHIP_TX]);
	  			set_rf_pwr_setpoint(dbm_to_alc(rru_settings.tx_pwr_dbm));
	  			rf_pa_en(1);
	  			/*if(trx_state==TRX_IDLE && dev_err==ERR_OK)
				{
					  //reset the buffer and state machine
					  memset(circ_bsb_tx_buff, 0, sizeof(circ_bsb_tx_buff));
					  circ_bsb_buff_tail = 0;
					  circ_bsb_buff_head = 0;

					  //config CC1200
					  trx_state = TRX_TX;
					  trx_config(MODE_TX, trx_data);
					  HAL_Delay(10);

					  //switch CC1200 to TX
					  trx_write_cmd(STR_STX);

					  //initiate samples transfer (write)
					  uint8_t header[2]={0x2F|0x40, 0x7E}; //CFM_TX_DATA_IN, burst access
					  trx_set_CS(0); //CS low
					  HAL_SPI_Transmit(&hspi1, header, 2, 2); //send 2-byte header

					  //signal TX state with LEDs
					  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 1);
					  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);

					  //enable external baseband sample triggering
					  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

					  //reply
					  interface_resp_byte(cid, ERR_OK);
				}
				else
				{
					if (dev_err!=ERR_OK)
						interface_resp_byte(cid, ERR_OTHER);
					else
						interface_resp_byte(cid, ERR_BUSY);
				}*/
	  		}
	  		else //stop
	  		{
	  			rf_pa_en(0);
	  			/*if(trx_state==TRX_TX && dev_err==ERR_OK)
				{
					  //disable external baseband sample triggering
					  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

					  //finalize samples transfer
					  trx_set_CS(1); //CS high

					  //switch state
					  trx_state = TRX_IDLE;

					  //config CC1200
					  trx_config(MODE_RX, trx_data);
					  HAL_Delay(10);

					  //switch CC1200 to RX state
					  trx_write_cmd(STR_SRX);

					  //turn off the TX LED
					  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);

					  //reply
					  interface_resp_byte(cid, ERR_OK);
				}
				else
				{
					if (dev_err!=ERR_OK)
						interface_resp_byte(cid, ERR_OTHER);
					else
						interface_resp_byte(cid, ERR_NOP);
				}*/
	  		}
	  	break;

/*	  	case CMD_RX_START:
	  		if (pld_len == 1)
	  		{
	  			u8_val = *pld;
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

	  		if(u8_val) //start
	  		{
	  			if(trx_state==TRX_IDLE && dev_err==ERR_OK)
	  			{
	  				  //reset buffers
	  				  memset(bsb_rx, 0, sizeof(bsb_rx));
	  				  rx_pend = 0;
	  				  rx_num_wr = 0;

	  				  //config CC1200
	  				  trx_config(MODE_RX, trx_data);
	  				  HAL_Delay(10);

	  				  //switch CC1200 to RX
	  				  trx_write_cmd(STR_SRX);
	  				  trx_state = TRX_RX;

	  				  //initiate samples transfer (readout)
	  				  uint8_t header[2] = {0x2F|0xC0, 0x7D};	//CFM_RX_DATA_OUT, burst access
	  				  trx_set_CS(0);							//CS low
	  				  HAL_SPI_Transmit(&hspi1, header, 2, 10);	//send 2-byte header

	  				  //for some reason, the external signal runs at 75.7582kHz instead of expected 24kHz
	  				  FIX_TIMER_TRIGGER(&htim11);
	  				  TIM11->CNT = 0;
	  				  HAL_TIM_Base_Start_IT(&htim11);

	  				  //signal RX state with LEDs
	  				  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 1);
	  				  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);

	  				  //reply
	  				  interface_resp_byte(cid, ERR_OK);
	  			}
	  			else
	  			{
	  				if (dev_err!=ERR_OK)
	  					interface_resp_byte(cid, ERR_OTHER);
	  				else
	  					interface_resp_byte(cid, ERR_BUSY);
	  			}
	  		}
	  		else //stop
	  		{
	  			if (trx_state==TRX_RX && dev_err==ERR_OK)
	  			{
					  //disable read baseband trigger signal
					  HAL_TIM_Base_Stop_IT(&htim11);

					  //finalize SPI samples transfer
					  trx_set_CS(1);

					  //switch device state
					  trx_state = TRX_IDLE;

					  //turn off the RX LED
					  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);

					  //reply
					  interface_resp_byte(cid, ERR_OK); //OK
	  			}
	  			else
	  			{
	  				if (dev_err!=ERR_OK)
	  					interface_resp_byte(cid, ERR_OTHER);
	  				else
	  					interface_resp_byte(cid, ERR_NOP);
	  			}
	  		  }
	  	break;

	  	case CMD_TX_DATA:
	  		  if (trx_state==TRX_TX && pld_len==960)
	  		  {
	  			  //enough space in the circular buffer?
	  			  if ((circ_bsb_buff_tail - circ_bsb_buff_head - 1 + BSB_TX_BUFF_SIZE) % BSB_TX_BUFF_SIZE >= 960)
	  			  {
	  				  u32_val = BSB_TX_BUFF_SIZE - circ_bsb_buff_head;

	  				  if (u32_val >= 960)
	  				  {
	  					  memcpy(&circ_bsb_tx_buff[circ_bsb_buff_head], pld, 960);
	  				  }
	  				  else
	  				  {
					      memcpy(&circ_bsb_tx_buff[circ_bsb_buff_head], pld, u32_val);
						  memcpy(&circ_bsb_tx_buff[0], pld + u32_val, 960 - u32_val);
	  				  }

					  circ_bsb_buff_head = (circ_bsb_buff_head+960) % BSB_TX_BUFF_SIZE;
					  interface_resp_byte(cid, ERR_OK);
	  			  }
	  			  else
	  			  {
	  				  interface_resp_byte(cid, ERR_BUFF_FULL);
	  			  }
	  		  }
	  	  break;*/

	  	  case CMD_GET_IDENT:
			  //reply with the device's IDENT string
			  interface_resp_short(cid, (uint8_t*)IDENT_STR, strlen(IDENT_STR));
		  break;

	  	  case CMD_GET_CAPS:
			  //FM only, half-duplex for now
			  interface_resp_byte(cid, 0x02);
		  break;

	  	  case CMD_GET_RX_FREQ:
	  		  interface_resp_short(cid, (uint8_t*)&trx_data[CHIP_RX].freq, sizeof(uint32_t));
		  break;

	  	  case CMD_GET_TX_FREQ:
	  		  interface_resp_short(cid, (uint8_t*)&trx_data[CHIP_TX].freq, sizeof(uint32_t));
		  break;

	  	  case CMD_GET_BSB_BUFF:
	  		  //TODO: put some working code here
	  		  interface_resp_byte(cid, 0);
	  	  break;

	  	  case CMD_GET_RSSI:
	  		  u8_val = trx_read_reg(CHIP_RX, 0x2F71); //RSSI
	  		  interface_resp_byte(cid, u8_val);
		  break;
	}
}

void uart_ctrl_process(void)
{
    uint8_t b;

    while (uart_ctrl_get_byte(&b))
    {
        switch (uart_state)
        {
			case WAIT_ID:
				cmd_id = b;
				uart_state = WAIT_LEN1;
			break;

			case WAIT_LEN1:
				cmd_len = b;
				uart_state = WAIT_LEN2;
			break;

			case WAIT_LEN2:
				cmd_len |= ((uint16_t)b << 8);
				payload_pos = 0;

				if (cmd_len == 3)
				{
					handle_command(cmd_id, NULL, 0);
					uart_state = WAIT_ID;
				}
				else
					uart_state = WAIT_PAYLOAD;
			break;

			case WAIT_PAYLOAD:
				payload[payload_pos++] = b;

				if (payload_pos >= cmd_len - 3)
				{
					handle_command(cmd_id, payload, payload_pos);
					uart_state = WAIT_ID;
				}
			break;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_TypeDef* tim = htim->Instance;

	//USART1 timeout timer
	if(tim == TIM6)
	{
		HAL_TIM_Base_Stop_IT(ctrl_uart_tot);
		uart_state = WAIT_ID;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//24kHz trigger signal from the RX CC1200
	if(GPIO_Pin == TX_TRIG_Pin)
	{
		set_TP(TP1, 1);

		;

		set_TP(TP1, 0);
	}
}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	USART_TypeDef* iface = huart->Instance;

	//UART for control
	if(iface==USART1)
	{
		;
	}
}*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == ctrl_uart)
    {
    	if (bsbq_busy)
    	{
    		bsbq_tail = (bsbq_tail+1) % LONG_TXQ_SIZE;

			//any more messages pending?
			if (bsbq_tail != bsbq_head)
			{
				HAL_UART_Transmit_DMA(huart, bsbq[bsbq_tail].data, bsbq[bsbq_tail].len);
			}
			else //TX queue is empty
			{
				bsbq_busy = 0;
			}
    	}

    	else if (txq_busy)
    	{
			txq_tail = (txq_tail+1) % SHORT_TXQ_SIZE;

			//any more messages pending?
			if (txq_tail != txq_head)
			{
				HAL_UART_Transmit_DMA(huart, txq[txq_tail].data, txq[txq_tail].len);
			}
			else //TX queue is empty
			{
				txq_busy = 0;
			}
    	}

    	//continue the transmission if there's data in the other queue
        if (!bsbq_busy && bsbq_tail != bsbq_head)
        {
            //start next long packet
            bsbq_busy = 1;
            HAL_UART_Transmit_DMA(huart, bsbq[bsbq_tail].data, bsbq[bsbq_tail].len);
            return;
        }

        if (!txq_busy && txq_tail != txq_head)
        {
            //start next short packet
            txq_busy = 1;
            HAL_UART_Transmit_DMA(huart, txq[txq_tail].data, txq[txq_tail].len);
            return;
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
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  platform_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*while(0) //PA test
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

  while(0) //PA test 2
  {
	  trx_writecmd(CHIP_TX, STR_IDLE);
	  HAL_Delay(10);
	  trx_data[CHIP_TX].frequency=438812500;
	  //trx_data[CHIP_TX].pwr=63;
	  config_rf(CHIP_TX, trx_data[CHIP_TX]);
	  HAL_Delay(10);
	  trx_writecmd(CHIP_TX, STR_STX);
	  //set_rf_pwr_setpoint(1200); //14.35dBm
	  //set_rf_pwr_setpoint(1500); //21.85-
	  //set_rf_pwr_setpoint(1700); //26
	  //set_rf_pwr_setpoint(2000); //33.6-
	  //set_rf_pwr_setpoint(2200); //37.6
	  //set_rf_pwr_setpoint(2300); //39.8
	  //set_rf_pwr_setpoint(2500); //45.15-
	  //set_rf_pwr_setpoint(2600); //46.1
	  //set_rf_pwr_setpoint(dbm_to_alc(37));

	  rf_pa_en(1);

	  while(1)
	  {
		  //trx_data[CHIP_TX].pwr=63;
		  //trx_writereg(CHIP_TX, 0x002B, trx_data[CHIP_TX].pwr);
		  HAL_Delay(1000);

		  //trx_data[CHIP_TX].pwr=3;
		  //trx_writereg(CHIP_TX, 0x002B, trx_data[CHIP_TX].pwr);
		  HAL_Delay(1000);
	  }
  }

  while(0) //PA test 3
  {
	  rf_pa_en(0);
	  set_rf_pwr_setpoint(0);
	  HAL_Delay(1000);
	  set_rf_pwr_setpoint(1024);
	  HAL_Delay(1000);
	  set_rf_pwr_setpoint(dbm_to_alc(rru_settings.tx_pwr_dbm)); //~2200
	  HAL_Delay(1000);
	  set_rf_pwr_setpoint(4095);
	  HAL_Delay(1000);
  }

  while(0) //temp test
  {
	  float temp=0.0;

	  get_adc_vals(NULL, NULL, &temp);
	  dbg_print(0, "T=%3.1f\n", temp);
	  HAL_Delay(500);
  }

  while(0) //RX test
  {
	  uint8_t val=trx_readreg(CHIP_RX, 0x2F7D);
	  //HAL_UART_Transmit(&huart3, &val, 1, 10);
	  set_dac_ch2((int8_t)val*31+2048);
  }

  while(0) //CC1200 TX output test
  {
	  trx_data[CHIP_TX].fcorr=roundf(-4000.0f/FCORR_STEP);
	  trx_writereg(CHIP_TX, 0x2F0A, (uint16_t)trx_data[CHIP_TX].fcorr>>8);
	  trx_writereg(CHIP_TX, 0x2F0B, (uint16_t)trx_data[CHIP_TX].fcorr&0xFF);

	  while(1)
	  {
		  trx_writecmd(CHIP_TX, STR_STX);
		  set_rf_pwr_setpoint(dbm_to_alc(30));
		  rf_pa_en(1);
		  HAL_Delay(1000);

		  set_rf_pwr_setpoint(0);
		  rf_pa_en(0);
		  trx_writecmd(CHIP_TX, STR_IDLE);
		  HAL_Delay(1000);
	  }
  }

  while(0) //CC1200 PLL test
  {
	  set_rf_pwr_setpoint(dbm_to_alc(47));

	  while(1)
	  {
		  //for(uint32_t f=430e6; f<=440e6; f+=1e6)
		  for(uint32_t f=435e6; f<=440e6; )
		  {
			  rf_pa_en(0);
			  uint32_t freq_word=roundf((float)f/5000000.0*((uint32_t)1<<16));
			  trx_writecmd(CHIP_TX, STR_IDLE);
			  trx_writereg(CHIP_TX, 0x2F0C, (freq_word>>16)&0xFF);
			  trx_writereg(CHIP_TX, 0x2F0D, (freq_word>>8)&0xFF);
			  trx_writereg(CHIP_TX, 0x2F0E, freq_word&0xFF);
			  HAL_Delay(10);
			  trx_writecmd(CHIP_TX, STR_STX);
			  HAL_Delay(10);
			  rf_pa_en(1);
			  //dbg_print(0, "f=%ld sta=%d\n", f, trx_readreg(CHIP_TX, STR_SNOP)>>4);
			  HAL_Delay(1000);
			  //for(int8_t i=-63; i<=63; i++)
			  while(1)
			  {
				  //trx_writereg(CHIP_TX, 0x2F7E, 256-63);
				  //HAL_Delay(10);
				  //trx_writereg(CHIP_TX, 0x2F7E, 63);
				  //HAL_Delay(10);
			  }
		  }
	  }
  }*/

  while(1)
  {
	uart_ctrl_process();
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
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8400-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart1.Init.BaudRate = 460800;
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
  huart3.Init.BaudRate = 460800;
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
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, DBG_TP1_Pin|DBG_TP2_Pin|PA_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RX_nCS_Pin|TX_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DBG_TP1_Pin DBG_TP2_Pin RX_nCS_Pin TX_nCS_Pin
                           PA_EN_Pin */
  GPIO_InitStruct.Pin = DBG_TP1_Pin|DBG_TP2_Pin|RX_nCS_Pin|TX_nCS_Pin
                          |PA_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_TRIG_Pin RX_TRIG_Pin */
  GPIO_InitStruct.Pin = TX_TRIG_Pin|RX_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //immediately disable it, as we don't need it right away
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
  DBG_TP2_GPIO_Port->BSRR=(uint32_t)DBG_TP2_Pin;
  while (1)
  {
	  ;
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
