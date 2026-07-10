/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM3_ARR 2999
#define INPUT_MIN_HIBI  0x00//lower limit of pwm wave gen hv_pwm
#define INPUT_MAX_HIBI 0x23//higher limit of pwm wave gen hv_pwm
#define INPUT_MIN_LOW  0x00//lower limit of pwm wave gen hv_pwm
#define INPUT_MAX_LOW  0xc8//higher limit of pwm wave gen hv_pwm
#define DEBOUNCE_MS 20   // safe for most mechanical buttons
#define HIGH_THRESHOLD_CH0 1.7f
#define LOW_THRESHOLD_CH0  0.0f
#define HIGH_THRESHOLD_CH1 3.2f
#define LOW_THRESHOLD_CH1  0.0f

#define ADC_MAX_VALUE   4095.0f
#define VDDA_VOLTAGE    3.3f    // change ONLY if VDDA is different
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int count;
int test;
//uint32_t ccr = 0;
uint8_t RxData[9];
uint8_t rxBuffer[9];
uint8_t rxData[9];
uint16_t rxIndex = 0;
uint16_t address, dataa, mode, high, bipolar, low, decimal_value, brightnes,
		current_page, previous_mode;
uint8_t duty;
bool number_eeprome_flag = false;
bool sett_pg_chg = false;
bool eeprom_err = false;
uint32_t err = 0;
bool ch_pg_flag = false;
uint8_t TxData[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x50, 0x02, 0x00, 0x0a };
uint8_t data[2], modee[2], highh[2], bipolarr[2], loww[2], low_array[2],
		brightness[2], current_page_array[2];
uint16_t prev_address;
uint16_t prev_dataa;
uint16_t previous_low;

static uint32_t last = 0;
uint32_t delta = 0;
float frequency = 0.0;
volatile uint8_t footswitch = 0;
volatile uint8_t active = 0;
volatile uint32_t pa7_last_irq = 0;
volatile uint8_t pa7_pending = 0;
//volatile uint8_t alert_hv = 0;
volatile uint32_t pb0_last_irq = 0;
volatile uint8_t pb0_pending = 0;
//uint32_t last_adc_tick = 0;
//uint16_t adc_raw = 0;
uint8_t pagechange[10] = { 0x5A, 0xA5, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01 };
uint8_t mapped_duty = 0;

uint16_t adc_dma_buf[2];
uint16_t adc_raw_ch1;
uint16_t adc_raw_ch0;
uint32_t last_adc_tick = 0;
uint8_t alert_hv = 0;
uint8_t alert_gate = 0;

float ch1_voltage;
float ch0_voltage=1.6f;
//uint16_t addr = 0x0100;   // not 0100
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void bi_hi_pwm(uint16_t dataa);
void lowfunction(uint16_t dataa);
void TIM3_SetDutyPercent(uint8_t duty);
void save_reading(void);
void send_loadings(void);
void debouncing(void);
void hv_adc_reading(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		// Append the received byte to the buffer
		if (rxIndex < sizeof(rxData)) {  // Prevent buffer overflow
			rxData[rxIndex++] = rxBuffer[0];
		}

		// Continue receiving more data
		HAL_UART_Receive_IT(&huart3, rxBuffer, 1);
		if ((rxIndex == 9) || (rxData[5] == 0x4b) || (rxData[6] == 0x4b)) {

			if (rxIndex == 9) {
				for(int i = 0; i < 9; i++) {
					RxData[i] = rxData[i];
				}
			}
			memset(rxData, 0, sizeof(rxData));
			rxIndex = 0;
		}
	}
}

//-----------------------------------------------------interrupt reading if button is pressed-------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_7) {
		pa7_last_irq = HAL_GetTick();  // timestamp
		pa7_pending = 1;              // mark for debounce
	}

	if (GPIO_Pin == GPIO_PIN_0) {
		pb0_last_irq = HAL_GetTick();  // timestamp
		pb0_pending = 1;              // mark for debounce
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1); // MUST be before DMA start
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, 2);

	HAL_Delay(3000);
	pagechange[9] = 0x01;
	HAL_UART_Transmit(&huart3, pagechange, 10, 10);
	HAL_UART_Receive_IT(&huart3, rxBuffer, 1);
//	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Variable PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Connstant PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //audio
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 300); //70%
//-----------------------------------------------------for  read eeprome value and transmit to display and store ram---------------------------------
	HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x0200, I2C_MEMADD_SIZE_16BIT,
			current_page_array, 2, 1000); //retrive corrent page of low page(0-10)page2,(10-20)page7 value
	HAL_Delay(10);
	current_page = (current_page_array[0] << 8) | current_page_array[1]; //currentpage number
	// Ensure current_page is always 2 for LOW mode (page 07 removed)
	if(current_page != 2) {
		current_page = 2;
	}
//---------------------------------------------------------------------
	HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x000A, I2C_MEMADD_SIZE_16BIT, modee, 2,
			1000); //retrive modes value
	HAL_Delay(10);
	mode = (modee[0] << 8) | modee[1];
	TxData[4] = 0x20;
	TxData[5] = 0x00;
	TxData[6] = modee[0];
	TxData[7] = modee[1];
	HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send corrent mode
//-----------------------------------------------------------------------------
	HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x0014, I2C_MEMADD_SIZE_16BIT, highh, 2,
			1000); //retrive high value
	HAL_Delay(10);
	high = (highh[0] << 8) | highh[1];
//-------------------------------------------------------------------------------
	HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x0018, I2C_MEMADD_SIZE_16BIT, bipolarr,
			2, 1000); //retrive bipolar value
	HAL_Delay(10);
	bipolar = (bipolarr[0] << 8) | bipolarr[1];
//----------------------------------------------perform correspondng data uploadings depending on modes .both modes share same variable location---------------------------------
	if (mode == 0x4f) {
		TxData[4] = 0x30;
		TxData[5] = 0x03;
		TxData[6] = highh[0];
		TxData[7] = highh[1];
		HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send high mode's variable data

		TxData[4] = 0x20;
		TxData[5] = 0x01;
		TxData[6] = highh[0];
		TxData[7] = highh[1];
		HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send loading of high mode
	} else if (mode == 0x4d) {
		TxData[4] = 0x30;
		TxData[5] = 0x03;
		TxData[6] = bipolarr[0];
		TxData[7] = bipolarr[1];
		HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send bipolar mode's variable data

		TxData[4] = 0x20;
		TxData[5] = 0x01;
		TxData[6] = bipolarr[0];
		TxData[7] = bipolarr[1];
		HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send loading of bipolar mode
	}
//--------------------------------------------------------------------------------------
	HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x002A, I2C_MEMADD_SIZE_16BIT, loww, 2,
			1000);
	HAL_Delay(10);
	low = (loww[0] << 8) | loww[1];
	TxData[4] = 0x30;
	TxData[5] = 0x02;
	TxData[6] = loww[0];
	TxData[7] = loww[1];
	HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send low mode's variable data

	decimal_value = (low / 10) + (39); //+39 is used to match same gui image number we add loading animation from39-41

	low_array[0] = (decimal_value >> 8) & 0xFF;  // high byte
	low_array[1] = decimal_value & 0xFF;         // low byte
	TxData[4] = 0x20;
	TxData[5] = 0x02;
	TxData[6] = low_array[0];
	TxData[7] = low_array[1];
	HAL_UART_Transmit(&huart3, TxData, 8, 10);         //to send loadings of low

	HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x0100, I2C_MEMADD_SIZE_16BIT,
			brightness, 2, 1000);
	brightnes = (brightness[0] << 8) | brightness[1];
	TxData[4] = 0x00;
	TxData[5] = 0x82;
	TxData[6] = brightness[0];
	TxData[7] = brightness[1];

	HAL_UART_Transmit(&huart3, TxData, 8, 10);
//---------------------------------------------------------------------------------------------------------------------------------------------------

	//to send last saved page
	if ((mode == 0x4f) || (mode == 0x4d)) {
		pagechange[9] = 0x01;
		HAL_UART_Transmit(&huart3, pagechange, 10, 10);
	} else {
		// LOW mode always uses page 02
		pagechange[9] = 0x02;
		HAL_UART_Transmit(&huart3, pagechange, 10, 10);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_UART_Receive_IT(&huart3, rxBuffer, 1); //to read uart display interupt continuously
		// Combine RxData[4] and RxData[5] into a 16-bit address
		address = (RxData[4] << 8) | RxData[5]; //copy adress from recived display data

		// Combine RxData[8] and RxData[9] into a 16-bit data
		dataa = (RxData[7] << 8) | RxData[8]; //copy data from recived display data
		if ((address != prev_address) || (dataa != prev_dataa)) //to detect data or adress is changed by touching humans?
				{
			number_eeprome_flag = true; //just raise a flag if touch happens othervise continue
//we cant able to add this in interrupt function of display so we need to taje this from loop
			prev_address = address; //to detect change remember past value
			prev_dataa = dataa; //to detect change remember past value
		}

//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, dataa);

		count++; //counter to understand is this device is in running?

		save_reading(); //to save recived datas to corresponding spaces
		send_loadings(); //if any data is changed we have to update gui a loading icon

		debouncing(); //debouncing logic

		hv_adc_reading();

		//--------------------------------------system working-----------------------------------------//
		if (((footswitch) || (active)) && (alert_hv == 0) && (alert_gate == 0)) {

			ch_pg_flag = true;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //70%pwm
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //audio

			HAL_GPIO_WritePin(hv_en_GPIO_Port, hv_en_Pin, GPIO_PIN_RESET);
			switch (mode) {
			case 0x4f:
				bi_hi_pwm(high);
				HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_RESET);
				pagechange[9] = 0x05;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);

				break;
			case 0x4e:
				lowfunction(low);
				HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_SET);
				pagechange[9] = 0x06;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				break;
			case 0x4d:
				bi_hi_pwm(bipolar);
				HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_SET);
				pagechange[9] = 0x04;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				break;
			}
			hv_adc_reading();

		} else {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); //audio
			HAL_GPIO_WritePin(hv_en_GPIO_Port, hv_en_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_RESET);

			//-----------------------------teturn from dettings-------------------------------/
			if (ch_pg_flag) {
				ch_pg_flag = false;
				if((alert_hv != 1) && (alert_gate != 1)) {
					if ((mode == 0x4f) || (mode == 0x4d)) {
						pagechange[9] = 0x01;
						HAL_UART_Transmit(&huart3, pagechange, 10, 10);
					} else {
						// LOW mode always uses page 02
						current_page = 2;
						pagechange[9] = 0x02;
						HAL_UART_Transmit(&huart3, pagechange, 10, 10);
					}
				} else if(alert_hv == 1) {
					pagechange[9] = 0x08;
					HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				} else if(alert_gate == 1) {
					pagechange[9] = 0x09;
					HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				}
			}
		}
	}
  /* USER CODE END 3 */
}

// ... (rest of the initialization functions remain the same)

/* USER CODE BEGIN 4 */

void save_reading(void) {
	if (number_eeprome_flag) {

		number_eeprome_flag = false;
		switch (address) {
		case 0x2000:  //engineers screen password location

			mode = dataa;
			modee[0] = RxData[7];
			modee[1] = RxData[8];
			if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
				HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x000A,
				I2C_MEMADD_SIZE_16BIT, modee, 2, 1000);
				HAL_Delay(10);
			} else {
				eeprom_err = true;
			}

			break;
		case 0x3003:  //engineers screen password location
			if (mode == 0x4f)  //same 3003 variable data guided to high variable
					{
				high = dataa;
				highh[0] = RxData[7];
				highh[1] = RxData[8];
				if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
					HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0014,
					I2C_MEMADD_SIZE_16BIT, highh, 2, 1000);
					HAL_Delay(10);
				} else {
					eeprom_err = true;
				}
			}
			if (mode == 0x4d) //same 3003 variable data guided to bipolar variable
					{
				bipolar = dataa;
				bipolarr[0] = RxData[7];
				bipolarr[1] = RxData[8];
				if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
					HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0018,
					I2C_MEMADD_SIZE_16BIT, bipolarr, 2, 1000);
					HAL_Delay(10);
				} else {
					eeprom_err = true;
				}
			}
			break;
		case 0x3002:

			low = dataa;
			loww[0] = RxData[7];
			loww[1] = RxData[8];

			if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
				HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x002A,
				I2C_MEMADD_SIZE_16BIT, loww, 2, 1000);
				HAL_Delay(10);
			} else {
				eeprom_err = true;
			}

			break;
		case 0x82:

			test++;

			brightnes = dataa;
			brightness[0] = RxData[7];
			brightness[1] = RxData[8];
			if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
				HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0100,
				I2C_MEMADD_SIZE_16BIT, brightness, 2, 1000);
				HAL_Delay(10);
			} else {
				eeprom_err = true;
			}

			break;

		case 0x2001:

			if (dataa == 0x01) {
				if ((mode == 0x4f) || (mode == 0x4d)) {
					pagechange[9] = 0x01;
					HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				} else if (mode == 0x4e) {
					// LOW mode always uses page 02
					current_page = 2;
					pagechange[9] = 0x02;
					HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				}
			}
			break;
		}
	}
}

void send_loadings(void) {
	if (mode != previous_mode) {
		if (mode == 0x4f) {
			TxData[4] = 0x30;
			TxData[5] = 0x03;
			TxData[6] = highh[0];
			TxData[7] = highh[1];
			HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send high mode's variable data

			TxData[4] = 0x20;
			TxData[5] = 0x01;
			TxData[6] = highh[0];
			TxData[7] = highh[1];
			HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send loading of high mode
		} else if (mode == 0x4d) {
			TxData[4] = 0x30;
			TxData[5] = 0x03;
			TxData[6] = bipolarr[0];
			TxData[7] = bipolarr[1];
			HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send bipolar mode's variable data

			TxData[4] = 0x20;
			TxData[5] = 0x01;
			TxData[6] = bipolarr[0];
			TxData[7] = bipolarr[1];
			HAL_UART_Transmit(&huart3, TxData, 8, 10); //to send loading of bipolar mode
		} else if (mode == 0x4e) {
			// LOW mode always uses page 02
			current_page = 2;
			pagechange[9] = 0x02;
			HAL_UART_Transmit(&huart3, pagechange, 10, 10);
		}

		previous_mode = mode;
	}

	if (low != previous_low) {
		previous_low = low;
		decimal_value = (low / 10) + (39); //+39 is used to match same gui image number we add loading animation from39-41
		low_array[0] = (decimal_value >> 8) & 0xFF;  // high byte
		low_array[1] = decimal_value & 0xFF;         // low byte
		TxData[4] = 0x20;
		TxData[5] = 0x02;
		TxData[6] = low_array[0];
		TxData[7] = low_array[1];
		HAL_UART_Transmit(&huart3, TxData, 8, 10);

		// Removed page 07 switching - LOW mode stays on page 02
	}
}

void debouncing(void) {
	if (pa7_pending) //only enter if interupt is raised and no debounce limit has reached
	{
		if ((HAL_GetTick() - pa7_last_irq) >= DEBOUNCE_MS) //only eneter debounce limit is reached
		{
			pa7_pending = 0;         //clear interupt raised flag

			// Read stable pin state AFTER bounce settled
			footswitch =
					(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);
		}
	}
	if (pb0_pending) {
		if ((HAL_GetTick() - pb0_last_irq) >= DEBOUNCE_MS) {  // FIXED: use pb0_last_irq instead of pa7_last_irq
			pb0_pending = 0;

			// Read stable pin state AFTER bounce settled
			active = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
		}
	}
}

void hv_adc_reading(void)
{
    if ((HAL_GetTick() - last_adc_tick) >= 10)
    {
        last_adc_tick = HAL_GetTick();

         adc_raw_ch1 = adc_dma_buf[0];  // Rank 1 → ADC_CHANNEL_1
         adc_raw_ch0 = adc_dma_buf[1];  // Rank 2 → ADC_CHANNEL_0

         ch1_voltage = (adc_raw_ch1 * VDDA_VOLTAGE) / ADC_MAX_VALUE;
         ch0_voltage = (adc_raw_ch0 * VDDA_VOLTAGE) / ADC_MAX_VALUE;

        // Clear alerts first, then check conditions
        alert_gate = 0;
        alert_hv = 0;

        if (ch0_voltage > HIGH_THRESHOLD_CH0 || ch0_voltage < LOW_THRESHOLD_CH0)
        {
            alert_gate = 1;
        }
        if (ch1_voltage > HIGH_THRESHOLD_CH1 || ch1_voltage < LOW_THRESHOLD_CH1)
        {
            alert_hv = 1;
        }
    }
}

void bi_hi_pwm(uint16_t dataa) {
	switch (dataa) {
	case 0:
		TIM3->CCR1 = 0;
		break;
	case 1:
		TIM3->CCR1 = 102;
		break;
	case 2:
		TIM3->CCR1 = 186;
		break;
	case 3:
		TIM3->CCR1 = 246;
		break;
	case 4:
		TIM3->CCR1 = 307;
		break;
	case 5:
		TIM3->CCR1 = 353;
		break;
	case 6:
		TIM3->CCR1 = 384;
		break;
	case 7:
		TIM3->CCR1 = 423;
		break;
	case 8:
		TIM3->CCR1 = 458;
		break;
	case 9:
		TIM3->CCR1 = 496;
		break;
	case 10:
		TIM3->CCR1 = 524;
		break;
	case 11:
		TIM3->CCR1 = 544;
		break;
	case 12:
		TIM3->CCR1 = 583;
		break;
	case 13:
		TIM3->CCR1 = 609;
		break;
	case 14:
		TIM3->CCR1 = 630;
		break;
	case 15:
		TIM3->CCR1 = 655;
		break;
	case 16:
		TIM3->CCR1 = 673;
		break;
	case 17:
		TIM3->CCR1 = 697;
		break;
	case 18:
		TIM3->CCR1 = 717;
		break;
	case 19:
		TIM3->CCR1 = 742;
		break;
	case 20:
		TIM3->CCR1 = 754;
		break;
	case 21:
		TIM3->CCR1 = 781;
		break;
	case 22:
		TIM3->CCR1 = 803;
		break;
	case 23:
		TIM3->CCR1 = 812;
		break;
	case 24:
		TIM3->CCR1 = 836;
		break;
	case 25:
		TIM3->CCR1 = 856;
		break;
	case 26:
		TIM3->CCR1 = 867;
		break;
	case 27:
		TIM3->CCR1 = 889;
		break;
	case 28:
		TIM3->CCR1 = 901;
		break;
	case 29:
		TIM3->CCR1 = 930;
		break;
	case 30:
		TIM3->CCR1 = 948;
		break;
	case 31:
		TIM3->CCR1 = 960;
		break;
	case 32:
		TIM3->CCR1 = 981;
		break;
	case 33:
		TIM3->CCR1 = 989;
		break;
	case 34:
		TIM3->CCR1 = 998;
		break;
	case 35:
		TIM3->CCR1 = 998;
		break;
	}
}

void lowfunction(uint16_t dataa) {
	switch (dataa) {
	case 0:
		TIM3->CCR1 = 0;
		break;
	case 1:
		TIM3->CCR1 = 12;
		break;
	case 2:
		TIM3->CCR1 = 42;
		break;
	case 3:
		TIM3->CCR1 = 69;
		break;
	case 4:
		TIM3->CCR1 = 98;
		break;
	case 5:
		TIM3->CCR1 = 114;
		break;
	case 6:
		TIM3->CCR1 = 139;
		break;
	case 7:
		TIM3->CCR1 = 158;
		break;
	case 8:
		TIM3->CCR1 = 171;
		break;
	case 9:
		TIM3->CCR1 = 181;
		break;
	case 10:
		TIM3->CCR1 = 196;
		break;
	case 11:
		TIM3->CCR1 = 207;
		break;
	case 12:
		TIM3->CCR1 = 215;
		break;
	case 13:
		TIM3->CCR1 = 226;
		break;
	case 14:
		TIM3->CCR1 = 226;
		break;
	case 15:
		TIM3->CCR1 = 239;
		break;
	case 16:
		TIM3->CCR1 = 249;
		break;
	case 17:
		TIM3->CCR1 = 262;
		break;
	case 18:
		TIM3->CCR1 = 271;
		break;
	case 19:
		TIM3->CCR1 = 271;
		break;
	case 20:
		TIM3->CCR1 = 279;
		break;
	case 21:
		TIM3->CCR1 = 298;
		break;
	case 22:
		TIM3->CCR1 = 298;
		break;
	case 23:
		TIM3->CCR1 = 309;
		break;
	case 24:
		TIM3->CCR1 = 319;
		break;
	case 25:
		TIM3->CCR1 = 319;
		break;
	case 26:
		TIM3->CCR1 = 332;
		break;
	case 27:
		TIM3->CCR1 = 332;
		break;
	case 28:
		TIM3->CCR1 = 341;
		break;
	case 29:
		TIM3->CCR1 = 341;
		break;
	case 30:
		TIM3->CCR1 = 352;
		break;
	case 31:
		TIM3->CCR1 = 363;
		break;
	case 32:
		TIM3->CCR1 = 363;
		break;
	case 33:
		TIM3->CCR1 = 375;
		break;
	case 34:
		TIM3->CCR1 = 375;
		break;
	case 35:
		TIM3->CCR1 = 385;
		break;
	case 36:
		TIM3->CCR1 = 385;
		break;
	case 37:
		TIM3->CCR1 = 396;
		break;
	case 38:
		TIM3->CCR1 = 396;
		break;
	case 39:
		TIM3->CCR1 = 405;
		break;
	case 40:
		TIM3->CCR1 = 408;
		break;
	case 41:
		TIM3->CCR1 = 419;
		break;
	case 42:
		TIM3->CCR1 = 419;
		break;
	case 43:
		TIM3->CCR1 = 428;
		break;
	case 44:
		TIM3->CCR1 = 428;
		break;
	case 45:
		TIM3->CCR1 = 438;
		break;
	case 46:
		TIM3->CCR1 = 438;
		break;
	case 47:
		TIM3->CCR1 = 438;
		break;
	case 48:
		TIM3->CCR1 = 454;
		break;
	case 49:
		TIM3->CCR1 = 454;
		break;
	case 50:
		TIM3->CCR1 = 463;
		break;
	case 51:
		TIM3->CCR1 = 462;
		break;
	case 52:
		TIM3->CCR1 = 475;
		break;
	case 53:
		TIM3->CCR1 = 475;
		break;
	case 54:
		TIM3->CCR1 = 475;
		break;
	case 55:
		TIM3->CCR1 = 482;
		break;
	case 56:
		TIM3->CCR1 = 488;
		break;
	case 57:
		TIM3->CCR1 = 501;
		break;
	case 58:
		TIM3->CCR1 = 501;
		break;
	case 59:
		TIM3->CCR1 = 501;
		break;
	case 60:
		TIM3->CCR1 = 502;
		break;
	case 61:
		TIM3->CCR1 = 512;
		break;
	case 62:
		TIM3->CCR1 = 524;
		break;
	case 63:
		TIM3->CCR1 = 524;
		break;
	case 64:
		TIM3->CCR1 = 524;
		break;
	case 65:
		TIM3->CCR1 = 537;
		break;
	case 66:
		TIM3->CCR1 = 537;
		break;
	case 67:
		TIM3->CCR1 = 547;
		break;
	case 68:
		TIM3->CCR1 = 547;
		break;
	case 69:
		TIM3->CCR1 = 547;
		break;
	case 70:
		TIM3->CCR1 = 557;
		break;
	case 71:
		TIM3->CCR1 = 557;
		break;
	case 72:
		TIM3->CCR1 = 557;
		break;
	case 73:
		TIM3->CCR1 = 568;
		break;
	case 74:
		TIM3->CCR1 = 568;
		break;
	case 75:
		TIM3->CCR1 = 568;
		break;
	case 76:
		TIM3->CCR1 = 577;
		break;
	case 77:
		TIM3->CCR1 = 577;
		break;
	case 78:
		TIM3->CCR1 = 577;
		break;
	case 79:
		TIM3->CCR1 = 589;
		break;
	case 80:
		TIM3->CCR1 = 589;
		break;
	case 81:
		TIM3->CCR1 = 589;
		break;
	case 82:
		TIM3->CCR1 = 598;
		break;
	case 83:
		TIM3->CCR1 = 598;
		break;
	case 84:
		TIM3->CCR1 = 598;
		break;
	case 85:
		TIM3->CCR1 = 611;
		break;
	case 86:
		TIM3->CCR1 = 611;
		break;
	case 87:
		TIM3->CCR1 = 611;
		break;
	case 88:
		TIM3->CCR1 = 621;
		break;
	case 89:
		TIM3->CCR1 = 621;
		break;
	case 90:
		TIM3->CCR1 = 621;
		break;
	case 91:
		TIM3->CCR1 = 633;
		break;
	case 92:
		TIM3->CCR1 = 633;
		break;
	case 93:
		TIM3->CCR1 = 633;
		break;
	case 94:
		TIM3->CCR1 = 647;
		break;
	case 95:
		TIM3->CCR1 = 647;
		break;
	case 96:
		TIM3->CCR1 = 647;
		break;
	case 97:
		TIM3->CCR1 = 658;
		break;
	case 98:
		TIM3->CCR1 = 658;
		break;
	case 99:
		TIM3->CCR1 = 671;
		break;
	case 100:
		TIM3->CCR1 = 671;
		break;
	case 110:
		TIM3->CCR1 = 702;
		break;
	case 120:
		TIM3->CCR1 = 725;
		break;
	case 130:
		TIM3->CCR1 = 757;
		break;
	case 140:
		TIM3->CCR1 = 790;
		break;
	case 150:
		TIM3->CCR1 = 813;
		break;
	case 160:
		TIM3->CCR1 = 847;
		break;
	case 170:
		TIM3->CCR1 = 869;
		break;
	case 180:
		TIM3->CCR1 = 895;
		break;
	case 190:
		TIM3->CCR1 = 930;
		break;
	case 200:
		TIM3->CCR1 = 952;
		break;
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
	__disable_irq();
	while (1) {
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
