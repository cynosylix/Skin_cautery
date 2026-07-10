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
uint8_t pagechange[10] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00,
		0x22 };
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
		rxData[rxIndex++] = rxBuffer[0];

		// Continue receiving more data
//        thisis fore correcting and filling the reciving data accept on;y 8byte lessthan 8 will remove
		HAL_UART_Receive_IT(&huart3, rxBuffer, 1);
		if ((rxIndex == 9) || (rxData[5] == 0x4b) || (rxData[6] == 0x4b)) {

			if (rxIndex == 9) {
				RxData[0] = rxData[0];
				RxData[1] = rxData[1];
				RxData[2] = rxData[2];
				RxData[3] = rxData[3];
				RxData[4] = rxData[4];
				RxData[5] = rxData[5];
				RxData[6] = rxData[6];
				RxData[7] = rxData[7];
				RxData[8] = rxData[8];
			}
			rxData[0] = 0x00;
			rxData[1] = 0x00;
			rxData[2] = 0x00;
			rxData[3] = 0x00;
			rxData[4] = 0x00;
			rxData[5] = 0x00;
			rxData[6] = 0x00;
			rxData[7] = 0x00;
			rxData[8] = 0x00;
			rxIndex = 0;

//        		      test++;
		}

	}

}
//---------------------------------------------------------this is a timer 3ch1 interupt function -----------------------------
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//
//		uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //read every rising edge count
//		delta = now - last; //total ticks between two rising edges or 1 wave
//		last = now; //copy last count to find next diffrence
//		frequency = 1000000 / delta; //total ticks per sec divided by our ticks or 1/time period = should me 1/43us we have to covert us into sec .000024 same we added that much zeros in nwumwrator we get 1000000/delta or we can say max ticks in 1s /our ticks
//		// For 24 kHz:
//		// delta should be ~41–42 (with PSC = 71 → 1 µs tick)
//	}
//}

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
		if (current_page == 2) {

			pagechange[9] = 0x02;
			HAL_UART_Transmit(&huart3, pagechange, 10, 10);
		} else {

			pagechange[9] = 0x07;
			HAL_UART_Transmit(&huart3, pagechange, 10, 10);

		}
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

//		hv_adc_reading();

		//--------------------------------------system working-----------------------------------------//
		if (((footswitch) || (active))&&(alert_hv==0)&&(alert_gate==0)) {

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
if((alert_hv!=1)&&(alert_gate!=1))
{
				if ((mode == 0x4f) || (mode == 0x4d)) {
					pagechange[9] = 0x01;
					HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				} else {
					if (current_page == 2) {
						current_page = 2;
						pagechange[9] = 0x02;
						HAL_UART_Transmit(&huart3, pagechange, 10, 10);
					} else {
						current_page = 7;
						pagechange[9] = 0x07;
						HAL_UART_Transmit(&huart3, pagechange, 10, 10);

					}
				}
			}
else if(alert_hv==1)
{
	pagechange[9] = 0x08;
	HAL_UART_Transmit(&huart3, pagechange, 10, 10);
}
else if(alert_gate==1)
{
	pagechange[9] = 0x09;
	HAL_UART_Transmit(&huart3, pagechange, 10, 10);
}
			}

		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 44;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, biprly_Pin|highrly_Pin|lowrly_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(hv_en_GPIO_Port, hv_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : biprly_Pin highrly_Pin lowrly_Pin */
  GPIO_InitStruct.Pin = biprly_Pin|highrly_Pin|lowrly_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : hv_en_Pin */
  GPIO_InitStruct.Pin = hv_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hv_en_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
//				test++;
				if ((mode == 0x4f) || (mode == 0x4d)) {
					pagechange[9] = 0x01;
					HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				} else if (mode == 0x4e) {
					if (current_page == 2) {
						current_page = 2;
						pagechange[9] = 0x02;
						HAL_UART_Transmit(&huart3, pagechange, 10, 10);
					} else {
						current_page = 7;
						pagechange[9] = 0x07;
						HAL_UART_Transmit(&huart3, pagechange, 10, 10);

					}
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
			if (current_page == 2) {
				current_page = 2;
				pagechange[9] = 0x02;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);
			} else {
				current_page = 7;
				pagechange[9] = 0x07;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);

			}
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
		if (low == 100) {
			if (current_page == 2) {
//				test++;
				pagechange[9] = 0x07;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				current_page = 7;
				current_page_array[0] = 0x00;
				current_page_array[1] = 0x07;
				if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
					HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0200,
					I2C_MEMADD_SIZE_16BIT, current_page_array, 2, 1000);
					HAL_Delay(10);
				} else {
					eeprom_err = true;
				}
			} else if (current_page == 7) {
				pagechange[9] = 0x02;
				HAL_UART_Transmit(&huart3, pagechange, 10, 10);
				current_page = 2;
				current_page_array[0] = 0x00;
				current_page_array[1] = 0x02;
				if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
					HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0200,
					I2C_MEMADD_SIZE_16BIT, current_page_array, 2, 1000);
					HAL_Delay(10);
				} else {
					eeprom_err = true;
				}
			}
		}

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
		if ((HAL_GetTick() - pa7_last_irq) >= DEBOUNCE_MS) {
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
        if (ch0_voltage > HIGH_THRESHOLD_CH0)
        {
            alert_gate = 1;
        }
        else if (ch0_voltage < LOW_THRESHOLD_CH0)
        {
        	alert_gate = 1;
        }
        if (ch1_voltage > HIGH_THRESHOLD_CH1)
        {
            alert_hv = 1;
        }
        else if (ch1_voltage < LOW_THRESHOLD_CH1)
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
//	test++;
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
