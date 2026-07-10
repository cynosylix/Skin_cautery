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
#include <string.h>   // For memset()

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
#define VP_LOW_VALUE    0x1000  // DWIN variable address for low value
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
uint16_t address, dataa, mode, high, bipolar, low, icon_id, brightnes,
		current_page, previous_mode;
uint8_t duty;
bool number_eeprome_flag = false;
bool sett_pg_chg = false;
bool eeprom_err = false;
uint32_t err = 0;
bool ch_pg_flag = false;
uint8_t TxData[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x50, 0x02, 0x00, 0x0a };
uint8_t data[2], modee[2], highh[2], bipolarr[2], loww[2], icon_array[2],
		brightness[2], current_page_array[2];
uint16_t prev_address;
uint16_t prev_dataa;
uint16_t previous_low;
// static uint32_t last = 0;
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
uint8_t pagechange[10] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x01 };
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
void send_low_to_display(void);
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

	// NEW ICON MAPPING: Map 0-200 to icon 0-100
	icon_id = low / 2;  // 0-200 maps to 0-100

	icon_array[0] = (icon_id >> 8) & 0xFF;  // high byte
	icon_array[1] = icon_id & 0xFF;         // low byte
	TxData[4] = 0x20;
	TxData[5] = 0x02;
	TxData[6] = icon_array[0];
	TxData[7] = icon_array[1];
	HAL_UART_Transmit(&huart3, TxData, 8, 10);         //to send loadings of low

	// Send current low value to display variable icon at 0x1000
	send_low_to_display();

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

void send_low_to_display(void) {
    // Send current low value to DWIN variable icon at address 0x1000
    uint8_t data[6] = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0x00};
    uint16_t display_value = low;  // Value 0-200

    data[4] = (display_value >> 8) & 0xFF;  // High byte
    data[5] = display_value & 0xFF;         // Low byte

    HAL_UART_Transmit(&huart3, data, 6, 10);
}

void save_reading(void) {
	if (number_eeprome_flag) {
		number_eeprome_flag = false;

		// Handle DWIN variable icon at 0x1000 (Low value control)
		if (address == VP_LOW_VALUE) {
			uint16_t new_low = dataa;
			// Limit the value to 0-200 range
			if (new_low > 200) new_low = 200;

			if (new_low != low) {
				low = new_low;
				loww[0] = (low >> 8) & 0xFF;
				loww[1] = low & 0xFF;

				// Save to EEPROM
				if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
					HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x002A,
					I2C_MEMADD_SIZE_16BIT, loww, 2, 1000);
					HAL_Delay(10);

					// Update PWM immediately if LOW mode is active
					if (mode == 0x4e && ((footswitch) || (active))) {
						lowfunction(low);
					}

					// NEW: Update display loading icon using new mapping (0-200 -> 0-100)
					icon_id = low / 2;  // Map 0-200 to 0-100
					icon_array[0] = (icon_id >> 8) & 0xFF;
					icon_array[1] = icon_id & 0xFF;
					TxData[4] = 0x20;
					TxData[5] = 0x02;
					TxData[6] = icon_array[0];
					TxData[7] = icon_array[1];
					HAL_UART_Transmit(&huart3, TxData, 8, 10);

					// Update display variable icon
					send_low_to_display();
				} else {
					eeprom_err = true;
				}
			}
		}

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
			send_low_to_display();
		}

		previous_mode = mode;
	}

	if (low != previous_low) {
		previous_low = low;
		// NEW: Map 0-200 to icon 0-100
		icon_id = low / 2;
		icon_array[0] = (icon_id >> 8) & 0xFF;  // high byte
		icon_array[1] = icon_id & 0xFF;         // low byte
		TxData[4] = 0x20;
		TxData[5] = 0x02;
		TxData[6] = icon_array[0];
		TxData[7] = icon_array[1];
		HAL_UART_Transmit(&huart3, TxData, 8, 10);

		// Update display variable icon
		send_low_to_display();
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
	default:
		TIM3->CCR1 = 0;
		break;
	}
}

void lowfunction(uint16_t dataa) {
    // Limit input to valid range
    if (dataa > 200) dataa = 200;

    uint32_t pwm_value = 0;

    switch (dataa) {
        // 0-10 range
        case 0:  pwm_value = 0;    break;
        case 1:  pwm_value = 12;   break;
        case 2:  pwm_value = 42;   break;
        case 3:  pwm_value = 69;   break;
        case 4:  pwm_value = 98;   break;
        case 5:  pwm_value = 114;  break;
        case 6:  pwm_value = 139;  break;
        case 7:  pwm_value = 158;  break;
        case 8:  pwm_value = 171;  break;
        case 9:  pwm_value = 181;  break;
        case 10: pwm_value = 196;  break;

        // 11-20 range
        case 11: pwm_value = 207;  break;
        case 12: pwm_value = 215;  break;
        case 13: pwm_value = 226;  break;
        case 14: pwm_value = 226;  break;
        case 15: pwm_value = 239;  break;
        case 16: pwm_value = 249;  break;
        case 17: pwm_value = 262;  break;
        case 18: pwm_value = 271;  break;
        case 19: pwm_value = 271;  break;
        case 20: pwm_value = 279;  break;

        // 21-30 range
        case 21: pwm_value = 298;  break;
        case 22: pwm_value = 298;  break;
        case 23: pwm_value = 309;  break;
        case 24: pwm_value = 319;  break;
        case 25: pwm_value = 319;  break;
        case 26: pwm_value = 332;  break;
        case 27: pwm_value = 332;  break;
        case 28: pwm_value = 341;  break;
        case 29: pwm_value = 341;  break;
        case 30: pwm_value = 352;  break;

        // 31-40 range
        case 31: pwm_value = 363;  break;
        case 32: pwm_value = 363;  break;
        case 33: pwm_value = 375;  break;
        case 34: pwm_value = 375;  break;
        case 35: pwm_value = 385;  break;
        case 36: pwm_value = 385;  break;
        case 37: pwm_value = 396;  break;
        case 38: pwm_value = 396;  break;
        case 39: pwm_value = 405;  break;
        case 40: pwm_value = 408;  break;

        // 41-50 range
        case 41: pwm_value = 419;  break;
        case 42: pwm_value = 419;  break;
        case 43: pwm_value = 428;  break;
        case 44: pwm_value = 428;  break;
        case 45: pwm_value = 438;  break;
        case 46: pwm_value = 438;  break;
        case 47: pwm_value = 438;  break;
        case 48: pwm_value = 454;  break;
        case 49: pwm_value = 454;  break;
        case 50: pwm_value = 463;  break;

        // 51-60 range
        case 51: pwm_value = 462;  break;
        case 52: pwm_value = 475;  break;
        case 53: pwm_value = 475;  break;
        case 54: pwm_value = 475;  break;
        case 55: pwm_value = 482;  break;
        case 56: pwm_value = 488;  break;
        case 57: pwm_value = 501;  break;
        case 58: pwm_value = 501;  break;
        case 59: pwm_value = 501;  break;
        case 60: pwm_value = 502;  break;

        // 61-70 range
        case 61: pwm_value = 512;  break;
        case 62: pwm_value = 524;  break;
        case 63: pwm_value = 524;  break;
        case 64: pwm_value = 524;  break;
        case 65: pwm_value = 537;  break;
        case 66: pwm_value = 537;  break;
        case 67: pwm_value = 547;  break;
        case 68: pwm_value = 547;  break;
        case 69: pwm_value = 547;  break;
        case 70: pwm_value = 557;  break;

        // 71-80 range
        case 71: pwm_value = 557;  break;
        case 72: pwm_value = 557;  break;
        case 73: pwm_value = 568;  break;
        case 74: pwm_value = 568;  break;
        case 75: pwm_value = 568;  break;
        case 76: pwm_value = 577;  break;
        case 77: pwm_value = 577;  break;
        case 78: pwm_value = 577;  break;
        case 79: pwm_value = 589;  break;
        case 80: pwm_value = 589;  break;

        // 81-90 range
        case 81: pwm_value = 589;  break;
        case 82: pwm_value = 598;  break;
        case 83: pwm_value = 598;  break;
        case 84: pwm_value = 598;  break;
        case 85: pwm_value = 611;  break;
        case 86: pwm_value = 611;  break;
        case 87: pwm_value = 611;  break;
        case 88: pwm_value = 621;  break;
        case 89: pwm_value = 621;  break;
        case 90: pwm_value = 621;  break;

        // 91-100 range
        case 91: pwm_value = 633;  break;
        case 92: pwm_value = 633;  break;
        case 93: pwm_value = 633;  break;
        case 94: pwm_value = 647;  break;
        case 95: pwm_value = 647;  break;
        case 96: pwm_value = 647;  break;
        case 97: pwm_value = 658;  break;
        case 98: pwm_value = 658;  break;
        case 99: pwm_value = 671;  break;
        case 100: pwm_value = 671; break;

        // 101-110 range
        case 101: pwm_value = 682; break;
        case 102: pwm_value = 686; break;
        case 103: pwm_value = 690; break;
        case 104: pwm_value = 694; break;
        case 105: pwm_value = 698; break;
        case 106: pwm_value = 700; break;
        case 107: pwm_value = 701; break;
        case 108: pwm_value = 702; break;
        case 109: pwm_value = 702; break;
        case 110: pwm_value = 702; break;

        // 111-120 range
        case 111: pwm_value = 705; break;
        case 112: pwm_value = 708; break;
        case 113: pwm_value = 710; break;
        case 114: pwm_value = 713; break;
        case 115: pwm_value = 716; break;
        case 116: pwm_value = 718; break;
        case 117: pwm_value = 721; break;
        case 118: pwm_value = 723; break;
        case 119: pwm_value = 724; break;
        case 120: pwm_value = 725; break;

        // 121-130 range
        case 121: pwm_value = 730; break;
        case 122: pwm_value = 735; break;
        case 123: pwm_value = 740; break;
        case 124: pwm_value = 745; break;
        case 125: pwm_value = 748; break;
        case 126: pwm_value = 750; break;
        case 127: pwm_value = 753; break;
        case 128: pwm_value = 755; break;
        case 129: pwm_value = 756; break;
        case 130: pwm_value = 757; break;

        // 131-140 range
        case 131: pwm_value = 762; break;
        case 132: pwm_value = 767; break;
        case 133: pwm_value = 772; break;
        case 134: pwm_value = 777; break;
        case 135: pwm_value = 782; break;
        case 136: pwm_value = 785; break;
        case 137: pwm_value = 787; break;
        case 138: pwm_value = 789; break;
        case 139: pwm_value = 790; break;
        case 140: pwm_value = 790; break;

        // 141-150 range
        case 141: pwm_value = 795; break;
        case 142: pwm_value = 798; break;
        case 143: pwm_value = 801; break;
        case 144: pwm_value = 804; break;
        case 145: pwm_value = 807; break;
        case 146: pwm_value = 809; break;
        case 147: pwm_value = 811; break;
        case 148: pwm_value = 812; break;
        case 149: pwm_value = 813; break;
        case 150: pwm_value = 813; break;

        // 151-160 range
        case 151: pwm_value = 818; break;
        case 152: pwm_value = 823; break;
        case 153: pwm_value = 828; break;
        case 154: pwm_value = 833; break;
        case 155: pwm_value = 838; break;
        case 156: pwm_value = 841; break;
        case 157: pwm_value = 844; break;
        case 158: pwm_value = 845; break;
        case 159: pwm_value = 846; break;
        case 160: pwm_value = 847; break;

        // 161-170 range
        case 161: pwm_value = 852; break;
        case 162: pwm_value = 856; break;
        case 163: pwm_value = 860; break;
        case 164: pwm_value = 863; break;
        case 165: pwm_value = 866; break;
        case 166: pwm_value = 868; break;
        case 167: pwm_value = 869; break;
        case 168: pwm_value = 869; break;
        case 169: pwm_value = 869; break;
        case 170: pwm_value = 869; break;

        // 171-180 range
        case 171: pwm_value = 874; break;
        case 172: pwm_value = 879; break;
        case 173: pwm_value = 884; break;
        case 174: pwm_value = 887; break;
        case 175: pwm_value = 890; break;
        case 176: pwm_value = 892; break;
        case 177: pwm_value = 893; break;
        case 178: pwm_value = 894; break;
        case 179: pwm_value = 895; break;
        case 180: pwm_value = 895; break;

        // 181-190 range
        case 181: pwm_value = 900; break;
        case 182: pwm_value = 905; break;
        case 183: pwm_value = 910; break;
        case 184: pwm_value = 915; break;
        case 185: pwm_value = 918; break;
        case 186: pwm_value = 920; break;
        case 187: pwm_value = 923; break;
        case 188: pwm_value = 926; break;
        case 189: pwm_value = 928; break;
        case 190: pwm_value = 930; break;

        // 191-200 range
        case 191: pwm_value = 934; break;
        case 192: pwm_value = 938; break;
        case 193: pwm_value = 941; break;
        case 194: pwm_value = 944; break;
        case 195: pwm_value = 947; break;
        case 196: pwm_value = 949; break;
        case 197: pwm_value = 950; break;
        case 198: pwm_value = 951; break;
        case 199: pwm_value = 952; break;
        case 200: pwm_value = 952; break;

        default: pwm_value = 0; break;
    }

    TIM3->CCR1 = pwm_value;
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
