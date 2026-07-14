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
#define INPUT_MIN_HIBI  0x00
#define INPUT_MAX_HIBI 0x23
#define INPUT_MIN_LOW  0x00
#define INPUT_MAX_LOW  0xc8
#define DEBOUNCE_MS 20
#define HIGH_THRESHOLD_CH0 1.7f
#define LOW_THRESHOLD_CH0  0.0f
#define HIGH_THRESHOLD_CH1 3.2f
#define LOW_THRESHOLD_CH1  0.0f

#define ADC_MAX_VALUE   4095.0f
#define VDDA_VOLTAGE    3.3f
#define VP_LOW_VALUE         0x1000  // DWIN variable address for low value
#define VP_LOW_DATA          0x3002  // DWIN variable address for low numeric display
#define VP_VAR_ICON          0x2000  // DWIN variable address for variable icon
#define VP_MODE              0x3000  // DWIN variable address for mode (was wrongly 0x2000)
#define VP_MODE_DATA         0x3003  // DWIN variable address (high/bipolar/low page value)
#define EEPROM_ADDR_LOW      0x002A  // EEPROM storage for VP 0x1000 low value
#define EEPROM_ADDR_VP3003   0x002C  // EEPROM storage for VP 0x3003
#define EEPROM_ADDR_PAGE     0x002E  // EEPROM storage for last DWIN page (1 or 2)
#define EEPROM_PAGE_MAGIC    0xA5    // Valid page record marker byte
#define EEPROM_ADDR_PAGE_OLD 0x0200  // Legacy page address (migration read)
#define EEPROM_ADDR_VAR_ICON 0x0030  // EEPROM storage for VP 0x2000 var icon
#define EEPROM_ADDR_VAR_ICON_OLD 0x0300  // Legacy icon address (migration read)
#define EEPROM_ICON_MAGIC    0xA5    // Valid icon record marker byte
#define LOW_VALUE_MAX        200     // Maximum low value (INPUT_MAX_LOW)
#define BOOT_VAR_ICON_VALUE  78      // Default icon (low mode)
#define MIN_VAR_ICON_VALUE   77      // Bipolar icon
#define VAR_ICON_KEY_MAX     79      // High icon
#define ICON_BIPOLAR         77      // Return key 0x004D
#define ICON_LOW             78      // Return key 0x004E
#define ICON_HIGH            79      // Return key 0x004F
#define MODE_KEY_HIGH        0x004F  // VP 0x2000 return key — high mode
#define MODE_KEY_LOW         0x004E  // VP 0x2000 return key — low mode
#define MODE_KEY_BIPOLAR     0x004D  // VP 0x2000 return key — bipolar mode
#define DEFAULT_BRIGHTNESS   100     // Boot default if EEPROM empty (0-100)
#define MIN_BRIGHTNESS       30      // VP 0x0082 minimum — never go below this
#define VP_BRIGHTNESS        0x0082  // DWIN backlight variable
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
uint8_t RxData[12];
uint8_t rxBuffer[12];
uint8_t rxData[12];
volatile uint16_t rxIndex = 0;
static uint8_t uart_rx_started = 0;
uint16_t address, dataa, mode, high, bipolar, low, icon_id, brightnes,
		current_page, previous_mode;
uint8_t duty;
bool number_eeprome_flag = false;
bool sett_pg_chg = false;
bool eeprom_err = false;
uint32_t err = 0;
bool ch_pg_flag = false;
uint8_t TxData[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x50, 0x02, 0x00, 0x0a };
uint8_t data[2], modee[2], highh[2], bipolarr[2], loww[2], vp3003w[2], icon_array[2],
		brightness[2], current_page_array[2];
uint16_t vp3003_val = 0;
uint16_t prev_address;
uint16_t prev_dataa;
uint16_t previous_low = 0;
volatile uint8_t dwin_pkt_ready = 0;
static uint32_t low_save_enable_tick = 0;
static uint32_t low_display_push_until = 0;
static uint32_t var_icon_display_push_until = 0;
static uint32_t var_icon_boot_push_until = 0;
static uint32_t var_icon_aggressive_until = 0;
static uint32_t var_icon_save_enable_tick = 0;
static uint32_t brightness_display_push_until = 0;
static uint32_t brightness_save_enable_tick = 0;
static uint8_t brightness_user_touched = 0;
static uint8_t low_user_touched = 0;
static uint32_t low_last_accept_tick = 0;
static uint32_t vp3003_save_enable_tick = 0;
static uint32_t vp3003_page1_restore_until = 0;
static uint8_t vp3003_user_touched = 0;
static uint8_t saved_settings_page = 0;
static uint8_t var_icon_user_locked = 0;
static uint32_t boot_page_guard_until = 0;
uint32_t delta = 0;
float frequency = 0.0;
volatile uint8_t footswitch = 0;
volatile uint8_t active = 0;
volatile uint32_t pa7_last_irq = 0;
volatile uint8_t pa7_pending = 0;
volatile uint32_t pb0_last_irq = 0;
volatile uint8_t pb0_pending = 0;
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

// Variable icon storage (VP 0x2000)
uint16_t var_icon_value = BOOT_VAR_ICON_VALUE;
uint8_t var_icon_data[2] = {0x00, 0x4E};  // 78 = 0x004E low icon
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
void send_page_change(uint8_t page_num);
void dwin_boot_to_page(uint8_t page_num);
void dwin_uart_rx_start(void);
void low_value_user_changed(uint16_t new_low);
void send_variable_data(uint8_t addr_high, uint8_t addr_low, uint8_t data_high, uint8_t data_low);
void dwin_set_var_icon(uint16_t icon_id);
void send_var_icon_to_display(void);
void dwin_apply_boot_var_icon(void);
void eeprom_read_low_value(void);
void eeprom_save_low_value(void);
void eeprom_read_brightness(void);
void dwin_send_brightness(void);
void dwin_refresh_brightness(void);
void low_sync_both_vps(void);
void low_push_display_from_eeprom(void);
void eeprom_read_vp3003_value(void);
void eeprom_save_vp3003_value(void);
void eeprom_read_var_icon(void);
void eeprom_save_var_icon(void);
void eeprom_save_current_page(uint8_t page_num);
static void dwin_restore_page_from_eeprom(uint8_t page_num);
static void dwin_show_settings_page(uint8_t page_num);
static void dwin_arm_page2_restore_timers(void);
static void dwin_arm_var_icon_timers(void);
static void dwin_restore_var_icon_from_eeprom(uint8_t burst_count);
static void dwin_push_icon_from_eeprom(void);
static void dwin_icon_burst_from_eeprom(uint8_t count);
static uint8_t var_icon_from_mode_key(uint16_t key);
static uint8_t var_icon_from_mode(uint16_t m);
static void var_icon_apply_return_key(uint16_t key);
static uint8_t dwin_get_boot_page(void);
void dwin_restore_saved_state(uint8_t page_num);
void vp3003_send_to_display(void);
void vp3003_user_changed(uint16_t new_val);
void dwin_refresh_page2_values(void);
void dwin_open_page2(void);
void dwin_refresh_page1_values(void);
void dwin_open_page1(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t i2c_eeprom_read16(uint16_t addr, uint8_t *buf);
static void send_var_icon_fast(void);
static void dwin_icon_burst_fast(uint8_t count);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (rxIndex < sizeof(rxData)) {
			rxData[rxIndex++] = rxBuffer[0];
		}

		HAL_UART_Receive_IT(&huart3, rxBuffer, 1);

		if (rxIndex >= 5) {
			if (rxData[0] == 0x5A && rxData[1] == 0xA5) {
				uint8_t data_len = rxData[2];
				uint8_t total_len = data_len + 3;

				if (rxIndex >= total_len) {
					for(int i = 0; i < total_len && i < (int)sizeof(RxData); i++) {
						RxData[i] = rxData[i];
					}
					dwin_pkt_ready = 1;
					memset(rxData, 0, sizeof(rxData));
					rxIndex = 0;
				}
			} else {
				memset(rxData, 0, sizeof(rxData));
				rxIndex = 0;
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_7) {
		pa7_last_irq = HAL_GetTick();
		pa7_pending = 1;
	}

	if (GPIO_Pin == GPIO_PIN_0) {
		pb0_last_irq = HAL_GetTick();
		pb0_pending = 1;
	}
}

void dwin_uart_rx_start(void) {
    if (!uart_rx_started) {
        rxIndex = 0;
        memset(rxData, 0, sizeof(rxData));
        HAL_UART_Receive_IT(&huart3, rxBuffer, 1);
        uart_rx_started = 1;
    }
}

void dwin_rx_get_vp(uint16_t *addr, uint16_t *data, uint8_t *valid) {
    *valid = 0;
    if (RxData[0] != 0x5A || RxData[1] != 0xA5 || RxData[2] < 3) {
        return;
    }

    *addr = (uint16_t)(((uint16_t)RxData[4] << 8) | RxData[5]);
    uint16_t max_val = LOW_VALUE_MAX;
    if (*addr == VP_MODE_DATA) {
        if (mode == 0x4f || mode == 0x4d) {
            max_val = INPUT_MAX_HIBI;
        }
    } else if (*addr == VP_VAR_ICON) {
        max_val = VAR_ICON_KEY_MAX;
    } else if (*addr == VP_BRIGHTNESS) {
        max_val = 100;
    }

    uint8_t cmd = RxData[3];
    uint8_t len = RxData[2];

    if (*addr == VP_BRIGHTNESS && cmd == 0x83) {
        if (len >= 6) {
            uint16_t v = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[8]);
            if (v <= 100U) {
                *data = v;
                *valid = 1;
            } else if (RxData[7] <= 100U) {
                *data = RxData[7];
                *valid = 1;
            } else if (RxData[8] <= 100U) {
                *data = RxData[8];
                *valid = 1;
            }
            return;
        }
        if (len >= 4) {
            uint16_t be = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
            uint16_t le = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
            if (be <= 100U) {
                *data = be;
                *valid = 1;
            } else if (le <= 100U) {
                *data = le;
                *valid = 1;
            } else if (RxData[7] <= 100U) {
                *data = RxData[7];
                *valid = 1;
            }
            return;
        }
    }

    /* DWIN brightness write is often 5A A5 04 82 00 82 VV (len=4, one data byte) */
    if (*addr == VP_BRIGHTNESS && cmd == 0x82) {
        if (len >= 4 && RxData[6] <= 100U) {
            *data = RxData[6];
            *valid = 1;
            return;
        }
        if (len >= 5) {
            uint16_t be = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
            uint16_t le = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
            if (be <= 100U) {
                *data = be;
                *valid = 1;
            } else if (le <= 100U) {
                *data = le;
                *valid = 1;
            }
            return;
        }
    }

    /* VP 0x1000 / 0x3002 — DWIN 0x83 auto-upload uses bytes 7-8, not 6-7 */
    if ((*addr == VP_LOW_VALUE || *addr == VP_LOW_DATA) && cmd == 0x83) {
        if (len >= 6) {
            *data = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[8]);
            *valid = 1;
            return;
        }
        if (len >= 4) {
            uint16_t be = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
            uint16_t le = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
            if (be <= LOW_VALUE_MAX) {
                *data = be;
                *valid = 1;
            } else if (le <= LOW_VALUE_MAX) {
                *data = le;
                *valid = 1;
            }
            return;
        }
    }

    if ((*addr == VP_LOW_VALUE || *addr == VP_LOW_DATA) && cmd == 0x82 && len >= 5) {
        uint16_t be = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
        uint16_t le = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
        if (be <= LOW_VALUE_MAX) {
            *data = be;
            *valid = 1;
        } else if (le <= LOW_VALUE_MAX) {
            *data = le;
            *valid = 1;
        }
        return;
    }

    if (cmd == 0x82 && len >= 5) {
        uint16_t be = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
        uint16_t le = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
        if (be <= max_val) {
            *data = be;
            *valid = 1;
        } else if (le <= max_val) {
            *data = le;
            *valid = 1;
        }
        return;
    }

    if (cmd == 0x83) {
        uint16_t c1 = 0, c2 = 0, c3 = 0;
        if (len >= 6) {
            c1 = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[8]);
            c2 = (uint16_t)(((uint16_t)RxData[8] << 8) | RxData[7]);
            c3 = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
        } else if (len >= 4) {
            c1 = (uint16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);
            c2 = (uint16_t)(((uint16_t)RxData[7] << 8) | RxData[6]);
        }
        if (c1 <= max_val) {
            *data = c1;
            *valid = 1;
        } else if (c2 <= max_val) {
            *data = c2;
            *valid = 1;
        } else if (c3 <= max_val) {
            *data = c3;
            *valid = 1;
        }
    }
}

void low_value_user_changed(uint16_t new_low) {
    if (new_low > LOW_VALUE_MAX) {
        new_low = LOW_VALUE_MAX;
    }

    /* DWIN VP 0x2002 icon=low/2 snaps even values back (20→19, 200→199) */
    if (new_low == low - 1 && (low & 1U) == 0U && low > 0U &&
        (HAL_GetTick() - low_last_accept_tick) < 500U) {
        return;
    }

    /* 22.bin floods 200/110 on page load — never save, never UART echo */
    if ((new_low == LOW_VALUE_MAX || new_low == 110) && low < (LOW_VALUE_MAX - 3)) {
        return;
    }

    if (new_low == low) {
        return;
    }

    low = new_low;
    loww[0] = (uint8_t)((low >> 8) & 0xFF);
    loww[1] = (uint8_t)(low & 0xFF);
    eeprom_save_low_value();
    low_user_touched = 1;
    low_display_push_until = 0;
    low_last_accept_tick = HAL_GetTick();

    if (mode == 0x4e && ((footswitch) || (active))) {
        lowfunction(low);
    }
}

static void low_handle_rx(uint16_t new_low) {
    if (current_page != 2U) {
        return;
    }
    if (HAL_GetTick() < low_save_enable_tick) {
        return;
    }
    low_value_user_changed(new_low);
}

static void dwin_uart_send(const uint8_t *data, uint16_t len) {
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    if (HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000) != HAL_OK) {
        HAL_Delay(10);
        HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000);
    }
}

void send_page_change(uint8_t page_num) {
    uint8_t cmd[10] = {
        0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84,
        0x5A, 0x01, 0x00, page_num
    };
    dwin_uart_send(cmd, 10);
    current_page = page_num;
    if (page_num == 1U || page_num == 2U) {
        var_icon_user_locked = 0;
        dwin_icon_burst_from_eeprom(4);
        eeprom_save_current_page(page_num);
    }
}

void dwin_boot_to_page(uint8_t page_num) {
    HAL_Delay(3000);  // show 00.bmp for 3 seconds
    send_page_change(page_num);
    HAL_Delay(100);
}

void send_variable_data(uint8_t addr_high, uint8_t addr_low, uint8_t data_high, uint8_t data_low) {
    uint8_t cmd[8] = {
        0x5A, 0xA5, 0x05, 0x82,
        addr_high, addr_low, data_high, data_low
    };
    dwin_uart_send(cmd, 8);
    HAL_Delay(10);
}

void dwin_set_var_icon(uint16_t icon_id) {
    var_icon_value = icon_id;
    var_icon_data[0] = (uint8_t)((icon_id >> 8) & 0xFF);
    var_icon_data[1] = (uint8_t)(icon_id & 0xFF);
}

void send_var_icon_to_display(void) {
    uint8_t cmd[8] = {
        0x5A, 0xA5, 0x05, 0x82,
        (uint8_t)((VP_VAR_ICON >> 8) & 0xFF),
        (uint8_t)(VP_VAR_ICON & 0xFF),
        var_icon_data[0], var_icon_data[1]
    };
    dwin_uart_send(cmd, 8);
    HAL_Delay(2);
}

static void send_var_icon_fast(void) {
    uint8_t cmd[8] = {
        0x5A, 0xA5, 0x05, 0x82,
        (uint8_t)((VP_VAR_ICON >> 8) & 0xFF),
        (uint8_t)(VP_VAR_ICON & 0xFF),
        var_icon_data[0], var_icon_data[1]
    };
    dwin_uart_send(cmd, 8);
}

void eeprom_save_current_page(uint8_t page_num) {
    if (page_num != 1U && page_num != 2U) {
        return;
    }
    saved_settings_page = page_num;
    current_page_array[0] = EEPROM_PAGE_MAGIC;
    current_page_array[1] = page_num;
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, EEPROM_ADDR_PAGE,
                          I2C_MEMADD_SIZE_16BIT, current_page_array, 2, 1000);
        HAL_Delay(10);
    }
}

static uint8_t eeprom_parse_saved_page(const uint8_t *buf) {
    if (buf[0] == EEPROM_PAGE_MAGIC &&
        (buf[1] == 1U || buf[1] == 2U)) {
        return buf[1];
    }
    if (buf[0] == 0x00U && (buf[1] == 1U || buf[1] == 2U)) {
        return buf[1];
    }
    return 0U;
}

static uint8_t var_icon_from_mode_key(uint16_t key) {
    if (key == MODE_KEY_HIGH || key == 0x004f || key == 0x4f) {
        return ICON_HIGH;
    }
    if (key == MODE_KEY_LOW || key == 0x004e || key == 0x4e) {
        return ICON_LOW;
    }
    if (key == MODE_KEY_BIPOLAR || key == 0x004d || key == 0x4d) {
        return ICON_BIPOLAR;
    }
    return 0U;
}

static uint8_t var_icon_from_mode(uint16_t m) {
    if (m == 0x4f) {
        return ICON_HIGH;
    }
    if (m == 0x4e) {
        return ICON_LOW;
    }
    if (m == 0x4d) {
        return ICON_BIPOLAR;
    }
    return BOOT_VAR_ICON_VALUE;
}

void eeprom_read_var_icon(void) {
    uint8_t buf[2] = {0xFF, 0xFF};
    uint8_t legacy[2] = {0xFF, 0xFF};

    dwin_set_var_icon(var_icon_from_mode(mode));

    if (i2c_eeprom_read16(EEPROM_ADDR_VAR_ICON, buf)) {
        if (buf[0] == EEPROM_ICON_MAGIC &&
            buf[1] >= MIN_VAR_ICON_VALUE && buf[1] <= VAR_ICON_KEY_MAX) {
            dwin_set_var_icon((uint16_t)buf[1]);
            return;
        }

        uint16_t icon = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        if (icon >= MIN_VAR_ICON_VALUE && icon <= VAR_ICON_KEY_MAX) {
            dwin_set_var_icon(icon);
            eeprom_save_var_icon();
            return;
        }
    }

    if (i2c_eeprom_read16(EEPROM_ADDR_VAR_ICON_OLD, legacy)) {
        if (legacy[0] == EEPROM_ICON_MAGIC &&
            legacy[1] >= MIN_VAR_ICON_VALUE && legacy[1] <= VAR_ICON_KEY_MAX) {
            dwin_set_var_icon((uint16_t)legacy[1]);
            eeprom_save_var_icon();
            return;
        }

        uint16_t icon = (uint16_t)(((uint16_t)legacy[0] << 8) | legacy[1]);
        if (icon >= MIN_VAR_ICON_VALUE && icon <= VAR_ICON_KEY_MAX) {
            dwin_set_var_icon(icon);
            eeprom_save_var_icon();
            return;
        }
    }

    eeprom_save_var_icon();
}

void eeprom_save_var_icon(void) {
    uint8_t buf[2];

    if (var_icon_value < MIN_VAR_ICON_VALUE || var_icon_value > VAR_ICON_KEY_MAX) {
        return;
    }

    buf[0] = EEPROM_ICON_MAGIC;
    buf[1] = (uint8_t)var_icon_value;
    var_icon_data[0] = 0x00;
    var_icon_data[1] = buf[1];

    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, EEPROM_ADDR_VAR_ICON,
                          I2C_MEMADD_SIZE_16BIT, buf, 2, 1000);
        HAL_Delay(10);
        if (current_page == 1U || current_page == 2U) {
            eeprom_save_current_page((uint8_t)current_page);
        }
    } else {
        eeprom_err = true;
    }
}

static uint8_t dwin_get_boot_page(void) {
    if (saved_settings_page == 1U || saved_settings_page == 2U) {
        return saved_settings_page;
    }
    if (mode == 0x4f || mode == 0x4d) {
        return 1U;
    }
    return 2U;
}

static void eeprom_read_saved_page(void) {
    uint8_t legacy[2] = {0xFF, 0xFF};
    saved_settings_page = 0;
    current_page = 0;
    current_page_array[0] = 0xFF;
    current_page_array[1] = 0xFF;

    if (i2c_eeprom_read16(EEPROM_ADDR_PAGE, current_page_array)) {
        saved_settings_page = eeprom_parse_saved_page(current_page_array);
    }

    if (saved_settings_page == 0U &&
        i2c_eeprom_read16(EEPROM_ADDR_PAGE_OLD, legacy)) {
        saved_settings_page = eeprom_parse_saved_page(legacy);
        if (saved_settings_page != 0U) {
            eeprom_save_current_page(saved_settings_page);
        }
    }

    if (saved_settings_page == 1U || saved_settings_page == 2U) {
        current_page = saved_settings_page;
    }
}

void eeprom_read_low_value(void) {
    low = 0;
    loww[0] = 0x00;
    loww[1] = 0x00;

    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 3, 50) != HAL_OK) {
        return;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, EEPROM_ADDR_LOW, I2C_MEMADD_SIZE_16BIT,
                         loww, 2, 1000) != HAL_OK) {
        loww[0] = 0x00;
        loww[1] = 0x00;
        return;
    }
    HAL_Delay(10);

    if (loww[0] == 0xFF && loww[1] == 0xFF) {
        low = 0;
        loww[0] = 0x00;
        loww[1] = 0x00;
        eeprom_save_low_value();
        return;
    }

    low = (uint16_t)((loww[0] << 8) | loww[1]);
    if (low > LOW_VALUE_MAX) {
        low = LOW_VALUE_MAX;
        loww[0] = (uint8_t)((low >> 8) & 0xFF);
        loww[1] = (uint8_t)(low & 0xFF);
        eeprom_save_low_value();
    }
}

void eeprom_save_low_value(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, EEPROM_ADDR_LOW, I2C_MEMADD_SIZE_16BIT,
                          loww, 2, 1000);
        HAL_Delay(10);
        if (current_page == 2U) {
            eeprom_save_current_page(2U);
        }
    } else {
        eeprom_err = true;
    }
}

static uint8_t i2c_eeprom_read16(uint16_t addr, uint8_t *buf) {
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 2, 20) != HAL_OK) {
        return 0;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, addr, I2C_MEMADD_SIZE_16BIT, buf, 2, 200) != HAL_OK) {
        return 0;
    }
    HAL_Delay(5);
    return 1;
}

static void eeprom_apply_brightness(uint16_t val) {
    if (val == 0 || val == 0xFFFF || val > 100) {
        val = DEFAULT_BRIGHTNESS;
    } else if (val < MIN_BRIGHTNESS) {
        val = MIN_BRIGHTNESS;
    }
    brightnes = val;
    brightness[0] = 0x00;
    brightness[1] = (uint8_t)val;
}

static void eeprom_save_brightness(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0100, I2C_MEMADD_SIZE_16BIT,
                          brightness, 2, 1000);
        HAL_Delay(10);
    }
}

void eeprom_read_brightness(void) {
    eeprom_apply_brightness(DEFAULT_BRIGHTNESS);

    if (!i2c_eeprom_read16(0x0100, brightness)) {
        eeprom_save_brightness();
        return;
    }

    uint16_t val = (uint16_t)((brightness[0] << 8) | brightness[1]);
    if (val == 0xFFFF || (brightness[0] == 0xFF && brightness[1] == 0xFF)) {
        eeprom_save_brightness();
        return;
    }

    uint16_t before = val;
    eeprom_apply_brightness(val);
    if (brightnes != before) {
        eeprom_save_brightness();
    }
}

static void dwin_send_brightness_quick(void) {
    uint8_t v = (uint8_t)brightnes;
    if (v < MIN_BRIGHTNESS) {
        v = MIN_BRIGHTNESS;
    }
    uint8_t cmd[7] = {0x5A, 0xA5, 0x04, 0x82, 0x00, 0x82, v};
    dwin_uart_send(cmd, 7);
}

void dwin_send_brightness(void) {
    uint8_t v = (uint8_t)brightnes;
    if (v < MIN_BRIGHTNESS) {
        v = MIN_BRIGHTNESS;
        brightnes = v;
        brightness[1] = v;
    }

    /* DWIN spec: 5A A5 04 82 00 82 VV — value in one byte, not 00 VV */
    dwin_send_brightness_quick();
    HAL_Delay(20);

    /* Alternate: duplicate data bytes VV VV */
    uint8_t cmd_dup[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x82, v, v};
    dwin_uart_send(cmd_dup, 8);
}

static void dwin_disable_backlight_standby(void) {
    /* VP 0x0080: bit2=0 standby off, bit3=1 touch buzzer ON (0x30 mutes buzzer) */
    uint8_t cmd[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x80, 0x5A, 0x00, 0x00, 0x38};
    dwin_uart_send(cmd, 10);
}

static void brightness_user_changed(uint16_t requested) {
    if (requested > 100U) {
        return;
    }

    uint16_t before = brightnes;
    eeprom_apply_brightness(requested);
    if (brightnes == before) {
        return;
    }

    eeprom_save_brightness();
    brightness_user_touched = 1;
    brightness_display_push_until = 0;

    if (requested < MIN_BRIGHTNESS) {
        dwin_send_brightness_quick();
    }
}

static void brightness_handle_rx(uint16_t reported) {
    if (HAL_GetTick() < brightness_save_enable_tick) {
        return;
    }
    brightness_user_changed(reported);
}

static void dwin_poll_brightness(void) {
    static uint32_t last_poll = 0;

    if (HAL_GetTick() < brightness_save_enable_tick) {
        return;
    }
    if (HAL_GetTick() - last_poll < 800U) {
        return;
    }
    last_poll = HAL_GetTick();

    uint8_t cmd[7] = {0x5A, 0xA5, 0x04, 0x83, 0x00, 0x82, 0x01};
    dwin_uart_send(cmd, 7);
}

void dwin_refresh_brightness(void) {
    eeprom_read_brightness();
    brightness_user_touched = 0;
    brightness_display_push_until = HAL_GetTick() + 8000;
    brightness_save_enable_tick = HAL_GetTick() + 500;
    dwin_disable_backlight_standby();
    dwin_send_brightness();
}

void eeprom_read_vp3003_value(void) {
    vp3003_val = 0;
    vp3003w[0] = 0x00;
    vp3003w[1] = 0x00;

    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 3, 50) != HAL_OK) {
        return;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, EEPROM_ADDR_VP3003, I2C_MEMADD_SIZE_16BIT,
                         vp3003w, 2, 1000) != HAL_OK) {
        return;
    }
    HAL_Delay(10);

    if (vp3003w[0] == 0xFF && vp3003w[1] == 0xFF) {
        if (mode == 0x4f) {
            vp3003_val = high;
            vp3003w[0] = highh[0];
            vp3003w[1] = highh[1];
        } else if (mode == 0x4d) {
            vp3003_val = bipolar;
            vp3003w[0] = bipolarr[0];
            vp3003w[1] = bipolarr[1];
        } else {
            vp3003_val = low;
            vp3003w[0] = loww[0];
            vp3003w[1] = loww[1];
        }
        eeprom_save_vp3003_value();
        return;
    }

    vp3003_val = (uint16_t)((vp3003w[0] << 8) | vp3003w[1]);

    if (mode == 0x4f) {
        if (vp3003_val > INPUT_MAX_HIBI) {
            vp3003_val = INPUT_MAX_HIBI;
        }
        high = vp3003_val;
        highh[0] = vp3003w[0];
        highh[1] = vp3003w[1];
    } else if (mode == 0x4d) {
        if (vp3003_val > INPUT_MAX_HIBI) {
            vp3003_val = INPUT_MAX_HIBI;
        }
        bipolar = vp3003_val;
        bipolarr[0] = vp3003w[0];
        bipolarr[1] = vp3003w[1];
    } else if (vp3003_val > LOW_VALUE_MAX) {
        vp3003_val = LOW_VALUE_MAX;
        vp3003w[0] = (uint8_t)((vp3003_val >> 8) & 0xFF);
        vp3003w[1] = (uint8_t)(vp3003_val & 0xFF);
        eeprom_save_vp3003_value();
    }
}

void eeprom_save_vp3003_value(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, EEPROM_ADDR_VP3003, I2C_MEMADD_SIZE_16BIT,
                          vp3003w, 2, 1000);
        HAL_Delay(10);
        if (current_page == 1U) {
            eeprom_save_current_page(1U);
        }
    } else {
        eeprom_err = true;
    }
}

static uint8_t vp3003_is_page_default(uint16_t val) {
    if (val == 0) {
        return 1;
    }
    if ((mode == 0x4f || mode == 0x4d) && val == INPUT_MAX_HIBI) {
        return 1;
    }
    if (mode == 0x4e && (val == LOW_VALUE_MAX || val == 110)) {
        return 1;
    }
    return 0;
}

static void vp3003_stop_push(void) {
    vp3003_page1_restore_until = 0;
    vp3003_save_enable_tick = 0;
}

static void vp3003_apply_mode_key(uint16_t new_mode) {
    mode = new_mode;
    modee[0] = (uint8_t)((new_mode >> 8) & 0xFF);
    modee[1] = (uint8_t)(new_mode & 0xFF);
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x000A,
                          I2C_MEMADD_SIZE_16BIT, modee, 2, 1000);
        HAL_Delay(10);
    } else {
        eeprom_err = true;
    }
    previous_mode = new_mode;
    send_variable_data(0x30, 0x00, modee[0], modee[1]);
    dwin_show_settings_page(1U);
}

static uint8_t vp3003_is_mode_return_key(uint16_t val) {
    return (var_icon_from_mode_key(val) != 0U);
}

static void var_icon_apply_return_key(uint16_t key) {
    uint8_t icon_id = var_icon_from_mode_key(key);
    if (icon_id == 0U) {
        return;
    }

    dwin_set_var_icon(icon_id);
    eeprom_save_var_icon();
    var_icon_user_locked = 1;

    if (key == MODE_KEY_HIGH || key == 0x004f || key == 0x4f) {
        vp3003_apply_mode_key(0x4f);
    } else if (key == MODE_KEY_LOW || key == 0x004e || key == 0x4e) {
        mode = 0x4e;
        modee[0] = 0x00;
        modee[1] = 0x4e;
        if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
            HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x000A,
                              I2C_MEMADD_SIZE_16BIT, modee, 2, 1000);
            HAL_Delay(10);
        } else {
            eeprom_err = true;
        }
        previous_mode = 0x4e;
        send_variable_data(0x30, 0x00, modee[0], modee[1]);
        dwin_show_settings_page(2U);
    } else {
        vp3003_apply_mode_key(0x4d);
    }

    dwin_icon_burst_fast(3);
}

static void vp3003_handle_page1_rx(uint16_t new_val) {
    if (current_page != 1) {
        dwin_show_settings_page(1U);
        if (vp3003_is_page_default(new_val)) {
            return;
        }
    }
    if (HAL_GetTick() < vp3003_save_enable_tick && vp3003_is_page_default(new_val)) {
        vp3003_send_to_display();
        return;
    }
    vp3003_user_touched = 1;
    vp3003_stop_push();
    vp3003_user_changed(new_val);
}

void vp3003_send_to_display(void) {
    send_variable_data((uint8_t)((VP_MODE_DATA >> 8) & 0xFF),
                       (uint8_t)(VP_MODE_DATA & 0xFF),
                       vp3003w[0], vp3003w[1]);
}

void vp3003_user_changed(uint16_t new_val) {
    if (mode == 0x4f || mode == 0x4d) {
        if (new_val > INPUT_MAX_HIBI) {
            new_val = INPUT_MAX_HIBI;
        }
        if (new_val == INPUT_MAX_HIBI && vp3003_val < (INPUT_MAX_HIBI - 3)) {
            return;
        }
        if (mode == 0x4f) {
            high = new_val;
            highh[0] = (uint8_t)((new_val >> 8) & 0xFF);
            highh[1] = (uint8_t)(new_val & 0xFF);
            if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
                HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0014,
                I2C_MEMADD_SIZE_16BIT, highh, 2, 1000);
                HAL_Delay(10);
            }
        } else {
            bipolar = new_val;
            bipolarr[0] = (uint8_t)((new_val >> 8) & 0xFF);
            bipolarr[1] = (uint8_t)(new_val & 0xFF);
            if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
                HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0018,
                I2C_MEMADD_SIZE_16BIT, bipolarr, 2, 1000);
                HAL_Delay(10);
            }
        }
    } else {
        if (new_val > LOW_VALUE_MAX) {
            new_val = LOW_VALUE_MAX;
        }
        if ((new_val == LOW_VALUE_MAX || new_val == 110) && vp3003_val < (LOW_VALUE_MAX - 3)) {
            return;
        }
    }

    if (new_val == vp3003_val) {
        return;
    }

    vp3003_val = new_val;
    vp3003w[0] = (uint8_t)((vp3003_val >> 8) & 0xFF);
    vp3003w[1] = (uint8_t)(vp3003_val & 0xFF);
    eeprom_save_vp3003_value();
}

void low_sync_both_vps(void) {
    uint8_t dh = (uint8_t)((low >> 8) & 0xFF);
    uint8_t dl = (uint8_t)(low & 0xFF);

    send_variable_data((uint8_t)((VP_LOW_VALUE >> 8) & 0xFF),
                       (uint8_t)(VP_LOW_VALUE & 0xFF), dh, dl);
    send_variable_data((uint8_t)((VP_LOW_DATA >> 8) & 0xFF),
                       (uint8_t)(VP_LOW_DATA & 0xFF), dh, dl);
}

void low_push_display_from_eeprom(void) {
    eeprom_read_low_value();
    low_sync_both_vps();
    previous_low = low;
}

void low_send_to_display(void) {
    low_push_display_from_eeprom();
}

static void dwin_arm_var_icon_timers(void) {
    var_icon_save_enable_tick = HAL_GetTick() + 5000;
    var_icon_display_push_until = HAL_GetTick() + 15000;
    var_icon_boot_push_until = HAL_GetTick() + 20000;
    var_icon_aggressive_until = HAL_GetTick() + 3000;
    eeprom_read_var_icon();
}

static void dwin_arm_page2_restore_timers(void) {
    low_user_touched = 0;
    low_save_enable_tick = HAL_GetTick() + 5000;
    dwin_arm_var_icon_timers();
    low_display_push_until = HAL_GetTick() + 15000;
}

static void dwin_push_icon_from_eeprom(void) {
    eeprom_read_var_icon();
    send_var_icon_to_display();
}

static void dwin_icon_burst_fast(uint8_t count) {
    while (count > 0U) {
        send_var_icon_fast();
        count--;
    }
}

static void dwin_icon_burst_from_eeprom(uint8_t count) {
    eeprom_read_var_icon();
    dwin_icon_burst_fast(count);
}

static void dwin_restore_var_icon_from_eeprom(uint8_t burst_count) {
    dwin_arm_var_icon_timers();
    eeprom_read_var_icon();
    if (burst_count > 0U) {
        dwin_icon_burst_fast(burst_count);
    } else {
        send_var_icon_fast();
    }
}

static void dwin_push_page2_from_eeprom(void) {
    eeprom_read_low_value();
    previous_low = low;
    low_sync_both_vps();
    eeprom_read_var_icon();
    dwin_icon_burst_fast(2);
}

static void dwin_restore_page_from_eeprom(uint8_t page_num) {
    if (page_num == 1U) {
        eeprom_read_vp3003_value();
        vp3003_user_touched = 0;
        vp3003_save_enable_tick = HAL_GetTick() + 3000;
        vp3003_page1_restore_until = HAL_GetTick() + 5000;
        vp3003_send_to_display();
        eeprom_read_var_icon();
        dwin_icon_burst_fast(2);
    } else if (page_num == 2U) {
        dwin_arm_page2_restore_timers();
        dwin_push_page2_from_eeprom();
    }
}

static void dwin_show_settings_page(uint8_t page_num) {
    if (page_num != 1U && page_num != 2U) {
        return;
    }

    if (page_num == 2U) {
        eeprom_read_low_value();
        eeprom_read_var_icon();
    } else {
        eeprom_read_vp3003_value();
    }

    if (current_page != page_num) {
        send_page_change(page_num);
    } else {
        eeprom_save_current_page(page_num);
        dwin_icon_burst_fast(4);
    }

    if (page_num == 2U) {
        dwin_arm_page2_restore_timers();
        eeprom_read_low_value();
        low_sync_both_vps();
        dwin_icon_burst_fast(2);
    } else {
        dwin_arm_var_icon_timers();
        dwin_restore_page_from_eeprom(1U);
        vp3003_send_to_display();
        dwin_refresh_brightness();
        dwin_icon_burst_fast(2);
    }
}

void dwin_refresh_page2_values(void) {
    dwin_arm_page2_restore_timers();
    eeprom_read_low_value();
    eeprom_read_var_icon();
    low_sync_both_vps();
    dwin_icon_burst_fast(3);
}

void dwin_refresh_page1_values(void) {
    dwin_restore_page_from_eeprom(1U);
    eeprom_read_var_icon();
    dwin_icon_burst_fast(3);
}

void dwin_restore_saved_state(uint8_t page_num) {
    current_page = page_num;
    saved_settings_page = page_num;
    if (page_num == 1U || page_num == 2U) {
        eeprom_save_current_page(page_num);
    }

    dwin_arm_var_icon_timers();

    send_variable_data(0x30, 0x00, modee[0], modee[1]);

    brightness_user_touched = 0;
    eeprom_read_brightness();
    dwin_disable_backlight_standby();
    dwin_send_brightness_quick();
    brightness_save_enable_tick = HAL_GetTick() + 500;
    brightness_display_push_until = HAL_GetTick() + 5000;

    dwin_restore_page_from_eeprom(page_num);
    if (page_num == 2U) {
        eeprom_read_low_value();
        low_sync_both_vps();
    } else {
        vp3003_send_to_display();
    }
    dwin_icon_burst_from_eeprom(6);
}

void dwin_open_page1(void) {
    dwin_show_settings_page(1U);
}

void dwin_open_page2(void) {
    dwin_show_settings_page(2U);
}

void dwin_apply_boot_var_icon(void) {
    eeprom_read_var_icon();
    dwin_arm_var_icon_timers();
    dwin_icon_burst_fast(4);
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, 2);

  HAL_Delay(200);

  low_save_enable_tick = 0;

  /* Read EEPROM before any display UART (so we know the real low value) */
  eeprom_read_low_value();
  eeprom_read_saved_page();

  i2c_eeprom_read16(0x000A, modee);
  mode = (modee[0] << 8) | modee[1];
  previous_mode = mode;

  i2c_eeprom_read16(0x0014, highh);
  high = (highh[0] << 8) | highh[1];

  i2c_eeprom_read16(0x0018, bipolarr);
  bipolar = (bipolarr[0] << 8) | bipolarr[1];

  eeprom_read_vp3003_value();
  eeprom_read_brightness();
  eeprom_read_var_icon();

  {
  uint8_t boot_page = dwin_get_boot_page();

  dwin_boot_to_page(boot_page);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 300);

  dwin_restore_saved_state(boot_page);
  boot_page_guard_until = HAL_GetTick() + 8000;
  previous_mode = mode;

  dwin_uart_rx_start();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);

    if (HAL_GetTick() < var_icon_aggressive_until) {
        send_var_icon_fast();
    } else if (HAL_GetTick() < var_icon_boot_push_until) {
        static uint32_t last_boot_icon = 0;
        if (HAL_GetTick() - last_boot_icon >= 30) {
            send_var_icon_fast();
            last_boot_icon = HAL_GetTick();
        }
    } else if (HAL_GetTick() < var_icon_display_push_until) {
        static uint32_t last_icon_push = 0;
        if (HAL_GetTick() - last_icon_push >= 80) {
            dwin_push_icon_from_eeprom();
            last_icon_push = HAL_GetTick();
        }
    }

    if (dwin_pkt_ready) {
        dwin_pkt_ready = 0;

        uint16_t rx_addr = 0;
        uint16_t rx_data = 0;
        uint8_t rx_valid = 0;

        dwin_rx_get_vp(&rx_addr, &rx_data, &rx_valid);
        if (rx_valid) {
            address = rx_addr;
            dataa = rx_data;
            number_eeprome_flag = true;
            prev_address = address;
            prev_dataa = dataa;
        }
    }

    if (current_page == 2U && !low_user_touched &&
        HAL_GetTick() < low_display_push_until) {
        static uint32_t last_low_push = 0;
        if (HAL_GetTick() - last_low_push >= 350) {
            eeprom_read_low_value();
            previous_low = low;
            low_sync_both_vps();
            last_low_push = HAL_GetTick();
        }
    } else if (!brightness_user_touched &&
               HAL_GetTick() < brightness_display_push_until) {
        static uint32_t last_brightness_push = 0;
        if (HAL_GetTick() - last_brightness_push >= 800) {
            dwin_send_brightness_quick();
            last_brightness_push = HAL_GetTick();
        }
    }

    if (current_page == 1 && !vp3003_user_touched &&
        HAL_GetTick() < vp3003_page1_restore_until) {
        static uint32_t last_vp3003_push = 0;
        if (HAL_GetTick() - last_vp3003_push >= 400) {
            vp3003_send_to_display();
            last_vp3003_push = HAL_GetTick();
        }
    }

    count++;

    save_reading();
    send_loadings();

    dwin_poll_brightness();

    debouncing();
    hv_adc_reading();

    if (((footswitch) || (active)) && (alert_hv == 0) && (alert_gate == 0)) {

        ch_pg_flag = true;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

        HAL_GPIO_WritePin(hv_en_GPIO_Port, hv_en_Pin, GPIO_PIN_RESET);
        switch (mode) {
        case 0x4f:
            bi_hi_pwm(high);
            HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_RESET);
            send_page_change(0x05);
            break;
        case 0x4e:
            lowfunction(low);
            HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_SET);
            send_page_change(0x06);
            break;
        case 0x4d:
            bi_hi_pwm(bipolar);
            HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_SET);
            send_page_change(0x04);
            break;
        }
        hv_adc_reading();

    } else {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(hv_en_GPIO_Port, hv_en_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, highrly_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, biprly_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, lowrly_Pin, GPIO_PIN_RESET);

        if (ch_pg_flag) {
            ch_pg_flag = false;
            if((alert_hv != 1) && (alert_gate != 1)) {
                if ((mode == 0x4f) || (mode == 0x4d)) {
                    dwin_open_page1();
                } else {
                    dwin_open_page2();
                }
            } else if(alert_hv == 1) {
                send_page_change(0x08);
            } else if(alert_gate == 1) {
                send_page_change(0x09);
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }

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

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

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

  __HAL_RCC_DMA1_CLK_ENABLE();

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

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, biprly_Pin|highrly_Pin|lowrly_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(hv_en_GPIO_Port, hv_en_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = biprly_Pin|highrly_Pin|lowrly_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = hv_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hv_en_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static uint8_t var_icon_is_user_key(uint16_t val) {
    return (val >= MIN_VAR_ICON_VALUE && val <= VAR_ICON_KEY_MAX);
}

static void var_icon_user_changed(uint16_t new_value) {
    if (!var_icon_is_user_key(new_value)) {
        return;
    }
    if (new_value == var_icon_value) {
        return;
    }
    dwin_set_var_icon(new_value);
    eeprom_save_var_icon();
    var_icon_user_locked = 1;
}

void send_low_to_display(void) {
    low_sync_both_vps();
}

void save_reading(void) {
    if (number_eeprome_flag) {
        number_eeprome_flag = false;

        // ============== FIX: Handle variable icon at VP 0x2000 ==============
        if (address == VP_VAR_ICON) {
            uint16_t new_value = dataa;

            if (var_icon_from_mode_key(new_value) != 0U) {
                var_icon_apply_return_key(new_value);
                return;
            }

            if (new_value == var_icon_value) {
                return;
            }

            if (var_icon_is_user_key(new_value)) {
                var_icon_user_changed(new_value);
                return;
            }

            if (var_icon_user_locked) {
                return;
            }

            if (HAL_GetTick() < var_icon_save_enable_tick) {
                return;
            }

            // Spurious increment writes — ignore silently (re-send causes blink)
            if (new_value < MIN_VAR_ICON_VALUE ||
                (new_value <= 100 && new_value == (uint16_t)(low / 2))) {
                return;
            }

            return;
        }

        // ============== FIX: Handle mode at VP 0x3000 ==============
        if (address == VP_MODE) {
            mode = dataa;
            modee[0] = (uint8_t)((dataa >> 8) & 0xFF);
            modee[1] = (uint8_t)(dataa & 0xFF);
            if (HAL_I2C_IsDeviceReady(&hi2c1, 0x50 << 1, 1, 10) == HAL_OK) {
                HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x000A,
                I2C_MEMADD_SIZE_16BIT, modee, 2, 1000);
                HAL_Delay(10);
            } else {
                eeprom_err = true;
            }
            var_icon_user_changed(var_icon_from_mode(mode));
            dwin_icon_burst_fast(3);
            if (mode == 0x4e) {
                if (HAL_GetTick() >= boot_page_guard_until) {
                    if (current_page != 2U) {
                        dwin_show_settings_page(2U);
                    } else {
                        dwin_refresh_page2_values();
                    }
                }
            } else if (mode == 0x4f || mode == 0x4d) {
                if (HAL_GetTick() >= boot_page_guard_until) {
                    if (current_page != 1U) {
                        dwin_show_settings_page(1U);
                    } else {
                        eeprom_save_current_page(1);
                        eeprom_read_vp3003_value();
                        dwin_refresh_page1_values();
                    }
                }
            }
            dwin_restore_var_icon_from_eeprom(3);
            return;
        }

        // VP 0x1000 or VP 0x3002 — save EEPROM and sync both VPs
        if (address == VP_LOW_VALUE || address == VP_LOW_DATA) {
            low_handle_rx(dataa);
            return;
        }

        // VP 0x3003 / page 1 value — restore from EEPROM on show, save on user change
        if (address == VP_MODE_DATA) {
            vp3003_handle_page1_rx(dataa);
            return;
        }

        // VP 0x0082 backlight — save EEPROM, enforce minimum 80
        if (address == VP_BRIGHTNESS || address == 0x82) {
            brightness_handle_rx(dataa);
            return;
        }

        // Handle other addresses
        switch (address) {
        case 0x2001:
            if (dataa == 0x01) {
                if ((mode == 0x4f) || (mode == 0x4d)) {
                    dwin_open_page1();
                } else if (mode == 0x4e) {
                    dwin_open_page2();
                }
            }
            break;
        }
    }
}

void send_loadings(void) {
    if (mode != previous_mode) {
        if (HAL_GetTick() < boot_page_guard_until) {
            previous_mode = mode;
            return;
        }
        if (mode == 0x4e) {
            dwin_show_settings_page(2U);
        } else if (mode == 0x4f || mode == 0x4d) {
            if (current_page == 1U) {
                dwin_refresh_page1_values();
            } else {
                dwin_show_settings_page(1U);
            }
        }
        dwin_restore_var_icon_from_eeprom(3);
        previous_mode = mode;
    }
}

void debouncing(void) {
    if (pa7_pending) {
        if ((HAL_GetTick() - pa7_last_irq) >= DEBOUNCE_MS) {
            pa7_pending = 0;
            footswitch = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET);
        }
    }
    if (pb0_pending) {
        if ((HAL_GetTick() - pb0_last_irq) >= DEBOUNCE_MS) {
            pb0_pending = 0;
            active = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
        }
    }
}

void hv_adc_reading(void)
{
    if ((HAL_GetTick() - last_adc_tick) >= 10)
    {
        last_adc_tick = HAL_GetTick();

         adc_raw_ch1 = adc_dma_buf[0];
         adc_raw_ch0 = adc_dma_buf[1];

         ch1_voltage = (adc_raw_ch1 * VDDA_VOLTAGE) / ADC_MAX_VALUE;
         ch0_voltage = (adc_raw_ch0 * VDDA_VOLTAGE) / ADC_MAX_VALUE;

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
    case 0:  TIM3->CCR1 = 0;    break;
    case 1:  TIM3->CCR1 = 102;  break;
    case 2:  TIM3->CCR1 = 186;  break;
    case 3:  TIM3->CCR1 = 246;  break;
    case 4:  TIM3->CCR1 = 307;  break;
    case 5:  TIM3->CCR1 = 353;  break;
    case 6:  TIM3->CCR1 = 384;  break;
    case 7:  TIM3->CCR1 = 423;  break;
    case 8:  TIM3->CCR1 = 458;  break;
    case 9:  TIM3->CCR1 = 496;  break;
    case 10: TIM3->CCR1 = 524;  break;
    case 11: TIM3->CCR1 = 544;  break;
    case 12: TIM3->CCR1 = 583;  break;
    case 13: TIM3->CCR1 = 609;  break;
    case 14: TIM3->CCR1 = 630;  break;
    case 15: TIM3->CCR1 = 655;  break;
    case 16: TIM3->CCR1 = 673;  break;
    case 17: TIM3->CCR1 = 697;  break;
    case 18: TIM3->CCR1 = 717;  break;
    case 19: TIM3->CCR1 = 742;  break;
    case 20: TIM3->CCR1 = 754;  break;
    case 21: TIM3->CCR1 = 781;  break;
    case 22: TIM3->CCR1 = 803;  break;
    case 23: TIM3->CCR1 = 812;  break;
    case 24: TIM3->CCR1 = 836;  break;
    case 25: TIM3->CCR1 = 856;  break;
    case 26: TIM3->CCR1 = 867;  break;
    case 27: TIM3->CCR1 = 889;  break;
    case 28: TIM3->CCR1 = 901;  break;
    case 29: TIM3->CCR1 = 930;  break;
    case 30: TIM3->CCR1 = 948;  break;
    case 31: TIM3->CCR1 = 960;  break;
    case 32: TIM3->CCR1 = 981;  break;
    case 33: TIM3->CCR1 = 989;  break;
    case 34: TIM3->CCR1 = 998;  break;
    case 35: TIM3->CCR1 = 998;  break;
    default: TIM3->CCR1 = 0;    break;
    }
}

void lowfunction(uint16_t dataa) {
    if (dataa > 200) dataa = 200;

    uint32_t pwm_value = 0;

    switch (dataa) {
        case 0:   pwm_value = 0;    break;   case 1:   pwm_value = 12;   break;
        case 2:   pwm_value = 42;   break;   case 3:   pwm_value = 69;   break;
        case 4:   pwm_value = 98;   break;   case 5:   pwm_value = 114;  break;
        case 6:   pwm_value = 139;  break;   case 7:   pwm_value = 158;  break;
        case 8:   pwm_value = 171;  break;   case 9:   pwm_value = 181;  break;
        case 10:  pwm_value = 196;  break;   case 11:  pwm_value = 207;  break;
        case 12:  pwm_value = 215;  break;   case 13:  pwm_value = 226;  break;
        case 14:  pwm_value = 226;  break;   case 15:  pwm_value = 239;  break;
        case 16:  pwm_value = 249;  break;   case 17:  pwm_value = 262;  break;
        case 18:  pwm_value = 271;  break;   case 19:  pwm_value = 271;  break;
        case 20:  pwm_value = 279;  break;   case 21:  pwm_value = 298;  break;
        case 22:  pwm_value = 298;  break;   case 23:  pwm_value = 309;  break;
        case 24:  pwm_value = 319;  break;   case 25:  pwm_value = 319;  break;
        case 26:  pwm_value = 332;  break;   case 27:  pwm_value = 332;  break;
        case 28:  pwm_value = 341;  break;   case 29:  pwm_value = 341;  break;
        case 30:  pwm_value = 352;  break;   case 31:  pwm_value = 363;  break;
        case 32:  pwm_value = 363;  break;   case 33:  pwm_value = 375;  break;
        case 34:  pwm_value = 375;  break;   case 35:  pwm_value = 385;  break;
        case 36:  pwm_value = 385;  break;   case 37:  pwm_value = 396;  break;
        case 38:  pwm_value = 396;  break;   case 39:  pwm_value = 405;  break;
        case 40:  pwm_value = 408;  break;   case 41:  pwm_value = 419;  break;
        case 42:  pwm_value = 419;  break;   case 43:  pwm_value = 428;  break;
        case 44:  pwm_value = 428;  break;   case 45:  pwm_value = 438;  break;
        case 46:  pwm_value = 438;  break;   case 47:  pwm_value = 438;  break;
        case 48:  pwm_value = 454;  break;   case 49:  pwm_value = 454;  break;
        case 50:  pwm_value = 463;  break;   case 51:  pwm_value = 462;  break;
        case 52:  pwm_value = 475;  break;   case 53:  pwm_value = 475;  break;
        case 54:  pwm_value = 475;  break;   case 55:  pwm_value = 482;  break;
        case 56:  pwm_value = 488;  break;   case 57:  pwm_value = 501;  break;
        case 58:  pwm_value = 501;  break;   case 59:  pwm_value = 501;  break;
        case 60:  pwm_value = 502;  break;   case 61:  pwm_value = 512;  break;
        case 62:  pwm_value = 524;  break;   case 63:  pwm_value = 524;  break;
        case 64:  pwm_value = 524;  break;   case 65:  pwm_value = 537;  break;
        case 66:  pwm_value = 537;  break;   case 67:  pwm_value = 547;  break;
        case 68:  pwm_value = 547;  break;   case 69:  pwm_value = 547;  break;
        case 70:  pwm_value = 557;  break;   case 71:  pwm_value = 557;  break;
        case 72:  pwm_value = 557;  break;   case 73:  pwm_value = 568;  break;
        case 74:  pwm_value = 568;  break;   case 75:  pwm_value = 568;  break;
        case 76:  pwm_value = 577;  break;   case 77:  pwm_value = 577;  break;
        case 78:  pwm_value = 577;  break;   case 79:  pwm_value = 589;  break;
        case 80:  pwm_value = 589;  break;   case 81:  pwm_value = 589;  break;
        case 82:  pwm_value = 598;  break;   case 83:  pwm_value = 598;  break;
        case 84:  pwm_value = 598;  break;   case 85:  pwm_value = 611;  break;
        case 86:  pwm_value = 611;  break;   case 87:  pwm_value = 611;  break;
        case 88:  pwm_value = 621;  break;   case 89:  pwm_value = 621;  break;
        case 90:  pwm_value = 621;  break;   case 91:  pwm_value = 633;  break;
        case 92:  pwm_value = 633;  break;   case 93:  pwm_value = 633;  break;
        case 94:  pwm_value = 647;  break;   case 95:  pwm_value = 647;  break;
        case 96:  pwm_value = 647;  break;   case 97:  pwm_value = 658;  break;
        case 98:  pwm_value = 658;  break;   case 99:  pwm_value = 671;  break;
        case 100: pwm_value = 671; break;    case 101: pwm_value = 682; break;
        case 102: pwm_value = 686; break;    case 103: pwm_value = 690; break;
        case 104: pwm_value = 694; break;    case 105: pwm_value = 698; break;
        case 106: pwm_value = 700; break;    case 107: pwm_value = 701; break;
        case 108: pwm_value = 702; break;    case 109: pwm_value = 702; break;
        case 110: pwm_value = 702; break;    case 111: pwm_value = 705; break;
        case 112: pwm_value = 708; break;    case 113: pwm_value = 710; break;
        case 114: pwm_value = 713; break;    case 115: pwm_value = 716; break;
        case 116: pwm_value = 718; break;    case 117: pwm_value = 721; break;
        case 118: pwm_value = 723; break;    case 119: pwm_value = 724; break;
        case 120: pwm_value = 725; break;    case 121: pwm_value = 730; break;
        case 122: pwm_value = 735; break;    case 123: pwm_value = 740; break;
        case 124: pwm_value = 745; break;    case 125: pwm_value = 748; break;
        case 126: pwm_value = 750; break;    case 127: pwm_value = 753; break;
        case 128: pwm_value = 755; break;    case 129: pwm_value = 756; break;
        case 130: pwm_value = 757; break;    case 131: pwm_value = 762; break;
        case 132: pwm_value = 767; break;    case 133: pwm_value = 772; break;
        case 134: pwm_value = 777; break;    case 135: pwm_value = 782; break;
        case 136: pwm_value = 785; break;    case 137: pwm_value = 787; break;
        case 138: pwm_value = 789; break;    case 139: pwm_value = 790; break;
        case 140: pwm_value = 790; break;    case 141: pwm_value = 795; break;
        case 142: pwm_value = 798; break;    case 143: pwm_value = 801; break;
        case 144: pwm_value = 804; break;    case 145: pwm_value = 807; break;
        case 146: pwm_value = 809; break;    case 147: pwm_value = 811; break;
        case 148: pwm_value = 812; break;    case 149: pwm_value = 813; break;
        case 150: pwm_value = 813; break;    case 151: pwm_value = 818; break;
        case 152: pwm_value = 823; break;    case 153: pwm_value = 828; break;
        case 154: pwm_value = 833; break;    case 155: pwm_value = 838; break;
        case 156: pwm_value = 841; break;    case 157: pwm_value = 844; break;
        case 158: pwm_value = 845; break;    case 159: pwm_value = 846; break;
        case 160: pwm_value = 847; break;    case 161: pwm_value = 852; break;
        case 162: pwm_value = 856; break;    case 163: pwm_value = 860; break;
        case 164: pwm_value = 863; break;    case 165: pwm_value = 866; break;
        case 166: pwm_value = 868; break;    case 167: pwm_value = 869; break;
        case 168: pwm_value = 869; break;    case 169: pwm_value = 869; break;
        case 170: pwm_value = 869; break;    case 171: pwm_value = 874; break;
        case 172: pwm_value = 879; break;    case 173: pwm_value = 884; break;
        case 174: pwm_value = 887; break;    case 175: pwm_value = 890; break;
        case 176: pwm_value = 892; break;    case 177: pwm_value = 893; break;
        case 178: pwm_value = 894; break;    case 179: pwm_value = 895; break;
        case 180: pwm_value = 895; break;    case 181: pwm_value = 900; break;
        case 182: pwm_value = 905; break;    case 183: pwm_value = 910; break;
        case 184: pwm_value = 915; break;    case 185: pwm_value = 918; break;
        case 186: pwm_value = 920; break;    case 187: pwm_value = 923; break;
        case 188: pwm_value = 926; break;    case 189: pwm_value = 928; break;
        case 190: pwm_value = 930; break;    case 191: pwm_value = 934; break;
        case 192: pwm_value = 938; break;    case 193: pwm_value = 941; break;
        case 194: pwm_value = 944; break;    case 195: pwm_value = 947; break;
        case 196: pwm_value = 949; break;    case 197: pwm_value = 950; break;
        case 198: pwm_value = 951; break;    case 199: pwm_value = 952; break;
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
