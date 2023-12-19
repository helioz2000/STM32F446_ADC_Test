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
#include <stdio.h>
#include "global.h"
#include "term.h"
#include "cmd.h"
#include "calc.h"
#include "ee24.h"
#include "wifi.h"
#ifdef USE_DISPLAY
#include "display.h"
#endif

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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const char product_msg[] = "PM1";
const char copyright_msg[] = "(C)2023 Control Technologies P/L";
char msg_buf[256];
char prt_buf[1024];
uint8_t sample_buf_lock = 0xFF;	// lock sample buffer to prevent override

// Command Line Interface
__IO uint16_t cli_rx_count = 0;
__IO uint8_t cli_rx_byte;
__IO uint8_t cli_rx_buff[128];
__IO uint8_t cli_rx_cmd_ready = 0;

// WiFi (ESP-01) interface
extern uint16_t esp_rx_count;
extern uint16_t esp_rx_error_count;
extern bool esp_rx_buffer_overflow;
extern uint8_t esp_rx_buf[];
__IO uint8_t esp_rx_byte;
uint16_t esp_rx_count_last = 0;

// EEPROM
__IO uint8_t eeprom_buf[16];
extern uint8_t ee24_lock;
bool ee24_read_done = false;

__IO uint8_t display_activate = 0;	// screen saver off
__IO uint8_t display_change = 0;		// change active screen

uint8_t adc_restart = 0;
uint8_t tft_display = 0;
uint8_t esp_mode = 0;
uint16_t new_time_period = 0;
uint8_t display_screen = 0;		// 0 = splash, 1 = main

__IO int32_t adc1_dma_l_count = 0;
__IO int32_t adc1_dma_h_count = 0;
__IO int32_t adc2_dma_l_count = 0;
__IO int32_t adc2_dma_h_count = 0;

__IO uint16_t adc_dma_buf[ADC_NUM][ADC_DMA_BUF_SIZE];		// one DMA buffer for each ADC (contains both channels)
uint16_t adc_raw_buf[ADC_NUM_BUFFERS][ADC_NUM_DATA];		// buffer for 4 channels of raw ADC data
uint16_t sample_buf[ADC_NUM_BUFFERS][SAMPLE_BUF_SIZE];		// buffer for 4 channels of downsampled data

float metervalue_v, metervalue_i1, metervalue_va1, metervalue_w1, metervalue_pf1;

uint32_t display_off_ticks;
uint32_t display_splash_ticks;
uint32_t display_update_ticks;
uint32_t now_ticks, last_ticks;
uint32_t next_measurement_time;
uint32_t next_process_time;
#define PROCESS_INTERVAL 100;	// run slow process every n ms
#define SCREEN_MAX 2

#define CLI_UART huart2
#define ESP_UART huart3

//uint8_t adc_read_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void start_adcs() {
	// Start ADC1 - keeps running via TIM2
	if ( HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf[ADC1_IDX], ADC_DMA_BUF_SIZE) != HAL_OK) {
	  term_print("Error starting ADC1 DMA\r\n");
  	  Error_Handler();
	}
	//Start ADC2 - keeps running via TIM2
	if ( HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_dma_buf[ADC2_IDX], ADC_DMA_BUF_SIZE) != HAL_OK) {
	  term_print("Error starting ADC2 DMA\r\n");
 	  Error_Handler();
	}
}

/*
 * Change TIM2 Period value (ARR) for timer tuning with oscilloscope
 * This function is used in conjunction with the debug GPIO to tune each individual
 * board to produce 25us signal which is shown on the oscilloscope
 * as a 20kHz square wave (period 50us) as the signal changes
 * with every TIM2 call
 */
void adjust_TIM2_period(uint16_t newPeriod, uint8_t store) {
	if ( (newPeriod > 2500) || (newPeriod < 2000) ) {
		term_print("Invalid period for TIM (%u)\r\n", newPeriod);
		return;
	}
	TIM2->ARR = (uint32_t) newPeriod;	// change register directly
	term_print("TIM2 ARR = %u\r\n", newPeriod);
	/*
	if (store) {
		// Store new value in Flash memory
		HAL_FLASH_Unlock();
		HAL_StatusTypeDef result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, EEPROM_USER_START_ADDR, (uint64_t)newPeriod);
		HAL_FLASH_Lock();
		er = HAL_FLASH_GetError();
		if (er !=0) {
			term_print("Flash Error Code: %Xu\r\n", er);
		}
		if (result != HAL_OK) {
			term_print("EEPROM write failed\r\n");
		} else {
			term_print("EEPROM write %u\r\n", newPeriod);
		}
	}*/
}

/*
 * @brief  Function to manage version change and update stored parameters
 */
void version_change(uint8_t old_major, uint8_t old_minor) {
	// update version number in EEPROM
	eeprom_buf[0] = VERSION_MAJOR; eeprom_buf[1] = VERSION_MINOR;
	if (ee24_write_word(EEPROM_ADDR_VERSION,(uint16_t *) &eeprom_buf) != true ) {
		term_print("Error: EEPROM write failed\r\n");
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

#ifdef USE_DISPLAY
  // TFT Display
  display_init(); // THIS FUNCTION MUST PRECEED ANY OTHER DISPLAY FUNCTION CALL.
#endif

  // Start CLI UART receive via interrupt
  if (HAL_UART_Receive_IT(&CLI_UART, (uint8_t*)&cli_rx_byte, 1) != HAL_OK) {
    Error_Handler();
  }
#ifdef USE_WIFI
  // Start ESP UART receive via interrupt
  if (HAL_UART_Receive_IT(&ESP_UART, (uint8_t*)&esp_rx_byte, 1) != HAL_OK) {
      Error_Handler();
  }
#endif

  // Start Timer for ADC readings
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
     Error_Handler();
  }

  // Start ADCs
  start_adcs();

#ifdef USE_DISPLAY
  display_splash_screen();
  display_splash_ticks = HAL_GetTick() + SPLASH_SCREEN_TIME;
#endif

  // Startup message
  sprintf(msg_buf, "\r\n%s V%d.%02d\r\n%s\r\n",  product_msg ,VERSION_MAJOR, VERSION_MINOR, copyright_msg);
  if (HAL_UART_Transmit(&CLI_UART, (uint8_t*)msg_buf, strlen(msg_buf), 1000) != HAL_OK) {
	  Error_Handler();
  }
  // Show active TIM2 configuration (for 25us ADC trigger)
  term_print("TIM2 ARR = %d\r\n",TIM2->ARR);

#ifdef USE_WIFI
  // Enable ESP 01
  HAL_GPIO_WritePin (ESP01_EN_GPIO_Port, ESP01_EN_Pin, GPIO_PIN_SET);
  // Perform reset
  HAL_GPIO_WritePin (ESP01_RST_GPIO_Port, ESP01_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin (ESP01_RST_GPIO_Port, ESP01_RST_Pin, GPIO_PIN_SET);

  // Get ESP version info
  /*sprintf(msg_buf, "AT+GMR\r\n");
  if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)msg_buf, strlen(msg_buf), 1000) != HAL_OK) {
  	  Error_Handler();
  }*/
#endif

  if (!ee24_isConnected()) {
	  term_print("Error: EEPROM not found\r\n");
  } else {
	  if (ee24_read_word(EEPROM_ADDR_VERSION, (uint16_t *) &eeprom_buf) != true) {
		  term_print("Error: EEPROM read error\r\n");
	  } else {
		term_print("EEPROM Version: V%d.%02d\r\n", eeprom_buf[0], eeprom_buf[1]);
		if ((eeprom_buf[0] == 0xFF) && (eeprom_buf[1] == 0xFF)) {		// new/blank EEPROM
			eeprom_buf[0] = VERSION_MAJOR; eeprom_buf[1] = VERSION_MINOR;
			if (ee24_write_word(EEPROM_ADDR_VERSION,(uint16_t *) &eeprom_buf) != true ) {
				term_print("Error: EEPROM write failed\r\n");
			}
		}
		// Detect version change
		if ((eeprom_buf[0]!=VERSION_MAJOR) || (eeprom_buf[0]!=VERSION_MINOR)) {
			version_change(eeprom_buf[0], eeprom_buf[1]);
		}
	  }
  }
  /*
  eeprom_buf[0] = 0x33;
  ee24_write_byte(0x01, (uint8_t *) eeprom_buf);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  next_process_time = HAL_GetTick() + PROCESS_INTERVAL;
  //term_print("current: %lu next: %lu\r\n", HAL_GetTick(), next_process_time);
  next_measurement_time = HAL_GetTick() + MEASUREMENT_INTERVAL;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	now_ticks = HAL_GetTick();
	// look for ticks overrun
	if (now_ticks < last_ticks) {
		next_process_time = now_ticks + PROCESS_INTERVAL;
		if (display_off_ticks) { display_off_ticks = now_ticks + DISPLAY_TIMEOUT; }
		display_update_ticks = now_ticks + DISPLAY_UPDATE_TIME;
		next_measurement_time = now_ticks + MEASUREMENT_INTERVAL;
	}
	last_ticks = now_ticks;		// store for compare in next iteration

	// perform measurements
	if ( now_ticks >= next_measurement_time ) {
		next_measurement_time += MEASUREMENT_INTERVAL;
		calc_measurements();
#ifdef USE_DISPLAY
		if ((HAL_GPIO_ReadPin(DISPL_LED_GPIO_Port, DISPL_LED_Pin) == GPIO_PIN_SET) && (display_screen)) {
			display_update_meter(display_screen);
		}
#endif
	}

	// process slow tasks
	if ( now_ticks >= next_process_time ) {
		next_process_time = now_ticks + PROCESS_INTERVAL;

		// clear splash screen
		if (display_splash_ticks) {
			if (now_ticks >= display_splash_ticks) {
				display_splash_ticks = 0;
#ifdef USE_DISPLAY
				//display_meter_mask();
				display_screen = 1;		// set to main screen
				display_off_ticks = now_ticks + DISPLAY_TIMEOUT;
#endif
			}
		}
		/*else {
		// Meter display update
			if (now_ticks >= display_update_ticks) {
				display_update_ticks = now_ticks + DISPLAY_UPDATE_TIME;
				// Don't update unless the display back light is on
				if (HAL_GPIO_ReadPin(DISPL_LED_GPIO_Port, DISPL_LED_Pin) == GPIO_PIN_SET) {
					//display_update_meter();
				}
			}
		}*/

		// Handle CLI UART communication
		if (cli_rx_cmd_ready) {
			if (esp_mode) {
				if (cli_rx_buff[0] == '~') { // cancel ESP mode
					esp_mode = 0;
					term_print("\r\nESP mode deactivated\r\n");
				} else {
					sprintf(msg_buf, "%s\r\n", cli_rx_buff);	// send command line to ESP01
					if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)msg_buf, strlen(msg_buf), 1000) != HAL_OK) {
						Error_Handler();
					}
					term_print("%s", msg_buf);
				}
			} else {
				CMD_Handler((uint8_t*)cli_rx_buff);
			}
			cli_rx_count = 0;
			cli_rx_cmd_ready = 0;
		}

#ifdef USE_WIFI
		// Handle ESP UART communication
		if (esp_rx_count > 0) {		// do we have any RX data from the ESP?
			if (esp_rx_count_last != esp_rx_count) { 	// has the RX count changed since last iteration?
				esp_rx_count_last = esp_rx_count;		// yes -> update last count, RX not finished yet
			} else {	// count hasn't changed since last iteration, we assume RX is completed
				if (!esp_mode) {
					wifi_handle_esp_rx_data();
				} else {		// ESP in terminal mode
					esp_rx_buf[esp_rx_count] = 0;	// Set EOS
					term_print("%s", esp_rx_buf);
				}
				esp_rx_count = 0;
				esp_rx_count_last = esp_rx_count;
			}
		}
#endif		// USE_WIFI

		if (adc_restart) {
		  adc_restart = 0;
		  start_adcs();
		}

		if (new_time_period) {
		  // change timer period to new value
		  adjust_TIM2_period(new_time_period, 1);
		  new_time_period = 0;
		}

#ifdef USE_DISPLAY

		if (display_change) {
			display_screen++;
			if (display_screen > SCREEN_MAX) {
				display_screen = 1;
			}
			display_change = 0;
		}

		// display timeout
		if (display_off_ticks && (now_ticks >= display_off_ticks)) {
			Displ_BackLight('0');
	  		display_off_ticks = 0;
	  	}

		if (tft_display) {
			if (tft_display == 9) {
				term_print("Running TFT performance test ...\r\n");
				Displ_BackLight('1');
				Displ_TestAll();
				Displ_BackLight('0');
				term_print("....completed\r\n");
			} else {
				if (tft_display == 1) {
					Displ_BackLight('0');
				} else {
					Displ_BackLight('1');
					display_off_ticks = HAL_GetTick() + DISPLAY_TIMEOUT;
					display_update_mask();
				}
			}
		tft_display = 0;
		}

		if (display_activate) {		// set by touch screen or blue button
			display_activate = 0;
			display_update_mask();
			Displ_BackLight('1');
			display_off_ticks = HAL_GetTick() + DISPLAY_TIMEOUT;
		}

#endif

		}

		// Check if we have missed processing DMA data sets
		// This occurs if the main loop execution takes longer than 20ms (e.g. terminal output of lots of data)
		if ( (adc1_dma_l_count > 1) || (adc1_dma_h_count > 1) || (adc2_dma_l_count > 1) || (adc2_dma_h_count > 1)) {
			//term_print("Processing has missed data - %lu %lu %lu %lu\r\n", adc1_dma_l_count, adc1_dma_h_count, adc2_dma_l_count, adc2_dma_h_count);
			if (adc1_dma_l_count > 1) { adc1_dma_l_count = 1; }
			if (adc1_dma_h_count > 1) { adc1_dma_h_count = 1; }
			if (adc2_dma_l_count > 1) { adc2_dma_l_count = 1; }
			if (adc2_dma_h_count > 1) { adc2_dma_h_count = 1; }
		}

		// Process DMA buffers
		if (adc1_dma_l_count > 0) {
			if (calc_process_dma_buffer(0,ADC1_IDX) != 0) {
				term_print("Processing ADC1 DMA 1st half failed\r\n");
			}
			adc1_dma_l_count--;
		}
		if (adc1_dma_h_count > 0) {
			if (calc_process_dma_buffer(1,ADC1_IDX) != 0) {
				term_print("Processing ADC1 DMA 2nd half failed\r\n");
			}
			adc1_dma_h_count--;
		}
		if (adc2_dma_l_count > 0) {
			if (calc_process_dma_buffer(0,ADC2_IDX) != 0) {
			term_print("Processing ADC2 DMA 1st half failed\r\n");
			}
			adc2_dma_l_count--;
		}
		if (adc2_dma_h_count > 0) {
			if (calc_process_dma_buffer(1,ADC2_IDX) != 0) {
			term_print("Processing ADC2 DMA 2nd half failed\r\n");
			}
			adc2_dma_h_count--;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2286;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DISPL_LED_Pin|DISPL_DC_Pin|DEBUG_Pin
                          |ESP01_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISPL_CS_GPIO_Port, DISPL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPL_RST_Pin|ESP01_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DISPL_LED_Pin DEBUG_Pin ESP01_EN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DISPL_LED_Pin|DEBUG_Pin|ESP01_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DISPL_DC_Pin */
  GPIO_InitStruct.Pin = DISPL_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DISPL_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DISPL_CS_Pin */
  GPIO_InitStruct.Pin = DISPL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DISPL_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_CS_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TOUCH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPL_RST_Pin ESP01_RST_Pin */
  GPIO_InitStruct.Pin = DISPL_RST_Pin|ESP01_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	ee24_lock = 0;
//	my_printf("HAL_I2C_MemTxCpltCallback");

}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	ee24_lock = 0;
	ee24_read_done = true;
//	my_printf("HAL_I2C_MemRxCpltCallback");
}

// External GPIO Interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
	case GPIO_PIN_4:		// IRQ from Touch screen
		display_activate = 1;
		break;
	case GPIO_PIN_13:		// Blue button on Development board
		display_change = 1;
		//display_activate = 1;
		break;
	}
}

// ADC conversion - DMA buffer 2nd half full
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1) {
		adc1_dma_h_count++;
	} else {
		adc2_dma_h_count++;
	}
}

// ADC conversion - DMA buffer 1st half full
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1) {
		adc1_dma_l_count++;
	} else {
		adc2_dma_l_count++;
	}
}

// ADC errors
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	int adc_num;
	if (hadc == &hadc1) {
	  adc_num = 1;
	} else {
	  adc_num = 2;
	}
	switch (hadc->ErrorCode) {
	case HAL_ADC_ERROR_NONE:
		term_print("ADC%d No Error (0x%08lx)\r\n", adc_num, hadc->ErrorCode);
		break;
	case HAL_ADC_ERROR_INTERNAL:
		term_print("ADC%d Internal Error (0x%08lx)\r\n", adc_num, hadc->ErrorCode);
		break;
	case HAL_ADC_ERROR_OVR:
		term_print("ADC%d Overrun Error (0x%08lx)\r\n", adc_num, hadc->ErrorCode);
		break;
	case HAL_ADC_ERROR_DMA:
		term_print("ADC%d DMA Error (0x%08lx)\r\n", adc_num, hadc->ErrorCode);
		break;
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 1)
	case HAL_ADC_ERROR_INVALID_CALLBACK:
		term_print("ADC%d Callback Error (0x%08lx)  [%d]\r\n", adc_num, hadc->ErrorCode);
		break;
#endif /* USE_HAL_ADC_REGISTER_CALLBACKS */
	default:
		term_print("ADC Unknown Error: 0x%08lx\r\n", hadc->ErrorCode);
	}
}

// UART has received data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// CLI command
	if (huart == &CLI_UART) {
		if (cli_rx_count >= sizeof(cli_rx_buff)) {
			cli_rx_count = 0;		// wrap back to start
		}
		if ( HAL_UART_Receive_IT(&CLI_UART, (uint8_t*)&cli_rx_byte, 1) == HAL_UART_ERROR_NONE) {
			// check for End of input (CR or LF)
			if ( (cli_rx_byte != 0x0A) && (cli_rx_byte !=  0x0D) ) {
				cli_rx_buff[cli_rx_count++] = cli_rx_byte;
			} else { // CR or LF detected
				if (cli_rx_count != 0) {	// a CR or LF without any pre-ceeding chars gets ignored
					cli_rx_cmd_ready = 1;
					cli_rx_buff[cli_rx_count++] = 0;	// end of string
				}
			}
		} // else { rx_error_count++; } // this should never happen
		return;
	}
#ifdef USE_WIFI
	// receive data from ESP
	if (huart == &ESP_UART) {
		if ( HAL_UART_Receive_IT(&ESP_UART, (uint8_t*)&esp_rx_byte, 1) == HAL_UART_ERROR_NONE) {
			if (esp_rx_count >= ESP_RX_BUF_SIZE) {		// prevent RX buffer overrun
				esp_rx_count = 0;
				esp_rx_buffer_overflow = true;				// set error flag
			}
			esp_rx_buf[esp_rx_count++] = esp_rx_byte;		// add received byte to RX buffer
		} else {	// this should never happen
			esp_rx_error_count++;
		}
	}
#endif
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
	printf("Error_Handler() called - program execution stopped");
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
