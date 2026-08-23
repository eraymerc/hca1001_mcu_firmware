/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include "hca_lib.h"
#include "unipolar_spwm_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARR_VAL 4249
#define SWITCH_RATE 20000.0f // Hz, TIM8 carrier frequency (170MHz / (2*(ARR_VAL+1)))

/** ADC ISR runs at 40kHz; only push every Nth sample -> 40kHz/8 = 5kHz stream rate.
 *  5kHz puts Nyquist at 2.5kHz, so the host FFT resolves harmonics up to H50 of a
 *  50Hz fundamental. Costs 19 bytes * 5000 Hz = 95,000 B/s on a link good for
 *  209,700 B/s (2,097,000 baud, 8N1), i.e. about 45% of the wire. */
#define ADC_STREAM_DECIMATION 8U

/** Streaming frame ring buffer depth (must be a power of two) */
/* Each frame now carries STREAM_BATCH_SAMPLES samples, so 64 frames is 512
 * samples -- ~102ms of slack at the 5kHz stream rate, more than the 256
 * single-sample frames it replaces, for 4.8KB of RAM. */
#define STREAM_FIFO_LEN  64U
#define STREAM_FIFO_MASK (STREAM_FIFO_LEN - 1U)

/** LPUART1 RX ring filled by DMA, drained in the main loop (must be a power of two).
 *
 *  Polling HAL_UART_Receive() a byte at a time cannot keep up here: one byte is
 *  4.77us at 2,097,000 baud, while the main loop can sit inside a blocking
 *  HAL_UART_Transmit for ~90us per stream frame and is preempted by the 40kHz
 *  ADC ISR every 25us. A single-byte command survives that (it just waits in
 *  RDR), but every byte after the first in a multi-byte command is overrun and
 *  lost -- which is why SET_COEFF payloads never completed. DMA takes the CPU
 *  out of the capture path entirely, so main-loop latency no longer matters.
 *
 *  64 bytes is over three SET_COEFF commands' worth of slack. */
#define UART_RX_DMA_LEN  64U
#define UART_RX_DMA_MASK (UART_RX_DMA_LEN - 1U)
#define MODULATION_INDEX 0.85f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_lpuart1_tx;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
volatile uint16_t adc1_raw;  // voltage (single ADC channel)
volatile HCA_Handle_t hca;   // HCA Handler type

/** Set/cleared by 'S'/'X' commands received over LPUART1 (see HandleStreamCommand) */
volatile uint8_t streaming_enabled = 0;

/* LPUART1 receive path: DMA writes here continuously, the main loop follows it.
 * hdma_lpuart1_rx itself is CubeMX-generated (LPUART1 -> DMA Settings in the
 * .ioc: DMA1 Channel2, circular, byte-wide). */
static uint8_t    uart_rx_dma[UART_RX_DMA_LEN];
static uint16_t   uart_rx_tail = 0;

/* Single-producer (ADC ISR) / single-consumer (main loop) ring buffer of frames */
static AdcStreamFrame_t  stream_fifo[STREAM_FIFO_LEN];
static volatile uint16_t stream_fifo_head = 0;
static volatile uint16_t stream_fifo_tail = 0;
static volatile uint32_t stream_seq = 0;
/* Set while a stream frame is in flight on DMA1_Channel3; cleared by the
 * transfer-complete callback, which is also where the FIFO tail advances. The
 * slot must stay untouched until then, since the DMA is reading straight out
 * of it. */
static volatile uint8_t  stream_tx_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void HandleStreamCommand(uint8_t cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  const float fundamental_freq = 50.0f;     // your output AC frequency, Hz
  const float switching_freq   = SWITCH_RATE;
  const uint8_t oversample_ratio = 2;       // 20kHz * 2 = 40kHz control loop (center-aligned TRGO fires twice/period)
  const float output_limit = 1.0f;          // matches USPWM's ±1.0 saturation

  HCA_Init(&hca,
          fundamental_freq,
          switching_freq,
          oversample_ratio,
          output_limit);

  Complex_t kp1 = {0.5f, -0.03f}; //real, complex
  Complex_t ki1 = {0.8407f, 0.0657f}; //real, complex
  
  Complex_t kp3 = {0.001f, -0.4f}; //real, complex
  Complex_t ki3 = {0.0508f, -0.8f}; //real, complex

  Complex_t kp5 = {0.010f, 0.01f}; //real, complex
  Complex_t ki5 = {0.01f, 0.02f}; //real, complex

  Complex_t kp7 = {0.001f, 0.001f}; //real, complex
  Complex_t ki7 = {0.25f, 3.005f}; //real, complex

  Complex_t kp9 = {0.001f, 0.01f}; //real, complex
  Complex_t ki9 = {0.5f, 3.0025f}; //real, complex

  Complex_t kp11 = {0.001f, 0.01f}; //real, complex
  Complex_t ki11 = {0.05f, 1.0025f}; //real, complex

  Complex_t kp13 = {0.001f, 0.01f}; //real, complex
  Complex_t ki13 = {0.05f, 1.0025f}; //real, complex

  Complex_t kp15 = {0.001f, 0.01f}; //real, complex
  Complex_t ki15 = {0.05f, 1.0025f}; //real, complex

  //Complex_t kp17 = {0.001f, 0.01f}; //real, complex
  //Complex_t ki17 = {0.05f, 1.0025f}; //real, complex

  HCA_Add_Channel(&hca, 1, kp1, ki1);  // Fundamental
  HCA_Add_Channel(&hca, 3, kp3, ki3);  
  HCA_Add_Channel(&hca, 5, kp5, ki5);  
  HCA_Add_Channel(&hca, 7, kp7, ki7);  
  HCA_Add_Channel(&hca, 9, kp9, ki9);  
  HCA_Add_Channel(&hca, 11, kp11, ki11);  
  HCA_Add_Channel(&hca, 13, kp13, ki13);
  HCA_Add_Channel(&hca, 15, kp15, ki15);
  //HCA_Add_Channel(&hca, 17, kp17, ki17);    
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1_raw, 1);

  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Everything DMA has written since the last pass. CNDTR counts down, so
     * the write position is the buffer length minus what is left to fill. */
    uint16_t rx_head = (uint16_t)(UART_RX_DMA_LEN
                                  - __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx));
    while (uart_rx_tail != rx_head)
    {
      HandleStreamCommand(uart_rx_dma[uart_rx_tail]);
      uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) & UART_RX_DMA_MASK);
    }

    /* DMA transmit is wired up (HAL_UART_TxCpltCallback below) but stays off
     * until the LPUART1 global interrupt is enabled in CubeMX: HAL signals a
     * DMA transmit's completion from the UART's TC interrupt, not the DMA's,
     * so without that NVIC entry gState never returns to READY and the stream
     * stops dead after one frame. Swap these two blocks once it is enabled.
     *
     *   if (!stream_tx_busy && (stream_fifo_tail != stream_fifo_head))
     *   {
     *     AdcStreamFrame_t *frame = &stream_fifo[stream_fifo_tail];
     *     if (HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)frame,
     *                               (uint16_t)sizeof(AdcStreamFrame_t)) == HAL_OK)
     *     {
     *       stream_tx_busy = 1;
     *     }
     *   }
     */
    if (stream_fifo_tail != stream_fifo_head)
    {
      AdcStreamFrame_t *frame = &stream_fifo[stream_fifo_tail];
      if (HAL_UART_Transmit(&hlpuart1, (uint8_t*)frame,
                            (uint16_t)sizeof(AdcStreamFrame_t), 5) == HAL_OK)
      {
        stream_fifo_tail = (uint16_t)((stream_fifo_tail + 1U) & STREAM_FIFO_MASK);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 2097000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  /* Circular: this never completes and never needs restarting. */
  if (HAL_UART_Receive_DMA(&hlpuart1, uart_rx_dma, UART_RX_DMA_LEN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END LPUART1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim8.Init.Period = 4249;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 68;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Stops switching when Clock Security System (CSS) detects a clock failure.
volatile uint8_t clock_fault_flag = 0;
void HAL_RCC_CSSCallback(void)
{
    // Force PWM outputs off immediately — this is the critical line
    __HAL_TIM_MOE_DISABLE(&htim8);

    // Optional but recommended: fully stop timer channels too
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);

    clock_fault_flag = 1;
}

#define ADC_VREF              3.3f
#define ADC_FULL_SCALE_CODES  2048.0f  // ADC1 is differential; signed code -2048..2047 spans -VREF..+VREF
#define V_PEAK_NOM            200.0f

/**
 * Sensor transfer function (measured/derived from the actual circuit):
 *   Vout_p = Vin * (250/22000) *  0.43 + 1.65V
 *   Vout_n = Vin * (250/22000) * -0.43 + 1.65V
 * Both differential pins are biased at 1.65V (VDDA/2) with the signal
 * riding symmetrically in opposite directions, so the differential the ADC
 * actually measures is:
 *   Vinp - Vinn = Vout_p - Vout_n = SENSOR_GAIN * Vin
 * where SENSOR_GAIN = 2 * (250/22000) * 0.43. The 1.65V bias on each pin
 * cancels out in the subtraction -- no bias removal needed in software.
 */
#define SENSOR_GAIN (2.0f * (250.0f / 44000.0f) * 0.4779f)
#define DC_CAL 21.5f
#define GAIN_CAL 172.0f/159.0f

/**
 * ADC1 in differential mode reports Vinp-Vinn as a 12-bit *straight offset
 * binary* code (0..4095, unsigned) -- NOT two's complement. Code 2048
 * (0x800) means Vinp-Vinn = 0V; 0 means -VREF; 4095 means ~+VREF. Center
 * it on zero by subtracting the half-scale offset.
 */
static inline int16_t DifferentialCode(uint16_t raw12)
{
    return (int16_t)raw12 - (int16_t)ADC_FULL_SCALE_CODES;
}

static inline float adcToVoltsActual(int16_t adc_signed){
    float v_adc = (float)adc_signed * (ADC_VREF / ADC_FULL_SCALE_CODES); // differential volts at the ADC pins
    return (v_adc / SENSOR_GAIN)*GAIN_CAL + DC_CAL;                                          // invert sensor formula -> HV line volts
}

static inline float normaliseVoltage(int16_t adc_signed){
    return adcToVoltsActual(adc_signed) / V_PEAK_NOM;       // normalize to ±1.0 like r_t
}

/**
 * @param adc_signed ADC1 differential reading, zero-centered (see DifferentialCode)
 */
static inline float Execute_HCA_Control(int16_t adc_signed, uint8_t update)
{
    static uint32_t step_fundamental = (uint32_t)((50.0f / (2.0f*SWITCH_RATE)) * 4294967296.0f);
    static uint32_t angle_fundamental = 0;

    uint32_t theta = angle_fundamental;
    float r_t = HCA_fastSin(theta)*0.8f;

    float error = r_t - (float)normaliseVoltage(adc_signed);
    float hca_out = HCA_Process(&hca, error);

    angle_fundamental += step_fundamental;

    if ((update & 0x1) == 0) {
      USPWM(htim8.Instance, hca_out, ARR_VAL, MODULATION_INDEX);  // modulation_index=1.0, already applied above
    }

    return error;
}

static uint8_t StreamChecksum(const AdcStreamFrame_t *f)
{
    const uint8_t *p = (const uint8_t*)&f->seq;
    const uint16_t len = (uint16_t)(sizeof(AdcStreamFrame_t) - sizeof(f->sync0)
                                     - sizeof(f->sync1) - sizeof(f->checksum));
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += p[i];
    }
    return sum;
}

/* Batch under construction. Only the ISR touches it, so it needs no guarding. */
static AdcStreamFrame_t stream_accum;
static uint8_t          stream_accum_count = 0;

/**
 * Producer side (called from ADC ISR). Accumulates STREAM_BATCH_SAMPLES samples
 * and pushes them as one frame; drops the whole batch if the FIFO is full.
 *
 * seq is advanced only on a successful push, which is what lets the host tell
 * the two loss modes apart: a FIFO overflow here leaves seq contiguous but
 * slower than wall-clock, while bytes lost on the wire leave holes in seq.
 */
static inline void PushStreamFrame(float voltage, float error)
{
    if (stream_accum_count == 0U) {
        stream_accum.timestamp_ms = HAL_GetTick();
    }

    stream_accum.samples[stream_accum_count].voltage = voltage;
    stream_accum.samples[stream_accum_count].error   = error;

    if (++stream_accum_count < STREAM_BATCH_SAMPLES) {
        return; // batch still filling
    }
    stream_accum_count = 0;

    uint16_t next_head = (uint16_t)((stream_fifo_head + 1U) & STREAM_FIFO_MASK);
    if (next_head == stream_fifo_tail) {
        return; // consumer (UART) can't keep up, drop this batch
    }

    AdcStreamFrame_t *f = &stream_fifo[stream_fifo_head];
    *f              = stream_accum;
    f->sync0        = STREAM_SYNC0;
    f->sync1        = STREAM_SYNC1;
    f->seq          = stream_seq++;
    f->checksum     = StreamChecksum(f);

    stream_fifo_head = next_head;
}

// Manages the HCA loop 40kHz sample rate and 5kHz telemetry rate
volatile uint32_t tick_counter = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        int16_t v_adc_signed = DifferentialCode(adc1_raw); // voltage, differential

        tick_counter++;

        float error = Execute_HCA_Control(v_adc_signed, tick_counter);

        if (streaming_enabled && ((tick_counter % ADC_STREAM_DECIMATION) == 0U))
        {
            float voltage_v = adcToVoltsActual(v_adc_signed);
            PushStreamFrame(voltage_v, error);
        }
    }
}

/**
 * Stream frame finished on the wire. Only the stream path uses DMA, so this
 * cannot be reached by the command replies below.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        stream_fifo_tail = (uint16_t)((stream_fifo_tail + 1U) & STREAM_FIFO_MASK);
        stream_tx_busy = 0;
    }
}

/**
 * Command replies still transmit in blocking mode -- they are rare and
 * user-triggered. HAL refuses a transmit while gState is BUSY_TX, so wait out
 * any stream frame first rather than letting the reply be dropped. Bounded by
 * one frame time, ~360us at 2,097,000 baud.
 */
static void StreamTxWaitIdle(void)
{
    while (stream_tx_busy)
    {
        /* the DMA1_Channel3 IRQ clears this */
    }
}

/* ---- SET_COEFF payload reassembly ----------------------------------------
 * The main loop hands us one byte at a time, so a multi-byte command has to be
 * collected across iterations. Doing a blocking 18-byte read instead would
 * stall the stream pump for as long as the host took to finish the command. */
static uint8_t  coeff_rx[COEFF_CMD_PAYLOAD_LEN];
static uint8_t  coeff_rx_len = 0;
static uint8_t  coeff_rx_active = 0;
static uint32_t coeff_rx_started_ms = 0;

static uint8_t CoeffChecksum(const uint8_t *p, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += p[i];
    }
    return sum;
}

/** Emit one channel's gains. Blocking, but only ~105us at 2,097,000 baud. */
static void SendCoeffFrame(uint8_t index, uint8_t count, const HCA_Channel_t *ch)
{
    HcaCoeffFrame_t f;
    f.sync0   = STREAM_SYNC0;
    f.sync1   = COEFF_SYNC1;
    f.index   = index;
    f.count   = count;
    f.order   = ch->harmonic_order;
    f.kp_real = ch->Kp.real;
    f.kp_imag = ch->Kp.imag;
    f.ki_real = ch->Ki.real;
    f.ki_imag = ch->Ki.imag;
    /* Covers index..ki_imag: everything but the two sync bytes and itself. */
    f.checksum = CoeffChecksum((const uint8_t*)&f.index,
                               (uint16_t)(sizeof(f) - 3U));

    StreamTxWaitIdle();
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&f, (uint16_t)sizeof(f), 10);
}

static void SendAllCoeffFrames(void)
{
    HCA_Handle_t *h = (HCA_Handle_t*)&hca;
    uint8_t count = h->active_channel_count;
    for (uint8_t i = 0; i < count; i++) {
        SendCoeffFrame(i, count, &h->channels[i]);
    }
}

/** Validate a fully received SET_COEFF payload and push it into the controller. */
static void ApplyCoeffCommand(void)
{
    if (CoeffChecksum(coeff_rx, COEFF_CMD_PAYLOAD_LEN - 1U)
            != coeff_rx[COEFF_CMD_PAYLOAD_LEN - 1U]) {
        return; // corrupted on the wire; the host re-sends after its ack times out
    }

    uint8_t   order = coeff_rx[0];
    Complex_t kp, ki;
    /* memcpy rather than a cast: coeff_rx is byte-aligned and the FPU's VLDR
     * faults on an unaligned float load. */
    memcpy(&kp.real, &coeff_rx[1],  sizeof(float));
    memcpy(&kp.imag, &coeff_rx[5],  sizeof(float));
    memcpy(&ki.real, &coeff_rx[9],  sizeof(float));
    memcpy(&ki.imag, &coeff_rx[13], sizeof(float));

    /* Gains only -- this never creates a channel. The set of active channels is
     * fixed at boot by the HCA_Add_Channel calls in main(), so the ISR's
     * per-sample workload cannot change underneath it while running.
     *
     * Safe from the main loop; see the thread-safety note on HCA_UpdateChannel.
     * An order with no channel is ignored there, and the absent echo below is
     * what tells the host the write did not land. */
    HCA_Handle_t *h = (HCA_Handle_t*)&hca;
    HCA_UpdateChannel(h, order, kp, ki);

    for (uint8_t i = 0; i < h->active_channel_count; i++) {
        if (h->channels[i].harmonic_order == order) {
            SendCoeffFrame(i, h->active_channel_count, &h->channels[i]);
            return;
        }
    }
}

/** Command byte polled from LPUART1 in the main loop (see HAL_UART_Receive call in USER CODE 3). */
static void HandleStreamCommand(uint8_t cmd)
{
    if (coeff_rx_active)
    {
        if ((HAL_GetTick() - coeff_rx_started_ms) > COEFF_CMD_TIMEOUT_MS) {
            coeff_rx_active = 0;  // stale payload; fall through and read this byte as a command
        } else {
            coeff_rx[coeff_rx_len++] = cmd;
            if (coeff_rx_len >= COEFF_CMD_PAYLOAD_LEN) {
                coeff_rx_active = 0;
                coeff_rx_len = 0;
                ApplyCoeffCommand();
            }
            return;
        }
    }

    switch (cmd)
    {
        case STREAM_CMD_GET_COEFF:
            SendAllCoeffFrames();
            break;

        case STREAM_CMD_RESET_INT:
            /* Zeroes each channel's PI integrator and disperser accumulator and
             * clears the shared delay line. Runs from the main loop while the
             * ISR keeps calling HCA_Process, so the controller sees the state
             * vanish mid-sample -- that is the point of the command, but it does
             * put a transient on the output. The ~16KB memset costs roughly one
             * 40kHz ISR period.
             *
             * Gains are untouched, so echoing them back doubles as the ack. */
            HCA_reset_accumulators((HCA_Handle_t*)&hca);
            SendAllCoeffFrames();
            break;

        case STREAM_CMD_SET_COEFF:
            coeff_rx_active = 1;
            coeff_rx_len = 0;
            coeff_rx_started_ms = HAL_GetTick();
            break;

        case STREAM_CMD_START:
            streaming_enabled = 1;
            break;

        case STREAM_CMD_STOP:
            streaming_enabled = 0;
            break;

        case STREAM_CMD_PING:
            StreamTxWaitIdle();
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)STREAM_PING_REPLY,
                               (uint16_t)(sizeof(STREAM_PING_REPLY) - 1U), 10);
            break;

        default:
            break; // ignore unknown bytes (e.g. line endings from a terminal)
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
  while (1)
  {
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
