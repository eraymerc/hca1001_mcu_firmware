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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARR_VAL 8499
#define SWITCH_RATE 10000.0f  // Hz, TIM8 carrier frequency (170MHz / (2*(ARR_VAL+1)))

/* Center-aligned TIM8 raises an update event at both underflow and overflow
   (RepetitionCounter = 0), so the control ISR runs at twice the carrier rate. */
#define CONTROL_RATE  (2.0f * SWITCH_RATE)   // 20 kHz
#define CONTROL_DT    (1.0f / CONTROL_RATE)  // 50 us

/* ---------------------------------------------------------------------------
 * Open-loop V/f drive parameters -- ported from foc.hvr:
 *   module openloop  = open_loop<4, 95.5m, 0.5, 300.0>
 *   module generator = PWM_generator_3phase<2k, 310>
 *   module omegaGen  = reference_generator<>
 * ------------------------------------------------------------------------ */
#define POLE_PAIRS        4.0f    /**< p: mechanical -> electrical speed */
#define K_VF              0.0955f /**< V per electrical rad/s (= magnet flux) */
#define V_BOOST           0.5f    /**< phase amplitude floor near zero speed [V] */
#define V_MAX             300.0f  /**< phase peak voltage ceiling [V] */

/* reference_generator: mechanical speed ref, ramped from 0 over RAMP_TIME */
#define OMEGA_REF_FINAL   32.0f   /**< final mechanical speed reference [rad/s] */
#define OMEGA_REF_RAMP_S  10.0f    /**< ramp duration [s] */

/* PWM_generator_3phase */
#define VDC               310.0f  /**< DC bus voltage [V]; leg output is +-VDC/2 */
#define MODULATION_INDEX  1.0f    /**< SVPWM stays linear up to 2/sqrt(3) = 1.154 */

#define TWO_PI            6.2831853071795865f
#define SQRT3_OVER_2      0.8660254037844386f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
/* Live drive state, updated by the control ISR (handy as debugger watch items) */
volatile float ol_theta_e   = 0.0f;  /**< electrical angle [rad] */
volatile float ol_omega_ref = 0.0f;  /**< mechanical speed reference [rad/s] */
volatile float ol_v_amp     = 0.0f;  /**< V/f phase peak voltage [V] */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void execute_open_loop_control(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  /* Park all three legs at 50% before enabling the outputs */
  TIM8->CCR1 = (ARR_VAL + 1U) / 2U;
  TIM8->CCR2 = (ARR_VAL + 1U) / 2U;
  TIM8->CCR3 = (ARR_VAL + 1U) / 2U;

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

  /* Control loop tick: TIM8 update event (20 kHz, see CONTROL_RATE) */
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM8_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* All modulation work happens in the TIM8 update ISR. */
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
  htim8.Init.Period = 8499;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 192;
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
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);

    clock_fault_flag = 1;
}

/**
 * Turn one normalised leg reference (-1..+1, where +-1 is +-VDC/2) into a
 * center-aligned compare value. This replaces the triangleWave() comparator
 * of PWM_generator_3phase: the timer's up/down counter *is* the carrier, so
 * the software only has to place the crossing point.
 *   duty = (ARR/2) * (1 + m)   ->  m = 0 gives 50%
 */
static inline uint32_t leg_compare(float m)
{
    if (m >  1.0f) { m =  1.0f; }
    if (m < -1.0f) { m = -1.0f; }
    return (uint32_t)(((float)ARR_VAL * 0.5f) * (1.0f + m));
}

/**
 * Open-loop (V/f) three-phase drive. Port of the foc.hvr digital modules:
 * reference_generator -> open_loop -> PWM_generator_3phase, all folded into
 * one routine. Called from the TIM8 update ISR at CONTROL_RATE, so the
 * simulation's `dt` becomes CONTROL_DT and `time` becomes an accumulator.
 *
 * There is no current or position feedback -- the electrical angle is
 * integrated from the speed reference and the voltage amplitude follows the
 * V/f profile.
 */
static void execute_open_loop_control(void)
{
    static float theta    = 0.0f;  /**< electrical angle [rad], `state double theta` */
    static float run_time = 0.0f;  /**< seconds since start, stands in for `time` */

    /* --- reference_generator: 20 rad/s, ramped up over the first 2 s ------ */
    float omega_ref = OMEGA_REF_FINAL;
    if (run_time < OMEGA_REF_RAMP_S && run_time > 1.0f)
    {
        omega_ref = (OMEGA_REF_FINAL / OMEGA_REF_RAMP_S) * run_time;
        run_time += CONTROL_DT;   /* frozen once the ramp is done -- keeps float precision */
    }

    /* --- open_loop 1: synchronous angle integration ---------------------- */
    float omega_e_ref = POLE_PAIRS * omega_ref;

    theta += omega_e_ref * CONTROL_DT;
    if (theta >  TWO_PI) { theta -= TWO_PI; }
    if (theta <    0.0f) { theta += TWO_PI; }

    /* --- open_loop 2: V/f profile ---------------------------------------- */
    float w_abs = fabsf(omega_e_ref);
    float v_amp = K_VF * w_abs + V_BOOST;
    if (v_amp > V_MAX) { v_amp = V_MAX; }

    /* --- open_loop 3: desired dq voltage vector -------------------------- */
    /* No current loop, so Vd/Vq are set directly instead of coming from a PI.
       Vd < 0 would give field weakening / reluctance torque on an IPMSM;
       Vd = 0 is pure q-axis drive. */
    float Vd_des = 0.0f;
    float Vq_des = v_amp;
    if(run_time < 1.0f){
      Vd_des = 30.0f;
      Vq_des = 0.0f;
    }


    /* --- open_loop 4: inverse Park (dq -> alpha/beta) -------------------- */
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    float v_alpha = Vd_des * cos_t - Vq_des * sin_t;
    float v_beta  = Vd_des * sin_t + Vq_des * cos_t;

    /* --- open_loop 5: inverse Clarke (alpha/beta -> abc) ----------------- */
    float ref_a = v_alpha;
    float ref_b = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float ref_c = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    /* --- PWM_generator_3phase 1: min-max zero-sequence injection --------- */
    float v_max_ph = ref_a;
    if (ref_b > v_max_ph) { v_max_ph = ref_b; }
    if (ref_c > v_max_ph) { v_max_ph = ref_c; }

    float v_min_ph = ref_a;
    if (ref_b < v_min_ph) { v_min_ph = ref_b; }
    if (ref_c < v_min_ph) { v_min_ph = ref_c; }

    /* Adding the common-mode term turns SPWM into SVPWM (saddle wave) */
    float v_offset = -0.5f * (v_max_ph + v_min_ph);

    /* --- PWM_generator_3phase 2: normalise to +-1 ------------------------ */
    /* A leg swings +-VDC/2, so the divisor must be VDC/2 as well -- otherwise
       the modulator gain and the inverter gain disagree. */
    const float inv_v_half = MODULATION_INDEX / (VDC * 0.5f);

    float m_a = (ref_a + v_offset) * inv_v_half;
    float m_b = (ref_b + v_offset) * inv_v_half;
    float m_c = (ref_c + v_offset) * inv_v_half;

    /* --- PWM_generator_3phase 3: carrier comparison (in hardware) -------- */
    TIM8->CCR1 = leg_compare(m_a);
    TIM8->CCR2 = leg_compare(m_b);
    TIM8->CCR3 = leg_compare(m_c);

    ol_theta_e   = theta;
    ol_omega_ref = omega_ref;
    ol_v_amp     = v_amp;
}

/** TIM8 update event -- the control tick (see USER CODE 2 for the NVIC setup). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        execute_open_loop_control();
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
