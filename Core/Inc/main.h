/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_nucleo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/**
 * @brief Binary frame sent over USART2 (DMA) for ADC1/HCA live monitoring.
 *
 * Full float precision, one frame per sample -- affordable again now that
 * the link runs at 2,000,000 baud (200,000 B/s capacity). Decimated to
 * 1kHz from the 40kHz control loop (see ADC_STREAM_DECIMATION in main.c);
 * 19 bytes * 1000 Hz = 19,000 B/s, comfortably within budget.
 *
 * Layout is fixed and packed so the Python host can parse it with
 * struct.unpack('<BBIIffB', ...) without any padding surprises.
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;         /**< 0xA5 */
    uint8_t  sync1;         /**< 0x5A */
    uint32_t seq;           /**< Frame sequence number, increments every pushed frame */
    uint32_t timestamp_ms;  /**< HAL_GetTick() at push time */
    float    voltage;       /**< ADC1 channel, scaled to actual sense volts */
    float    error;         /**< HCA control loop error signal (r_t - measured) */
    uint8_t  checksum;      /**< 8-bit additive checksum over seq..error */
} AdcStreamFrame_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define STREAM_SYNC0        0xA5U
#define STREAM_SYNC1        0x5AU

/** Single-byte host->device commands received over USART2 */
#define STREAM_CMD_START     'S'   /**< Start streaming frames */
#define STREAM_CMD_STOP      'X'   /**< Stop streaming frames */
#define STREAM_CMD_PING      'P'   /**< Request identification reply */

#define STREAM_PING_REPLY    "HCA1001_ADC_STREAM_V1\n"

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/** Set by 'S'/'X' commands received over USART2 (see HAL_UART_RxCpltCallback). */
extern volatile uint8_t streaming_enabled;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
