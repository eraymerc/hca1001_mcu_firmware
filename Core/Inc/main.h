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

/** Samples carried by one AdcStreamFrame_t. */
#define STREAM_BATCH_SAMPLES 8U

/**
 * @brief Binary frame sent over LPUART1 for ADC1/HCA live monitoring.
 *
 * Carries STREAM_BATCH_SAMPLES samples, not one. Sending a frame per sample
 * cost 19 bytes for 8 bytes of payload, and at the 5kHz stream rate that
 * 95,000 B/s exceeded what the ST-LINK's virtual COM port would carry: roughly
 * one sample in five went missing, and the host -- which derives frequency from
 * an assumed sample rate -- read a 50Hz fundamental as 62.5Hz.
 *
 * Batching amortises the 11 bytes of framing over 8 samples: 75 bytes per
 * batch is 9.4 B/sample, so 5kHz now needs 46,900 B/s, half of what it did. It
 * also drops the transmit rate from 5,000 to 625 frames/s, which cuts the time
 * the main loop spends inside the blocking HAL_UART_Transmit by the same
 * factor.
 *
 * Layout is fixed and packed so the Python host can parse it with
 * struct.unpack('<BBII' + 'ff'*8 + 'B', ...) without any padding surprises.
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;         /**< 0xA5 */
    uint8_t  sync1;         /**< 0x5A */
    uint32_t seq;           /**< Batch sequence number, increments every pushed frame */
    uint32_t timestamp_ms;  /**< HAL_GetTick() when samples[0] was taken */
    struct {
        float voltage;      /**< ADC1 channel, scaled to actual sense volts */
        float error;        /**< HCA control loop error signal (r_t - measured) */
    } samples[STREAM_BATCH_SAMPLES];
    uint8_t  checksum;      /**< 8-bit additive checksum over seq..samples */
} AdcStreamFrame_t;

/* The host hard-codes these sizes in its struct format strings (python_gui/
 * protocol.py). Catch any padding or layout drift here rather than as garbled
 * samples on the wire. */
_Static_assert(sizeof(AdcStreamFrame_t) == 75, "AdcStreamFrame_t must stay 75 bytes");

/**
 * @brief Coefficient report frame sent over LPUART1, one per active HCA channel.
 *
 * Sent in reply to STREAM_CMD_GET_COEFF (all channels) and as an echo of a
 * STREAM_CMD_SET_COEFF that was applied (just the channel that changed), so
 * the host always displays gains the controller is actually running.
 *
 * Uses a different second sync byte (COEFF_SYNC1) than the ADC stream frame,
 * which lets the host demultiplex the two on one link: it resyncs on
 * STREAM_SYNC0 and then picks the frame length from the byte that follows.
 *
 * Packed for struct.unpack('<BBBBBffffB', ...) on the host -- 22 bytes.
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;         /**< 0xA5 */
    uint8_t  sync1;         /**< 0x5B */
    uint8_t  index;         /**< Channel slot, 0-based */
    uint8_t  count;         /**< Number of active channels, so the host knows when it has them all */
    uint8_t  order;         /**< Harmonic order of this channel */
    float    kp_real;       /**< Complex proportional gain, real part */
    float    kp_imag;       /**< Complex proportional gain, imaginary part */
    float    ki_real;       /**< Complex integral gain, real part */
    float    ki_imag;       /**< Complex integral gain, imaginary part */
    uint8_t  checksum;      /**< 8-bit additive checksum over index..ki_imag */
} HcaCoeffFrame_t;

_Static_assert(sizeof(HcaCoeffFrame_t) == 22, "HcaCoeffFrame_t must stay 22 bytes");

/**
 * @brief Reference-multiplier report sent over LPUART1.
 *
 * Sent in reply to STREAM_CMD_GET_REF and as an echo of a STREAM_CMD_SET_REF,
 * carrying the value the device *actually* runs after clamping -- the host
 * cannot know the limit on its own because it depends on IS_OPENLOOP, so it
 * displays what comes back here rather than what it asked for.
 *
 * Third sync byte variant (REF_SYNC1), demultiplexed the same way as
 * HcaCoeffFrame_t. Packed for struct.unpack('<BBfBfB', ...) -- 12 bytes.
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;      /**< 0xA5 */
    uint8_t  sync1;      /**< 0x5C */
    float    value;      /**< Reference multiplier in effect, after clamping */
    uint8_t  open_loop;  /**< 1 when the firmware was built open loop */
    float    limit;      /**< Largest value this build accepts */
    uint8_t  checksum;   /**< 8-bit additive checksum over value..limit */
} HcaRefFrame_t;

_Static_assert(sizeof(HcaRefFrame_t) == 12, "HcaRefFrame_t must stay 12 bytes");

/**
 * @brief Sensor-calibration report sent over LPUART1.
 *
 * Sent in reply to STREAM_CMD_CALIBRATE, STREAM_CMD_GET_CAL and
 * STREAM_CMD_CAL_DEFAULT. Carries both the coefficients now in force and the
 * raw measurements they were derived from, so a calibration that ran against
 * a dead output or a disconnected sensor is visible on the host rather than
 * silently accepted.
 *
 * Packed for struct.unpack('<BBBffffffB', ...) -- 28 bytes.
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;          /**< 0xA5 */
    uint8_t  sync1;          /**< 0x5D */
    uint8_t  status;         /**< CAL_STATUS_*, see below */
    float    vdc;            /**< DC bus voltage the host supplied */
    float    gain;           /**< Sensor gain correction now in force */
    float    offset;         /**< Sensor offset correction now in force, volts */
    float    raw_peak;       /**< Uncalibrated sensed fundamental peak, volts */
    float    raw_dc;         /**< Uncalibrated sensed DC component, volts */
    float    expected_peak;  /**< Fundamental peak the modulator actually commanded, volts */
    uint8_t  checksum;       /**< 8-bit additive checksum over status..expected_peak */
} HcaCalFrame_t;

_Static_assert(sizeof(HcaCalFrame_t) == 28, "HcaCalFrame_t must stay 28 bytes");

/** @name Calibration outcome codes carried in HcaCalFrame_t.status
 *  Anything but CAL_STATUS_OK / _RESTORED leaves the coefficients untouched. */
/**@{*/
#define CAL_STATUS_OK           0U  /**< Ran, passed its checks, applied */
#define CAL_STATUS_REPORT       1U  /**< Nothing ran; this is just the current state */
#define CAL_STATUS_RESTORED     2U  /**< Build-time defaults put back */
#define CAL_STATUS_BUSY         3U  /**< A calibration was already in progress */
#define CAL_STATUS_BAD_VDC      4U  /**< Vdc outside CAL_VDC_MIN..CAL_VDC_MAX */
#define CAL_STATUS_NO_SIGNAL    5U  /**< Commanded or sensed fundamental too small to divide by */
#define CAL_STATUS_OUT_OF_RANGE 6U  /**< Result implausible, see CAL_GAIN_MIN/MAX */
/**@}*/

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define STREAM_SYNC0        0xA5U
#define STREAM_SYNC1        0x5AU

/** Single-byte host->device commands received over USART2 */
#define STREAM_CMD_START     'S'   /**< Start streaming frames */
#define STREAM_CMD_STOP      'X'   /**< Stop streaming frames */
#define STREAM_CMD_PING      'P'   /**< Request identification reply */
#define STREAM_CMD_GET_COEFF 'G'   /**< Report every channel's Kp/Ki as HcaCoeffFrame_t */
#define STREAM_CMD_SET_COEFF 'C'   /**< Followed by COEFF_CMD_PAYLOAD_LEN payload bytes, see below */
#define STREAM_CMD_RESET_INT 'R'   /**< Clear every channel's integrator and the disperser window */
#define STREAM_CMD_SET_REF   'M'   /**< Followed by REF_CMD_PAYLOAD_LEN payload bytes, see below */
#define STREAM_CMD_GET_REF   'N'   /**< Report the reference multiplier as HcaRefFrame_t */
#define STREAM_CMD_CALIBRATE 'K'   /**< Followed by CAL_CMD_PAYLOAD_LEN payload bytes, see below */
#define STREAM_CMD_GET_CAL   'Q'   /**< Report the sensor calibration as HcaCalFrame_t */
#define STREAM_CMD_CAL_DEFAULT 'D' /**< Restore the build-time sensor calibration */

/** Second sync byte of HcaCoeffFrame_t; distinguishes it from an AdcStreamFrame_t */
#define COEFF_SYNC1         0x5BU

/** Second sync byte of HcaRefFrame_t */
#define REF_SYNC1           0x5CU

/** Second sync byte of HcaCalFrame_t */
#define CAL_SYNC1           0x5DU

/**
 * Payload following a STREAM_CMD_SET_COEFF byte, little-endian and packed:
 *   uint8_t order; float kp_real, kp_imag, ki_real, ki_imag; uint8_t checksum;
 * The checksum is the 8-bit additive sum over the preceding 17 bytes.
 */
#define COEFF_CMD_PAYLOAD_LEN  18U

/** A half-sent SET_COEFF payload is abandoned after this long, so a host that
 *  dies mid-command cannot leave the parser swallowing later S/X/P bytes. */
#define COEFF_CMD_TIMEOUT_MS   100U

/**
 * Payload following a STREAM_CMD_SET_REF byte, little-endian and packed:
 *   float multiplier; uint8_t checksum;
 * The checksum is the 8-bit additive sum over the preceding 4 bytes.
 */
#define REF_CMD_PAYLOAD_LEN    5U

/**
 * Payload following a STREAM_CMD_CALIBRATE byte, little-endian and packed:
 *   float vdc; uint8_t checksum;
 * vdc is the DC bus voltage, which only the operator knows -- it is what
 * turns the modulator's commanded duty into the volts it must have produced.
 */
#define CAL_CMD_PAYLOAD_LEN    5U

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
