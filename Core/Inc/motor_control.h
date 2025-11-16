/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   BTS7960 Motor Driver Control Library Header
  * @author  Auto-generated for Capstone Project
  * @date    2025-11-11
  ******************************************************************************
  *
  * Motor Configuration:
  * - 4 DC motors controlled by BTS7960 H-bridge drivers
  * - Each motor uses 2 PWM signals (RPWM + LPWM) for direction control
  * - PWM frequency: ~952 Hz (APB1/APB2 timer @ 8 MHz / 2 / 4200)
  *
  * Timer Mapping:
  * - TIM2 CH1 (PA0)  - Motor 1 RPWM (front-left?)
  * - TIM3 CH3 (PB0)  - Motor 2 RPWM
  * - TIM3 CH4 (PB1)  - Motor 3 RPWM
  * - TIM9 CH1 (PE5)  - Motor 4 RPWM (testing)
  * - TIM9 CH2 (PE6)  - Motor 4 LPWM (testing)
  *
  * Note: This is initial version for TIM9 PWM testing.
  *       Full 4-motor control requires additional GPIO for direction pins.
  *
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define MOTOR_COUNT 4
#define MOTOR_PWM_MAX 4199  // ARR value (100% duty) for motors 2,3,4
#define MOTOR_1_PWM_MAX 8299  // ARR value (100% duty) for motor 1 (higher voltage)

/* Motor IDs */
#define MOTOR_1 0  // TIM2 CH1
#define MOTOR_2 1  // TIM3 CH3
#define MOTOR_3 2  // TIM3 CH4
#define MOTOR_4 3  // TIM9 CH1+CH2 (for testing)

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE
} Motor_Direction_t;

typedef struct {
    TIM_HandleTypeDef *htim;     // Timer handle
    uint32_t channel_rpwm;       // PWM channel for RPWM (forward)
    uint32_t channel_lpwm;       // PWM channel for LPWM (reverse)
    uint16_t current_speed;      // Current PWM duty (0-4199)
    Motor_Direction_t direction; // Current direction
    bool is_initialized;         // Initialization flag
} Motor_TypeDef;

/* Exported variables --------------------------------------------------------*/
extern Motor_TypeDef motors[MOTOR_COUNT];

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize motor control library
  * @note   Call this after all timer init functions in main()
  * @retval None
  */
void Motor_Init(void);

/**
  * @brief  Stop all motors (emergency stop)
  * @retval None
  */
void Motor_Stop_All(void);
//void motor_slide_left(uint8_t speed);
//void motor_slide_right(uint8_t speed);

/**
  * @brief  Set motor speed and direction
  * @param  motor_id: Motor ID (0-3)
  * @param  speed: Speed percentage (-100 to +100)
  *                Negative = reverse, Positive = forward, 0 = stop
  * @retval None
  */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed);

/**
  * @brief  Set raw PWM duty cycle for motor
  * @param  motor_id: Motor ID (0-3)
  * @param  duty_rpwm: RPWM duty (0-4199)
  * @param  duty_lpwm: LPWM duty (0-4199)
  * @note   For advanced control, use Motor_SetSpeed() instead
  * @retval None
  */
void Motor_SetDuty_Raw(uint8_t motor_id, uint16_t duty_rpwm, uint16_t duty_lpwm);

/**
  * @brief  Move robot forward
  * @param  speed: Speed percentage (0-100)
  * @retval None
  */
void Motor_Forward(uint8_t speed);

/**
  * @brief  Move robot backward (reverse)
  * @param  speed: Speed percentage (0-100)
  * @retval None
  */
void Motor_Reverse(uint8_t speed);

/**
  * @brief  Turn robot left (differential drive)
  * @param  speed: Turn speed percentage (0-100)
  * @retval None
  */
void Motor_Turn_Left(uint8_t speed);

/**
  * @brief  Turn robot right (differential drive)
  * @param  speed: Turn speed percentage (0-100)
  * @retval None
  */
void Motor_Turn_Right(uint8_t speed);

/**
  * @brief  Rotate robot in place (spin left)
  * @param  speed: Rotation speed percentage (0-100)
  * @retval None
  */
void Motor_Rotate_Left(uint8_t speed);

/**
  * @brief  Rotate robot in place (spin right)
  * @param  speed: Rotation speed percentage (0-100)
  * @retval None
  */
void Motor_Rotate_Right(uint8_t speed);

/**
  * @brief  Get current motor speed
  * @param  motor_id: Motor ID (0-3)
  * @retval Speed as percentage (-100 to +100), 0 if invalid
  */
int16_t Motor_GetSpeed(uint8_t motor_id);

/**
  * @brief  Test function: Set TIM9 PWM for oscilloscope testing
  * @param  duty_percent: Duty cycle percentage (0-100)
  * @retval None
  */
void Motor_Test_TIM9_PWM(uint8_t duty_percent);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
