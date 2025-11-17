/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   BTS7960 Motor Driver Control Library Implementation
  * @author  Auto-generated for Capstone Project
  * @date    2025-11-11
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

/* Private variables ---------------------------------------------------------*/
Motor_TypeDef motors[MOTOR_COUNT];

/* Private function prototypes -----------------------------------------------*/
static uint16_t speed_to_duty(int16_t speed);
static uint16_t speed_to_duty_motor(uint8_t motor_id, int16_t speed);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize motor control library
  * @note   IMPORTANT: Motor kiri (C & D) polaritas terbalik!
  *         - Motor Kanan (A & B): RPWM = maju, LPWM = mundur (normal)
  *         - Motor Kiri (C & D): LPWM = maju, RPWM = mundur (inversi)
  * @retval None
  */
void Motor_Init(void) {
    // Motor 1 (B - Kanan Depan): TIM9 CH1+CH2 (PE5/PE6) - Full H-bridge control
    motors[MOTOR_1].htim = &htim9;
    motors[MOTOR_1].channel_rpwm = TIM_CHANNEL_1;  // PE5 - RPWM (forward)
    motors[MOTOR_1].channel_lpwm = TIM_CHANNEL_2;  // PE6 - LPWM (reverse)
    motors[MOTOR_1].current_speed = 0;
    motors[MOTOR_1].direction = MOTOR_DIR_STOP;
    motors[MOTOR_1].is_initialized = true;

    // Motor 2 (C - Kiri Depan): TIM12 CH1+CH2 (PB14/PB15) - Full H-bridge control
    motors[MOTOR_2].htim = &htim12;
    motors[MOTOR_2].channel_rpwm = TIM_CHANNEL_1;  // PB14 - RPWM (forward)
    motors[MOTOR_2].channel_lpwm = TIM_CHANNEL_2;  // PB15 - LPWM (reverse)
    motors[MOTOR_2].current_speed = 0;
    motors[MOTOR_2].direction = MOTOR_DIR_STOP;
    motors[MOTOR_2].is_initialized = true;

    // Motor 3 (A - Kanan Belakang): TIM3 CH3+CH4 (PB0/PB1) - Full H-bridge control
    motors[MOTOR_3].htim = &htim3;
    motors[MOTOR_3].channel_rpwm = TIM_CHANNEL_3;  // PB0 - RPWM (forward)
    motors[MOTOR_3].channel_lpwm = TIM_CHANNEL_4;  // PB1 - LPWM (reverse)
    motors[MOTOR_3].current_speed = 0;
    motors[MOTOR_3].direction = MOTOR_DIR_STOP;
    motors[MOTOR_3].is_initialized = true;

    // Motor 4 (D - Kiri Belakang): TIM2 CH1+CH2 (PA0/PA1) - Full H-bridge control
    motors[MOTOR_4].htim = &htim2;
    motors[MOTOR_4].channel_rpwm = TIM_CHANNEL_1;  // PA0 - RPWM (forward)
    motors[MOTOR_4].channel_lpwm = TIM_CHANNEL_2;  // PA1 - LPWM (reverse)
    motors[MOTOR_4].current_speed = 0;
    motors[MOTOR_4].direction = MOTOR_DIR_STOP;
    motors[MOTOR_4].is_initialized = true;

    // Start all PWM channels
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);   // Motor 1 RPWM (PE5)
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);   // Motor 1 LPWM (PE6)
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  // Motor 2 RPWM (PB14)
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  // Motor 2 LPWM (PB15)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   // Motor 3 RPWM (PB0)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   // Motor 3 LPWM (PB1)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // Motor 4 RPWM (PA0)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   // Motor 4 LPWM (PA1)

    // Initialize all motors to stopped state
    Motor_Stop_All();
}

/**
  * @brief  Stop all motors
  * @retval None
  */
void Motor_Stop_All(void) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i].is_initialized) {
            __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel_rpwm, 0);
            __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel_lpwm, 0);
            motors[i].current_speed = 0;
            motors[i].direction = MOTOR_DIR_STOP;
        }
    }
}

/**
  * @brief  Convert speed percentage to PWM duty cycle
  * @param  speed: Speed percentage (-100 to +100)
  * @retval PWM duty cycle value (0-4199)
  */
static uint16_t speed_to_duty(int16_t speed) {
    // Clamp speed to valid range
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // Get absolute value
    int16_t abs_speed = (speed < 0) ? -speed : speed;

    // Convert to duty cycle (0-MOTOR_PWM_MAX)
    return (uint16_t)((abs_speed * MOTOR_PWM_MAX) / 100);
}

/**
  * @brief  Convert speed percentage to PWM duty cycle for specific motor
  * @param  motor_id: Motor ID (0-3)
  * @param  speed: Speed percentage (-100 to +100)
  * @retval PWM duty cycle value (motor-specific maximum)
  */
static uint16_t speed_to_duty_motor(uint8_t motor_id, int16_t speed) {
    // Clamp speed to valid range
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // Get absolute value
    int16_t abs_speed = (speed < 0) ? -speed : speed;

    // Get PWM maximum for specific motor
    uint16_t pwm_max = (motor_id == MOTOR_1) ? MOTOR_1_PWM_MAX : MOTOR_PWM_MAX;

    // Convert to duty cycle (0-motor_specific_max)
    return (uint16_t)((abs_speed * pwm_max) / 100);
}

/**
  * @brief  Set motor speed and direction
  * @param  motor_id: Motor ID (0-3)
  * @param  speed: Speed percentage (-100 to +100)
  * @retval None
  */
void Motor_SetSpeed(uint8_t motor_id, int16_t speed) {
    if (motor_id >= MOTOR_COUNT || !motors[motor_id].is_initialized) {
        return;
    }

    uint16_t duty = speed_to_duty_motor(motor_id, speed);

    // Polarity based on user's observation: Invert previous logic
    bool is_left_motor = (motor_id == MOTOR_2 || motor_id == MOTOR_4);

    if (speed > 0) { // MAJU (sekarang akan memutar motor ke arah yang sebelumnya mundur)
        motors[motor_id].direction = MOTOR_DIR_FORWARD;
        if (is_left_motor) {
            // Motor Kiri Maju = RPWM (dibalik dari sebelumnya)
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_rpwm, duty);
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_lpwm, 0);
        } else {
            // Motor Kanan Maju = LPWM (dibalik dari sebelumnya)
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_rpwm, 0);
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_lpwm, duty);
        }
    }
    else if (speed < 0) { // MUNDUR (sekarang akan memutar motor ke arah yang sebelumnya maju)
        motors[motor_id].direction = MOTOR_DIR_REVERSE;
        if (is_left_motor) {
            // Motor Kiri Mundur = LPWM (dibalik dari sebelumnya)
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_rpwm, 0);
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_lpwm, duty);
        } else {
            // Motor Kanan Mundur = RPWM (dibalik dari sebelumnya)
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_rpwm, duty);
            __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_lpwm, 0);
        }
    }
    else { // BERHENTI
        motors[motor_id].direction = MOTOR_DIR_STOP;
        __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_rpwm, 0);
        __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_lpwm, 0);
    }

    motors[motor_id].current_speed = speed;
}

/**
  * @brief  Set raw PWM duty cycle
  * @param  motor_id: Motor ID (0-3)
  * @param  duty_rpwm: RPWM duty (0-4199)
  * @param  duty_lpwm: LPWM duty (0-4199)
  * @retval None
  */
void Motor_SetDuty_Raw(uint8_t motor_id, uint16_t duty_rpwm, uint16_t duty_lpwm) {
    if (motor_id >= MOTOR_COUNT || !motors[motor_id].is_initialized) {
        return;
    }

    // Clamp to valid range
    if (duty_rpwm > MOTOR_PWM_MAX) duty_rpwm = MOTOR_PWM_MAX;
    if (duty_lpwm > MOTOR_PWM_MAX) duty_lpwm = MOTOR_PWM_MAX;

    __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_rpwm, duty_rpwm);
    __HAL_TIM_SET_COMPARE(motors[motor_id].htim, motors[motor_id].channel_lpwm, duty_lpwm);

    // Determine direction based on which channel is active
    if (duty_rpwm > 0 && duty_lpwm == 0) {
        motors[motor_id].direction = MOTOR_DIR_FORWARD;
        motors[motor_id].current_speed = duty_rpwm;
    }
    else if (duty_lpwm > 0 && duty_rpwm == 0) {
        motors[motor_id].direction = MOTOR_DIR_REVERSE;
        motors[motor_id].current_speed = duty_lpwm;
    }
    else {
        motors[motor_id].direction = MOTOR_DIR_STOP;
        motors[motor_id].current_speed = 0;
    }
}

/**
  * @brief  Move robot forward
  * @param  speed: Speed percentage (0-100)
  * @retval None
  */
void Motor_Forward(uint8_t speed) {
    if (speed > 100) speed = 100;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_SetSpeed(i, speed);
    }
}

/**
  * @brief  Move robot backward
  * @param  speed: Speed percentage (0-100)
  * @retval None
  */
void Motor_Reverse(uint8_t speed) {
    if (speed > 100) speed = 100;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_SetSpeed(i, -speed);
    }
}

/**
  * @brief  Turn robot left (differential drive)
  * @param  speed: Turn speed percentage (0-100)
  * @retval None
  * @note   Assuming motor layout: 0=FL, 1=FR, 2=RL, 3=RR
  *         Left turn: Left motors slow, Right motors fast
  */
void Motor_Turn_Left(uint8_t speed) {
    if (speed > 100) speed = 100;

    // Right side motors at full speed
    Motor_SetSpeed(MOTOR_1, speed);  // Assuming MOTOR_1 is right side
    Motor_SetSpeed(MOTOR_3, speed);

    // Left side motors at half speed
    Motor_SetSpeed(MOTOR_2, -speed);
    Motor_SetSpeed(MOTOR_4, -speed);
}

/**
  * @brief  Turn robot right (differential drive)
  * @param  speed: Turn speed percentage (0-100)
  * @retval None
  */
void Motor_Turn_Right(uint8_t speed) {
    if (speed > 100) speed = 100;

    // Left side motors at full speed
    Motor_SetSpeed(MOTOR_2, speed);
    Motor_SetSpeed(MOTOR_4, speed);

    // Right side motors at half speed
    Motor_SetSpeed(MOTOR_1, speed / 2);
    Motor_SetSpeed(MOTOR_3, speed / 2);
}

/**
  * @brief  Rotate robot in place (spin left)
  * @param  speed: Rotation speed percentage (0-100)
  * @retval None
  * @note   Left motors reverse, Right motors forward
  */
void Motor_Rotate_Left(uint8_t speed) {
    if (speed > 100) speed = 100;

    // Right side forward
    Motor_SetSpeed(MOTOR_1, speed);
    Motor_SetSpeed(MOTOR_3, speed);

    // Left side reverse
    Motor_SetSpeed(MOTOR_2, -speed);
    Motor_SetSpeed(MOTOR_4, -speed);
}

/**
  * @brief  Rotate robot in place (spin right)
  * @param  speed: Rotation speed percentage (0-100)
  * @retval None
  */
void Motor_Rotate_Right(uint8_t speed) {
    if (speed > 100) speed = 100;

    // Left side forward
    Motor_SetSpeed(MOTOR_2, speed);
    Motor_SetSpeed(MOTOR_4, speed);

    // Right side reverse
    Motor_SetSpeed(MOTOR_1, -speed);
    Motor_SetSpeed(MOTOR_3, -speed);
}

/**
  * @brief  Get current motor speed
  * @param  motor_id: Motor ID (0-3)
  * @retval Speed as percentage (-100 to +100)
  */
int16_t Motor_GetSpeed(uint8_t motor_id) {
    if (motor_id >= MOTOR_COUNT || !motors[motor_id].is_initialized) {
        return 0;
    }

    int16_t speed = (motors[motor_id].current_speed * 100) / MOTOR_PWM_MAX;

    if (motors[motor_id].direction == MOTOR_DIR_REVERSE) {
        speed = -speed;
    }
    else if (motors[motor_id].direction == MOTOR_DIR_STOP) {
        speed = 0;
    }

    return speed;
}

/**
  * @brief  Test function: Set TIM9 PWM for oscilloscope testing
  * @param  duty_percent: Duty cycle percentage (0-100)
  * @retval None
  * @note   Sets both CH1 (PE5) and CH2 (PE6) to same duty cycle
  */
void Motor_Test_TIM9_PWM(uint8_t duty_percent) {
    if (duty_percent > 100) duty_percent = 100;

    uint16_t duty = (duty_percent * MOTOR_PWM_MAX) / 100;

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, duty);
}

void motor_slide_left(uint8_t speed) {
    if (speed > 100) speed = 100;

    // Right side forward
    Motor_SetSpeed(MOTOR_1, speed);
    Motor_SetSpeed(MOTOR_4, speed);

    // Left side reverse
    Motor_SetSpeed(MOTOR_2, -speed);
    Motor_SetSpeed(MOTOR_3, -speed);
}

/**
  * @brief  Rotate robot in place (spin right)
  * @param  speed: Rotation speed percentage (0-100)
  * @retval None
  */
void motor_slide_right (uint8_t speed) {
    if (speed > 100) speed = 100;

    // Left side forward
    Motor_SetSpeed(MOTOR_2, speed);
    Motor_SetSpeed(MOTOR_3, speed);

    // Right side reverse
    Motor_SetSpeed(MOTOR_1, -speed);
    Motor_SetSpeed(MOTOR_4, -speed);
}
