/**
  ******************************************************************************
  * @file    hcsr04_sensor.h
  * @brief   HC-SR04 Ultrasonic Sensor Library Header
  * @author  Auto-generated for Capstone Project
  * @date    2025-11-11
  ******************************************************************************
  */

#ifndef __HCSR04_SENSOR_H
#define __HCSR04_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define NUM_SENSORS 8
#define FILTER_SIZE 3
#define SENSOR_TIMEOUT_US 25000  // 25ms timeout

/* Exported types ------------------------------------------------------------*/
typedef struct {
    TIM_HandleTypeDef *htim;      // Timer handle for input capture
    uint32_t channel;              // Timer channel (TIM_CHANNEL_1, etc.)
    GPIO_TypeDef *TRIG_PORT;       // Trigger GPIO port
    uint16_t TRIG_PIN;             // Trigger GPIO pin
    const char *name;              // Sensor name (US1-US8)

    // Measurement data
    volatile uint32_t ic_val1;     // First capture (rising edge)
    volatile uint32_t ic_val2;     // Second capture (falling edge)
    volatile uint32_t difference;  // Pulse width in microseconds
    volatile bool is_first_captured;
    volatile bool is_captured;

    // Median filter
    float buffer[FILTER_SIZE];     // Circular buffer
    uint8_t buffer_index;
    bool buffer_full;

    // Distance values
    float distance;                // Raw distance (cm)
    float filtered_distance;       // Filtered distance (cm)
} HC_SR04_IC;

/* Exported variables --------------------------------------------------------*/
extern HC_SR04_IC sensors[NUM_SENSORS];
extern volatile uint8_t sensors_captured_count;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize microsecond delay timer
  * @param  htim: Timer handle for delay (TIM2 or TIM5)
  * @retval None
  */
void HC_SR04_Delay_Init(TIM_HandleTypeDef *htim);

/**
  * @brief  Microsecond delay function
  * @param  us: Delay time in microseconds
  * @retval None
  */
void delay_us(uint32_t us);

/**
  * @brief  Initialize all sensor structures and start timers
  * @retval None
  */
void HC_SR04_Init(void);

/**
  * @brief  Trigger all 8 sensors simultaneously
  * @retval None
  */
void HC_SR04_Trigger_All(void);

/**
  * @brief  Calculate distance from pulse width
  * @param  sensor: Pointer to sensor structure
  * @retval Distance in cm (0.0 if invalid)
  */
float HC_SR04_Calculate_Distance(HC_SR04_IC *sensor);

/**
  * @brief  Update median filter with new value
  * @param  sensor: Pointer to sensor structure
  * @param  new_value: New distance reading
  * @retval None
  */
void HC_SR04_Update_Filter(HC_SR04_IC *sensor, float new_value);

/**
  * @brief  Print all sensor data via UART
  * @retval None
  */
void HC_SR04_Print_Data(void);

/**
  * @brief  Median filter helper function
  * @param  buffer: Array of float values
  * @param  size: Number of elements
  * @retval Median value
  */
float HC_SR04_Median_Filter(float *buffer, uint8_t size);

/**
  * @brief  Input Capture Callback (called from HAL_TIM_IC_CaptureCallback)
  * @param  htim: Timer handle
  * @retval None
  */
void HC_SR04_Capture_Callback(TIM_HandleTypeDef *htim);
void motor_slide_left(uint8_t speed);
void motor_slide_right(uint8_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __HCSR04_SENSOR_H */
