/**
  ******************************************************************************
  * @file    hcsr04_sensor.c
  * @brief   HC-SR04 Ultrasonic Sensor Library Implementation
  * @author  Auto-generated for Capstone Project
  * @date    2025-11-11
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hcsr04_sensor.h"
#include <string.h>
#include <stdio.h>

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;  // Will be changed to htim5 later
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart1;

/* Private variables ---------------------------------------------------------*/
HC_SR04_IC sensors[NUM_SENSORS];
volatile uint8_t sensors_captured_count = 0;

static TIM_HandleTypeDef *delay_timer = NULL;

/* Private function prototypes -----------------------------------------------*/
static void Sensors_Pin_Init(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize microsecond delay timer
  * @param  htim: Timer handle for delay
  * @retval None
  */
void HC_SR04_Delay_Init(TIM_HandleTypeDef *htim) {
    delay_timer = htim;
    HAL_TIM_Base_Start(delay_timer);
}

/**
  * @brief  Microsecond delay function
  * @param  us: Delay time in microseconds
  * @retval None
  */
void delay_us(uint32_t us) {
    if (delay_timer == NULL) return;
    __HAL_TIM_SET_COUNTER(delay_timer, 0);
    while (__HAL_TIM_GET_COUNTER(delay_timer) < us);
}

/**
  * @brief  Median filter implementation
  * @param  buffer: Array of float values
  * @param  size: Number of elements
  * @retval Median value
  */
float HC_SR04_Median_Filter(float *buffer, uint8_t size) {
    if (size == 0) return 0.0f;
    if (size == 1) return buffer[0];

    float temp[FILTER_SIZE];
    memcpy(temp, buffer, size * sizeof(float));

    // Bubble sort
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }

    return temp[size / 2];
}

/**
  * @brief  Update median filter with new value
  * @param  sensor: Pointer to sensor structure
  * @param  new_value: New distance reading
  * @retval None
  */
void HC_SR04_Update_Filter(HC_SR04_IC *sensor, float new_value) {
    sensor->buffer[sensor->buffer_index] = new_value;
    sensor->buffer_index++;

    if (sensor->buffer_index >= FILTER_SIZE) {
        sensor->buffer_index = 0;
        sensor->buffer_full = true;
    }

    if (sensor->buffer_full) {
        sensor->filtered_distance = HC_SR04_Median_Filter(sensor->buffer, FILTER_SIZE);
    } else {
        sensor->filtered_distance = HC_SR04_Median_Filter(sensor->buffer, sensor->buffer_index);
    }
}

/**
  * @brief  Initialize sensor pin mapping
  * @retval None
  */
static void Sensors_Pin_Init(void) {
    // Sensor 1: TIM1_CH1 (PE9) - Trig PE10
    sensors[0].htim = &htim1;
    sensors[0].channel = TIM_CHANNEL_1;
    sensors[0].TRIG_PORT = GPIOE;
    sensors[0].TRIG_PIN = Trig_1_Pin;
    sensors[0].name = "US1";

    // Sensor 2: TIM1_CH2 (PE11) - Trig PE12
    sensors[1].htim = &htim1;
    sensors[1].channel = TIM_CHANNEL_2;
    sensors[1].TRIG_PORT = GPIOE;
    sensors[1].TRIG_PIN = Trig_2_Pin;
    sensors[1].name = "US2";

    // Sensor 3: TIM1_CH3 (PE13) - Trig PE14
    sensors[2].htim = &htim1;
    sensors[2].channel = TIM_CHANNEL_3;
    sensors[2].TRIG_PORT = GPIOE;
    sensors[2].TRIG_PIN = Trig_3_Pin;
    sensors[2].name = "US3";

    // Sensor 4: TIM1_CH4 (PA11) - Trig PA12
    sensors[3].htim = &htim1;
    sensors[3].channel = TIM_CHANNEL_4;
    sensors[3].TRIG_PORT = Trig_4_GPIO_Port;
    sensors[3].TRIG_PIN = Trig_4_Pin;
    sensors[3].name = "US4";

    // Sensor 5: TIM8_CH4 (PC9) - Trig PD15
    sensors[4].htim = &htim8;
    sensors[4].channel = TIM_CHANNEL_4;
    sensors[4].TRIG_PORT = GPIOD;
    sensors[4].TRIG_PIN = Trig_5_Pin;
    sensors[4].name = "US5";

    // Sensor 6: TIM8_CH3 (PC8) - Trig PD14
    sensors[5].htim = &htim8;
    sensors[5].channel = TIM_CHANNEL_3;
    sensors[5].TRIG_PORT = GPIOD;
    sensors[5].TRIG_PIN = Trig_6_Pin;
    sensors[5].name = "US6";

    // Sensor 7: TIM8_CH2 (PC7) - Trig PD13
    sensors[6].htim = &htim8;
    sensors[6].channel = TIM_CHANNEL_2;
    sensors[6].TRIG_PORT = GPIOD;
    sensors[6].TRIG_PIN = Trig_7_Pin;
    sensors[6].name = "US7";

    // Sensor 8: TIM8_CH1 (PC6) - Trig PD12
    sensors[7].htim = &htim8;
    sensors[7].channel = TIM_CHANNEL_1;
    sensors[7].TRIG_PORT = GPIOD;
    sensors[7].TRIG_PIN = Trig_8_Pin;
    sensors[7].name = "US8";

    // Initialize all sensor variables
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].is_first_captured = false;
        sensors[i].is_captured = false;
        sensors[i].difference = 0;
        sensors[i].distance = 0.0f;
        sensors[i].filtered_distance = 0.0f;
        sensors[i].buffer_index = 0;
        sensors[i].buffer_full = false;

        for (int j = 0; j < FILTER_SIZE; j++) {
            sensors[i].buffer[j] = 0.0f;
        }
    }
}

/**
  * @brief  Initialize all sensors and start input capture
  * @retval None
  */
void HC_SR04_Init(void) {
    // Initialize pin mapping
    Sensors_Pin_Init();

    // Start TIM1 input capture with interrupts (4 channels)
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

    // Start TIM8 input capture with interrupts (4 channels)
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
}

/**
  * @brief  Trigger all sensors simultaneously
  * @retval None
  */
void HC_SR04_Trigger_All(void) {
    // Reset flags and counter
    sensors_captured_count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].is_captured = false;
        sensors[i].is_first_captured = false;
        // Ensure polarity is set to RISING
        __HAL_TIM_SET_CAPTUREPOLARITY(sensors[i].htim, sensors[i].channel, TIM_INPUTCHANNELPOLARITY_RISING);
    }

    // Set all trigger pins HIGH
    for (int i = 0; i < NUM_SENSORS; i++) {
        HAL_GPIO_WritePin(sensors[i].TRIG_PORT, sensors[i].TRIG_PIN, GPIO_PIN_SET);
    }

    delay_us(12);  // 10-12us trigger pulse

    // Set all trigger pins LOW
    for (int i = 0; i < NUM_SENSORS; i++) {
        HAL_GPIO_WritePin(sensors[i].TRIG_PORT, sensors[i].TRIG_PIN, GPIO_PIN_RESET);
    }

    // Start timeout timer
    if (delay_timer != NULL) {
        __HAL_TIM_SET_COUNTER(delay_timer, 0);
    }
}

/**
  * @brief  Calculate distance from pulse width
  * @param  sensor: Pointer to sensor structure
  * @retval Distance in cm (0.0 if invalid)
  */
float HC_SR04_Calculate_Distance(HC_SR04_IC *sensor) {
    if (!sensor->is_captured) {
        return 0.0f;
    }

    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (time * speed) / 2
    // Distance = (time_us * 0.0343) / 2 = time_us * 0.01715
    float distance = (float)sensor->difference * 0.01715f;

    // Validate range (2.5cm - 400cm for HC-SR04)
    if (distance < 2.5f || distance > 400.0f) {
        return 0.0f;
    }

    sensor->distance = distance;
    return distance;
}

/**
  * @brief  Print all sensor data via UART
  * @retval None
  */
void HC_SR04_Print_Data(void) {
    char buffer[200];
    int len;

    len = snprintf(buffer, sizeof(buffer),
                   "US1: %.1fcm | US2: %.1fcm | US3: %.1fcm | US4: %.1fcm | "
                   "US5: %.1fcm | US6: %.1fcm | US7: %.1fcm | US8: %.1fcm\r\n",
                   sensors[0].filtered_distance,
                   sensors[1].filtered_distance,
                   sensors[2].filtered_distance,
                   sensors[3].filtered_distance,
                   sensors[4].filtered_distance,
                   sensors[5].filtered_distance,
                   sensors[6].filtered_distance,
                   sensors[7].filtered_distance);

    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

/**
  * @brief  Input Capture Callback (to be called from main.c)
  * @param  htim: Timer handle
  * @retval None
  */
void HC_SR04_Capture_Callback(TIM_HandleTypeDef *htim) {
    HC_SR04_IC *sensor = NULL;
    uint32_t capture_value;

    // Fast sensor lookup
    if (htim->Instance == TIM1) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1: sensor = &sensors[0]; break;
            case HAL_TIM_ACTIVE_CHANNEL_2: sensor = &sensors[1]; break;
            case HAL_TIM_ACTIVE_CHANNEL_3: sensor = &sensors[2]; break;
            case HAL_TIM_ACTIVE_CHANNEL_4: sensor = &sensors[3]; break;
            default: return;
        }
    }
    else if (htim->Instance == TIM8) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1: sensor = &sensors[7]; break;
            case HAL_TIM_ACTIVE_CHANNEL_2: sensor = &sensors[6]; break;
            case HAL_TIM_ACTIVE_CHANNEL_3: sensor = &sensors[5]; break;
            case HAL_TIM_ACTIVE_CHANNEL_4: sensor = &sensors[4]; break;
            default: return;
        }
    }
    else {
        return;
    }

    capture_value = HAL_TIM_ReadCapturedValue(htim, sensor->channel);

    if (!sensor->is_first_captured) {
        // First capture (RISING edge)
        sensor->ic_val1 = capture_value;
        sensor->is_first_captured = true;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, sensor->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else {
        // Second capture (FALLING edge)
        sensor->ic_val2 = capture_value;

        // Calculate pulse width (handle timer overflow)
        sensor->difference = (sensor->ic_val2 >= sensor->ic_val1) ?
                           (sensor->ic_val2 - sensor->ic_val1) :
                           (0xFFFF - sensor->ic_val1 + sensor->ic_val2);

        // Validate pulse width (150us - 23200us = 2.5cm - 400cm)
        sensor->is_captured = (sensor->difference >= 150 && sensor->difference <= 23200);

        // Reset for next measurement
        sensor->is_first_captured = false;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, sensor->channel, TIM_INPUTCHANNELPOLARITY_RISING);

        // Increment completion counter
        sensors_captured_count++;
    }
}
