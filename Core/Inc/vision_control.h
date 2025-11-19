/*
 * vision_control.h
 *
 *  Created on: 18 Nov 2025
 *      Author: Gemini
 *
 *  @brief High-level library to control the ESP32-CAM vision subsystem.
 *  This library abstracts the underlying Modbus RTU communication.
 */

#ifndef INC_VISION_CONTROL_H_
#define INC_VISION_CONTROL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

// Status enum to abstract away raw Modbus status values
typedef enum {
    VISION_OK,
    VISION_STATUS_IDLE,
    VISION_STATUS_BUSY,
    VISION_STATUS_SUCCESS,
    VISION_STATUS_ERROR,
    VISION_ERR_TIMEOUT,
    VISION_ERR_COMM, // General communication error (CRC, invalid response)
    VISION_ERR_SLAVE_REJECTED // Slave sent an exception
} Vision_Status_t;

/**
 * @brief Initializes the vision control system and the underlying Modbus master.
 * @param huart Pointer to the UART_HandleTypeDef for Modbus communication (e.g., &huart3).
 */
void Vision_Init(UART_HandleTypeDef *huart);

/**
 * @brief Checks if the ESP32-CAM vision slave is ready.
 * @return VISION_OK if ready, VISION_ERR_COMM or VISION_ERR_TIMEOUT otherwise.
 */
Vision_Status_t Vision_Is_Ready(void);

/**
 * @brief Sets the Group ID for the current photo session.
 * @param group_id The ID of the current group/corridor.
 * @return VISION_OK on success, error code on failure.
 */
Vision_Status_t Vision_Set_Group_ID(uint16_t group_id);

/**
 * @brief Starts a photo capture sequence on the ESP32-CAM.
 *        This is a non-blocking function that only sends the command.
 * @param photo_id The ID for the photo to be taken.
 * @return VISION_OK on success, error code on failure.
 */
Vision_Status_t Vision_Start_Capture(uint16_t photo_id);

/**
 * @brief Gets the current status from the ESP32-CAM.
 *        This is a polling function.
 * @return Vision_Status_t enum representing the current state.
 */
Vision_Status_t Vision_Get_Status(void);

/**
 * @brief Reads and prints error code from ESP32-CAM when upload fails.
 *        This is called when Vision_Get_Status returns VISION_STATUS_ERROR.
 */
void Vision_Print_Error_Code(void);

/**
 * @brief Sends END_SESSION command to ESP32-CAM to close current session.
 *        ESP32 will call /session/end API to backend.
 * @return VISION_OK on success, VISION_ERR_COMM on failure.
 */
Vision_Status_t Vision_End_Session(void);


#endif /* INC_VISION_CONTROL_H_ */