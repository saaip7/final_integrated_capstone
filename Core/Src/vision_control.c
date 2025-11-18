/*
 * vision_control.c
 *
 *  Created on: 18 Nov 2025
 *      Author: Gemini
 *
 *  @brief High-level library to control the ESP32-CAM vision subsystem.
 */

#include "vision_control.h"
#include "modbus_master.h"
#include <stdio.h>

// Private global instance of the Modbus Master handle
static ModbusMaster_t modbus_handle;

/**
 * @brief Initializes the vision control system.
 */
void Vision_Init(UART_HandleTypeDef *huart) {
    ModbusMaster_Init(&modbus_handle, huart, MODBUS_TIMEOUT_MS);
    printf("Vision Control library initialized using Modbus.\r\n");
}

/**
 * @brief Checks if the ESP32-CAM vision slave is ready.
 */
Vision_Status_t Vision_Is_Ready(void) {
    uint16_t esp32_ready_status = 0;

    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    int result = ModbusMaster_ReadHoldingRegisters(&modbus_handle, MODBUS_SLAVE_ADDR, MODBUS_REG_ESP32_READY, 1, &esp32_ready_status);

    if (result != MODBUS_OK) {
        return VISION_ERR_COMM; // Communication error
    }

    if (esp32_ready_status == 1) {
        return VISION_OK; // Ready
    }

    return VISION_STATUS_BUSY; // Not ready yet
}

/**
 * @brief Sets the Group ID for the current photo session.
 */
Vision_Status_t Vision_Set_Group_ID(uint16_t group_id) {
    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    int result = ModbusMaster_WriteSingleRegister(&modbus_handle, MODBUS_SLAVE_ADDR, MODBUS_REG_GROUP_ID, group_id);

    if (result == MODBUS_OK) {
        return VISION_OK;
    }
    return VISION_ERR_COMM;
}

/**
 * @brief Starts a photo capture sequence on the ESP32-CAM.
 */
Vision_Status_t Vision_Start_Capture(uint16_t photo_id) {
    // Step 1: Set the Photo ID
    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    int result = ModbusMaster_WriteSingleRegister(&modbus_handle, MODBUS_SLAVE_ADDR, MODBUS_REG_PHOTO_ID, photo_id);
    if (result != MODBUS_OK) {
        printf("[Vision] ERROR: Failed to set Photo ID (Error: %d)\r\n", result);
        return VISION_ERR_COMM;
    }

    // Step 2: Send the Capture Command
    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    result = ModbusMaster_SendCaptureCommand(&modbus_handle);
    if (result != MODBUS_OK) {
        printf("[Vision] ERROR: Failed to send Capture Command (Error: %d)\r\n", result);
        return VISION_ERR_COMM;
    }

    printf("[Vision] Capture command sent for Photo ID %d.\r\n", photo_id);
    return VISION_OK;
}

/**
 * @brief Gets the current status from the ESP32-CAM.
 */
Vision_Status_t Vision_Get_Status(void) {
    uint16_t raw_status = 0;

    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    int result = ModbusMaster_ReadStatus(&modbus_handle, &raw_status);

    if (result != MODBUS_OK) {
        if (result == MODBUS_ERR_TIMEOUT) {
            return VISION_ERR_TIMEOUT;
        }
        return VISION_ERR_COMM;
    }

    // Translate raw Modbus status to our Vision_Status_t enum
    switch (raw_status) {
        case STATUS_IDLE:
            return VISION_STATUS_IDLE;
        case STATUS_BUSY:
            return VISION_STATUS_BUSY;
        case STATUS_SUCCESS:
            return VISION_STATUS_SUCCESS;
        case STATUS_ERROR:
            return VISION_STATUS_ERROR;
        default:
            return VISION_ERR_COMM; // Should not happen
    }
}
