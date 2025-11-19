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

/**
 * @brief Reads and prints error code from ESP32-CAM when upload fails.
 */
void Vision_Print_Error_Code(void) {
    uint16_t error_code = 0;

    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    int result = ModbusMaster_ReadHoldingRegisters(&modbus_handle, MODBUS_SLAVE_ADDR, MODBUS_REG_ERROR_CODE, 1, &error_code);

    if (result == MODBUS_OK) {
        printf("  -> Error Code: 0x%04X ", error_code);

        // Decode error code (sama seperti kode lama)
        switch(error_code) {
            case 0x0001: printf("(Camera capture failed)\r\n"); break;
            case 0x0002: printf("(WiFi disconnected)\r\n"); break;
            case 0x0003: printf("(Image upload failed)\r\n"); break;
            case 0x0004: printf("(Metadata upload failed)\r\n"); break;
            case 0x0005: printf("(Session ID retrieval failed)\r\n"); break;
            case 0x0006: printf("(Invalid Photo ID)\r\n"); break;
            default: printf("(Unknown error)\r\n"); break;
        }
    } else {
        printf("  -> Failed to read error code (Modbus error: %d)\r\n", result);
    }
}

/**
 * @brief Sends END_SESSION command to ESP32-CAM to close current session.
 *        ESP32 will call /session/end API to backend.
 */
Vision_Status_t Vision_End_Session(void) {
    printf("[Vision] Sending END_SESSION command to ESP32-CAM...\r\n");

    ModbusMaster_FlushRxBuffer(modbus_handle.huart);
    int result = ModbusMaster_WriteSingleRegister(&modbus_handle, MODBUS_SLAVE_ADDR,
                                                    MODBUS_REG_SESSION_CONTROL, 2); // 2 = END_SESSION

    if (result == MODBUS_OK) {
        printf("[Vision] END_SESSION command sent successfully.\r\n");
        printf("[Vision] Waiting for ESP32 to complete API call...\r\n");
        HAL_Delay(3000);  // Wait for ESP32 to call /session/end API
        printf("[Vision] Session ended.\r\n");
        return VISION_OK;
    } else {
        printf("[Vision] WARNING: Failed to send END_SESSION (Error: %d)\r\n", result);
        return VISION_ERR_COMM;
    }
}