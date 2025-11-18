/**
  ******************************************************************************
  * @file           : modbus_master.h
  * @brief          : Modbus RTU Master implementation header
  * @author         : STM32 Capstone Project
  ******************************************************************************
  * @attention
  *
  * Modbus RTU Master untuk komunikasi STM32 (Master) <-> ESP32-CAM (Slave)
  *
  * Protocol: Modbus RTU over UART3 (9600 baud, 8N1)
  * Slave Address: 0x01 (ESP32-CAM)
  *
  * Function Codes:
  * - 0x03: Read Holding Registers
  * - 0x06: Write Single Register
  *
  * Register Map:
  * - 0x0001: Command Register (1=CAPTURE, 0=IDLE)
  * - 0x0002: Status Register (0=IDLE, 1=BUSY, 2=SUCCESS, 3=ERROR)
  * - 0x0003: Error Code Register
  *
  ******************************************************************************
  */

#ifndef MODBUS_MASTER_H_
#define MODBUS_MASTER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Modbus Configuration */
#define MODBUS_SLAVE_ADDR           0x01    // ESP32-CAM slave address
#define MODBUS_TIMEOUT_MS           1000    // Response timeout
#define MODBUS_MAX_FRAME_SIZE       256     // Maximum Modbus frame size

/* Modbus Function Codes */
#define MODBUS_FC_READ_HOLDING_REG  0x03
#define MODBUS_FC_WRITE_SINGLE_REG  0x06

/* Register Addresses */
#define MODBUS_REG_COMMAND          0x0001  // Command register
#define MODBUS_REG_STATUS           0x0002  // Status register
#define MODBUS_REG_ERROR_CODE       0x0003  // Error code register
#define MODBUS_REG_PHOTO_ID         0x0004  // Photo ID register
#define MODBUS_REG_GROUP_ID         0x0005  // Group ID register
#define MODBUS_REG_ESP32_READY      0x0006  // ESP32 Ready flag (Fix 2: Handshake)

/* Command Values */
#define CMD_IDLE                    0x0000
#define CMD_CAPTURE                 0x0001

/* Status Values */
#define STATUS_IDLE                 0x0000
#define STATUS_BUSY                 0x0001
#define STATUS_SUCCESS              0x0002
#define STATUS_ERROR                0x0003

/* Error Codes */
#define MODBUS_OK                   0
#define MODBUS_ERR_TIMEOUT          -1
#define MODBUS_ERR_CRC              -2
#define MODBUS_ERR_EXCEPTION        -3
#define MODBUS_ERR_INVALID_RESPONSE -4

/* Modbus Exception Codes */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR      0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE   0x04

/* Modbus Master Handle Structure */
typedef struct {
    UART_HandleTypeDef *huart;      // UART handle
    uint8_t tx_buffer[MODBUS_MAX_FRAME_SIZE];  // TX buffer
    uint8_t rx_buffer[MODBUS_MAX_FRAME_SIZE];  // RX buffer
    uint16_t tx_length;             // TX data length
    uint16_t rx_length;             // RX data length
    uint32_t timeout_ms;            // Response timeout in ms
} ModbusMaster_t;

/* Function Prototypes */

/**
 * @brief Initialize Modbus Master
 * @param modbus Pointer to ModbusMaster structure
 * @param huart Pointer to UART handle
 * @param timeout_ms Response timeout in milliseconds
 */
void ModbusMaster_Init(ModbusMaster_t *modbus, UART_HandleTypeDef *huart, uint32_t timeout_ms);

/**
 * @brief Flush UART RX buffer (Fix 4: Clear stale data)
 * @param huart Pointer to UART handle
 */
void ModbusMaster_FlushRxBuffer(UART_HandleTypeDef *huart);

/**
 * @brief Write single register (Function Code 0x06)
 * @param modbus Pointer to ModbusMaster structure
 * @param slave_addr Slave address
 * @param reg_addr Register address
 * @param reg_value Register value to write
 * @return MODBUS_OK on success, error code on failure
 */
int ModbusMaster_WriteSingleRegister(ModbusMaster_t *modbus, uint8_t slave_addr,
                                      uint16_t reg_addr, uint16_t reg_value);

/**
 * @brief Read holding registers (Function Code 0x03)
 * @param modbus Pointer to ModbusMaster structure
 * @param slave_addr Slave address
 * @param start_addr Starting register address
 * @param num_regs Number of registers to read
 * @param output_buffer Buffer to store read values (must be num_regs * 2 bytes)
 * @return MODBUS_OK on success, error code on failure
 */
int ModbusMaster_ReadHoldingRegisters(ModbusMaster_t *modbus, uint8_t slave_addr,
                                       uint16_t start_addr, uint16_t num_regs,
                                       uint16_t *output_buffer);

/**
 * @brief Calculate Modbus CRC16
 * @param buffer Data buffer
 * @param length Data length
 * @return CRC16 value
 */
uint16_t ModbusMaster_CRC16(uint8_t *buffer, uint16_t length);

/**
 * @brief Send capture command to ESP32-CAM
 * @param modbus Pointer to ModbusMaster structure
 * @return MODBUS_OK on success, error code on failure
 */
int ModbusMaster_SendCaptureCommand(ModbusMaster_t *modbus);

/**
 * @brief Read status from ESP32-CAM
 * @param modbus Pointer to ModbusMaster structure
 * @param status Pointer to store status value
 * @return MODBUS_OK on success, error code on failure
 */
int ModbusMaster_ReadStatus(ModbusMaster_t *modbus, uint16_t *status);

/**
 * @brief Wait for capture completion with polling
 * @param modbus Pointer to ModbusMaster structure
 * @param max_wait_ms Maximum wait time in milliseconds
 * @return MODBUS_OK on success, error code on failure
 */
int ModbusMaster_WaitForCompletion(ModbusMaster_t *modbus, uint32_t max_wait_ms);

#endif /* MODBUS_MASTER_H_ */
