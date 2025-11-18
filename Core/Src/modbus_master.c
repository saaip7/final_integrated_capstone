/**
  ******************************************************************************
  * @file           : modbus_master.c
  * @brief          : Modbus RTU Master implementation
  * @author         : STM32 Capstone Project
  ******************************************************************************
  */

#include "modbus_master.h"
#include <string.h>

/* Private function prototypes */
static int ModbusMaster_SendFrame(ModbusMaster_t *modbus, uint16_t tx_length);
static int ModbusMaster_ReceiveFrame(ModbusMaster_t *modbus, uint16_t expected_length);
static bool ModbusMaster_VerifyCRC(uint8_t *buffer, uint16_t length);

/**
 * @brief Initialize Modbus Master
 */
void ModbusMaster_Init(ModbusMaster_t *modbus, UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    modbus->huart = huart;
    modbus->timeout_ms = timeout_ms;
    modbus->tx_length = 0;
    modbus->rx_length = 0;
    memset(modbus->tx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    memset(modbus->rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
}

/**
 * @brief Flush UART RX buffer (Fix 4: Clear stale data)
 * This prevents CRC errors from old/corrupt data in buffer
 */
void ModbusMaster_FlushRxBuffer(UART_HandleTypeDef *huart)
{
    uint8_t dummy;
    // Read and discard any pending data in RX buffer (with short timeout)
    while (HAL_UART_Receive(huart, &dummy, 1, 10) == HAL_OK) {
        // Empty loop - just draining the buffer
    }
}

/**
 * @brief Calculate Modbus CRC16
 * CRC16-MODBUS (Polynomial: 0xA001, Init: 0xFFFF, RefIn: true, RefOut: true)
 */
uint16_t ModbusMaster_CRC16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Verify CRC of received frame
 */
static bool ModbusMaster_VerifyCRC(uint8_t *buffer, uint16_t length)
{
    if (length < 4) return false;  // Minimum frame: addr(1) + fc(1) + data(0+) + crc(2)

    // Calculate CRC of data (excluding last 2 bytes which is the CRC)
    uint16_t calculated_crc = ModbusMaster_CRC16(buffer, length - 2);

    // Extract received CRC (CRC is sent LSB first)
    uint16_t received_crc = (uint16_t)buffer[length - 1] << 8 | buffer[length - 2];

    return (calculated_crc == received_crc);
}

/**
 * @brief Send Modbus frame via UART
 */
static int ModbusMaster_SendFrame(ModbusMaster_t *modbus, uint16_t tx_length)
{
    // Calculate and append CRC
    uint16_t crc = ModbusMaster_CRC16(modbus->tx_buffer, tx_length);
    modbus->tx_buffer[tx_length++] = (uint8_t)(crc & 0xFF);        // CRC Low
    modbus->tx_buffer[tx_length++] = (uint8_t)((crc >> 8) & 0xFF); // CRC High

    // Send frame
    HAL_StatusTypeDef status = HAL_UART_Transmit(modbus->huart, modbus->tx_buffer,
                                                   tx_length, modbus->timeout_ms);

    if (status != HAL_OK) {
        return MODBUS_ERR_TIMEOUT;
    }

    return MODBUS_OK;
}

/**
 * @brief Receive Modbus frame via UART
 */
static int ModbusMaster_ReceiveFrame(ModbusMaster_t *modbus, uint16_t expected_length)
{
    // Clear RX buffer
    memset(modbus->rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);

    // Receive frame with timeout
    HAL_StatusTypeDef status = HAL_UART_Receive(modbus->huart, modbus->rx_buffer,
                                                  expected_length, modbus->timeout_ms);

    if (status != HAL_OK) {
        return MODBUS_ERR_TIMEOUT;
    }

    modbus->rx_length = expected_length;

    // Verify CRC
    if (!ModbusMaster_VerifyCRC(modbus->rx_buffer, expected_length)) {
        return MODBUS_ERR_CRC;
    }

    // Check for exception response (function code with MSB set)
    if (modbus->rx_buffer[1] & 0x80) {
        return MODBUS_ERR_EXCEPTION;
    }

    return MODBUS_OK;
}

/**
 * @brief Write single register (FC 0x06)
 * Frame format: [Addr][FC][Reg_Hi][Reg_Lo][Val_Hi][Val_Lo][CRC_Lo][CRC_Hi]
 * Response: Same as request if successful
 */
int ModbusMaster_WriteSingleRegister(ModbusMaster_t *modbus, uint8_t slave_addr,
                                      uint16_t reg_addr, uint16_t reg_value)
{
    // Build request frame
    modbus->tx_buffer[0] = slave_addr;
    modbus->tx_buffer[1] = MODBUS_FC_WRITE_SINGLE_REG;
    modbus->tx_buffer[2] = (uint8_t)((reg_addr >> 8) & 0xFF);  // Register address high
    modbus->tx_buffer[3] = (uint8_t)(reg_addr & 0xFF);         // Register address low
    modbus->tx_buffer[4] = (uint8_t)((reg_value >> 8) & 0xFF); // Register value high
    modbus->tx_buffer[5] = (uint8_t)(reg_value & 0xFF);        // Register value low

    // Send request (6 bytes + 2 CRC = 8 bytes total)
    int result = ModbusMaster_SendFrame(modbus, 6);
    if (result != MODBUS_OK) {
        return result;
    }

    // Wait for response (same format as request: 8 bytes)
    result = ModbusMaster_ReceiveFrame(modbus, 8);
    if (result != MODBUS_OK) {
        return result;
    }

    // Verify response matches request
    if (modbus->rx_buffer[0] != slave_addr ||
        modbus->rx_buffer[1] != MODBUS_FC_WRITE_SINGLE_REG ||
        modbus->rx_buffer[2] != modbus->tx_buffer[2] ||
        modbus->rx_buffer[3] != modbus->tx_buffer[3]) {
        return MODBUS_ERR_INVALID_RESPONSE;
    }

    return MODBUS_OK;
}

/**
 * @brief Read holding registers (FC 0x03)
 * Request: [Addr][FC][Start_Hi][Start_Lo][Count_Hi][Count_Lo][CRC_Lo][CRC_Hi]
 * Response: [Addr][FC][ByteCount][Data...][CRC_Lo][CRC_Hi]
 */
int ModbusMaster_ReadHoldingRegisters(ModbusMaster_t *modbus, uint8_t slave_addr,
                                       uint16_t start_addr, uint16_t num_regs,
                                       uint16_t *output_buffer)
{
    if (num_regs == 0 || num_regs > 125) {
        return MODBUS_ERR_INVALID_RESPONSE;
    }

    // Build request frame
    modbus->tx_buffer[0] = slave_addr;
    modbus->tx_buffer[1] = MODBUS_FC_READ_HOLDING_REG;
    modbus->tx_buffer[2] = (uint8_t)((start_addr >> 8) & 0xFF); // Start address high
    modbus->tx_buffer[3] = (uint8_t)(start_addr & 0xFF);        // Start address low
    modbus->tx_buffer[4] = (uint8_t)((num_regs >> 8) & 0xFF);   // Number of regs high
    modbus->tx_buffer[5] = (uint8_t)(num_regs & 0xFF);          // Number of regs low

    // Send request (6 bytes + 2 CRC = 8 bytes total)
    int result = ModbusMaster_SendFrame(modbus, 6);
    if (result != MODBUS_OK) {
        return result;
    }

    // Calculate expected response length: Addr(1) + FC(1) + ByteCount(1) + Data(n*2) + CRC(2)
    uint16_t expected_length = 3 + (num_regs * 2) + 2;

    // Receive response
    result = ModbusMaster_ReceiveFrame(modbus, expected_length);
    if (result != MODBUS_OK) {
        return result;
    }

    // Verify response header
    if (modbus->rx_buffer[0] != slave_addr ||
        modbus->rx_buffer[1] != MODBUS_FC_READ_HOLDING_REG ||
        modbus->rx_buffer[2] != (num_regs * 2)) {
        return MODBUS_ERR_INVALID_RESPONSE;
    }

    // Extract register values (Big Endian)
    for (uint16_t i = 0; i < num_regs; i++) {
        uint8_t high_byte = modbus->rx_buffer[3 + (i * 2)];
        uint8_t low_byte = modbus->rx_buffer[3 + (i * 2) + 1];
        output_buffer[i] = (uint16_t)(high_byte << 8 | low_byte);
    }

    return MODBUS_OK;
}

/**
 * @brief Send capture command to ESP32-CAM
 */
int ModbusMaster_SendCaptureCommand(ModbusMaster_t *modbus)
{
    return ModbusMaster_WriteSingleRegister(modbus, MODBUS_SLAVE_ADDR,
                                             MODBUS_REG_COMMAND, CMD_CAPTURE);
}

/**
 * @brief Read status from ESP32-CAM
 */
int ModbusMaster_ReadStatus(ModbusMaster_t *modbus, uint16_t *status)
{
    return ModbusMaster_ReadHoldingRegisters(modbus, MODBUS_SLAVE_ADDR,
                                              MODBUS_REG_STATUS, 1, status);
}

/**
 * @brief Wait for capture completion with polling
 * Polls status register until status changes to SUCCESS or ERROR
 */
int ModbusMaster_WaitForCompletion(ModbusMaster_t *modbus, uint32_t max_wait_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint16_t status = STATUS_IDLE;
    uint8_t consecutive_errors = 0;
    const uint8_t MAX_CONSECUTIVE_ERRORS = 5;  // Allow up to 5 consecutive errors before giving up

    while ((HAL_GetTick() - start_tick) < max_wait_ms) {
        // Read status register
        int result = ModbusMaster_ReadStatus(modbus, &status);

        // ===================================================================
        // CRITICAL FIX: Don't exit immediately on communication errors
        // Allow retries for transient errors (ESP32 busy during init/upload)
        // Only exit if too many consecutive errors (complete comm failure)
        // ===================================================================
        if (result != MODBUS_OK) {
            consecutive_errors++;

            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                // Too many consecutive errors - complete communication failure
                return result;
            }

            // Transient error, retry after delay
            HAL_Delay(200);
            continue;
        }

        // Communication successful, reset error counter
        consecutive_errors = 0;

        // Check status
        if (status == STATUS_SUCCESS) {
            return MODBUS_OK;  // Success!
        } else if (status == STATUS_ERROR) {
            return MODBUS_ERR_EXCEPTION;  // Slave reported error
        }

        // Status is IDLE or BUSY, wait and retry
        HAL_Delay(200);  // Poll every 200ms (increased frequency for Azure HTTPS latency)
    }

    // Timeout - exceeded max_wait_ms
    return MODBUS_ERR_TIMEOUT;
}
