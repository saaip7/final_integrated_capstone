/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - 8 HC-SR04 Optimized (LOW LATENCY)
 ******************************************************************************
 * Informasi 8 sensor ultrasonik
 * Sensor 1: Echo_1 = PE9  (TIM1_CH1) Trig_1 = PE10 (Dp_Kn)
 * Sensor 2: Echo_2 = PE11 (TIM1_CH2) Trig_2 = PE12 (Dp_Kr)
 * Sensor 3: Echo_3 = PE13 (TIM1_CH3) Trig_3 = PE14 (Sp_Kr_Dp)
 * Sensor 4: Echo_4 = PA11 (TIM1_CH4) Trig_4 = PA12 (Sp_Kr_Bl)
 * Sensor 5: Echo_5 = PC9  (TIM8_CH4) Trig_5 = PD15 (Bl_Kr)
 * Sensor 6: Echo_6 = PC8  (TIM8_CH3) Trig_6 = PD14 (Bl_Kn)
 * Sensor 7: Echo_7 = PC7  (TIM8_CH2) Trig_7 = PD13 (Sp_Kn_Bl)
 * Sensor 8: Echo_8 = PC6  (TIM8_CH1) Trig_8 = PD12 (Sp_Kn_Dp)
 * UART: TX = PA9, RX = PA10 (USART1) --> ke laptop
 * Ket
 * Dp = depan		Kn = Kanan		Sp = Samping
 * Bl = Belakang	Kr = Kiri
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "hcsr04_sensor.h"
#include "motor_control.h"
#include "i2c.h"
#include "math.h"
#include "mpu6050.h"
#include "vision_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Definisi setiap keadaan (state) yang mungkin untuk robot.
// Ini menggantikan flag boolean 'orientasi_...'
typedef enum
{
  STATE_SYSTEM_INIT, // State untuk handshake dengan ESP32
  STATE_START,

  // Capture Sequence States
  STATE_TRIGGER_CAPTURE,
  STATE_WAIT_FOR_CAPTURE,

  // Lintasan 1 (Normal)
  STATE_LINTASAN_1_MAJU,
  // STATE_LINTASAN_1_MANUVER_DEPAN,
  STATE_LINTASAN_1_MUNDUR_DARI_DEPAN,
  STATE_LINTASAN_1_PUTAR_KIRI,
  STATE_LINTASAN_1_KOREKSI_LURUS,
  STATE_LINTASAN_1_MANUVER_BELAKANG,   // State baru sesuai update
  STATE_LINTASAN_1_MAJU_DARI_BELAKANG, // State baru sesuai update

  // Lintasan 2 (Normal)
  STATE_LINTASAN_2_MAJU,
  STATE_LINTASAN_2_MUNDUR_SEBENTAR,
  STATE_LINTASAN_2_PUTAR_BALIK,
  STATE_LINTASAN_2_MUNDUR_KOREKSI,
  STATE_LINTASAN_2_KOREKSI_LURUS,

  // Lintasan 3 (Terbalik)
  STATE_LINTASAN_3_MAJU,
  STATE_LINTASAN_3_PUTAR_KANAN,
  STATE_LINTASAN_3_KOREKSI_LURUS,

  // Lintasan 4 (Terbalik)
  STATE_LINTASAN_4_MAJU,
  STATE_LINTASAN_4_PUTAR_BALIK,
  STATE_LINTASAN_4_MUNDUR_KOREKSI,
  STATE_LINTASAN_4_KOREKSI_LURUS,

  STATE_SELESAI,
  STATE_ERROR,

  // Diagnostic Mode
  STATE_PUTAR_90,
  STATE_KOREKSI_LURUS_1,
  STATE_RESET_KEDUA,
  STATE_PUTAR_MINUS_90,
  STATE_KOREKSI_LURUS_2,
  STATE_LOOP_KEMBALI
} KeadaanRobot_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Konstanta untuk parameter gerak dan sensor
#define batas_jarak_depan 20.0f    // jarak robot dengan dinding untuk sensor depan
#define batas_jarak_belakang 15.0f // jarak robot dengan dingding untuk sensor belakang

#define batas_jauh_samping 22.0f
#define batas_dekat_samping 18.0f
#define kecepatan_motor 23
#define delay_jalan_ms 1000
#define delay_berhenti_ms 1000
#define take_photo_ms 1000

#define CAPTURE_TIMEOUT_MS 90000 // 90 detik timeout untuk capture

#define jarak_stop_depan 15.0f    // jarak robot mundur setelah mentok
#define jarak_stop_belakang 30.0f // jarak robot mundur setelah mentok

#define batas_error_lurus 1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;  // ESP-01 WiFi Serial Bridge (esp-link)
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Variabel utama untuk state machine
volatile KeadaanRobot_t keadaan_robot = STATE_SYSTEM_INIT; // Mulai dari state init
volatile KeadaanRobot_t state_selanjutnya_setelah_capture; // Untuk menyimpan state tujuan setelah capture

// Photo & Group ID counters
uint16_t photo_counter = 1;
uint16_t group_counter = 1;

// Retry logic untuk capture yang gagal
uint8_t capture_retry_count = 0;
const uint8_t MAX_CAPTURE_RETRIES = 3;

// System start control - GREEN button trigger
bool system_started = false;

// Variabel untuk manuver dan timing
bool mentok = false;
uint32_t waktu_terakhir_gerak = 0;
bool sedang_bergerak = false;

// Variable untuk koreksi lurus
uint32_t waktu_mulai_koreksi = 0;
uint8_t counter_koreksi_stabil = 0;

// Variable untuk putaran
uint32_t waktu_mulai_putar_180 = 0;
uint32_t waktu_mulai_putar_90 = 0;
uint32_t waktu_mulai_putar_neg_90 = 0;
uint32_t waktu_mulai_putar_neg_180 = 0;

// Variable untuk capture
uint32_t waktu_mulai_capture = 0;

// Variable untuk MPU6050
MPU6050_t MPU6050;
float yawAngle_deg = 0.0f;
uint32_t lastTick = 0;
float dt = 0.0f;
float Gz_bias = 0.0f; // Gyroscope Z-axis bias (offset)
char uart_buf[150];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);  // ESP-01 WiFi Serial Bridge
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
// Functions now in hcsr04_sensor.c library

// Button control functions (Active LOW dengan pull-up)
uint8_t read_green_button(void); // Returns 1 if pressed, 0 if not
uint8_t read_red_button(void);   // Returns 1 if pressed, 0 if not
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// =================================================================
// == BUTTON CONTROL FUNCTIONS ==
// Active LOW buttons dengan internal pull-up (pressed = 0, not pressed = 1)
// =================================================================

// Read GREEN button (PC0)
// Returns 1 if pressed, 0 if not pressed
uint8_t read_green_button(void)
{
  // Active LOW: Button pressed = GPIO_PIN_RESET (0)
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
  {
    HAL_Delay(50); // Debounce 50ms
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
      return 1; // Pressed
    }
  }
  return 0; // Not pressed
}

// Read RED button (PC2)
// Returns 1 if pressed, 0 if not pressed
uint8_t read_red_button(void)
{
  // Active LOW: Button pressed = GPIO_PIN_RESET (0)
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET)
  {
    HAL_Delay(50); // Debounce 50ms
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
      return 1; // Pressed
    }
  }
  return 0; // Not pressed
}

// =================================================================
// == BUZZER ERROR ALERT (I2C/Critical Hardware Error) ==
// Pattern: 3 LONG beeps → continuous FAST beeps
// =================================================================
void buzzer_error_alert(void)
{
  // Beep pattern: 3 long beeps (500ms ON, 500ms OFF)
  for (int i = 0; i < 3; i++)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Buzzer ON
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Buzzer OFF
    HAL_Delay(500);
  }

  // Continuous fast beep to indicate error persists
  while (1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Buzzer ON
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Buzzer OFF
    HAL_Delay(200);
  }
}

// =================================================================
// == BUZZER STATE ERROR ALERT ==
// Pattern: 5 SHORT beeps → continuous SLOW beeps
// =================================================================
void buzzer_state_error_alert(void)
{
  // Beep pattern: 5 short beeps (200ms ON, 200ms OFF)
  for (int i = 0; i < 5; i++)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Buzzer ON
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Buzzer OFF
    HAL_Delay(200);
  }

  // Continuous slow beep to indicate state error persists
  while (1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Buzzer ON
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Buzzer OFF
    HAL_Delay(500);
  }
}

// ==================== Override printf untuk UART ====================
// Dual output: USB-TTL debug (UART1) + WiFi esp-link (UART2)
int _write(int file, char *ptr, int len)
{
  (void)file; // Unused parameter
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100); // USB-TTL debug (115200 baud)
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100); // WiFi esp-link (115200 baud)
  return len;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();  // ESP-01 WiFi Serial Bridge (esp-link)
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize HC-SR04 sensor library
  HC_SR04_Delay_Init(&htim5); // Initialize delay timer
  HAL_Delay(100);             // Wait for timers to stabilize
  HC_SR04_Init();             // Initialize all 8 sensors + start input capture
  HAL_Delay(200);             // Wait for sensors to settle after power-on

  // Dummy reads to clear any noise
  HC_SR04_Trigger_All();
  HAL_Delay(60);
  HC_SR04_Trigger_All();
  HAL_Delay(60);

  Motor_Init();

  // I2C Scanner dengan retry logic
  printf("Scanning I2C bus...\r\n");
  const uint8_t I2C_SCAN_MAX_RETRIES = 3;
  uint8_t found_devices = 0;
  uint8_t scan_retry_count = 0;

  for (scan_retry_count = 0; scan_retry_count < I2C_SCAN_MAX_RETRIES; scan_retry_count++)
  {
    if (scan_retry_count > 0)
    {
      printf("I2C scan attempt #%d/%d...\r\n", scan_retry_count + 1, I2C_SCAN_MAX_RETRIES);
    }

    found_devices = 0; // Reset counter for each retry
    for (uint8_t addr = 1; addr < 128; addr++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
      {
        printf("Device found at address 0x%02X\r\n", addr);
        found_devices++;
      }
    }

    if (found_devices > 0)
    {
      printf("Found %d I2C device(s) on attempt #%d.\r\n\r\n", found_devices, scan_retry_count + 1);
      break; // Success, exit retry loop
    }
    else
    {
      printf("No I2C devices found (attempt #%d/%d).\r\n", scan_retry_count + 1, I2C_SCAN_MAX_RETRIES);
      HAL_Delay(500); // Wait before retry
    }
  }

  // Check if scan failed after all retries
  if (found_devices == 0)
  {
    printf("\r\n========================================\r\n");
    printf("CRITICAL ERROR: I2C SCAN FAILED!\r\n");
    printf("No devices found after %d attempts.\r\n", I2C_SCAN_MAX_RETRIES);
    printf("Check: I2C connections (SCL=PB6, SDA=PB7)\r\n");
    printf("Check: MPU6050 power supply\r\n");
    printf("Check: Pull-up resistors (4.7k on SCL/SDA)\r\n");
    printf("========================================\r\n");
    printf("BUZZER ALERT ACTIVATED!\r\n");
    printf("Press RESET button to retry.\r\n");
    printf("========================================\r\n\r\n");

    buzzer_error_alert(); // Activate buzzer and halt
  }
  HAL_Delay(500);

  // Initialize MPU6050 (single try - I2C scan already verified device exists)
  printf("Initializing MPU6050...\r\n");
  uint8_t mpu_status = MPU6050_Init(&hi2c1);
  if (mpu_status == 0)
  {
    printf("MPU6050 initialized successfully.\r\n");
  }
  else
  {
    printf("ERROR: MPU6050 initialization FAILED!\r\n");
    printf("Expected address 0x68, check AD0 pin (should be LOW).\r\n");
    while (1)
      ; // Stop execution (I2C scan passed, so this is MPU-specific issue)
  }
  HAL_Delay(100);

  // Kalibrasi Gyroscope Bias (robot harus DIAM!)
  printf("Kalibrasi gyro dimulai... Jangan gerakkan robot!\r\n");
  float Gz_sum = 0;
  const int calibration_samples = 100;

  for (int i = 0; i < calibration_samples; i++)
  {
    MPU6050_Read_All(&hi2c1, &MPU6050);
    Gz_sum += MPU6050.Gz;
    HAL_Delay(10);
  }

  Gz_bias = Gz_sum / calibration_samples; // Store in global variable
  printf("Kalibrasi selesai! Gz_bias = %.2f deg/s\r\n", Gz_bias);
  HAL_Delay(500);

  // Initialize Vision Control System
  Vision_Init(&huart3);

  // Initialize timing untuk yaw calculation
  lastTick = HAL_GetTick();

  printf("\r\n========================================\r\n");
  printf("SYSTEM READY!\r\n");
  printf("All sensors initialized successfully.\r\n");
  printf("========================================\r\n");
  printf("WiFi Serial Monitor: ACTIVE (UART2)\r\n");
  printf("  - ESP-01 esp-link firmware required\r\n");
  printf("  - Access via: http://<esp-link-IP>\r\n");
  printf("  - Telnet: telnet <esp-link-IP> 23\r\n");
  printf("========================================\r\n");
  printf("Press GREEN BUTTON to start operation...\r\n");
  printf("(Press RED BUTTON anytime to stop)\r\n");
  printf("========================================\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // FLAGD
#define DIAGNOSTIC_MODE 1 // Set to 1 for normal operation, 0 for commented test code

    // yawAngle_deg = 0;
#if DIAGNOSTIC_MODE == 0
    // Motor_Reverse(30);

    // DIAGNOSTIC MODE: Test Buzzer
    // ========================================================================
    //	      printf("=== DIAGNOSTIC MODE ACTIVE ===\r\n");
    //	      printf("Testing buzzer on PC13...\r\n");
    //	      printf("Pattern: 3 long beeps, then continuous fast beeps\r\n");
    //	      printf("Press RESET button to stop.\r\n\r\n");
    //	      HAL_Delay(2000);
    //	      buzzer_error_alert();  // This will loop forever


    // ======= FOR SENSOR TEST =============

    //	  HC_SR04_Trigger_All();
    //	      HAL_Delay(50); // Beri waktu untuk proses echo (interrupt akan menangkapnya)
    //
    //	      float sensor_a = HC_SR04_Calculate_Distance(&sensors[0]); // Depan Kanan
    //	      float sensor_b = HC_SR04_Calculate_Distance(&sensors[1]); // Depan Kiri
    //	      float sensor_c = HC_SR04_Calculate_Distance(&sensors[2]); // Samping Kiri Depan
    //	      float sensor_d = HC_SR04_Calculate_Distance(&sensors[3]); // Samping Kiri Belakang
    //	      float sensor_e = HC_SR04_Calculate_Distance(&sensors[4]); // Belakang Kiri
    //	      float sensor_f = HC_SR04_Calculate_Distance(&sensors[5]); // Belakang Kanan
    //	      float sensor_g = HC_SR04_Calculate_Distance(&sensors[6]); // Samping Kanan Belakang
    //	      float sensor_h = HC_SR04_Calculate_Distance(&sensors[7]); // Samping Kanan Depan
    //
    //	  printf("State:%d | Dpn(A,B):%.0f,%.0f | Bkg(E,F):%.0f,%.0f | Kn(G,H):%.0f,%.0f | Kr(C,D):%.0f,%.0f\r\n " ,
    //	     		keadaan_robot, sensor_a, sensor_b, sensor_e, sensor_f, sensor_g, sensor_h, sensor_c, sensor_d);

    // ================= FOR MPU TEST =============

    // uint32_t currentTick = HAL_GetTick();
    // dt = (float)(currentTick - lastTick) / 1000.0f; // Konversi ke detik
    // lastTick = currentTick;

    // // 2. Baca data MPU6050
    // MPU6050_Read_All(&hi2c1, &MPU6050);

    // // 3. Update yaw angle dengan integrasi gyroscope (DENGAN BIAS CORRECTION!)
    // float Gz_corrected = MPU6050.Gz - Gz_bias;
    // yawAngle_deg += Gz_corrected * dt;

    // printf("dt: %.4f | Gz_raw: %.2f | Gz_corr: %.2f | Yaw: %.1f\r\n",
    //        dt, MPU6050.Gz, Gz_corrected, yawAngle_deg);
    // HAL_Delay(100); // Tambahkan delay untuk stabilitas

    // Motor_Rotate_Right(35);
    // if (yawAngle_deg < -180.0f)
    // {
    //   Motor_Stop_All();
    // }

      // ========================================================================
      // CAMERA DIAGNOSTIC MODE - Test ESP32-CAM Capture & Upload
      // ========================================================================

      static uint8_t camera_test_state = 0;  // State: 0=boot_wait, 1=check_ready, 2=capture, 3=poll
      static uint32_t test_timer = 0;
      static uint16_t test_photo_id = 1;      // Test photo ID counter
      static uint16_t test_group_id = 99;     // Test group ID (99 = diagnostic)
      static uint8_t test_retry_count = 0;
      static uint8_t boot_complete = 0;       // Flag untuk boot delay selesai

      printf("\r\n=== CAMERA DIAGNOSTIC MODE ACTIVE ===\r\n");

      switch (camera_test_state) {

          case 0:  // BOOT WAIT - Wait for ESP32 to complete initialization
              if (!boot_complete) {
                  printf("\r\n========================================\r\n");
                  printf("[Test] Waiting for ESP32-CAM boot...\r\n");
                  printf("[Test] ESP32 needs time to:\r\n");
                  printf("[Test]   - Connect to WiFi\r\n");
                  printf("[Test]   - Fetch session ID from server\r\n");
                  printf("[Test]   - Initialize Modbus\r\n");
                  printf("[Test] Please wait 10 seconds...\r\n");
                  printf("========================================\r\n");

                  test_timer = HAL_GetTick();
                  boot_complete = 1;  // Set flag so message only prints once
              }

              // Wait 10 seconds for ESP32 boot
              uint32_t boot_elapsed = HAL_GetTick() - test_timer;

              // Print progress dot every second
              if ((boot_elapsed % 1000) < 100) {  // Trigger setiap ~1 detik
                  printf(".");
              }

              if (boot_elapsed >= 10000) {  // 10 seconds elapsed
                  printf("\r\n[Test] Boot wait complete!\r\n");
                  camera_test_state = 1;  // Move to check ready state
              }

              HAL_Delay(100);
              break;

          case 1:  // CHECK READY - Verify ESP32 is ready
              printf("[Test] Step 1: Checking if ESP32-CAM is ready...\r\n");

              Vision_Status_t ready_status = Vision_Is_Ready();

              if (ready_status == VISION_OK) {
                  printf("[Test] SUCCESS: ESP32-CAM is READY!\r\n");
                  printf("[Test] Setting Group ID to %d...\r\n", test_group_id);

                  if (Vision_Set_Group_ID(test_group_id) == VISION_OK) {
                      printf("[Test] Group ID set successfully.\r\n");
                      printf("[Test] Starting camera test loop...\r\n\r\n");
                      camera_test_state = 2;  // Move to capture state
                      test_timer = HAL_GetTick();
                  } else {
                      printf("[Test] ERROR: Failed to set Group ID!\r\n");
                      HAL_Delay(2000);  // Wait before retry
                  }

              } else if (ready_status == VISION_STATUS_BUSY) {
                  printf("[Test] ESP32-CAM not ready yet, waiting...\r\n");
                  HAL_Delay(1000);  // Wait 1 second before retry

              } else {
                  printf("[Test] ERROR: Communication error with ESP32-CAM!\r\n");
                  printf("[Test] Check UART3 connection (PB10/PB11, 9600 baud)\r\n");
                  printf("[Test] Check ESP32-CAM power and WiFi connection\r\n");
                  printf("[Test] Retrying in 3 seconds...\r\n");
                  HAL_Delay(3000);  // Wait 3 seconds before retry
              }
              break;

          case 2:  // CAPTURE - Send capture command
              printf("\r\n[Test] ========================================\r\n");
              printf("[Test] Step 2: Triggering capture for Photo ID %d\r\n", test_photo_id);
              printf("[Test] ========================================\r\n");

              Vision_Status_t capture_result = Vision_Start_Capture(test_photo_id);

              if (capture_result == VISION_OK) {
                  printf("[Test] Capture command sent successfully!\r\n");
                  printf("[Test] Waiting for ESP32 to capture and upload...\r\n");
                  camera_test_state = 3;  // Move to polling state
                  test_timer = HAL_GetTick();
                  test_retry_count = 0;
              } else {
                  printf("[Test] ERROR: Failed to send capture command!\r\n");
                  HAL_Delay(2000);
              }
              break;

          case 3:  // POLL - Wait for completion
              {
                  uint32_t elapsed = HAL_GetTick() - test_timer;

                  // Poll status every 500ms
                  if (elapsed % 500 == 0) {
                      Vision_Status_t poll_status = Vision_Get_Status();

                      if (poll_status == VISION_STATUS_SUCCESS) {
                          printf("\r\n[Test] ✓ SUCCESS! Photo %d uploaded successfully!\r\n", test_photo_id);
                          printf("[Test] Time taken: %lu ms\r\n", elapsed);
                          printf("[Test] ========================================\r\n\r\n");

                          // Increment photo ID for next test
                          test_photo_id++;

                          // Wait 5 seconds before next capture
                          printf("[Test] Waiting 5 seconds before next capture...\r\n");
                          HAL_Delay(5000);

                          // Return to capture state for continuous testing
                          camera_test_state = 2;

                      } else if (poll_status == VISION_STATUS_BUSY) {
                          printf(".");  // Progress indicator
                          HAL_Delay(500);

                      } else if (poll_status == VISION_STATUS_ERROR) {
                          printf("\r\n[Test] ✗ ERROR: Photo %d upload FAILED!\r\n", test_photo_id);
                          Vision_Print_Error_Code();  // Print detailed error

                          // Retry logic
                          test_retry_count++;
                          if (test_retry_count < 3) {
                              printf("[Test] Retrying... (Attempt %d/3)\r\n", test_retry_count + 1);
                              HAL_Delay(2000);
                              camera_test_state = 2;  // Retry capture
                          } else {
                              printf("[Test] Max retries reached. Skipping to next photo.\r\n");
                              test_photo_id++;
                              test_retry_count = 0;
                              HAL_Delay(3000);
                              camera_test_state = 2;
                          }

                      } else {
                          printf("\r\n[Test] Communication error during polling!\r\n");
                          HAL_Delay(1000);
                      }
                  }

                  // Timeout after 60 seconds
                  if (elapsed > 60000) {
                      printf("\r\n[Test] ✗ TIMEOUT: No response after 60 seconds!\r\n");
                      printf("[Test] ESP32 might be stuck. Skipping to next photo.\r\n");
                      test_photo_id++;
                      HAL_Delay(2000);
                      camera_test_state = 2;
                  }
              }
              break;

          default:
              camera_test_state = 0;
              break;
      }

      HAL_Delay(100);  // Small delay for stability

	  //Motor_Forward(35);


#else

    // ========================================================================
    // LANGKAH 1: BACA SEMUA SENSOR (SETIAP SAAT)
    // ========================================================================
    HC_SR04_Trigger_All();
    HAL_Delay(50); // Beri waktu untuk proses echo (interrupt akan menangkapnya)

    float sensor_a = HC_SR04_Calculate_Distance(&sensors[0]); // Depan Kanan
    float sensor_b = HC_SR04_Calculate_Distance(&sensors[1]); // Depan Kiri
    float sensor_c = HC_SR04_Calculate_Distance(&sensors[2]); // Samping Kiri Depan
    float sensor_d = HC_SR04_Calculate_Distance(&sensors[3]); // Samping Kiri Belakang
    float sensor_e = HC_SR04_Calculate_Distance(&sensors[4]); // Belakang Kiri
    float sensor_f = HC_SR04_Calculate_Distance(&sensors[5]); // Belakang Kanan
    float sensor_g = HC_SR04_Calculate_Distance(&sensors[6]); // Samping Kanan Belakang
    float sensor_h = HC_SR04_Calculate_Distance(&sensors[7]); // Samping Kanan Depan

    // ========================================================================
    // LANGKAH 1.5: UPDATE MPU6050 DAN YAW ANGLE
    // ========================================================================
    // 1. Hitung delta time (dt)
    uint32_t currentTick = HAL_GetTick();
    dt = (float)(currentTick - lastTick) / 1000.0f; // Konversi ke detik
    lastTick = currentTick;

    // 2. Baca data MPU6050
    MPU6050_Read_All(&hi2c1, &MPU6050);

    // 3. Update yaw angle dengan integrasi gyroscope (DENGAN BIAS CORRECTION!)
    float Gz_corrected = MPU6050.Gz - Gz_bias;
    yawAngle_deg += Gz_corrected * dt;

    // ========================================================================
    // LANGKAH 1.7: CHECK GREEN BUTTON TO START OPERATION
    // System harus menunggu GREEN button ditekan setelah inisialisasi sebelum mulai beroperasi
    // ========================================================================
    if (!system_started)
    {
      // System belum dimulai, tunggu GREEN button press
      if (read_green_button())
      {
        printf("\r\n========================================\r\n");
        printf("[System] GREEN BUTTON PRESSED!\r\n");
        printf("[System] Starting autonomous operation...\r\n");
        printf("========================================\r\n\r\n");
        system_started = true;
        HAL_Delay(500); // Debounce delay
      }
      else
      {
        // Belum ditekan, skip rest of loop dan tunggu
        HAL_Delay(100); // Small delay to prevent busy loop
        continue;       // Skip ke iterasi berikutnya (tidak proses sensor, tidak printf spam)
      }
    }

    // ===========================================================	=============
    // LANGKAH 2: PROSES DATA SENSOR MENJADI INFORMASI
    // ========================================================================
    // Kondisi Jarak
    bool ada_halangan_depan = (sensor_a < batas_jarak_depan) && (sensor_b < batas_jarak_depan);

    // bool ada_halangan_belakang = (sensor_e < batas_jarak_belakang) || (sensor_f < batas_jarak_belakang);

    // Kondisi Wall Following Kanan
    // bool lurus_dgn_kanan = fabs(sensor_g - sensor_h) <= 1.0f;
    // bool terlalu_jauh_kanan = (sensor_h > batas_jauh_samping && sensor_g > batas_jauh_samping);
    // bool terlalu_dekat_kanan = (sensor_h < batas_dekat_samping && sensor_g < batas_dekat_samping);

    // Kondisi Wall Following Kiri
    // bool lurus_dgn_kiri = fabs(sensor_c - sensor_d) <= 1.0f;
    // bool terlalu_jauh_kiri = (sensor_c > batas_jauh_samping && sensor_d > batas_jauh_samping);
    // bool terlalu_dekat_kiri = (sensor_c < batas_dekat_samping && sensor_d < batas_dekat_samping);

    // ========================================================================
    // LANGKAH 3: DEBUGGING PRINTF
    // ========================================================================
    // Cetak informasi penting untuk debugging di satu baris

    printf("State:%d | Dpn(A,B):%.0f,%.0f | Bkg(E,F):%.0f,%.0f \r\n",
           keadaan_robot, sensor_a, sensor_b, sensor_e, sensor_f);
    // printf(" State: %d | Yaw: %.0f \r\n", keadaan_robot, yawAngle_deg);

    // ========================================================================
    // LANGKAH 3.5: CHECK RED BUTTON FOR EMERGENCY STOP
    // User dapat menekan RED button kapan saja untuk end session
    // ========================================================================
    if (read_red_button())
    {
      printf("\r\n========================================\r\n");
      printf("[System] RED BUTTON PRESSED! Stopping robot...\r\n");
      printf("========================================\r\n\r\n");

      Motor_Stop_All(); // Stop semua motor segera

      // Send END_SESSION command to ESP32-CAM
      Vision_End_Session();

      printf("\r\n========================================\r\n");
      printf("[System] SESSION ENDED BY USER\r\n");
      printf("[System] Total photos captured: %d\r\n", photo_counter - 1);
      printf("[System] Press RESET button to restart.\r\n");
      printf("========================================\r\n\r\n");

      // Infinite loop - robot stopped, requires reset
      while (1)
      {
        HAL_Delay(1000);
      }
    }

    // ========================================================================
    // LANGKAH 4: STATE MACHINE UTAMA
    // ========================================================================
    switch (keadaan_robot)
    {
    case STATE_SYSTEM_INIT:
      printf("[System] Waiting for ESP32-CAM initialization...\r\n");
      // Lakukan handshake dengan ESP32-CAM
      Vision_Status_t ready_status = Vision_Is_Ready();
      if (ready_status == VISION_OK)
      {
        printf("[System] ESP32-CAM ready! Setting Group ID to %d...\r\n", group_counter);
        if (Vision_Set_Group_ID(group_counter) == VISION_OK)
        {
          printf("[System] System initialized. Starting navigation.\r\n");
          keadaan_robot = STATE_START;
        }
        else
        {
          printf("[System] ERROR: Failed to set Group ID. Retrying...\r\n");
          HAL_Delay(1000); // Retry after 1 second
        }
      }
      else
      {
        // Jika belum siap, tunggu dan coba lagi
        HAL_Delay(1000); // Tunggu 1 detik sebelum cek lagi
      }
      break;
    case STATE_START:
      printf("STATE: START -> LINTASAN_1_MAJU\r\n");

      // FLAG-S
      keadaan_robot = STATE_LINTASAN_1_MAJU;
      waktu_terakhir_gerak = HAL_GetTick();
      sedang_bergerak = true; // Mulai dengan bergerak
      break;

    case STATE_TRIGGER_CAPTURE:
      if (capture_retry_count == 0)
      {
        printf("[Capture] Starting photo #%d...\r\n", photo_counter);
      }
      else
      {
        printf("[Capture] Retry #%d for photo #%d...\r\n", capture_retry_count, photo_counter);
      }

      Vision_Status_t trigger_status = Vision_Start_Capture(photo_counter);

      if (trigger_status == VISION_OK)
      {
        waktu_mulai_capture = HAL_GetTick(); // Mulai timer timeout
        keadaan_robot = STATE_WAIT_FOR_CAPTURE;
      }
      else
      {
        printf("[Capture] ERROR: Failed to trigger capture command.\r\n");
        capture_retry_count++;

        if (capture_retry_count >= MAX_CAPTURE_RETRIES)
        {
          printf("[Capture] FAILED after %d retries. Skipping photo #%d.\r\n",
                 MAX_CAPTURE_RETRIES, photo_counter);
          photo_counter++;                                   // Skip photo ID setelah max retries
          capture_retry_count = 0;                           // Reset retry counter
          keadaan_robot = state_selanjutnya_setelah_capture; // Lanjut navigasi
          waktu_terakhir_gerak = HAL_GetTick();
          sedang_bergerak = false;
        }
        else
        {
          printf("[Capture] Retrying in 1 second... (%d/%d)\r\n",
                 capture_retry_count, MAX_CAPTURE_RETRIES);
          HAL_Delay(1000); // Tunggu 1 detik sebelum retry
                           // Tetap di STATE_TRIGGER_CAPTURE untuk retry
        }
      }
      break;

    case STATE_WAIT_FOR_CAPTURE:
      // Cek timeout
      if (HAL_GetTick() - waktu_mulai_capture > CAPTURE_TIMEOUT_MS)
      {
        printf("[Capture] TIMEOUT! ESP32-CAM tidak selesai dalam %d ms.\r\n", CAPTURE_TIMEOUT_MS);
        capture_retry_count++;

        if (capture_retry_count >= MAX_CAPTURE_RETRIES)
        {
          printf("[Capture] FAILED after %d retries. Skipping photo #%d.\r\n",
                 MAX_CAPTURE_RETRIES, photo_counter);
          photo_counter++;         // Skip setelah max retries
          capture_retry_count = 0; // Reset retry counter
          keadaan_robot = state_selanjutnya_setelah_capture;
          waktu_terakhir_gerak = HAL_GetTick();
          sedang_bergerak = false;
        }
        else
        {
          printf("[Capture] Retrying capture... (%d/%d)\r\n",
                 capture_retry_count, MAX_CAPTURE_RETRIES);
          keadaan_robot = STATE_TRIGGER_CAPTURE; // Retry dari awal
        }
        break;
      }

      // Poll status dari vision system
      Vision_Status_t capture_status = Vision_Get_Status();
      if (capture_status == VISION_STATUS_SUCCESS)
      {
        printf("[Capture] SUCCESS! Photo #%d uploaded.\r\n", photo_counter);
        photo_counter++;                                   // Increment untuk foto berikutnya
        capture_retry_count = 0;                           // Reset retry counter untuk foto berikutnya
        keadaan_robot = state_selanjutnya_setelah_capture; // Lanjut ke state navigasi berikutnya
        waktu_terakhir_gerak = HAL_GetTick();
        sedang_bergerak = false;
      }
      else if (capture_status == VISION_STATUS_ERROR)
      {
        printf("[Capture] ERROR! ESP32-CAM melaporkan kegagalan.\r\n");
        Vision_Print_Error_Code(); // Baca dan tampilkan error code detail
        capture_retry_count++;

        if (capture_retry_count >= MAX_CAPTURE_RETRIES)
        {
          printf("[Capture] FAILED after %d retries. Skipping photo #%d.\r\n",
                 MAX_CAPTURE_RETRIES, photo_counter);
          photo_counter++;         // Skip setelah max retries
          capture_retry_count = 0; // Reset retry counter
          keadaan_robot = state_selanjutnya_setelah_capture;
          waktu_terakhir_gerak = HAL_GetTick();
          sedang_bergerak = false;
        }
        else
        {
          printf("[Capture] Retrying capture... (%d/%d)\r\n",
                 capture_retry_count, MAX_CAPTURE_RETRIES);
          HAL_Delay(1000);                       // Tunggu 1 detik sebelum retry
          keadaan_robot = STATE_TRIGGER_CAPTURE; // Retry dari awal
        }
      }
      else if (capture_status == VISION_ERR_COMM || capture_status == VISION_ERR_TIMEOUT)
      {
        printf("[Capture] Communication error with ESP32-CAM.\r\n");
        printf("[Recovery] Retrying status poll...\r\n");
        HAL_Delay(200); // Tunggu dan retry
      }
      else
      {
        // Masih BUSY atau IDLE, tunggu saja.
        HAL_Delay(200); // Poll setiap 200ms
      }
      break;

      // ======================= LINTASAN 1 ======================================

    case STATE_LINTASAN_1_MAJU:
      // Cek kondisi transisi state: jika ada halangan di depan
      if (ada_halangan_depan)
      {
        Motor_Stop_All();
        printf("STATE: Halangan depan terdeteksi. Masuk ke manuver.\r\n");
        keadaan_robot = STATE_LINTASAN_1_MUNDUR_DARI_DEPAN;
        break;
      }

      // Logika gerak 2 detik jalan, 1 detik berhenti
      if (sedang_bergerak)
      {
        if (HAL_GetTick() - waktu_terakhir_gerak >= delay_jalan_ms)
        {
          Motor_Stop_All();

          // Simpan state tujuan, lalu trigger capture
          state_selanjutnya_setelah_capture = keadaan_robot; // Kembali ke state ini setelah capture
          keadaan_robot = STATE_TRIGGER_CAPTURE;
          sedang_bergerak = false;
          waktu_terakhir_gerak = HAL_GetTick();
        }
      }
      else
      {
        if (HAL_GetTick() - waktu_terakhir_gerak >= delay_berhenti_ms)
        {
          sedang_bergerak = true;
          waktu_terakhir_gerak = HAL_GetTick();
          printf("Melanjutkan gerak maju...\r\n");
        }
      } // Jika sedang dalam periode gerak, lakukan wall following
      if (sedang_bergerak)
      {
        // Kita asumsikan robot lurus dan hanya fokus pada gerak maju.
        // Motor_Forward(kecepatan_motor);

        if (sensor_a > batas_jarak_depan || sensor_b > batas_jarak_depan)
        {
          Motor_Forward(kecepatan_motor);
        }
        else
        {
          // Target tercapai, berhenti
          Motor_Stop_All();
          HAL_Delay(2000);
        }
      }
      break;

      // GADIPAKE
      //        case STATE_LINTASAN_1_MANUVER_DEPAN:
      //            // Tujuan: Maju pelan sampai jarak < 2cm
      //            if (sensor_a > otw_mentok_depan && sensor_b > otw_mentok_depan) {
      //                // Masih jauh, lanjutkan maju pelan
      //                Motor_Forward(kecepatan_motor);
      //            } else {
      //                // Target tercapai, berhenti
      //                Motor_Stop_All();
      //                HAL_Delay(1500);
      //                printf("Mentok depan tercapai. Capture #1 (Depan)!\r\n");
      //                HAL_Delay(take_photo_ms); // Blocking delay untuk capture
      //                printf("STATE: Selesai manuver depan, lanjut mundur.\r\n");
      //                keadaan_robot = STATE_LINTASAN_1_MUNDUR_DARI_DEPAN;
      //            }
      //            break;

    case STATE_LINTASAN_1_MUNDUR_DARI_DEPAN:
      // Tujuan: Mundur sampai jarak tertentu
      if (sensor_a < jarak_stop_depan || sensor_b < jarak_stop_depan)
      {
        // Masih terlalu dekat, lanjutkan mundur
        Motor_Reverse(kecepatan_motor);
      }
      else
      {
        // Target tercapai, berhenti
        Motor_Stop_All();
        printf("STATE: Posisi mundur aman tercapai, lanjut putar kiri.\r\n");
        printf("Yaw angle di-reset ke 0.\r\n");
        yawAngle_deg = 0.0f;      // RESET YAW SEBELUM PINDAH STATE
        lastTick = HAL_GetTick(); // RESET TIMING untuk mencegah dt yang besar
        keadaan_robot = STATE_LINTASAN_1_PUTAR_KIRI;
        waktu_mulai_putar_90 = HAL_GetTick(); // Mulai timer timeout untuk putaran

        HAL_Delay(1500);
      }
      break;

    case STATE_LINTASAN_1_PUTAR_KIRI:

      // LOGIC PUTAR DENGAN SENSOR
      //  Cek apakah ada dinding di belakang untuk dijadikan referensi dan apakah robot sudah lurus.
      //  Sensor harus membaca > 0 (valid) dan < 50cm (dinding terdeteksi)
      //            if ( (sensor_e > 0 && sensor_f > 0 && sensor_e < 50 && sensor_f < 50) && (fabs(sensor_e - sensor_f) <= batas_error_lurus) ) {
      //                // Dinding terdeteksi DAN robot sudah lurus.
      //                Motor_Stop_All();
      //                printf("STATE: Putaran selesai, robot lurus dengan dinding belakang.\r\n");
      //                HAL_Delay(2000);
      //
      //                keadaan_robot = STATE_LINTASAN_1_KOREKSI_LURUS;
      //                waktu_mulai_koreksi = HAL_GetTick();
      //                counter_koreksi_stabil = 0;
      //                printf("STATE: Masuk koreksi lurus dengan sensor samping kanan.\r\n");
      //            } else {
      //                // Jika belum lurus atau belum ada dinding, lanjutkan berputar ke kiri.
      //                printf("Memutar ke kiri untuk mencari dinding belakang...\r\n");
      //                Motor_Rotate_Left(20);
      //
      //            }
      //            break;

      // Timeout protection - maksimal 10 detik untuk putar 180
      if (HAL_GetTick() - waktu_mulai_putar_90 > 10000)
      {
        Motor_Stop_All();
        printf("STATE L1: Timeout putar 90° (5 detik), paksa lanjut!\r\n");
        keadaan_robot = STATE_LINTASAN_1_KOREKSI_LURUS;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_90 = 0;
        break;
      }

      // Cek apakah sudah putar lebih dari 180° (menggunakan absolute value)
      if (yawAngle_deg > 90.0f)
      {
        // Sudah putar 90° (meskipun belum tentu lurus)
        Motor_Stop_All();
        printf("STATE L1: Putaran 90° SELESAI! Yaw angle: %.1f°\r\n", yawAngle_deg);
        HAL_Delay(500);

        // Transisi ke koreksi lurus dengan sensor belakang
        printf("STATE L1: Masuk koreksi lurus dengan sensor belakang.\r\n");
        keadaan_robot = STATE_LINTASAN_1_KOREKSI_LURUS;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_90 = 0;
      }
      else
      {
        // Belum 90°, lanjut putar
        Motor_Rotate_Left(30);

        // Debug info (setiap ~500ms untuk tidak spam)
        static uint32_t last_debug_print = 0;
        if (HAL_GetTick() - last_debug_print > 500)
        {
          printf("Putar Yaw=%.1f° \r\n", yawAngle_deg);
          last_debug_print = HAL_GetTick();
        }
      }
      break;

    case STATE_LINTASAN_1_KOREKSI_LURUS:
      // Timeout check - maksimal 5 detik untuk koreksi
      if (HAL_GetTick() - waktu_mulai_koreksi > 5000)
      {
        Motor_Stop_All();
        printf("STATE: Timeout koreksi lurus (5 detik), paksa lanjut.\r\n");
        keadaan_robot = STATE_LINTASAN_1_MANUVER_BELAKANG;
        break;
      }

      // Validasi: Pastikan sensor samping kanan (G & H) mendeteksi dinding
      if ((sensor_g > 0 && sensor_g < 50) && (sensor_h > 0 && sensor_h < 50))
      {

        float selisih_samping = fabs(sensor_h - sensor_g);

        if (selisih_samping > 0.5f)
        {
          // Robot belum lurus, perlu koreksi
          counter_koreksi_stabil = 0; // Reset counter stabilitas

          if (sensor_h > sensor_g)
          {
            // Kondisi: Bagian DEPAN robot lebih jauh dari dinding kanan
            // Artinya: Ekor robot lebih dekat ke dinding (robot miring ke kiri)
            // Solusi: Putar KANAN untuk meluruskan
            printf("Koreksi: Putar kanan | H:%.1f > G:%.1f | Diff:%.1f\r\n",
                   sensor_h, sensor_g, selisih_samping);
            Motor_Rotate_Right(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
          else
          {
            // Kondisi: Bagian BELAKANG robot lebih jauh dari dinding kanan
            // Artinya: Kepala robot lebih dekat ke dinding (robot miring ke kanan)
            // Solusi: Putar KIRI untuk meluruskan
            printf("Koreksi: Putar kiri | G:%.1f > H:%.1f | Diff:%.1f\r\n",
                   sensor_g, sensor_h, selisih_samping);
            Motor_Rotate_Left(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
        }
        else
        {
          // Robot sudah cukup lurus (selisih <= 0.5cm)
          counter_koreksi_stabil++;
          printf("Lurus! H:%.1f G:%.1f | Diff:%.1f | Stabil:%d/3\r\n",
                 sensor_h, sensor_g, selisih_samping, counter_koreksi_stabil);

          if (counter_koreksi_stabil >= 3)
          {
            // Validasi: Sudah lurus 3x pembacaan berturut-turut
            Motor_Stop_All();
            printf("STATE: Koreksi lurus SELESAI! Lanjut manuver belakang.\r\n");
            HAL_Delay(1000);
            keadaan_robot = STATE_LINTASAN_1_MANUVER_BELAKANG;
          }
          else
          {
            // Tunggu pembacaan sensor berikutnya untuk validasi
            Motor_Stop_All();
            HAL_Delay(100);
          }
        }
      }
      else
      {
        // Sensor samping kanan tidak mendeteksi dinding (nilai 0 atau > 50cm)
        printf("Warning: Dinding samping kanan tidak terdeteksi (G:%.1f H:%.1f)\r\n",
               sensor_g, sensor_h);
        printf("STATE: Skip koreksi, langsung ke manuver belakang.\r\n");
        Motor_Stop_All();
        keadaan_robot = STATE_LINTASAN_1_MANUVER_BELAKANG;
      }
      break;

    case STATE_LINTASAN_1_MANUVER_BELAKANG:
      // Tujuan: Mundur pelan sampai jarak belakang < batas_jarak_belakang (5cm)
      if (sensor_e > batas_jarak_belakang && sensor_f > batas_jarak_belakang)
      {
        // Masih jauh, lanjutkan mundur pelan
        Motor_Reverse(kecepatan_motor);
      }
      else
      {
        // Target tercapai, berhenti
        Motor_Stop_All();
        printf("Mentok belakang tercapai. Capture #2 (Belakang)!\r\n");

        // ===================================================================
        // INCREMENT GROUP ID SEBELUM capture di posisi belakang
        // Foto dari belakang ini adalah awal group baru (group 2)
        // Robot sudah selesai dokumentasi group 1 (foto-foto di koridor pertama)
        // ===================================================================
        group_counter++; // Increment group ID (1 → 2)
        printf("\r\n========================================\r\n");
        printf("[Group] Entering new corridor section!\r\n");
        printf("[Group] Group ID incremented to: %d\r\n", group_counter);
        printf("[Group] Updating ESP32-CAM with new Group ID...\r\n");

        Vision_Status_t group_status = Vision_Set_Group_ID(group_counter);
        if (group_status == VISION_OK)
        {
          printf("[Group] Group ID updated successfully.\r\n");
          printf("[Group] Next photo will use Group ID %d.\r\n", group_counter);
        }
        else
        {
          printf("[Group] WARNING: Failed to update Group ID on ESP32-CAM!\r\n");
          printf("[Group] Photos will continue with old Group ID.\r\n");
        }
        printf("========================================\r\n\r\n");

        // Simpan state tujuan, lalu trigger capture
        state_selanjutnya_setelah_capture = STATE_LINTASAN_1_MAJU_DARI_BELAKANG;
        keadaan_robot = STATE_TRIGGER_CAPTURE;
      }
      break;

    case STATE_LINTASAN_1_MAJU_DARI_BELAKANG:
      // Tujuan: Maju sampai sensor belakang (e dan f) membaca jarak > jarak_stop_belakang (30cm)
      if (sensor_e < jarak_stop_belakang || sensor_f < jarak_stop_belakang)
      {
        // Masih terlalu dekat dengan dinding belakang, lanjutkan maju
        Motor_Forward(kecepatan_motor);
      }
      else
      {
        // Sudah mencapai jarak yang diinginkan, berhenti
        Motor_Stop_All();

        // Simpan state tujuan, lalu trigger capture
        state_selanjutnya_setelah_capture = STATE_LINTASAN_2_MAJU;
        keadaan_robot = STATE_TRIGGER_CAPTURE;
        waktu_terakhir_gerak = HAL_GetTick();
        sedang_bergerak = true;
      }
      break;

    case STATE_LINTASAN_2_MAJU:
      // Cek kondisi transisi state: jika ada halangan di depan
      if (ada_halangan_depan)
      {
        Motor_Stop_All();
        printf("STATE L2: Halangan depan terdeteksi. Mundur sedikit.\r\n");
        keadaan_robot = STATE_LINTASAN_2_MUNDUR_SEBENTAR;
        break;
      }

      // Logika gerak 2 detik jalan, 1 detik berhenti
      if (sedang_bergerak)
      {
        if (HAL_GetTick() - waktu_terakhir_gerak >= delay_jalan_ms)
        {
          Motor_Stop_All();

          // Simpan state tujuan, lalu trigger capture
          state_selanjutnya_setelah_capture = keadaan_robot; // Kembali ke state ini setelah capture
          keadaan_robot = STATE_TRIGGER_CAPTURE;
          sedang_bergerak = false;
          waktu_terakhir_gerak = HAL_GetTick();
        }
      }
      else
      {
        if (HAL_GetTick() - waktu_terakhir_gerak >= delay_berhenti_ms)
        {
          sedang_bergerak = true;
          waktu_terakhir_gerak = HAL_GetTick();
          printf("Melanjutkan gerak maju (L2)...\r\n");
        }
      }

      // Jika sedang dalam periode gerak, lakukan wall following
      if (sedang_bergerak)
      {
        if (sensor_a > batas_jarak_depan || sensor_b > batas_jarak_depan)
        {
          Motor_Forward(kecepatan_motor);
        }
        else
        {
          // Target tercapai, berhenti
          Motor_Stop_All();
          HAL_Delay(2000);
        }
      }
      break;

    case STATE_LINTASAN_2_MUNDUR_SEBENTAR:
      // Mundur hingga jarak depan > 20cm
      if (sensor_a < jarak_stop_depan || sensor_b < jarak_stop_depan)
      {
        Motor_Reverse(kecepatan_motor);
      }
      else
      {
        Motor_Stop_All();
        printf("STATE L2: Posisi mundur aman, siap putar 180.\r\n");

        // Reset yaw angle ke 0 sebelum mulai putar
        printf("Yaw angle di-reset ke 0.\r\n");
        yawAngle_deg = 0.0f;
        lastTick = HAL_GetTick(); // RESET TIMING untuk mencegah dt yang besar

        // FLAG-S minimum track
        keadaan_robot = STATE_ERROR;
        // keadaan_robot = STATE_LINTASAN_2_PUTAR_BALIK;
        waktu_mulai_putar_180 = HAL_GetTick(); // Mulai timer untuk timeout+
      }
      break;

    case STATE_LINTASAN_2_PUTAR_BALIK:
      // Timeout protection - maksimal 10 detik untuk putar 180
      if (HAL_GetTick() - waktu_mulai_putar_180 > 10000)
      {
        Motor_Stop_All();
        printf("STATE L2: Timeout putar 180° (10 detik), paksa lanjut!\r\n");
        keadaan_robot = STATE_LINTASAN_2_KOREKSI_LURUS;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_180 = 0;
        break;
      }

      // Cek apakah sudah putar lebih dari 180° (menggunakan absolute value)
      if (yawAngle_deg < -180.0f)
      {
        // Sudah putar 180° (meskipun belum tentu lurus)
        Motor_Stop_All();
        printf("STATE L2: Putaran 180° SELESAI! Yaw angle: %.1f°\r\n", yawAngle_deg);
        HAL_Delay(500);

        // Transisi ke koreksi lurus dengan sensor belakang
        printf("STATE L2: Masuk koreksi lurus dengan sensor belakang.\r\n");
        keadaan_robot = STATE_LINTASAN_2_MUNDUR_KOREKSI;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_180 = 0;
      }
      else
      {
        // Belum 180°, lanjut putar
        Motor_Rotate_Right(25);

        // Debug info (setiap ~500ms untuk tidak spam)
        static uint32_t last_debug_print = 0;
        if (HAL_GetTick() - last_debug_print > 500)
        {
          printf("Putar 180: Yaw=%.1f° | Target=180°\r\n", yawAngle_deg);
          last_debug_print = HAL_GetTick();
        }
      }
      break;

    case STATE_LINTASAN_2_MUNDUR_KOREKSI:
      if (sensor_e > batas_jarak_belakang && sensor_f > batas_jarak_belakang)
      {
        // Masih jauh, lanjutkan mundur pelan
        Motor_Reverse(kecepatan_motor);
      }
      else
      {
        // Target tercapai, berhenti
        Motor_Stop_All();
        printf("Mentok Siap koreksi!\r\n");
        keadaan_robot = STATE_LINTASAN_2_KOREKSI_LURUS;
      }
      break;

    case STATE_LINTASAN_2_KOREKSI_LURUS:
      // Timeout check - maksimal 5 detik untuk koreksi
      if (HAL_GetTick() - waktu_mulai_koreksi > 5000)
      {
        Motor_Stop_All();
        printf("STATE L2: Timeout koreksi lurus (5 detik), paksa lanjut.\r\n");
        keadaan_robot = STATE_LINTASAN_3_MAJU;
        waktu_terakhir_gerak = HAL_GetTick();
        sedang_bergerak = true;
        break;
      }

      // Validasi: Pastikan sensor belakang (E & F) mendeteksi dinding
      if ((sensor_e > 0 && sensor_e < 150) && (sensor_f > 0 && sensor_f < 150))
      {

        float selisih_belakang = fabs(sensor_e - sensor_f);

        if (selisih_belakang > 1.0f)
        {
          // Robot belum lurus, perlu koreksi
          counter_koreksi_stabil = 0; // Reset counter stabilitas

          if (sensor_e > sensor_f)
          {
            // Sensor E (belakang kiri) lebih jauh
            // Putar kiri untuk luruskan
            printf("Koreksi L2: Putar kiri | E:%.1f > F:%.1f | Diff:%.1f\r\n",
                   sensor_e, sensor_f, selisih_belakang);
            Motor_Rotate_Left(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
          else
          {
            // Sensor F (belakang kanan) lebih jauh
            // Putar kanan untuk luruskan
            printf("Koreksi L2: Putar kanan | F:%.1f > E:%.1f | Diff:%.1f\r\n",
                   sensor_f, sensor_e, selisih_belakang);
            Motor_Rotate_Right(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
        }
        else
        {
          // Robot sudah cukup lurus (selisih <= 1.0cm)
          counter_koreksi_stabil++;
          printf("Lurus L2! E:%.1f F:%.1f | Diff:%.1f | Stabil:%d/3\r\n",
                 sensor_e, sensor_f, selisih_belakang, counter_koreksi_stabil);

          if (counter_koreksi_stabil >= 3)
          {
            // Validasi: Sudah lurus 3x pembacaan berturut-turut
            Motor_Stop_All();
            printf("STATE L2: Koreksi lurus SELESAI! Lanjut Lintasan 3.\r\n");
            HAL_Delay(1000);
            keadaan_robot = STATE_LINTASAN_3_MAJU;
            waktu_terakhir_gerak = HAL_GetTick();
            sedang_bergerak = true;
          }
          else
          {
            // Tunggu pembacaan sensor berikutnya untuk validasi
            Motor_Stop_All();
            HAL_Delay(100);
          }
        }
      }
      else
      {
        // Sensor belakang tidak mendeteksi dinding (nilai 0 atau > 50cm)
        printf("Warning L2: Dinding belakang tidak terdeteksi (E:%.1f F:%.1f)\r\n",
               sensor_e, sensor_f);
        printf("STATE L2: Skip koreksi, langsung ke Lintasan 3.\r\n");
        Motor_Stop_All();
        keadaan_robot = STATE_LINTASAN_3_MAJU;
        waktu_terakhir_gerak = HAL_GetTick();
        sedang_bergerak = true;
      }
      break;

    case STATE_LINTASAN_3_MAJU:
      // Cek kondisi transisi state: jika ada halangan di depan
      if (ada_halangan_depan)
      {
        Motor_Stop_All();
        printf("STATE: Halangan depan terdeteksi. Masuk ke manuver.\r\n");
        keadaan_robot = STATE_LINTASAN_3_PUTAR_KANAN;
        break; // Langsung keluar untuk iterasi berikutnya
      }

      // Jika sedang dalam periode gerak, lakukan wall following
      if (sedang_bergerak)
      {
        if (sensor_a > batas_jarak_depan || sensor_b > batas_jarak_depan)
        {
          Motor_Forward(kecepatan_motor);
        }
        else
        {
          // Target tercapai, berhenti
          Motor_Stop_All();
          HAL_Delay(2000);
        }
      }
      waktu_mulai_putar_neg_90 = HAL_GetTick(); // Mulai timer untuk timeout
      yawAngle_deg = 0.0f;
      lastTick = HAL_GetTick(); // RESET TIMING untuk mencegah dt yang besar
      break;

    case STATE_LINTASAN_3_PUTAR_KANAN:
      if (yawAngle_deg < -90.0f)
      {
        // Sudah putar 180° (meskipun belum tentu lurus)
        Motor_Stop_All();
        printf("STATE L1: Putaran -90° SELESAI! Yaw angle: %.1f°\r\n", yawAngle_deg);
        HAL_Delay(500);

        // Transisi ke koreksi lurus dengan sensor belakang
        printf("STATE L3: Masuk koreksi lurus dengan sensor belakang.\r\n");
        keadaan_robot = STATE_LINTASAN_3_KOREKSI_LURUS;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_neg_90 = 0;
      }
      else
      {
        // Belum 90°, lanjut putar
        Motor_Rotate_Right(25);

        // Debug info (setiap ~500ms untuk tidak spam)
        static uint32_t last_debug_print = 0;
        if (HAL_GetTick() - last_debug_print > 500)
        {
          printf("Putar 180: Yaw=%.1f° | Target=180°\r\n", yawAngle_deg);
          last_debug_print = HAL_GetTick();
        }
      }
      break;

    case STATE_LINTASAN_3_KOREKSI_LURUS:
      // Timeout check - maksimal 5 detik untuk koreksi
      if (HAL_GetTick() - waktu_mulai_koreksi > 5000)
      {
        Motor_Stop_All();
        printf("STATE: Timeout koreksi lurus (5 detik), paksa lanjut.\r\n");
        keadaan_robot = STATE_LINTASAN_4_MAJU;
        break;
      }

      // Validasi: Pastikan sensor samping kanan (G & H) mendeteksi dinding
      if ((sensor_c > 0 && sensor_c < 50) && (sensor_d > 0 && sensor_d < 50))
      {

        float selisih_samping = fabs(sensor_c - sensor_d);

        if (selisih_samping > 0.5f)
        {
          // Robot belum lurus, perlu koreksi
          counter_koreksi_stabil = 0; // Reset counter stabilitas

          if (sensor_c > sensor_d)
          {
            // Kondisi: Bagian DEPAN robot lebih jauh dari dinding kanan
            // Artinya: Ekor robot lebih dekat ke dinding (robot miring ke kiri)
            // Solusi: Putar KANAN untuk meluruskan
            printf("Koreksi: Putar kanan | C:%.1f > D:%.1f | Diff:%.1f\r\n",
                   sensor_c, sensor_d, selisih_samping);
            Motor_Rotate_Left(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
          else
          {
            // Kondisi: Bagian BELAKANG robot lebih jauh dari dinding kanan
            // Artinya: Kepala robot lebih dekat ke dinding (robot miring ke kanan)
            // Solusi: Putar KIRI untuk meluruskan
            printf("Koreksi: Putar kiri | G:%.1f > H:%.1f | Diff:%.1f\r\n",
                   sensor_c, sensor_d, selisih_samping);
            Motor_Rotate_Right(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
        }
        else
        {
          // Robot sudah cukup lurus (selisih <= 0.5cm)
          counter_koreksi_stabil++;
          printf("Lurus! C:%.1f D:%.1f | Diff:%.1f | Stabil:%d/3\r\n",
                 sensor_c, sensor_d, selisih_samping, counter_koreksi_stabil);

          if (counter_koreksi_stabil >= 3)
          {
            // Validasi: Sudah lurus 3x pembacaan berturut-turut
            Motor_Stop_All();
            printf("STATE: Koreksi lurus SELESAI! Lanjut manuver belakang.\r\n");
            HAL_Delay(1000);
            keadaan_robot = STATE_LINTASAN_4_MAJU;
          }
          else
          {
            // Tunggu pembacaan sensor berikutnya untuk validasi
            Motor_Stop_All();
            HAL_Delay(100);
          }
        }
      }
      else
      {
        // Sensor samping kanan tidak mendeteksi dinding (nilai 0 atau > 50cm)
        printf("Warning: Dinding samping kanan tidak terdeteksi (C:%.1f D:%.1f)\r\n",
               sensor_c, sensor_d);
        printf("STATE: Skip koreksi, langsung ke maju.\r\n");
        Motor_Stop_All();
        keadaan_robot = STATE_LINTASAN_4_MAJU;
      }
      break;

    case STATE_LINTASAN_4_MAJU:
      // Cek kondisi transisi state: jika ada halangan di depan
      if (ada_halangan_depan)
      {
        Motor_Stop_All();
        printf("STATE: Halangan depan terdeteksi. Masuk ke manuver.\r\n");
        keadaan_robot = STATE_LINTASAN_4_PUTAR_BALIK;
        break; // Langsung keluar untuk iterasi berikutnya
      }

      // Jika sedang dalam periode gerak, lakukan wall following
      if (sedang_bergerak)
      {
        if (sensor_a > batas_jarak_depan || sensor_b > batas_jarak_depan)
        {
          Motor_Forward(kecepatan_motor);
        }
        else
        {
          // Target tercapai, berhenti
          Motor_Stop_All();
          HAL_Delay(2000);
        }

        /*
        if (lurus_dgn_kanan) {
            if (terlalu_jauh_kanan) {
                motor_slide_right(kecepatan_motor);
            } else if (terlalu_dekat_kanan) {
                motor_slide_left(kecepatan_motor);
            } else { // Jarak pas
                Motor_Forward(kecepatan_motor);
            }
        } else { // Jika tidak lurus, coba luruskan badan
            if (sensor_h > sensor_g) {
                Motor_Rotate_Right(kecepatan_motor);
            } else {
                Motor_Rotate_Left(kecepatan_motor);
            }
        }
        */
      }
      waktu_mulai_putar_neg_180 = HAL_GetTick(); // Mulai timer untuk timeout
      yawAngle_deg = 0.0f;
      lastTick = HAL_GetTick(); // RESET TIMING untuk mencegah dt yang besar
      break;

    case STATE_LINTASAN_4_PUTAR_BALIK:
      // Timeout protection - maksimal 10 detik untuk putar 180
      if (HAL_GetTick() - waktu_mulai_putar_neg_180 > 5000)
      {
        Motor_Stop_All();
        printf("STATE L4: Timeout putar -180° (10 detik), paksa lanjut!\r\n");
        keadaan_robot = STATE_LINTASAN_2_KOREKSI_LURUS;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_neg_180 = 0;
        break;
      }

      // Cek apakah sudah putar lebih dari 180° (menggunakan absolute value)
      if (yawAngle_deg < -250.0f)
      {
        // Sudah putar 180° (meskipun belum tentu lurus)
        Motor_Stop_All();
        printf("STATE L4: Putaran -180° SELESAI! Yaw angle: %.1f°\r\n", yawAngle_deg);
        HAL_Delay(500);

        // Transisi ke koreksi lurus dengan sensor belakang
        printf("STATE L4: Masuk mundur koreksi.\r\n");
        keadaan_robot = STATE_LINTASAN_4_MUNDUR_KOREKSI;
        waktu_mulai_koreksi = HAL_GetTick();
        counter_koreksi_stabil = 0;
        waktu_mulai_putar_neg_180 = 0;
      }
      else
      {
        // Belum 180°, lanjut putar
        Motor_Rotate_Left(25);

        // Debug info (setiap ~500ms untuk tidak spam)
        static uint32_t last_debug_print = 0;
        if (HAL_GetTick() - last_debug_print > 500)
        {
          printf("Putar -180: Yaw=%.1f° | Target=180°\r\n", yawAngle_deg);
          last_debug_print = HAL_GetTick();
        }
      }
      break;

    case STATE_LINTASAN_4_MUNDUR_KOREKSI:
      if (sensor_e > batas_jarak_belakang && sensor_f > batas_jarak_belakang)
      {
        // Masih jauh, lanjutkan mundur pelan
        Motor_Reverse(kecepatan_motor);
      }
      else
      {
        // Target tercapai, berhenti
        Motor_Stop_All();
        printf("Mentok Siap koreksi!\r\n");
        keadaan_robot = STATE_LINTASAN_4_KOREKSI_LURUS;
      }
      break;

    case STATE_LINTASAN_4_KOREKSI_LURUS:
      // Timeout check - maksimal 5 detik untuk koreksi
      if (HAL_GetTick() - waktu_mulai_koreksi > 5000)
      {
        Motor_Stop_All();
        printf("STATE L4: Timeout koreksi lurus (5 detik), paksa lanjut.\r\n");
        keadaan_robot = STATE_SELESAI;
        waktu_terakhir_gerak = HAL_GetTick();
        sedang_bergerak = true;
        break;
      }

      // Validasi: Pastikan sensor belakang (E & F) mendeteksi dinding
      if ((sensor_e > 0 && sensor_e < 150) && (sensor_f > 0 && sensor_f < 150))
      {

        float selisih_belakang = fabs(sensor_e - sensor_f);

        if (selisih_belakang > 1.0f)
        {
          // Robot belum lurus, perlu koreksi
          counter_koreksi_stabil = 0; // Reset counter stabilitas

          if (sensor_e > sensor_f)
          {
            // Sensor E (belakang kiri) lebih jauh
            // Putar kiri untuk luruskan
            printf("Koreksi L4: Putar kiri | E:%.1f > F:%.1f | Diff:%.1f\r\n",
                   sensor_e, sensor_f, selisih_belakang);
            Motor_Rotate_Left(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
          else
          {
            // Sensor F (belakang kanan) lebih jauh
            // Putar kanan untuk luruskan
            printf("Koreksi L4: Putar kanan | F:%.1f > E:%.1f | Diff:%.1f\r\n",
                   sensor_f, sensor_e, selisih_belakang);
            Motor_Rotate_Right(25);
            HAL_Delay(100);
            Motor_Stop_All();
            HAL_Delay(50);
          }
        }
        else
        {
          // Robot sudah cukup lurus (selisih <= 1.0cm)
          counter_koreksi_stabil++;
          printf("Lurus L4! E:%.1f F:%.1f | Diff:%.1f | Stabil:%d/3\r\n",
                 sensor_e, sensor_f, selisih_belakang, counter_koreksi_stabil);

          if (counter_koreksi_stabil >= 3)
          {
            // Validasi: Sudah lurus 3x pembacaan berturut-turut
            Motor_Stop_All();
            printf("STATE L4: Koreksi lurus SELESAI! Lanjut Lintasan 3.\r\n");
            HAL_Delay(1000);
            keadaan_robot = STATE_SELESAI;
            waktu_terakhir_gerak = HAL_GetTick();
            sedang_bergerak = true;
          }
          else
          {
            // Tunggu pembacaan sensor berikutnya untuk validasi
            Motor_Stop_All();
            HAL_Delay(100);
          }
        }
      }
      else
      {
        // Sensor belakang tidak mendeteksi dinding (nilai 0 atau > 50cm)
        printf("Warning L2: Dinding belakang tidak terdeteksi (E:%.1f F:%.1f)\r\n",
               sensor_e, sensor_f);
        printf("STATE L2: Skip koreksi, langsung ke Lintasan 3.\r\n");
        Motor_Stop_All();
        keadaan_robot = STATE_SELESAI;
        waktu_terakhir_gerak = HAL_GetTick();
        sedang_bergerak = true;
      }
      break;

    case STATE_SELESAI:
      printf("STATE: Siklus Selesai. Mengulang dari awal.\r\n");
      HAL_Delay(2000);
      keadaan_robot = STATE_START;
      break;

    case STATE_ERROR:
      printf("\r\n========================================\r\n");
      printf("CRITICAL ERROR: Robot entered ERROR state!\r\n");
      printf("========================================\r\n");
      printf("Stopping all motors...\r\n");
      printf("Activating STATE ERROR buzzer alert...\r\n");
      printf("Pattern: 5 short beeps → continuous slow beeps\r\n");
      printf("Press RESET button to restart.\r\n");
      printf("========================================\r\n\r\n");

      Motor_Stop_All();
      HAL_Delay(1000);            // Wait 1 second before buzzer
      buzzer_state_error_alert(); // Activate buzzer (infinite loop)
      break;

    default:
      printf("STATE: undefined! Masuk ke mode Error.\r\n");
      keadaan_robot = STATE_ERROR;
      break;
    }
    HAL_Delay(10); // Delay kecil untuk stabilitas
#endif
  } // Close while(1) loop

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

#if 0
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
#endif

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  /* USER CODE END TIM8_Init 2 */
}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 8799;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);
}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 4199;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function (ESP-01 WiFi Bridge)
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Trig_1_Pin | Trig_2_Pin | Trig_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Trig_8_Pin | Trig_7_Pin | Trig_6_Pin | Trig_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_4_GPIO_Port, Trig_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_0 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_1_Pin Trig_2_Pin Trig_3_Pin */
  GPIO_InitStruct.Pin = Trig_1_Pin | Trig_2_Pin | Trig_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_8_Pin Trig_7_Pin Trig_6_Pin Trig_5_Pin */
  GPIO_InitStruct.Pin = Trig_8_Pin | Trig_7_Pin | Trig_6_Pin | Trig_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_4_Pin */
  GPIO_InitStruct.Pin = Trig_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trig_4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pins : PC0 (GREEN BUTTON) and PC2 (RED BUTTON) */
  // Active LOW dengan internal pull-up
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 (BUZZER/LED indicator) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Set PC13 initial state to OFF
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Forward interrupt to sensor library
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  HC_SR04_Capture_Callback(htim);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
