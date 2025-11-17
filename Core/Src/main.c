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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Definisi setiap keadaan (state) yang mungkin untuk robot.
// Ini menggantikan flag boolean 'orientasi_...'
typedef enum {
	STATE_START,
	// Lintasan 1 (Normal)
	STATE_LINTASAN_1_MAJU,
	STATE_LINTASAN_1_MANUVER_DEPAN,
	STATE_LINTASAN_1_MUNDUR_DARI_DEPAN,
	STATE_LINTASAN_1_PUTAR_KIRI,
	STATE_LINTASAN_1_KOREKSI_LURUS,
	STATE_LINTASAN_1_MANUVER_BELAKANG, // State baru sesuai update
	STATE_LINTASAN_1_MAJU_DARI_BELAKANG,   // State baru sesuai update

	// Lintasan 2 (Normal)
	STATE_LINTASAN_2_MAJU,
	STATE_LINTASAN_2_MUNDUR_SEBENTAR,
	STATE_LINTASAN_2_PUTAR_180,
	STATE_LINTASAN_2_KOREKSI_LURUS_180,

	// Lintasan 3 (Terbalik)
	STATE_LINTASAN_3_MAJU,
	STATE_LINTASAN_3_MANUVER_DEPAN,
	STATE_LINTASAN_3_MUNDUR_DARI_DEPAN,
	STATE_LINTASAN_3_PUTAR_KANAN,

	// Lintasan 4 (Terbalik)
	STATE_LINTASAN_4_MAJU,
	STATE_LINTASAN_4_MANUVER_DEPAN,
	STATE_LINTASAN_4_MUNDUR_DARI_DEPAN,
	STATE_LINTASAN_4_PUTAR_180,

	STATE_SELESAI,
	STATE_ERROR
} KeadaanRobot_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Konstanta untuk parameter gerak dan sensor
#define batas_jarak_depan 10.0f //jarak robot dengan dinding untuk sensor depan
#define batas_jarak_belakang 10.0f //jarak robot dengan dingding untuk sensor belakang


#define batas_jauh_samping 22.0f
#define batas_dekat_samping 18.0f
#define kecepatan_motor 15
#define delay_jalan_ms 2000
#define delay_berhenti_ms 1000
#define take_photo_ms 1000
#define otw_mentok_depan 2.0f
#define otw_mentok_belakang 2.0f

#define batas_error_lurus 1.0f

#define jarak_stop_depan 20.0f //jarak robot mundur setelah mentok
#define jarak_stop_belakang 30.0f //jarak robot mundur setelah mentok
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Variabel utama untuk state machine
volatile KeadaanRobot_t keadaan_robot = STATE_START;

// Variabel untuk manuver dan timing
bool mentok = false;
uint32_t waktu_terakhir_gerak = 0;
bool sedang_bergerak = false;

// Variable untuk koreksi lurus
uint32_t waktu_mulai_koreksi = 0;
uint8_t counter_koreksi_stabil = 0;

// Variable untuk putar 180
uint32_t waktu_mulai_putar_180 = 0;

// Variable untuk MPU6050
MPU6050_t MPU6050;
float yawAngle_deg = 0.0f;
uint32_t lastTick = 0;
float dt = 0.0f;
char uart_buf[150];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
// Functions now in hcsr04_sensor.c library
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== Override printf untuk UART ====================
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize HC-SR04 sensor library
  HC_SR04_Delay_Init(&htim5);  // Initialize delay timer
  HAL_Delay(100);              // Wait for timers to stabilize
  HC_SR04_Init();              // Initialize all 8 sensors + start input capture
  HAL_Delay(200);              // Wait for sensors to settle after power-on

  // Dummy reads to clear any noise
  HC_SR04_Trigger_All();
  HAL_Delay(60);
  HC_SR04_Trigger_All();
  HAL_Delay(60);

  Motor_Init();

  // Initialize MPU6050
  MPU6050_Init(&hi2c1);
  printf("MPU6050 initialized.\r\n");
  HAL_Delay(100);

  // Initialize timing untuk yaw calculation
  lastTick = HAL_GetTick();

  printf("SYSTEM READY - CONTINUOUS FORWARD MODE\r\n");
  printf("Sensor settling complete.\r\n");
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    #define DIAGNOSTIC_MODE 0  // Set to 1 for motor test, 0 for normal operation

#if DIAGNOSTIC_MODE == 1
    // ========================================================================
    // DIAGNOSTIC: Print all 8 sensors with abbreviated locations (1 ROW)
    // ========================================================================
    // Trigger all sensors
//    HC_SR04_Trigger_All();
//    HAL_Delay(50); // Wait for echo capture
//
//    // Calculate distances
//    float dists[NUM_SENSORS];
//    for (int i = 0; i < NUM_SENSORS; i++) {
//        dists[i] = HC_SR04_Calculate_Distance(&sensors[i]);
//    }
//
//    // Print all 8 sensor values on a single line with abbreviations
//    // DKn:DepanKanan, DKi:DepanKiri, SKiD:SampingKiDpn, SKiB:SampingKiBlk
//    // BKi:BelakangKiri, BKn:BelakangKanan, SKnB:SampingKnBlk, SKnD:SampingKnDpn
//    printf("DKn:%.0f DKi:%.0f | SKiD:%.0f SKiB:%.0f | BKi:%.0f BKn:%.0f | SKnB:%.0f SKnD:%.0f\r\n",
//           dists[0], dists[1], dists[2], dists[3], dists[4], dists[5], dists[6], dists[7]);
//
//    HAL_Delay(100); // Repeat every 100ms
//

//	  Motor_Turn_Left(25);
//	  HAL_Delay(700);
//	  Motor_Stop_All();
//	  HAL_Delay(1500);

	  motor_slide_right(60);
	  HAL_Delay(1500);

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

    // 3. Update yaw angle dengan integrasi gyroscope
    yawAngle_deg += MPU6050.Gz * dt;

    // ========================================================================
    // LANGKAH 2: PROSES DATA SENSOR MENJADI INFORMASI
    // ========================================================================
    // Kondisi Jarak
    bool ada_halangan_depan = (sensor_a < batas_jarak_depan && sensor_a > 0) || (sensor_b < batas_jarak_depan && sensor_b > 0);
    bool ada_halangan_belakang = (sensor_e < batas_jarak_belakang) || (sensor_f < batas_jarak_belakang);

    // Kondisi Wall Following Kanan
    bool lurus_dgn_kanan = fabs(sensor_g - sensor_h) <= 1.0f;
    bool jarak_pas_kanan = (sensor_h < batas_jauh_samping && sensor_g < batas_jauh_samping) && (sensor_h > batas_dekat_samping && sensor_g > batas_dekat_samping);
    bool terlalu_jauh_kanan = (sensor_h > batas_jauh_samping && sensor_g > batas_jauh_samping);
    bool terlalu_dekat_kanan = (sensor_h < batas_dekat_samping && sensor_g < batas_dekat_samping);

    // Kondisi Wall Following Kiri
    bool lurus_dgn_kiri = fabs(sensor_c - sensor_d) <= 1.0f;
    bool terlalu_jauh_kiri = (sensor_c > batas_jauh_samping && sensor_d > batas_jauh_samping);
    bool terlalu_dekat_kiri = (sensor_c < batas_dekat_samping && sensor_d < batas_dekat_samping);


    // ========================================================================
    // LANGKAH 3: DEBUGGING PRINTF
    // ========================================================================
    // Cetak informasi penting untuk debugging di satu baris
    printf("State:%d | Dpn(A,B):%.0f,%.0f | Bkg(E,F):%.0f,%.0f",
    		keadaan_robot, sensor_a, sensor_b, sensor_e, sensor_f);


    // ========================================================================
    // LANGKAH 4: STATE MACHINE UTAMA
    // ========================================================================
    switch (keadaan_robot) {
        case STATE_START:
            printf("STATE: START -> LINTASAN_1_MAJU\r\n");

            //test langsung ke lintasan 2
            keadaan_robot = STATE_LINTASAN_2_MAJU;
            waktu_terakhir_gerak = HAL_GetTick();
            sedang_bergerak = true; // Mulai dengan bergerak
            break;


      // ======================= LINTASAN 1 ======================================

        case STATE_LINTASAN_1_MAJU:
            // Cek kondisi transisi state: jika ada halangan di depan
            if (ada_halangan_depan) {
                Motor_Stop_All();
                printf("STATE: Halangan depan terdeteksi. Masuk ke manuver.\r\n");
                keadaan_robot = STATE_LINTASAN_1_MUNDUR_DARI_DEPAN;
                break; // Langsung keluar untuk iterasi berikutnya
            }

            // Logika gerak 2 detik jalan, 1 detik berhenti
            if (sedang_bergerak) {
                if (HAL_GetTick() - waktu_terakhir_gerak >= delay_jalan_ms) {
                    Motor_Stop_All();
                    sedang_bergerak = false;
                    waktu_terakhir_gerak = HAL_GetTick();
                    printf("Berhenti sejenak...\r\n");
                }
            } else {
                if (HAL_GetTick() - waktu_terakhir_gerak >= delay_berhenti_ms) {
                    sedang_bergerak = true;
                    waktu_terakhir_gerak = HAL_GetTick();
                    printf("Melanjutkan gerak maju...\r\n");
                }
            }

            // Jika sedang dalam periode gerak, lakukan wall following
            if (sedang_bergerak) {
                // SEMENTARA DINONAKTIFKAN: Logika wall-following (geser/putar) di-comment.
                // Kita asumsikan robot lurus dan hanya fokus pada gerak maju.
                //Motor_Forward(kecepatan_motor);

            	if (sensor_a > batas_jarak_depan || sensor_b > batas_jarak_depan) {
            	   Motor_Forward(15);
            	   } else {
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
            break;

//        case STATE_LINTASAN_1_MANUVER_DEPAN:
//            // Tujuan: Maju pelan sampai jarak < 2cm
//            if (sensor_a > otw_mentok_depan && sensor_b > otw_mentok_depan) {
//                // Masih jauh, lanjutkan maju pelan
//                Motor_Forward(10);
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
            if (sensor_a < jarak_stop_depan || sensor_b < jarak_stop_depan) {
                // Masih terlalu dekat, lanjutkan mundur
                Motor_Reverse(15);
            } else {
                // Target tercapai, berhenti
                Motor_Stop_All();
                HAL_Delay(2000);
                printf("STATE: Posisi mundur aman tercapai, lanjut putar kiri.\r\n");
                keadaan_robot = STATE_LINTASAN_1_PUTAR_KIRI;
            }
            break;

        case STATE_LINTASAN_1_PUTAR_KIRI:
            // Cek apakah ada dinding di belakang untuk dijadikan referensi dan apakah robot sudah lurus.
            // Sensor harus membaca > 0 (valid) dan < 50cm (dinding terdeteksi)
            if ( (sensor_e > 0 && sensor_f > 0 && sensor_e < 50 && sensor_f < 50) && (fabs(sensor_e - sensor_f) <= batas_error_lurus) ) {
                // Dinding terdeteksi DAN robot sudah lurus.
                Motor_Stop_All();
                printf("STATE: Putaran selesai, robot lurus dengan dinding belakang.\r\n");
                HAL_Delay(2000);

                keadaan_robot = STATE_LINTASAN_1_KOREKSI_LURUS;
                waktu_mulai_koreksi = HAL_GetTick();
                counter_koreksi_stabil = 0;
                printf("STATE: Masuk koreksi lurus dengan sensor samping kanan.\r\n");
            } else {
                // Jika belum lurus atau belum ada dinding, lanjutkan berputar ke kiri.
                printf("Memutar ke kiri untuk mencari dinding belakang...\r\n");
                Motor_Rotate_Left(20);

            }
            break;

        case STATE_LINTASAN_1_KOREKSI_LURUS:
            // ========================================================================
            // TUJUAN: Fine-tuning kelurusan robot dengan dinding samping kanan
            // SENSOR: G (Samping Kanan Belakang) & H (Samping Kanan Depan)
            // ========================================================================

            // Timeout check - maksimal 5 detik untuk koreksi
            if (HAL_GetTick() - waktu_mulai_koreksi > 5000) {
                Motor_Stop_All();
                printf("STATE: Timeout koreksi lurus (5 detik), paksa lanjut.\r\n");
                keadaan_robot = STATE_LINTASAN_1_MANUVER_BELAKANG;
                break;
            }

            // Validasi: Pastikan sensor samping kanan (G & H) mendeteksi dinding
            if ( (sensor_g > 0 && sensor_g < 50) && (sensor_h > 0 && sensor_h < 50) ) {

                float selisih_samping = fabs(sensor_h - sensor_g);

                if (selisih_samping > 0.5f) {
                    // Robot belum lurus, perlu koreksi
                    counter_koreksi_stabil = 0;  // Reset counter stabilitas

                    if (sensor_h > sensor_g) {
                        // Kondisi: Bagian DEPAN robot lebih jauh dari dinding kanan
                        // Artinya: Ekor robot lebih dekat ke dinding (robot miring ke kiri)
                        // Solusi: Putar KANAN untuk meluruskan
                        printf("Koreksi: Putar kanan | H:%.1f > G:%.1f | Diff:%.1f\r\n",
                               sensor_h, sensor_g, selisih_samping);
                        Motor_Rotate_Right(15);
                        HAL_Delay(100);
                        Motor_Stop_All();
                        HAL_Delay(50);

                    } else {
                        // Kondisi: Bagian BELAKANG robot lebih jauh dari dinding kanan
                        // Artinya: Kepala robot lebih dekat ke dinding (robot miring ke kanan)
                        // Solusi: Putar KIRI untuk meluruskan
                        printf("Koreksi: Putar kiri | G:%.1f > H:%.1f | Diff:%.1f\r\n",
                               sensor_g, sensor_h, selisih_samping);
                        Motor_Rotate_Left(15);
                        HAL_Delay(100);
                        Motor_Stop_All();
                        HAL_Delay(50);
                    }

                } else {
                    // Robot sudah cukup lurus (selisih <= 0.5cm)
                    counter_koreksi_stabil++;
                    printf("Lurus! H:%.1f G:%.1f | Diff:%.1f | Stabil:%d/3\r\n",
                           sensor_h, sensor_g, selisih_samping, counter_koreksi_stabil);

                    if (counter_koreksi_stabil >= 3) {
                        // Validasi: Sudah lurus 3x pembacaan berturut-turut
                        Motor_Stop_All();
                        printf("STATE: Koreksi lurus SELESAI! Lanjut manuver belakang.\r\n");
                        HAL_Delay(1000);
                        keadaan_robot = STATE_LINTASAN_1_MANUVER_BELAKANG;
                    } else {
                        // Tunggu pembacaan sensor berikutnya untuk validasi
                        Motor_Stop_All();
                        HAL_Delay(100);
                    }
                }

            } else {
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
            if (sensor_e > batas_jarak_belakang && sensor_f > batas_jarak_belakang) {
                // Masih jauh, lanjutkan mundur pelan
                Motor_Reverse(15);
            } else {
                // Target tercapai, berhenti
                Motor_Stop_All();
                printf("Mentok belakang tercapai. Capture #2 (Belakang)!\r\n");
                HAL_Delay(take_photo_ms); // Jeda untuk capture masih blocking, bisa kita perbaiki nanti
                printf("STATE: Selesai manuver belakang, lanjut maju dari belakang.\r\n");
                keadaan_robot = STATE_LINTASAN_1_MAJU_DARI_BELAKANG;
            }
            break;

        case STATE_LINTASAN_1_MAJU_DARI_BELAKANG:
            // Tujuan: Maju sampai sensor belakang (e dan f) membaca jarak > jarak_stop_belakang (30cm)
            if (sensor_e < jarak_stop_belakang || sensor_f < jarak_stop_belakang) {
                // Masih terlalu dekat dengan dinding belakang, lanjutkan maju
                Motor_Forward(15);
            } else {
                // Sudah mencapai jarak yang diinginkan, berhenti
                Motor_Stop_All();
                printf("STATE: Maju dari belakang selesai, siap untuk lintasan 2.\r\n");
                keadaan_robot = STATE_LINTASAN_2_MAJU;
                // Reset timer untuk state berikutnya jika STATE_LINTASAN_2_MAJU menggunakan timer
                waktu_terakhir_gerak = HAL_GetTick();
                sedang_bergerak = true;
            }
            break;

        case STATE_LINTASAN_2_MAJU:
            // Logika maju sama persis dengan Lintasan 1
            if (ada_halangan_depan) {
                Motor_Stop_All();
                printf("STATE L2: Halangan depan terdeteksi. Mundur sedikit.\r\n");
                keadaan_robot = STATE_LINTASAN_2_MUNDUR_SEBENTAR;
                break;
            }

            // Logika gerak 2 detik jalan, 1 detik berhenti
            if (sedang_bergerak) {
                if (HAL_GetTick() - waktu_terakhir_gerak >= delay_jalan_ms) {
                    Motor_Stop_All();
                    sedang_bergerak = false;
                    waktu_terakhir_gerak = HAL_GetTick();
                    printf("Berhenti sejenak (L2)...\r\n");
                }
            } else {
                if (HAL_GetTick() - waktu_terakhir_gerak >= delay_berhenti_ms) {
                    sedang_bergerak = true;
                    waktu_terakhir_gerak = HAL_GetTick();
                    printf("Melanjutkan gerak maju (L2)...\r\n");
                }
            }

            // Jika sedang dalam periode gerak, maju terus
            if (sedang_bergerak) {
                //Motor_Forward(kecepatan_motor);
            	if (sensor_a > batas_jarak_depan || sensor_b > batas_jarak_depan) {
            	    Motor_Forward(15);
            	    } else {
            	    // Target tercapai, berhenti
            	      Motor_Stop_All();
            	      HAL_Delay(2000);
            	    }

            }
            break;

        case STATE_LINTASAN_2_MUNDUR_SEBENTAR:
            // Mundur hingga jarak depan > 20cm
            if (sensor_a < jarak_stop_depan || sensor_b < jarak_stop_depan) {
                Motor_Reverse(15);
            } else {
                Motor_Stop_All();
                printf("STATE L2: Posisi mundur aman, siap putar 180.\r\n");

                // Reset yaw angle ke 0 sebelum mulai putar
                yawAngle_deg = 0.0f;
                printf("Yaw angle di-reset ke 0.\r\n");

                keadaan_robot = STATE_LINTASAN_2_PUTAR_180;
                waktu_mulai_putar_180 = HAL_GetTick(); // Mulai timer untuk timeout
            }
            break;

        case STATE_LINTASAN_2_PUTAR_180:
            // ========================================================================
            // TUJUAN: Putar 180 derajat dengan MPU6050 (yaw angle)
            // STRATEGI: Reset yaw = 0, putar sampai yaw > 180°, lalu koreksi lurus
            // ========================================================================

            // Timeout protection - maksimal 10 detik untuk putar 180
            if (HAL_GetTick() - waktu_mulai_putar_180 > 10000) {
                Motor_Stop_All();
                printf("STATE L2: Timeout putar 180° (10 detik), paksa lanjut!\r\n");
                keadaan_robot = STATE_LINTASAN_2_KOREKSI_LURUS_180;
                waktu_mulai_koreksi = HAL_GetTick();
                counter_koreksi_stabil = 0;
                waktu_mulai_putar_180 = 0;
                break;
            }

            // Cek apakah sudah putar lebih dari 180° (menggunakan absolute value)
            if (fabs(yawAngle_deg) > 180.0f) {
                // Sudah putar 180° (meskipun belum tentu lurus)
                Motor_Stop_All();
                printf("STATE L2: Putaran 180° SELESAI! Yaw angle: %.1f°\r\n", yawAngle_deg);
                HAL_Delay(500);

                // Transisi ke koreksi lurus dengan sensor belakang
                printf("STATE L2: Masuk koreksi lurus dengan sensor belakang.\r\n");
                keadaan_robot = STATE_LINTASAN_2_KOREKSI_LURUS_180;
                waktu_mulai_koreksi = HAL_GetTick();
                counter_koreksi_stabil = 0;
                waktu_mulai_putar_180 = 0;

            } else {
                // Belum 180°, lanjut putar
                Motor_Rotate_Right(25);

                // Debug info (setiap ~500ms untuk tidak spam)
                static uint32_t last_debug_print = 0;
                if (HAL_GetTick() - last_debug_print > 500) {
                    printf("Putar 180: Yaw=%.1f° | Target=180°\r\n", yawAngle_deg);
                    last_debug_print = HAL_GetTick();
                }
            }
            break;

        case STATE_LINTASAN_2_KOREKSI_LURUS_180:
            // ========================================================================
            // TUJUAN: Fine-tuning kelurusan robot dengan sensor belakang (E & F)
            // Setelah putar 180° dengan MPU6050, pastikan lurus dengan dinding
            // ========================================================================

            // Timeout check - maksimal 5 detik untuk koreksi
            if (HAL_GetTick() - waktu_mulai_koreksi > 5000) {
                Motor_Stop_All();
                printf("STATE L2: Timeout koreksi lurus (5 detik), paksa lanjut.\r\n");
                keadaan_robot = STATE_LINTASAN_3_MAJU;
                waktu_terakhir_gerak = HAL_GetTick();
                sedang_bergerak = true;
                break;
            }

            // Validasi: Pastikan sensor belakang (E & F) mendeteksi dinding
            if ( (sensor_e > 0 && sensor_e < 50) && (sensor_f > 0 && sensor_f < 50) ) {

                float selisih_belakang = fabs(sensor_e - sensor_f);

                if (selisih_belakang > 1.0f) {
                    // Robot belum lurus, perlu koreksi
                    counter_koreksi_stabil = 0;  // Reset counter stabilitas

                    if (sensor_e > sensor_f) {
                        // Sensor E (belakang kiri) lebih jauh
                        // Putar kiri untuk luruskan
                        printf("Koreksi L2: Putar kiri | E:%.1f > F:%.1f | Diff:%.1f\r\n",
                               sensor_e, sensor_f, selisih_belakang);
                        Motor_Rotate_Left(15);
                        HAL_Delay(100);
                        Motor_Stop_All();
                        HAL_Delay(50);

                    } else {
                        // Sensor F (belakang kanan) lebih jauh
                        // Putar kanan untuk luruskan
                        printf("Koreksi L2: Putar kanan | F:%.1f > E:%.1f | Diff:%.1f\r\n",
                               sensor_f, sensor_e, selisih_belakang);
                        Motor_Rotate_Right(15);
                        HAL_Delay(100);
                        Motor_Stop_All();
                        HAL_Delay(50);
                    }

                } else {
                    // Robot sudah cukup lurus (selisih <= 1.0cm)
                    counter_koreksi_stabil++;
                    printf("Lurus L2! E:%.1f F:%.1f | Diff:%.1f | Stabil:%d/3\r\n",
                           sensor_e, sensor_f, selisih_belakang, counter_koreksi_stabil);

                    if (counter_koreksi_stabil >= 3) {
                        // Validasi: Sudah lurus 3x pembacaan berturut-turut
                        Motor_Stop_All();
                        printf("STATE L2: Koreksi lurus SELESAI! Lanjut Lintasan 3.\r\n");
                        HAL_Delay(1000);
                        keadaan_robot = STATE_LINTASAN_3_MAJU;
                        waktu_terakhir_gerak = HAL_GetTick();
                        sedang_bergerak = true;
                    } else {
                        // Tunggu pembacaan sensor berikutnya untuk validasi
                        Motor_Stop_All();
                        HAL_Delay(100);
                    }
                }

            } else {
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
             printf("STATE: LINTASAN 3 MAJU (Belum diimplementasikan penuh).\r\n");
             // TODO: Implementasi logika maju untuk lintasan 3 (wall following KIRI)
             HAL_Delay(3000);
             keadaan_robot = STATE_LINTASAN_4_MAJU; // Langsung skip untuk contoh
             break;

        case STATE_LINTASAN_4_MAJU:
             printf("STATE: LINTASAN 4 MAJU (Belum diimplementasikan penuh).\r\n");
             // TODO: Implementasi logika maju untuk lintasan 4 (wall following KIRI)
             HAL_Delay(3000);
             keadaan_robot = STATE_SELESAI; // Langsung skip untuk contoh
             break;

        case STATE_SELESAI:
            printf("STATE: Siklus Selesai. Mengulang dari awal.\r\n");
            HAL_Delay(2000);
            keadaan_robot = STATE_START;
            break;

        case STATE_ERROR:
            printf("STATE: ERROR! Robot berhenti.\r\n");
            Motor_Stop_All();
            break;

        default:
            printf("STATE: undefined! Masuk ke mode Error.\r\n");
            keadaan_robot = STATE_ERROR;
            break;
    }
    HAL_Delay(10); // Delay kecil untuk stabilitas
#endif
  }  // Close while(1) loop

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  htim1.Init.Prescaler = 83;
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
  htim5.Init.Prescaler = 15;
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
  htim8.Init.Prescaler = 83;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Trig_1_Pin|Trig_2_Pin|Trig_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Trig_8_Pin|Trig_7_Pin|Trig_6_Pin|Trig_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trig_4_GPIO_Port, Trig_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Trig_1_Pin Trig_2_Pin Trig_3_Pin */
  GPIO_InitStruct.Pin = Trig_1_Pin|Trig_2_Pin|Trig_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Trig_8_Pin Trig_7_Pin Trig_6_Pin Trig_5_Pin */
  GPIO_InitStruct.Pin = Trig_8_Pin|Trig_7_Pin|Trig_6_Pin|Trig_5_Pin;
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
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Forward interrupt to sensor library
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
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

#ifdef  USE_FULL_ASSERT
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
