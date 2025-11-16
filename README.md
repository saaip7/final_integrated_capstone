# Final Integrated Project: HC-SR04 Sensors + BTS7960 Motor Control

**Date:** 2025-11-11
**Board:** STM32F407VG Discovery
**Clock:** 168 MHz (HSI + PLL)

---

## 📋 Project Overview

Integrated system combining:
- **8× HC-SR04 ultrasonic sensors** for obstacle detection
- **4× BTS7960 motor drivers** for DC motor control
- **Simple threshold-based PWM control** (distance-based start/stop)

---

## 🚀 Quick Start

### 1. Hardware Connections

**Sensors (HC-SR04):**
| Sensor | Echo Pin | Trigger Pin | Timer |
|--------|----------|-------------|-------|
| US1 | PE9 (TIM1_CH1) | PE10 | TIM1 |
| US2 | PE11 (TIM1_CH2) | PE12 | TIM1 |
| US3-US8 | ... | ... | TIM1/TIM8 |

**Motors (BTS7960):**
| Motor | RPWM Pin | LPWM Pin | Timer | Status |
|-------|----------|----------|-------|--------|
| Motor 4 | PE5 (TIM9_CH1) | PE6 (TIM9_CH2) | TIM9 | ✅ Tested |
| Motor 1-3 | PA0, PB0, PB1 | - | TIM2, TIM3 | Single channel |

### 2. Build & Flash

```bash
# In STM32CubeIDE:
1. Build project (Ctrl+B)
2. Flash to STM32 (Run/Debug)
3. Open serial monitor (115200 baud)
```

### 3. Expected Output

```
SYSTEM READY - SIMPLE THRESHOLD TEST
Distance threshold: 10.0 cm
Distance > 10cm: PWM 50%
Distance < 10cm: PWM 0% (STOP)

NO OBSTACLE (undefined) - PWM 50%
US1: 0.0cm | US2: 0.0cm | US3: 0.0cm | ...
```

---

## 🎯 System Logic

**Simple threshold control:**
```
Sensor Reading:
├── 0cm (no obstacle detected/undefined) → PWM 50% (JALAN)
├── distance < 10cm → PWM 0% (STOP)
└── distance ≥ 10cm → PWM 50% (JALAN)
```

**Safety feature:**
- Robot only stops when it DEFINITELY sees obstacle <10cm
- If sensor returns 0cm (too far/undefined) → keep moving
- Prevents unexpected stops in open areas

---

## 📁 Project Structure

```
final_integrated/
├── Core/
│   ├── Inc/
│   │   ├── hcsr04_sensor.h      # Sensor library API
│   │   └── motor_control.h      # Motor library API
│   │
│   ├── Src/
│   │   ├── main.c               # Main control loop (~200 lines)
│   │   ├── hcsr04_sensor.c      # Sensor implementation
│   │   └── motor_control.c      # Motor implementation
│   │
├── camera_etc.ioc               # STM32CubeMX config
└── README.md                    # This file
```

---

## 🔧 Key Configuration

### Timer Usage

| Timer | Clock | Prescaler | Purpose |
|-------|-------|-----------|---------|
| TIM1 | 168 MHz | 167 | Input Capture (US1-4), 1 tick = 1 μs |
| TIM5 | 84 MHz | 83 | Base timer for delay_us() |
| TIM8 | 168 MHz | 167 | Input Capture (US5-8), 1 tick = 1 μs |
| TIM9 | 168 MHz | 1 | PWM ~20 kHz for Motor 4 |
| TIM2/TIM3 | 84 MHz | 1 | PWM ~10 kHz for Motor 1-3 |

### Clock Tree
```
HSI 16 MHz → PLL (×168/16/2) → 168 MHz System Clock
├── APB1: 42 MHz → Timers: 84 MHz (TIM2,3,5)
└── APB2: 84 MHz → Timers: 168 MHz (TIM1,8,9)
```

### Distance Calculation
```c
// Speed of sound: 343 m/s = 0.0343 cm/μs
// Formula already includes ÷2 for round trip
distance_cm = time_us × 0.01715
```

---

## 📝 API Usage

### Sensor Library

```c
#include "hcsr04_sensor.h"

// Initialize (call once in main)
HC_SR04_Delay_Init(&htim5);
HC_SR04_Init();

// In main loop
HC_SR04_Trigger_All();
// Wait for captures...
float distance = HC_SR04_Calculate_Distance(&sensors[0]);
HC_SR04_Update_Filter(&sensors[0], distance);
HC_SR04_Print_Data();  // Print all sensors via UART
```

### Motor Library

```c
#include "motor_control.h"

// Initialize (call once in main)
Motor_Init();

// Simple PWM control
Motor_Test_TIM9_PWM(50);  // 50% duty cycle on PE5 & PE6
Motor_Test_TIM9_PWM(0);   // Stop (0% duty)

// Advanced control (all 4 motors)
Motor_SetSpeed(MOTOR_4, 50);   // Forward 50%
Motor_SetSpeed(MOTOR_4, -50);  // Reverse 50%
Motor_Stop_All();              // Emergency stop
```

---

## 🐛 Troubleshooting

### Distance readings 2x actual value
**Fixed:** Timer prescaler corrected from 83 to 167 (TIM1/TIM8)

### Robot stops when no obstacle detected (0cm reading)
**Fixed:** Logic changed to treat 0cm as "clear path" → keep moving

### PWM doesn't respond to obstacle
**Fixed:** Threshold logic implemented in main loop

---

## ✅ Testing Checklist

- [x] Sensor library works (8 sensors reading correctly)
- [x] Timer prescaler fixed (distance accuracy verified)
- [x] Motor PWM visible on oscilloscope (PE5/PE6)
- [x] Threshold logic working (10cm stop/go)
- [x] Undefined sensor handling (0cm = keep moving)
- [ ] Full motor integration (all 4 motors)
- [ ] Obstacle avoidance tuning (adjust threshold/speed)

---

## 🔗 References

- **STM32F407VG Datasheet:** https://www.st.com/resource/en/datasheet/stm32f407vg.pdf
- **HC-SR04 Datasheet:** Valid range 2.5-400 cm, 10μs trigger pulse
- **BTS7960 Driver:** H-bridge, RPWM/LPWM control, 43A max

---

**Project Status:** ✅ Core functionality complete, ready for fine-tuning
