/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "pid.h"
#include "pwm.h"
#include "sd_card.h"    // SD Card (SDIO)
#include "w25q128.h"    // SPI Flash
#include "bmp280.h"     // Barometer
#include "button.h"     // Boot button
#include "led.h"       // RGB LED
#include "gps.h"       // GPS Module
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Flight control system status
typedef enum {
  FC_ARMED,
  FC_DISARMED,
  FC_ERROR
} FC_Status;

// Flight modes
typedef enum {
  FLIGHT_MODE_MANUAL,
  FLIGHT_MODE_STABILIZE,
  FLIGHT_MODE_POS_HOLD,
  FLIGHT_MODE_RETURN_HOME
} Flight_Mode;

// Storage system configuration
typedef struct {
  uint8_t config_valid;
  float roll_pid_Kp;
  float roll_pid_Ki;
  float roll_pid_Kd;
  float pitch_pid_Kp;
  float pitch_pid_Ki;
  float pitch_pid_Kd;
  float yaw_pid_Kp;
  float yaw_pid_Ki;
  float yaw_pid_Kd;
  float alt_pid_Kp;
  float alt_pid_Ki;
  float alt_pid_Kd;
  float sea_level_pressure;
  double home_latitude;
  double home_longitude;
} FC_Config;

// Sensor data structure
typedef struct {
  IMU_Data imu;
  BMP280_Data baro;
  GPS_Data gps;
} SensorData;

// Navigation data
typedef struct {
  float target_altitude;
  double target_latitude;
  double target_longitude;
  float distance_to_target;
  float bearing_to_target;
} NavigationData;

// LED color definitions
typedef enum {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_YELLOW,
    LED_CYAN,
    LED_MAGENTA,
    LED_WHITE
} LED_Color;

// Button states
typedef enum {
    BUTTON_RELEASED = 0,
    BUTTON_PRESSED,
    BUTTON_HELD
} Button_State;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// SD Card parameters
#define SD_LOG_FILENAME    "flight.csv"
#define SD_GPS_LOG_FILENAME "gps_log.csv"
#define SD_CONFIG_FILENAME "config.txt"

// SPI Flash parameters
#define FLASH_CONFIG_ADDR  0x000000
#define FLASH_CONFIG_MAGIC 0xAA

// Timing parameters
#define LOGGING_INTERVAL_MS 100  // 10Hz logging
#define STATUS_UPDATE_MS    200  // 5Hz status updates
#define BUTTON_HOLD_MS     3000  // 3s for button hold
#define GPS_PROCESS_MS     100   // 10Hz GPS processing
#define NAVIGATION_UPDATE_MS 200 // 5Hz navigation updates

// Safety parameters
#define MAX_DISTANCE_FROM_HOME 500.0f // meters
#define MIN_GPS_SATELLITES 6
#define GPS_ALTITUDE_DEADBAND 2.0f // meters
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// Safety macros
#define MOTORS_ARM()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define MOTORS_DISARM() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

// Button macro
#define IS_BUTTON_PRESSED() (HAL_GPIO_ReadPin(BOOT_BUTTON_PORT, BOOT_BUTTON_PIN) == GPIO_PIN_RESET)

// GPS macros
#define GPS_HAS_FIX() (sensors.gps.fix_quality > 0 && sensors.gps.satellites >= MIN_GPS_SATELLITES)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Storage functions
void FC_SaveConfig(void);
void FC_LoadConfig(void);
void FC_LogData(float* accel, float* gyro, float altitude);
void FC_LogGPSData(void);

// Safety functions
void FC_ArmMotors(void);
void FC_DisarmMotors(void);
void FC_EmergencyStop(void);

// Navigation functions
void FC_UpdateNavigation(void);
void FC_SetHomePosition(void);
void FC_ReturnToHome(void);

// LED status functions
void LED_ShowSystemStatus(void);
void LED_ShowGPSStatus(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
// Hardware pin definitions (adjust according to your schematic)
#define MOTOR1_PIN GPIO_PIN_8
#define MOTOR1_PORT GPIOA
#define MOTOR2_PIN GPIO_PIN_9
#define MOTOR2_PORT GPIOA
#define MOTOR3_PIN GPIO_PIN_10
#define MOTOR3_PORT GPIOA
#define MOTOR4_PIN GPIO_PIN_11
#define MOTOR4_PORT GPIOA

// SD Card detect pin
#define SD_CD_PIN GPIO_PIN_12
#define SD_CD_PORT GPIOC

// SPI Flash chip select
#define FLASH_CS_PIN GPIO_PIN_12
#define FLASH_CS_PORT GPIOB

// Boot Button (Active Low)
#define BOOT_BUTTON_PIN GPIO_PIN_0
#define BOOT_BUTTON_PORT GPIOA

// RGB LED Pins (PWM capable pins recommended)
#define LED_RED_PIN GPIO_PIN_6
#define LED_RED_PORT GPIOB
#define LED_GREEN_PIN GPIO_PIN_7
#define LED_GREEN_PORT GPIOB
#define LED_BLUE_PIN GPIO_PIN_8
#define LED_BLUE_PORT GPIOB

// GPS UART
#define GPS_UART huart6
#define GPS_BAUDRATE 9600

// System parameters
#define DEFAULT_SEA_LEVEL_PRESSURE 1013.25f // hPa
#define SAFE_MOTOR_PULSE_MIN 1000
#define SAFE_MOTOR_PULSE_MAX 2000
#define DEFAULT_TARGET_ALTITUDE 10.0f // meters
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
