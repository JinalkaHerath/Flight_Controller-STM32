#include "main.h"
#include "imu.h"
#include "pid.h"
#include "pwm.h"
#include "sd_card.h"
#include "w25q128.h"
#include "bmp280.h"
#include "button.h"
#include "led.h"
#include "gps.h"

// Global variables
SensorData sensors;
PID_Controller roll_pid, pitch_pid, yaw_pid, alt_pid;
NavigationData navigation;
FC_Config config;
uint16_t motor_pulse[4] = {1000, 1000, 1000, 1000};
FC_Status system_status = FC_DISARMED;
Flight_Mode flight_mode = FLIGHT_MODE_STABILIZE;
char log_buffer[256];

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();      // IMU and Barometer
    MX_TIM1_Init();      // Motor PWM
    MX_TIM4_Init();      // LED PWM
    MX_SPI1_Init();      // SPI Flash
    MX_SDIO_SD_Init();   // SD Card
    MX_USART6_UART_Init(); // GPS UART

    // Initialize modules
    Button_Init();
    LED_Init();
    IMU_Init(&hi2c1);
    if(!BMP280_Init(&hi2c1)) Error_Handler();
    PWM_Init(&htim1);
    GPS_Init(&GPS_UART);

    // Initialize PID controllers with default values
    PID_Init(&roll_pid, 0.8f, 0.05f, 0.2f, -500, 500);
    PID_Init(&pitch_pid, 0.8f, 0.05f, 0.2f, -500, 500);
    PID_Init(&yaw_pid, 0.5f, 0.01f, 0.1f, -200, 200);
    PID_Init(&alt_pid, 1.2f, 0.1f, 0.5f, -300, 300);

    // Load configuration
    W25Q128_Init(&hspi1);
    SD_Init();
    FC_LoadConfig();

    // Set default navigation parameters
    navigation.target_altitude = DEFAULT_TARGET_ALTITUDE;
    if(config.home_latitude == 0 && config.home_longitude == 0) {
        FC_SetHomePosition(); // Set home position if not configured
    }

    // Initial LED status sequence
    LED_SetColor(LED_BLUE);
    HAL_Delay(500);
    LED_SetColor(LED_GREEN);
    HAL_Delay(500);
    LED_SetColor(LED_RED);
    HAL_Delay(500);
    LED_SetColor(LED_OFF);

    // Timing variables
    uint32_t last_log_time = 0;
    uint32_t last_baro_time = 0;
    uint32_t last_status_time = 0;
    uint32_t last_gps_time = 0;
    uint32_t last_nav_time = 0;

    while (1) {
        // Handle button input
        Button_State btn_state = Button_GetState();
        if(btn_state == BUTTON_PRESSED) {
            if(system_status == FC_DISARMED) {
                FC_ArmMotors();
                system_status = FC_ARMED;
                LED_SetColor(LED_GREEN);
            } else {
                FC_DisarmMotors();
                system_status = FC_DISARMED;
                LED_SetColor(LED_RED);
            }
        } else if(btn_state == BUTTON_HELD) {
            FC_EmergencyStop();
        }

        // 1. Read IMU data (500Hz)
        IMU_ReadData(&sensors.imu);

        // 2. Read Barometer data (50Hz)
        if(HAL_GetTick() - last_baro_time >= 20) {
            BMP280_ReadData(&hi2c1, &sensors.baro);
            BMP280_CalculateAltitude(&sensors.baro, config.sea_level_pressure);
            last_baro_time = HAL_GetTick();
        }

        // 3. Process GPS data (10Hz)
        if(HAL_GetTick() - last_gps_time >= GPS_PROCESS_MS) {
            if(GPS_ParseNMEA(&sensors.gps)) {
                if(sensors.gps.data_valid) {
                    FC_LogGPSData();

                    // Update home position if we don't have one
                    if(config.home_latitude == 0 && config.home_longitude == 0) {
                        FC_SetHomePosition();
                    }
                }
            }
            last_gps_time = HAL_GetTick();
        }

        // 4. Update navigation (5Hz)
        if(HAL_GetTick() - last_nav_time >= NAVIGATION_UPDATE_MS) {
            FC_UpdateNavigation();

            // Handle flight modes
            switch(flight_mode) {
                case FLIGHT_MODE_RETURN_HOME:
                    FC_ReturnToHome();
                    break;

                case FLIGHT_MODE_POS_HOLD:
                    // Implement position hold logic here
                    break;

                case FLIGHT_MODE_STABILIZE:
                case FLIGHT_MODE_MANUAL:
                default:
                    // Manual control or basic stabilization
                    break;
            }
            last_nav_time = HAL_GetTick();
        }

        // 5. Run PID control (250Hz) - only if armed
        static uint32_t last_pid_time = 0;
        if(system_status == FC_ARMED && HAL_GetTick() - last_pid_time >= 4) {
            float roll_output = PID_Update(&roll_pid, 0 - sensors.imu.gyro[0], 0.004f);
            float pitch_output = PID_Update(&pitch_pid, 0 - sensors.imu.gyro[1], 0.004f);
            float yaw_output = PID_Update(&yaw_pid, 0 - sensors.imu.gyro[2], 0.004f);
            float alt_output = PID_Update(&alt_pid, navigation.target_altitude - sensors.baro.altitude, 0.004f);

            // Motor mixing with altitude control
            motor_pulse[0] = SAFE_MOTOR_PULSE_MIN + roll_output - pitch_output + yaw_output + alt_output;
            motor_pulse[1] = SAFE_MOTOR_PULSE_MIN - roll_output - pitch_output - yaw_output + alt_output;
            motor_pulse[2] = SAFE_MOTOR_PULSE_MIN + roll_output + pitch_output + yaw_output + alt_output;
            motor_pulse[3] = SAFE_MOTOR_PULSE_MIN - roll_output + pitch_output - yaw_output + alt_output;

            // Constrain motor outputs
            for(int i = 0; i < 4; i++) {
                if(motor_pulse[i] < SAFE_MOTOR_PULSE_MIN) motor_pulse[i] = SAFE_MOTOR_PULSE_MIN;
                if(motor_pulse[i] > SAFE_MOTOR_PULSE_MAX) motor_pulse[i] = SAFE_MOTOR_PULSE_MAX;
            }

            SetMotorPWM(motor_pulse);
            last_pid_time = HAL_GetTick();
        }

        // 6. Log data (10Hz)
        if(HAL_GetTick() - last_log_time >= LOGGING_INTERVAL_MS) {
            snprintf(log_buffer, sizeof(log_buffer),
                     "%.3f,%.3f,%.3f,%.2f,%.2f,%.6f,%.6f,%.1f",
                     sensors.imu.accel[0], sensors.imu.accel[1], sensors.imu.accel[2],
                     sensors.baro.altitude, sensors.baro.pressure,
                     sensors.gps.latitude, sensors.gps.longitude, sensors.gps.altitude);
            SD_WriteData(SD_LOG_FILENAME, log_buffer);
            last_log_time = HAL_GetTick();
        }

        // 7. Update LED status (5Hz)
        if(HAL_GetTick() - last_status_time >= STATUS_UPDATE_MS) {
            LED_ShowSystemStatus();
            LED_ShowGPSStatus();
            last_status_time = HAL_GetTick();
        }
    }
}

void FC_LoadConfig(void) {
    // Try to load from SD card first
    if(SD_ReadConfig(&config, sizeof(config))) {
        // Update PID controllers with loaded values
        PID_SetTunings(&roll_pid, config.roll_pid_Kp, config.roll_pid_Ki, config.roll_pid_Kd);
        PID_SetTunings(&pitch_pid, config.pitch_pid_Kp, config.pitch_pid_Ki, config.pitch_pid_Kd);
        PID_SetTunings(&yaw_pid, config.yaw_pid_Kp, config.yaw_pid_Ki, config.yaw_pid_Kd);
        PID_SetTunings(&alt_pid, config.alt_pid_Kp, config.alt_pid_Ki, config.alt_pid_Kd);
    } else {
        // Set default values
        config.roll_pid_Kp = 0.8f; config.roll_pid_Ki = 0.05f; config.roll_pid_Kd = 0.2f;
        config.pitch_pid_Kp = 0.8f; config.pitch_pid_Ki = 0.05f; config.pitch_pid_Kd = 0.2f;
        config.yaw_pid_Kp = 0.5f; config.yaw_pid_Ki = 0.01f; config.yaw_pid_Kd = 0.1f;
        config.alt_pid_Kp = 1.2f; config.alt_pid_Ki = 0.1f; config.alt_pid_Kd = 0.5f;
        config.sea_level_pressure = DEFAULT_SEA_LEVEL_PRESSURE;
        config.home_latitude = 0;
        config.home_longitude = 0;
        config.config_valid = 1;
    }
}

void FC_SaveConfig(void) {
    // Save to both SD card and flash
    SD_WriteConfig(&config, sizeof(config));
    W25Q128_WriteData(FLASH_CONFIG_ADDR, (uint8_t*)&config, sizeof(config));
}

void FC_LogGPSData(void) {
    snprintf(log_buffer, sizeof(log_buffer),
             "%.6f,%.6f,%.1f,%.1f,%.1f,%d,%d",
             sensors.gps.latitude, sensors.gps.longitude,
             sensors.gps.altitude, sensors.gps.speed,
             sensors.gps.course, sensors.gps.satellites,
             sensors.gps.fix_quality);
    SD_WriteData(SD_GPS_LOG_FILENAME, log_buffer);
}

void FC_SetHomePosition(void) {
    if(GPS_HAS_FIX()) {
        config.home_latitude = sensors.gps.latitude;
        config.home_longitude = sensors.gps.longitude;
        FC_SaveConfig();
    }
}

void FC_UpdateNavigation(void) {
    if(GPS_HAS_FIX()) {
        // Calculate distance and bearing to home
        navigation.distance_to_target = GPS_DistanceBetween(
            sensors.gps.latitude, sensors.gps.longitude,
            config.home_latitude, config.home_longitude);

        navigation.bearing_to_target = GPS_BearingTo(
            sensors.gps.latitude, sensors.gps.longitude,
            config.home_latitude, config.home_longitude);
    }
}

void FC_ReturnToHome(void) {
    if(!GPS_HAS_FIX()) {
        // Can't return home without GPS fix
        return;
    }

    // Simple implementation - just face home and reduce distance
    if(navigation.distance_to_target > 10.0f) {
        // Face home
        float yaw_error = navigation.bearing_to_target - sensors.imu.yaw;
        // Normalize angle
        while(yaw_error > 180) yaw_error -= 360;
        while(yaw_error < -180) yaw_error += 360;

        // Add yaw correction to PID controller
        PID_SetSetpoint(&yaw_pid, yaw_error);

        // Move forward (simplified)
        if(navigation.distance_to_target > 20.0f) {
            navigation.target_altitude = DEFAULT_TARGET_ALTITUDE;
        } else {
            // We're close, start descending
            navigation.target_altitude = sensors.gps.altitude - 1.0f;
            if(navigation.target_altitude < 2.0f) {
                navigation.target_altitude = 2.0f;
                flight_mode = FLIGHT_MODE_POS_HOLD;
            }
        }
    } else {
        // We've arrived at home position
        flight_mode = FLIGHT_MODE_POS_HOLD;
    }
}

void FC_ArmMotors(void) {
    // Safety checks before arming
    if(sensors.imu.calibrated &&
      (flight_mode != FLIGHT_MODE_RETURN_HOME || GPS_HAS_FIX())) {
        MOTORS_ARM();
        system_status = FC_ARMED;
    }
}

void FC_DisarmMotors(void) {
    MOTORS_DISARM();
    system_status = FC_DISARMED;
    SetAllMotors(&htim1, SAFE_MOTOR_PULSE_MIN);
}

void FC_EmergencyStop(void) {
    FC_DisarmMotors();
    system_status = FC_ERROR;
    LED_Blink(LED_RED, 100);
}

void LED_ShowSystemStatus(void) {
    static uint8_t blink_state = 0;

    switch(system_status) {
        case FC_ARMED:
            LED_SetColor(LED_GREEN);
            break;
        case FC_DISARMED:
            if(blink_state ^= 1) {
                LED_SetColor(LED_BLUE);
            } else {
                LED_SetColor(LED_OFF);
            }
            break;
        case FC_ERROR:
            LED_Blink(LED_RED, 200);
            break;
    }
}

void LED_ShowGPSStatus(void) {
    // Use blue LED to show GPS status
    if(GPS_HAS_FIX()) {
        LED_SetBlue(100); // Solid blue when good fix
    } else if(sensors.gps.satellites > 0) {
        LED_Blink(LED_BLUE, 500); // Slow blink when acquiring
    } else {
        LED_SetBlue(0); // Off when no GPS
    }
}
