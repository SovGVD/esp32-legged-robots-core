#ifndef CONFIG_H
#define CONFIG_H

/**
 * Configuration file
 */

//Fail safe
#define FS_TIMEOUT_MS 500  // stop when no move package arrives for this long (the web client sends one every 50 ms)

// Serial
#define SERIAL_BAUD 115200 

/*
 * Supported I2C devices
 *  - PWM PCA9685
 *  - IMU MPU 9250
 * Other devices
 *  - ESP32CAM
 */

//#define ESP32CAMERA
//#define ESP32CAMERA_FRAMESIZE    FRAMESIZE_QVGA
//#define ESP32CAMERA_JPEG_QUALITY 12
//#define ESP32CAMERA_FPS          25

//#define PWM_CONTROLLER_ADDRESS 0x40
//#define PWM_CONTROLLER_TYPE    PCA9685
//#define PWM_CONTROLLER_TYPE    ESP32PWM

//#define IMU_ADDRESS            0x68
//#define IMU_TYPE               MPU9250

//#define POWER_SENSOR           INA219

#define LEG_TRIM_INC   0.002   // radian
#define LEG_TRIM_LIMIT  0.24   // See settingsUint8ToDouble()

// Servo config (set as max as possible for init, but use servoProfile)
#define SERVO_MIN 600
#define SERVO_MAX 2400
#define SERVO_FREQ 330	// for PCA9685

#endif
