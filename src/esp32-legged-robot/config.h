/**
 * Configuration file
 */

//Cores loop time
#define LOOP_TIME 4000   // microseconds (10^-6 seconds)
#define SERVICE_LOOP_TIME 100000
#define SERVICE_FAST_LOOP_TIME 5000

//Fail safe
#define FS_WS_THR 20  // cycles, 1 second = FS_WS_THR*LOOP_TIME, e.g. 1 000 * 1 000 = 1 000 000 = 1 second

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
#define SERVO_FREQ 330

