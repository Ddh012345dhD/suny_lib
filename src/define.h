#ifndef define_H
#define define_H
//==============================Sensor adrress define =============================================
#define PCF8574_addr 0x38
#define TCA9548_addr 0x70
#define PCA9685_addr 0x40
#define RGB_PIN 32
#define RGB_NUM 8
#define Sonar_Sensor_addr 0x10
#define TouchLed_Sensor_addr 0xFE
#define Line_SenSor_addr 0xFD
#define Temperature_Sensor_addr 0xFC
#define DHT11_SenSor_aadr 0xFB
#define MPU6050_sensor_addr 0x68
#define Light_sensor_addr 0x20
#define Tracfic_light_addr 0x22
#define Buzzer_addr 0x20
#define Button_addr 0x20
#define IR_PCF_addr 0x21
#define Touch_PCF_addr 0x21
#define Line_PCF_addr 0x26

//==============================Motor define ======================================================
#define f 1
#define b 0
#define r 2
#define l 3

#define M1 1
#define M2 2
#define M_ALL 3
#define M_L 4
#define M_R 5

#define M_IN1 5
#define M_IN2 6
#define M_IN3 7
#define M_IN4 8
#define PWMA 19
#define PWMB 18

#define encoderM1CHA 12
#define encoderM1CHB 13
#define encoderM2CHA 12
#define encoderM2CHB 13
#define encoderPPR  135

#define defaultM 0
#define timeM 1
#define rotationM 2
const int freq = 100000;
const int ledChannel1 = 0;
const int ledChannel2 = 0;
const int resolution = 8;
#define AIN1 P0
#define AIN2 P1
#define BIN1 P2
#define BIN2 P3
const float Ki=0.1;
const float Kp=0.1;
const float Kd=0.1;

//======================================Servo define ==========================================
#define SERVOMIN  125 
#define SERVOMAX  575

//======================================Sensor cmd=============================================
#define cmd_DHT_temp 0
#define cmd_DHT_hum 1
#define cmd_light_di 0
#define cmd_light_ai 1
//======================================Tracfic cmd========================================
#define cmd_tracfic_R  B00000011
#define cmd_tracfic_Y  B00000101
#define cmd_tracfic_G  B00000110
#define cmd_tracfic_default B00000111


//======================================cmd sensor Max30102==============================
#define cmd_SPO2 1
#define cmd_heartRate 2
//======================================Cmd sensor Gryro==================================
#define cmd_temp       1
#define cmd_accX       2
#define cmd_accY       3
#define cmd_accZ       4
#define cmd_GryroX     5
#define cmd_GryroY     6
#define cmd_GryroZ     7
#define cmd_AccAngleX  8
#define cmd_AccAngleY  9
#define cmd_GyroAngleX 10
#define cmd_GyroAngleY 11
#define cmd_GyroAngleZ 12
#define cmd_AngleX     13
#define cmd_AngleY     14
#define cmd_AngleZ     15
//===========================================cmd sensor DHT12============================
#define cmd_temp 1
#define cmd_hum 2
//============================================ DISPLAY====================================
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//===========================================cmd Button======================================
#define button1 1
#define button2 2
#define buttonLed1 1
#define buttonLed2 2
#define buttonLedALL 3
//============================================cmd Line PCF=====================================
#define lineRight 1
#define lineLeft  2

#endif