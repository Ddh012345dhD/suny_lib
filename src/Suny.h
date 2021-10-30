#ifndef Suny_H
#define Suny_H
#include <lib.h>
#include <define.h>
class Suny
{
  public :
 
   //-----------------Begin-----------------------------------
     void Init(void); 
  //----------------- PORT------------------------------------
     uint8_t SelectPortI(uint8_t port);
    
  //--------------------SENSOR--------------------------------
     void SetLed(int i,String color);
     void SetAllLed(String color);
     void TurnOFF(int num);
     void TurnOFFALL();
     int getSonarValue(int port);
     int getTouchLedValue(int port);
     int getLineValue(int port);
     int getTempertatureValue(int por);
     int getDHT11Value(int port,int mode);
     void SetTouchLed(int port,byte data);
     int getLightValue(int port,int mode);
     void SetTracficLigt(int port,String light,String status);
     bool getTouchLedPCF(int port);
     void SetTouchLedPCF(int port,String color);
     bool getButtonVal(int port,int button);
     void SetButtonLed(int port,int led,String color);
     void Suny_Max30102_Init();
     int32_t getMax30102(int port,int mode);
     void Suny_Gryro_Init();
     int getGryro(int port,int mode);
     void Suny_DHT12_Init();
     int getDHT12(int port,int mode);
     bool getLinePCF(int port,int line);
     bool getIRPCF(int port);

    //--------------------MORTOR-------------------------
    //mode 1 : defaut,mode 2:timer,mode 3:rotation
     void Motor1(int M,int VSpeed,int dir );
     void Motor2(int M,int VSpeed,int dir,float time);
     void Mortor3(int M,int dir,float rotation);
     void MForward();
     void MBackward();
     void MLeft();
     void MRight();
     void MStop();
     //======================Servo=========================
     void Servo(int port,int ServoAng);
     //======================OUTPUT=========================
     void PWM_OUTPUT(int port,int duty);
     void DiG_OUTPUT(int port,bool level);
    //==========================OTA=========================
     
      void LC_OTAHandle(void);
      void LC_OTAInit(const char* name,const char* host);
     //==========================WIFI=============================
      void Wifi_Init(const char* ssidwifi,const char* password);
     //========================DISPLAY================================
     void Display_Init();
     void Display_clear();
     void DisPlay_showNumber(int num);
     void Display_showString(String text);
     void Display_setCursor(int16_t w,int16_t h);
     
     
  private:
  //====================================SENSOR/==============================
    int ReadValue(uint8_t addr,int port);
   
   //========================PORT OUTPUT
     int SelectPortO_Digi(int port);
     int SelectPortO_PWM(int port);
     void SendCmd(byte addr,uint8_t port,byte dt);
     uint16_t num_duty;
   
   //========================MOTOR========================
     void Mfor1();
     void Mback1();
     void Stop1();
     void Mfor2();
     void Mback2();
     void Stop2();
     void MotorEncoder_Init();
     float partP(float err, float p);
     float partD(float err, float d);
     float partI(float err, float i);
     float  partPID(int M,int SV_rotation) ;
      int32_t errorA=0;
      int32_t desPosA=0;
      int32_t errorB=0;
      int32_t desPosB=0;
      float KpidA=0;
      float KpidB=0;
      int valuePWM;
   //========================SERVO==========================
     int angleToPulse(int ang);
     int Servo_num;
     int pulse_servo;
     int num;
    ///-----------------------OUTPUT-------------------
    void pinModeOUTPUT();
    int num_out;
    //-----------------------Convert port sensor---------------------
    uint8_t ConvertPortSensor(uint8_t portt);
    uint8_t porrtt;
    //========================OTA======================
   
  uint8_t Array_Color[3];
  uint8_t Servonum;
  uint8_t ServoAng;
  int  ServoPin[8] = {1,2,3,4,9,10,11,12}; 
  int dataTouchLED[10]= {1,2,3,4,5,6,7,8,9,10};
  byte Cmd_Set_Color_Touch_PCF[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
  String Cmd_Color_Touch_PCF[8]={"Off","R","G","B","Y","V","W","O"};
  
  uint8_t Sorna_val,Touch_val,Line_val,Temp_val,DHT_Temp,DHT_Hum,DHT_data,
   Light_Di_val,Light_Ai_val,Light_val;
     int Port;
     int a=0;
     int c=0;
     unsigned long Start_time_Motor =0;
     bool Touch_led_val;
     int32_t SPO2; 
     int8_t SPO2Valid; 
     int32_t heartRate;
     int8_t heartRateValid; 
     int MPU_temp,AccX,AccY,AccZ,GryroX,GryroY,GryroZ,AccAngleX,AccAngleY,GryroAngleX,GryroAngleY,GryroAngleZ,AngleX,AngleY,AngleZ;
     int DHT_temp,DHT_hum;
     bool Button_data[2]={};
     String Cmd_Color_Button_LED[8]={"Off","R","G","B","Y","V","W","O"};
     byte  Cmd_Set_Color_Button_LED[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
     bool LinePCF_data[2]={};
     bool IRPCF_data;
    
};

#endif