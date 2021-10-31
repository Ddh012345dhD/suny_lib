#include <Suny.h>
//-----------------------DEFINE------------------------------
Adafruit_NeoPixel Px(RGB_NUM,RGB_PIN,NEO_GRB + NEO_KHZ800);
PCF8574 _pinO(PCF8574_addr);
PCA9685 Pwm = PCA9685(PCA9685_addr);
MAX30102 Suny_Max30102;
MPU6050 gryro(Wire);
DHT12  dht12;
SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
hw_timer_t * timerM = NULL;
hw_timer_t * timerS = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int flagMotor;
volatile bool flagSensor=false;

//--------------------------BRGIN-----------------------------
void IRAM_ATTR onTimerM()
{
   portENTER_CRITICAL_ISR(&timerMux);
   flagMotor++;
   portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR onTimerS()
{
   flagSensor =true;
}
void Suny::Init()
{ 
   Serial.begin(115200);
   Wire.begin();
  _pinO.begin();
  Pwm.begin();
  Pwm.setPWMFreq(60);  
  Pwm.setOscillatorFrequency(27000000);
 // Pwm.setPWMFreq(1600); 
  Wire.setClock(100000);
  Px.begin();
  Px.setBrightness(255);
  this->MotorEncoder_Init();
 
//==================================== 
 
//=====================================
  timerM = timerBegin(0, 80, true);
  timerAttachInterrupt(timerM, &onTimerM, true);
  timerAlarmWrite(timerM, 10000, true);
  timerAlarmEnable(timerM);
  flagMotor =0;
  //===========================================
  timerS = timerBegin(1, 80, true);
  timerAttachInterrupt(timerS, &onTimerS, true);
  timerAlarmWrite(timerS, 10000, true);
  timerAlarmEnable(timerS);
  // flagSensor =false;
   //===================OTA====================
 
   
}

//------------------------SENSOR-------------------------------
void Suny::SetLed(int a,String color)
{
 
  if(color == "R")    {Array_Color[0]=255,Array_Color[1]=0;Array_Color[2]=0;}
  else if(color=="G") {Array_Color[0]=0,Array_Color[1]=255;Array_Color[2]=0;}
  else if(color=="B") {Array_Color[0]=0,Array_Color[1]=0;Array_Color[2]=255;}
  else if(color == "Y") {Array_Color[0]=255,Array_Color[1]=255;Array_Color[2]=0;}
  else if(color == "W") {Array_Color[0]=255,Array_Color[1]=255;Array_Color[2]=255;}
  else if(color == "P") {Array_Color[0]=255,Array_Color[1]=0;Array_Color[2]=255;}
  else if(color == "Cy") {Array_Color[0]=0,Array_Color[1]=255;Array_Color[2]=255;}
  else if(color == "Or") {Array_Color[0]=255,Array_Color[1]=160;Array_Color[2]=16;}
  else if(color == "V") {Array_Color[0]=125,Array_Color[1]=0;Array_Color[2]=255;}
 Px.setPixelColor(a,Px.Color(Array_Color[0],Array_Color[1],Array_Color[2]));
 Px.show();
}
void Suny::SetAllLed(String color)
{
  for(int i=0 ;i<RGB_NUM;i++)
   {
     this->SetLed(i,color);
   }
}
void Suny::TurnOFF(int num)
{
 Array_Color[0] =0;
 Array_Color[1] =0;
 Array_Color[2] =0;
 Px.setPixelColor(num,Px.Color(Array_Color[0],Array_Color[1],Array_Color[2]));
 Px.show();
}
void Suny::TurnOFFALL(void)
{
  for(int i =0;i<RGB_NUM;i++)
  {
     this->TurnOFF(i);
  }
}
int Suny::ConvertPortSensor(int port)
{
  
  int porrtt;
  if(port>8) return -1;
  if(port ==1) porrtt =3;
  else if(port ==2) porrtt=2;
  else if(port==3) porrtt=1;
  else if(port==4) porrtt=0;
  else if(port==5) porrtt =4;
  else if(port==6) porrtt =5;
  else if(port==7) porrtt =6;
  else if(port==8) porrtt =7;
 return porrtt;
}
// Selectport INPUT
int Suny::SelectPortI(int port)
{
  int portt = this->ConvertPortSensor(port);
  Wire.beginTransmission(0x70);
  Wire.write(1 << portt);
  Wire.endTransmission();
 
}
void Suny::SendCmd(byte addr,uint8_t port,byte data)
{
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
 // this->SelectPortI(port);
}
//PinMode Output

//Selectport OUTPUT
void Suny::pinModeOUTPUT()
{
  _pinO.pinMode(P0,OUTPUT);
  _pinO.pinMode(P1,OUTPUT);
  _pinO.pinMode(P2,OUTPUT);
  _pinO.pinMode(P3,OUTPUT);
  _pinO.pinMode(P4,OUTPUT);
  _pinO.pinMode(P5,OUTPUT);
  _pinO.pinMode(P6,OUTPUT);
  _pinO.pinMode(P7,OUTPUT);
}
int Suny::SelectPortO_Digi(int port)
{  
      if(port==1) return num_out =  P0;
   else if(port==2) return num_out = P1;
   else if(port ==3) return num_out = P2;
   else if(port ==4) return num_out = P3;
   else if(port ==5) return num_out =  P4;
   else if(port ==6) return num_out = P5;
   else if(port ==7) return num_out= P6;
   else if(port ==8) return num_out =P7;
   return num_out;
}

int Suny::SelectPortO_PWM(int port)
{
        if(port==1) return num =  8;
   else if(port==2) return num = 9;
   else if(port ==3) return num =10;
   else if(port ==4) return num = 11;
   else if(port ==5) return num =  12;
   else if(port ==6) return num = 13;
   else if(port ==7) return num= 14;
   else if(port ==8) return num =15;
   return num;
}
 
// Read data from Port 
byte Suny::ReadValue(byte addr,int port)
{
     volatile byte buff ;
     this->SelectPortI(port);
      Wire.requestFrom(addr, (uint8_t)1);
     if(Wire.available()>0)
       {
          buff = (uint8_t)Wire.read();  
         // delay(1);   
       }     
   return buff;
}

int Suny::getSonarValue(int port)
{
  return Sorna_val = this ->ReadValue(Sonar_Sensor_addr,port);
}
int Suny::getTouchLedValue(int port)
{
 return Touch_val = this ->ReadValue(TouchLed_Sensor_addr,port);

}
int Suny::getLineValue(int port)
{
 return Line_val = this->ReadValue(Line_SenSor_addr,port);
}
int Suny::getDHT11Value(int port,int mode)
{ 
   if(mode == cmd_DHT_temp)
   {
     SendCmd(DHT11_SenSor_aadr,this->SelectPortI(port),cmd_DHT_temp);
     DHT_Temp = this ->ReadValue(DHT11_SenSor_aadr,port);
     return DHT_data = DHT_Temp;
   }
   else if(mode==cmd_DHT_hum)
   {
       SendCmd(DHT11_SenSor_aadr,this->SelectPortI(port),cmd_DHT_hum);
       DHT_Hum = this ->ReadValue(DHT11_SenSor_aadr,port);
     return DHT_data = DHT_Hum;
   }
  return DHT_data;
}
int Suny::getTempertatureValue(int port)
{
 return Temp_val = this->ReadValue(Temperature_Sensor_addr,port);
}
void Suny::SetTouchLed(int port,uint8_t data)
{
   
   this->SendCmd(TouchLed_Sensor_addr,this->SelectPortI(port),data);
}
int Suny::getLightValue(int port,int mode)
{
   if(mode == cmd_light_di)
   {
     this->SendCmd(TouchLed_Sensor_addr,this->SelectPortI(port),cmd_light_di);
    Light_Di_val = this->ReadValue(Light_sensor_addr,port);
    return Light_val = Light_Di_val;
   }
   else if(mode == cmd_light_ai)
   {
      this->SendCmd(TouchLed_Sensor_addr,this->SelectPortI(port),cmd_light_ai);
     Light_Ai_val = this->ReadValue(Light_sensor_addr,port);
      return Light_val = Light_Ai_val;
   }
  return Light_val;
}
void Suny::SetTracficLigt(int port,String light,String status)
{
  // int pt = this->SelectPortI(port);
    
    if(light == "R" && status=="ON") this->SendCmd(Tracfic_light_addr,this->SelectPortI(port),cmd_tracfic_R);
    else  if(light == "R"&& status =="NONE") this->SendCmd(Tracfic_light_addr,this->SelectPortI(port),cmd_tracfic_default);
    else if(light == "Y"&& status=="ON") this->SendCmd(Tracfic_light_addr,this->SelectPortI(port),cmd_tracfic_Y);
    else  if(light == "Y"&& status =="NONE") this->SendCmd(Tracfic_light_addr,this->SelectPortI(port),cmd_tracfic_default);
    else if(light == "G"&& status=="ON") this->SendCmd(Tracfic_light_addr,this->SelectPortI(port),cmd_tracfic_G);
    else  if(light == "G"&& status =="NONE") this->SendCmd(Tracfic_light_addr,this->SelectPortI(port),cmd_tracfic_default); 
}
bool Suny::getTouchLedPCF(int port)
{
   //uint8_t tempTouchLedPCF = this->ReadValue(Touch_PCF_addr,port) ;
   return Touch_led_val = bitRead(this->ReadValue(Touch_PCF_addr,port),0);
}
void Suny::SetTouchLedPCF(int port,String Color)
{
  // this->SelectPortI(port);
   for(int i =0;i<= 8;i++)
   {
     if(Color == Cmd_Color_Touch_PCF[i]) 
     {
     this->SendCmd(Touch_PCF_addr,this->SelectPortI(port),Cmd_Set_Color_Touch_PCF[i]);
     }
   }
}
bool Suny::getButtonVal(int port,int button)
{
 // uint8_t temp_button = this->ReadValue(Button_addr,port);
  if(button == button1) return Button_data[0]= bitRead(this->ReadValue(Button_addr,port),0);  
  else if(button == button2)  return Button_data[1]= bitRead(this->ReadValue(Button_addr,port),1);  
  else return -1;
}
void Suny::SetButtonLed(int port,int led,String Color)
{
  //this->SelectPortI(port);
  //==================================================
  if(led == buttonLed1)
   {
   for(int i =0;i<= 8;i++)
   {
     if(Color == Cmd_Color_Button_LED[i]) 
     {
     this->SendCmd(Touch_PCF_addr,this->SelectPortI(port),Cmd_Set_Color_Button_LED[i]);
     }
   }
   }
   //==================================================
    if(led == buttonLed2)
   {
   for(int i =0;i<= 8;i++)
   {
     if(Color == Cmd_Color_Button_LED[i]) 
     {
     this->SendCmd(Touch_PCF_addr,this->SelectPortI(port),Cmd_Set_Color_Button_LED[i]);
     }
   }
   }
   //==================================================
    if(led == buttonLedALL)
   {
   for(int i =0;i<= 8;i++)
  {
     if(Color == Cmd_Color_Button_LED[i]) 
     {
     this->SendCmd(Touch_PCF_addr,this->SelectPortI(port),Cmd_Set_Color_Button_LED[i]);
     }
   }
    }
}
bool Suny::getLinePCF(int port,int line )
{
  if(this->ReadValue(Line_PCF_addr,port) <252) return -1;
  else
  {
    if(line==lineRight) return bitRead(ReadValue(Line_PCF_addr,port),0)?HIGH:LOW;
    else if(line==lineLeft) return bitRead(ReadValue(Line_PCF_addr,port),1)?HIGH:LOW;
    else return -1;
  }
}
bool Suny::getIRPCF(int port)
{
  if(this->ReadValue(IR_PCF_addr,port) <254) return -1;
  else return IRPCF_data = bitRead(this->ReadValue(IR_PCF_addr,port),0)?HIGH:LOW;
}
void Suny::Suny_Max30102_Init()
{
  //  Suny_Max30102.begin();
   while (!Suny_Max30102.begin()) {
    //Serial.println("MAX30102 was not found");
    delay(100);
  }
   Suny_Max30102.sensorConfiguration(/*ledBrightness=*/0x1F, /*sampleAverage=*/SAMPLEAVG_4, \
                                  /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_400, \
                                  /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_4096);
}
int32_t Suny::getMax30102(int port,int mode)
{
   this->SelectPortI(port);
   
   if(mode==cmd_SPO2) 
   {
     Suny_Max30102.heartrateAndOxygenSaturation(&SPO2,&SPO2Valid,&heartRate,&heartRateValid );
     //Serial.println(SPO2);
   } 
   else if(mode==cmd_heartRate) return heartRate; 
   else return 0;
}
void Suny::Suny_Gryro_Init()
{
  gryro.begin();
  gryro.calcGyroOffsets(true);
}
int Suny::getGryro(int port,int mode)
{ 
   this->SelectPortI(port);
   gryro.update();
   if(mode ==cmd_temp) return MPU_temp = gryro.getTemp();
   else if(mode ==cmd_accX) return AccX = gryro.getAccX();
   else if(mode==cmd_accY) return AccY=gryro.getAccY();
   else if(mode==cmd_accZ)  return AccZ=gryro.getAccZ();
   else if(mode==cmd_GryroX) return GryroX = gryro.getGyroX();
   else if(mode==cmd_GryroY) return GryroY = gryro.getGyroY();
   else if(mode==cmd_GryroZ) return GryroZ = gryro.getGyroZ();
   else if(mode==cmd_AccAngleX) return AccAngleX = gryro.getAccAngleX();
   else if(mode==cmd_AccAngleY) return AccAngleY = gryro.getAccAngleY();
   else if(mode==cmd_GyroAngleX) return GryroAngleX = gryro.getGyroAngleX();
   else if(mode==cmd_GyroAngleY) return GryroAngleY = gryro.getGyroAngleY();
   else if(mode==cmd_GyroAngleZ) return GryroAngleZ = gryro.getGyroAngleZ();
   else if(mode==cmd_AngleX) return AngleX = gryro.getAngleX();
   else if(mode==cmd_AngleY) return AngleY = gryro.getAngleY();
   else if(mode==cmd_AngleZ) return AngleZ = gryro.getAngleZ();
   else return 0;
}
void Suny::Suny_DHT12_Init()
{
   dht12.begin();
}
int Suny::getDHT12(int port,int mode)
{
    this->SelectPortI(port);
    if(mode==cmd_temp) return DHT_Temp=dht12.readTemperature();
    else if(mode == cmd_hum) return DHT_hum= dht12.readHumidity();
    else return 0;
   if(isnan(DHT_Temp) || isnan(DHT_hum)) return -1;
}
//---------------------------------MORTOR---------------------------
volatile int32_t curPosA = 0;
volatile int32_t curPosB = 0;
void Suny::MotorEncoder_Init()
{
 ledcSetup(ledChannel1, freq, resolution);
 ledcSetup(ledChannel2, freq, resolution);
 ledcAttachPin(PWMA, ledChannel1);
 ledcAttachPin(PWMB, ledChannel2);
 //====================================
 this->pinModeOUTPUT();
_pinO.pinMode(AIN1,OUTPUT);
_pinO.pinMode(AIN2,OUTPUT);
_pinO.pinMode(BIN1,OUTPUT);
_pinO.pinMode(BIN2,OUTPUT);
  pinMode(encoderM1CHA,INPUT_PULLUP);
  pinMode(encoderM1CHB,INPUT_PULLUP);
  pinMode(encoderM2CHA,INPUT_PULLUP);
  pinMode(encoderM2CHB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderM1CHA), [] {
    if (digitalRead(encoderM1CHB))
      curPosA--;
    else
      curPosA++;
  }, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderM2CHA), [] {
    if (digitalRead(encoderM2CHB))
      curPosB--;
    else
      curPosB++;
  }, RISING);

}
float Suny::partP(float err, float p)
{
  return err * p;
}
float Suny::partD(float err, float d)
{
  static float preErr;
  uint32_t preTime = 0;
  float dErr, ret;
  dErr = err - preErr;
  preErr = err;
  ret = dErr * d / float(millis() - preTime);
  preTime = millis();
  return ret;
}
float Suny::partI(float err, float i)
{
  static float sum;
  float ret;
  static uint32_t preTime = 0;
  if (abs(err) < 20) // các bạn có thể giới hạn khâu I bằng cách này.
    sum += err;
  ret = sum * i * float(millis() - preTime);
  preTime = millis();
//  ret = constrain(ret, -150.0, 150.0); các bạn có thể giới hạn khâu I bằng cách này cũng được.
  
  return ret;
}
float Suny::partPID(int M,int SV_rotation)
{
   if(M==1)
    {
      desPosA =(int32_t) (SV_rotation*encoderPPR);
      errorA = desPosA - curPosA;
      KpidA = this->partP(errorA, Kp) + this->partD(errorA, Kd) + this->partI(errorA, Ki);
      KpidA =abs(KpidA);
      return valuePWM = (int) (constrain(KpidA, 0.0, 255.0));
    }
    else if(M==2)
    {
      desPosB =(int32_t) (SV_rotation*encoderPPR);
      errorB = desPosB - curPosB;
      KpidB = this->partP(errorA, Kp) + this->partD(errorA, Kd) + this->partI(errorA, Ki);
      KpidB =abs(KpidB);
      return valuePWM = (int) (constrain(KpidB, 0.0, 255.0));
    }
    else return -1;
}
//MODE 1: DEFAULT
void Suny::Motor1(int M,int Vspeed,int dir)
{
       if(M==M1 && dir ==f)
      {
         this ->Mfor1();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
      }
      else if(M==M2 && dir==b)
      {
         this ->Mback1();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
      }
      else if(M==M2&& dir==f)
      {
        this->Mfor2();
        ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
      }
      else if(M==M2 && dir==b)
      {
         this ->Mback2();
         ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
      }
      else if (M==M_ALL && dir ==f)
      {
         this ->Mfor1();
         this->Mfor2();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
         ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
      }
      else if (M==M_ALL && dir ==b)
      {
         this ->Mback2();
         this ->Mback1();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
         ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
      }
      else if (M==M_ALL && dir ==r)
      {
         this->Mfor1();
         this ->Mfor2();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
         ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
      }
      else if (M==M_ALL && dir ==l)
      {
         this->Mback1();
         this ->Mback2();
        ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
         ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
      }

     
}
//MODE 2 :TIMER
void Suny::Motor2(int M,int Vspeed,int dir,float time)
{
  int times = (int)(time*1000);
   if(M==M1 && dir ==f)
      {
         this ->Mfor1();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
         delay(times);
         this->Stop1();
         delay(100);
      }
      else if(M==M1 && dir==b)
      {
         this ->Mback1();
         ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
         delay(times);
         this->Stop1();
         delay(100);
      }
      else if(M==M2&& dir==f)
      {
        this->Mfor2();
        ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
        delay(times);
         this->Stop2();
         delay(100);
      }
      else if(M==M2 && dir==b)
      {
         this ->Mback2();
         ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
         delay(times);
         this->Stop2();
         delay(100);
      }
      else if(M==M_ALL &&dir== f)
      {
        this->Mfor1();
        this->Mfor2();
        ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
        delay(times);
         this->Stop2();
         this->Stop1();
         delay(100);
      }
      else if(M==M_ALL &&dir== b)
      {
        this->Mback1();
         this->Mback2();
        ledcWrite(ledChannel1, map(Vspeed,0,100,0,255));
        ledcWrite(ledChannel2, map(Vspeed,0,100,0,255));
        delay(times);
         this->Stop2();
         this->Stop1();
         delay(100);
      }
  
       
}
//MODE 3:ROTATION
void Suny::Mortor3(int M,int dir,float rotation)
{ 
 // int sped;
   if(M == M1 && dir == f)
   {
       this ->Mfor1();
      // sped= this->partPID(M,rotation);
       ledcWrite(ledChannel1, this->partPID(M,rotation));      
   }
   else if(M==M1 && dir ==b)
   {
       this ->Mback1();
      // sped= this->partPID(M,rotation);
       ledcWrite(ledChannel1,this->partPID(M,rotation));
   }
    else if(M==M2&& dir==f)
    {
      this ->Mfor2();
      // sped= this->partPID(M,rotation);
       ledcWrite(ledChannel2, this->partPID(M,rotation)); 
    }
    else if(M==2&& dir==b)
    {
       this ->Mback2();
      // sped= this->partPID(M,rotation);
       ledcWrite(ledChannel2, this->partPID(M,rotation));
    }
}

void Suny::MForward( )
{

  Pwm.setPWM(M_IN1,4096,0);
  Pwm.setPWM(M_IN2,0,4096);
  Pwm.setPWM(M_IN3,4096,0);
  Pwm.setPWM(M_IN4,0,4096);
  
}
void Suny::MBackward()
{
  Pwm.setPWM(M_IN1,0,4096);
  Pwm.setPWM(M_IN2,4096,0);
  Pwm.setPWM(M_IN3,0,4096);
  Pwm.setPWM(M_IN4,4096,0);
}
void Suny::MLeft()
{
 Pwm.setPWM(M_IN1,4096,0);
 Pwm.setPWM(M_IN2,0,4096);
 Pwm.setPWM(M_IN3,0,4096);
 Pwm.setPWM(M_IN4,4096,0);
}
void Suny::MRight()
{
 Pwm.setPWM(M_IN1,0,4096);
 Pwm.setPWM(M_IN2,4096,0);
 Pwm.setPWM(M_IN3,4096,0);
 Pwm.setPWM(M_IN4,0,4096);
}
void Suny::Mfor1()
{
  // Pwm.setPWM(M_IN1,4096,0);
 //  Pwm.setPWM(M_IN2,0,4096);
  _pinO.digitalWrite(AIN1, HIGH);
  _pinO.digitalWrite(AIN2, LOW);
}
void Suny::Mback1()
{
   //  Pwm.setPWM(M_IN1,0,4096);
   //  Pwm.setPWM(M_IN2,4096,0);
    _pinO.digitalWrite(AIN1, LOW);
   _pinO.digitalWrite(AIN2, HIGH);
}
void Suny::Stop1()
{
    _pinO.digitalWrite(AIN1, LOW);
   _pinO.digitalWrite(AIN2, LOW);
}
void Suny::Mfor2()
{
  //  Pwm.setPWM(M_IN3,4096,0);
   // Pwm.setPWM(M_IN4,0,4096);
     _pinO.digitalWrite(BIN1, HIGH);
     _pinO.digitalWrite(BIN2, LOW);
}
void Suny::Mback2()
{
 // Pwm.setPWM(M_IN3,0,4096);
 // Pwm.setPWM(M_IN4,4096,0);
  _pinO.digitalWrite(BIN1, LOW);
  _pinO.digitalWrite(BIN2, HIGH);
}
void Suny::Stop2()
{
   _pinO.digitalWrite(BIN1, LOW);
  _pinO.digitalWrite(BIN2, LOW);
} 
//------------------------SERVO----------------------------------
int Suny::angleToPulse(int ang)
{
  int pulse =(int) (map(ang,0, 180, SERVOMIN,SERVOMAX));
  return pulse;
}
void Suny::Servo(int port,int ServoAng)
{ 
 Pwm.setPWM(this->SelectPortO_PWM(port), 0, this->angleToPulse(ServoAng));
}
//--------------------------------OUTPUT-----------------------------
void Suny::PWM_OUTPUT(int port,int duty)
{
  num_duty = (uint16_t) (map(duty,0,100,0,4096));
 // Serial.println(num_duty);
  Pwm.setPWM(this->SelectPortO_PWM(port),num_duty,0);
}
void Suny::DiG_OUTPUT(int port ,bool level)
{
  _pinO.digitalWrite(this->SelectPortO_Digi(port),level);
}
//===========================================OTA=========================
/*
void Suny::LC_OTAHandle(void)
{
ArduinoOTA.handle();
}
void Suny::LC_OTAInit(const char* wifi,const char* host)
{
   
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(wifi);
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(host);
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR)
        Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)
        Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR)
        Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)
        Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)
        Serial.println("End Failed");
    });
  ArduinoOTA.setTimeout(60000);
  ArduinoOTA.begin();
}

//===========================================END_OTA=========================
//===========================================WIFI============================

void Suny::Wifi_Init(const char* id,const char* pas)
{
  WiFi.begin(id,pas);
  Serial.print("Connect to" +(String)id);
  while ((!(WiFi.status() == WL_CONNECTED)))
  {
  Serial.println(".");
    delay(100);
  }
}
*/
//==========================================DISPLAY==============================
void Suny::Display_Init()
{
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  
}
void Suny::Display_clear()
{
  display.clearDisplay();
}
void Suny::DisPlay_showNumber(int num)
{
 
}
void Suny::Display_showString(String text)
{
   
}
void Suny::Display_setCursor(int16_t w,int16_t h)
{
  display.setCursor(w,h);
}
//===================================