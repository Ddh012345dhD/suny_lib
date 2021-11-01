#include <Arduino.h>
#include <Suny.h>

Suny Rob;

void setup()
{
  Rob.Init();
 
}
int count=0;
void loop()
{ 
  if(Rob.getIRPCF(1))
  {
    count=count+1;
    if(count>7) count =0;
     delay(1000);
  }
  Rob.setIRPCFLed(1,3,count);
  //Rob.setIRPCFLed(1,2,count);
 
}
