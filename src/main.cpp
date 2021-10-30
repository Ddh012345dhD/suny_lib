#include <Arduino.h>
#include <Suny.h>

Suny Rob;

void setup()
{
  Rob.Init();
 
}
void loop()
{ 
 Serial.println("port3:"+(String) Rob.getLinePCF(3,1) + ":"  +(String) Rob.getLinePCF(3,2));
 Serial.println("port1:"+(String) Rob.getLinePCF(2,1) + ":"  +(String) Rob.getLinePCF(2,2));
 delay(1000);
}
