#include <Arduino.h>
#include <Suny.h>

Suny Rob;

void setup()
{
  Rob.Init();
 
}
void loop()
{ 
 Serial.println("port1:"+(String) Rob.getLinePCF(1,1) + ":"  +(String) Rob.getLinePCF(1,2));
 Serial.println("port8:"+(String) Rob.getLinePCF(8,1) + ":"  +(String) Rob.getLinePCF(8,2));
 delay(100);
}
