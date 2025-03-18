/*

arquivo 

*/
#include <Arduino.h>
#define Heartbeat_PIN 13                 // monitoração de "batimentos" para o watchdog de hardware

void heartBeat(void);

void heartBeat() {
  //pinMode(Heartbeat_PIN,OUTPUT);
  digitalWrite(Heartbeat_PIN, LOW);
  delay(15);
  digitalWrite(Heartbeat_PIN, HIGH);
  delay(15);
  //pinMode(Heartbeat_PIN,INPUT);
  //Serial.println("HB OK");
  //Serial.print("Data/hora do sistema:  ");
  //Serial.println(RTC_Time);
}