#include <Arduino.h>
#include <mbed.h>
#include "motorControl.h"
rtos::Thread motorThread(osPriorityRealtime);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  MotorControl::setup();
  motorThread.start(MotorControl::task_MotorFOC);
}

void loop()
{
  rtos::ThisThread::yield();
}
