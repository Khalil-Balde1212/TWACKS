#include <Arduino.h>
#include <mbed.h>
#include "motorControl.h"
#include "fullstate.h"
rtos::Thread motorThread(osPriorityRealtime);
rtos::Thread stateThread(osPriorityHigh);
rtos::Thread printThread(osPriorityLow);

void setup()
{
  Serial.begin(115200);

  FullState::setup();
  stateThread.start(FullState::task_UpdateState);
  printThread.start(FullState::task_PrintState);

  MotorControl::setup();
  motorThread.start(MotorControl::task_MotorFOC);
}

void loop()
{
  FullState::drainSerial();
  rtos::ThisThread::yield();
}
