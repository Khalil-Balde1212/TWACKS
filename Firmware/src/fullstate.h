#ifndef FULLSTATE_H
#define FULLSTATE_H

#define FULLSTATE_UPDATE_INTERVAL_MS 2 // 500hz
#define FULLSTATE_PRINT_INTERVAL_MS 200
#define FULLSTATE_SERIAL_TX_BUF_SIZE 512

#include <ArduinoEigen.h>

namespace FullState
{
    extern float leftTheta, d_leftTheta;
    extern float rightTheta, d_rightTheta;

    const float GRAVITY_MAGNITUDE = 9.81f;

    extern float lin_acc_x, lin_acc_y, lin_acc_z;
    extern float ang_vel_x, ang_vel_y, ang_vel_z;

    extern float yaw, pitch, roll;
    extern float d_yaw, d_pitch, d_roll;

    void setup();
    void task_UpdateState();
    void task_PrintState();
    void serialPrintln(const char *msg);
    void drainSerial();
    void reset();

}

#endif