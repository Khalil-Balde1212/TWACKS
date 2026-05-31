#include "fullstate.h"

#include <Arduino.h>
#include <mbed.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoEigen.h>
#include "motorControl.h"
namespace FullState
{

    float leftTheta = 0.0f, d_leftTheta = 0.0f;
    float rightTheta = 0.0f, d_rightTheta = 0.0f;

    float lin_acc_x = 0.0f, lin_acc_y = 0.0f, lin_acc_z = 0.0f;
    float ang_vel_x = 0.0f, ang_vel_y = 0.0f, ang_vel_z = 0.0f;

    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;

    static char tx_buf[FULLSTATE_SERIAL_TX_BUF_SIZE];
    static volatile int tx_in = 0;
    static volatile int tx_out = 0;
    static rtos::Mutex tx_mutex;

    void serialPrintln(const char *msg)
    {
        tx_mutex.lock();
        for (int i = 0; msg[i] != '\0'; i++)
        {
            int next = (tx_in + 1) % FULLSTATE_SERIAL_TX_BUF_SIZE;
            if (next != tx_out)
            {
                tx_buf[tx_in] = msg[i];
                tx_in = next;
            }
        }
        for (const char *nl = "\r\n"; *nl; nl++)
        {
            int next = (tx_in + 1) % FULLSTATE_SERIAL_TX_BUF_SIZE;
            if (next != tx_out)
            {
                tx_buf[tx_in] = *nl;
                tx_in = next;
            }
        }
        tx_mutex.unlock();
    }

    void drainSerial()
    {
        if (!Serial)
        {
            return;
        }

        int head = tx_in;
        while (tx_out != head)
        {
            Serial.write(tx_buf[tx_out]);
            tx_out = (tx_out + 1) % FULLSTATE_SERIAL_TX_BUF_SIZE;
        }
    }

    void setup()
    {
        if (!IMU.begin())
        {
            Serial.println("Failed to initialize IMU!");
            while (1)
                ;
        }

        IMU.readAcceleration(lin_acc_x, lin_acc_y, lin_acc_z);
        IMU.readGyroscope(ang_vel_x, ang_vel_y, ang_vel_z);

        // Initial orientation
        float magnitude = sqrt(lin_acc_x * lin_acc_x + lin_acc_y * lin_acc_y + lin_acc_z * lin_acc_z);
        yaw = 0.0f;
        roll = asin(-lin_acc_x / magnitude);
        pitch = atan2(lin_acc_y, lin_acc_z);
    }

    void task_UpdateState()
    {

        while (true)
        {
            IMU.readAcceleration(lin_acc_x, lin_acc_y, lin_acc_z);
            IMU.readGyroscope(ang_vel_x, ang_vel_y, ang_vel_z);

            roll = asin(-lin_acc_x / GRAVITY_MAGNITUDE);
            pitch = atan2(lin_acc_y, lin_acc_z);
            yaw += ang_vel_z * (FULLSTATE_UPDATE_INTERVAL_MS / 1000.0f);
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(FULLSTATE_UPDATE_INTERVAL_MS));
        }
    }

    void task_PrintState()
    {
        while (true)
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "Yaw: %.3f Pitch: %.3f Roll: %.3f", yaw, pitch, roll);
            serialPrintln(buf);
            rtos::ThisThread::sleep_for(std::chrono::milliseconds(FULLSTATE_PRINT_INTERVAL_MS));
        }
    }
}