#include <Wire.h>
#include "motorControl.h"
#include "constants.h"

namespace MotorControl
{
    arduino::MbedI2C right_I2C(RIGHT_AS5600_SDA_PIN, RIGHT_AS5600_SCL_PIN);

    MagneticSensorI2C leftSensor = MagneticSensorI2C(AS5600_I2C);
    BLDCMotor leftMotor = BLDCMotor(11);
    BLDCDriver3PWM leftDriver = BLDCDriver3PWM(LEFT_MOTOR_A_PIN, LEFT_MOTOR_B_PIN, LEFT_MOTOR_C_PIN);

    MagneticSensorI2C rightSensor = MagneticSensorI2C(AS5600_I2C);
    BLDCMotor rightMotor = BLDCMotor(11);
    BLDCDriver3PWM rightDriver = BLDCDriver3PWM(RIGHT_MOTOR_A_PIN, RIGHT_MOTOR_B_PIN, RIGHT_MOTOR_C_PIN);

    rtos::Mutex target_mutex;
    float target = 0.0f;
    bool motorEnabled = true;

    // ======================================================================
    // Setup
    // ======================================================================

    void setup()
    {
        Wire.begin();
        right_I2C.begin();

        leftSensor.init(&Wire);
        leftMotor.linkSensor(&leftSensor);
        leftDriver.init();
        leftMotor.linkDriver(&leftDriver);

        rightSensor.init(&right_I2C);
        rightMotor.linkSensor(&rightSensor);
        rightDriver.init();
        rightMotor.linkDriver(&rightDriver);

        // Motor Configs =================================================
        leftDriver.voltage_power_supply = 16;
        leftDriver.voltage_limit = 12;

        rightDriver.voltage_power_supply = 16;
        rightDriver.voltage_limit = 12;

        // filtering
        leftMotor.LPF_velocity.Tf = 0.05f;
        rightMotor.LPF_velocity.Tf = 0.05f;

        // PID configs =================================================
        leftMotor.P_angle.P = rightMotor.P_angle.P = ANGLE_KP;
        leftMotor.P_angle.I = rightMotor.P_angle.I = ANGLE_KI;
        leftMotor.P_angle.D = rightMotor.P_angle.D = ANGLE_KD;
        leftMotor.PID_velocity.P = rightMotor.PID_velocity.P = VELOCITY_KP;
        leftMotor.PID_velocity.I = rightMotor.PID_velocity.I = VELOCITY_KI;
        leftMotor.PID_velocity.D = rightMotor.PID_velocity.D = VELOCITY_KD;

        leftMotor.controller = MotionControlType::angle;
        rightMotor.controller = MotionControlType::angle;

        leftMotor.voltage_limit = 12;
        rightMotor.voltage_limit = 12;
        leftMotor.velocity_limit = 50;
        rightMotor.velocity_limit = 50;
        // =================================================================

        leftMotor.init();
        leftMotor.initFOC();
        rightMotor.init();
        rightMotor.initFOC();

        leftMotor.enable();
        rightMotor.enable();

        target = 0;

        Serial.println("Motor setup complete");
    }

    // ======================================================================
    // RTOS task
    // ======================================================================

    void task_MotorFOC()
    {
        while (true)
        {
            leftSensor.update();
            rightSensor.update();

            target_mutex.lock();
            float t = target;
            target_mutex.unlock();

            leftMotor.loopFOC();
            rightMotor.loopFOC();
            if (motorEnabled)
            {
                leftMotor.move(t);
                rightMotor.move(t);
            }

            rtos::ThisThread::sleep_for(std::chrono::milliseconds((int)(1000.0f / MOTOR_CONTROL_LOOP_RATE_HZ))); // ~1 kHz
        }
    }
}
