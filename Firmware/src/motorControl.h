#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <mbed.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#define MOTOR_CONTROL_LOOP_INTERVAL_MS 1 // 1000hz

#define ANGLE_KP 10.0
#define ANGLE_KI 0.0
#define ANGLE_KD 0.0

#define VELOCITY_KP 0.5
#define VELOCITY_KI 10.0
#define VELOCITY_KD 0.0

namespace MotorControl
{
    extern arduino::MbedI2C right_I2C;

    extern BLDCMotor leftMotor;
    extern BLDCDriver3PWM leftDriver;
    extern MagneticSensorI2C leftSensor;

    extern BLDCMotor rightMotor;
    extern BLDCDriver3PWM rightDriver;
    extern MagneticSensorI2C rightSensor;

    // ======================================================================
    // Setup
    // ======================================================================
    void setup();

    // ======================================================================
    // RTOS task — runs the FOC loop
    // ======================================================================
    void task_MotorFOC();
}

#endif
