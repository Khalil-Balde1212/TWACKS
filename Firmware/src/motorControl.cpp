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
    // RTOS task
    // ======================================================================

    void task_MotorFOC()
    {

        initLeftMotor();
        // initRightMotor();

        target = 0.0f;
        while (true)
        {
            leftSensor.update();
            leftMotor.loopFOC();
            leftMotor.move(target);

            // rightSensor.update();
            // rightMotor.loopFOC();
            // rightMotor.move(target);

            Serial.println(leftMotor.shaftAngle());

            rtos::ThisThread::sleep_for(std::chrono::milliseconds(MOTOR_CONTROL_LOOP_INTERVAL_MS));
        }
    }

    void initLeftMotor()
    {
        Serial.println("Initializing left motor...");
        Wire.begin();
        leftSensor.init(&Wire);
        leftDriver.voltage_power_supply = 16;
        leftDriver.voltage_limit = 12;

        leftDriver.init();
        // Serial.println("Left driver initialized");

        leftMotor.P_angle.P = ANGLE_KP;
        leftMotor.P_angle.I = ANGLE_KI;
        leftMotor.P_angle.D = ANGLE_KD;
        leftMotor.PID_velocity.P = VELOCITY_KP;
        leftMotor.PID_velocity.I = VELOCITY_KI;
        leftMotor.PID_velocity.D = VELOCITY_KD;

        leftMotor.controller = MotionControlType::velocity;

        leftMotor.voltage_limit = 12;
        leftMotor.velocity_limit = 50;
        leftMotor.LPF_velocity.Tf = 0.2f;

        leftMotor.linkSensor(&leftSensor);
        leftMotor.linkDriver(&leftDriver);
        // Serial.println("Left motor linked to driver and sensor");

        leftMotor.init();
        leftMotor.initFOC();
        leftMotor.enable();
        // Serial.println("Left motor FOC initialized and enabled");
        // Serial.println("Left motor initialized");
    }

    void initRightMotor()
    {
        right_I2C.begin();
        rightSensor.init(&right_I2C);

        rightDriver.voltage_power_supply = 16;
        rightDriver.voltage_limit = 12;

        rightDriver.init();

        rightMotor.P_angle.P = ANGLE_KP;
        rightMotor.P_angle.I = ANGLE_KI;
        rightMotor.P_angle.D = ANGLE_KD;
        rightMotor.PID_velocity.P = VELOCITY_KP;
        rightMotor.PID_velocity.I = VELOCITY_KI;
        rightMotor.PID_velocity.D = VELOCITY_KD;

        rightMotor.controller = MotionControlType::velocity;

        rightMotor.voltage_limit = 12;
        rightMotor.velocity_limit = 50;
        rightMotor.LPF_velocity.Tf = 0.2f;

        rightMotor.linkSensor(&rightSensor);
        rightMotor.linkDriver(&rightDriver);

        rightMotor.init();
        rightMotor.initFOC();
        rightMotor.enable();
    }
}
