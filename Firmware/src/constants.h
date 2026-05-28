#ifndef PIN_MAP_H
#define PIN_MAP_H

// Simple FOC Pins
#define LEFT_MOTOR_A_PIN 2
#define LEFT_MOTOR_B_PIN 3
#define LEFT_MOTOR_C_PIN 4

#define RIGHT_MOTOR_A_PIN 5
#define RIGHT_MOTOR_B_PIN 6
#define RIGHT_MOTOR_C_PIN 7

// A4/A5 = GP12/GP13
#define LEFT_AS5600_SDA_PIN 18
#define LEFT_AS5600_SCL_PIN 19

// D13/D11 = GP6/GP7
#define RIGHT_AS5600_SDA_PIN 13
#define RIGHT_AS5600_SCL_PIN 11

// Current Sensor
#define LEFT_CURRENT_SENSOR_PIN A0  // Current sensor analog pin
#define RIGHT_CURRENT_SENSOR_PIN A1 // Current sensor analog pin
#define CURRENT_SENSOR_GAIN 0.5f    // 0.5V/A for the hall sensor

#endif