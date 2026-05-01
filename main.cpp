/*
 TEST_MODE:
 1 = Straight corridor with static obstacles
 2 = Zig-zag corridor with varying widths
 3 = Dynamic corridor with moving obstacle

 Control approach:
* PD controller implemented for steering correction
* Ki term removed to avoid integral windup
* Threshold-based obstacle detection added
* Finite State-machine (FSM) used to prioritise safety over navigation 
*/

#include "mbed.h"
#include "VL53L0X.h"
#include "SRF05.h"

// SELECT TEST MODE
#define TEST_MODE 1   // Change to 1, 2, or 3 test mode

// HARDWARE PIN CONFIGURATION
I2C i2c(PB_9, PB_8);    // ToF sensor: SDA = D14, SCL = D15
SRF05 srf_Left(PA_10, PB_3);   // Left ultrasonic: Trig = D2, Echo = D3
SRF05 srf_Right(PA_8, PA_9);   // Right ultrasonic: Trig = D7, Echo = D8

PwmOut Steering(PB_0);         // Steering servo
PwmOut Motor_FWD(PB_10);       // Motor forward signal
PwmOut Motor_REV(PB_4);        // Motor reverse signal

VL53L0X tofSensor(&i2c);
Serial usb(USBTX, USBRX, 9600);

// CONTROL THRESHOLDS
const float FRONT_STOP_MM = 250.0f;          // Static obstacle stop distance
const float FRONT_DYNAMIC_STOP_MM = 350.0f;  // Earlier stop for moving obstacle
const float DYNAMIC_RATE_LIMIT_MM = 80.0f;   // Sudden approach detection
const float MAX_VALID_DISTANCE_MM = 1200.0f; // Prevent extreme noisy values

// Servo pulse width limits
const int SERVO_CENTRE_US = 1500;
const int SERVO_MIN_US = 1000;
const int SERVO_MAX_US = 2000;

// Motor speed settings
const int MOTOR_FULL_MS = 20;
const int MOTOR_HALF_MS = 10;
const int MOTOR_SLOW_MS = 7;

// PD / PID-BASED STEERING PARAMETERS
float Kp = 0.70f;
float Ki = 0.00f;   // kept zero to avoid integral windup
float Kd = 0.35f;

// PID memory variables
float errorNow = 0.0f;
float errorPrevious = 0.0f;
float errorSum = 0.0f;

// Limit steering correction added to 1500 us
const float PID_OUTPUT_LIMIT_US = 400.0f;

// SENSOR VARIABLES
float leftDistanceMM = 0.0f;
float rightDistanceMM = 0.0f;
float tofFilteredMM = 1000.0f;
float previousToFMM = 1000.0f;

// BASIC UTILITY FUNCTIONS
float clampFloat(float value, float minValue, float maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}
int clampInt(int value, int minValue, int maxValue)
{
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

// MOTOR CONTROL FUNCTIONS
void stopMotor()
{
    Motor_FWD.pulsewidth_ms(0);
    Motor_REV.pulsewidth_ms(0);
}
void fullForward()
{
    Motor_REV.pulsewidth_ms(0);
    Motor_FWD.pulsewidth_ms(MOTOR_FULL_MS);
}
void halfForward()
{
    Motor_REV.pulsewidth_ms(0);
    Motor_FWD.pulsewidth_ms(MOTOR_HALF_MS);
}
void slowForward()
{
    Motor_REV.pulsewidth_ms(0);
    Motor_FWD.pulsewidth_ms(MOTOR_SLOW_MS);
}
void slowReverse()
{
    Motor_FWD.pulsewidth_ms(0);
    Motor_REV.pulsewidth_ms(MOTOR_SLOW_MS);
}

// STEERING CONTROL FUNCTIONS

void setSteeringUS(int pulseWidthUS)
{
    pulseWidthUS = clampInt(pulseWidthUS, SERVO_MIN_US, SERVO_MAX_US);
    Steering.pulsewidth_us(pulseWidthUS);
}
void setStraight()
{
    setSteeringUS(SERVO_CENTRE_US);
}
void setHalfLeft()
{
    setSteeringUS(1750);
}
void setFullLeft()
{
    setSteeringUS(2000);
}
void setHalfRight()
{
    setSteeringUS(1250);
}
void setFullRight()
{
    setSteeringUS(1000);
}

// PID / PD CONTROL FUNCTIONS

void resetControllerMemory()
{
    errorNow = 0.0f;
    errorPrevious = 0.0f;
    errorSum = 0.0f;
}
float computePIDOutput()
{
    errorNow = leftDistanceMM - rightDistanceMM;

    float pOut = Kp * errorNow;

    errorSum += errorNow;
    errorSum = clampFloat(errorSum, -PID_OUTPUT_LIMIT_US, PID_OUTPUT_LIMIT_US);
    float iOut = Ki * errorSum;

    float dOut = Kd * (errorNow - errorPrevious);

    float output = pOut + iOut + dOut;
    output = clampFloat(output, -PID_OUTPUT_LIMIT_US, PID_OUTPUT_LIMIT_US);

    errorPrevious = errorNow;

    return output;
}
void applyPIDSteering()
{
    float pidOutputUS = computePIDOutput();

    int servoCommandUS = SERVO_CENTRE_US + (int)pidOutputUS;
    servoCommandUS = clampInt(servoCommandUS, SERVO_MIN_US, SERVO_MAX_US);

    setSteeringUS(servoCommandUS);

    printf("PID: %.1f us : Servo: %d us  ", pidOutputUS, servoCommandUS);
}

// SENSOR ACQUISITION

void readSensors()
{
    float rawToF = tofSensor.getRangeMillimeters();

    if (rawToF <= 0 || rawToF > MAX_VALID_DISTANCE_MM) {
        rawToF = tofFilteredMM;
    }

    tofFilteredMM = (0.2f * tofFilteredMM) + (0.8f * rawToF);

    leftDistanceMM = 10.0f * srf_Left.read();
    rightDistanceMM = 10.0f * srf_Right.read();

    if (leftDistanceMM <= 0 || leftDistanceMM > MAX_VALID_DISTANCE_MM) {
        leftDistanceMM = MAX_VALID_DISTANCE_MM;
    }

    if (rightDistanceMM <= 0 || rightDistanceMM > MAX_VALID_DISTANCE_MM) {
        rightDistanceMM = MAX_VALID_DISTANCE_MM;
    }
}

// MODE 1: STRAIGHT CORRIDOR WITH STATIC OBSTACLES 
void modeStraightCorridor()
{
    if (tofFilteredMM < FRONT_STOP_MM) {
        slowReverse();
        resetControllerMemory();

        if (leftDistanceMM > rightDistanceMM) {
            setHalfLeft();
            printf("MODE 1 - Front obstacle - Reverse + left escape");
        } else {
            setHalfRight();
            printf("MODE 1 - Front obstacle - Reverse + right escape");
        }
    }
    else {
        fullForward();
        applyPIDSteering();
        printf("MODE 1 - Straight corridor - PD wall following");
    }
}

// MODE 2: ZIG-ZAG CORRIDOR WITH VARYING WIDTHS
void modeZigZagCorridor()
{
    if (tofFilteredMM < FRONT_STOP_MM) {
        slowReverse();
        resetControllerMemory();

        if (leftDistanceMM > rightDistanceMM) {
            setFullLeft();
            printf("MODE 2 - Front obstacle - Reverse + full left");
        } else {
            setFullRight();
            printf("MODE 2 - Front obstacle - Reverse + full right");
        }
    }
    else {
        halfForward();
        applyPIDSteering();
        printf("MODE 2 - Zig-zag corridor - PD damping active");
    }
}

// MODE 3: DYNAMIC CORRIDOR WITH MOVING OBSTACLE

void modeDynamicObstacle()
{
    float approachRateMM = previousToFMM - tofFilteredMM;

    if (tofFilteredMM < FRONT_DYNAMIC_STOP_MM || approachRateMM > DYNAMIC_RATE_LIMIT_MM) {
        stopMotor();
        setStraight();
        resetControllerMemory();

        printf("MODE 3 - Dynamic obstacle detected - STOP - Rate: %.1f mm", approachRateMM);

        wait(0.4);

        if (leftDistanceMM > rightDistanceMM) {
            slowForward();
            setHalfLeft();
            printf("Resume left");
        } else {
            slowForward();
            setHalfRight();
            printf("Resume right");
        }
    }
    else {
        halfForward();
        applyPIDSteering();
        printf("MODE 3 - Dynamic corridor - Safe PD navigation");
    }

    previousToFMM = tofFilteredMM;
}

// MAIN PROGRAM

int main()
{
    wait(2);

    Steering.period_ms(20);
    Motor_FWD.period_ms(20);
    Motor_REV.period_ms(20);

    stopMotor();
    setStraight();

    tofSensor.init();
    tofSensor.setModeContinuous();
    tofSensor.startContinuous();

    tofFilteredMM = 1000.0f;
    previousToFMM = 1000.0f;
    resetControllerMemory();

    printf("ENGR7023 Autonomous Corridor Navigation\n");
    printf("PD/PID-Based Real-Time Steering Control\n");
    printf("Selected TEST_MODE = %d\n", TEST_MODE);

    while (1) {

        readSensors();

        printf("ToF: %.0f mm | Left: %.0f mm | Right: %.0f mm | Error: %.0f mm | ",
               tofFilteredMM,
               leftDistanceMM,
               rightDistanceMM,
               leftDistanceMM - rightDistanceMM);

        if (TEST_MODE == 1) {
            modeStraightCorridor();
        }
        else if (TEST_MODE == 2) {
            modeZigZagCorridor();
        }
        else if (TEST_MODE == 3) {
            modeDynamicObstacle();
        }
        else {
            stopMotor();
            setStraight();
            printf("INVALID TEST MODE");
        }

        printf("\n");

        wait(0.1);
    }
}
