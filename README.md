# Line-Following Robot with PID Control Using VEX Cortex M4

This project implements a line-following robot using the VEX Cortex M4 microcontroller. The robot utilizes three line follower sensors and a PID (Proportional-Integral-Derivative) controller to follow a line with precision. An on/off button is also incorporated to control the robot's movement.

## Table of Contents:

- [Introduction](#introduction)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Sensor Placement](#sensor-placement)
- [Code Explanation](#code-explanation)
- [How to Run](#how-to-run)
- [Result](#result)

## Introduction

Line-following robots are a fundamental project in robotics, teaching the basics of sensor integration and control systems. This project takes it a step further by implementing a PID controller to smooth out the robot's movement and improve its line-following capabilities.

## Hardware Requirements

<ul>
    <li>VEX Cortex M4 Microcontroller</li>
    <li>VEX 393 Motors</li>
    <li>Line Follower Sensors (3 units)</li>
    <li>Bumper Switch (1 unit)</li>
    <li>Connecting Cables and Wires</li>
</ul>

## Software Requirements

<ul>
    <li>ROBOTC for VEX Robotics (compatible with VEX Cortex)</li>
    <li>ROBOTC Firmware installed on the VEX Cortex M4</li>
    <li>Computer with USB port for programming</li>
</ul>

## Sensor Placement

- Mount the three line follower sensors at the front of the robot, evenly spaced, and close to the ground.
- The left and right sensors should be positioned to detect the edges of the line, while the center sensor tracks the middle.

## Code Explanation

### Configuration

```c
#pragma config(Sensor, in1,    lineFollowerRIGHT, sensorLineFollower)
#pragma config(Sensor, in2,    lineFollowerCENTER, sensorLineFollower)
#pragma config(Sensor, in8,    lineFollowerLEFT, sensorReflection)
#pragma config(Sensor, dgtl5,  leftBumper,     sensorTouch)
#pragma config(Motor,  port1,  leftMotor,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10, rightMotor,    tmotorVex393_HBridge, openLoop)
```

<ul>
    <li>Sensors: Defines the ports for the line follower sensors and the bumper switch.</li>
    <li>Motors: Assigns the left and right motors to specific ports</li>
</ul>

### On/Off button

```c
bool motorDisabled = true;
bool bClickInProgress = false;

void processButton() {
    if (bClickInProgress) {
        if (SensorValue[leftBumper])
            bClickInProgress = false;
    } else {
        if (!SensorValue[leftBumper]) {
            motorDisabled = !motorDisabled;
            bClickInProgress = true;
        }
    }
}
```
<ul>
    <li>Toggles the robot's movement when the bumper switch is pressed.</li>
    <li>Prevents multiple toggles from a single press using a debounce mechanism.</li>
</ul>

### PID Controller Variables

```c
int error;
int lastError = 0;
int integral = 0;
int derivative = 0;

int turn;
int nLeftMotor;
int nRightMotor;

int offset = 1381;
int Tp = 70;
int Kp = 30;
int Ki = 7;
int Kd = 45;

int lightValue;
```

<ul>
    <li>PID Variables: Used to calculate the control effort.</li>
    <li>Constants:
    <ul>
        <li>offset: Baseline sensor value when the robot is perfectly aligned.</li>
        <li>Tp: Base motor speed.</li>
        <li>Kp, Ki, Kd: PID coefficients for tuning the controller.</li>
    </ul>
    </li>
</ul>

### Sensor Reading Function

```c
int getLightValue() {
    int light1 = SensorValue[lineFollowerLEFT];
    int light2 = SensorValue[lineFollowerCENTER];
    int light3 = SensorValue[lineFollowerRIGHT];
    int total = light1 + light2 + light3;
    return total / 3;
}
```
<ul>
    <li>Reads the values from the three line follower sensors.</li>
    <li>Calculates the average light value to determine the robot's position relative to the line.</li>
</ul>

##Main Control Loop

```c
task main() {
    wait1Msec(2000);
    while(true) {
        processButton();
        lightValue = getLightValue();
        error = lightValue - offset;
        integral = (2.0 / 3.0) * integral + error;
        derivative = error - lastError;

        turn = (Kp * error + Ki * integral + Kd * derivative) / 1000;

        if (turn > 0) {
            nLeftMotor  = Tp - turn;
            nRightMotor = Tp + turn;
        } else {
            nLeftMotor  = Tp + turn;
            nRightMotor = Tp - turn;
        }

        if (motorDisabled) {
            motor[leftMotor]  = 0;
            motor[rightMotor] = 0;
        } else {
            motor[leftMotor]  = nLeftMotor;
            motor[rightMotor] = nRightMotor;
        }
        lastError = error;
    }
}
```
<ul>
    <li>Process Button: Checks the state of the on/off button.</li>
    <li>PID Calculations:
        <ul>
            <li>Error: Difference between current light value and the offset.</li>
            <li>Integral: Accumulates error over time, mitigates steady-state error.</li>
            <li>Derivative: Predicts future error based on the rate of change.</li>
            <li>Turn: The control effort calculated from the PID formula.</li>
        </ul>
    </li>
    <li>Motor Control:
        <ul>
            <li>Adjusts motor speeds based on the turn value.</li>
            <li>Ensures smooth turning by proportionally increasing or decreasing motor speeds.</li>
        </ul>
    </li>
    <li>Motor Enable/Disable:
        <ul>
            <li>Stops the motors if motorDisabled is true.</li>
        </ul>
    </li>
</ul>

## How to Run
<ol>
    <li>Set Up the Hardware</li>
    <li>Install ROBOTC: Ensure that ROBOTC is installed on your computer and the VEX Cortex firmware is up to date.</li>
    <li>Load the Code:
        <ul>
            <li>Open ROBOTC and create a new project.</li>
            <li>Copy and paste the provided code into the editor.</li>
        </ul>
    </li>
    <li>Compile and Download:
        <ul>
            <li>Compile the code to check for errors.</li>
            <li>Connect the VEX Cortex to your computer via USB.</li>
            <li>Download the program to the VEX Cortex.</li>
        </ul>
    </li>
    <li>Run the Robot:
        <ul>
            <li>Place the robot on a track with a clear line to follow.</li>
            <li>Press the bumper switch to enable the motors.</li>
            <li>The robot should begin following the line autonomously.</li>
        </ul>
    </li>
    <li>Stop the Robot:
        <ul>
            <li>Press the bumper switch again to disable the motors.</li>
        </ul>
    </li>
</ol>

## Note: 

This project is a practical implementation of control systems and robotics principles. It serves as an educational tool for learning about sensors, actuators, and PID control in embedded systems.

## Result:

[![Watch the video](https://i.ytimg.com/an_webp/kapof8Yd0SY/mqdefault_6s.webp?du=3000&sqp=CJbs-rcG&rs=AOn4CLAOr7Qal_qAqNN3M9KzlApnAQQ9Mg)](https://youtu.be/kapof8Yd0SY)
