# Autonomous Robot Control System

This project implements a PD-based control system for autonomous navigation using sensor feedback.

## Features
- Straight corridor navigation
- Zig-zag corridor handling
- Dynamic obstacle detection and avoidance

## Hardware Used
- STM32 Nucleo Board
- VL53L0X ToF Sensor
- Ultrasonic Sensors (Left & Right)
- Motor driver and steering servo

## Control Strategy
The system uses a Proportional-Derivative (PD) controller:

u(t) = Kp e(t) + Kd (de(t)/dt)

Where error is calculated as:
error = left distance - right distance


