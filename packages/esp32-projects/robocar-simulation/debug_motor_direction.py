#!/usr/bin/env python3
"""
Debug script to analyze motor direction issues
"""

import sys

sys.path.append("src")

from robot_model import DifferentialDriveRobot
import numpy as np


def analyze_motor_direction():
    """Analyze why motors are spinning in wrong direction"""

    # Create robot instance
    robot = DifferentialDriveRobot("config/robot_config.yaml")

    # Set positive motor commands
    robot.set_motor_commands(100, 100)

    print("Motor Direction Analysis")
    print("=" * 50)
    print("PWM: 100, 100 (should be forward)")
    print("Voltage per motor: {:.2f}V".format(robot.pwm_to_voltage(100)))
    print()

    print("Step | Voltage | BackEMF | Current | Torque | RPM | Angular_Vel")
    print("-" * 60)

    for i in range(10):
        robot.update()

        motor = robot.motor_left
        voltage = robot.pwm_to_voltage(100)
        back_emf = motor.back_emf_constant * motor.angular_velocity

        print(
            f"{i:4d} | {voltage:.2f}V   | {back_emf:.4f}V | {motor.current:.3f}A | {motor.torque:.4f} | {robot.state.motor_left_rpm:.1f} | {motor.angular_velocity:.3f}"
        )

        # Check the motor update calculation
        if i == 0:
            print(f"\nDetailed motor analysis for step {i}:")
            print(f"  Applied voltage: {voltage:.3f}V")
            print(f"  Back EMF: {back_emf:.6f}V")
            print(f"  Resistance: {motor.resistance:.3f}Ω")
            print(f"  Inductance: {motor.inductance:.6f}H")
            print(f"  Voltage drop: {voltage - back_emf - motor.resistance * motor.current:.6f}V")
            print(
                f"  Current derivative: {(voltage - back_emf - motor.resistance * motor.current) / motor.inductance:.6f}A/s"
            )
            print(f"  Load torque: 0.01 N⋅m")
            print(f"  Torque constant: {motor.torque_constant:.6f} N⋅m/A")
            print()


if __name__ == "__main__":
    analyze_motor_direction()
