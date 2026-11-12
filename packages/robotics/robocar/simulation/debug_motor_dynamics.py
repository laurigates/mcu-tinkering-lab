#!/usr/bin/env python3
"""
Debug script to analyze motor dynamics and energy transitions
"""

import sys

sys.path.append("src")

import numpy as np

from robot_model import DifferentialDriveRobot


def analyze_motor_dynamics():
    """Analyze the motor dynamics during the energy drop"""

    # Create robot instance
    robot = DifferentialDriveRobot("config/robot_config.yaml")

    # Set constant motor commands
    robot.set_motor_commands(100, 100)

    print("Motor Dynamics Analysis")
    print("=" * 60)
    print(
        "Step | KE     | RE     | Total  | v     | ω     | I_left | I_right | Torque_L | Torque_R"
    )
    print("-" * 60)

    # Run simulation and analyze critical steps
    for i in range(25):
        robot.update()

        # Calculate energies
        kinetic_energy = 0.5 * robot.mass * robot.state.v**2
        rotational_energy = 0.5 * robot.inertia * robot.state.omega**2
        total_energy = kinetic_energy + rotational_energy

        # Motor state
        current_left = robot.motor_left.current
        current_right = robot.motor_right.current
        torque_left = robot.motor_left.torque
        torque_right = robot.motor_right.torque

        print(
            f"{i:4d} | {kinetic_energy:.4f} | {rotational_energy:.4f} | {total_energy:.4f} | "
            f"{robot.state.v:.3f} | {robot.state.omega:.3f} | {current_left:.3f} | {current_right:.3f} | "
            f"{torque_left:.4f} | {torque_right:.4f}"
        )

    print("\nMotor friction analysis:")
    motor = robot.motor_left
    print(f"Static friction: {motor.friction_static:.6f} N⋅m")
    print(f"Kinetic friction: {motor.friction_kinetic:.6f} N⋅m")
    print(f"Viscous friction: {motor.friction_viscous:.6f} N⋅m⋅s/rad")
    print("Motor rotor inertia: 0.001 kg⋅m²")
    print("Load torque: 0.01 N⋅m")
    print(f"Max angular velocity: {motor.max_rpm * 2 * np.pi / 60:.2f} rad/s")

    # Calculate voltage and power
    voltage = robot.pwm_to_voltage(100)
    print(f"\nVoltage per motor: {voltage:.2f}V")
    print(f"Back EMF constant: {motor.back_emf_constant:.6f} V⋅s/rad")
    print(f"Torque constant: {motor.torque_constant:.6f} N⋅m/A")

    return robot


if __name__ == "__main__":
    robot = analyze_motor_dynamics()
    print("\nMotor dynamics analysis complete!")
