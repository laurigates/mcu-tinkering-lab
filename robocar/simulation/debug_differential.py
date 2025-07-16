#!/usr/bin/env python3
"""
Debug script to analyze differential drive kinematics
"""

import sys
sys.path.append('src')

from robot_model import DifferentialDriveRobot
import numpy as np

def analyze_differential_drive():
    """Analyze why differential drive isn't working"""
    
    # Create robot instance
    robot = DifferentialDriveRobot("config/robot_config.yaml")
    
    # Set motors for turning (left motor slower)
    robot.set_motor_commands(50, 100)
    
    print("Differential Drive Analysis")
    print("=" * 80)
    print("Left PWM: 50, Right PWM: 100")
    print("Left Voltage: {:.2f}V, Right Voltage: {:.2f}V".format(
        robot.pwm_to_voltage(50), robot.pwm_to_voltage(100)))
    print("Track width: {:.3f}m, Wheel radius: {:.3f}m".format(
        robot.track_width, robot.wheel_radius))
    print()
    
    print("Step | L_RPM | R_RPM | L_ωmotor | R_ωmotor | L_vwheel | R_vwheel | v_robot | ω_robot")
    print("-" * 80)
    
    for i in range(20):
        robot.update()
        
        # Motor angular velocities (rad/s)
        omega_left = robot.motor_left.angular_velocity
        omega_right = robot.motor_right.angular_velocity
        
        # Wheel linear velocities (m/s)
        v_left = omega_left * robot.wheel_radius
        v_right = omega_right * robot.wheel_radius
        
        # Robot velocities
        v_robot = robot.state.v
        omega_robot = robot.state.omega
        
        print(f"{i:4d} | {robot.state.motor_left_rpm:5.1f} | {robot.state.motor_right_rpm:5.1f} | "
              f"{omega_left:8.3f} | {omega_right:8.3f} | {v_left:8.3f} | {v_right:8.3f} | "
              f"{v_robot:7.3f} | {omega_robot:8.3f}")
        
        if i == 10:
            print(f"\nStep {i} detailed analysis:")
            print(f"  Left motor: {robot.state.motor_left_rpm:.1f} RPM = {omega_left:.3f} rad/s")
            print(f"  Right motor: {robot.state.motor_right_rpm:.1f} RPM = {omega_right:.3f} rad/s")
            print(f"  Left wheel velocity: {v_left:.6f} m/s")
            print(f"  Right wheel velocity: {v_right:.6f} m/s")
            print(f"  Expected robot linear velocity: {(v_left + v_right)/2:.6f} m/s")
            print(f"  Expected robot angular velocity: {(v_right - v_left)/robot.track_width:.6f} rad/s")
            print(f"  Actual robot linear velocity: {v_robot:.6f} m/s")
            print(f"  Actual robot angular velocity: {omega_robot:.6f} rad/s")
            print()
    
    print(f"\nMotor configuration:")
    print(f"  Max RPM: {robot.motor_left.max_rpm}")
    print(f"  Max angular velocity: {robot.motor_left.max_rpm * 2 * np.pi / 60:.3f} rad/s")
    
    return robot

if __name__ == "__main__":
    robot = analyze_differential_drive()
    print("\nDifferential drive analysis complete!")