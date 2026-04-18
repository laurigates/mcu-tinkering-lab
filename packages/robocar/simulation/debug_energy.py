#!/usr/bin/env python3
"""
Debug script to analyze energy conservation in robot simulation
"""

import sys

sys.path.append("src")


from robot_model import DifferentialDriveRobot


def analyze_energy_conservation():
    """Analyze energy conservation in the robot simulation"""

    # Create robot instance
    robot = DifferentialDriveRobot("config/robot_config.yaml")

    # Set constant motor commands
    robot.set_motor_commands(100, 100)

    # Data storage
    times = []
    kinetic_energies = []
    rotational_energies = []
    total_energies = []
    velocities = []
    angular_velocities = []
    motor_powers = []

    # Run simulation
    for i in range(100):
        robot.update()

        # Calculate energies
        kinetic_energy = 0.5 * robot.mass * robot.state.v**2
        rotational_energy = 0.5 * robot.inertia * robot.state.omega**2
        total_energy = kinetic_energy + rotational_energy

        # Calculate motor power input
        voltage_left = robot.pwm_to_voltage(robot.state.motor_left_pwm)
        voltage_right = robot.pwm_to_voltage(robot.state.motor_right_pwm)
        power_left = voltage_left * robot.motor_left.current
        power_right = voltage_right * robot.motor_right.current
        total_power = power_left + power_right

        # Store data
        times.append(i * robot.dt)
        kinetic_energies.append(kinetic_energy)
        rotational_energies.append(rotational_energy)
        total_energies.append(total_energy)
        velocities.append(robot.state.v)
        angular_velocities.append(robot.state.omega)
        motor_powers.append(total_power)

        # Print key points
        if i % 10 == 0:
            print(
                f"Step {i}: KE={kinetic_energy:.4f}J, RE={rotational_energy:.4f}J, "
                f"Total={total_energy:.4f}J, v={robot.state.v:.3f}m/s, "
                f"ω={robot.state.omega:.3f}rad/s, P={total_power:.3f}W"
            )

    # Find where energy starts decreasing
    for i in range(15, len(total_energies)):
        if total_energies[i] < total_energies[i - 1] * 0.9:
            print(f"\nEnergy dropped significantly at step {i}")
            print(f"Previous energy: {total_energies[i - 1]:.6f}J")
            print(f"Current energy: {total_energies[i]:.6f}J")
            print(
                f"Drop: {((total_energies[i - 1] - total_energies[i]) / total_energies[i - 1] * 100):.1f}%"
            )
            break

    # Check if energy reaches equilibrium
    final_energy = total_energies[-1]
    equilibrium_reached = abs(total_energies[-1] - total_energies[-10]) < 0.001
    print(f"\nFinal energy: {final_energy:.6f}J")
    print(f"Equilibrium reached: {equilibrium_reached}")

    # Print motor friction analysis
    print("\nMotor friction analysis:")
    print(f"Static friction: {robot.motor_left.friction_static:.6f} N⋅m")
    print(f"Kinetic friction: {robot.motor_left.friction_kinetic:.6f} N⋅m")
    print(f"Viscous friction: {robot.motor_left.friction_viscous:.6f} N⋅m⋅s/rad")
    print("Load torque: 0.01 N⋅m")
    print(f"Motor torque constant: {robot.motor_left.torque_constant:.6f} N⋅m/A")

    return (
        times,
        kinetic_energies,
        rotational_energies,
        total_energies,
        velocities,
        angular_velocities,
        motor_powers,
    )


if __name__ == "__main__":
    print("Energy Conservation Analysis")
    print("=" * 50)

    times, ke, re, te, v, omega, power = analyze_energy_conservation()

    print("\nAnalysis complete!")
