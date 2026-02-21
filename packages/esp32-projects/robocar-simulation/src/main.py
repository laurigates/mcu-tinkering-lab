#!/usr/bin/env python3
"""
ESP32 Robot Car Simulation - Main Entry Point

This is the main entry point for the ESP32 robot car simulation using
Robotics Toolbox for Python with Swift visualizer.
"""

import asyncio
import argparse
import os
import sys
import matplotlib.pyplot as plt
from pathlib import Path
import signal
import time
import threading

# Add src directory to path
sys.path.append(str(Path(__file__).parent))

from robot_model import DifferentialDriveRobot
from communication_bridge import ESP32CommunicationBridge
from genesis_visualizer import GenesisSimulation, SwiftSimulation, HAS_GENESIS


class SimulationManager:
    """Manages the complete simulation environment"""

    def __init__(
        self,
        config_path: str,
        enable_visualizer: bool = True,
        enable_bridge: bool = True,
        serial_port: str = None,
        viz_mode: str = "headless",
    ):
        self.config_path = config_path
        self.enable_visualizer = enable_visualizer
        self.enable_bridge = enable_bridge
        self.serial_port = serial_port
        self.viz_mode = viz_mode

        # Components
        self.robot = DifferentialDriveRobot(config_path)
        self.bridge = None
        self.visualizer = None
        self.swift_sim = None

        # Control
        self.running = False
        self.bridge_task = None

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\nReceived signal {signum}, shutting down...")
        self.stop()

    def initialize(self):
        """Initialize all simulation components"""
        print("Initializing ESP32 Robot Car Simulation...")

        # Initialize communication bridge
        if self.enable_bridge:
            self.bridge = ESP32CommunicationBridge(self.config_path, self.robot)
            print("✓ Communication bridge initialized")

        # Initialize Genesis visualizer
        if self.enable_visualizer:
            if HAS_GENESIS:
                # Initialize Genesis simulation - much simpler than Swift
                try:
                    self.swift_sim = GenesisSimulation(self.config_path, viz_mode=self.viz_mode)
                    print(f"✓ Genesis visualizer initialized (mode: {self.viz_mode})")
                except Exception as e:
                    print(f"⚠ Genesis visualizer failed to initialize: {e}")
                    print("⚠ Falling back to headless mode")
                    try:
                        self.swift_sim = GenesisSimulation(self.config_path, viz_mode="headless")
                        print("✓ Genesis visualizer initialized (headless fallback)")
                    except Exception as e2:
                        print(f"⚠ Headless fallback also failed: {e2}")
                        print("⚠ Continuing without visualization")
                        self.enable_visualizer = False
            else:
                # Genesis not available, but we can still use matplotlib fallback
                print("ℹ Using matplotlib fallback for visualization")
                try:
                    self.swift_sim = GenesisSimulation(self.config_path, viz_mode=self.viz_mode)
                    print(f"✓ Matplotlib visualizer initialized (mode: {self.viz_mode})")
                except Exception as e:
                    print(f"⚠ Matplotlib fallback failed: {e}")
                    print("⚠ Continuing without visualization")
                    self.enable_visualizer = False

        print("Initialization complete!")

    async def start(self) -> None:
        """Start the simulation"""
        print("\nStarting simulation components...")
        self.running = True

        tasks = []

        # Start communication bridge
        if self.bridge:
            bridge_task = asyncio.create_task(self.bridge.start(self.serial_port))
            tasks.append(bridge_task)
            print("✓ Communication bridge started")

        # Start Genesis visualization - handle GUI on main thread
        if self.swift_sim:
            # Check if using matplotlib fallback
            if (
                hasattr(self.swift_sim, "visualizer")
                and hasattr(self.swift_sim.visualizer, "using_fallback")
                and self.swift_sim.visualizer.using_fallback
            ):
                # For matplotlib, start visualization on main thread
                if self.swift_sim.visualizer.fallback_viz.enabled:
                    print("Starting matplotlib visualization on main thread...")
                    self.swift_sim.visualizer.fallback_viz.start_update_loop()
                    print("✓ Matplotlib visualization started on main thread")
                # Run simulation logic in background thread
                try:
                    simulation_thread = threading.Thread(
                        target=self._run_background_simulation, daemon=True
                    )
                    simulation_thread.start()
                    print("✓ Background simulation started")
                except RuntimeError as e:
                    print(f"⚠ Failed to start background simulation thread: {e}")
            else:
                # For Genesis, run in separate thread since it's not async
                try:
                    genesis_thread = threading.Thread(
                        target=self._run_genesis_simulation, daemon=True
                    )
                    genesis_thread.start()
                    print("✓ Genesis visualization started")
                except RuntimeError as e:
                    print(f"⚠ Failed to start Genesis thread: {e}")

        # Start demo movement if no bridge
        if not self.bridge:
            demo_task = asyncio.create_task(self._demo_movement())
            tasks.append(demo_task)
            print("✓ Demo movement started")

        # Handle different execution modes
        if (
            self.swift_sim
            and hasattr(self.swift_sim, "visualizer")
            and hasattr(self.swift_sim.visualizer, "using_fallback")
            and self.swift_sim.visualizer.using_fallback
        ):
            # For matplotlib mode, keep main thread alive for GUI
            if self.swift_sim.visualizer.fallback_viz.enabled:
                print("Matplotlib GUI mode: keeping main thread active...")
                try:
                    # Keep the main thread alive for matplotlib GUI
                    while self.running:
                        plt.pause(0.1)  # Process GUI events
                        if tasks:
                            # Check if any tasks are done
                            done = [task for task in tasks if task.done()]
                            if done:
                                break
                except KeyboardInterrupt:
                    print("\nInterrupted by user")
            else:
                # Headless matplotlib mode, run async tasks
                if tasks:
                    try:
                        await asyncio.gather(*tasks)
                    except asyncio.CancelledError:
                        pass
        else:
            # Genesis mode or no visualization, run async tasks
            if tasks:
                try:
                    await asyncio.gather(*tasks)
                except asyncio.CancelledError:
                    pass

    def _run_genesis_simulation(self):
        """Run Genesis simulation in separate thread"""
        try:
            # Set some default commands for demo
            if not self.bridge:
                self.robot.set_motor_commands(100, 80)  # Slight turn

            # Run for extended duration
            self.swift_sim.run_simulation(duration=300.0)  # 5 minutes
        except Exception as e:
            print(f"Genesis simulation error: {e}")

    def _run_background_simulation(self):
        """Run simulation logic in background thread (for matplotlib GUI mode)"""
        try:
            print("Starting background simulation logic...")

            # Set some default commands for demo
            if not self.bridge:
                self.robot.set_motor_commands(100, 80)  # Slight turn

            # Simple update loop for robot state
            start_time = time.time()
            last_update = start_time
            duration = 300.0  # 5 minutes

            while self.running and (time.time() - start_time) < duration:
                current_time = time.time()
                dt = current_time - last_update

                # Update robot simulation
                if dt > 0.01:  # 100Hz max update rate
                    self.robot.update(dt)
                    last_update = current_time

                time.sleep(0.01)  # Small sleep to prevent busy waiting

        except Exception as e:
            print(f"Background simulation error: {e}")

    async def _demo_movement(self) -> None:
        """Demo movement pattern when no ESP32 is connected"""
        print("Running demo movement pattern...")

        movement_patterns = [
            (100, 100, 3.0),  # Forward
            (100, 50, 2.0),  # Turn right
            (100, 100, 2.0),  # Forward
            (50, 100, 2.0),  # Turn left
            (100, 100, 3.0),  # Forward
            (0, 0, 1.0),  # Stop
        ]

        # For demo mode, run for a limited time (30 seconds total)
        demo_start_time = time.time()
        demo_duration = 30.0  # Run demo for 30 seconds

        cycle_count = 0
        max_cycles = 3  # Run the pattern a few times

        while (
            self.running
            and (time.time() - demo_start_time) < demo_duration
            and cycle_count < max_cycles
        ):
            print(f"Demo cycle {cycle_count + 1}/{max_cycles}")

            for left_pwm, right_pwm, duration in movement_patterns:
                if not self.running or (time.time() - demo_start_time) >= demo_duration:
                    break

                print(f"Setting motors: left={left_pwm}, right={right_pwm}")
                self.robot.set_motor_commands(left_pwm, right_pwm)

                await asyncio.sleep(duration)

            cycle_count += 1
            if cycle_count < max_cycles and self.running:
                await asyncio.sleep(1.0)  # Brief pause between cycles

        # Stop the robot and end demo
        print("Demo complete! Stopping motors...")
        self.robot.set_motor_commands(0, 0)
        self.stop()  # Stop the simulation

    def stop(self):
        """Stop all simulation components"""
        print("\nStopping simulation...")
        self.running = False

        if self.bridge:
            self.bridge.stop()

        if self.swift_sim:
            try:
                if hasattr(self.swift_sim, "visualizer") and self.swift_sim.visualizer:
                    self.swift_sim.visualizer.stop()
            except Exception as e:
                print(f"Note: Error stopping visualizer: {e}")

        print("Simulation stopped")

    def print_status(self):
        """Print current simulation status"""
        state = self.robot.get_state()
        print(f"\nRobot Status:")
        print(f"Position: ({state.x:.3f}, {state.y:.3f}) m")
        print(f"Orientation: {state.theta:.3f} rad ({state.theta * 180 / 3.14159:.1f}°)")
        print(f"Velocity: {state.v:.3f} m/s, {state.omega:.3f} rad/s")
        print(f"Motors: L={state.motor_left_pwm}, R={state.motor_right_pwm}")
        print(f"Ultrasonic: {state.ultrasonic_distance:.3f} m")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="ESP32 Robot Car Simulation with Genesis Visualizer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with default headless mode
  python main.py
  
  # Run with GUI window visualization
  python main.py --visual
  
  # Run with browser-based visualization  
  python main.py --browser
  
  # Run with specific visualization mode
  python main.py --viz-mode visual
  
  # Run with serial connection to ESP32
  python main.py --serial /dev/ttyUSB0 --visual
  
  # Run completely headless (no visualization)
  python main.py --headless
  
  # Use environment variable for visualization mode
  SWIFT_VIZ_MODE=browser python main.py
  
  # Run with custom configuration
  python main.py --config ../config/custom_config.yaml --visual
        """,
    )

    # Default config path relative to this script's location
    default_config = Path(__file__).parent.parent / "config" / "robot_config.yaml"

    parser.add_argument(
        "--config", "-c", default=str(default_config), help="Configuration file path"
    )

    parser.add_argument(
        "--serial", "-s", help="Serial port for ESP32 connection (e.g., /dev/ttyUSB0, COM3)"
    )

    parser.add_argument(
        "--no-viz", "--headless", action="store_true", help="Run without Genesis visualization"
    )

    parser.add_argument(
        "--visual", "--gui", action="store_true", help="Run with visual window (GUI mode)"
    )

    parser.add_argument(
        "--browser", action="store_true", help="Run with browser-based visualization"
    )

    parser.add_argument(
        "--viz-mode",
        choices=["headless", "visual", "browser"],
        help="Visualization mode: headless (no window), visual (GUI window), or browser (web-based)",
    )

    parser.add_argument("--no-bridge", action="store_true", help="Run without communication bridge")

    parser.add_argument(
        "--demo-only", action="store_true", help="Run demo movement only (no communication)"
    )

    args = parser.parse_args()

    # Determine visualization mode from args and environment
    viz_mode = "headless"  # Default

    # Check environment variable first
    env_viz_mode = os.getenv("GENESIS_VIZ_MODE", "").lower()
    if env_viz_mode in ["headless", "visual", "browser"]:
        viz_mode = env_viz_mode

    # Command line arguments override environment
    if args.viz_mode:
        viz_mode = args.viz_mode
    elif args.visual:
        viz_mode = "visual"
    elif args.browser:
        viz_mode = "browser"
    elif args.no_viz:
        viz_mode = "headless"

    # Validate configuration file
    config_path = Path(args.config)
    if not config_path.exists():
        print(f"Error: Configuration file not found: {config_path}")
        sys.exit(1)

    # Print banner
    print("=" * 60)
    print("ESP32 Robot Car Simulation with Genesis Framework")
    print("=" * 60)
    print(f"Configuration: {config_path}")
    print(f"Visualization: {viz_mode.title()} Mode" if viz_mode != "headless" else "Disabled")
    print(f"Communication: {'Enabled' if not args.no_bridge else 'Disabled'}")
    if args.serial:
        print(f"Serial Port: {args.serial}")
    print("=" * 60)

    # Create simulation manager
    sim_manager = SimulationManager(
        config_path=str(config_path),
        enable_visualizer=viz_mode != "headless",
        enable_bridge=not args.no_bridge and not args.demo_only,
        serial_port=args.serial,
        viz_mode=viz_mode,
    )

    try:
        # Initialize
        sim_manager.initialize()

        # Print instructions
        print("\nSimulation Controls:")
        print("- Ctrl+C: Stop simulation")
        if not args.no_bridge:
            print("- Connect ESP32 via serial or WebSocket (ws://localhost:8765)")
            print("- Send motor commands via WebSocket JSON:")
            print('  {"type": "motor_command", "payload": {"left_pwm": 100, "right_pwm": 100}}')

        # Print visualization guidance
        if viz_mode != "headless" and HAS_GENESIS:
            print(f"\n3D Visualization ({viz_mode.title()} Mode):")
            if viz_mode == "visual":
                print("- Running with GUI window visualization")
                print("- A window should open showing the 3D robot simulation")
            elif viz_mode == "browser":
                print("- Running with browser-based visualization")
                print("- Access the visualization at: http://localhost:52000")
                print("- The browser should open automatically")
            print("\nVisualization Mode Options:")
            print("- --headless or --no-viz: No visualization")
            print("- --visual or --gui: GUI window visualization")
            print("- --browser: Browser-based visualization")
            print("- --viz-mode [headless|visual|browser]: Explicit mode selection")
            print("- Environment variable: GENESIS_VIZ_MODE=[headless|visual|browser]")

        # Start simulation
        asyncio.run(sim_manager.start())

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        sim_manager.stop()


if __name__ == "__main__":
    main()
