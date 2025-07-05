"""
Communication Bridge between ESP32 and Simulation

This module provides the communication interface between the ESP32 robot hardware
and the Python simulation environment, supporting multiple protocols including
WebSocket, MQTT, and UART.
"""

import asyncio
import websockets
import json
import serial
import time
import threading
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, asdict
from enum import Enum
import struct
import crc8
import yaml
from pathlib import Path

from robot_model import RobotState, DifferentialDriveRobot


class MessageType(Enum):
    """I2C Protocol message types matching ESP32 implementation"""
    MOVE = 0x01
    SOUND = 0x02
    SERVO = 0x03
    DISPLAY = 0x04
    STATUS = 0x05
    LED = 0x06
    STOP = 0x07
    SENSOR_DATA = 0x08
    AI_COMMAND = 0x09


@dataclass
class I2CMessage:
    """I2C message structure matching ESP32 protocol"""
    message_type: MessageType
    data: bytes
    checksum: int = 0
    
    def __post_init__(self):
        if self.checksum == 0:
            self.checksum = self._calculate_checksum()
    
    def _calculate_checksum(self) -> int:
        """Calculate CRC8 checksum"""
        hash_obj = crc8.crc8()
        hash_obj.update(bytes([self.message_type.value]))
        hash_obj.update(self.data)
        return hash_obj.digest()[0]
    
    def to_bytes(self) -> bytes:
        """Convert message to bytes for transmission"""
        return bytes([self.message_type.value]) + self.data + bytes([self.checksum])
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'I2CMessage':
        """Create message from bytes"""
        if len(data) < 2:
            raise ValueError("Message too short")
        
        msg_type = MessageType(data[0])
        msg_data = data[1:-1]
        checksum = data[-1]
        
        return cls(msg_type, msg_data, checksum)


class ESP32CommunicationBridge:
    """Communication bridge between ESP32 and simulation"""
    
    def __init__(self, config_path: str, robot: DifferentialDriveRobot):
        self.config = self._load_config(config_path)
        self.robot = robot
        
        # Communication channels
        self.websocket_server = None
        self.serial_connection = None
        self.clients = set()
        
        # Message handlers
        self.message_handlers = {
            MessageType.MOVE: self._handle_move_command,
            MessageType.SERVO: self._handle_servo_command,
            MessageType.SOUND: self._handle_sound_command,
            MessageType.LED: self._handle_led_command,
            MessageType.STOP: self._handle_stop_command,
            MessageType.STATUS: self._handle_status_request,
        }
        
        # State
        self.running = False
        self.last_sensor_update = 0
        self.sensor_update_interval = 0.1  # 10Hz
        
        # Threading
        self.update_thread = None
        self.lock = threading.Lock()
        
    def _load_config(self, config_path: str) -> Dict:
        """Load configuration from YAML file"""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    async def start_websocket_server(self):
        """Start WebSocket server for communication"""
        bridge_config = self.config['simulation']['bridge']
        host = bridge_config['host']
        port = bridge_config['port']
        
        print(f"Starting WebSocket server on {host}:{port}")
        
        async def handle_client(websocket, path):
            """Handle WebSocket client connections"""
            self.clients.add(websocket)
            print(f"Client connected: {websocket.remote_address}")
            
            try:
                async for message in websocket:
                    await self._handle_websocket_message(websocket, message)
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.clients.discard(websocket)
                print(f"Client disconnected: {websocket.remote_address}")
        
        self.websocket_server = await websockets.serve(
            handle_client, host, port, ping_interval=10, ping_timeout=5
        )
        print(f"WebSocket server started on ws://{host}:{port}")
        
    async def _handle_websocket_message(self, websocket, message: str):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            payload = data.get('payload', {})
            
            if msg_type == 'motor_command':
                left_pwm = payload.get('left_pwm', 0)
                right_pwm = payload.get('right_pwm', 0)
                self.robot.set_motor_commands(left_pwm, right_pwm)
                
            elif msg_type == 'servo_command':
                angle = payload.get('angle', 0)
                self.robot.state.camera_pan_angle = angle
                
            elif msg_type == 'get_state':
                await self._send_robot_state(websocket)
                
            elif msg_type == 'reset':
                x = payload.get('x', 0.0)
                y = payload.get('y', 0.0)
                theta = payload.get('theta', 0.0)
                self.robot.reset(x, y, theta)
                
            else:
                print(f"Unknown message type: {msg_type}")
                
        except json.JSONDecodeError:
            print(f"Invalid JSON message: {message}")
        except Exception as e:
            print(f"Error handling message: {e}")
    
    async def _send_robot_state(self, websocket=None):
        """Send robot state to WebSocket clients"""
        state = self.robot.get_state()
        
        # Convert state to dictionary
        state_dict = {
            'type': 'robot_state',
            'timestamp': time.time(),
            'payload': {
                'pose': {
                    'x': state.x,
                    'y': state.y,
                    'theta': state.theta
                },
                'velocity': {
                    'linear': state.v,
                    'angular': state.omega
                },
                'motors': {
                    'left_pwm': state.motor_left_pwm,
                    'right_pwm': state.motor_right_pwm,
                    'left_rpm': state.motor_left_rpm,
                    'right_rpm': state.motor_right_rpm
                },
                'sensors': {
                    'ultrasonic_distance': state.ultrasonic_distance,
                    'imu_accel': state.imu_accel.tolist(),
                    'imu_gyro': state.imu_gyro.tolist(),
                    'camera_pan_angle': state.camera_pan_angle
                }
            }
        }
        
        message = json.dumps(state_dict)
        
        if websocket:
            await websocket.send(message)
        else:
            # Broadcast to all clients
            if self.clients:
                await asyncio.gather(
                    *[client.send(message) for client in self.clients],
                    return_exceptions=True
                )
    
    def start_serial_connection(self, port: str, baudrate: int = 115200):
        """Start serial connection to ESP32"""
        try:
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1
            )
            print(f"Serial connection established on {port}")
            
            # Start serial reading thread
            serial_thread = threading.Thread(
                target=self._serial_read_loop,
                daemon=True
            )
            serial_thread.start()
            
        except serial.SerialException as e:
            print(f"Failed to open serial port {port}: {e}")
    
    def _serial_read_loop(self):
        """Serial communication reading loop"""
        while self.running and self.serial_connection:
            try:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    self._process_serial_data(data)
                    
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                break
            except Exception as e:
                print(f"Unexpected error in serial loop: {e}")
                break
    
    def _process_serial_data(self, data: bytes):
        """Process incoming serial data"""
        try:
            # Try to decode as JSON first (for debug messages)
            try:
                text = data.decode('utf-8').strip()
                if text.startswith('{'):
                    json_data = json.loads(text)
                    print(f"ESP32 Debug: {json_data}")
                    return
            except:
                pass
            
            # Try to decode as I2C message
            if len(data) >= 2:
                try:
                    message = I2CMessage.from_bytes(data)
                    self._handle_i2c_message(message)
                except Exception as e:
                    print(f"Failed to decode I2C message: {e}")
                    
        except Exception as e:
            print(f"Error processing serial data: {e}")
    
    def _handle_i2c_message(self, message: I2CMessage):
        """Handle I2C protocol messages"""
        if message.message_type in self.message_handlers:
            self.message_handlers[message.message_type](message)
        else:
            print(f"Unknown message type: {message.message_type}")
    
    def _handle_move_command(self, message: I2CMessage):
        """Handle move command from ESP32"""
        if len(message.data) >= 2:
            left_pwm = struct.unpack('b', message.data[0:1])[0]
            right_pwm = struct.unpack('b', message.data[1:2])[0]
            
            # Scale from -127,127 to -255,255
            left_pwm = int(left_pwm * 255 / 127)
            right_pwm = int(right_pwm * 255 / 127)
            
            self.robot.set_motor_commands(left_pwm, right_pwm)
            print(f"Move command: left={left_pwm}, right={right_pwm}")
    
    def _handle_servo_command(self, message: I2CMessage):
        """Handle servo command from ESP32"""
        if len(message.data) >= 1:
            angle = struct.unpack('b', message.data[0:1])[0]
            self.robot.state.camera_pan_angle = angle
            print(f"Servo command: angle={angle}°")
    
    def _handle_sound_command(self, message: I2CMessage):
        """Handle sound command from ESP32"""
        if len(message.data) >= 2:
            frequency = struct.unpack('H', message.data[0:2])[0]
            duration = struct.unpack('H', message.data[2:4])[0] if len(message.data) >= 4 else 100
            print(f"Sound command: frequency={frequency}Hz, duration={duration}ms")
    
    def _handle_led_command(self, message: I2CMessage):
        """Handle LED command from ESP32"""
        if len(message.data) >= 3:
            r = message.data[0]
            g = message.data[1]
            b = message.data[2]
            print(f"LED command: RGB({r}, {g}, {b})")
    
    def _handle_stop_command(self, message: I2CMessage):
        """Handle stop command from ESP32"""
        self.robot.set_motor_commands(0, 0)
        print("Stop command received")
    
    def _handle_status_request(self, message: I2CMessage):
        """Handle status request from ESP32"""
        # Send back robot status
        if self.serial_connection:
            state = self.robot.get_state()
            status_data = struct.pack('fff', state.x, state.y, state.theta)
            response = I2CMessage(MessageType.STATUS, status_data)
            self.serial_connection.write(response.to_bytes())
    
    def send_sensor_data_to_esp32(self):
        """Send sensor data to ESP32"""
        if self.serial_connection:
            state = self.robot.get_state()
            
            # Pack sensor data
            sensor_data = struct.pack(
                'fffff',
                state.ultrasonic_distance,
                state.imu_accel[0],
                state.imu_accel[1],
                state.imu_accel[2],
                state.imu_gyro[2]  # Only Z-axis gyro for heading
            )
            
            message = I2CMessage(MessageType.SENSOR_DATA, sensor_data)
            self.serial_connection.write(message.to_bytes())
    
    async def update_loop(self):
        """Main update loop for communication and simulation"""
        while self.running:
            current_time = time.time()
            
            # Update robot simulation
            self.robot.update()
            
            # Send sensor data to ESP32 at regular intervals
            if current_time - self.last_sensor_update > self.sensor_update_interval:
                self.send_sensor_data_to_esp32()
                self.last_sensor_update = current_time
            
            # Broadcast state to WebSocket clients
            if self.clients:
                await self._send_robot_state()
            
            # Sleep for simulation timestep
            await asyncio.sleep(self.robot.dt)
    
    async def start(self, serial_port: Optional[str] = None):
        """Start the communication bridge"""
        self.running = True
        
        # Start WebSocket server
        await self.start_websocket_server()
        
        # Start serial connection if port provided
        if serial_port:
            self.start_serial_connection(serial_port)
        
        # Start update loop
        print("Communication bridge started")
        await self.update_loop()
    
    def stop(self):
        """Stop the communication bridge"""
        self.running = False
        
        if self.websocket_server:
            self.websocket_server.close()
        
        if self.serial_connection:
            self.serial_connection.close()
        
        print("Communication bridge stopped")


async def main():
    """Main function for testing"""
    config_path = "../config/robot_config.yaml"
    robot = DifferentialDriveRobot(config_path)
    bridge = ESP32CommunicationBridge(config_path, robot)
    
    try:
        await bridge.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
        bridge.stop()


if __name__ == "__main__":
    print("ESP32 Communication Bridge Test")
    print("=" * 40)
    print("Starting communication bridge...")
    print("Connect ESP32 via serial or WebSocket client")
    print("Press Ctrl+C to stop")
    
    asyncio.run(main())