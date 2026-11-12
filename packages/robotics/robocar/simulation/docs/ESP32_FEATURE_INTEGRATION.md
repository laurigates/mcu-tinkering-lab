# ESP32 Feature Integration Documentation

## Overview

This documentation covers the comprehensive ESP32 feature integration for the robocar simulation project, specifically focusing on WiFi Manager and OTA (Over-The-Air) update functionality. The simulation accurately models real ESP32 behavior including partition management, connection states, and update processes.

## Architecture

### Dual ESP32 Board Design

The robocar uses a dual ESP32 architecture:
- **Main Board**: Motor control, sensors, navigation logic
- **Camera Board**: Computer vision, AI processing, image streaming

Both boards integrate with the WiFi and OTA simulation systems for realistic ESP-IDF behavior testing.

### Key Components

1. **WiFi Manager Simulation** (`wifi_simulation.py`)
2. **OTA Simulation** (`ota_simulation.py`)
3. **Robot Model Integration** (`robot_model.py`)
4. **Error Handling Framework** (`error_handling.py`)
5. **Configuration Management** (`robot_config.yaml`)

## WiFi Manager Simulation

### Overview

The `WiFiManagerSimulation` class provides a comprehensive simulation of ESP32 WiFi functionality, matching ESP-IDF WiFi manager behavior.

### Features

- **State Management**: Tracks WiFi connection states (DISCONNECTED, CONNECTING, CONNECTED, ERROR)
- **Network Scanning**: Simulates discovery of available WiFi networks
- **Connection Management**: Handles connection attempts with realistic timing
- **Signal Strength Simulation**: Dynamic RSSI updates with realistic variations
- **Auto-Reconnection**: Automatic reconnection on connection loss
- **Power Management**: Power save mode simulation
- **OTA Readiness**: Monitors connection stability for OTA operations

### WiFi States

```python
class WiFiState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"
```

### Configuration

WiFi simulation is configured in `robot_config.yaml`:

```yaml
simulation:
  wifi:
    enabled: true
    auto_connect: true
    default_ssid: "ESP32-RoboCar"
    default_password: "robocar123"
    connection_stability_threshold: 30.0  # seconds for OTA readiness
    max_retry_attempts: 3
    retry_delay_ms: 5000
    power_save_mode: true
```

### Usage Examples

#### Basic WiFi Operations

```python
from wifi_simulation import WiFiManagerSimulation

# Initialize WiFi manager
wifi_manager = WiFiManagerSimulation("config/robot_config.yaml")
wifi_manager.init()

# Connect to network
success = wifi_manager.connect("MyNetwork", "password123")
if success:
    print("Connected successfully")

# Get connection info
connection_info = wifi_manager.get_connection_info()
if connection_info:
    ssid, rssi = connection_info
    print(f"Connected to {ssid} with RSSI {rssi} dBm")

# Scan for networks
networks = wifi_manager.scan_networks()
for network in networks:
    print(f"SSID: {network['ssid']}, RSSI: {network['rssi']} dBm")
```

#### Connection State Monitoring

```python
# Check current state
state = wifi_manager.get_state()
print(f"Current WiFi state: {state.value}")

# Get detailed status
status = wifi_manager.get_status_info()
print(f"Status: {status}")

# Check OTA readiness
if wifi_manager.is_ota_ready():
    print("WiFi connection is stable enough for OTA")
```

### API Reference

#### Core Methods

- `init() -> bool`: Initialize WiFi manager
- `connect(ssid: str, password: str) -> bool`: Connect to network
- `disconnect() -> bool`: Disconnect from current network
- `get_state() -> WiFiState`: Get current connection state
- `get_connection_info() -> Optional[Tuple[str, int]]`: Get SSID and RSSI
- `scan_networks() -> List[Dict]`: Scan for available networks
- `is_ota_ready() -> bool`: Check if connection is stable for OTA
- `set_auto_reconnect(enable: bool)`: Enable/disable auto-reconnection
- `set_power_save_mode() -> bool`: Enable power saving mode

#### Status Information

- `get_status_info() -> Dict`: Comprehensive status information

## OTA Simulation

### Overview

The `OTASimulation` class provides realistic ESP32 OTA update functionality, including partition management, firmware verification, and rollback capabilities.

### Features

- **Partition Management**: Simulates ESP32 partition table with factory, OTA_0, and OTA_1 partitions
- **Firmware Download**: Realistic download simulation with progress tracking
- **Verification Process**: SHA-256 verification with configurable failure rates
- **Rollback Support**: Factory firmware rollback capability
- **Status Tracking**: Detailed OTA operation status and progress
- **Error Recovery**: Integration with error handling framework

### OTA States

```python
class OTAState(Enum):
    IDLE = "idle"
    DOWNLOADING = "downloading"
    VERIFYING = "verifying"
    READY_TO_APPLY = "ready_to_apply"
    APPLYING = "applying"
    SUCCESS = "success"
    FAILED = "failed"
    ABORTED = "aborted"
```

### Partition Types

```python
class PartitionType(Enum):
    FACTORY = "factory"
    OTA_0 = "ota_0"
    OTA_1 = "ota_1"
```

### Configuration

OTA simulation is configured in `robot_config.yaml`:

```yaml
simulation:
  ota:
    enabled: true
    simulate_download_speed: 50000  # bytes/s (50KB/s)
    verification_delay: 1.0  # seconds
    failure_probability: 0.05  # 5% chance of verification failure
    partition_size: 1572864  # 1.5MB (0x180000)
```

### Usage Examples

#### Basic OTA Operations

```python
from ota_simulation import OTASimulation
from wifi_simulation import WiFiManagerSimulation

# Initialize OTA manager
ota_manager = OTASimulation("config/robot_config.yaml")

# Connect WiFi manager (required for OTA)
wifi_manager = WiFiManagerSimulation("config/robot_config.yaml")
wifi_manager.init()
wifi_manager.connect("MyNetwork", "password123")
ota_manager.set_wifi_manager(wifi_manager)

# Start firmware update
firmware_url = "https://example.com/firmware.bin"
success = ota_manager.start_update(firmware_url, version="1.2.0")
if success:
    print("OTA update started")

    # Monitor progress
    while True:
        status = ota_manager.get_status()
        print(f"OTA State: {status.state.value}, Progress: {status.progress}%")

        if status.state in [OTAState.SUCCESS, OTAState.FAILED, OTAState.ABORTED]:
            break

        time.sleep(1)
```

#### Partition Management

```python
# Get current boot partition
current_partition = ota_manager.get_boot_partition()
print(f"Currently booting from: {current_partition}")

# Get partition information
partitions = ota_manager.get_partition_info()
for name, info in partitions.items():
    print(f"Partition {name}: {info}")

# Get firmware versions
versions = ota_manager.get_firmware_versions()
for partition, version_info in versions.items():
    print(f"{partition}: {version_info}")

# Rollback to factory firmware
success = ota_manager.rollback_to_factory()
if success:
    print("Rolled back to factory firmware")
```

#### System Information

```python
# Get comprehensive system info
system_info = ota_manager.get_system_info()
print(f"System Info: {system_info}")

# Check OTA readiness
if ota_manager.is_ota_ready():
    print("System ready for OTA update")
else:
    print("WiFi connection required for OTA")
```

### API Reference

#### Core Methods

- `is_ota_ready() -> bool`: Check if system is ready for OTA
- `start_update(firmware_url: str, version: str = None) -> bool`: Start firmware update
- `abort_update() -> bool`: Abort current update
- `get_status() -> OTAStatus`: Get current OTA status
- `get_status_dict() -> Dict`: Get status as dictionary

#### Partition Management

- `set_boot_partition(partition_name: str) -> bool`: Set boot partition
- `get_boot_partition() -> str`: Get current boot partition
- `get_partition_info() -> Dict[str, Dict]`: Get partition table information
- `rollback_to_factory() -> bool`: Rollback to factory firmware

#### Information Methods

- `get_firmware_versions() -> Dict[str, Dict]`: Get all firmware version info
- `get_system_info() -> Dict`: Get comprehensive system information

## Robot Model Integration

### Overview

The WiFi and OTA simulations are integrated into the main robot model (`robot_model.py`) to provide seamless ESP32 functionality within the larger simulation framework.

### Integration Points

```python
class RobotModel:
    def __init__(self, config_path: str):
        # Initialize WiFi simulation
        self.wifi_simulation = None
        self.use_wifi_simulation = self.config['simulation'].get('wifi', {}).get('enabled', True)

        if self.use_wifi_simulation:
            self.wifi_simulation = WiFiManagerSimulation(config_path)
            self.wifi_simulation.init()
            print("Robot: WiFi simulation initialized")

        # Initialize OTA simulation
        self.ota_simulation = None
        self.use_ota_simulation = self.config['simulation'].get('ota', {}).get('enabled', True)

        if self.use_ota_simulation:
            self.ota_simulation = OTASimulation(config_path)
            if self.wifi_simulation:
                self.ota_simulation.set_wifi_manager(self.wifi_simulation)
            print("Robot: OTA simulation initialized")
```

### Access Methods

The robot model provides access to WiFi and OTA functionality through its interface:

```python
# Access WiFi manager
if robot.wifi_simulation:
    wifi_state = robot.wifi_simulation.get_state()

# Access OTA manager
if robot.ota_simulation:
    ota_status = robot.ota_simulation.get_status()
```

## Error Handling Integration

### Overview

Both WiFi and OTA simulations integrate with the project's comprehensive error handling framework, providing resilient operation and automatic recovery.

### Error Recovery Strategies

#### WiFi Recovery

```python
def wifi_recovery(error) -> bool:
    """Recovery strategy for WiFi errors"""
    if hasattr(error, 'error_code'):
        if error.error_code == 'WIFI_CONNECTION_FAILED':
            # Attempt to reconnect
            time.sleep(2)
            return self.connect(self._last_ssid, self._last_password)
    return False
```

#### OTA Recovery

```python
def ota_recovery(error) -> bool:
    """Recovery strategy for OTA errors"""
    if hasattr(error, 'error_code'):
        if error.error_code == 'OTA_DOWNLOAD_FAILED':
            # Reset state and prepare for retry
            self._current_state = OTAState.IDLE
            return True
    return False
```

### Resilient Operations

Critical operations use the resilient operation wrapper:

```python
@resilient_operation(
    component_name="WiFi Manager",
    max_retries=3,
    severity=ErrorSeverity.HIGH
)
def connect(self, ssid: str, password: str) -> bool:
    # Connection implementation with automatic retry
    pass
```

## Configuration Reference

### Complete WiFi Configuration

```yaml
simulation:
  wifi:
    enabled: true                    # Enable WiFi simulation
    auto_connect: true              # Auto-connect on startup
    default_ssid: "ESP32-RoboCar"   # Default network SSID
    default_password: "robocar123"  # Default network password
    connection_stability_threshold: 30.0  # Seconds for OTA readiness
    max_retry_attempts: 3           # Max connection retry attempts
    retry_delay_ms: 5000           # Delay between retries
    power_save_mode: true          # Enable power saving simulation
```

### Complete OTA Configuration

```yaml
simulation:
  ota:
    enabled: true                   # Enable OTA simulation
    simulate_download_speed: 50000  # Download speed in bytes/s
    verification_delay: 1.0         # Verification process delay
    failure_probability: 0.05       # Probability of verification failure
    partition_size: 1572864         # Partition size in bytes (1.5MB)
```

### Hardware Configuration

```yaml
communication:
  wifi:
    ssid: "ESP32-RoboCar"          # Hardware WiFi configuration
    password: "robocar123"         # Hardware WiFi password
```

## Testing and Validation

### Unit Testing

Both modules include comprehensive testing capabilities:

```python
# Test WiFi functionality
def test_wifi_simulation():
    wifi = WiFiManagerSimulation("config/robot_config.yaml")
    assert wifi.init() == True
    assert wifi.connect("TestSSID", "password") == True
    assert wifi.get_state() == WiFiState.CONNECTED

# Test OTA functionality
def test_ota_simulation():
    ota = OTASimulation("config/robot_config.yaml")
    assert ota.start_update("http://example.com/firmware.bin") == True
    status = ota.get_status()
    assert status.state == OTAState.DOWNLOADING
```

### Integration Testing

Test the complete integration:

```python
def test_full_integration():
    # Initialize robot with all features
    robot = RobotModel("config/robot_config.yaml")

    # Test WiFi connectivity
    assert robot.wifi_simulation.connect("TestNetwork", "password")

    # Test OTA readiness
    time.sleep(35)  # Wait for stability threshold
    assert robot.ota_simulation.is_ota_ready()

    # Test OTA update
    assert robot.ota_simulation.start_update("http://example.com/fw.bin")
```

## Troubleshooting

### Common Issues

1. **WiFi Not Connecting**
   - Check SSID and password configuration
   - Verify network simulation is enabled
   - Check error logs for connection failures

2. **OTA Not Ready**
   - Ensure WiFi is connected and stable
   - Wait for connection stability threshold (default 30 seconds)
   - Check WiFi manager integration

3. **OTA Download Failures**
   - Verify firmware URL accessibility
   - Check download speed configuration
   - Monitor verification process

### Debug Information

Enable detailed logging:

```python
# Enable debug output
wifi_manager = WiFiManagerSimulation("config/robot_config.yaml")
wifi_manager.init()

# Monitor status
status = wifi_manager.get_status_info()
print(f"Debug Info: {status}")
```

## Best Practices

### WiFi Management

1. **Connection Monitoring**: Always monitor connection state before critical operations
2. **Auto-Reconnection**: Enable auto-reconnection for robust operation
3. **Signal Quality**: Monitor RSSI for connection quality assessment
4. **Power Management**: Use power save mode when appropriate

### OTA Management

1. **WiFi Stability**: Ensure stable WiFi before starting OTA
2. **Progress Monitoring**: Monitor download and verification progress
3. **Error Handling**: Implement proper error handling and recovery
4. **Rollback Strategy**: Always have a rollback plan

### Configuration

1. **Environment Specific**: Adjust timeouts and thresholds for your environment
2. **Testing**: Use conservative settings for testing, optimize for production
3. **Documentation**: Document any configuration changes and their rationale

## Future Enhancements

### Planned Features

1. **Multiple Network Support**: Support for multiple configured networks
2. **Advanced Security**: WPA3 and enterprise authentication simulation
3. **Mesh Networking**: ESP-MESH protocol simulation
4. **OTA Security**: Signed firmware verification
5. **Performance Metrics**: Detailed performance and reliability metrics

### Extension Points

The current implementation provides extension points for:

- Custom network configurations
- Additional security protocols
- Alternative OTA sources
- Enhanced error recovery strategies
- Performance optimization

## Related Documentation

- [Robot Model Documentation](ROBOT_MODEL.md)
- [Error Handling Framework](ERROR_HANDLING.md)
- [Camera Simulation](CAMERA_SIMULATION.md)
- [Motor Control System](MOTOR_CONTROL.md)

## Support

For issues or questions regarding the ESP32 feature integration:

1. Check the troubleshooting section above
2. Review error logs and debug output
3. Verify configuration settings
4. Test individual components in isolation
5. Consult the API reference for proper usage
