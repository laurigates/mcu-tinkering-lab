# WiFi Setup Guide for Heltec WiFi LoRa 32 V1 Main Controller

## Overview
This document explains how to configure and use WiFi connectivity on the main controller to enable OTA updates and remote monitoring.

## Quick Start

### Method 1: Configure via Code (Compile-time)
1. Copy `wifi_config_example.h` to `wifi_config.h`
2. Edit `wifi_config.h` with your WiFi credentials:
   ```c
   #define WIFI_SSID     "your-wifi-ssid"
   #define WIFI_PASSWORD "your-wifi-password"
   ```
3. Rebuild and flash the firmware

### Method 2: Configure via Serial Console (Runtime)
1. Connect to the device via serial console (115200 baud)
2. Use WiFi commands (to be implemented):
   - `WIFI:CONNECT:ssid:password` - Connect to network
   - `WIFI:SAVE` - Save credentials to NVS
   - `WIFI:STATUS` - Show connection status
   - `WIFI:SCAN` - Scan for available networks

### Method 3: Configure Programmatically (Runtime)
```c
// Connect with specific credentials
wifi_manager_connect("MyNetwork", "MyPassword");

// Save credentials for auto-connect
wifi_credentials_t creds = {
    .ssid = "MyNetwork",
    .password = "MyPassword"
};
wifi_manager_save_credentials(&creds);
```

## Features

### Automatic Reconnection
- The WiFi manager automatically reconnects on disconnection
- Uses exponential backoff to prevent network flooding
- Maximum of 10 retry attempts by default

### Connection Monitoring
The WiFi manager provides real-time connection status:
```c
wifi_info_t info;
wifi_manager_get_info(&info);
printf("Connected: %s\n", info.is_connected ? "Yes" : "No");
printf("IP Address: %s\n", info.ip_address);
printf("RSSI: %d dBm\n", info.rssi);
```

### Power Management
For robotics applications, we recommend using maximum performance mode:
```c
// No power saving, lowest latency
wifi_manager_set_power_mode(WIFI_POWER_MAX_PERFORMANCE);
```

Available modes:
- `WIFI_POWER_MAX_PERFORMANCE` - No power saving (recommended)
- `WIFI_POWER_BALANCED` - Balanced power/performance
- `WIFI_POWER_MIN_MODEM` - Minimum modem power saving
- `WIFI_POWER_MAX_MODEM` - Maximum modem power saving

### Secure Credential Storage
Credentials are stored in encrypted NVS (Non-Volatile Storage):
- Survives power cycles and resets
- Can be cleared with `wifi_manager_clear_credentials()`
- Protected from casual access

## Network Recovery
The WiFi manager implements robust recovery mechanisms:

1. **Exponential Backoff**: Retry delays increase exponentially (1s, 2s, 4s, 8s, 16s, 30s)
2. **Connection Monitoring**: Automatic detection of network issues
3. **State Tracking**: Always know the current connection state
4. **Event Callbacks**: Register callbacks for state changes

## Example Usage

### Basic Connection
```c
// Initialize WiFi manager (done automatically in main.c)
wifi_manager_init();

// Connect to network
wifi_manager_connect("MyNetwork", "MyPassword");

// Check connection status
if (wifi_manager_is_connected()) {
    printf("Connected successfully!\n");
}
```

### With State Monitoring
```c
void wifi_state_callback(wifi_state_t state, void* data) {
    printf("WiFi state changed to: %s\n",
           wifi_manager_state_to_string(state));
}

// Register callback
wifi_manager_register_callback(wifi_state_callback, NULL);
```

### Network Scanning
```c
// Start scan
wifi_manager_start_scan();

// Get results
wifi_ap_record_t ap_records[10];
uint16_t count;
wifi_manager_get_scan_results(ap_records, 10, &count);

for (int i = 0; i < count; i++) {
    printf("SSID: %s, RSSI: %d\n",
           ap_records[i].ssid,
           ap_records[i].rssi);
}
```

## Troubleshooting

### Connection Fails
1. Check credentials are correct
2. Ensure WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
3. Check serial console for detailed error messages
4. Try increasing `WIFI_MAXIMUM_RETRY`

### Poor Connection Quality
1. Check RSSI value (should be > -70 dBm for good connection)
2. Move device closer to access point
3. Consider using external antenna if available
4. Switch to `WIFI_POWER_MAX_PERFORMANCE` mode

### Credentials Not Saving
1. Ensure NVS partition exists in partition table
2. Check for NVS initialization errors in logs
3. Try erasing NVS with `wifi_manager_clear_credentials()`

## LED Status Indicators
When connected:
- RGB LEDs will flash green briefly on successful connection
- OLED display shows WiFi status and IP address

## Security Considerations
1. Never commit `wifi_config.h` to version control
2. Use WPA2 or better encryption on your network
3. Consider implementing certificate-based authentication for production
4. Regularly update ESP-IDF for security patches

## Next Steps
- Configure OTA update server address
- Implement remote monitoring endpoints
- Set up secure communication protocols
