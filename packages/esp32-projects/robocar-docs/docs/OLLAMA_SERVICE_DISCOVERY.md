# Ollama Service Discovery with SRV Records

## Overview

The ESP32-CAM now supports automatic discovery of Ollama services using DNS SRV records and mDNS. This eliminates the need for hardcoded IP addresses and makes deployment more flexible.

## Features

- **mDNS Discovery**: Automatically find Ollama services on the local network using `_ollama._tcp.local`
- **DNS SRV Records**: Support for standard DNS SRV record resolution
- **Fallback Mechanism**: Falls back to configured static IP if discovery fails
- **Priority/Weight Support**: Selects the best available service based on SRV record priority and weight
- **IPv4/IPv6 Support**: Works with both IPv4 and IPv6 addresses

## Configuration

The service discovery is configured in `main/config.h`:

```c
// Ollama Service Discovery Configuration
#define OLLAMA_USE_SERVICE_DISCOVERY    1                           // Enable SRV record discovery
#define OLLAMA_SRV_RECORD              "_ollama._tcp.local"         // SRV record for mDNS discovery
#define OLLAMA_FALLBACK_URL            "http://192.168.0.115:11434" // Fallback URL if discovery fails
#define OLLAMA_DISCOVERY_TIMEOUT_MS    5000                         // DNS query timeout
#define OLLAMA_USE_MDNS                1                            // Enable mDNS for local discovery
```

## Setting Up Local mDNS Service

### macOS (Recommended: Built-in dns-sd)

macOS includes `dns-sd` by default, making this the simplest approach:

1. **Publish Ollama service**:
   ```bash
   # Run in foreground (for testing)
   dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate

   # Run in background (for permanent use)
   dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate &
   ```

2. **Verify the service**:
   ```bash
   # Browse for Ollama services
   dns-sd -B _ollama._tcp

   # Look up the specific service details
   dns-sd -L "Ollama AI" _ollama._tcp .

   # Test hostname resolution
   ping ollama.local
   ```

3. **Make it persistent** (auto-start on boot):

   Create a LaunchAgent:
   ```bash
   cat > ~/Library/LaunchAgents/com.ollama.mdns.plist << 'EOF'
   <?xml version="1.0" encoding="UTF-8"?>
   <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
   <plist version="1.0">
   <dict>
       <key>Label</key>
       <string>com.ollama.mdns</string>
       <key>ProgramArguments</key>
       <array>
           <string>/usr/bin/dns-sd</string>
           <string>-R</string>
           <string>Ollama AI</string>
           <string>_ollama._tcp</string>
           <string>.</string>
           <string>11434</string>
           <string>path=/api/generate</string>
       </array>
       <key>RunAtLoad</key>
       <true/>
       <key>KeepAlive</key>
       <true/>
   </dict>
   </plist>
   EOF

   # Load and start the service
   launchctl load ~/Library/LaunchAgents/com.ollama.mdns.plist
   launchctl start com.ollama.mdns
   ```

### Linux (Recommended: Avahi)

Most Linux distributions include Avahi for mDNS services:

1. **Install Avahi** (if not already installed):
   ```bash
   # Ubuntu/Debian
   sudo apt-get update
   sudo apt-get install avahi-utils avahi-daemon

   # CentOS/RHEL/Fedora
   sudo yum install avahi-tools avahi
   # or on newer versions:
   sudo dnf install avahi-tools avahi

   # Arch Linux
   sudo pacman -S avahi
   ```

2. **Start Avahi daemon**:
   ```bash
   # Enable and start the avahi daemon
   sudo systemctl enable avahi-daemon
   sudo systemctl start avahi-daemon

   # Check if it's running
   sudo systemctl status avahi-daemon
   ```

3. **Publish Ollama service**:
   ```bash
   # Run in foreground (for testing)
   avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate

   # Run in background (for permanent use)
   avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate &
   ```

4. **Verify the service**:
   ```bash
   # Browse for all Ollama services
   avahi-browse -r _ollama._tcp -t

   # Browse for all services (to see if yours appears)
   avahi-browse -a -t | grep ollama

   # Test hostname resolution
   ping ollama.local
   ```

5. **Make it persistent** with systemd:

   Create a systemd service:
   ```bash
   sudo tee /etc/systemd/system/ollama-mdns.service << 'EOF'
   [Unit]
   Description=Ollama mDNS Service Advertisement
   After=network.target avahi-daemon.service
   Requires=avahi-daemon.service

   [Service]
   Type=simple
   ExecStart=/usr/bin/avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate
   Restart=always
   RestartSec=5
   User=nobody
   Group=nogroup

   [Install]
   WantedBy=multi-user.target
   EOF

   # Enable and start the service
   sudo systemctl daemon-reload
   sudo systemctl enable ollama-mdns.service
   sudo systemctl start ollama-mdns.service

   # Check status
   sudo systemctl status ollama-mdns.service
   ```

### Alternative Method: Using Avahi on macOS

If you prefer Avahi on macOS (for consistency with Linux):

1. **Install Avahi**:
   ```bash
   brew install avahi
   brew services start avahi
   ```

2. **Publish the service**:
   ```bash
   avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate
   ```

3. **Verify**:
   ```bash
   avahi-browse -r _ollama._tcp -t
   ```

### Method 3: Docker with Avahi

Create a `docker-compose.yml` to run Ollama with mDNS advertisement:

```yaml
version: '3.8'
services:
  ollama:
    image: ollama/ollama
    ports:
      - "11434:11434"
    volumes:
      - ollama:/root/.ollama
    restart: unless-stopped

  avahi:
    image: flungo/avahi
    network_mode: host
    volumes:
      - /var/run/dbus:/var/run/dbus
    environment:
      - SERVICES=ollama:_ollama._tcp:11434:path=/api/generate
    restart: unless-stopped

volumes:
  ollama:
```

## Testing Service Discovery

### 1. Check mDNS Resolution

From your development machine:

```bash
# Test DNS resolution
ping ollama.local

# Browse for services
avahi-browse -r _ollama._tcp -t
```

### 2. Monitor ESP32-CAM Logs

When the ESP32-CAM starts up, you should see logs like:

```
I (1234) ollama_discovery: Service discovery initialized with SRV record: _ollama._tcp.local
I (1245) ollama_discovery: mDNS initialized successfully
I (1256) ollama_discovery: Found service: ollama.local:11434 (192.168.0.115)
I (1267) ollama_discovery: Best service: ollama.local:11434 (priority=0, weight=0)
I (1278) ollama_backend: Using discovered Ollama service: http://192.168.0.115:11434/api/generate
```

### 3. Test Fallback Behavior

To test the fallback mechanism:

1. Stop the mDNS service advertisement
2. Restart the ESP32-CAM
3. Should see logs like:

```
W (1234) ollama_discovery: No Ollama services discovered: ESP_ERR_NOT_FOUND
I (1245) ollama_backend: Using configured API URL: http://192.168.0.115:11434/api/generate
```

## Troubleshooting

### Common Issues

1. **"AI backend not available"**
   - Check if Ollama is running
   - Verify mDNS service is published
   - Check fallback URL configuration

2. **"Failed to resolve hostname"**
   - Ensure devices are on the same network
   - Check firewall settings
   - Verify mDNS is working: `ping ollama.local`

3. **"mDNS query failed"**
   - Check if Avahi daemon is running
   - Verify service is published correctly
   - Try restarting the ESP32-CAM

### Debug Commands

```bash
# Check if service is advertised
avahi-browse -a -t | grep ollama

# Test DNS resolution
nslookup ollama.local

# Check network connectivity
ping [ESP32_IP_ADDRESS]
```

## Network Requirements

- ESP32-CAM and Ollama server must be on the same local network for mDNS
- Multicast traffic must be allowed (UDP port 5353)
- Firewall should allow HTTP traffic on Ollama port (default 11434)

## Benefits

1. **Zero Configuration**: Automatically finds services without manual IP configuration
2. **Network Resilience**: Adapts to IP address changes and service relocations
3. **Multiple Services**: Can discover and prioritize multiple Ollama instances
4. **Development Friendly**: Easy setup for local development environments
5. **Production Ready**: Supports proper DNS infrastructure in production

## SRV Record Format

For production environments using DNS servers, create SRV records in this format:

```
_ollama._tcp.example.com. 300 IN SRV 10 5 11434 ollama-server.example.com.
```

Where:
- `10` = Priority (lower numbers have higher priority)
- `5` = Weight (higher numbers preferred among same priority)
- `11434` = Port number
- `ollama-server.example.com.` = Target hostname
