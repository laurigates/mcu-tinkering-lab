# ESP32 WireGuard Home Assistant Example

A complete example project demonstrating how to connect an ESP32 to Home Assistant via a WireGuard VPN tunnel using ESPHome. This enables secure remote access to IoT devices without exposing them directly to the internet.

## Features

- **WireGuard VPN Integration**: Secure VPN tunnel for remote device access
- **Home Assistant Native API**: Direct, low-latency communication with Home Assistant
- **Connection Monitoring**: Real-time status of both WiFi and WireGuard connections
- **Encrypted Communication**: API encryption for secure data transmission
- **OTA Updates**: Wireless firmware updates over WiFi or VPN
- **Example Sensors**: WiFi signal, uptime, and VPN handshake monitoring

## Use Cases

This example is useful when:
- Home Assistant is on a different network (e.g., cloud server, different location)
- You want secure remote access without port forwarding
- Running IoT devices across multiple locations connected via VPN
- Testing ESPHome over WireGuard before deploying to production

## Hardware Requirements

- **ESP32 Development Board** (ESP32-DevKitC, NodeMCU-32S, or similar)
  - Note: WireGuard is only supported on ESP32, ESP8266, and BK72xx platforms
- **USB cable** for initial programming
- **WiFi network** with internet access

## Prerequisites

### 1. WireGuard VPN Server

You need a WireGuard server that both the ESP32 and Home Assistant can connect to. Options include:

- **Self-hosted**: Install on Linux server, Raspberry Pi, or router
- **Cloud VPS**: DigitalOcean, Linode, AWS Lightsail
- **Pre-configured**: PiVPN, Tailscale, or similar solutions

**Important**: Ensure your WireGuard server allows peer-to-peer communication.

### 2. Home Assistant

- Home Assistant instance accessible via the VPN network
- ESPHome integration enabled (usually auto-discovered)

### 3. Development Tools

- Python 3.9 or newer
- ESPHome CLI (`pip install esphome`)
- WireGuard tools (optional, for key generation): `apt-get install wireguard-tools`

## Quick Start

### 1. Initialize Project

```bash
make init
```

This will:
- Install ESPHome
- Create `secrets.yaml` from the template

### 2. Generate WireGuard Keys

```bash
make wg-keys
```

Or manually:
```bash
wg genkey | tee private.key | wg pubkey > public.key
```

Save the **private key** for your ESP32's `secrets.yaml`.
Share the **public key** with your WireGuard server administrator.

### 3. Configure WireGuard Server

Add the ESP32 as a peer on your WireGuard server. Example server configuration:

```ini
# /etc/wireguard/wg0.conf

[Interface]
Address = 10.0.0.1/24
ListenPort = 51820
PrivateKey = <server-private-key>

# ESP32 peer
[Peer]
PublicKey = <esp32-public-key>
AllowedIPs = 10.0.0.100/32
```

Restart WireGuard server: `sudo wg-quick down wg0 && sudo wg-quick up wg0`

### 4. Configure Secrets

Edit `secrets.yaml` with your configuration:

```yaml
# WiFi
wifi_ssid: "MyWiFi"
wifi_password: "MyPassword"

# WireGuard
wg_address: "10.0.0.100"
wg_private_key: "<esp32-private-key>"
wg_peer_endpoint: "vpn.example.com"
wg_peer_public_key: "<server-public-key>"
wg_peer_port: "51820"
wg_allowed_ips: "10.0.0.0/24"  # Or "0.0.0.0/0" for all traffic

# Network (optional - can use DHCP)
device_ip: "192.168.1.100"
gateway_ip: "192.168.1.1"
# ... etc
```

### 5. Upload Firmware

First upload via USB:

```bash
make upload
```

### 6. Monitor Logs

Watch the connection process:

```bash
make logs
```

You should see:
1. WiFi connection established
2. Time synchronized via NTP
3. WireGuard handshake completed
4. Home Assistant API connected

### 7. Add to Home Assistant

The device should appear automatically in Home Assistant:

1. Navigate to **Settings** → **Devices & Services**
2. Look for "ESP32 WireGuard HA Example" under ESPHome
3. Click **Configure** and enter the API encryption key from `secrets.yaml`

## Configuration Explained

### Time Synchronization

WireGuard requires accurate time for cryptographic operations:

```yaml
time:
  - platform: sntp
    timezone: "UTC"
```

**Important**: Do NOT use the `homeassistant` time platform if Home Assistant is accessed via WireGuard, as this creates a circular dependency.

### WireGuard Settings

```yaml
wireguard:
  address: "10.0.0.100"          # Device's VPN IP
  private_key: !secret wg_private_key
  peer_endpoint: !secret wg_peer_endpoint
  peer_public_key: !secret wg_peer_public_key
  peer_port: 51820
  peer_allowed_ips:              # Networks routed through VPN
    - "10.0.0.0/24"              # Just VPN network
    # - "0.0.0.0/0"              # ALL traffic (use carefully)
  peer_persistent_keepalive: 25s # NAT traversal
  netmask: 255.255.255.0
```

### Allowed IPs

The `peer_allowed_ips` setting determines what traffic goes through the VPN:

- `10.0.0.0/24` - Only VPN traffic (recommended)
- `192.168.50.0/24` - Specific network (e.g., Home Assistant's network)
- `0.0.0.0/0` - All traffic (makes ESP32 fully remote)

## Monitoring

The configuration includes several sensors for monitoring:

### Binary Sensors
- **WireGuard Status**: Shows if VPN peer is online
- **Device Status**: Overall device health

### Sensors
- **WiFi Signal**: Signal strength in dB
- **Uptime**: Device uptime in seconds
- **WG Latest Handshake**: Timestamp of last successful handshake

### Text Sensors
- **WG Address**: Current VPN address
- **Device IP**: Local WiFi IP
- **ESPHome Version**: Installed firmware version

## Troubleshooting

### WireGuard Not Connecting

**Check time synchronization:**
```bash
make logs | grep -i "time"
```

Time must be synchronized before WireGuard can connect.

**Verify server connectivity:**
- Ensure the WireGuard server is reachable from your WiFi network
- Check firewall rules allow UDP port 51820
- Verify DNS resolution of `wg_peer_endpoint`

**Check server configuration:**
```bash
# On WireGuard server
sudo wg show
```

Look for the ESP32 peer and verify its public key matches.

### Home Assistant Not Discovering Device

**If using WireGuard for HA connection:**
- Ensure `wg_allowed_ips` includes Home Assistant's network
- Verify WireGuard peer is online (check HA binary sensor)
- Check Home Assistant can ping the ESP32's VPN IP

**API encryption key mismatch:**
- Re-generate key: `openssl rand -base64 32`
- Update in both `secrets.yaml` and Home Assistant

### Common Issues

**"Time is not synchronized"**
- Check NTP server accessibility
- Verify internet connection from WiFi network
- Consider using local NTP server

**"Handshake did not complete"**
- Verify all WireGuard keys are correct
- Check server allows the ESP32's VPN IP in AllowedIPs
- Ensure keepalive is set (important for NAT)

**"API connection failed"**
- Verify encryption key matches in Home Assistant
- Check WireGuard connection is established first
- Ensure Home Assistant is accessible via VPN

## Advanced Configuration

### Adding Sensors

Uncomment the I2C sensor example in the YAML file:

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22

sensor:
  - platform: bme280
    temperature:
      name: "Temperature"
    # ...
```

### Adding Controls

Enable the relay/LED example:

```yaml
switch:
  - platform: gpio
    pin: GPIO2
    name: "Relay 1"
```

### Full VPN Routing

To route ALL ESP32 traffic through VPN:

```yaml
wireguard:
  # ...
  peer_allowed_ips:
    - "0.0.0.0/0"
```

**Warning**: This may impact OTA updates if the VPN server has limited bandwidth.

## Security Considerations

1. **Never commit `secrets.yaml`** - It's in `.gitignore` by default
2. **Use API encryption** - Always set an encryption key
3. **Strong OTA password** - Prevent unauthorized firmware updates
4. **Firewall rules** - Only allow necessary traffic on WireGuard server
5. **Regular updates** - Keep ESPHome and WireGuard server updated

## Network Topology

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   ESP32     │ WiFi    │  WireGuard   │ Internet│    Home     │
│  (10.0.0.100)◄────────►    Server    ◄─────────►  Assistant  │
│             │         │  (10.0.0.1)  │         │ (10.0.0.5)  │
└─────────────┘         └──────────────┘         └─────────────┘
      │                                                  │
      └──────────────── VPN Tunnel ────────────────────┘
```

## References

- [ESPHome WireGuard Component](https://esphome.io/components/wireguard/)
- [ESPHome Native API](https://esphome.io/components/api/)
- [WireGuard Official Documentation](https://www.wireguard.com/)
- [Home Assistant ESPHome Integration](https://www.home-assistant.io/integrations/esphome/)

## License

This example is provided as-is for educational and development purposes.

## Contributing

This is part of the MCU Tinkering Lab repository. Feel free to submit improvements or report issues.

## Next Steps

1. **Add real sensors** - Temperature, humidity, motion, etc.
2. **Create automations** - Use Home Assistant to automate based on sensor data
3. **Multi-location deployment** - Connect devices from different physical locations
4. **Mesh network** - Connect multiple ESP32 devices via WireGuard
5. **Monitoring dashboard** - Create a Home Assistant dashboard for all VPN-connected devices
