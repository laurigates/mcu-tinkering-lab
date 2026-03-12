#!/usr/bin/env python3
"""Scan for xbox-bridge-log WiFi AP using CoreWLAN.

Works on macOS Sequoia+ where airport CLI was removed and SSIDs are
redacted without Location Services. Targeted SSID scan bypasses redaction.
"""

import sys

try:
    import objc
except ImportError:
    print("pyobjc not installed (pip install pyobjc-framework-CoreWLAN)", file=sys.stderr)
    sys.exit(2)

objc.loadBundle(
    "CoreWLAN",
    globals(),
    bundle_path="/System/Library/Frameworks/CoreWLAN.framework",
)

iface = CWWiFiClient.sharedWiFiClient().interface()  # noqa: F821

# Targeted scan for our specific SSID (not redacted)
networks, err = iface.scanForNetworksWithSSID_error_(b"xbox-bridge-log", None)
if networks and len(networks) > 0:
    for n in networks:
        print(
            f"xbox-bridge-log: FOUND "
            f"({n.rssiValue()} dBm, ch{n.wlanChannel().channelNumber()}, "
            f"BSSID={n.bssid()})"
        )
else:
    print("xbox-bridge-log: NOT FOUND")

# Full scan (SSIDs redacted on macOS Sequoia+ without Location Services)
all_nets, _ = iface.scanForNetworksWithName_error_(None, None)
count = len(all_nets) if all_nets else 0
print(f"\n{count} nearby APs detected (SSIDs redacted by macOS privacy)")
print("  Tip: use System Settings > Wi-Fi to see SSID names")
