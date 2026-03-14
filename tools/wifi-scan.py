#!/usr/bin/env python3
"""Scan for WiFi APs using CoreWLAN.

Works on macOS Sequoia+ where airport CLI was removed and SSIDs are
redacted without Location Services.

Usage:
    wifi-scan.py              # Broad scan (SSIDs redacted on Sequoia+)
    wifi-scan.py <ssid>       # Targeted scan (bypasses redaction)
"""

import sys

try:
    import objc
except ImportError:
    print(
        "pyobjc not installed (pip install pyobjc-framework-CoreWLAN)", file=sys.stderr
    )
    sys.exit(2)

objc.loadBundle(
    "CoreWLAN",
    globals(),
    bundle_path="/System/Library/Frameworks/CoreWLAN.framework",
)

iface = CWWiFiClient.sharedWiFiClient().interface()  # type: ignore[name-defined]  # noqa: F821

ssid = sys.argv[1] if len(sys.argv) > 1 else None

if ssid:
    # Targeted scan for a specific SSID (not redacted)
    networks, err = iface.scanForNetworksWithSSID_error_(ssid.encode(), None)
    if networks and len(networks) > 0:
        for n in networks:
            print(
                f"{ssid}: FOUND "
                f"({n.rssiValue()} dBm, ch{n.wlanChannel().channelNumber()}, "
                f"BSSID={n.bssid()})"
            )
    else:
        print(f"{ssid}: NOT FOUND")
else:
    # Broad scan (SSIDs redacted on macOS Sequoia+ without Location Services)
    all_nets, _ = iface.scanForNetworksWithName_error_(None, None)
    count = len(all_nets) if all_nets else 0
    print(f"{count} nearby APs detected (SSIDs redacted by macOS privacy)")
    print("  Tip: use System Settings > Wi-Fi to see SSID names")
    print("  Or pass an SSID argument for a targeted scan: wifi-scan.py <ssid>")
