# WiFi Signal Monitor for Heltec V3

Turns a Heltec WiFi Kit 32 V3 or WiFi LoRa 32 V3 (ESP32-S3 with a 0.96" OLED) into a pocket WiFi signal meter. It lists every network in range, strongest first. Pick one to track it live with a big dBm readout, signal bars and a scrolling history graph.

No WiFi password is needed. Readings come from WiFi scans, so it works with any network in range.

## Controls

Everything runs from the **PRG** button (GPIO0):

| Screen | Tap | Hold |
|---|---|---|
| Network list | Move to the next network | Track the highlighted network |
| Tracking | Switch between the graph and the stats | Go back to the list |

- **Tracking view:** dBm, quality %, a label (Great / Good / Fair / Weak / Poor), 4-bar icon and a ~30 s history graph with a dotted line at -70 dBm. It updates 4 times a second.
- **Stats view:** current, min, max and average RSSI, sample count, channel, security type and BSSID.
- **Signal lost:** the screen shows **LOST** and the device scans every channel until the access point comes back, in case it changed channel.

## Serial output

115200 baud. The network list is printed after every scan. While tracking, it prints `RSSI:<dBm>` lines, so **Tools → Serial Plotter** draws a live graph on your PC.

## Build and flash

1. Install the **esp32** board package by Espressif, version 3.x.
2. Install the **Adafruit SSD1306** and **Adafruit GFX** libraries.
3. Open `WiFiSignalMonitor.ino`, select **Heltec WiFi Kit 32(V3)** or **Heltec WiFi LoRa 32(V3)**, and upload.

## Tuning

These constants at the top of the sketch control the timing and thresholds:

| Constant | Default | What it does |
|---|---|---|
| `TRACK_INTERVAL_MS` | 250 | Time between RSSI samples while tracking |
| `LIST_SCAN_MS_PER_CH` | 120 | Dwell time per channel in the full scan |
| `LOST_AFTER_MISSES` | 4 | Missed samples before the screen shows LOST |
| `LONG_PRESS_MS` | 600 | Hold time that counts as a long press |

The bar and label thresholds are in `barsFor()`: -55, -65, -75 and -85 dBm.

## Pins (Heltec V3)

| Function | GPIO |
|---|---|
| OLED SDA / SCL / RST | 17 / 18 / 21 |
| Vext (OLED power, active LOW) | 36 |
| PRG button | 0 |
