# WiFi Motion Detector for Heltec V3

Detects people moving nearby with **only the Heltec V3**. You don't need an RCWL-0516, a PIR or any other sensor. The board listens to your router and watches how the signal changes. When a person moves between or near the board and the router, the radio waves reflect differently and the signal starts to wobble.

The sketch uses **WiFi CSI** (the amplitude of about 50 subcarriers per packet), which is much more sensitive than plain RSSI. If CSI isn't available, it falls back to RSSI automatically. You don't need the WiFi password because it only listens to the router's beacons.

## How to use

1. Flash it. The screen lists nearby networks, strongest first.
2. **Tap** PRG to move to your router, then **hold** PRG to select it.
3. **Keep still, or leave the room, for 10 s** while it calibrates.
4. The screen shows **Clear** or **MOTION**. The white LED lights up on motion.

| Screen | Tap | Hold |
|---|---|---|
| Network list | Move to the next network | Use this network |
| Monitoring | Recalibrate | Back to the list |

Monitor screen: status, `x1.3` (score ÷ threshold, where anything above 1 counts as motion), `n=` (motion events), a level bar (the middle tick is the threshold) and a ~30 s graph with a dotted threshold line. The header shows CSI or RSSI and packets/s.

## Placement tips

- Put the board 2–5 m from the router, with the area you want to watch **between** them.
- Aim for at least 10 packets/s in the header. Busy routers give more.
- Ceiling fans, curtains in a breeze and pets also count as motion.
- It detects **movement**. A person sitting perfectly still may read as Clear.

## Tuning (top of the sketch)

| Constant | Default | What it does |
|---|---|---|
| `SENSITIVITY` | 4.0 | Lower means more sensitive, higher means fewer false alarms |
| `MIN_THRESHOLD` | 0.6 | Lowest allowed threshold |
| `HOLD_MS` | 3000 | How long MOTION stays on after movement stops |
| `CALIBRATE_MS` | 10000 | Length of the calibration |
| `TRIGGER_BUCKETS` | 2 | Consecutive 250 ms windows above the threshold before triggering |

## Serial

115200 baud prints `score:<x> threshold:<y> motion:<0|1>`, so open **Tools → Serial Plotter** to watch it live.

## Build

The esp32 core 3.x (by Espressif) and the **Adafruit SSD1306** and **Adafruit GFX** libraries. Select the **Heltec WiFi LoRa 32(V3)** or **Heltec WiFi Kit 32(V3)** board.
