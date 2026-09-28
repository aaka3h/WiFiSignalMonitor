/*
  WiFi Motion Detector - Heltec V3 (WiFi Kit 32 V3 / WiFi LoRa 32 V3)

  Detects people moving around using only the board's WiFi radio - no
  RCWL-0516, PIR or any other sensor. The board listens to a nearby access
  point (your router) and watches how its signal changes. A still room
  gives a steady signal; a person walking through it bends and blocks the
  radio waves, and the signal starts to wobble.

  It uses WiFi CSI (Channel State Information: amplitude of ~50 separate
  subcarriers per packet), which is far more sensitive than plain RSSI.
  If CSI isn't available in your core, it falls back to RSSI automatically.
  No WiFi password is needed - it only listens to the router's beacons.

  PRG button (GPIO0)
    Network list : tap  = move to the next network
                   hold = use the highlighted network
    Monitoring   : tap  = recalibrate
                   hold = back to the network list

  Best results: put the board 2-5 m from the router with the area you
  want to watch between them. Keep still (or leave the room) during the
  10 s calibration.

  Serial (115200): prints "score:<x> threshold:<y> motion:<0|1>" lines
  for Tools > Serial Plotter.

  Board:     Heltec WiFi Kit 32(V3) or Heltec WiFi LoRa 32(V3), esp32 core 3.x
  Libraries: Adafruit SSD1306, Adafruit GFX
*/

#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Heltec V3 pins ----------
const uint8_t PIN_OLED_SDA = 17;
const uint8_t PIN_OLED_SCL = 18;
const uint8_t PIN_OLED_RST = 21;
const uint8_t PIN_VEXT     = 36;  // drive LOW to power the OLED
const uint8_t PIN_BUTTON   = 0;   // PRG button, active LOW
const uint8_t PIN_LED      = 35;  // white onboard LED, lights on motion
const uint8_t OLED_ADDR    = 0x3C;

// ---------- Tuning ----------
const uint32_t BUCKET_MS       = 250;    // packets are averaged into 250 ms buckets
const int      SCORE_BUCKETS   = 4;      // score = average of the last 4 buckets (1 s)
const uint32_t CALIBRATE_MS    = 10000;  // quiet-room learning time
const float    SENSITIVITY     = 4.0;    // threshold = mean + SENSITIVITY x std dev (lower = more sensitive)
const float    MIN_THRESHOLD   = 0.6;    // floor so a perfectly quiet room doesn't trigger on noise
const int      TRIGGER_BUCKETS = 2;      // buckets above threshold before MOTION
const uint32_t HOLD_MS         = 3000;   // MOTION stays on this long after the last movement
const uint32_t NO_SIGNAL_MS    = 3000;   // no packets for this long = "No signal"
const uint32_t LIST_SCAN_MS_PER_CH = 120;
const uint32_t LONG_PRESS_MS   = 600;
const uint32_t DEBOUNCE_MS     = 30;
const uint32_t REDRAW_MS       = 100;

// CSI subcarriers used (LLTF, HT20). Buffer order is 0..31 then -32..-1;
// 0, 27..37 are DC/guard carriers, and the ones next to them are noisy.
const int SC_LO_START = 2, SC_LO_END = 26;   // subcarriers +2..+26
const int SC_HI_START = 38, SC_HI_END = 62;  // subcarriers -26..-2
const int MAX_SC = 64;

const int SCREEN_W  = 128;
const int SCREEN_H  = 64;
const int MAX_NETS  = 32;
const int LIST_ROWS = 5;
const int GRAPH_TOP = 40;
const int GRAPH_H   = 24;

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, PIN_OLED_RST);

struct Net {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
};

// One packet's worth of "how much did the signal change since the last one"
struct Sample {
  float change;
  int8_t rssi;
};

enum Mode { MODE_LIST, MODE_CALIBRATE, MODE_MONITOR };
enum Press { PRESS_NONE, PRESS_SHORT, PRESS_LONG };

Mode mode = MODE_LIST;

// Network list, sorted strongest first
Net nets[MAX_NETS];
int netCount = 0;
int selected = 0;
int scrollTop = 0;
bool haveScan = false;
bool scanning = false;

// Target access point, shared with the WiFi callbacks
Net target;
volatile bool useCsi = false;
volatile bool resetCsiPrev = true, resetRssiPrev = true;
QueueHandle_t sampleQueue;

// Bucketing and scoring (main loop only)
float bucketSum = 0;
uint32_t bucketCount = 0;
uint32_t bucketStart = 0;
uint32_t packetsThisSec = 0, packetRate = 0, rateStart = 0;
uint32_t lastPacketAt = 0;
int8_t lastRssi = 0;
float recent[SCORE_BUCKETS];
int recentCount = 0, recentPos = 0;
float score = 0;

// Calibration
uint32_t calibrateStart = 0;
float calSum = 0, calSqSum = 0;
int calCount = 0;
float baseMean = 0, threshold = 1;

// Detection
bool motion = false;
int aboveCount = 0;
uint32_t lastAboveAt = 0;
uint32_t motionEvents = 0;

float history[SCREEN_W];  // score per bucket, NAN = no data
int histPos = 0, histCount = 0;

// ---------- WiFi callbacks (run in the WiFi task, keep them short) ----------

// CSI: compares the shape of the subcarrier amplitudes with the previous
// packet. Amplitudes are normalised by their mean so AGC gain steps and
// distance don't matter, only how the multipath pattern changes.
void onCsi(void *ctx, wifi_csi_info_t *info) {
  static float prev[MAX_SC];
  static bool havePrev = false;
  if (!info || !info->buf || info->len < 128) return;
  if (memcmp(info->mac, target.bssid, 6) != 0) return;

  float amp[MAX_SC];
  float sum = 0;
  int n = 0;
  for (int sc = 0; sc < MAX_SC; sc++) {
    bool used = (sc >= SC_LO_START && sc <= SC_LO_END) || (sc >= SC_HI_START && sc <= SC_HI_END);
    if (!used) continue;
    float im = info->buf[sc * 2], re = info->buf[sc * 2 + 1];
    amp[sc] = sqrtf(re * re + im * im);
    sum += amp[sc];
    n++;
  }
  if (sum <= 0) return;
  float mean = sum / n;

  if (resetCsiPrev) {
    havePrev = false;
    resetCsiPrev = false;
  }
  float diff = 0;
  for (int sc = 0; sc < MAX_SC; sc++) {
    bool used = (sc >= SC_LO_START && sc <= SC_LO_END) || (sc >= SC_HI_START && sc <= SC_HI_END);
    if (!used) continue;
    float a = amp[sc] / mean;
    diff += fabsf(a - prev[sc]);
    prev[sc] = a;
  }
  if (!havePrev) {
    havePrev = true;
    return;
  }
  Sample s = {diff / n * 100.0f, (int8_t)info->rx_ctrl.rssi};
  xQueueSend(sampleQueue, &s, 0);
}

// RSSI fallback: change in received signal strength between packets.
void onPromiscuous(void *buf, wifi_promiscuous_pkt_type_t type) {
  static int8_t prevRssi = 0;
  static bool havePrev = false;
  if (useCsi) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  if (p->rx_ctrl.sig_len < 24) return;
  if (memcmp(p->payload + 10, target.bssid, 6) != 0) return;  // addr2 = transmitter

  int8_t rssi = p->rx_ctrl.rssi;
  if (resetRssiPrev) {
    havePrev = false;
    resetRssiPrev = false;
  }
  if (!havePrev) {
    havePrev = true;
    prevRssi = rssi;
    return;
  }
  Sample s = {(float)abs(rssi - prevRssi), rssi};
  prevRssi = rssi;
  xQueueSend(sampleQueue, &s, 0);
}

// ---------- Sensing start / stop ----------

void startSensing() {
  WiFi.scanDelete();
  xQueueReset(sampleQueue);
  resetCsiPrev = true;
  resetRssiPrev = true;

  esp_wifi_set_promiscuous_rx_cb(onPromiscuous);
  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(target.channel, WIFI_SECOND_CHAN_NONE);

  wifi_csi_config_t cfg = {};
  cfg.lltf_en = true;
  cfg.htltf_en = false;
  cfg.stbc_htltf2_en = false;
  cfg.ltf_merge_en = false;
  cfg.channel_filter_en = false;
  cfg.manu_scale = false;
  cfg.shift = 0;
  bool ok = esp_wifi_set_csi_config(&cfg) == ESP_OK &&
            esp_wifi_set_csi_rx_cb(onCsi, nullptr) == ESP_OK &&
            esp_wifi_set_csi(true) == ESP_OK;
  useCsi = ok;

  const uint8_t *b = target.bssid;
  Serial.printf("\nWatching %s (%02X:%02X:%02X:%02X:%02X:%02X) ch%d using %s\n",
                target.ssid[0] ? target.ssid : "<hidden>", b[0], b[1], b[2], b[3], b[4], b[5],
                target.channel, ok ? "CSI" : "RSSI (CSI not available)");
}

void stopSensing() {
  esp_wifi_set_csi(false);
  esp_wifi_set_promiscuous(false);
  digitalWrite(PIN_LED, LOW);
}

// ---------- Detection ----------

void resetScoring() {
  bucketSum = 0;
  bucketCount = 0;
  bucketStart = millis();
  recentCount = recentPos = 0;
  score = 0;
  histPos = histCount = 0;
  motion = false;
  aboveCount = 0;
}

void startCalibration() {
  mode = MODE_CALIBRATE;
  calibrateStart = millis();
  calSum = calSqSum = 0;
  calCount = 0;
  resetScoring();
  digitalWrite(PIN_LED, LOW);
  Serial.println("Calibrating - keep still...");
}

void finishCalibration() {
  if (calCount < 4) {  // barely any packets: try again
    startCalibration();
    return;
  }
  baseMean = calSum / calCount;
  float var = calSqSum / calCount - baseMean * baseMean;
  float sd = sqrtf(max(var, 0.0f));
  threshold = max(max(baseMean + SENSITIVITY * sd, baseMean * 1.8f), MIN_THRESHOLD);
  mode = MODE_MONITOR;
  motionEvents = 0;
  Serial.printf("Calibrated: quiet level %.2f, sd %.2f, threshold %.2f\n", baseMean, sd, threshold);
}

void addHistory(float v) {
  history[histPos] = v;
  histPos = (histPos + 1) % SCREEN_W;
  if (histCount < SCREEN_W) histCount++;
}

void updateMotion(uint32_t now) {
  if (score > threshold) {
    lastAboveAt = now;
    if (++aboveCount >= TRIGGER_BUCKETS && !motion) {
      motion = true;
      motionEvents++;
      Serial.println("MOTION detected");
    }
  } else {
    aboveCount = 0;
    if (motion && now - lastAboveAt >= HOLD_MS) {
      motion = false;
      Serial.println("Clear");
    }
  }
  digitalWrite(PIN_LED, motion ? HIGH : LOW);
}

// Called every BUCKET_MS with the average change of the packets in it.
void onBucket(uint32_t now) {
  if (bucketCount == 0) {
    addHistory(NAN);
    return;
  }
  recent[recentPos] = bucketSum / bucketCount;
  recentPos = (recentPos + 1) % SCORE_BUCKETS;
  if (recentCount < SCORE_BUCKETS) recentCount++;
  float s = 0;
  for (int i = 0; i < recentCount; i++) s += recent[i];
  score = s / recentCount;
  addHistory(score);

  if (mode == MODE_CALIBRATE && recentCount == SCORE_BUCKETS) {
    calSum += score;
    calSqSum += score * score;
    calCount++;
  } else if (mode == MODE_MONITOR) {
    updateMotion(now);
    Serial.printf("score:%.2f threshold:%.2f motion:%d\n", score, threshold, motion ? 1 : 0);
  }
}

void serviceSensing() {
  Sample s;
  uint32_t now = millis();
  while (xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
    bucketSum += s.change;
    bucketCount++;
    packetsThisSec++;
    lastPacketAt = now;
    lastRssi = s.rssi;
  }
  if (now - rateStart >= 1000) {
    packetRate = packetsThisSec;
    packetsThisSec = 0;
    rateStart = now;
  }
  while (now - bucketStart >= BUCKET_MS) {
    onBucket(now);
    bucketSum = 0;
    bucketCount = 0;
    bucketStart += BUCKET_MS;
  }
  if (mode == MODE_CALIBRATE && now - calibrateStart >= CALIBRATE_MS) finishCalibration();
}

bool noSignal() {
  return millis() - lastPacketAt >= NO_SIGNAL_MS;
}

// ---------- Drawing helpers ----------

// Returns s shortened to maxChars (ending in '~' if cut); "<hidden>" if empty.
const char *fit(const char *s, int maxChars) {
  static char buf[34];
  if (!*s) s = "<hidden>";
  if ((int)strlen(s) <= maxChars) return s;
  memcpy(buf, s, maxChars - 1);
  buf[maxChars - 1] = '~';
  buf[maxChars] = '\0';
  return buf;
}

// Size-1 text whose last character ends at xEnd (default font is 6 px wide).
void printRight(int xEnd, int y, const char *s) {
  display.setCursor(xEnd - (int)strlen(s) * 6, y);
  display.print(s);
}

void drawHeader(const char *left, const char *right) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(left);
  printRight(SCREEN_W, 0, right);
  display.drawFastHLine(0, 9, SCREEN_W, SSD1306_WHITE);
}

// Score history, oldest on the left; dotted line = threshold.
void drawGraph(float scaleMax) {
  int thrY = GRAPH_TOP + GRAPH_H - 1 - (int)(threshold * (GRAPH_H - 1) / scaleMax);
  for (int x = 0; x < SCREEN_W; x += 4) display.drawPixel(x, thrY, SSD1306_WHITE);

  int prevX = -1, prevY = 0;
  for (int k = 0; k < histCount; k++) {
    int idx = (histPos - histCount + k + SCREEN_W) % SCREEN_W;
    int x = SCREEN_W - histCount + k;
    if (isnan(history[idx])) {
      prevX = -1;
      continue;
    }
    float v = min(history[idx], scaleMax);
    int y = GRAPH_TOP + GRAPH_H - 1 - (int)(v * (GRAPH_H - 1) / scaleMax);
    if (prevX >= 0) display.drawLine(prevX, prevY, x, y, SSD1306_WHITE);
    else display.drawPixel(x, y, SSD1306_WHITE);
    prevX = x;
    prevY = y;
  }
}

// ---------- Screens ----------

void drawSplash() {
  display.clearDisplay();
  drawHeader("WiFi Motion Detector", "");
  display.setCursor(0, 14);
  display.print("Pick your router,");
  display.setCursor(0, 24);
  display.print("then keep still for");
  display.setCursor(0, 34);
  display.print("10 s to calibrate.");
  display.setCursor(0, 50);
  display.print("Scanning...");
  display.display();
}

void drawList() {
  static const char spinner[] = "|/-\\";
  char right[16];
  char spin = scanning ? spinner[(millis() / 150) % 4] : ' ';
  snprintf(right, sizeof(right), "%c %d", spin, netCount);
  drawHeader("Pick router", right);

  if (netCount == 0) {
    display.setCursor(0, 30);
    display.print(haveScan ? "No networks found" : "Scanning...");
    return;
  }

  for (int r = 0; r < LIST_ROWS && scrollTop + r < netCount; r++) {
    int i = scrollTop + r;
    const Net &n = nets[i];
    int y = 11 + r * 10;
    uint16_t fg = SSD1306_WHITE;
    if (i == selected) {
      display.fillRect(0, y - 1, SCREEN_W, 10, SSD1306_WHITE);
      fg = SSD1306_BLACK;
    }
    display.setTextColor(fg);
    display.setCursor(2, y);
    display.print(fit(n.ssid, 15));
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", n.rssi);
    printRight(SCREEN_W - 2, y, buf);
  }
  display.setTextColor(SSD1306_WHITE);
}

void drawCalibrate() {
  char buf[24];
  drawHeader("Calibrating", useCsi ? "CSI" : "RSSI");
  display.setCursor(0, 14);
  display.print("Keep still, or leave");
  display.setCursor(0, 24);
  display.print("the room...");

  uint32_t elapsed = min(millis() - calibrateStart, CALIBRATE_MS);
  display.drawRect(0, 38, SCREEN_W, 8, SSD1306_WHITE);
  display.fillRect(2, 40, (SCREEN_W - 4) * elapsed / CALIBRATE_MS, 4, SSD1306_WHITE);

  if (noSignal()) snprintf(buf, sizeof(buf), "No packets from AP!");
  else snprintf(buf, sizeof(buf), "%lus left  %lu pkt/s", (unsigned long)((CALIBRATE_MS - elapsed + 999) / 1000),
                (unsigned long)packetRate);
  display.setCursor(0, 52);
  display.print(buf);
}

void drawMonitor() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%s %lu/s", useCsi ? "CSI" : "RSSI", (unsigned long)packetRate);
  drawHeader(fit(target.ssid, 10), buf);

  display.setTextSize(2);
  if (noSignal()) {
    display.setCursor(0, 13);
    display.print("NO SIGNAL");
  } else if (motion) {
    display.fillRect(0, 11, 76, 18, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(3, 13);
    display.print("MOTION");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(3, 13);
    display.print("Clear");
  }
  display.setTextSize(1);
  if (!noSignal()) {
    snprintf(buf, sizeof(buf), "x%.1f", score / threshold);  // how far over/under the threshold
    printRight(SCREEN_W, 12, buf);
    snprintf(buf, sizeof(buf), "n=%lu", (unsigned long)motionEvents);
    printRight(SCREEN_W, 21, buf);
  }

  // Level bar: threshold sits in the middle
  float scaleMax = threshold * 2;
  for (int k = 0; k < histCount; k++) {
    float v = history[(histPos - histCount + k + SCREEN_W) % SCREEN_W];
    if (!isnan(v) && v > scaleMax) scaleMax = v;
  }
  int barW = (int)(min(score / (threshold * 2), 1.0f) * (SCREEN_W - 1));
  display.drawRect(0, 31, SCREEN_W, 6, SSD1306_WHITE);
  display.fillRect(0, 31, barW, 6, SSD1306_WHITE);
  display.drawFastVLine(SCREEN_W / 2, 29, 10, SSD1306_WHITE);

  drawGraph(scaleMax);
}

// ---------- Network list ----------

void clampScroll() {
  if (selected < scrollTop) scrollTop = selected;
  if (selected >= scrollTop + LIST_ROWS) scrollTop = selected - LIST_ROWS + 1;
  scrollTop = constrain(scrollTop, 0, max(netCount - LIST_ROWS, 0));
}

// Inserts into nets[] keeping it sorted strongest first; drops the weakest when full.
void insertNet(const Net &n) {
  if (netCount == MAX_NETS) {
    if (n.rssi <= nets[MAX_NETS - 1].rssi) return;
    netCount--;
  }
  int i = netCount++;
  while (i > 0 && nets[i - 1].rssi < n.rssi) {
    nets[i] = nets[i - 1];
    i--;
  }
  nets[i] = n;
}

void handleListResults(int count) {
  uint8_t selBssid[6];
  bool hadSel = netCount > 0;
  if (hadSel) memcpy(selBssid, nets[selected].bssid, 6);

  netCount = 0;
  for (int i = 0; i < count; i++) {
    Net e;
    strlcpy(e.ssid, WiFi.SSID(i).c_str(), sizeof(e.ssid));
    WiFi.BSSID(i, e.bssid);
    e.rssi = WiFi.RSSI(i);
    e.channel = WiFi.channel(i);
    insertNet(e);
  }
  haveScan = true;

  // Keep the highlight on the same network after re-sorting
  int keep = min(selected, max(netCount - 1, 0));
  if (hadSel) {
    for (int i = 0; i < netCount; i++) {
      if (memcmp(nets[i].bssid, selBssid, 6) == 0) {
        keep = i;
        break;
      }
    }
  }
  selected = keep;
  clampScroll();
}

void serviceScan() {
  if (mode != MODE_LIST) return;
  if (scanning) {
    int16_t n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;
    if (n >= 0) handleListResults(n);
    WiFi.scanDelete();
    scanning = false;
  }
  scanning = WiFi.scanNetworks(true, true, false, LIST_SCAN_MS_PER_CH) != WIFI_SCAN_FAILED;
}

// Waits for any running scan so the radio can switch to listening mode.
void finishScan() {
  while (scanning && WiFi.scanComplete() == WIFI_SCAN_RUNNING) delay(10);
  WiFi.scanDelete();
  scanning = false;
}

// ---------- Button ----------

Press readButton() {
  static bool down = false, longSent = false;
  static uint32_t changedAt = 0, downAt = 0;
  uint32_t now = millis();
  bool pressed = digitalRead(PIN_BUTTON) == LOW;

  if (pressed != down && now - changedAt >= DEBOUNCE_MS) {
    down = pressed;
    changedAt = now;
    if (down) {
      downAt = now;
      longSent = false;
    } else if (!longSent) {
      return PRESS_SHORT;
    }
  }
  if (down && !longSent && now - downAt >= LONG_PRESS_MS) {
    longSent = true;
    return PRESS_LONG;
  }
  return PRESS_NONE;
}

// Returns true if the press changed something on screen.
bool handleButton(Press p) {
  if (p == PRESS_NONE) return false;
  if (mode == MODE_LIST) {
    if (netCount == 0) return false;
    if (p == PRESS_SHORT) {
      selected = (selected + 1) % netCount;
      clampScroll();
    } else {
      finishScan();
      target = nets[selected];
      lastPacketAt = millis();
      startSensing();
      startCalibration();
    }
  } else {
    if (p == PRESS_SHORT) {
      startCalibration();
    } else {
      stopSensing();
      mode = MODE_LIST;
    }
  }
  return true;
}

// ---------- Main ----------

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(50);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) {
    Serial.println("SSD1306 init failed");
  }

  sampleQueue = xQueueCreate(128, sizeof(Sample));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  serviceScan();  // runs in the background while the splash is up
  drawSplash();
  delay(1500);
}

void loop() {
  static uint32_t lastDraw = 0;
  bool changed = handleButton(readButton());
  serviceScan();
  if (mode != MODE_LIST) serviceSensing();

  if (changed || millis() - lastDraw >= REDRAW_MS) {
    lastDraw = millis();
    display.clearDisplay();
    if (mode == MODE_LIST) drawList();
    else if (mode == MODE_CALIBRATE) drawCalibrate();
    else drawMonitor();
    display.display();
  }
}
