/*
  WiFi Signal Monitor - Heltec V3 (WiFi Kit 32 V3 / WiFi LoRa 32 V3)

  Scans nearby WiFi networks and shows their signal strength on the
  built-in 128x64 OLED. Pick one to track it live: big dBm readout,
  signal bars and a scrolling history graph. No password needed - the
  readings come from WiFi scans, so it works with any network in range.

  PRG button (GPIO0)
    Network list : tap  = move to the next network
                   hold = track the highlighted network
    Tracking     : tap  = switch between graph and stats
                   hold = back to the network list

  Serial (115200): while tracking, prints "RSSI:<dBm>" lines, so
  Tools > Serial Plotter draws a live graph as well.

  Board:     Heltec WiFi Kit 32(V3) or Heltec WiFi LoRa 32(V3), esp32 core 3.x
  Libraries: Adafruit SSD1306, Adafruit GFX
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Heltec V3 pins ----------
const uint8_t PIN_OLED_SDA = 17;
const uint8_t PIN_OLED_SCL = 18;
const uint8_t PIN_OLED_RST = 21;
const uint8_t PIN_VEXT     = 36;  // drive LOW to power the OLED
const uint8_t PIN_BUTTON   = 0;   // PRG button, active LOW
const uint8_t OLED_ADDR    = 0x3C;

// ---------- Tuning ----------
const uint32_t LIST_SCAN_MS_PER_CH  = 120;  // full scan takes ~13 channels x this
const uint32_t TRACK_SCAN_MS_PER_CH = 100;  // single-channel scan while tracking
const uint32_t TRACK_INTERVAL_MS    = 250;  // one RSSI sample every 250 ms
const uint32_t SCAN_RETRY_MS        = 500;  // back-off after a failed scan
const uint8_t  LOST_AFTER_MISSES    = 4;    // missed samples before showing LOST
const uint32_t LONG_PRESS_MS        = 600;
const uint32_t DEBOUNCE_MS          = 30;
const uint32_t REDRAW_MS            = 100;

const int SCREEN_W      = 128;
const int SCREEN_H      = 64;
const int MAX_NETS      = 32;
const int LIST_ROWS     = 5;
const int GRAPH_TOP     = 40;
const int GRAPH_H       = 24;
const int GRAPH_MIN_DBM = -100;
const int GRAPH_MAX_DBM = -30;
const int8_t NO_SAMPLE  = 0;  // marks a gap in the history graph

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, PIN_OLED_RST);

struct Net {
  char ssid[33];
  uint8_t bssid[6];
  int8_t rssi;
  uint8_t channel;
  wifi_auth_mode_t auth;
};

enum Mode { MODE_LIST, MODE_TRACK };
enum ScanKind { SCAN_NONE, SCAN_LIST, SCAN_TRACK };
enum Press { PRESS_NONE, PRESS_SHORT, PRESS_LONG };

Mode mode = MODE_LIST;
bool showStats = false;

ScanKind scanKind = SCAN_NONE;
uint32_t nextScanAt = 0;

// Network list, sorted strongest first
Net nets[MAX_NETS];
int netCount = 0;
int selected = 0;
int scrollTop = 0;
bool haveScan = false;

// Tracked network
Net target;
uint8_t misses = 0;
int8_t history[SCREEN_W];
int histPos = 0;  // next write index
int histCount = 0;
int8_t minRssi = 0, maxRssi = 0;
int32_t rssiSum = 0;
uint32_t sampleCount = 0;

// ---------- Signal helpers ----------

int barsFor(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

const char *labelFor(int rssi) {
  static const char *labels[] = {"Poor", "Weak", "Fair", "Good", "Great"};
  return labels[barsFor(rssi)];
}

int qualityFor(int rssi) {
  return constrain(2 * (rssi + 100), 0, 100);
}

const char *authName(wifi_auth_mode_t a) {
  switch (a) {
    case WIFI_AUTH_OPEN:            return "Open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Ent";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "Secured";
  }
}

bool isLost() {
  return misses >= LOST_AFTER_MISSES;
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

// 4-bar signal icon, bars bottom-aligned at yBottom.
void drawBars(int x, int yBottom, int filled, int barW, int gap, int maxH, uint16_t color) {
  for (int i = 0; i < 4; i++) {
    int h = maxH * (i + 1) / 4;
    int bx = x + i * (barW + gap);
    if (i < filled) display.fillRect(bx, yBottom - h + 1, barW, h, color);
    else if (barW > 2) display.drawRect(bx, yBottom - h + 1, barW, h, color);
    else display.drawFastHLine(bx, yBottom, barW, color);
  }
}

void drawHeader(const char *left, const char *right) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(left);
  printRight(SCREEN_W, 0, right);
  display.drawFastHLine(0, 9, SCREEN_W, SSD1306_WHITE);
}

int dbmToY(int dbm) {
  dbm = constrain(dbm, GRAPH_MIN_DBM, GRAPH_MAX_DBM);
  return GRAPH_TOP + GRAPH_H - 1 -
         (dbm - GRAPH_MIN_DBM) * (GRAPH_H - 1) / (GRAPH_MAX_DBM - GRAPH_MIN_DBM);
}

// ---------- Screens ----------

void drawSplash() {
  display.clearDisplay();
  drawHeader("WiFi Signal Monitor", "");
  display.setCursor(0, 16);
  display.print("PRG button:");
  display.setCursor(0, 28);
  display.print(" tap  = next / view");
  display.setCursor(0, 40);
  display.print(" hold = track / back");
  display.setCursor(0, 54);
  display.print("Scanning...");
  display.display();
}

void drawList() {
  static const char spinner[] = "|/-\\";
  char right[16];
  char spin = scanKind != SCAN_NONE ? spinner[(millis() / 150) % 4] : ' ';
  snprintf(right, sizeof(right), "%c %d found", spin, netCount);
  drawHeader("WiFi Scan", right);

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
      display.fillRect(0, y - 1, SCREEN_W - 1, 10, SSD1306_WHITE);
      fg = SSD1306_BLACK;
    }
    display.setTextColor(fg);
    display.setCursor(2, y);
    display.print(fit(n.ssid, 14));
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", n.rssi);
    printRight(114, y, buf);
    drawBars(116, y + 7, barsFor(n.rssi), 2, 1, 8, fg);
  }
  display.setTextColor(SSD1306_WHITE);

  // Scrollbar on the right edge when the list is longer than the screen
  if (netCount > LIST_ROWS) {
    int trackH = LIST_ROWS * 10;
    int thumbH = max(3, trackH * LIST_ROWS / netCount);
    int thumbY = 10 + (trackH - thumbH) * scrollTop / (netCount - LIST_ROWS);
    display.drawFastVLine(SCREEN_W - 1, thumbY, thumbH, SSD1306_WHITE);
  }
}

// RSSI history: oldest on the left, newest at the right edge.
void drawGraph() {
  // Dotted guide line at -70 dBm (roughly where streaming/calls get shaky)
  for (int x = 0; x < SCREEN_W; x += 4) display.drawPixel(x, dbmToY(-70), SSD1306_WHITE);

  int prevX = -1, prevY = 0;
  for (int k = 0; k < histCount; k++) {
    int idx = (histPos - histCount + k + SCREEN_W) % SCREEN_W;
    int x = SCREEN_W - histCount + k;
    if (history[idx] == NO_SAMPLE) {
      prevX = -1;
      continue;
    }
    int y = dbmToY(history[idx]);
    if (prevX >= 0) display.drawLine(prevX, prevY, x, y, SSD1306_WHITE);
    else display.drawPixel(x, y, SSD1306_WHITE);
    prevX = x;
    prevY = y;
  }
}

void drawLive() {
  char buf[12];
  if (isLost()) {
    display.setTextSize(2);
    display.setCursor(0, 12);
    display.print("LOST");
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.print("searching...");
    drawBars(100, 35, 0, 5, 2, 24, SSD1306_WHITE);
  } else {
    int r = target.rssi;
    snprintf(buf, sizeof(buf), "%d", r);
    display.setTextSize(strlen(buf) > 3 ? 2 : 3);
    display.setCursor(0, 12);
    display.print(buf);
    display.setTextSize(1);
    display.setCursor(58, 12);
    display.print("dBm");
    snprintf(buf, sizeof(buf), "%d%%", qualityFor(r));
    display.setCursor(58, 21);
    display.print(buf);
    display.setCursor(58, 30);
    display.print(labelFor(r));
    drawBars(100, 35, barsFor(r), 5, 2, 24, SSD1306_WHITE);
  }
  drawGraph();
}

void drawStats() {
  char buf[32];
  if (isLost()) snprintf(buf, sizeof(buf), "Now  lost");
  else snprintf(buf, sizeof(buf), "Now  %d dBm %s", target.rssi, labelFor(target.rssi));
  display.setCursor(0, 12);
  display.print(buf);

  if (sampleCount > 0) {
    snprintf(buf, sizeof(buf), "Min  %d  Max  %d", minRssi, maxRssi);
    display.setCursor(0, 22);
    display.print(buf);
    snprintf(buf, sizeof(buf), "Avg  %.1f  n=%lu", (float)rssiSum / sampleCount, (unsigned long)sampleCount);
    display.setCursor(0, 32);
    display.print(buf);
  }

  snprintf(buf, sizeof(buf), "Ch %d  %s", target.channel, authName(target.auth));
  display.setCursor(0, 42);
  display.print(buf);

  const uint8_t *b = target.bssid;
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", b[0], b[1], b[2], b[3], b[4], b[5]);
  display.setCursor(0, 52);
  display.print(buf);
}

void drawTrack() {
  char ch[8];
  snprintf(ch, sizeof(ch), "ch%d", target.channel);
  drawHeader(fit(target.ssid, 16), ch);
  if (showStats) drawStats();
  else drawLive();
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
    e.auth = WiFi.encryptionType(i);
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

  Serial.printf("\n%d networks:\n", netCount);
  for (int i = 0; i < netCount; i++) {
    const Net &e = nets[i];
    const uint8_t *b = e.bssid;
    Serial.printf("%4d dBm  ch%-2d  %02X:%02X:%02X:%02X:%02X:%02X  %-9s  %s\n", e.rssi, e.channel,
                  b[0], b[1], b[2], b[3], b[4], b[5], authName(e.auth), e.ssid[0] ? e.ssid : "<hidden>");
  }
}

// ---------- Tracking ----------

void addSample(int8_t rssi) {
  history[histPos] = rssi;
  histPos = (histPos + 1) % SCREEN_W;
  if (histCount < SCREEN_W) histCount++;
  if (rssi == NO_SAMPLE) return;
  if (sampleCount == 0 || rssi < minRssi) minRssi = rssi;
  if (sampleCount == 0 || rssi > maxRssi) maxRssi = rssi;
  rssiSum += rssi;
  sampleCount++;
}

void startTracking(const Net &n) {
  target = n;
  misses = 0;
  showStats = false;
  histPos = histCount = 0;
  rssiSum = 0;
  sampleCount = 0;
  addSample(n.rssi);
  mode = MODE_TRACK;
  nextScanAt = millis();

  const uint8_t *b = n.bssid;
  Serial.printf("\nTracking %s (%02X:%02X:%02X:%02X:%02X:%02X) on ch%d\n", n.ssid[0] ? n.ssid : "<hidden>",
                b[0], b[1], b[2], b[3], b[4], b[5], n.channel);
}

void handleTrackResults(int count) {
  uint8_t b[6];
  for (int i = 0; i < count; i++) {
    WiFi.BSSID(i, b);
    if (memcmp(b, target.bssid, 6) == 0) {
      target.rssi = WiFi.RSSI(i);
      target.channel = WiFi.channel(i);
      misses = 0;
      addSample(target.rssi);
      Serial.printf("RSSI:%d\n", target.rssi);
      return;
    }
  }
  if (misses < 255) misses++;
  if (isLost()) addSample(NO_SAMPLE);
}

// ---------- Scanning (async, so the button and screen stay responsive) ----------

void startScan() {
  int16_t r;
  if (mode == MODE_LIST) {
    r = WiFi.scanNetworks(true, true, false, LIST_SCAN_MS_PER_CH);
    scanKind = SCAN_LIST;
  } else {
    // Only the target's channel, filtered to its BSSID. Once it's lost,
    // sweep every channel in case the access point moved.
    uint8_t ch = isLost() ? 0 : target.channel;
    r = WiFi.scanNetworks(true, true, false, TRACK_SCAN_MS_PER_CH, ch, nullptr, target.bssid);
    scanKind = SCAN_TRACK;
    nextScanAt = millis() + TRACK_INTERVAL_MS;
  }
  if (r == WIFI_SCAN_FAILED) {
    scanKind = SCAN_NONE;
    nextScanAt = millis() + SCAN_RETRY_MS;
  }
}

void serviceScan() {
  if (scanKind != SCAN_NONE) {
    int16_t n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;
    // Results of a scan started before a mode switch are dropped
    if (n >= 0 && scanKind == SCAN_LIST && mode == MODE_LIST) handleListResults(n);
    if (n >= 0 && scanKind == SCAN_TRACK && mode == MODE_TRACK) handleTrackResults(n);
    if (n < 0) nextScanAt = millis() + SCAN_RETRY_MS;
    WiFi.scanDelete();
    scanKind = SCAN_NONE;
  }
  if ((int32_t)(millis() - nextScanAt) >= 0) startScan();
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
      startTracking(nets[selected]);
    }
  } else {
    if (p == PRESS_SHORT) {
      showStats = !showStats;
    } else {
      mode = MODE_LIST;
      nextScanAt = millis();
    }
  }
  return true;
}

// ---------- Main ----------

void setup() {
  Serial.setTxBufferSize(4096);  // scan dumps shouldn't block the UI
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  delay(50);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) {
    Serial.println("SSD1306 init failed");
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  startScan();  // runs in the background while the splash is up
  drawSplash();
  delay(1500);
}

void loop() {
  static uint32_t lastDraw = 0;
  bool changed = handleButton(readButton());
  serviceScan();

  if (changed || millis() - lastDraw >= REDRAW_MS) {
    lastDraw = millis();
    display.clearDisplay();
    if (mode == MODE_LIST) drawList();
    else drawTrack();
    display.display();
  }
}
