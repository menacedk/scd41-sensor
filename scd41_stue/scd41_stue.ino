/*
 * SCD41 CO2 sensor on Wemos D1 mini (Pro) (ESP8266)
 * -------------------------------------------------
 *  - Up to 3 WiFi profiles, each with ITS OWN MQTT broker
 *  - WiFiMulti: automatically connects to the strongest saved network
 *  - WiFiManager portal for adding networks (no reflash needed)
 *  - WiFi lost for more than WIFI_LOST_REBOOT_MS: reboot -> AP mode (portal)
 *  - mDNS:  http://scd41-stue.local
 *  - Live web server, NTP time, forced calibration (400 ppm)
 *  - Power-on timestamp + uptime, 1 h / 24 h averages on the web page
 *  - Status LED:  even fast blink in AP mode, slower blink when WiFi is up
 *                 but MQTT is not (only if the profile has an MQTT host),
 *                 very slow blink while searching for WiFi,
 *                 solid on at CO2>=1300 (off again below 1200)
 *  - MQTT (manual topics read by HA; not auto-discovery)
 *  - SGP41: VOC and NOx index (1-500), temp/humidity compensated from SCD41
 *
 * Wiring (I2C, shared bus):  VDD->3V3 | GND->G | SCL->D1 | SDA->D2
 *   (SCD41 = addr 0x62, SGP41 = addr 0x59 — no conflict, wired in parallel)
 * Status LED: the built-in LED (D4/GPIO2) — nothing extra to connect.
 *
 * Libraries (Arduino Library Manager):
 *   - "Sensirion I2C SCD4x" (+ "Sensirion Core")
 *   - "Sensirion I2C SGP41"
 *   - "Sensirion Gas Index Algorithm"
 *   - "WiFiManager" (by tzapu)
 *   - "PubSubClient" (by Nick O'Leary)   // only if ENABLE_MQTT
 *   (ESP8266WiFiMulti, LittleFS, mDNS, Ticker come with the ESP8266 board package)
 *
 * Board: "LOLIN(WEMOS) D1 mini Pro"
 *
 * USAGE:
 *   - Add network:  open http://scd41-stue.local -> "Settings",
 *                   or connect to the AP "SCD41-Setup" (192.168.4.1).
 *                   Enter WiFi + the MQTT broker that belongs to THAT place. Save.
 *   - Move places:  the device picks the saved network that is in range.
 *   - No network:   after WIFI_LOST_REBOOT_MS without WiFi it reboots and
 *                   opens "SCD41-Setup". The portal closes after 3 min and
 *                   the device tries the saved networks again (and so on).
 *   - Reset all:    /reset (forgets all profiles).
 */

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>
#include <LittleFS.h>
#include <coredecls.h>          // settimeofday_cb
#include <time.h>
#include <Ticker.h>
#include <Wire.h>
#include <SensirionI2cScd4x.h>   // latest Sensirion library (1.x)
#include <SensirionI2CSgp41.h>           // VOC/NOx raw signals
#include <SensirionGasIndexAlgorithm.h>  // converts raw -> index 1-500

// ====================== FIXED SETTINGS ======================
#define ENABLE_MQTT 1
#define MAX_PROFILES 3

const char* DEVICE_NAME = "Living room CO2";
const char* DEVICE_ID   = "scd41_stue";
const char* MDNS_HOST   = "scd41-stue";
const char* AP_NAME     = "SCD41-Setup";
const char* MQTT_BASE   = "home/scd41_1";   // topic base HA already reads from

const unsigned long PUBLISH_INTERVAL    = 5000;
const unsigned long WIFI_LOST_REBOOT_MS = 60000;   // this long without WiFi -> reboot into AP mode
const unsigned long SENSOR_STALE_MS     = 60000;   // no valid SCD41 reading this long -> g_valid=false
const uint16_t      CALIBRATION_PPM     = 400;
const char*         NTP_SERVER          = "pool.ntp.org";

// Averages: ring of 10-minute buckets. 6 buckets = 1 h, 144 buckets = 24 h.
// Kept in RAM only, so they start over after a reboot.
const unsigned long AVG_BUCKET_MS  = 600000UL;
const uint8_t       AVG_BUCKETS    = 144;
const uint8_t       AVG_1H_BUCKETS = 6;

// Built-in blue LED on D1 mini (Pro) = D4/GPIO2, active LOW.
// External LED on D5 instead: LED_PIN = D5, LED_ON = HIGH, LED_OFF = LOW.
const uint8_t  LED_PIN           = LED_BUILTIN;   // D4 / GPIO2
const uint8_t  LED_ON            = LOW;           // active LOW
const uint8_t  LED_OFF           = HIGH;
const uint16_t CO2_LED_ON        = 1300;   // solid on at/above this CO2
const uint16_t CO2_LED_OFF       = 1200;   // off again when CO2 drops below this (hysteresis)
// ============================================================

// One profile = one WiFi + its MQTT broker
struct Profile {
  char ssid[33];
  char pass[65];
  char mqttHost[40];
  char mqttPort[6];
  char mqttUser[24];
  char mqttPass[24];
  char mqttBase[40];   // MQTT topic/prefix, e.g. "home/scd41_bedroom". Empty = use MQTT_BASE.
};
Profile profiles[MAX_PROFILES];
int     profileCount  = 0;
int     activeProfile = -1;     // which profile we are connected to

// Types must come before the first function so Arduino's auto-generated
// prototypes (placed at the top) know them. The patterns are defined further down.
enum LedMode { LED_MODE_OFF, LED_MODE_SOLID, LED_MODE_AP, LED_MODE_NOWIFI, LED_MODE_NOMQTT };
struct LedSeg { bool on; uint16_t ms; };
struct AvgBucket { float co2, temp, hum, voc, nox; uint16_t nClimate, nVoc, nNox; };
struct Avg { float co2, temp, hum, voc, nox; bool hasClimate, hasVoc, hasNox; };

#if ENABLE_MQTT
#include <PubSubClient.h>
WiFiClient   espClient;
PubSubClient mqtt(espClient);
#endif

ESP8266WiFiMulti  wifiMulti;
ESP8266WebServer  server(80);
SensirionI2cScd4x scd4x;
SensirionI2CSgp41 sgp41;
GasIndexAlgorithmParams vocParams;   // C API (the GasIndexAlgorithm class doesn't exist in this lib version)
GasIndexAlgorithmParams noxParams;
Ticker            ledTicker;

// --- SGP41 state ---
// Index 1-500 (100 = normal baseline for your home). -1 = not ready yet.
int32_t  g_voc = -1, g_nox = -1;
uint16_t g_sgpRawVoc = 0, g_sgpRawNox = 0;
bool     g_sgpValid = false;             // has the sensor answered at least once
unsigned long g_lastSgpTick = 0;         // 1 Hz tick for the gas algorithm
unsigned long g_sgpStartMs  = 0;         // when conditioning started
const unsigned long SGP_CONDITION_MS = 10000;  // ~10 s NOx warm-up

uint16_t g_co2 = 0; float g_temp = 0, g_hum = 0; bool g_valid = false;
unsigned long g_lastValidMs = 0;  // millis() of the latest valid SCD41 reading
unsigned long g_lastPublish = 0;
time_t g_lastMeasurement = 0, g_lastNtpSync = 0, g_lastCalibration = 0;
time_t g_powerOn = 0;           // power-on time (UTC), back-calculated at first NTP sync
uint16_t g_frcRaw = 0;          // latest FRC correction (raw value, as HA expects)
bool g_requestPortal = false;   // set by /addwifi, handled in loop
bool g_internet = false;        // last internet check
unsigned long g_lastNetCheck = 0;
unsigned long g_lastMqttTry = 0;  // backoff on MQTT reconnect (avoid spam)
bool g_co2Alarm = false;        // CO2 alarm (hysteresis between CO2_LED_OFF and CO2_LED_ON)
bool g_wifiLost = false;        // is WiFi currently gone?
unsigned long g_wifiLostMs = 0; // when it went away

AvgBucket g_avg[AVG_BUCKETS];   // zero-initialised (global)
uint32_t  g_avgBucketId = 0;    // millis()/AVG_BUCKET_MS of the current bucket

// Active internet check: TCP out to Cloudflare (tests real internet access, not just LAN)
void checkInternet() {
  WiFiClient c;
  c.setTimeout(2000);
  g_internet = c.connect(IPAddress(1, 1, 1, 1), 80);
  c.stop();
}

// ---------- Escaping (SSIDs can contain ' " & # spaces etc.) ----------
String htmlEsc(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&':  o += F("&amp;");  break;
      case '<':  o += F("&lt;");   break;
      case '>':  o += F("&gt;");   break;
      case '"':  o += F("&quot;"); break;
      case '\'': o += F("&#39;");  break;
      default:   o += c;
    }
  }
  return o;
}
String urlEnc(const String& s) {
  const char* hex = "0123456789ABCDEF";
  String o; o.reserve(s.length() * 3);
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') o += (char)c;
    else { o += '%'; o += hex[c >> 4]; o += hex[c & 0xF]; }
  }
  return o;
}
String jsonEsc(const String& s) {
  String o; o.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c >= 0x20) o += c;   // drop control characters
  }
  return o;
}

// ---------- Averages (ring of 10-minute buckets) ----------
// Move to the current bucket; buckets we skipped over are cleared.
void avgAdvance() {
  uint32_t id = millis() / AVG_BUCKET_MS;
  if (id == g_avgBucketId) return;
  uint32_t steps = id - g_avgBucketId;      // millis() wrap (49 days) -> huge -> clear all
  if (steps > AVG_BUCKETS) steps = AVG_BUCKETS;
  for (uint32_t i = 1; i <= steps; i++)
    memset(&g_avg[(g_avgBucketId + i) % AVG_BUCKETS], 0, sizeof(AvgBucket));
  g_avgBucketId = id;
}
AvgBucket& avgCur() { avgAdvance(); return g_avg[g_avgBucketId % AVG_BUCKETS]; }

void avgAddClimate(uint16_t co2, float t, float h) {
  AvgBucket& b = avgCur();
  b.co2 += co2; b.temp += t; b.hum += h; b.nClimate++;
}
// Index 0 = the algorithm's start-up blackout (~45 s) -> not counted
void avgAddGas(int32_t voc, int32_t nox) {
  AvgBucket& b = avgCur();
  if (voc > 0) { b.voc += voc; b.nVoc++; }
  if (nox > 0) { b.nox += nox; b.nNox++; }
}

// Average over the latest n buckets (the current one included)
Avg avgOver(uint8_t n) {
  avgAdvance();
  float co2 = 0, t = 0, h = 0, voc = 0, nox = 0;
  uint32_t nc = 0, nv = 0, nn = 0;
  for (uint8_t i = 0; i < n && i < AVG_BUCKETS; i++) {
    const AvgBucket& b = g_avg[(g_avgBucketId + AVG_BUCKETS - i) % AVG_BUCKETS];
    co2 += b.co2; t += b.temp; h += b.hum; nc += b.nClimate;
    voc += b.voc; nv += b.nVoc;
    nox += b.nox; nn += b.nNox;
  }
  Avg a;
  a.hasClimate = nc > 0; a.hasVoc = nv > 0; a.hasNox = nn > 0;
  a.co2  = nc ? co2 / nc : 0;  a.temp = nc ? t / nc : 0;  a.hum = nc ? h / nc : 0;
  a.voc  = nv ? voc / nv : 0;  a.nox  = nn ? nox / nn : 0;
  return a;
}

String jsonNum(float v, bool ok, int dec) { return ok ? String(v, dec) : String("null"); }
String avgJson(const Avg& a) {
  return String("{\"co2\":") + jsonNum(a.co2, a.hasClimate, 0) +
    ",\"temperature\":" + jsonNum(a.temp, a.hasClimate, 2) +
    ",\"humidity\":" + jsonNum(a.hum, a.hasClimate, 2) +
    ",\"voc_index\":" + jsonNum(a.voc, a.hasVoc, 0) +
    ",\"nox_index\":" + jsonNum(a.nox, a.hasNox, 0) + "}";
}

// ---------- Status LED (pattern player) ----------
// A pattern = a sequence of segments {on?, duration ms}. It is played by a
// Ticker that re-arms itself (once_ms), so it also runs while the AP portal
// blocks in runPortal() and while connectWiFi() waits at boot.
// Change pattern with setLedMode().
// (LedMode/LedSeg are defined at the top so the Arduino auto-prototypes know them.)

// AP mode:            --blink--blink--blink--  (even, fast)
const LedSeg PAT_AP[]     = {{true,150},{false,300}};
// WiFi yes, no MQTT:  ----blink----blink----   (twice the gap)
const LedSeg PAT_NOMQTT[] = {{true,150},{false,600}};
// Searching for WiFi: very slow single blink
const LedSeg PAT_NOWIFI[] = {{true,150},{false,1500}};

LedMode       g_ledMode = LED_MODE_OFF;
const LedSeg* g_pat     = nullptr;
uint8_t       g_patLen  = 0, g_patIdx = 0;

void ledStep();   // forward

void ledArm() {
  if (!g_pat || g_patLen == 0) return;
  const LedSeg& s = g_pat[g_patIdx];
  digitalWrite(LED_PIN, s.on ? LED_ON : LED_OFF);
  ledTicker.once_ms(s.ms, ledStep);
}
void ledStep() { g_patIdx = (g_patIdx + 1) % g_patLen; ledArm(); }

void ledStartPattern(const LedSeg* pat, uint8_t len) {
  ledTicker.detach();
  g_pat = pat; g_patLen = len; g_patIdx = 0;
  ledArm();
}

// Change LED mode. Does nothing if unchanged (cheap to call in loop).
void setLedMode(LedMode m) {
  if (m == g_ledMode) return;
  g_ledMode = m;
  switch (m) {
    case LED_MODE_AP:     ledStartPattern(PAT_AP,     sizeof(PAT_AP)/sizeof(LedSeg));     break;
    case LED_MODE_NOWIFI: ledStartPattern(PAT_NOWIFI, sizeof(PAT_NOWIFI)/sizeof(LedSeg)); break;
    case LED_MODE_NOMQTT: ledStartPattern(PAT_NOMQTT, sizeof(PAT_NOMQTT)/sizeof(LedSeg)); break;
    case LED_MODE_SOLID:  ledTicker.detach(); g_pat = nullptr; digitalWrite(LED_PIN, LED_ON);  break;
    default:              ledTicker.detach(); g_pat = nullptr; digitalWrite(LED_PIN, LED_OFF); break;
  }
}

Profile* active();   // forward (defined in the profile section)

// Pick pattern from current state. Priority: WiFi > MQTT > CO2 alarm > off.
// MQTT only counts if the active profile actually has an MQTT host.
// (AP mode is set separately by apCallback during the blocking portal.)
void ledUpdate() {
  if (WiFi.status() != WL_CONNECTED) { setLedMode(LED_MODE_NOWIFI); return; }
#if ENABLE_MQTT
  Profile* p = active();
  if (p && strlen(p->mqttHost) && !mqtt.connected()) { setLedMode(LED_MODE_NOMQTT); return; }
#endif
  setLedMode(g_co2Alarm ? LED_MODE_SOLID : LED_MODE_OFF);
}

void apCallback(WiFiManager*) { setLedMode(LED_MODE_AP); }

// ---------- Config file (LittleFS) ----------
static String readLine(File& f) {
  String s = f.readStringUntil('\n');
  s.replace("\r", "");
  return s;
}

void loadConfig() {
  if (!LittleFS.begin()) { Serial.println("LittleFS mount error"); return; }
  File f = LittleFS.open("/config.txt", "r");
  if (!f) { Serial.println("No saved profiles yet"); return; }
  // The first line may be a version marker ("# v2"). Old files (v1) start
  // directly with the profile count and do NOT have the mqttBase field.
  int ver = 1;
  String first = readLine(f);
  if (first.startsWith("# v")) { ver = first.substring(3).toInt(); first = readLine(f); }
  profileCount = first.toInt();
  if (profileCount < 0 || profileCount > MAX_PROFILES) profileCount = 0;
  for (int i = 0; i < profileCount; i++) {
    readLine(f).toCharArray(profiles[i].ssid,     sizeof(profiles[i].ssid));
    readLine(f).toCharArray(profiles[i].pass,     sizeof(profiles[i].pass));
    readLine(f).toCharArray(profiles[i].mqttHost, sizeof(profiles[i].mqttHost));
    readLine(f).toCharArray(profiles[i].mqttPort, sizeof(profiles[i].mqttPort));
    readLine(f).toCharArray(profiles[i].mqttUser, sizeof(profiles[i].mqttUser));
    readLine(f).toCharArray(profiles[i].mqttPass, sizeof(profiles[i].mqttPass));
    if (ver >= 2) readLine(f).toCharArray(profiles[i].mqttBase, sizeof(profiles[i].mqttBase));
    else          profiles[i].mqttBase[0] = '\0';   // v1: no topic -> fall back to MQTT_BASE
    Serial.printf("Profile %d: SSID '%s' MQTT %s:%s topic '%s'\n",
                  i, profiles[i].ssid, profiles[i].mqttHost, profiles[i].mqttPort,
                  strlen(profiles[i].mqttBase) ? profiles[i].mqttBase : MQTT_BASE);
  }
  f.close();
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) { Serial.println("Cannot save config"); return; }
  f.printf("# v2\n%d\n", profileCount);   // version marker so mqttBase is read correctly
  for (int i = 0; i < profileCount; i++) {
    f.printf("%s\n%s\n%s\n%s\n%s\n%s\n%s\n",
             profiles[i].ssid, profiles[i].pass, profiles[i].mqttHost,
             profiles[i].mqttPort, profiles[i].mqttUser, profiles[i].mqttPass,
             profiles[i].mqttBase);
  }
  f.close();
  Serial.printf("Config saved (%d profiles)\n", profileCount);
}

// Calibration is stored separately so last_calibration/frc survive a reboot
void loadCal() {
  File f = LittleFS.open("/cal.txt", "r");
  if (!f) return;
  g_lastCalibration = (time_t) readLine(f).toInt();
  g_frcRaw = (uint16_t) readLine(f).toInt();
  f.close();
  Serial.printf("Calibration loaded: epoch=%ld frc=%u\n", (long)g_lastCalibration, g_frcRaw);
}
void saveCal() {
  File f = LittleFS.open("/cal.txt", "w");
  if (!f) return;
  f.printf("%ld\n%u\n", (long)g_lastCalibration, g_frcRaw);
  f.close();
}

// Add/replace profile. Same SSID -> update. Full list -> push out the oldest.
// Returns true if an old profile was pushed out (so WiFiMulti should be rebuilt).
bool addOrReplaceProfile(const char* ssid, const char* pass, const char* host,
                         const char* port, const char* user, const char* pw,
                         const char* base) {
  bool evicted = false;
  int idx = -1;
  for (int i = 0; i < profileCount; i++)
    if (strcmp(profiles[i].ssid, ssid) == 0) { idx = i; break; }
  if (idx < 0) {
    if (profileCount < MAX_PROFILES) idx = profileCount++;
    else {  // full -> shift everything down one, insert last
      for (int i = 0; i < MAX_PROFILES - 1; i++) profiles[i] = profiles[i + 1];
      idx = MAX_PROFILES - 1;
      evicted = true;
    }
  }
  memset(&profiles[idx], 0, sizeof(Profile));   // ensures null termination in every field
  strncpy(profiles[idx].ssid,     ssid, sizeof(profiles[idx].ssid) - 1);
  strncpy(profiles[idx].pass,     pass, sizeof(profiles[idx].pass) - 1);
  strncpy(profiles[idx].mqttHost, host, sizeof(profiles[idx].mqttHost) - 1);
  strncpy(profiles[idx].mqttPort, (strlen(port) ? port : "1883"), sizeof(profiles[idx].mqttPort) - 1);
  strncpy(profiles[idx].mqttUser, user, sizeof(profiles[idx].mqttUser) - 1);
  strncpy(profiles[idx].mqttPass, pw,   sizeof(profiles[idx].mqttPass) - 1);
  strncpy(profiles[idx].mqttBase, base, sizeof(profiles[idx].mqttBase) - 1);
  return evicted;
}

bool removeProfile(const char* ssid) {
  int idx = -1;
  for (int i = 0; i < profileCount; i++)
    if (strcmp(profiles[i].ssid, ssid) == 0) { idx = i; break; }
  if (idx < 0) return false;
  for (int i = idx; i < profileCount - 1; i++) profiles[i] = profiles[i + 1];
  profileCount--;
  return true;
}

Profile* active() {
  return (activeProfile >= 0 && activeProfile < profileCount) ? &profiles[activeProfile] : nullptr;
}
void updateActiveProfile() {
  if (WiFi.status() != WL_CONNECTED) { activeProfile = -1; return; }
  String s = WiFi.SSID();
  for (int i = 0; i < profileCount; i++)
    if (s == profiles[i].ssid) { activeProfile = i; return; }
  activeProfile = -1;
}

// ---------- WiFi ----------
bool connectWiFi() {
  if (profileCount == 0) return false;
  for (int i = 0; i < profileCount; i++) wifiMulti.addAP(profiles[i].ssid, profiles[i].pass);
  setLedMode(LED_MODE_NOWIFI);   // show that we are searching (loop() takes over afterwards)
  Serial.print("Connecting (WiFiMulti)");
  unsigned long start = millis();
  while (millis() - start < 20000) {
    if (wifiMulti.run() == WL_CONNECTED) break;
    delay(300); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    updateActiveProfile();
    Serial.printf("Connected to '%s', IP %s\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    return true;
  }
  return false;
}

// Opens the setup portal (blocking). Adds a new profile and reboots.
void runPortal() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wm.setAPCallback(apCallback);
  WiFiManagerParameter pHost("host", "MQTT host/IP",  "",        39);
  WiFiManagerParameter pPort("port", "MQTT port",     "1883",    5);
  WiFiManagerParameter pUser("user", "MQTT user",     "",        23);
  WiFiManagerParameter pPass("pass", "MQTT password", "",        23);
  WiFiManagerParameter pBase("base", "MQTT topic/prefix (e.g. home/scd41_bedroom)", MQTT_BASE, 39);
  wm.addParameter(&pHost); wm.addParameter(&pPort);
  wm.addParameter(&pUser); wm.addParameter(&pPass);
  wm.addParameter(&pBase);

  bool ok = wm.startConfigPortal(AP_NAME);
  setLedMode(LED_MODE_OFF);
  if (!ok) { Serial.println("Portal timeout -> reboot"); delay(500); ESP.restart(); }

  addOrReplaceProfile(wm.getWiFiSSID().c_str(), wm.getWiFiPass().c_str(),
                      pHost.getValue(), pPort.getValue(), pUser.getValue(),
                      pPass.getValue(), pBase.getValue());
  saveConfig();
  Serial.println("Network added -> reboot");
  delay(500); ESP.restart();
}

// ---------- Time ----------
void onNtpSync() {
  g_lastNtpSync = time(nullptr);
  if (!g_powerOn) g_powerOn = g_lastNtpSync - (time_t)(millis() / 1000);   // first sync: back-calculate
}
String isoTime(time_t t) {
  if (t < 1000000000) return "n/a";
  struct tm tmv; gmtime_r(&t, &tmv);
  char buf[25]; strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
  return String(buf);
}
// Uptime in seconds. Uses the clock once NTP is synced (survives millis() wrap after 49 days).
unsigned long uptimeSec() {
  time_t now = time(nullptr);
  return (g_powerOn && now > g_powerOn) ? (unsigned long)(now - g_powerOn) : millis() / 1000;
}

// ---------- Calibration ----------
#if ENABLE_MQTT
void publishStatus();   // defined in the MQTT section further down
#endif
bool calibrate() {
  scd4x.stopPeriodicMeasurement(); delay(500);
  uint16_t frc = 0;
  uint16_t err = scd4x.performForcedRecalibration(CALIBRATION_PPM, frc);
  delay(400); scd4x.startPeriodicMeasurement();
  if (err || frc == 0xFFFF) { Serial.println("Calibration failed"); return false; }
  g_frcRaw = frc;                     // raw value as HA's status expects
  g_lastCalibration = time(nullptr);
  saveCal();                          // save so it survives a reboot
#if ENABLE_MQTT
  publishStatus();                    // update HA with the new calibration
#endif
  Serial.printf("Calibrated, correction %d ppm (raw %u)\n", (int)frc - 0x8000, frc);
  return true;
}

#if ENABLE_MQTT
// Topics HA is already set up to read (manual MQTT sensors, NOT discovery).
// The topic (prefix) comes from the active profile; if empty, MQTT_BASE is used as before.
String mqttBase() {
  Profile* p = active();
  return (p && strlen(p->mqttBase)) ? String(p->mqttBase) : String(MQTT_BASE);
}
String tState()  { return mqttBase() + "/state"; }
String tAvail()  { return mqttBase() + "/availability"; }
String tStatus() { return mqttBase() + "/status"; }

// Measurement -> same JSON structure as the original firmware (+ VOC/NOx from SGP41).
// The old HA sensors only read co2/temp_c/rh and ignore the extra fields.
// voc_index/nox_index are -1 until the SGP41 has finished warming up.
void publishState() {
  if (!mqtt.connected() || !g_valid) return;
  String p = String("{\"co2\":") + g_co2 +
    ",\"temp_c\":" + String(g_temp, 2) +
    ",\"rh\":" + String(g_hum, 2) +
    ",\"voc_index\":" + g_voc +
    ",\"nox_index\":" + g_nox +
    ",\"voc_raw\":" + g_sgpRawVoc +
    ",\"nox_raw\":" + g_sgpRawNox +
    ",\"meas_ts_epoch\":" + (long)g_lastMeasurement +
    ",\"meas_ts_zulu\":\"" + isoTime(g_lastMeasurement) + "\"" +
    ",\"ntp_sync_ts_epoch\":" + (long)g_lastNtpSync +
    ",\"ntp_sync_ts_zulu\":\"" + isoTime(g_lastNtpSync) + "\"" +
    ",\"last_cal_epoch\":" + (long)g_lastCalibration +
    ",\"last_cal_zulu\":\"" + isoTime(g_lastCalibration) + "\"}";
  mqtt.publish(tState().c_str(), p.c_str(), true);
}

// Status/calibration info (same fields as original)
void publishStatus() {
  if (!mqtt.connected()) return;
  String p = String("{\"status\":\"ok\",\"last_calibration_epoch\":") + (long)g_lastCalibration +
    ",\"last_calibration_zulu\":\"" + isoTime(g_lastCalibration) + "\"" +
    ",\"frc_correction\":" + g_frcRaw + "}";
  mqtt.publish(tStatus().c_str(), p.c_str(), true);
}

void mqttConnect() {
  Profile* p = active();
  if (!p || strlen(p->mqttHost) == 0 || mqtt.connected()) return;
  mqtt.setServer(p->mqttHost, atoi(p->mqttPort));
  String avail = tAvail();
  // Unique client ID per PHYSICAL device (chip ID), so several devices on the same
  // broker don't kick each other off with identical IDs (that caused endless reconnect spam).
  String cid = String(DEVICE_ID) + "_" + String(ESP.getChipId(), HEX);
  // Last Will: the broker sets "offline" automatically if the device disappears
  if (mqtt.connect(cid.c_str(), p->mqttUser, p->mqttPass,
                   avail.c_str(), 0, true, "offline")) {
    mqtt.publish(avail.c_str(), "online", true);
    publishStatus();
    Serial.printf("MQTT connected (%s) as '%s'\n", p->mqttHost, cid.c_str());
  } else {
    Serial.printf("MQTT connection failed (state %d), retrying shortly\n", mqtt.state());
  }
}
#endif

// ---------- Web server ----------
void handleRoot() {
  String html = F(
    "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Wemos D1 Mini + SCD41</title><style>"
    "body{font-family:system-ui,Arial,sans-serif;margin:0;padding:20px;color:#1f2937}"
    ".card{max-width:620px;border:1px solid #d1d5db;border-radius:10px;padding:24px}"
    "h1{font-size:1.1rem;font-weight:600;margin:0 0 6px}"
    ".ppm{font-size:2.6rem;font-weight:800;margin:0}"
    ".m{font-size:1.05rem;margin:10px 0 0}.meta{color:#6b7280;font-size:.85rem;margin:4px 0}"
    ".avg{color:#6b7280;font-size:.8rem;margin:2px 0 0}.big{margin-bottom:14px}"
    "button{margin:10px 8px 10px 0;padding:10px 16px;font-size:.95rem;border:1px solid #9ca3af;"
    "background:#f9fafb;border-radius:8px;cursor:pointer}button:hover{background:#f3f4f6}"
    ".status{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0 10px}"
    ".b{font-size:.8rem;padding:4px 11px;border-radius:999px;background:#e5e7eb;color:#374151}"
    ".b.ok{background:#dcfce7;color:#166534}.b.bad{background:#fee2e2;color:#991b1b}"
    "</style></head><body><div class='card'>"
    "<h1>Wemos D1 Mini + SCD41 + SGP41</h1>"
    "<div class='big'><div class='ppm'><span id='co2'>--</span> ppm</div>"
    "<div class='avg'>avg 1 h: <span id='co2_1'>--</span> &middot; 24 h: <span id='co2_24'>--</span></div></div>"
    "<div class='m'>Temp: <span id='temp'>--</span> &deg;C</div>"
    "<div class='avg'>avg 1 h: <span id='temp_1'>--</span> &middot; 24 h: <span id='temp_24'>--</span></div>"
    "<div class='m'>RH: <span id='hum'>--</span> %</div>"
    "<div class='avg'>avg 1 h: <span id='hum_1'>--</span> &middot; 24 h: <span id='hum_24'>--</span></div>"
    "<div class='m'>VOC index: <span id='voc'>--</span></div>"
    "<div class='avg'>avg 1 h: <span id='voc_1'>--</span> &middot; 24 h: <span id='voc_24'>--</span></div>"
    "<div class='m'>NOx index: <span id='nox'>--</span></div>"
    "<div class='avg'>avg 1 h: <span id='nox_1'>--</span> &middot; 24 h: <span id='nox_24'>--</span></div>"
    "<div class='meta' style='margin-top:10px'>VOC/NOx: 100 = normal for the room, higher = worse air (1-500). "
    "Averages restart after a reboot.</div>"
    "<div class='status'>"
    "<span class='b' id='b_wifi'>WiFi</span>"
    "<span class='b' id='b_net'>Internet</span>"
    "<span class='b' id='b_mqtt'>MQTT</span>"
    "</div>"
    "<div class='meta'>Network: <span id='wifi'>--</span> (<span id='rssi'>--</span> dBm, <span id='ip'>--</span>)</div>"
    "<div class='meta'>MQTT broker: <span id='mq'>--</span></div>"
    "<div class='meta'>MQTT topic: <span id='mqb'>--</span></div>"
    "<div class='meta'>Powered on (UTC): <span id='tp'>--</span> (uptime <span id='up'>--</span>)</div>"
    "<div class='meta'>Last measurement (UTC): <span id='tm'>--</span></div>"
    "<div class='meta'>Last NTP sync (UTC): <span id='tn'>--</span></div>"
    "<div class='meta'>Last calibration (UTC): <span id='tc'>--</span></div>"
    "<button id='cal' onclick='cal()'>Calibrate (400 ppm)</button>"
    "<button onclick=\"location='/config'\">Settings</button>"
    "<button onclick=\"if(confirm('Open SCD41-Setup to add a WiFi network?'))location='/addwifi'\">Portal (backup)</button>"
    "<div class='meta'>Endpoints: /json, /calibrate (POST), /config, /addwifi, /reset</div>"
    "<div class='meta'>Refresh: 5s</div>"
    "</div><script>"
    "function badge(id,on){let e=document.getElementById(id);e.className='b '+(on?'ok':'bad');}"
    "function f(v,d){return v==null?'--':v.toFixed(d);}"
    "function up(s){let d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);"
    "return (d?d+'d ':'')+h+'h '+m+'m';}"
    "function avg(k,a1,a24,d){document.getElementById(k+'_1').textContent=f(a1,d);"
    "document.getElementById(k+'_24').textContent=f(a24,d);}"
    "async function u(){try{let d=await(await fetch('/json')).json();"
    "co2.textContent=d.valid?d.co2:'--';"
    "temp.textContent=d.valid?d.temperature.toFixed(2):'--';hum.textContent=d.valid?d.humidity.toFixed(2):'--';"
    "voc.textContent=d.sgp_conditioning?'warming up...':(d.voc_index<=0?'--':d.voc_index);"
    "nox.textContent=d.sgp_conditioning?'warming up...':(d.nox_index<=0?'--':d.nox_index);"
    "let a=d.avg_1h,b=d.avg_24h;"
    "avg('co2',a.co2,b.co2,0);avg('temp',a.temperature,b.temperature,2);avg('hum',a.humidity,b.humidity,2);"
    "avg('voc',a.voc_index,b.voc_index,0);avg('nox',a.nox_index,b.nox_index,0);"
    "wifi.textContent=d.wifi;mq.textContent=d.mqtt||'(none)';rssi.textContent=d.rssi;ip.textContent=d.ip;"
    "mqb.textContent=d.mqtt_base||'--';"
    "badge('b_wifi',d.wifi_connected);badge('b_net',d.internet);"
    "let bm=document.getElementById('b_mqtt');"
    "if(!d.mqtt_enabled){bm.className='b';bm.textContent='MQTT off';}"
    "else if(!d.mqtt){bm.className='b';bm.textContent='MQTT n/a';}"
    "else{badge('b_mqtt',d.mqtt_connected);bm.textContent=d.mqtt_connected?'MQTT':'MQTT \\u2717';}"
    "tp.textContent=d.power_on;document.getElementById('up').textContent=up(d.uptime_s);"
    "tm.textContent=d.last_measurement;tn.textContent=d.last_ntp_sync;tc.textContent=d.last_calibration;"
    "}catch(e){}}"
    "async function cal(){cal_btn=document.getElementById('cal');cal_btn.disabled=true;"
    "cal_btn.textContent='Calibrating...';try{await fetch('/calibrate',{method:'POST'});}catch(e){}"
    "setTimeout(()=>{cal_btn.disabled=false;cal_btn.textContent='Calibrate (400 ppm)';u();},1500);}"
    "u();setInterval(u,5000);"
    "</script></body></html>");
  server.send(200, "text/html", html);
}
void handleJson() {
  Profile* p = active();
#if ENABLE_MQTT
  bool mqEnabled = true, mqConnected = mqtt.connected();
#else
  bool mqEnabled = false, mqConnected = false;
#endif
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  bool sgpConditioning = (millis() - g_sgpStartMs) < SGP_CONDITION_MS;
  auto b = [](bool v) { return v ? "true" : "false"; };
  String j = String("{\"co2\":") + g_co2 +
    ",\"temperature\":" + String(g_temp, 2) + ",\"humidity\":" + String(g_hum, 2) +
    ",\"valid\":" + b(g_valid) +
    ",\"avg_1h\":" + avgJson(avgOver(AVG_1H_BUCKETS)) +
    ",\"avg_24h\":" + avgJson(avgOver(AVG_BUCKETS)) +
    ",\"wifi\":\"" + jsonEsc(WiFi.SSID()) + "\"" +
    ",\"rssi\":" + WiFi.RSSI() +
    ",\"ip\":\"" + WiFi.localIP().toString() + "\"" +
    ",\"mqtt\":\"" + jsonEsc(p ? String(p->mqttHost) : String()) + "\"" +
    ",\"mqtt_base\":\"" + jsonEsc((p && strlen(p->mqttBase)) ? String(p->mqttBase) : String(MQTT_BASE)) + "\"" +
    ",\"profiles\":" + profileCount +
    ",\"wifi_connected\":" + b(wifiOk) +
    ",\"internet\":" + b(g_internet) +
    ",\"mqtt_enabled\":" + b(mqEnabled) +
    ",\"mqtt_connected\":" + b(mqConnected) +
    ",\"voc_index\":" + g_voc +
    ",\"nox_index\":" + g_nox +
    ",\"voc_raw\":" + g_sgpRawVoc +
    ",\"nox_raw\":" + g_sgpRawNox +
    ",\"sgp_valid\":" + b(g_sgpValid) +
    ",\"sgp_conditioning\":" + b(sgpConditioning) +
    ",\"power_on\":\"" + isoTime(g_powerOn) + "\"" +
    ",\"uptime_s\":" + uptimeSec() +
    ",\"last_measurement\":\"" + isoTime(g_lastMeasurement) + "\"" +
    ",\"last_ntp_sync\":\"" + isoTime(g_lastNtpSync) + "\"" +
    ",\"last_calibration\":\"" + isoTime(g_lastCalibration) + "\"}";
  server.send(200, "application/json", j);
}
void handleCalibrate() {
  bool ok = calibrate();
  server.send(ok ? 200 : 500, "application/json",
    String("{\"calibrated\":") + (ok ? "true" : "false") + "}");
}
void handleAddWifi() {
  server.send(200, "text/html",
    "<html><meta charset='utf-8'><body style='font-family:sans-serif'>"
    "Opening <b>SCD41-Setup</b>... connect with your phone/PC and add WiFi + MQTT.<br>"
    "The device reboots automatically afterwards.</body></html>");
  g_requestPortal = true;   // handled in loop (the portal is blocking)
}
void handleReset() {
  server.send(200, "text/html",
    "<html><meta charset='utf-8'><body style='font-family:sans-serif'>"
    "All profiles deleted. Connect to <b>SCD41-Setup</b> to set up again...</body></html>");
  delay(500);
  LittleFS.remove("/config.txt");
  WiFiManager wm; wm.resetSettings();
  delay(500); ESP.restart();
}

// Response page that reboots the device and sends the browser back to /config afterwards.
// Used when WiFiMulti must be rebuilt (it has no removeAP).
void sendRestartPage(const char* msg) {
  server.send(200, "text/html",
    String("<html><head><meta charset='utf-8'><meta http-equiv='refresh' content='12;url=/config'></head>"
           "<body style='font-family:sans-serif'>") + msg +
    " The device is rebooting &ndash; this page reloads shortly...</body></html>");
  delay(500); ESP.restart();
}

// Settings page: enter profiles from the computer (the network doesn't have to be in range)
void handleConfigPage() {
  String h = F(
    "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>SCD41 settings</title><style>"
    "body{font-family:system-ui,Arial,sans-serif;margin:0;padding:20px;color:#1f2937}"
    ".card{max-width:620px;border:1px solid #d1d5db;border-radius:10px;padding:24px;margin-bottom:16px}"
    "h1,h2{margin:0 0 12px}h2{font-size:1rem}"
    "table{width:100%;border-collapse:collapse;font-size:.9rem}td,th{text-align:left;padding:6px 4px;border-bottom:1px solid #eee}"
    "label{display:block;font-size:.85rem;color:#374151;margin:8px 0 2px}"
    "input{width:100%;padding:8px;border:1px solid #cbd5e1;border-radius:6px;box-sizing:border-box}"
    ".row{display:flex;gap:10px}.row>div{flex:1}"
    "button,a.btn{display:inline-block;margin-top:14px;padding:9px 15px;border:1px solid #9ca3af;"
    "background:#f9fafb;border-radius:8px;cursor:pointer;text-decoration:none;color:#111}"
    ".del{color:#b91c1c;font-size:.85rem}.edit{color:#1d4ed8;font-size:.85rem}"
    ".meta{color:#6b7280;font-size:.8rem;margin-top:8px}"
    "</style></head><body>"
    "<div class='card'><h1>SCD41 settings</h1>"
    "<h2>Saved profiles</h2><table><tr><th>WiFi (SSID)</th><th>MQTT</th><th></th></tr>");
  for (int i = 0; i < profileCount; i++) {
    String s    = htmlEsc(profiles[i].ssid);
    String host = htmlEsc(profiles[i].mqttHost);
    String port = htmlEsc(profiles[i].mqttPort);
    String topic = htmlEsc(strlen(profiles[i].mqttBase) ? profiles[i].mqttBase : MQTT_BASE);
    h += "<tr><td>" + s + (i == activeProfile ? " &bull; active" : "") +
         "</td><td>" + host + ":" + port +
         "<br><span style='color:#6b7280'>" + topic + "</span>"
         "</td><td>"
         "<a class='edit' href='#form' onclick=\"editRow(this);return false\""
         " data-ssid='" + s + "'"
         " data-host='" + host + "'"
         " data-port='" + port + "'"
         " data-user='" + htmlEsc(profiles[i].mqttUser) + "'"
         " data-base='" + topic + "'>edit</a> &middot; "
         "<a class='del' href='/delwifi?ssid=" + urlEnc(profiles[i].ssid) +
         "' onclick=\"return confirm('Delete this profile?')\">delete</a></td></tr>";
  }
  if (profileCount == 0) h += "<tr><td colspan='3'>(none yet)</td></tr>";
  h += F("</table><div class='meta'>Max 3 profiles. Same SSID overwrites. A full list pushes out the oldest.</div></div>");

  h += F(
    "<div class='card' id='form'><h2>Add / edit profile</h2>"
    "<form method='POST' action='/saveprofile'>"
    "<label>WiFi name (SSID)</label><input id='ssid' name='ssid' required>"
    "<label>WiFi password</label><input id='wpass' name='wpass' type='password' placeholder='(empty = keep current)'>"
    "<div class='row'>"
    "<div><label>MQTT host/IP</label><input id='mhost' name='mhost'></div>"
    "<div><label>MQTT port</label><input id='mport' name='mport' value='1883'></div></div>"
    "<div class='row'>"
    "<div><label>MQTT user</label><input id='muser' name='muser'></div>"
    "<div><label>MQTT password</label><input id='mpass' name='mpass' type='password' placeholder='(empty = keep)'></div></div>"
    "<label>MQTT topic/prefix</label><input id='mbase' name='mbase' placeholder='home/scd41_1'>"
    "<button type='submit'>Save profile</button> "
    "<a class='btn' href='/'>Back</a>"
    "<div class='meta'>Click <b>edit</b> next to a profile to load its values into the form and change only e.g. MQTT. "
    "The network does not need to be in range.</div>"
    "</form></div>"
    "<script>"
    "function editRow(a){var d=a.dataset;"
    "document.getElementById('ssid').value=d.ssid;"
    "document.getElementById('mhost').value=d.host;"
    "document.getElementById('mport').value=d.port;"
    "document.getElementById('muser').value=d.user;"
    "document.getElementById('mbase').value=d.base;"
    "document.getElementById('wpass').value='';"
    "document.getElementById('mpass').value='';"
    "document.getElementById('form').scrollIntoView();"
    "document.getElementById('mhost').focus();}"
    "</script></body></html>");
  server.send(200, "text/html", h);
}

void handleSaveProfile() {
  String ssid = server.arg("ssid"); ssid.trim();
  if (ssid.length() == 0) { server.send(400, "text/plain", "SSID missing"); return; }
  if (ssid.length() > 32) { server.send(400, "text/plain", "SSID too long"); return; }

  String wpass = server.arg("wpass");
  String mhost = server.arg("mhost");
  String mport = server.arg("mport");
  String muser = server.arg("muser");
  String mpass = server.arg("mpass");
  String mbase = server.arg("mbase"); mbase.trim();

  // Find an existing one -> empty field means "keep current"
  Profile* ex = nullptr;
  for (int i = 0; i < profileCount; i++)
    if (ssid == profiles[i].ssid) { ex = &profiles[i]; break; }
  bool wifiChanged = !ex || wpass.length() > 0;   // new profile or new WiFi password
  if (wpass.length() == 0 && ex) wpass = ex->pass;
  if (mpass.length() == 0 && ex) mpass = ex->mqttPass;
  if (mbase.length() == 0 && ex) mbase = ex->mqttBase;   // empty = keep (otherwise falls back to MQTT_BASE)

  bool evicted = addOrReplaceProfile(ssid.c_str(), wpass.c_str(), mhost.c_str(),
                                     mport.c_str(), muser.c_str(), mpass.c_str(), mbase.c_str());
  saveConfig();

  // A profile was pushed out -> WiFiMulti still knows the old network. Reboot.
  if (evicted) { sendRestartPage("Saved (oldest profile pushed out)."); return; }

  if (wifiChanged) wifiMulti.addAP(ssid.c_str(), wpass.c_str());   // make available for roaming right away
  updateActiveProfile();                                          // index may have moved
#if ENABLE_MQTT
  if (ssid == WiFi.SSID()) mqtt.disconnect();      // active profile changed -> reconnect with new MQTT info
#endif
  server.sendHeader("Location", "/config");
  server.send(303, "text/plain", "Saved");
}

void handleDelProfile() {
  String ssid = server.arg("ssid");
  if (!removeProfile(ssid.c_str())) {
    server.sendHeader("Location", "/config");
    server.send(303, "text/plain", "Not found");
    return;
  }
  saveConfig();
  // WiFiMulti has no removeAP -> reboot so the deleted network is fully forgotten
  sendRestartPage("Profile deleted.");
}

// ---------- SGP41 (VOC/NOx) ----------
// Must be called about once per second (the gas algorithm is tuned for 1 Hz).
// Uses the SCD41's temp/humidity for compensation. The first ~10 s run
// conditioning (the NOx heater warms up); after that real measurements + index.
void sgpTick() {
  // Compensation: convert temp/humidity to the "ticks" the SGP41 expects.
  uint16_t rhTicks = 0x8000, tTicks = 0x6666;   // defaults: 50 %RH, 25 °C
  if (g_valid) {
    float rh = g_hum; if (rh < 0) rh = 0;   if (rh > 100) rh = 100;
    float t  = g_temp; if (t < -45) t = -45; if (t > 130) t = 130;
    rhTicks = (uint16_t)(rh * 65535.0f / 100.0f + 0.5f);
    tTicks  = (uint16_t)((t + 45.0f) * 65535.0f / 175.0f + 0.5f);
  }

  uint16_t rawVoc = 0, rawNox = 0;
  bool conditioning = (millis() - g_sgpStartMs) < SGP_CONDITION_MS;

  if (conditioning) {                          // only raw VOC is returned during warm-up
    if (!sgp41.executeConditioning(rhTicks, tTicks, rawVoc)) {
      g_sgpRawVoc = rawVoc; g_sgpValid = true;
    }
    return;                                     // no index until conditioning is over
  }

  if (sgp41.measureRawSignals(rhTicks, tTicks, rawVoc, rawNox)) return;  // I2C error
  g_sgpRawVoc = rawVoc; g_sgpRawNox = rawNox; g_sgpValid = true;

  int32_t voc = 0, nox = 0;
  GasIndexAlgorithm_process(&vocParams, (int32_t)rawVoc, &voc);
  GasIndexAlgorithm_process(&noxParams, (int32_t)rawNox, &nox);
  g_voc = voc; g_nox = nox;
  avgAddGas(voc, nox);
}

// SCD41: read a new measurement if one is ready (new data every ~5 s).
// Runs regardless of WiFi, so the averages keep going without network.
void scdTick() {
  bool ready = false; scd4x.getDataReadyStatus(ready);   // renamed in 1.x
  if (!ready) return;
  uint16_t co2; float t, h;
  if (scd4x.readMeasurement(co2, t, h) || co2 == 0) return;
  g_co2 = co2; g_temp = t; g_hum = h; g_valid = true;
  g_lastValidMs = millis();
  g_lastMeasurement = time(nullptr);
  avgAddClimate(co2, t, h);
  if (co2 >= CO2_LED_ON)       g_co2Alarm = true;    // hysteresis: on at/above 1300...
  else if (co2 < CO2_LED_OFF)  g_co2Alarm = false;   // ...off only when below 1200
  Serial.printf("CO2: %u ppm  Temp: %.2f C  RH: %.2f %%\n", co2, t, h);
#if ENABLE_MQTT
  publishState();
#endif
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200); delay(100);
  Serial.println("\nSCD41 Wemos starting...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  Wire.begin();
  scd4x.begin(Wire, SCD41_I2C_ADDR_62);   // address is required in 1.x
  scd4x.stopPeriodicMeasurement(); delay(500);
  scd4x.startPeriodicMeasurement();

  sgp41.begin(Wire);                 // VOC/NOx sensor on the same I2C bus (0x59)
  GasIndexAlgorithm_init(&vocParams, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
  GasIndexAlgorithm_init(&noxParams, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
  g_sgpStartMs = millis();           // start ~10 s conditioning window

  loadConfig();
  loadCal();
  if (!connectWiFi()) runPortal();   // no network (e.g. after WiFi-lost reboot) -> AP mode; reboots afterwards

  if (MDNS.begin(MDNS_HOST)) { MDNS.addService("http", "tcp", 80);
    Serial.printf("mDNS: http://%s.local\n", MDNS_HOST); }

  settimeofday_cb(onNtpSync);
  configTime(0, 0, NTP_SERVER);

  server.on("/", handleRoot);
  server.on("/json", handleJson);
  server.on("/calibrate", HTTP_POST, handleCalibrate);   // POST only, so a visit/prefetch doesn't calibrate
  server.on("/addwifi", handleAddWifi);
  server.on("/config", handleConfigPage);
  server.on("/saveprofile", HTTP_POST, handleSaveProfile);
  server.on("/delwifi", handleDelProfile);
  server.on("/reset", handleReset);
  server.begin();

#if ENABLE_MQTT
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(2);          // short timeout so a dead broker doesn't block the SGP41 tick for long
#endif
}

// ---------- Loop ----------
void loop() {
  if (g_requestPortal) { g_requestPortal = false; runPortal(); }   // reboots

  // The SGP41 must tick at 1 Hz independently of WiFi, so the algorithm's baseline stays stable
  if (millis() - g_lastSgpTick >= 1000) { g_lastSgpTick = millis(); sgpTick(); }

  unsigned long now = millis();
  if (now - g_lastPublish >= PUBLISH_INTERVAL) { g_lastPublish = now; scdTick(); }

  // The SCD41 hasn't answered for a long time -> stop publishing old values
  if (g_valid && millis() - g_lastValidMs >= SENSOR_STALE_MS) {
    g_valid = false;
    Serial.println("SCD41 not responding -> readings marked invalid");
  }

  ledUpdate();   // pick blink pattern from WiFi/MQTT/CO2 (AP mode is handled by the portal)

  if (WiFi.status() != WL_CONNECTED) {
    if (!g_wifiLost) { g_wifiLost = true; g_wifiLostMs = millis();
      Serial.println("WiFi lost"); }
    // None of the saved networks came back -> reboot. At boot connectWiFi()
    // then fails and setup() opens the AP portal "SCD41-Setup".
    if (millis() - g_wifiLostMs >= WIFI_LOST_REBOOT_MS) {
      Serial.println("No WiFi for too long -> reboot into AP mode");
      delay(200); ESP.restart();
    }
    wifiMulti.run();           // try to jump to an available saved network
    updateActiveProfile();
    delay(200);
    return;
  }
  if (g_wifiLost) { g_wifiLost = false; updateActiveProfile();
    Serial.printf("WiFi back: '%s'\n", WiFi.SSID().c_str()); }

  MDNS.update();
#if ENABLE_MQTT
  if (!mqtt.connected() && millis() - g_lastMqttTry >= 5000) {
    g_lastMqttTry = millis();
    mqttConnect();
  }
  mqtt.loop();
#endif
  server.handleClient();

  now = millis();
  if (now - g_lastNetCheck >= 30000) { g_lastNetCheck = now; checkInternet(); }
}
