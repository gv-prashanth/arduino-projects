#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Audio.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ── Pin definitions ──────────────────────────────────────
#define I2S_LRCK 4
#define I2S_BCLK 5
#define I2S_DOUT 6
#define TFT_CS 10
#define TFT_DC 13
#define TFT_RST 14
#define TFT_MOSI 11
#define TFT_SCK 12
#define ENC_CLK 1
#define ENC_DT  2
#define ENC_SW 42

// ── Constants ────────────────────────────────────────────
#define RB_HOST "http://de1.api.radio-browser.info"
#define RB_RESULT_LIMIT 40
#define VOL_MAX 21
#define INITIAL_VOLUME 18
#define WIFI_SSID "XXXXX"
#define WIFI_PASSWORD "YYYYY"
// ── Country list ─────────────────────────────────────────
struct CountryEntry { const char* code; const char* name; };
static const CountryEntry COUNTRIES[] = {
  { "all", "All" }, { "US", "USA" }, { "IN", "India" }, { "GB", "UK" },
  { "DE", "Germany" }, { "FR", "France" }, { "JP", "Japan" }, { "CA", "Canada" },
  { "AU", "Australia" }, { "IT", "Italy" }, { "ES", "Spain" }, { "BR", "Brazil" },
  { "MX", "Mexico" }, { "NL", "Netherlands" }, { "SE", "Sweden" }, { "NO", "Norway" },
  { "ZA", "South Africa" }, { "SG", "Singapore" }, { "AE", "UAE" },
};
static const int COUNTRY_COUNT = sizeof(COUNTRIES) / sizeof(COUNTRIES[0]);

// ── Genre list ───────────────────────────────────────────
struct GenreEntry { const char* tag; const char* label; };
static const GenreEntry GENRES[] = {
  { "all", "All" }, { "music", "Music" }, { "news", "News" },
  { "jazz", "Jazz" }, { "classical", "Classical" },
  { "indian classical", "Indian ClS" }, { "rock", "Rock" },
};
static const int GENRE_COUNT = sizeof(GENRES) / sizeof(GENRES[0]);

// ── Curated Indian Classical stations ────────────────────
struct CuratedStation { const char* name; const char* url; const char* country; const char* tag; };
static const CuratedStation CURATED_INDIAN_CLASSICAL[] = {
  { "AIR Raagam", "https://airhlspush.pc.cdn.bitgravity.com/httppush/hlspbaudio159/hlspbaudio15964kbps.m3u8", "IN", "indian classical" },
  { "AIR Ragam",  "https://air.pc.cdn.bitgravity.com/air/live/pbaudio158/playlist.m3u8",                     "IN", "indian classical" },
};
static const int CURATED_COUNT = sizeof(CURATED_INDIAN_CLASSICAL) / sizeof(CURATED_INDIAN_CLASSICAL[0]);

// ── Global objects ───────────────────────────────────────
Audio audio;
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(160, 128);
QueueHandle_t encQueue;
Preferences prefs;

// ── State ────────────────────────────────────────────────
enum EncEvent { EV_CW, EV_CCW, EV_PRESS, EV_LONG };
enum UiMode { MODE_NORMAL, MODE_BROWSE, MODE_EDIT };
enum FocusItem { F_NOWPLAYING, F_COUNTRY, F_TYPE };

struct Station { String name; String url; int bitrate; String country; };
Station stations[RB_RESULT_LIMIT + CURATED_COUNT];
int stationCount = 0;
int currentStation = 0;
int focusIndex = 0;
UiMode uiMode = MODE_NORMAL;
bool uiDirty = true;

String stationName = "Loading...";
String streamTitle = "";
String audioFormat = "---";
int audioBitrate = 0;

int previewStation = 0;
String previewTag = "all";
String previewCountry = "";
String selectedGenre = "all";
String selectedCountry = "all";
int bitrateCap = 96;

bool muted = false;
int lastVol = INITIAL_VOLUME;
uint32_t browseLastAction = 0;
volatile uint32_t holdStartMs = 0;
volatile bool buttonHolding = false;

String savedStationUrl = "";
String savedStationName = "";

// ══════════════════════════════════════════════════════════
//  NVS PERSISTENCE
// ══════════════════════════════════════════════════════════
void saveState() {
  prefs.begin("radio", false);
  prefs.putUChar("volume", (uint8_t)audio.getVolume());
  prefs.putBool("muted", muted);
  prefs.putString("country", selectedCountry);
  prefs.putString("genre", selectedGenre);
  prefs.putString("stUrl", savedStationUrl);
  prefs.putString("stName", savedStationName);
  prefs.end();
}

void loadState() {
  prefs.begin("radio", true);
  uint8_t v = prefs.getUChar("volume", INITIAL_VOLUME);
  muted = prefs.getBool("muted", false);
  selectedCountry = prefs.getString("country", "all");
  selectedGenre = prefs.getString("genre", "all");
  savedStationUrl = prefs.getString("stUrl", "");
  savedStationName = prefs.getString("stName", "");
  prefs.end();
  lastVol = (v == 0) ? INITIAL_VOLUME : v;
  audio.setVolume(v);
}

// ══════════════════════════════════════════════════════════
//  STATION FETCH & PLAYBACK
// ══════════════════════════════════════════════════════════
bool isIndianClassical() {
  bool inIndia = (selectedCountry == "all" || selectedCountry == "IN");
  bool classical = (selectedGenre == "classical" || selectedGenre == "indian classical");
  return inIndia && classical;
}

bool isCuratedDupe(const String& url) {
  for (int i = 0; i < stationCount; i++)
    if (stations[i].url == url) return true;
  return false;
}

void injectCuratedIndianClassical() {
  if (!isIndianClassical()) return;
  for (int i = 0; i < CURATED_COUNT && stationCount < (RB_RESULT_LIMIT + CURATED_COUNT); i++) {
    if (isCuratedDupe(CURATED_INDIAN_CLASSICAL[i].url)) continue;
    stations[stationCount].name = CURATED_INDIAN_CLASSICAL[i].name;
    stations[stationCount].url = CURATED_INDIAN_CLASSICAL[i].url;
    stations[stationCount].bitrate = 128;
    stations[stationCount].country = CURATED_INDIAN_CLASSICAL[i].country;
    stationCount++;
  }
}

bool fetchStations(String tag) {
  stationCount = 0;
  HTTPClient http;
  String filter = "name,url_resolved,bitrate,countrycode";
  String url = String(RB_HOST) + "/json/stations/search?limit=20"
    "&hidebroken=true&order=clickcount&reverse=true&codec=AAC"
    "&bitrateMax=" + String(bitrateCap) + "&fields=" + filter;
  if (tag != "all" && tag != "indian classical") url += "&tag=" + tag;
  else if (tag == "indian classical") url += "&tag=classical";
  if (selectedCountry != "all") url += "&countrycode=" + selectedCountry;

  http.begin(url);
  int code = http.GET();
  if (code == 200) {
    DynamicJsonDocument doc(24000);
    if (!deserializeJson(doc, http.getStream())) {
      for (JsonObject s : doc.as<JsonArray>()) {
        String u = s["url_resolved"] | "";
        if (!u.startsWith("http://") && !u.startsWith("https://")) continue;
        if (stationCount >= RB_RESULT_LIMIT) break;
        stations[stationCount].name = s["name"].as<String>();
        stations[stationCount].url = u;
        stations[stationCount].bitrate = s["bitrate"] | 0;
        stations[stationCount].country = s["countrycode"].as<String>();
        stationCount++;
      }
    }
  }
  http.end();
  injectCuratedIndianClassical();
  return stationCount > 0;
}

void playStation(int i, bool persist = true) {
  if (i < 0 || i >= stationCount) return;
  currentStation = i;
  stationName = stations[i].name;
  streamTitle = "";
  audioFormat = "---";
  audioBitrate = 0;
  savedStationUrl = stations[i].url;
  savedStationName = stations[i].name;
  audio.connecttohost(stations[i].url.c_str());
  if (persist) saveState();
  uiDirty = true;
}

void restoreLastStation() {
  if (savedStationUrl.length() > 0) {
    for (int i = 0; i < stationCount; i++) {
      if (stations[i].url == savedStationUrl) {
        playStation(i, false);
        return;
      }
    }
    // Not found in fetched list — play saved URL directly
    stationName = savedStationName.length() ? savedStationName : "Saved Station";
    streamTitle = "";
    audioFormat = "---";
    audioBitrate = 0;
    audio.connecttohost(savedStationUrl.c_str());
    uiDirty = true;
    return;
  }
  if (stationCount > 0) playStation(0, false);
}

String countryName(String c) {
  c.toUpperCase();
  for (int i = 0; i < COUNTRY_COUNT; i++)
    if (c == COUNTRIES[i].code) return String(COUNTRIES[i].name);
  return c;
}

String typeName() {
  for (int i = 0; i < GENRE_COUNT; i++)
    if (selectedGenre == GENRES[i].tag) return String(GENRES[i].label);
  return String(selectedGenre);
}

// ══════════════════════════════════════════════════════════
//  UI DRAWING
// ══════════════════════════════════════════════════════════
#define DISP canvas

void chip(int x, int y, String txt, uint16_t col) {
  int w = txt.length() * 6 + 10;
  bool activeEdit = (uiMode == MODE_EDIT &&
    ((focusIndex == 1 && x < 60) || (focusIndex == 2 && x > 60)));
  if (activeEdit) {
    DISP.fillRoundRect(x, y, w, 14, 3, col);
    DISP.setTextColor(ST77XX_BLACK);
  } else {
    DISP.drawRoundRect(x, y, w, 14, 3, col);
    DISP.setTextColor(col);
  }
  DISP.setCursor(x + 5, y + 4);
  DISP.print(txt);
}

void drawCenteredSize2(const String& s, int y, uint16_t color, bool highlight) {
  DISP.setTextSize(2);
  int w = s.length() * 12;
  int x = (160 - w) / 2;
  if (x < 0) x = 0;
  if (highlight) {
    DISP.fillRoundRect(x - 4, y - 2, w + 8, 20, 3, ST77XX_YELLOW);
    DISP.setTextColor(ST77XX_BLACK);
  } else {
    DISP.setTextColor(color);
  }
  DISP.setCursor(x, y);
  DISP.print(s);
  DISP.setTextSize(1);
}

void drawUI() {
  DISP.fillScreen(0x0000);

  uint16_t topBg  = tft.color565(50, 0, 60);
  uint16_t cardBg = tft.color565(20, 55, 75);
  uint16_t border = ST77XX_WHITE;
  uint16_t accent = ST77XX_GREEN;
  uint16_t warn   = ST77XX_YELLOW;
  uint16_t fmtCol = ST77XX_CYAN;

  // ── Top bar ─────────────────────────────────────────
  DISP.fillRoundRect(2, 2, 156, 14, 3, topBg);
  DISP.drawRoundRect(2, 2, 156, 14, 3, border);
  DISP.setTextSize(1);
  DISP.setCursor(6, 4);
  DISP.setTextColor(accent);
  DISP.print("GTS RADIO");

  // Volume bars
  uint16_t volCol = (uiMode == MODE_NORMAL) ? accent : 0x39E7;
  for (int i = 0; i < 12; i++) {
    uint16_t c = (i < audio.getVolume() * 12 / VOL_MAX) ? volCol : 0x2104;
    DISP.fillRoundRect(90 + i * 5, 5, 4, 6, 1, c);
  }

  // ── Main card ───────────────────────────────────────
  DISP.fillRoundRect(2, 18, 156, 84, 5, cardBg);
  DISP.drawRoundRect(2, 18, 156, 84, 5, border);

  // "Now Playing" label — size 2
  DISP.setTextSize(2);
  const char* npLabel = "Now Playing";
  int npW = strlen(npLabel) * 12;
  uint16_t npCol = (focusIndex == F_NOWPLAYING && uiMode != MODE_NORMAL) ? warn : ST77XX_WHITE;
  DISP.setTextColor(npCol);
  DISP.setCursor((160 - npW) / 2, 22);
  DISP.print(npLabel);
  DISP.setTextSize(1);

  // Station name — size 2
  String np;
  if (uiMode == MODE_EDIT && focusIndex == F_NOWPLAYING && stationCount > 0) {
    String raw = stations[previewStation].name;
    if (raw.length() > 11) raw = raw.substring(0, 11);
    np = "<" + raw + ">";
  } else {
    np = stationName;
    if (np.length() > 12) np = np.substring(0, 12);
  }
  drawCenteredSize2(np, 46, ST77XX_WHITE,
    uiMode == MODE_EDIT && focusIndex == F_NOWPLAYING);

  // ── Format + bitrate row ────────────────────────────
  String fmtLine = audioFormat;
  if (audioBitrate > 0) fmtLine += " " + String(audioBitrate) + "kbps";
  DISP.setTextSize(1);
  DISP.setTextColor(fmtCol);
  DISP.setCursor((160 - (int)fmtLine.length() * 6) / 2, 70);
  DISP.print(fmtLine);

  // ── Bottom chips ────────────────────────────────────
  String cCode = (uiMode == MODE_EDIT && focusIndex == F_COUNTRY)
    ? String(previewCountry)
    : (selectedCountry == "all"
        ? (stationCount > 0 && currentStation < stationCount
            ? stations[currentStation].country : "")
        : String(selectedCountry));
  chip(4, 84, countryName(cCode), focusIndex == F_COUNTRY ? warn : accent);

  String gLabel;
  if (uiMode == MODE_EDIT && focusIndex == F_TYPE) {
    gLabel = previewTag;
    for (int i = 0; i < GENRE_COUNT; i++)
      if (previewTag == GENRES[i].tag) { gLabel = GENRES[i].label; break; }
  } else {
    gLabel = typeName();
  }
  chip(92, 84, gLabel, focusIndex == F_TYPE ? warn : ST77XX_CYAN);

  // ── Bottom bar ──────────────────────────────────────
  DISP.fillRoundRect(2, 106, 156, 20, 4, ST77XX_BLACK);
  DISP.drawRoundRect(2, 106, 156, 20, 4,
    (uiMode == MODE_NORMAL) ? border : accent);

  DISP.setTextSize(1);
  DISP.setCursor(6, 112);
  DISP.setTextColor(ST77XX_WHITE);
  if (uiMode == MODE_NORMAL) DISP.print("Vol | Hold: Menu");
  else if (uiMode == MODE_BROWSE) DISP.print("Rotate Select");
  else DISP.print("Rotate Change");

  // Hold-progress circle
  DISP.drawCircle(149, 116, 8, ST77XX_WHITE);
  if (buttonHolding) {
    int prog = min(8, (int)((millis() - holdStartMs) / 87));
    for (int i = 0; i < prog; i++)
      DISP.fillCircle(149, 116, i, accent);
  }

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 160, 128);
  uiDirty = false;
}

// ══════════════════════════════════════════════════════════
//  EVENT HANDLING
// ══════════════════════════════════════════════════════════
void applyChange() {
  fetchStations(selectedGenre);
  playStation(0);
}

void handleEvent(uint8_t ev) {
  browseLastAction = millis();

  if (uiMode == MODE_NORMAL) {
    if (ev == EV_CW) {
      audio.setVolume(min(VOL_MAX, audio.getVolume() + 1));
      saveState();
    } else if (ev == EV_CCW) {
      audio.setVolume(max(0, audio.getVolume() - 1));
      saveState();
    } else if (ev == EV_PRESS) {
      muted = !muted;
      if (muted) { lastVol = audio.getVolume(); audio.setVolume(0); }
      else audio.setVolume(lastVol);
      saveState();
    } else if (ev == EV_LONG) {
      uiMode = MODE_BROWSE;
      previewStation = currentStation;
      previewTag = selectedGenre;
      previewCountry = selectedCountry;
    }
  } else if (uiMode == MODE_BROWSE) {
    if (ev == EV_CW) focusIndex = (focusIndex + 1) % 3;
    else if (ev == EV_CCW) focusIndex = (focusIndex + 2) % 3;
    else if (ev == EV_PRESS) uiMode = MODE_EDIT;
    else if (ev == EV_LONG) uiMode = MODE_NORMAL;
  } else if (uiMode == MODE_EDIT) {
    if (focusIndex == F_NOWPLAYING) {
      if (ev == EV_CW && stationCount > 0) previewStation = (previewStation + 1) % stationCount;
      else if (ev == EV_CCW && stationCount > 0) previewStation = (previewStation - 1 + stationCount) % stationCount;
      else if (ev == EV_PRESS) { playStation(previewStation); uiMode = MODE_BROWSE; }
      else if (ev == EV_LONG) uiMode = MODE_BROWSE;
    } else if (focusIndex == F_COUNTRY) {
      if (ev == EV_CW || ev == EV_CCW) {
        int ci = 0;
        for (int i = 0; i < COUNTRY_COUNT; i++) if (previewCountry == COUNTRIES[i].code) { ci = i; break; }
        int dir = (ev == EV_CW) ? 1 : -1;
        ci = (ci + dir + COUNTRY_COUNT) % COUNTRY_COUNT;
        previewCountry = COUNTRIES[ci].code;
      } else if (ev == EV_PRESS) { selectedCountry = previewCountry; saveState(); applyChange(); uiMode = MODE_BROWSE; }
      else if (ev == EV_LONG) uiMode = MODE_BROWSE;
    } else if (focusIndex == F_TYPE) {
      if (ev == EV_CW || ev == EV_CCW) {
        int gi = 0;
        for (int i = 0; i < GENRE_COUNT; i++) if (previewTag == GENRES[i].tag) { gi = i; break; }
        int dir = (ev == EV_CW) ? 1 : -1;
        gi = (gi + dir + GENRE_COUNT) % GENRE_COUNT;
        previewTag = GENRES[gi].tag;
      } else if (ev == EV_PRESS) { selectedGenre = previewTag; saveState(); applyChange(); uiMode = MODE_BROWSE; }
      else if (ev == EV_LONG) uiMode = MODE_BROWSE;
    }
  }
  uiDirty = true;
}

// ══════════════════════════════════════════════════════════
//  ROTARY ENCODER TASK
// ══════════════════════════════════════════════════════════
void taskRotary(void* p) {
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  int lastClk = digitalRead(ENC_CLK);
  bool lastBtn = HIGH;
  uint32_t pressAt = 0;

  for (;;) {
    int clk = digitalRead(ENC_CLK);
    if (clk != lastClk && clk == LOW) {
      uint8_t e = digitalRead(ENC_DT) ? EV_CW : EV_CCW;
      xQueueSend(encQueue, &e, 0);
    }
    lastClk = clk;

    bool btn = digitalRead(ENC_SW);
    if (btn == LOW && lastBtn == HIGH) {
      pressAt = millis();
      holdStartMs = pressAt;
      buttonHolding = true;
    }
    if (btn == HIGH && lastBtn == LOW) {
      buttonHolding = false;
      uint8_t e = (millis() - pressAt > 700) ? EV_LONG : EV_PRESS;
      xQueueSend(encQueue, &e, 0);
    }
    lastBtn = btn;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ══════════════════════════════════════════════════════════
//  SETUP / LOOP
// ══════════════════════════════════════════════════════════
void setup() {
 // Serial.begin(115200);
  delay(200);

  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);

  loadState();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);
  audio.setVolume(muted ? 0 : audio.getVolume());

  encQueue = xQueueCreate(16, 1);
  xTaskCreatePinnedToCore(taskRotary, "rotary", 4096, nullptr, 1, nullptr, 1);

  fetchStations(selectedGenre);
  restoreLastStation();
}

unsigned long lastFmtPoll = 0;

void loop() {
  audio.loop();

  uint8_t ev;
  while (xQueueReceive(encQueue, &ev, 0) == pdTRUE) handleEvent(ev);

  // Auto-timeout from browse/edit
  uint32_t timeoutMs = (uiMode == MODE_EDIT) ? 30000 : 10000;
  if ((uiMode == MODE_BROWSE || uiMode == MODE_EDIT) &&
      millis() - browseLastAction > timeoutMs) {
    uiMode = MODE_NORMAL;
    uiDirty = true;
  }

  // Poll codec name every 2 seconds
  if (millis() - lastFmtPoll > 2000) {
    lastFmtPoll = millis();
    String codec = audio.getCodecname();
    if (codec.length() > 0 && codec != audioFormat) {
      audioFormat = codec;
      audioFormat.toUpperCase();
      uiDirty = true;
    }
  }

  if (uiDirty) drawUI();
  delay(5);
}

// ══════════════════════════════════════════════════════════
//  AUDIO CALLBACKS
// ══════════════════════════════════════════════════════════
void audio_showstreamtitle(const char* info) {
  streamTitle = String(info);
  uiDirty = true;
}

void audio_bitrate(const char* info) {
  audioBitrate = atoi(info);
  uiDirty = true;
}

void audio_info(const char* info) {
  Serial.print("info: ");
  Serial.println(info);
}