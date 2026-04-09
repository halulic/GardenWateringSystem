#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_SHT4x.h>
#include <Arduino_GFX_Library.h>
#include <Preferences.h>
#include <math.h>

// =====================================================
// WATERING SYSTEM + BACKGROUND AI + USER-LOCKED WET/DRY
// FIXED EDIT MODE RESPONSIVENESS + DATE RETURNED IN ROW 01
// FIXED ROW 03 TARGET/DUR ENTER LOGIC + CLEAR ACTIVE FIELD
// =====================================================

// ------------------- PINS (WS FINAL) -------------------
#define TFT_CS   5
#define TFT_DC   27
#define TFT_RST  26
#define TFT_SCK  18
#define TFT_MOSI 23
#define TFT_MISO -1

#define I2C_SDA  21
#define I2C_SCL  22

#define PIN_UP      35
#define PIN_DOWN    33
#define PIN_ENTER   32

#define PIN_SENSE_EN   14
#define PIN_SENSE_ADC  34
#define PIN_VALVE      13

// ------------------- DISPLAY (ILI9488 480x320) -------------------
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 1 /*rotation*/, false /*IPS*/);

// ------------------- COLORS -------------------
uint16_t C_BLACK, C_WHITE, C_GREEN, C_YELLOW, C_RED, C_CYAN;
uint16_t C_SPRING, C_SUMMER, C_AMBER;

// ------------------- RTC + SHT41 -------------------
RTC_DS3231 rtc;
Adafruit_SHT4x sht4;

// ------------------- NVS -------------------
Preferences prefs;

// ------------------- UI GRID -------------------
static const int ROWS = 8;
static const int ROW_H = 40;
static const int LEFT = 8;

// ------------------- SETTINGS -------------------
int TARGET = 50;     // 30..85 %
int DUR_MIN = 5;     // 3..10 min
int SCALE_PCT = 75;  // 60..100 %

static const int WINDOW_START_H = 1;
static const int WINDOW_END_H   = 5;

static const int DAILY_READ_H1 = 6;
static const int DAILY_READ_H2 = 12;
static const int DAILY_READ_H3 = 18;

// edit korak za WET/DRY
static const int CAL_STEP = 10;

// SCALE granice
static const int SCALE_MIN = 60;
static const int SCALE_MAX = 100;

// TARGET granice
static const int TARGET_MIN = 30;
static const int TARGET_MAX = 85;

// Ulaz / izlaz edit moda
static const uint32_t CAL_ENTER_HOLD_MS = 5000; // ulaz u edit
static const uint32_t CAL_SAVE_EXIT_MS  = 2000; // save + exit

// Blink aktivnog polja
static const uint32_t BLINK_MS = 350;

// ------------------- SOIL RAW DETECT -------------------
#define SOIL_RAW_SHORT_MAX   60
#define SOIL_RAW_OPEN_MIN    3850

// =====================================================
// CALIBRATION (USER-LOCKED)
// =====================================================
static const int RAW_WET_INIT = 700;
static const int RAW_DRY_INIT = 2800;

int RAW_DRY = RAW_DRY_INIT;
int RAW_WET = RAW_WET_INIT;

int calDry = RAW_DRY_INIT;
int calWet = RAW_WET_INIT;

enum CalBgState {
  CAL_BG_LEARN = 0,
  CAL_BG_OK    = 1,
  CAL_BG_WARN  = 2
};

uint8_t calBgState = CAL_BG_OK;

// ------------------- STATE -------------------
float airT = NAN;
float airRH = NAN;

int moistRaw = 0;
int moistPct = 0;
int moistPctBaseFiltered = 0;

bool senseIdle = true;
bool tempBlock = false;

bool watering = false;
bool wateringManual = false;
uint32_t wateringEndMs = 0;

// Anti-flicker cache
String   lastLine[ROWS];
uint16_t lastFg[ROWS];

// AUTO slot guard
uint32_t lastAutoKey = 0;

// dnevna merenja guard
uint32_t lastDailyKey = 0;

// edit guards
bool calEnterHoldFired = false;
bool calSaveHoldFired = false;
bool calBlinkOn = true;
uint32_t calBlinkMs = 0;
bool calWaitReleaseAfterEnter = false;

// ------------------- BARGRAPH CACHE -------------------
int  lastBarPct = -1;
int  lastBarTgt = -1;
bool lastBarHl  = false;

// =====================================================
// BACKGROUND AI / FILTER / TREND SETTINGS
// =====================================================
static const int SOIL_READ_SAMPLES = 24;
static const int SOIL_READ_DELAY_MS = 2;
static const int SOIL_RAW_DEADBAND = 18;
static const int SOIL_RAW_SNAP_BAND = 10;

static const float SOIL_TEMP_REF_C = 20.0f;
static const float SOIL_TEMP_RAW_PER_C = 6.0f;

static const int AI_HISTORY_MAX = 32;

static const int AI_JUMP_RAW_THRESHOLD = 450;
static const int AI_STUCK_RAW_WINDOW = 12;
static const int AI_STUCK_RAW_DELTA = 4;
static const int AI_RECOVERY_MIN_DELTA = 40;
static const uint32_t AI_RECOVERY_WINDOW_MS = 45UL * 60UL * 1000UL;

// Startup / repeatability stabilization
static const uint16_t SOIL_STARTUP_SETTLE_MS = 220;
static const uint16_t SOIL_NORMAL_SETTLE_MS  = 150;
static const uint8_t  SOIL_STARTUP_INTERNAL_READS = 3;
static const uint8_t  SOIL_NORMAL_INTERNAL_READS  = 3;
static const int      SOIL_STARTUP_STABLE_BAND = 35;
static const int      SOIL_NORMAL_STABLE_BAND  = 30;
static const uint8_t  SOIL_MAX_RETRIES = 2;

// =====================================================
// BACKGROUND AI / FILTER / TREND STATE
// =====================================================
struct AiPoint {
  uint32_t epoch;
  int raw;
  int rawTempComp;
  int pct;
  float tC;
  float rh;
  bool wateringState;
  bool valid;
};

AiPoint aiHist[AI_HISTORY_MAX];
int aiHistCount = 0;
int aiHistHead = 0;

int   moistRawFiltered = 0;
int   moistPctFiltered = 0;
bool  moistFilterPrimed = false;

float aiDryRatePctPerHour = 0.0f;
float aiPredHoursToTarget = -1.0f;
float aiConfidence = 0.0f;

bool  aiAnomalyJump = false;
bool  aiAnomalyStuck = false;
bool  aiAnomalyNoRecovery = false;
bool  aiSensorHealthBad = false;

bool  aiSoilVeryWet = false;
bool  aiSoilWet = false;
bool  aiSoilOk = false;
bool  aiSoilDry = false;
bool  aiSoilCritical = false;

bool     aiPrevWatering = false;
uint32_t aiLastWaterStopMs = 0;
int      aiRawAtWaterStop = 0;

// ------------------- BUTTONS -------------------
struct Button {
  int pin;
  bool activeLow = true;

  bool stable = false;
  bool raw = false;
  uint32_t lastChangeMs = 0;

  bool shortPress = false;
  bool longPress = false;

  bool longFired = false;
  uint32_t pressStartMs = 0;

  uint16_t debounceMs = 25;
  uint16_t longMs = 650;

  void begin(int p){
    pin = p;
    if(pin >= 34) pinMode(pin, INPUT);
    else pinMode(pin, INPUT_PULLUP);

    bool r = readRawPressed();
    stable = r;
    raw = r;
    lastChangeMs = millis();
  }

  bool readRawPressed(){
    int v = digitalRead(pin);
    return activeLow ? (v == LOW) : (v == HIGH);
  }

  void update(){
    shortPress = false;
    longPress = false;

    bool r = readRawPressed();
    uint32_t now = millis();

    if(r != raw){
      raw = r;
      lastChangeMs = now;
    }

    if((now - lastChangeMs) >= debounceMs){
      if(stable != raw){
        stable = raw;
        if(stable){
          pressStartMs = now;
          longFired = false;
        } else {
          if(!longFired) shortPress = true;
        }
      }
    }

    if(stable && !longFired && (now - pressStartMs) >= longMs){
      longFired = true;
      longPress = true;
    }
  }
};

Button btnUp, btnDown, btnEnter;

// =====================================================
// UI STATE
// =====================================================
enum UiMode {
  UI_NAV,
  UI_EDIT_TIME_H, UI_EDIT_TIME_M, UI_EDIT_DATE_MO, UI_EDIT_DATE_D, UI_EDIT_DATE_Y,
  UI_EDIT_TARGET, UI_EDIT_DUR,
  UI_EDIT_WET_DRY,
  UI_EDIT_ROW4_SCALE
};

UiMode uiMode = UI_NAV;

// 0=ROW01 TIME
// 1=ROW02 MOIST
// 2=ROW03 TARGET/DUR
// 3=ROW04 AIR/WATER/SCL
// 4=ROW05 WINDOW
int selRow = 0;
const int SEL_MIN = 0;
const int SEL_MAX = 4;

// row 02 user-edit state
enum CalFieldSel {
  CAL_FIELD_WET = 0,
  CAL_FIELD_DRY = 1
};

CalFieldSel calEditField = CAL_FIELD_WET;
int eDry = RAW_DRY_INIT;
int eWet = RAW_WET_INIT;
int eScale = 75;

// RTC edit buffers
int eHour=0, eMin=0, eDay=1, eMonth=1, eYear=2026;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
int clampi(int x, int lo, int hi);
float clampf(float x, float lo, float hi);
String two(int v);

bool soilIsShort(int raw);
bool soilIsOpen(int raw);

uint32_t makeAutoKey(const DateTime &t);
uint32_t rtcNowEpoch();

int mapMoistPctCal(int raw);
int applyScaleToPct(int pct);
int applySoilTempCompToRaw(int raw, float tC);
int applyDeadbandToRaw(int newRaw, int oldRaw);
int filterSoilRawSimple(int newRaw);

void aiResetStates();
AiPoint aiGetHist(int idxFromOldest);
void aiPushPoint(uint32_t epoch, int raw, int rawTempComp, int pct, float tC, float rh, bool wateringState);
void aiUpdateSoilStateFromPct(int pct);
int aiCountRecentStableRaw();
void aiEvaluateJumpAnomaly();
void aiEvaluateStuckAnomaly();
void aiEvaluateRecoveryAnomaly(uint32_t nowMs, int currentRawTempComp);
void aiEvaluateSensorHealth();
void aiComputeDryRateAndPrediction();
void aiUpdateAll(uint32_t nowMs);
void aiHandleWateringEdge(uint32_t nowMs);

void calSyncDisplayedValues();
void calUpdateState();
void calSaveNow();
String calStateText();

void drawRowTextOnBlack(int r, const String &s, uint16_t fg);
String withSelPrefix(const String &s, bool sel);

int readSoilAveragedCore(uint16_t settleMs);
int readSoilValidated(bool startupMode, bool keepSenseOn);
int readSoilOncePulsed(bool startupMode = false);
int readSoilOnceCalFast(bool startupMode = false);

void processSoilMeasurement(int rawMeasured, bool pushToHistory);
void startWatering(bool manual, int minutes);
void stopWatering();

bool isLeap(int y);
int daysInMonth(int y, int m);
void normalizeDate();
void loadEditFromRTC();
void saveEditToRTC();

bool rowIsHighlighted(int r);
uint16_t seasonTimeColorByMonth(int month);
uint16_t colorSoilPctBands(int pct);
uint16_t colorTempBands(float tC);
uint16_t colorRH(float rh);

void drawMoistBar_NoFlicker(bool hl);
void drawRow2Colored_NoFlicker(bool hl);
void drawRow3Colored_NoFlicker(bool hl);

String buildTimeRow(const DateTime &now);
String buildRow2();
String buildRow3();
String buildRow4Idle();
String buildRow5();

void loadSettings();
void saveSettings();
void remeasureSoilNow(bool pushToHistory);

bool isDailyReadTime(const DateTime &t);
bool isWetDryEditMode();

int median3(int a, int b, int c);

// =====================================================
// HELPERS
// =====================================================
int clampi(int x, int lo, int hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}

float clampf(float x, float lo, float hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}

String two(int v){
  char b[8];
  snprintf(b, sizeof(b), "%02d", v);
  return String(b);
}

bool soilIsShort(int raw){ return (raw <= SOIL_RAW_SHORT_MAX); }
bool soilIsOpen (int raw){ return (raw >= SOIL_RAW_OPEN_MIN); }

uint32_t makeAutoKey(const DateTime &t){
  return (uint32_t)t.year() * 1000000UL +
         (uint32_t)t.month() * 10000UL +
         (uint32_t)t.day() * 100UL +
         (uint32_t)t.hour();
}

uint32_t rtcNowEpoch(){
  DateTime n = rtc.now();
  return n.unixtime();
}

int median3(int a, int b, int c){
  if(a > b){ int t = a; a = b; b = t; }
  if(b > c){ int t = b; b = c; c = t; }
  if(a > b){ int t = a; a = b; b = t; }
  return b;
}

// =====================================================
// RAW -> % MAP
// =====================================================
int mapMoistPctCal(int raw){
  raw = clampi(raw, 0, 4095);

  if(soilIsShort(raw)) return 100;
  if(soilIsOpen(raw))  return 0;

  int dry = calDry;
  int wet = calWet;

  if(wet >= dry){
    int t = wet; wet = dry; dry = t;
  }

  if(raw >= dry) return 0;
  if(raw <= wet) return 100;

  long num = (long)(dry - raw) * 100L;
  long den = (long)(dry - wet);
  if(den <= 0) return 0;

  int pct = (int)(num / den);
  return clampi(pct, 0, 100);
}

int applyScaleToPct(int pct){
  pct = clampi(pct, 0, 100);
  long scaled = ((long)pct * (long)SCALE_PCT + 50L) / 100L;
  return clampi((int)scaled, 0, 100);
}

int applySoilTempCompToRaw(int raw, float tC){
  if(!isfinite(tC)) return raw;
  if(soilIsOpen(raw) || soilIsShort(raw)) return raw;

  float deltaC = (SOIL_TEMP_REF_C - tC);
  float corr = deltaC * SOIL_TEMP_RAW_PER_C;

  int corrected = (int)lroundf((float)raw + corr);
  return clampi(corrected, 0, 4095);
}

int applyDeadbandToRaw(int newRaw, int oldRaw){
  if(!moistFilterPrimed) return newRaw;

  int diff = newRaw - oldRaw;
  if(abs(diff) <= SOIL_RAW_SNAP_BAND) return oldRaw;
  if(abs(diff) <= SOIL_RAW_DEADBAND)  return oldRaw;

  return newRaw;
}

int filterSoilRawSimple(int newRaw){
  if(!moistFilterPrimed){
    moistFilterPrimed = true;
    moistRawFiltered = newRaw;
    return moistRawFiltered;
  }

  newRaw = applyDeadbandToRaw(newRaw, moistRawFiltered);

  float filtered = 0.70f * (float)moistRawFiltered + 0.30f * (float)newRaw;
  moistRawFiltered = (int)lroundf(filtered);

  return moistRawFiltered;
}

// =====================================================
// CAL HELPERS
// =====================================================
void calSyncDisplayedValues(){
  RAW_DRY = calDry;
  RAW_WET = calWet;
}

void calUpdateState(){
  if(soilIsOpen(moistRaw) || soilIsShort(moistRaw) || aiSensorHealthBad){
    calBgState = CAL_BG_WARN;
    return;
  }

  if((calDry - calWet) < 250){
    calBgState = CAL_BG_WARN;
    return;
  }

  calBgState = CAL_BG_OK;
}

void calSaveNow(){
  saveSettings();
  calUpdateState();
}

String calStateText(){
  if(calBgState == CAL_BG_WARN) return "CAL!";
  if(calBgState == CAL_BG_OK)   return "CAL+";
  return "CAL~";
}

// =====================================================
// AI CORE
// =====================================================
void aiResetStates(){
  aiDryRatePctPerHour = 0.0f;
  aiPredHoursToTarget = -1.0f;
  aiConfidence = 0.0f;

  aiAnomalyJump = false;
  aiAnomalyStuck = false;
  aiAnomalyNoRecovery = false;
  aiSensorHealthBad = false;

  aiSoilVeryWet = false;
  aiSoilWet = false;
  aiSoilOk = false;
  aiSoilDry = false;
  aiSoilCritical = false;
}

AiPoint aiGetHist(int idxFromOldest){
  AiPoint emptyPoint;
  emptyPoint.epoch = 0;
  emptyPoint.raw = 0;
  emptyPoint.rawTempComp = 0;
  emptyPoint.pct = 0;
  emptyPoint.tC = NAN;
  emptyPoint.rh = NAN;
  emptyPoint.wateringState = false;
  emptyPoint.valid = false;

  if(idxFromOldest < 0 || idxFromOldest >= aiHistCount) return emptyPoint;

  int first = (aiHistHead - aiHistCount + AI_HISTORY_MAX) % AI_HISTORY_MAX;
  int idx = (first + idxFromOldest) % AI_HISTORY_MAX;
  return aiHist[idx];
}

void aiPushPoint(uint32_t epoch, int raw, int rawTempComp, int pct, float tC, float rh, bool wateringState){
  AiPoint p;
  p.epoch = epoch;
  p.raw = raw;
  p.rawTempComp = rawTempComp;
  p.pct = pct;
  p.tC = tC;
  p.rh = rh;
  p.wateringState = wateringState;
  p.valid = true;

  aiHist[aiHistHead] = p;
  aiHistHead = (aiHistHead + 1) % AI_HISTORY_MAX;
  if(aiHistCount < AI_HISTORY_MAX) aiHistCount++;
}

void aiUpdateSoilStateFromPct(int pct){
  aiSoilVeryWet  = (pct >= 85);
  aiSoilWet      = (pct >= 66 && pct < 85);
  aiSoilOk       = (pct >= 41 && pct < 66);
  aiSoilDry      = (pct >= 21 && pct < 41);
  aiSoilCritical = (pct <= 20);
}

int aiCountRecentStableRaw(){
  if(aiHistCount < 2) return 0;

  int count = 1;
  AiPoint prev = aiGetHist(aiHistCount - 1);

  for(int i = aiHistCount - 2; i >= 0; i--){
    AiPoint p = aiGetHist(i);
    if(!p.valid || !prev.valid) break;

    if(abs(prev.rawTempComp - p.rawTempComp) <= AI_STUCK_RAW_DELTA){
      count++;
      prev = p;
    } else {
      break;
    }
  }

  return count;
}

void aiEvaluateJumpAnomaly(){
  aiAnomalyJump = false;
  if(aiHistCount < 2) return;

  AiPoint a = aiGetHist(aiHistCount - 2);
  AiPoint b = aiGetHist(aiHistCount - 1);
  if(!a.valid || !b.valid) return;

  if(abs(b.rawTempComp - a.rawTempComp) >= AI_JUMP_RAW_THRESHOLD){
    aiAnomalyJump = true;
  }
}

void aiEvaluateStuckAnomaly(){
  aiAnomalyStuck = false;
  int stableCnt = aiCountRecentStableRaw();
  if(stableCnt >= AI_STUCK_RAW_WINDOW){
    aiAnomalyStuck = true;
  }
}

void aiEvaluateRecoveryAnomaly(uint32_t nowMs, int currentRawTempComp){
  aiAnomalyNoRecovery = false;

  if(aiLastWaterStopMs == 0) return;
  if(watering) return;

  uint32_t dt = nowMs - aiLastWaterStopMs;
  if(dt < AI_RECOVERY_WINDOW_MS) return;

  int delta = aiRawAtWaterStop - currentRawTempComp;
  if(delta < AI_RECOVERY_MIN_DELTA){
    aiAnomalyNoRecovery = true;
  }
}

void aiEvaluateSensorHealth(){
  aiSensorHealthBad = false;

  if(soilIsOpen(moistRawFiltered) || soilIsShort(moistRawFiltered)){
    aiSensorHealthBad = true;
    return;
  }

  if(aiAnomalyJump || aiAnomalyStuck || aiAnomalyNoRecovery){
    aiSensorHealthBad = true;
  }
}

void aiComputeDryRateAndPrediction(){
  aiDryRatePctPerHour = 0.0f;
  aiPredHoursToTarget = -1.0f;
  aiConfidence = 0.0f;

  if(aiHistCount < 3) return;

  AiPoint oldest = aiGetHist(0);
  AiPoint newest = aiGetHist(aiHistCount - 1);

  if(!oldest.valid || !newest.valid) return;
  if(newest.epoch <= oldest.epoch) return;

  int firstIdx = -1;
  int lastIdx = -1;

  for(int i = 0; i < aiHistCount; i++){
    AiPoint p = aiGetHist(i);
    if(p.valid && !p.wateringState){
      firstIdx = i;
      break;
    }
  }

  for(int i = aiHistCount - 1; i >= 0; i--){
    AiPoint p = aiGetHist(i);
    if(p.valid && !p.wateringState){
      lastIdx = i;
      break;
    }
  }

  if(firstIdx < 0 || lastIdx < 0 || lastIdx <= firstIdx) return;

  oldest = aiGetHist(firstIdx);
  newest = aiGetHist(lastIdx);

  float dtHours = (float)(newest.epoch - oldest.epoch) / 3600.0f;
  if(dtHours < 0.25f) return;

  aiDryRatePctPerHour = ((float)oldest.pct - (float)newest.pct) / dtHours;

  float conf = 0.0f;
  if(dtHours >= 6.0f) conf += 0.35f;
  else if(dtHours >= 3.0f) conf += 0.25f;
  else conf += 0.10f;

  if(aiHistCount >= 6) conf += 0.20f;
  if(!aiAnomalyJump) conf += 0.15f;
  if(!aiAnomalyStuck) conf += 0.15f;
  if(!aiAnomalyNoRecovery) conf += 0.15f;

  aiConfidence = clampf(conf, 0.0f, 1.0f);

  if(aiDryRatePctPerHour > 0.10f){
    float deltaToTarget = (float)moistPctFiltered - (float)TARGET;
    if(deltaToTarget > 0.0f){
      aiPredHoursToTarget = deltaToTarget / aiDryRatePctPerHour;
      if(aiPredHoursToTarget < 0.0f) aiPredHoursToTarget = -1.0f;
    }
  }
}

void aiUpdateAll(uint32_t nowMs){
  aiUpdateSoilStateFromPct(moistPctFiltered);
  aiEvaluateJumpAnomaly();
  aiEvaluateStuckAnomaly();
  aiEvaluateRecoveryAnomaly(nowMs, moistRawFiltered);
  aiEvaluateSensorHealth();
  aiComputeDryRateAndPrediction();
}

void aiHandleWateringEdge(uint32_t nowMs){
  if(aiPrevWatering && !watering){
    aiLastWaterStopMs = nowMs;
    aiRawAtWaterStop = moistRawFiltered;
  }
  aiPrevWatering = watering;
}

// =====================================================
// DISPLAY / DATE HELPERS
// =====================================================
void drawRowTextOnBlack(int r, const String &s, uint16_t fg){
  if(r < 0 || r >= ROWS) return;
  if(lastLine[r] == s && lastFg[r] == fg) return;

  lastLine[r] = s;
  lastFg[r] = fg;

  int y = r * ROW_H;
  gfx->setCursor(LEFT, y + 10);
  gfx->setTextColor(fg, C_BLACK);

  String padded = s;
  while(padded.length() < 34) padded += " ";
  gfx->print(padded);
}

String withSelPrefix(const String &s, bool sel){
  return (sel ? "> " : "  ") + s;
}

bool isLeap(int y){
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

int daysInMonth(int y, int m){
  static const int d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if(m == 2) return isLeap(y) ? 29 : 28;
  m = clampi(m,1,12);
  return d[m-1];
}

void normalizeDate(){
  eMonth = clampi(eMonth, 1, 12);
  int dim = daysInMonth(eYear, eMonth);
  eDay = clampi(eDay, 1, dim);
  eHour = clampi(eHour, 0, 23);
  eMin  = clampi(eMin, 0, 59);
}

void loadEditFromRTC(){
  DateTime n = rtc.now();
  eHour = n.hour();
  eMin = n.minute();
  eDay = n.day();
  eMonth = n.month();
  eYear = n.year();
  normalizeDate();
}

void saveEditToRTC(){
  normalizeDate();
  rtc.adjust(DateTime(eYear, eMonth, eDay, eHour, eMin, 0));
}

bool rowIsHighlighted(int r){
  if(uiMode == UI_NAV) return (r == selRow);
  if((uiMode >= UI_EDIT_TIME_H && uiMode <= UI_EDIT_DATE_Y) && r == 0) return true;
  if(uiMode == UI_EDIT_WET_DRY && r == 1) return true;
  if((uiMode == UI_EDIT_TARGET || uiMode == UI_EDIT_DUR) && r == 2) return true;
  if(uiMode == UI_EDIT_ROW4_SCALE && r == 3) return true;
  return false;
}

uint16_t seasonTimeColorByMonth(int month){
  if(month == 12 || month == 1 || month == 2) return C_WHITE;
  if(month >= 3 && month <= 5) return C_SPRING;
  if(month >= 6 && month <= 8) return C_SUMMER;
  return C_AMBER;
}

uint16_t colorSoilPctBands(int pct){
  if(pct <= 30) return C_YELLOW;
  if(pct <= 40) return C_AMBER;
  if(pct <= 65) return C_GREEN;
  return C_CYAN;
}

uint16_t colorTempBands(float tC){
  if(!isfinite(tC)) return C_WHITE;
  if(tC < 13.0f) return C_WHITE;
  if(tC < 20.0f) return C_CYAN;
  if(tC < 26.0f) return C_AMBER;
  if(tC < 36.0f) return gfx->color565(255,170,170);
  return C_RED;
}

uint16_t colorRH(float rh){
  if(!isfinite(rh)) return C_WHITE;
  int rhi = (int)lroundf(rh);
  if(rhi <= 40) return gfx->color565(0,0,255);
  if(rhi <= 70) return C_GREEN;
  return C_RED;
}

// =====================================================
// SOIL / WATER
// =====================================================
int readSoilAveragedCore(uint16_t settleMs){
  senseIdle = false;
  digitalWrite(PIN_SENSE_EN, HIGH);

  delay(settleMs);

  analogRead(PIN_SENSE_ADC);
  delay(4);

  long sum = 0;
  for(int i = 0; i < SOIL_READ_SAMPLES; i++){
    sum += analogRead(PIN_SENSE_ADC);
    delay(SOIL_READ_DELAY_MS);
  }

  return (int)(sum / SOIL_READ_SAMPLES);
}

int readSoilValidated(bool startupMode, bool keepSenseOn){
  const uint16_t settleMs = startupMode ? SOIL_STARTUP_SETTLE_MS : SOIL_NORMAL_SETTLE_MS;
  const uint8_t internalReads = startupMode ? SOIL_STARTUP_INTERNAL_READS : SOIL_NORMAL_INTERNAL_READS;
  const int stableBand = startupMode ? SOIL_STARTUP_STABLE_BAND : SOIL_NORMAL_STABLE_BAND;

  int bestValue = 0;

  for(uint8_t attempt = 0; attempt <= SOIL_MAX_RETRIES; attempt++){
    int vals[3] = {0,0,0};

    for(uint8_t i = 0; i < internalReads; i++){
      vals[i] = readSoilAveragedCore(settleMs);
      delay(8);
    }

    int med = median3(vals[0], vals[1], vals[2]);
    bestValue = med;

    int d01 = abs(vals[0] - vals[1]);
    int d12 = abs(vals[1] - vals[2]);
    int d02 = abs(vals[0] - vals[2]);

    bool stable = (d01 <= stableBand) && (d12 <= stableBand) && (d02 <= stableBand);
    if(stable) break;

    delay(40);
  }

  if(!keepSenseOn){
    digitalWrite(PIN_SENSE_EN, LOW);
    senseIdle = true;
  }

  return clampi(bestValue, 0, 4095);
}

int readSoilOncePulsed(bool startupMode){
  return readSoilValidated(startupMode, false);
}

int readSoilOnceCalFast(bool startupMode){
  return readSoilValidated(startupMode, true);
}

void processSoilMeasurement(int rawMeasured, bool pushToHistory){
  moistRaw = clampi(rawMeasured, 0, 4095);

  int rawTempComp = applySoilTempCompToRaw(moistRaw, airT);
  int rawFiltered = filterSoilRawSimple(rawTempComp);

  int pctFilteredBase = mapMoistPctCal(rawFiltered);
  int pctFilteredScaled = applyScaleToPct(pctFilteredBase);

  moistRawFiltered = rawFiltered;
  moistPctBaseFiltered = pctFilteredBase;
  moistPctFiltered = pctFilteredScaled;
  moistPct = moistPctFiltered;

  if(pushToHistory){
    aiPushPoint(rtcNowEpoch(), moistRaw, rawTempComp, moistPctFiltered, airT, airRH, watering);
  }

  aiUpdateAll(millis());
  calUpdateState();

  lastLine[1] = "";
  lastLine[2] = "";
  lastLine[3] = "";
  lastLine[4] = "";
  lastLine[5] = "";
  lastLine[6] = "";
}

void startWatering(bool manual, int minutes){
  watering = true;
  wateringManual = manual;
  digitalWrite(PIN_VALVE, HIGH);
  wateringEndMs = millis() + (uint32_t)minutes * 60UL * 1000UL;

  lastLine[3] = "";
  lastLine[5] = "";
}

void stopWatering(){
  watering = false;
  wateringManual = false;
  digitalWrite(PIN_VALVE, LOW);

  lastLine[3] = "";
  lastLine[5] = "";
}

bool isDailyReadTime(const DateTime &t){
  if(t.minute() != 0) return false;
  if(t.second() > 10) return false;
  int h = t.hour();
  return (h == DAILY_READ_H1 || h == DAILY_READ_H2 || h == DAILY_READ_H3);
}

bool isWetDryEditMode(){
  return (uiMode == UI_EDIT_WET_DRY);
}

// =====================================================
// BAR GRAPH / BUILDERS
// =====================================================
void drawMoistBar_NoFlicker(bool hl){
  const int yRow = 1 * ROW_H;
  const int barX = LEFT;
  const int barY = yRow + 30;
  const int barW = 480 - 2*LEFT;
  const int barH = 8;

  int pct = clampi(moistPct, 0, 100);
  int tgt = clampi(TARGET, 0, 100);

  if(pct == lastBarPct && tgt == lastBarTgt && hl == lastBarHl) return;
  lastBarPct = pct;
  lastBarTgt = tgt;
  lastBarHl  = hl;

  uint16_t fillCol = colorSoilPctBands(pct);
  uint16_t targetCol  = C_WHITE;
  uint16_t currentCol = C_RED;

  uint16_t emptyCol = gfx->color565(25,25,25);
  uint16_t frameCol = hl ? C_CYAN : C_WHITE;

  gfx->fillRect(barX, barY, barW, barH, C_BLACK);
  gfx->drawRect(barX, barY, barW, barH, frameCol);

  int innerX = barX + 1;
  int innerY = barY + 1;
  int innerW = barW - 2;
  int innerH = barH - 2;

  int fillW = (innerW * pct) / 100;
  fillW = clampi(fillW, 0, innerW);

  if(fillW > 0) gfx->fillRect(innerX, innerY, fillW, innerH, fillCol);
  if(fillW < innerW) gfx->fillRect(innerX + fillW, innerY, innerW - fillW, innerH, emptyCol);

  int mx = innerX + (innerW * tgt) / 100;
  mx = clampi(mx, innerX, innerX + innerW - 1);

  int cx = (fillW <= 0) ? innerX : (innerX + fillW - 1);
  cx = clampi(cx, innerX, innerX + innerW - 1);

  if(pct == 0 && innerW >= 4)  cx = innerX + 2;
  if(pct == 100 && innerW >= 4) cx = innerX + innerW - 3;

  if(abs(mx - cx) <= 1){
    int tpos = cx + 3;
    if(tpos > innerX + innerW - 2) tpos = cx - 3;
    if(tpos < innerX + 1) tpos = cx;
    mx = tpos;
  }

  gfx->drawFastVLine(mx, innerY, innerH, targetCol);
  if(mx + 1 <= innerX + innerW - 1) gfx->drawFastVLine(mx + 1, innerY, innerH, targetCol);

  if(cx - 2 >= innerX) gfx->drawFastVLine(cx - 2, innerY, innerH, C_BLACK);
  gfx->drawFastVLine(cx + 2 <= innerX + innerW - 1 ? cx + 2 : cx, innerY, innerH, C_BLACK);

  if(cx - 1 >= innerX) gfx->drawFastVLine(cx - 1, innerY, innerH, currentCol);
  gfx->drawFastVLine(cx, innerY, innerH, currentCol);
  if(cx + 1 <= innerX + innerW - 1) gfx->drawFastVLine(cx + 1, innerY, innerH, currentCol);
}

String buildRow2(){
  char r2[240];

  if(uiMode == UI_EDIT_WET_DRY){
    bool blinkNow = calBlinkOn;

    char wetBuf[24];
    char dryBuf[24];

    if(calEditField == CAL_FIELD_WET){
      if(blinkNow) snprintf(wetBuf, sizeof(wetBuf), "[%d]", eWet);
      else         snprintf(wetBuf, sizeof(wetBuf), " %d ", eWet);
      snprintf(dryBuf, sizeof(dryBuf), "%d", eDry);
    } else {
      snprintf(wetBuf, sizeof(wetBuf), "%d", eWet);
      if(blinkNow) snprintf(dryBuf, sizeof(dryBuf), "[%d]", eDry);
      else         snprintf(dryBuf, sizeof(dryBuf), " %d ", eDry);
    }

    snprintf(r2, sizeof(r2), "02 WET %s  DRY %s  M:%d%%", wetBuf, dryBuf, moistPct);
    return String(r2);
  }

  double t = isfinite(airT) ? (double)airT : 0.0;
  int rh = isfinite(airRH) ? (int)lroundf(airRH) : 0;

  snprintf(r2, sizeof(r2), "02 T:%.1fC RH:%d%% M:%d%%", t, rh, moistPct);
  return String(r2);
}

void drawRow2Colored_NoFlicker(bool hl){
  uint16_t fg = hl ? C_CYAN : C_WHITE;
  String r2 = withSelPrefix(buildRow2(), hl);
  drawRowTextOnBlack(1, r2, fg);
  drawMoistBar_NoFlicker(hl);
}

String buildRow3(){
  char r3[220];
  bool probeBlock = soilIsOpen(moistRaw);

  char tgtBuf[24];
  char durBuf[24];

  if(uiMode == UI_EDIT_TARGET){
    snprintf(tgtBuf, sizeof(tgtBuf), "[%d%%]", TARGET);
    snprintf(durBuf, sizeof(durBuf), "%02d:00", DUR_MIN);
  }
  else if(uiMode == UI_EDIT_DUR){
    snprintf(tgtBuf, sizeof(tgtBuf), "%d%%", TARGET);
    snprintf(durBuf, sizeof(durBuf), "[%02d:00]", DUR_MIN);
  }
  else {
    snprintf(tgtBuf, sizeof(tgtBuf), "%d%%", TARGET);
    snprintf(durBuf, sizeof(durBuf), "%02d:00", DUR_MIN);
  }

  snprintf(r3, sizeof(r3), "03 TARGET %s  DUR %s%s%s",
           tgtBuf, durBuf,
           tempBlock ? "  BLOCK<=12C" : "",
           probeBlock ? "  PROBE OPEN" : "");
  return String(r3);
}

void drawRow3Colored_NoFlicker(bool hl){
  uint16_t fg = hl ? C_CYAN : C_WHITE;
  String r3 = withSelPrefix(buildRow3(), hl);
  drawRowTextOnBlack(2, r3, fg);
}

// VRACEN DATUM U ROW 01
String buildTimeRow(const DateTime &now){
  String HH = two(now.hour());
  String MM = two(now.minute());
  String SS = two(now.second());
  String MO = two(now.month());
  String DD = two(now.day());
  String YY = String(now.year());

  if(uiMode != UI_NAV &&
     uiMode != UI_EDIT_TARGET &&
     uiMode != UI_EDIT_DUR &&
     uiMode != UI_EDIT_WET_DRY &&
     uiMode != UI_EDIT_ROW4_SCALE){
    HH = two(eHour);
    MM = two(eMin);
    SS = "00";
    MO = two(eMonth);
    DD = two(eDay);
    YY = String(eYear);

    if(uiMode == UI_EDIT_TIME_H)  HH = "[" + HH + "]";
    if(uiMode == UI_EDIT_TIME_M)  MM = "[" + MM + "]";
    if(uiMode == UI_EDIT_DATE_MO) MO = "[" + MO + "]";
    if(uiMode == UI_EDIT_DATE_D)  DD = "[" + DD + "]";
    if(uiMode == UI_EDIT_DATE_Y)  YY = "[" + YY + "]";
  }

  return "01 TIME " + HH + ":" + MM + ":" + SS + "  " + MO + "/" + DD + "/" + YY;
}

String buildRow4Idle(){
  char r4[120];

  if(uiMode == UI_EDIT_ROW4_SCALE){
    snprintf(r4, sizeof(r4), "04 SCL [%d%%] %s", eScale, calStateText().c_str());
  } else {
    snprintf(r4, sizeof(r4), "04 SCL %d%% %s", SCALE_PCT, calStateText().c_str());
  }

  return String(r4);
}

String buildRow5(){
  return "05 WINDOW 01:00-05:00";
}

// =====================================================
// LOAD / SAVE SETTINGS + REMEASURE
// =====================================================
void loadSettings(){
  prefs.begin("ws", true);

  TARGET = prefs.getInt("target", TARGET);
  DUR_MIN = prefs.getInt("durmin", DUR_MIN);
  SCALE_PCT = prefs.getInt("scale", SCALE_PCT);

  calDry = prefs.getInt("calDry", RAW_DRY_INIT);
  calWet = prefs.getInt("calWet", RAW_WET_INIT);

  int oldRawDry = prefs.getInt("rawdry", calDry);
  int oldRawWet = prefs.getInt("rawwet", calWet);

  prefs.end();

  TARGET = clampi(TARGET, TARGET_MIN, TARGET_MAX);
  DUR_MIN = clampi(DUR_MIN, 3, 10);
  SCALE_PCT = clampi(SCALE_PCT, SCALE_MIN, SCALE_MAX);

  calDry = clampi(calDry, 0, 4095);
  calWet = clampi(calWet, 0, 4095);
  oldRawDry = clampi(oldRawDry, 0, 4095);
  oldRawWet = clampi(oldRawWet, 0, 4095);

  if(calDry == RAW_DRY_INIT && oldRawDry != RAW_DRY_INIT) calDry = oldRawDry;
  if(calWet == RAW_WET_INIT && oldRawWet != RAW_WET_INIT) calWet = oldRawWet;

  if(calWet >= calDry){
    calWet = RAW_WET_INIT;
    calDry = RAW_DRY_INIT;
  }

  if((calDry - calWet) < 250){
    calDry = clampi(calWet + 250, 0, 4095);
  }

  calSyncDisplayedValues();
  calUpdateState();
}

void saveSettings(){
  prefs.begin("ws", false);
  prefs.putInt("target", TARGET);
  prefs.putInt("durmin", DUR_MIN);
  prefs.putInt("scale", SCALE_PCT);

  prefs.putInt("calDry", calDry);
  prefs.putInt("calWet", calWet);

  prefs.putInt("rawdry", calDry);
  prefs.putInt("rawwet", calWet);
  prefs.end();
}

void remeasureSoilNow(bool pushToHistory){
  if(isWetDryEditMode()){
    int raw = readSoilOnceCalFast(false);
    digitalWrite(PIN_SENSE_EN, LOW);
    senseIdle = true;
    processSoilMeasurement(raw, pushToHistory);
  } else {
    int raw = readSoilOncePulsed(false);
    processSoilMeasurement(raw, pushToHistory);
  }

  lastBarPct = -1;
  lastBarTgt = -1;
  lastBarHl  = false;

  lastLine[1] = "";
  lastLine[2] = "";
  lastLine[3] = "";
  lastLine[4] = "";
  lastLine[5] = "";
  lastLine[6] = "";
}

// =====================================================
// SETUP
// =====================================================
void setup(){
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_VALVE, LOW);

  pinMode(PIN_SENSE_EN, OUTPUT);
  digitalWrite(PIN_SENSE_EN, LOW);

  analogReadResolution(12);
  Wire.begin(I2C_SDA, I2C_SCL);

  gfx->begin();

  C_BLACK  = gfx->color565(0,0,0);
  C_WHITE  = gfx->color565(255,255,255);
  C_GREEN  = gfx->color565(0,255,0);
  C_YELLOW = gfx->color565(255,255,0);
  C_RED    = gfx->color565(255,0,0);
  C_CYAN   = gfx->color565(0,255,255);

  C_SPRING = gfx->color565(170,255,170);
  C_SUMMER = gfx->color565(255,170,170);
  C_AMBER  = gfx->color565(255,180,0);

  gfx->fillScreen(C_BLACK);
  gfx->setTextSize(2);

  for(int i=0;i<ROWS;i++){
    lastLine[i] = "";
    lastFg[i] = 0xFFFF;
  }

  lastBarPct = -1;
  lastBarTgt = -1;
  lastBarHl  = false;

  rtc.begin();

  sht4.begin();
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  btnUp.begin(PIN_UP);
  btnDown.begin(PIN_DOWN);
  btnEnter.begin(PIN_ENTER);

  loadSettings();

  int raw0 = readSoilOncePulsed(true);
  processSoilMeasurement(raw0, true);

  drawRowTextOnBlack(7, "  08 READY", C_GREEN);
}

// =====================================================
// LOOP
// =====================================================
void loop(){
  btnUp.update();
  btnDown.update();
  btnEnter.update();

  DateTime now = rtc.now();

  aiHandleWateringEdge(millis());

  if(uiMode == UI_EDIT_WET_DRY){
    if(millis() - calBlinkMs >= BLINK_MS){
      calBlinkMs = millis();
      calBlinkOn = !calBlinkOn;
      lastLine[1] = "";
    }
  } else {
    calBlinkOn = true;
  }

  if(!btnEnter.stable){
    calEnterHoldFired = false;
    calSaveHoldFired = false;

    if(calWaitReleaseAfterEnter){
      calWaitReleaseAfterEnter = false;
    }
  }

  static uint32_t lastSHTms = 0;
  if(millis() - lastSHTms >= 1000){
    lastSHTms = millis();

    sensors_event_t hum, temp;
    if(sht4.getEvent(&hum, &temp)){
      airT = temp.temperature;
      airRH = hum.relative_humidity;
    }

    bool newTempBlock = (isfinite(airT) && airT <= 12.0f);
    if(newTempBlock != tempBlock){
      tempBlock = newTempBlock;
      lastLine[2] = "";
      lastLine[5] = "";
    }

    lastLine[1] = "";
  }

  if(uiMode != UI_EDIT_WET_DRY){
    digitalWrite(PIN_SENSE_EN, LOW);
    senseIdle = true;

    if(isDailyReadTime(now)){
      uint32_t key = makeAutoKey(now);
      if(key != lastDailyKey){
        lastDailyKey = key;

        int raw = readSoilOncePulsed(false);
        processSoilMeasurement(raw, true);
      }
    }
  }

  if(uiMode == UI_NAV){

    if(btnUp.shortPress){
      selRow = clampi(selRow - 1, SEL_MIN, SEL_MAX);
      lastLine[0] = "";
      lastLine[1] = "";
      lastLine[2] = "";
      lastLine[3] = "";
      lastLine[4] = "";
    }

    if(btnDown.shortPress){
      selRow = clampi(selRow + 1, SEL_MIN, SEL_MAX);
      lastLine[0] = "";
      lastLine[1] = "";
      lastLine[2] = "";
      lastLine[3] = "";
      lastLine[4] = "";
    }

    if(btnEnter.shortPress){
      if(selRow == 0){
        loadEditFromRTC();
        uiMode = UI_EDIT_TIME_H;
        lastLine[0] = "";
      }
      else if(selRow == 2){
        uiMode = UI_EDIT_TARGET;
        lastLine[2] = "";
      }
      else if(selRow == 3){
        eScale = SCALE_PCT;
        uiMode = UI_EDIT_ROW4_SCALE;
        lastLine[3] = "";
      }
    }

    if(selRow == 1 && btnEnter.stable && !calEnterHoldFired){
      if((millis() - btnEnter.pressStartMs) >= CAL_ENTER_HOLD_MS){
        calEnterHoldFired = true;

        uiMode = UI_EDIT_WET_DRY;
        calEditField = CAL_FIELD_WET;
        eWet = calWet;
        eDry = calDry;
        calBlinkOn = true;
        calBlinkMs = millis();

        calWaitReleaseAfterEnter = true;
        calSaveHoldFired = false;

        digitalWrite(PIN_SENSE_EN, LOW);
        senseIdle = true;

        lastLine[1] = "";
        lastLine[2] = "";
        lastLine[6] = "";
      }
    }

    if(btnEnter.longPress && selRow != 1 && selRow != 2 && selRow != 3){
      if(watering) stopWatering();
      else startWatering(true, DUR_MIN);

      lastLine[3] = "";
      lastLine[5] = "";
    }
  }

  else if(uiMode == UI_EDIT_WET_DRY){

    if(!calWaitReleaseAfterEnter){

      if(btnUp.shortPress){
        if(calEditField == CAL_FIELD_WET){
          eWet = clampi(eWet + CAL_STEP, 0, eDry - 250);
        } else {
          eDry = clampi(eDry + CAL_STEP, eWet + 250, 4095);
        }
        lastLine[1] = "";
      }

      if(btnDown.shortPress){
        if(calEditField == CAL_FIELD_WET){
          eWet = clampi(eWet - CAL_STEP, 0, eDry - 250);
        } else {
          eDry = clampi(eDry - CAL_STEP, eWet + 250, 4095);
        }
        lastLine[1] = "";
      }

      if(btnEnter.shortPress){
        calEditField = (calEditField == CAL_FIELD_WET) ? CAL_FIELD_DRY : CAL_FIELD_WET;
        calBlinkOn = true;
        calBlinkMs = millis();
        lastLine[1] = "";
      }

      if(btnEnter.stable && !calSaveHoldFired){
        uint32_t held = millis() - btnEnter.pressStartMs;

        if(held >= CAL_SAVE_EXIT_MS){
          calSaveHoldFired = true;

          calWet = clampi(eWet, 0, 4095);
          calDry = clampi(eDry, 0, 4095);

          if((calDry - calWet) < 250){
            calDry = clampi(calWet + 250, 0, 4095);
          }

          calSyncDisplayedValues();
          calSaveNow();

          uiMode = UI_NAV;
          digitalWrite(PIN_SENSE_EN, LOW);
          senseIdle = true;

          remeasureSoilNow(true);

          lastLine[0] = "";
          lastLine[1] = "";
          lastLine[2] = "";
          lastLine[3] = "";
          lastLine[4] = "";
          lastLine[6] = "";
        }
      }
    }
  }

  else if(uiMode == UI_EDIT_TARGET){

    if(btnUp.shortPress){
      TARGET = clampi(TARGET + 1, TARGET_MIN, TARGET_MAX);
      saveSettings();
      lastLine[2] = "";
    }

    if(btnDown.shortPress){
      TARGET = clampi(TARGET - 1, TARGET_MIN, TARGET_MAX);
      saveSettings();
      lastLine[2] = "";
    }

    lastBarTgt = -1;

    if(btnEnter.shortPress){
      uiMode = UI_EDIT_DUR;
      lastLine[2] = "";
    }

    // longPress NAMERNO IGNORISAN na row 03
  }

  else if(uiMode == UI_EDIT_DUR){

    if(btnUp.shortPress){
      DUR_MIN = clampi(DUR_MIN + 1, 3, 10);
      saveSettings();
      lastLine[2] = "";
    }

    if(btnDown.shortPress){
      DUR_MIN = clampi(DUR_MIN - 1, 3, 10);
      saveSettings();
      lastLine[2] = "";
    }

    if(btnEnter.shortPress){
      uiMode = UI_NAV;
      lastLine[0] = "";
      lastLine[1] = "";
      lastLine[2] = "";
      lastLine[3] = "";
      lastLine[4] = "";
    }

    // longPress NAMERNO IGNORISAN na row 03
  }

  else if(uiMode == UI_EDIT_ROW4_SCALE){

    if(btnUp.shortPress){
      eScale = clampi(eScale + 1, SCALE_MIN, SCALE_MAX);
      lastLine[3] = "";
    }

    if(btnDown.shortPress){
      eScale = clampi(eScale - 1, SCALE_MIN, SCALE_MAX);
      lastLine[3] = "";
    }

    if(btnEnter.shortPress){
      SCALE_PCT = clampi(eScale, SCALE_MIN, SCALE_MAX);
      saveSettings();
      remeasureSoilNow(true);

      uiMode = UI_NAV;
      lastLine[1] = "";
      lastLine[2] = "";
      lastLine[3] = "";
      lastLine[4] = "";
      lastLine[6] = "";
    }

    if(btnEnter.longPress){
      uiMode = UI_NAV;
      lastLine[3] = "";
      lastLine[4] = "";
    }
  }

  else {
    if(btnEnter.longPress){
      uiMode = UI_NAV;
      lastLine[0] = "";
      lastLine[1] = "";
      lastLine[2] = "";
      lastLine[3] = "";
      lastLine[4] = "";
    } else {
      if(btnUp.shortPress || btnDown.shortPress){
        int dir = btnUp.shortPress ? +1 : -1;

        if(uiMode == UI_EDIT_TIME_H) eHour = (eHour + dir + 24) % 24;
        else if(uiMode == UI_EDIT_TIME_M) eMin = (eMin + dir + 60) % 60;
        else if(uiMode == UI_EDIT_DATE_MO){
          eMonth = clampi(eMonth + dir, 1, 12);
          normalizeDate();
        }
        else if(uiMode == UI_EDIT_DATE_D){
          int dim = daysInMonth(eYear, eMonth);
          eDay = clampi(eDay + dir, 1, dim);
        }
        else if(uiMode == UI_EDIT_DATE_Y){
          eYear = clampi(eYear + dir, 2020, 2099);
          normalizeDate();
        }

        lastLine[0] = "";
      }

      if(btnEnter.shortPress){
        if(uiMode == UI_EDIT_TIME_H) uiMode = UI_EDIT_TIME_M;
        else if(uiMode == UI_EDIT_TIME_M) uiMode = UI_EDIT_DATE_MO;
        else if(uiMode == UI_EDIT_DATE_MO) uiMode = UI_EDIT_DATE_D;
        else if(uiMode == UI_EDIT_DATE_D)  uiMode = UI_EDIT_DATE_Y;
        else if(uiMode == UI_EDIT_DATE_Y){
          saveEditToRTC();
          uiMode = UI_NAV;
          lastLine[0] = "";
          lastLine[1] = "";
          lastLine[2] = "";
          lastLine[3] = "";
          lastLine[4] = "";
        }
      }
    }
  }

  if(watering){
    if((int32_t)(wateringEndMs - millis()) <= 0){
      watering = false;
      wateringManual = false;
      digitalWrite(PIN_VALVE, LOW);

      lastLine[3] = "";
      lastLine[5] = "";

      int raw = readSoilOncePulsed(false);
      processSoilMeasurement(raw, true);
    }
  }

  if(!watering && !isWetDryEditMode()){
    if(now.hour() >= WINDOW_START_H && now.hour() <= WINDOW_END_H &&
       now.minute() == 0 && now.second() <= 10){

      uint32_t key = makeAutoKey(now);
      if(key != lastAutoKey){
        lastAutoKey = key;

        int raw = readSoilOncePulsed(false);
        processSoilMeasurement(raw, true);

        bool probeOpen = soilIsOpen(moistRaw);

        if(!tempBlock && !probeOpen){
          if(moistPct < TARGET){
            startWatering(false, DUR_MIN);
          }
        }

        lastLine[1] = "";
        lastLine[2] = "";
        lastLine[3] = "";
        lastLine[4] = "";
        lastLine[5] = "";
        lastLine[6] = "";
      }
    }
  }

  static int lastSec = -1;
  int secNow = now.second();

  if(secNow != lastSec || lastLine[0] == ""){
    lastSec = secNow;
    bool hl = rowIsHighlighted(0);
    drawRowTextOnBlack(0, withSelPrefix(buildTimeRow(now), hl), hl ? C_CYAN : seasonTimeColorByMonth(now.month()));
  }

  if(lastLine[1] == ""){
    bool hl = rowIsHighlighted(1);
    drawRow2Colored_NoFlicker(hl);
  } else {
    drawMoistBar_NoFlicker(rowIsHighlighted(1));
  }

  if(lastLine[2] == ""){
    bool hl = rowIsHighlighted(2);
    drawRow3Colored_NoFlicker(hl);
  }

  // =====================================================
  // FIX: ROW 04 countdown mora se crtati SVAKI LOOP dok watering traje
  // =====================================================
  {
    bool hl = rowIsHighlighted(3);

    if(watering && uiMode != UI_EDIT_ROW4_SCALE){
      int32_t remS = (int32_t)((wateringEndMs - millis()) / 1000);
      if(remS < 0) remS = 0;

      int mm = remS / 60;
      int ss = remS % 60;

      char r4[180];
      snprintf(r4, sizeof(r4),
               "04 WATER %s  %02d:%02d  SCL %d%% %s",
               wateringManual ? "MAN" : "AUTO",
               mm, ss, SCALE_PCT, calStateText().c_str());

      drawRowTextOnBlack(3, withSelPrefix(String(r4), hl), hl ? C_CYAN : C_RED);
    } else {
      if(lastLine[3] == ""){
        drawRowTextOnBlack(3, withSelPrefix(buildRow4Idle(), hl), hl ? C_CYAN : C_GREEN);
      } else {
        drawRowTextOnBlack(3, withSelPrefix(buildRow4Idle(), hl), hl ? C_CYAN : C_GREEN);
      }
    }
  }

  if(lastLine[4] == ""){
    bool hl = rowIsHighlighted(4);
    drawRowTextOnBlack(4, withSelPrefix(buildRow5(), hl), hl ? C_CYAN : C_GREEN);
  }

  if(lastLine[5] == ""){
    String r6 = String("  06 VALVE ") + (watering ? "ON" : "OFF") +
                (tempBlock ? " (TEMP BLOCK)" : "") +
                (soilIsOpen(moistRaw) ? " (PROBE OPEN)" : "");
    drawRowTextOnBlack(5, r6, watering ? C_RED : C_WHITE);
  }

  if(lastLine[6] == ""){
    char r7[180];
    const char *st = senseIdle ? "IDLE" : "MEAS";
    const char *tag =
      soilIsOpen(moistRaw) ? " (OPEN)" :
      (soilIsShort(moistRaw) ? " (SHORT)" : "");

    snprintf(r7, sizeof(r7), "  07 SENSE %s  RAW %d%s", st, moistRaw, tag);
    drawRowTextOnBlack(6, String(r7), soilIsOpen(moistRaw) ? C_AMBER : C_WHITE);
  }

  drawRowTextOnBlack(7, "  08 READY", C_GREEN);

  delay(10);
}