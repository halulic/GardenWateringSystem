#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_SHT4x.h>
#include <Arduino_GFX_Library.h>
#include <Preferences.h>
#include <math.h>

// ------------------- PINS (WS FINAL) -------------------
#define TFT_CS   5
#define TFT_DC   27
#define TFT_RST  26
#define TFT_SCK  18
#define TFT_MOSI 23
#define TFT_MISO -1

#define I2C_SDA  21
#define I2C_SCL  22

#define PIN_UP      35   // GPIO35 nema interni pullup -> tvoj 10k na 3.3V je OK
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

// ------------------- NVS (save settings) -------------------
Preferences prefs;

// ------------------- UI GRID -------------------
static const int ROWS = 8;
static const int ROW_H = 40;
static const int LEFT = 8;

// ------------------- SETTINGS -------------------
int TARGET = 50;     // 20..80 %
int DUR_MIN = 5;     // 3..10 min (fiksna duzina zalijevanja)

// AUTO window schedule: 01:00, 02:00, 03:00, 04:00, 05:00
static const int WINDOW_START_H = 1;
static const int WINDOW_END_H   = 5;

// dnevna merenja vlage: 06:00, 12:00, 18:00
static const int DAILY_READ_H1 = 6;
static const int DAILY_READ_H2 = 12;
static const int DAILY_READ_H3 = 18;

// ručni edit korak za DRY/WET
static const int CAL_STEP = 25;

// ULaz u CAL iz reda 02 (ENTER hold >= 5s)
static const uint32_t CAL_ENTER_HOLD_MS = 5000;

// ------------------- SOIL RAW DETECT (OPEN/SHORT) -------------------
#define SOIL_RAW_SHORT_MAX   60
#define SOIL_RAW_OPEN_MIN    3850   // <-- FINAL (manje laznih OPEN)

// Kalibracija po zemlji (FINALNO izmereno)
int RAW_DRY = 3488;  // 0%
int RAW_WET = 1129;  // 100%

// ------------------- STATE -------------------
float airT = NAN;
float airRH = NAN;

int moistRaw = 0;
int moistPct = 0;
bool senseIdle = true;

bool tempBlock = false;

bool watering = false;
bool wateringManual = false;
uint32_t wateringEndMs = 0;

// Anti-flicker cache
String   lastLine[ROWS];
uint16_t lastFg[ROWS];

// AUTO slot guard (da ne krene vise puta u istom satu)
uint32_t lastAutoKey = 0; // yyyymmddhh

// dnevna merenja guard (da ne okida vise puta)
uint32_t lastDailyKey = 0; // yyyymmddhh

// boot CAL flag
bool calBoot = false;

// CAL enter hold guard (da ne okida više puta dok držiš)
bool calEnterHoldFired = false;

// ------------------- BARGRAPH CACHE (ONLY FOR M) -------------------
int  lastBarPct = -1;
int  lastBarTgt = -1;
bool lastBarHl  = false;

// ------------------- HELPERS -------------------
int clampi(int x, int lo, int hi){ if(x<lo) return lo; if(x>hi) return hi; return x; }
String two(int v){ char b[8]; snprintf(b, sizeof(b), "%02d", v); return String(b); }

bool soilIsShort(int raw){ return (raw <= SOIL_RAW_SHORT_MAX); }
bool soilIsOpen (int raw){ return (raw >= SOIL_RAW_OPEN_MIN); }

uint32_t makeAutoKey(const DateTime &t){
  return (uint32_t)t.year() * 1000000UL + (uint32_t)t.month() * 10000UL + (uint32_t)t.day() * 100UL + (uint32_t)t.hour();
}

int mapMoistPctCal(int raw){
  raw = clampi(raw, 0, 4095);

  if(soilIsShort(raw)) return 100;
  if(soilIsOpen(raw))  return 0;

  int dry = RAW_DRY;
  int wet = RAW_WET;
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

// ---- Normal (pulsed) soil read ----
int readSoilOncePulsed(){
  senseIdle = false;
  digitalWrite(PIN_SENSE_EN, HIGH);
  delay(8);

  const int N = 12;
  long sum = 0;
  for(int i=0;i<N;i++){
    sum += analogRead(PIN_SENSE_ADC);
    delay(2);
  }

  digitalWrite(PIN_SENSE_EN, LOW);
  senseIdle = true;

  return (int)(sum / N);
}

// ---- CAL (continuous) soil read (SENSE_EN stays ON) ----
int readSoilOnceCalFast(){
  senseIdle = false;
  digitalWrite(PIN_SENSE_EN, HIGH);

  const int N = 12;
  long sum = 0;
  for(int i=0;i<N;i++){
    sum += analogRead(PIN_SENSE_ADC);
    delay(2);
  }
  senseIdle = true;
  return (int)(sum / N);
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

// ------------------- BUTTONS (debounce + short/long) -------------------
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
    stable = r; raw = r; lastChangeMs = millis();
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

// ------------------- UI STATE -------------------
enum UiMode {
  UI_NAV,
  UI_EDIT_TIME_H, UI_EDIT_TIME_M, UI_EDIT_DATE_MO, UI_EDIT_DATE_D, UI_EDIT_DATE_Y,
  UI_EDIT_TARGET, UI_EDIT_DUR,
  UI_CAL,
  UI_EDIT_CAL_DRY,
  UI_EDIT_CAL_WET
};

UiMode uiMode = UI_NAV;
int selRow = 0;                 // 0=TIME, 1=MOIST, 2=TARGET/DUR
const int SEL_MIN = 0;
const int SEL_MAX = 2;

// Edit buffers for RTC
int eHour=0, eMin=0, eDay=1, eMonth=1, eYear=2026;

// Edit buffers for manual DRY/WET
int eDry = 0;
int eWet = 0;

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
  if((uiMode == UI_EDIT_TARGET || uiMode == UI_EDIT_DUR) && r == 2) return true;
  if((uiMode == UI_CAL || uiMode == UI_EDIT_CAL_DRY || uiMode == UI_EDIT_CAL_WET) && r == 1) return true;
  return false;
}

uint16_t seasonTimeColorByMonth(int month){
  if(month == 12 || month == 1 || month == 2) return C_WHITE;
  if(month >= 3 && month <= 5) return C_SPRING;
  if(month >= 6 && month <= 8) return C_SUMMER;
  return C_AMBER;
}

// =====================================================
// UI COLOR HELPERS — ONLY UI (NO LOGIC CHANGES)
// =====================================================
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
  if(tC < 36.0f) return gfx->color565(255, 170, 170);
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
// BAR GRAPH — ONLY FOR SOIL MOISTURE (M)
// CURRENT: CRVENA (vidljiva uvek, pomerena kad je 0%/100%)
// TARGET: BELA (ako se poklope -> TARGET pored CURRENT)
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

  // TARGET position
  int mx = innerX + (innerW * tgt) / 100;
  mx = clampi(mx, innerX, innerX + innerW - 1);

  // CURRENT position (kraj popunjenog dela)
  int cx = (fillW <= 0) ? innerX : (innerX + fillW - 1);
  cx = clampi(cx, innerX, innerX + innerW - 1);

  // KLJUČNA IZMJENA:
  // Ako je 0% ili 100%, marker ne sme da stoji uz okvir (tamo “nestane”)
  if(pct == 0 && innerW >= 4)  cx = innerX + 2;
  if(pct == 100 && innerW >= 4) cx = innerX + innerW - 3;

  // Ako se TARGET i CURRENT poklope, TARGET ide pored CURRENT
  if(abs(mx - cx) <= 1){
    int tpos = cx + 3; // desno
    if(tpos > innerX + innerW - 2) tpos = cx - 3; // levo
    if(tpos < innerX + 1) tpos = cx; // fallback
    mx = tpos;
  }

  // TARGET marker (bela, 2px)
  gfx->drawFastVLine(mx, innerY, innerH, targetCol);
  if(mx + 1 <= innerX + innerW - 1) gfx->drawFastVLine(mx + 1, innerY, innerH, targetCol);

  // CURRENT marker (crvena, 3px + crni outline da se vidi uvek)
  // outline
  if(cx - 2 >= innerX) gfx->drawFastVLine(cx - 2, innerY, innerH, C_BLACK);
  gfx->drawFastVLine(cx + 2 <= innerX + innerW - 1 ? cx + 2 : cx, innerY, innerH, C_BLACK);

  // core 3px
  if(cx - 1 >= innerX) gfx->drawFastVLine(cx - 1, innerY, innerH, currentCol);
  gfx->drawFastVLine(cx, innerY, innerH, currentCol);
  if(cx + 1 <= innerX + innerW - 1) gfx->drawFastVLine(cx + 1, innerY, innerH, currentCol);
}

// -----------------------------------------------------
// ANTI-FLICKER ROW 02 TEXT (SHT41 + M tekst ostaje)
// -----------------------------------------------------
void drawRow2Colored_NoFlicker(bool hl){
  double t = isfinite(airT) ? (double)airT : 0.0;
  int rh = isfinite(airRH) ? (int)lroundf(airRH) : 0;

  char key[160];
  snprintf(key, sizeof(key), "R2NF|hl=%d|T=%.1f|RH=%d|M=%d|raw=%d|mode=%d",
           hl?1:0, t, rh, moistPct, moistRaw, (int)uiMode);
  String k = String(key);

  if(lastLine[1] == k){
    drawMoistBar_NoFlicker(hl);
    return;
  }
  lastLine[1] = k;

  int y = 1 * ROW_H;
  gfx->setCursor(LEFT, y + 10);

  uint16_t base = hl ? C_CYAN : (soilIsOpen(moistRaw) ? C_AMBER : C_WHITE);
  uint16_t colT  = colorTempBands(airT);
  uint16_t colRH = colorRH(airRH);
  uint16_t colM  = colorSoilPctBands(moistPct);

  int printed = 0;

  gfx->setTextColor(base, C_BLACK);
  if(hl){ gfx->print("> "); printed += 2; }
  else  { gfx->print("  "); printed += 2; }

  gfx->print("02 "); printed += 3;

  gfx->setTextColor(colT, C_BLACK);
  char tbuf[16];
  snprintf(tbuf, sizeof(tbuf), "T:%.1fC", t);
  gfx->print(tbuf);
  printed += strlen(tbuf);

  gfx->setTextColor(base, C_BLACK);
  gfx->print(" "); printed += 1;

  gfx->setTextColor(colRH, C_BLACK);
  char rhbuf[16];
  snprintf(rhbuf, sizeof(rhbuf), "RH:%d%%", rh);
  gfx->print(rhbuf);
  printed += strlen(rhbuf);

  gfx->setTextColor(base, C_BLACK);
  gfx->print(" "); printed += 1;

  gfx->setTextColor(colM, C_BLACK);
  char mbuf[16];
  snprintf(mbuf, sizeof(mbuf), "M:%d%%", moistPct);
  gfx->print(mbuf);
  printed += strlen(mbuf);

  gfx->setTextColor(base, C_BLACK);
  while(printed < 34){
    gfx->print(" ");
    printed++;
  }

  drawMoistBar_NoFlicker(hl);
}

void drawRow3Colored_NoFlicker(bool hl){
  bool probeBlock = soilIsOpen(moistRaw);

  char key[220];
  snprintf(key, sizeof(key), "R3NF|hl=%d|TGT=%d|DUR=%d|tb=%d|po=%d",
           hl?1:0, TARGET, DUR_MIN, tempBlock?1:0, probeBlock?1:0);
  String k = String(key);
  if(lastLine[2] == k) return;
  lastLine[2] = k;

  int y = 2 * ROW_H;
  gfx->setCursor(LEFT, y + 10);

  uint16_t base = hl ? C_CYAN : (tempBlock ? C_YELLOW : C_WHITE);
  uint16_t colTgt = colorSoilPctBands(TARGET);

  int printed = 0;

  gfx->setTextColor(base, C_BLACK);
  if(hl){ gfx->print("> "); printed += 2; }
  else  { gfx->print("  "); printed += 2; }

  gfx->print("03 "); printed += 3;

  gfx->setTextColor(colTgt, C_BLACK);
  char tgbuf[24];
  snprintf(tgbuf, sizeof(tgbuf), "TARGET %d%%", TARGET);
  gfx->print(tgbuf);
  printed += strlen(tgbuf);

  gfx->setTextColor(base, C_BLACK);
  char rest[80];
  snprintf(rest, sizeof(rest), "  DUR %02d:00", DUR_MIN);
  gfx->print(rest);
  printed += strlen(rest);

  if(tempBlock){
    const char *tb = "  BLOCK<=12C";
    gfx->print(tb); printed += strlen(tb);
  }
  if(probeBlock){
    const char *pb = "  PROBE OPEN";
    gfx->print(pb); printed += strlen(pb);
  }

  while(printed < 34){
    gfx->print(" ");
    printed++;
  }
}

// ------------------- ROW BUILDERS -------------------
String buildTimeRow(const DateTime &now){
  String HH = two(now.hour());
  String MM = two(now.minute());
  String SS = two(now.second());
  String MO = two(now.month());
  String DD = two(now.day());
  String YY = String(now.year());

  if(uiMode != UI_NAV && uiMode != UI_EDIT_TARGET && uiMode != UI_EDIT_DUR &&
     uiMode != UI_CAL && uiMode != UI_EDIT_CAL_DRY && uiMode != UI_EDIT_CAL_WET){
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

String buildRow2(){
  char r2[180];

  if(uiMode == UI_EDIT_CAL_DRY){
    snprintf(r2, sizeof(r2), "02 EDIT DRY %d  step %d", eDry, CAL_STEP);
    return String(r2);
  }
  if(uiMode == UI_EDIT_CAL_WET){
    snprintf(r2, sizeof(r2), "02 EDIT WET %d  step %d", eWet, CAL_STEP);
    return String(r2);
  }
  if(uiMode == UI_CAL){
    snprintf(r2, sizeof(r2), "02 CAL D=%d W=%d", RAW_DRY, RAW_WET);
    return String(r2);
  }

  double t = isfinite(airT) ? (double)airT : 0.0;
  int rh = isfinite(airRH) ? (int)lroundf(airRH) : 0;

  snprintf(r2, sizeof(r2), "02 T:%.1fC RH:%d%% M:%d%%", t, rh, moistPct);
  return String(r2);
}

String buildRow3(){
  char r3[200];
  bool probeBlock = soilIsOpen(moistRaw);
  snprintf(r3, sizeof(r3), "03 TARGET %d%%  DUR %02d:00%s%s",
           TARGET, DUR_MIN,
           tempBlock ? "  BLOCK<=12C" : "",
           probeBlock ? "  PROBE OPEN" : "");
  return String(r3);
}

// ------------------- LOAD/SAVE SETTINGS -------------------
void loadSettings(){
  prefs.begin("ws", true);
  TARGET = prefs.getInt("target", TARGET);
  DUR_MIN = prefs.getInt("durmin", DUR_MIN);
  RAW_DRY = prefs.getInt("rawdry", RAW_DRY);
  RAW_WET = prefs.getInt("rawwet", RAW_WET);
  prefs.end();

  TARGET = clampi(TARGET, 20, 80);
  DUR_MIN = clampi(DUR_MIN, 3, 10);
  RAW_DRY = clampi(RAW_DRY, 0, 4095);
  RAW_WET = clampi(RAW_WET, 0, 4095);
}

void saveSettings(){
  prefs.begin("ws", false);
  prefs.putInt("target", TARGET);
  prefs.putInt("durmin", DUR_MIN);
  prefs.putInt("rawdry", RAW_DRY);
  prefs.putInt("rawwet", RAW_WET);
  prefs.end();
}

// ------------------- SETUP -------------------
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

  C_SPRING = gfx->color565(170, 255, 170);
  C_SUMMER = gfx->color565(255, 170, 170);
  C_AMBER  = gfx->color565(255, 180, 0);

  gfx->fillScreen(C_BLACK);
  gfx->setTextSize(2);

  for(int i=0;i<ROWS;i++){
    lastLine[i] = "";
    lastFg[i] = 0xFFFF;
  }

  // bar cache reset
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

  // ---- BOOT CAL MODE (hold ENTER while power on) ----
  if(digitalRead(PIN_ENTER) == LOW){
    calBoot = true;
    uiMode = UI_CAL;
    digitalWrite(PIN_SENSE_EN, HIGH);
    lastLine[0] = ""; lastLine[1] = ""; lastLine[2] = ""; lastLine[6] = "";
  }

  // ---- INITIAL SOIL READ ----
  if(!calBoot){
    moistRaw = readSoilOncePulsed();
    moistPct = mapMoistPctCal(moistRaw);
    lastLine[1] = "";
    lastLine[2] = "";
    lastLine[6] = "";
  }

  drawRowTextOnBlack(4, "  05 WINDOW 01:00-05:00", C_GREEN);
  drawRowTextOnBlack(7, "  08 READY", C_GREEN);
}

// ------------------- AUTO SLOT CHECK -------------------
bool isDailyReadTime(const DateTime &t){
  if(t.minute() != 0) return false;
  if(t.second() > 10) return false;
  int h = t.hour();
  return (h == DAILY_READ_H1 || h == DAILY_READ_H2 || h == DAILY_READ_H3);
}

bool isCalMode(){
  return (uiMode == UI_CAL || uiMode == UI_EDIT_CAL_DRY || uiMode == UI_EDIT_CAL_WET);
}

// ------------------- LOOP -------------------
void loop(){
  btnUp.update();
  btnDown.update();
  btnEnter.update();

  DateTime now = rtc.now();

  if(!btnEnter.stable) calEnterHoldFired = false;
  if(uiMode != UI_NAV || selRow != 1) calEnterHoldFired = false;

  // ---- SHT41 every 1s ----
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

  // ---- SOIL READING CONTROL ----
  static uint32_t lastSoilMs = 0;

  if(isCalMode()){
    if(millis() - lastSoilMs >= 250){
      lastSoilMs = millis();
      moistRaw = readSoilOnceCalFast();
      moistPct = mapMoistPctCal(moistRaw);
      lastLine[1] = "";
      lastLine[2] = "";
      lastLine[6] = "";
    }
  } else {
    digitalWrite(PIN_SENSE_EN, LOW);

    if(isDailyReadTime(now)){
      uint32_t key = makeAutoKey(now);
      if(key != lastDailyKey){
        lastDailyKey = key;

        moistRaw = readSoilOncePulsed();
        moistPct = mapMoistPctCal(moistRaw);

        lastLine[1] = "";
        lastLine[2] = "";
        lastLine[6] = "";
      }
    }
  }

  // ---------------- UI LOGIC ----------------
  if(uiMode == UI_NAV){

    if(btnUp.shortPress){
      selRow = clampi(selRow - 1, 0, 2);
      lastLine[0] = ""; lastLine[1] = ""; lastLine[2] = "";
    }
    if(btnDown.shortPress){
      selRow = clampi(selRow + 1, 0, 2);
      lastLine[0] = ""; lastLine[1] = ""; lastLine[2] = "";
    }

    if(btnEnter.shortPress){
      if(selRow == 0){
        loadEditFromRTC();
        uiMode = UI_EDIT_TIME_H;
        lastLine[0] = ""; lastLine[1] = ""; lastLine[2] = "";
      } else if(selRow == 2){
        uiMode = UI_EDIT_TARGET;
        lastLine[2] = ""; lastLine[0] = ""; lastLine[1] = "";
      }
    }

    if(selRow == 1 && btnEnter.stable && !calEnterHoldFired){
      if((millis() - btnEnter.pressStartMs) >= CAL_ENTER_HOLD_MS){
        calEnterHoldFired = true;
        uiMode = UI_CAL;
        digitalWrite(PIN_SENSE_EN, HIGH);
        lastLine[1] = "";
        lastLine[2] = "";
        lastLine[6] = "";
      }
    }

    if(btnEnter.longPress && selRow != 1){
      if(watering) stopWatering();
      else startWatering(true, DUR_MIN);
      lastLine[3] = "";
      lastLine[5] = "";
    }
  }
  else if(uiMode == UI_CAL){
    if(btnUp.shortPress){
      RAW_DRY = moistRaw;
      saveSettings();
      lastLine[1] = "";
      lastLine[2] = "";
    }
    if(btnDown.shortPress){
      RAW_WET = moistRaw;
      saveSettings();
      lastLine[1] = "";
      lastLine[2] = "";
    }

    if(btnUp.longPress){
      eDry = RAW_DRY;
      uiMode = UI_EDIT_CAL_DRY;
      lastLine[1] = "";
    }
    if(btnDown.longPress){
      eWet = RAW_WET;
      uiMode = UI_EDIT_CAL_WET;
      lastLine[1] = "";
    }

    if(btnEnter.shortPress){
      uiMode = UI_NAV;
      calBoot = false;
      digitalWrite(PIN_SENSE_EN, LOW);
      lastLine[0] = ""; lastLine[1] = ""; lastLine[2] = ""; lastLine[6] = "";
    }
  }
  else if(uiMode == UI_EDIT_CAL_DRY){
    if(btnUp.shortPress)   { eDry = clampi(eDry + CAL_STEP, 0, 4095); lastLine[1] = ""; }
    if(btnDown.shortPress) { eDry = clampi(eDry - CAL_STEP, 0, 4095); lastLine[1] = ""; }

    if(btnEnter.shortPress){
      RAW_DRY = eDry;
      saveSettings();
      uiMode = UI_CAL;
      lastLine[1] = ""; lastLine[2] = "";
    }
    if(btnEnter.longPress){
      uiMode = UI_CAL;
      lastLine[1] = "";
    }
  }
  else if(uiMode == UI_EDIT_CAL_WET){
    if(btnUp.shortPress)   { eWet = clampi(eWet + CAL_STEP, 0, 4095); lastLine[1] = ""; }
    if(btnDown.shortPress) { eWet = clampi(eWet - CAL_STEP, 0, 4095); lastLine[1] = ""; }

    if(btnEnter.shortPress){
      RAW_WET = eWet;
      saveSettings();
      uiMode = UI_CAL;
      lastLine[1] = ""; lastLine[2] = "";
    }
    if(btnEnter.longPress){
      uiMode = UI_CAL;
      lastLine[1] = "";
    }
  }
  else if(uiMode == UI_EDIT_TARGET){
    if(btnUp.shortPress)   { TARGET = clampi(TARGET + 1, 20, 80); saveSettings(); lastLine[2] = ""; lastLine[1] = ""; }
    if(btnDown.shortPress) { TARGET = clampi(TARGET - 1, 20, 80); saveSettings(); lastLine[2] = ""; lastLine[1] = ""; }

    lastBarTgt = -1;

    if(btnEnter.shortPress){ uiMode = UI_EDIT_DUR; lastLine[2] = ""; }
    if(btnEnter.longPress) { uiMode = UI_NAV; lastLine[2] = ""; lastLine[0] = ""; lastLine[1] = ""; }
  }
  else if(uiMode == UI_EDIT_DUR){
    if(btnUp.shortPress)   { DUR_MIN = clampi(DUR_MIN + 1, 3, 10); saveSettings(); lastLine[2] = ""; }
    if(btnDown.shortPress) { DUR_MIN = clampi(DUR_MIN - 1, 3, 10); saveSettings(); lastLine[2] = ""; }

    if(btnEnter.shortPress || btnEnter.longPress){
      uiMode = UI_NAV;
      lastLine[2] = ""; lastLine[0] = ""; lastLine[1] = "";
    }
  }
  else {
    if(btnEnter.longPress){
      uiMode = UI_NAV;
      lastLine[0] = ""; lastLine[1] = ""; lastLine[2] = "";
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
          lastLine[1] = ""; lastLine[2] = "";
        }
        lastLine[0] = "";
      }
    }
  }

  // ---------------- WATER STOP ----------------
  if(watering){
    if((int32_t)(wateringEndMs - millis()) <= 0){
      watering = false;
      wateringManual = false;
      digitalWrite(PIN_VALVE, LOW);

      lastLine[3] = "";
      lastLine[5] = "";

      moistRaw = readSoilOncePulsed();
      moistPct = mapMoistPctCal(moistRaw);
      lastLine[1] = ""; lastLine[2] = ""; lastLine[6] = "";
    }
  }

  // ---------------- AUTO WATERING (01..05) ----------------
  if(!watering && !isCalMode()){
    if(now.hour() >= WINDOW_START_H && now.hour() <= WINDOW_END_H && now.minute() == 0 && now.second() <= 10){
      uint32_t key = makeAutoKey(now);
      if(key != lastAutoKey){
        lastAutoKey = key;

        moistRaw = readSoilOncePulsed();
        moistPct = mapMoistPctCal(moistRaw);

        bool probeOpen = soilIsOpen(moistRaw);

        if(!tempBlock && !probeOpen){
          if(moistPct < TARGET){
            startWatering(false, DUR_MIN);
          }
        }

        lastLine[1] = "";
        lastLine[2] = "";
        lastLine[3] = "";
        lastLine[5] = "";
        lastLine[6] = "";
      }
    }
  }

  // ---------------- UI DRAW ----------------
  static int lastSec = -1;
  int secNow = now.second();
  if(secNow != lastSec || lastLine[0] == ""){
    lastSec = secNow;

    bool hl = rowIsHighlighted(0);
    uint16_t fg = hl ? C_CYAN : seasonTimeColorByMonth(now.month());
    String r1 = withSelPrefix(buildTimeRow(now), hl);
    drawRowTextOnBlack(0, r1, fg);
  }

  // Row 02
  if(lastLine[1] == ""){
    bool hl = rowIsHighlighted(1);

    if(uiMode == UI_CAL || uiMode == UI_EDIT_CAL_DRY || uiMode == UI_EDIT_CAL_WET){
      uint16_t fg = hl ? C_CYAN : (soilIsOpen(moistRaw) ? C_AMBER : C_WHITE);
      String r2 = withSelPrefix(buildRow2(), hl);
      drawRowTextOnBlack(1, r2, fg);
    } else {
      drawRow2Colored_NoFlicker(hl);
    }
  } else {
    if(!(uiMode == UI_CAL || uiMode == UI_EDIT_CAL_DRY || uiMode == UI_EDIT_CAL_WET)){
      drawMoistBar_NoFlicker(rowIsHighlighted(1));
    }
  }

  // Row 03
  if(lastLine[2] == ""){
    bool hl = rowIsHighlighted(2);
    drawRow3Colored_NoFlicker(hl);
  }

  static uint32_t lastCountMs = 0;
  if(watering){
    if(millis() - lastCountMs >= 1000 || lastLine[3] == ""){
      lastCountMs = millis();
      int32_t remS = (int32_t)((wateringEndMs - millis()) / 1000);
      if(remS < 0) remS = 0;
      int mm = remS / 60;
      int ss = remS % 60;

      char r4[140];
      snprintf(r4, sizeof(r4), "  04 WATER %s  %02d:%02d  M=%d%%",
               wateringManual ? "MAN" : "AUTO", mm, ss, moistPct);
      drawRowTextOnBlack(3, String(r4), C_RED);
    }
  } else {
    drawRowTextOnBlack(3, "  04 AIR OK", C_GREEN);
  }

  drawRowTextOnBlack(4, "  05 WINDOW 01:00-05:00", C_GREEN);

  if(lastLine[5] == ""){
    String r6 = String("  06 VALVE ") + (watering ? "ON" : "OFF") +
                (tempBlock ? " (TEMP BLOCK)" : "") +
                (soilIsOpen(moistRaw) ? " (PROBE OPEN)" : "");
    drawRowTextOnBlack(5, r6, watering ? C_RED : C_WHITE);
  }

  if(lastLine[6] == ""){
    char r7[160];
    const char *st = senseIdle ? "IDLE" : "MEAS";
    const char *tag = soilIsOpen(moistRaw) ? " (OPEN)" : (soilIsShort(moistRaw) ? " (SHORT)" : "");
    snprintf(r7, sizeof(r7), "  07 SENSE %s  RAW %d%s", st, moistRaw, tag);
    drawRowTextOnBlack(6, String(r7), soilIsOpen(moistRaw) ? C_AMBER : C_WHITE);
  }

  drawRowTextOnBlack(7, "  08 READY", C_GREEN);

  delay(10);
}