/*
 * קלפי חכמה - גרסת חומרה
 * תרגיל בית מעשי A - ארדואינו מפעילים וחיישנים
 *
 * רכיבים: LCD 16x2 I2C, שתי נורות LED, כפתור, חיישן מרחק HC-SR04,
 *          מקלט + שלט IR, מנוע סרוו 360°, תא עומס + מגבר HX711 (משקל דיגיטלי).
 * ספריות: Servo, IRremote (4.x), HX711 (bogde), LiquidCrystal_I2C.
 * לפני השימוש: להריץ את kalpi_scale_calibration.ino ולעדכן את SCALE_FACTOR.
 */
#include <Servo.h>
#include <IRremote.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// מצבי המערכת (מוגדר כאן למעלה כדי ש-Arduino IDE יכיר אותו בכל הקובץ)
enum State { ST_CLOSED, ST_IDLE, ST_WAIT_ID, ST_SLOT_OPEN, ST_MESSAGE };

// ===================== הגדרות פינים =====================
const uint8_t PIN_BUTTON    = 2;   // כפתור מנהל קלפי (INPUT_PULLUP)
const uint8_t PIN_IR        = 3;   // מקלט IR
const uint8_t PIN_TRIG      = 6;   // חיישן מרחק אולטרסוני - TRIG
const uint8_t PIN_ECHO      = 7;   // חיישן מרחק אולטרסוני - ECHO
const uint8_t PIN_SERVO     = 9;   // מנוע סרוו - חריץ הקלפי
const uint8_t PIN_LED_GREEN = 10;  // נורה ירוקה - אישור (דרך נגד 1kΩ)
const uint8_t PIN_LED_RED   = 11;  // נורה אדומה - דחייה / קלפי סגורה (דרך נגד 1kΩ)

// ===================== קבועי מערכת =====================
const int      PRESENCE_CM        = 50;     // מרחק שמתחתיו מצביע נחשב "נוכח"
const float    ENVELOPE_MIN_G     = 2.0;    // תוספת משקל מינימלית שנחשבת מעטפה
const unsigned long SETTLE_MS       = 1000;  // התעלמות מהמשקל בזמן שהסרוו זז (רעידות)
const uint8_t  CONFIRM_READS      = 3;      // כמה קריאות רצופות מעל הסף כדי לאשר מעטפה
const unsigned long ID_TIMEOUT_MS   = 20000; // זמן מקסימלי בלי הקשה בזמן הזנת ת"ז
const unsigned long SLOT_TIMEOUT_MS = 20000; // זמן מקסימלי להטלת מעטפה
const unsigned long MSG_SHOW_MS     = 10000;  // משך הצגת הודעת תוצאה
const unsigned long DEBOUNCE_MS     = 50;    // סינון רעידות כפתור
const unsigned long IR_REPEAT_MS    = 300;   // התעלמות מאות חוזר כשמחזיקים מקש בשלט
const unsigned long SENSE_PERIOD_MS = 200;   // מחזור דגימת חיישן מרחק

const uint8_t ID_LEN   = 9;        // מספר תעודת זהות: 9 ספרות
const int     KEY_CLEAR = -2;       // מקש מחיקה בשלט (מוחזר מ-irCommandToDigit)

// פנקס הבוחרים: מספרי ת"ז הרשומים בקלפי זו (מספרים בדויים לבדיקה).
// כל מספר אחר שיוקש ייחשב "לא רשום".
const uint8_t NUM_VOTERS = 8;
const char* const VOTER_IDS[NUM_VOTERS] = {
  "123456789", "987654321", "111222333", "204060801",
  "555666777", "314159265", "246813579", "135792468"
};

// ===================== רכיבים ייחודיים לחומרה =====================
const uint8_t PIN_HX_DT  = 4;     // HX711 DOUT
const uint8_t PIN_HX_SCK = 5;     // HX711 SCK
const float   SCALE_FACTOR = 446.0;  // calibrated: raw 44600 for 100 g

HX711 scale;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // אם המסך ריק נסו 0x3F

void lcdInit()  { lcd.init(); lcd.backlight(); }
void lcdClear() { lcd.clear(); }
void lcdLine(uint8_t row, const char* text) { lcd.setCursor(0, row); lcd.print(text); }

// ----- סרוו 360° (סיבוב רציף) מהערכה -----
// בסרוו 360 הפקודה write() קובעת מהירות וכיוון, לא זווית:
// 90 = עצירה, מעל 90 = סיבוב לכיוון אחד, מתחת ל-90 = לכיוון השני.
// לכן פותחים/סוגרים את הדש ע"י סיבוב לזמן קצוב ואז עצירה.
const uint8_t SERVO_STOP        = 90;   // אם הסרוו "זוחל" במנוחה - לכוונן 88..92
const uint8_t SERVO_OPEN_SPEED  = 110;  // כיוון פתיחה
const uint8_t SERVO_CLOSE_SPEED = 70;   // כיוון סגירה
const unsigned long SLOT_TURN_MS = 350; // משך הסיבוב - לכוונן לפי רבע סיבוב בפועל
Servo slotServo;
bool slotIsOpen = false;                // בהדלקה הדש חייב להיות במצב סגור
bool servoMoving = false;
unsigned long servoStopAt = 0;

void slotInit() { slotServo.attach(PIN_SERVO); slotServo.write(SERVO_STOP); }

void setSlot(bool open) {
  if (open == slotIsOpen) return;       // כבר במצב הנכון - לא מסובבים שוב
  slotIsOpen = open;
  slotServo.write(open ? SERVO_OPEN_SPEED : SERVO_CLOSE_SPEED);
  servoMoving = true;
  servoStopAt = millis() + SLOT_TURN_MS;
}

// נקרא בכל loop: עוצר את הסרוו כשנגמר זמן הסיבוב
void slotUpdate() {
  if (servoMoving && (long)(millis() - servoStopAt) >= 0) {
    slotServo.write(SERVO_STOP);
    servoMoving = false;
  }
}

bool scaleOk = false;              // האם ה-HX711 מחובר ועונה

void weightInit() {
  scale.begin(PIN_HX_DT, PIN_HX_SCK);
  // בלי HX711 מחובר, tare() היה נתקע לנצח - לכן בודקים קודם עם מגבלת זמן
  if (!scale.wait_ready_timeout(1000)) {
    Serial.println(F("[SCALE] HX711 not found - check wiring (weight = 0)"));
    return;
  }
  scale.set_scale(SCALE_FACTOR);
  scale.tare();                    // איפוס: הקופסה הריקה = 0 גרם
  scaleOk = true;
  Serial.println(F("[SCALE] tared"));
}

// ממוצע של 3 דגימות בגרמים; אם HX711 לא מוכן מחזיר את הקריאה הקודמת
// קריאה טרייה (ממתינה ל-HX711), לשימוש ברגע פתיחת החריץ כדי שהבסיס לא יהיה ישן
float readWeightFresh() {
  if (!scaleOk) return 0;
  return scale.get_units(5);
}

float readWeightGrams() {
  static float last = 0;
  if (scaleOk && scale.is_ready()) last = scale.get_units(3);
  return last;
}

// מיפוי פקודות שלט ה-IR של הערכה (שלט 17 מקשים: 1-9, *, 0, #, חצים, OK) לספרות 0..9, ומקש * למחיקה.
// אם מודפס "[IR] unknown command" על ספרה - לעדכן את הערך לפי מה שמודפס.
int irCommandToDigit(uint8_t cmd) {
  switch (cmd) {
    case 0x19: return 0;  case 0x45: return 1;  case 0x46: return 2;
    case 0x47: return 3;  case 0x44: return 4;  case 0x40: return 5;
    case 0x43: return 6;  case 0x07: return 7;  case 0x15: return 8;
    case 0x09: return 9;
    case 0x16: return KEY_CLEAR;  // מקש * = מחיקה
    default: return -1;
  }
}

// ===================== מכונת מצבים =====================
const char* STATE_NAMES[] = {"CLOSED", "IDLE", "WAIT_ID", "SLOT_OPEN", "MESSAGE"};

State state = ST_CLOSED;
unsigned long stateStart = 0;       // מתי נכנסנו למצב הנוכחי
bool voted[NUM_VOTERS] = {false};    // מי כבר הצביע (לפי מיקום בפנקס)
int  votesCount   = 0;
int  currentVoter = -1;            // מיקום הבוחר הנוכחי בפנקס
char idBuf[ID_LEN + 1] = "";       // הספרות שהוקשו עד עכשיו
uint8_t idLen = 0;
float baseWeight  = 0;              // משקל הקופסה ברגע פתיחת החריץ

// ----- כפתור -----
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;

// ----- IR -----
int lastIrDigit = -1;
unsigned long lastIrTime = 0;

// ----- חיישן מרחק -----
unsigned long lastSenseTime = 0;
int lastDistanceCm = 999;

// ===================== פונקציות עזר =====================
void setLeds(bool green, bool red) {
  digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,   red   ? HIGH : LOW);
}

void showLcd(const char* line1, const char* line2) {
  lcdClear();
  lcdLine(0, line1);
  lcdLine(1, line2);
}

// מחפש מספר ת"ז בפנקס. מחזיר את מיקומו, או -1 אם אינו רשום
int findVoter(const char* id) {
  for (uint8_t i = 0; i < NUM_VOTERS; i++) if (strcmp(id, VOTER_IDS[i]) == 0) return i;
  return -1;
}

// מציג בשורה השנייה את הספרות שהוקשו, ו-_ במקום ספרה חסרה
void showIdEntry() {
  char line[ID_LEN + 1];
  for (uint8_t i = 0; i < ID_LEN; i++) line[i] = (i < idLen) ? idBuf[i] : '_';
  line[ID_LEN] = '\0';
  lcdLine(1, line);
}

// מקבל int (ולא State) כדי ש-Tinkercad/Arduino IDE לא ייכשלו בהצהרות האוטומטיות
void changeState(int next) {
  if (next != state) {                   // הדפסה רק כשיש שינוי מצב
    Serial.print(F("[STATE] "));
    Serial.print(STATE_NAMES[state]);
    Serial.print(F(" -> "));
    Serial.println(STATE_NAMES[next]);
  }
  state = (State)next;
  stateStart = millis();

  char buf[17];
  switch (next) {
    case ST_CLOSED:
      setSlot(false);
      setLeds(false, true);
      snprintf(buf, sizeof(buf), "Votes: %d/%d", votesCount, NUM_VOTERS);
      showLcd("Poll CLOSED", buf);
      break;
    case ST_IDLE:
      setSlot(false);
      setLeds(false, false);
      snprintf(buf, sizeof(buf), "Votes so far: %d", votesCount);
      showLcd("Poll OPEN", buf);
      break;
    case ST_WAIT_ID:
      setLeds(false, false);
      idLen = 0;
      idBuf[0] = '\0';
      showLcd("Enter ID number:", "");
      showIdEntry();
      break;
    case ST_SLOT_OPEN:
      baseWeight = readWeightFresh();
      Serial.print(F("[SCALE] base weight = "));
      Serial.print(baseWeight, 1);
      Serial.println(F(" g"));
      setSlot(true);
      setLeds(true, false);
      snprintf(buf, sizeof(buf), "ID %s OK", idBuf);
      showLcd(buf, "Insert envelope");
      break;
    case ST_MESSAGE:
      break;  // ההודעה עצמה מוצגת ע"י showResult
  }
}

// הצגת הודעת תוצאה למשך MSG_SHOW_MS ואז חזרה ל-IDLE
void showResult(const char* line1, const char* line2, bool green, bool red) {
  setSlot(false);
  setLeds(green, red);
  changeState(ST_MESSAGE);
  showLcd(line1, line2);
}

// מחזיר true פעם אחת ברגע לחיצה (ירידה ל-LOW) אחרי סינון רעידות
bool buttonPressed() {
  bool reading = digitalRead(PIN_BUTTON);
  bool pressed = false;
  if (reading != lastButtonReading) lastDebounceTime = millis();
  if (millis() - lastDebounceTime > DEBOUNCE_MS && reading != buttonState) {
    buttonState = reading;
    if (buttonState == LOW) pressed = true;
  }
  lastButtonReading = reading;
  return pressed;
}

// מחזיר ספרה 0..9 שנלחצה בשלט, KEY_CLEAR למקש מחיקה, או -1 אם אין לחיצה חדשה
int readIrDigit() {
  if (!IrReceiver.decode()) return -1;
  uint8_t cmd = IrReceiver.decodedIRData.command;
  IrReceiver.resume();
  int digit = irCommandToDigit(cmd);
  if (digit == KEY_CLEAR) { Serial.println(F("[IR] clear")); return KEY_CLEAR; }
  if (digit < 0) {
    Serial.print(F("[IR] unknown command 0x"));
    Serial.println(cmd, HEX);
    return -1;
  }
  if (digit == lastIrDigit && millis() - lastIrTime < IR_REPEAT_MS) return -1;  // חזרה
  lastIrDigit = digit;
  lastIrTime = millis();
  Serial.print(F("[IR] digit "));
  Serial.println(digit);
  return digit;
}

// מדידת מרחק בס"מ; 999 = אין הד (אף אחד לא עומד)
int readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (us == 0) return 999;
  return (int)(us / 58UL);
}

bool voterPresent() {
  if (millis() - lastSenseTime >= SENSE_PERIOD_MS) {
    lastSenseTime = millis();
    lastDistanceCm = readDistanceCm();
  }
  return lastDistanceCm < PRESENCE_CM;
}

// ===================== לוגיקה לכל מצב =====================
void handleIdIdentification() {
  int key = readIrDigit();
  if (key == -1) return;
  stateStart = millis();                 // כל הקשה מאפסת את מונה הזמן

  if (key == KEY_CLEAR) {                // מחיקת כל מה שהוקש
    idLen = 0;
    idBuf[0] = '\0';
    showIdEntry();
    return;
  }

  idBuf[idLen++] = '0' + key;
  idBuf[idLen] = '\0';
  showIdEntry();
  if (idLen < ID_LEN) return;            // עוד לא הוקשו 9 ספרות

  Serial.print(F("[ID] entered "));
  Serial.println(idBuf);

  // שאלה 1: האם הבוחר רשום בפנקס?
  currentVoter = findVoter(idBuf);
  if (currentVoter < 0) {
    Serial.println(F("[ID] -> REJECTED: not registered"));
    showResult("Not registered", "Access denied", false, true);
    return;
  }
  // שאלה 2: האם הבוחר כבר הצביע?
  if (voted[currentVoter]) {
    Serial.println(F("[ID] -> REJECTED: already voted"));
    showResult("Already voted!", "Access denied", false, true);
    return;
  }
  Serial.println(F("[ID] -> APPROVED"));
  changeState(ST_SLOT_OPEN);
}

void handleSlotOpen() {
  static uint8_t aboveCount = 0;
  if (millis() - stateStart > SLOT_TIMEOUT_MS) {   // עבר הזמן בלי מעטפה
    aboveCount = 0;
    Serial.println(F("[VOTE] timeout - no envelope, vote NOT counted"));
    showResult("No envelope", "Vote not counted", false, true);
    return;
  }
  if (millis() - stateStart < SETTLE_MS) { aboveCount = 0; return; }  // הסרוו עדיין זז
  if (!scaleOk || !scale.is_ready()) return;  // מחכים לקריאה חדשה של ה-HX711
  float w = readWeightGrams();
  float delta = w - baseWeight;
  Serial.print(F("[SCALE] w = "));       // הדפסה לאבחון: המשקל וההפרש בכל קריאה
  Serial.print(w, 1);
  Serial.print(F(" g, delta = "));
  Serial.println(delta, 1);
  if (delta >= ENVELOPE_MIN_G) aboveCount++; else aboveCount = 0;
  if (aboveCount >= CONFIRM_READS) {
    aboveCount = 0;
    voted[currentVoter] = true;
    votesCount++;
    Serial.print(F("[VOTE] envelope detected, +"));
    Serial.print(delta, 1);
    Serial.print(F(" g. ID "));
    Serial.print(idBuf);
    Serial.print(F(" counted. total = "));
    Serial.println(votesCount);
    char buf[17];
    snprintf(buf, sizeof(buf), "Total votes: %d", votesCount);
    showResult("Vote counted!", buf, true, false);
  }
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(9600);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  slotInit();
  IrReceiver.begin(PIN_IR);
  lcdInit();
  weightInit();

  Serial.println(F("=== Smart Ballot Box ready ==="));
  Serial.println(F("Press the button to OPEN the poll"));
  changeState(ST_CLOSED);
}

void loop() {
  slotUpdate();  // עצירת הסרוו בסוף תנועה (בחומרה)

  // כפתור מנהל: פתיחה / סגירה של הקלפי מכל מצב
  if (buttonPressed()) {
    if (state == ST_CLOSED) {
      Serial.println(F("[ADMIN] poll opened"));
      changeState(ST_IDLE);
    } else {
      Serial.print(F("[ADMIN] poll closed. votes = "));
      Serial.print(votesCount);
      Serial.print(F(" / "));
      Serial.println(NUM_VOTERS);
      changeState(ST_CLOSED);
    }
  }

  switch (state) {
    case ST_CLOSED:
      break;
    case ST_IDLE:
      if (voterPresent()) {
        Serial.print(F("[SENSOR] voter detected at "));
        Serial.print(lastDistanceCm);
        Serial.println(F(" cm"));
        changeState(ST_WAIT_ID);
      }
      readIrDigit();  // מנקה לחיצות שלט כשאין מצביע
      break;
    case ST_WAIT_ID:
      handleIdIdentification();
      if (state == ST_WAIT_ID && millis() - stateStart > ID_TIMEOUT_MS) {
        Serial.println(F("[ID] timeout - ID entry cancelled"));
        changeState(ST_IDLE);
      }
      break;
    case ST_SLOT_OPEN:
      handleSlotOpen();
      break;
    case ST_MESSAGE:
      if (millis() - stateStart > MSG_SHOW_MS) changeState(ST_IDLE);
      break;
  }
}
