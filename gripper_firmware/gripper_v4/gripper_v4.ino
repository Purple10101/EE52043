/*
  Gripper v4 (slim double wishbone, SG90) - serial-controlled servo firmware for an ESP32-S3.
  From gripper_v3.ino in Robot Resources/gripper/testing/v3, with v4's limits and its own calibration.

  Wiring (SG90):
    brown  -> GND, shared with the board
    red    -> +5 V, good for about 0.7 A
    orange -> the signal pin: GPIO44 (pad D7 on a XIAO ESP32-S3), or set with N. 3.3 V signals are fine.

  The v4's whole stroke is about 17.8 servo deg, about 200 us, and the gap changes about 0.68 mm per servo
  degree. A v3 stroke (500 us) would drive it far past its ends. On our gripper a higher pulse CLOSES, the
  opposite of v3: after a rebuild on 2026-11-12 the tips touch at about 1740 us and 1480 us is open (1660 and
  1480 before it). So 1500 us is not half open as the assembly guide says. The jaws can't be moved by hand, and steps under about
  20 us don't show.

  Safety:
    - Boots RELEASED (no pulses, servo limp). Nothing moves until "A <us>"; the servo then goes straight to
      that pulse width at full speed, so pick one you know is inside the jaw travel.
    - Until calibrated, moves are clamped to 1500 +/- UNCAL_HALF_US.
    - U (unlock, for calibration) widens that to 1500 +/- ENVELOPE_HALF_US and drops the speed to 1 us/tick;
      L locks again. Nothing stops you at the jaws: jog slowly.
    - Once calibrated, every move is clamped to [closed, open]. Never set "closed" past first contact.
    - All moves are ramped (speed in us per 10 ms tick).

  Serial protocol, 115200 baud, one command per line (case-insensitive):
    ?          status line:  S us=.. tgt=.. lo=.. hi=.. cal=0|1 closed=.. open=.. pct=.. att=0|1 spd=.. sweep=.. unl=0|1 pin=..
    M <us>     move to pulse width (clamped)
    J <dus>    jog relative to the current target
    P <0..100> percent open            (needs calibration)
    O / C      fully open / closed     (needs calibration)
    H          home: middle of the range
    V <1..50>  speed, us per 10 ms tick
    T <n>      sweep closed <-> open n times (needs calibration); any other move command stops it
    Q          stop where it is
    SC / SO    set closed / open to the current position (servo must be at rest)
    W          save calibration        X   clear calibration (RAM and flash)
    U / L      unlock (calibration envelope) / lock
    R          release (stop pulses, servo goes limp)
    A [us]     start pulses at us (clamped to the limits), or at the last position if no value is given
    N <gpio>   signal pin, while released
  Every command is answered with "OK ..." or "ERR ...". "READY" is printed after reset.
*/
#include <EEPROM.h>

const int DEFAULT_PIN = 44;        // pad D7 on a XIAO ESP32-S3: UART0 RX, free since Serial is on USB
const int CHANNEL = 0;
const int PWM_HZ = 50;
const int PWM_BITS = 14;
const long PERIOD_US = 20000;
const int CENTER_US = 1500;        // the middle of the limits below, not of the jaws
const int UNCAL_HALF_US = 60;      // about 5 deg either side of centre before calibration: inside the jaws' ends
const int ENVELOPE_HALF_US = 350;  // calibration points must lie within this of centre (250 until 2026-11-12,
                                   // when the rebuilt gripper's closed end reached it)
const int MIN_SPAN_US = 100;
const int MAX_SPAN_US = 400;       // 360 us since the rebuild (closed 1790, open 1430, 2026-11-12); 260 before
const uint16_t EE_MAGIC = 0x6734;  // "g4": a saved v3 calibration can't pass as v4's
const unsigned long TICK_MS = 10;

struct Cal {
  uint16_t magic;
  int16_t closedUs;
  int16_t openUs;
};

Cal cal;
bool calibrated = false;
bool attached = false;
bool unlocked = false;
int pin = DEFAULT_PIN;
int curUs = CENTER_US;
int tgtUs = CENTER_US;
int speedUs = 3;
int sweepLeft = 0;  // half-strokes still to do
unsigned long lastTick = 0;
char line[32];
uint8_t lineLen = 0;

int loUs() {
  if (unlocked) return CENTER_US - ENVELOPE_HALF_US;
  return calibrated ? min(cal.closedUs, cal.openUs) : CENTER_US - UNCAL_HALF_US;
}
int hiUs() {
  if (unlocked) return CENTER_US + ENVELOPE_HALF_US;
  return calibrated ? max(cal.closedUs, cal.openUs) : CENTER_US + UNCAL_HALF_US;
}
int clampUs(long us) { return (int)constrain(us, (long)loUs(), (long)hiUs()); }

float pctOf(int us) {
  if (!calibrated) return NAN;
  return 100.0 * (us - cal.closedUs) / (float)(cal.openUs - cal.closedUs);
}

int usOfPct(float pct) {
  pct = constrain(pct, 0.0, 100.0);
  return clampUs(lround(cal.closedUs + (cal.openUs - cal.closedUs) * pct / 100.0));
}

void writeUs(int us) { ledcWrite(CHANNEL, (uint32_t)((long)us * (1L << PWM_BITS) / PERIOD_US)); }

void loadCal() {
  EEPROM.get(0, cal);
  int span = abs(cal.openUs - cal.closedUs);
  calibrated = cal.magic == EE_MAGIC && span >= MIN_SPAN_US && span <= MAX_SPAN_US &&
               abs(cal.closedUs - CENTER_US) <= ENVELOPE_HALF_US && abs(cal.openUs - CENTER_US) <= ENVELOPE_HALF_US;
  if (!calibrated) cal = {0, 0, 0};
}

void saveCal() {
  cal.magic = calibrated ? EE_MAGIC : 0;
  EEPROM.put(0, cal);
  EEPROM.commit();
}

void attachServo() {
  ledcSetup(CHANNEL, PWM_HZ, PWM_BITS);
  writeUs(curUs);  // so the very first pulse is already curUs
  ledcAttachPin(pin, CHANNEL);
  attached = true;
}

void releaseServo() {
  ledcDetachPin(pin);
  pinMode(pin, INPUT);
  attached = false;
}

// GPIOs a signal can go on: not the strapping pins, USB (19, 20), or flash and PSRAM (26 to 37).
bool pinAllowed(long n) { return (n >= 1 && n <= 18 && n != 3) || n == 21 || (n >= 38 && n <= 44) || n == 47 || n == 48; }

void moveTo(long us) {
  sweepLeft = 0;
  tgtUs = clampUs(us);
}

void status() {
  float pct = pctOf(curUs);
  Serial.print(F("S us="));     Serial.print(curUs);
  Serial.print(F(" tgt="));     Serial.print(tgtUs);
  Serial.print(F(" lo="));      Serial.print(loUs());
  Serial.print(F(" hi="));      Serial.print(hiUs());
  Serial.print(F(" cal="));     Serial.print(calibrated);
  Serial.print(F(" closed="));  Serial.print(cal.closedUs);
  Serial.print(F(" open="));    Serial.print(cal.openUs);
  Serial.print(F(" pct="));     Serial.print(calibrated ? pct : -1, 1);
  Serial.print(F(" att="));     Serial.print(attached);
  Serial.print(F(" spd="));     Serial.print(speedUs);
  Serial.print(F(" sweep="));   Serial.print(sweepLeft);
  Serial.print(F(" unl="));     Serial.print(unlocked);
  Serial.print(F(" pin="));     Serial.println(pin);
}

bool needCal() {
  if (calibrated) return false;
  Serial.println(F("ERR not calibrated (set closed + open first)"));
  return true;
}

// Set one calibration point; the pair becomes active once both are set and valid.
void setPoint(bool closed) {
  if (curUs != tgtUs) { Serial.println(F("ERR still moving")); return; }
  if (abs(curUs - CENTER_US) > ENVELOPE_HALF_US) { Serial.println(F("ERR outside envelope")); return; }
  int other = closed ? cal.openUs : cal.closedUs;
  if (other != 0 && (abs(curUs - other) > MAX_SPAN_US || abs(curUs - other) < MIN_SPAN_US)) {
    Serial.println(F("ERR span to the other point must be 100..260 us"));
    return;
  }
  if (closed) cal.closedUs = curUs; else cal.openUs = curUs;
  calibrated = cal.closedUs != 0 && cal.openUs != 0;
  Serial.print(F("OK ")); Serial.print(closed ? F("closed=") : F("open=")); Serial.print(curUs);
  Serial.println(calibrated ? F(" (calibrated, W to save)") : F(""));
}

void handle(char *s) {
  for (char *p = s; *p; p++) *p = toupper(*p);
  char c = s[0];
  char *arg = s + 1;
  while (*arg == ' ') arg++;
  long n = atol(arg);
  float f = atof(arg);

  if (c == '?') { status(); return; }
  if (c == 'S' && (s[1] == 'C' || s[1] == 'O')) { setPoint(s[1] == 'C'); return; }
  switch (c) {
    case 'M': moveTo(n); break;
    case 'J': moveTo((long)tgtUs + n); break;
    case 'P': if (needCal()) return; moveTo(usOfPct(f)); break;
    case 'O': if (needCal()) return; moveTo(cal.openUs); break;
    case 'C': if (needCal()) return; moveTo(cal.closedUs); break;
    case 'H': moveTo((loUs() + hiUs()) / 2); break;
    case 'V': speedUs = constrain(n, 1, 50); break;
    case 'T':
      if (needCal()) return;
      tgtUs = cal.closedUs;            // start each sweep from closed
      sweepLeft = 2 * constrain(n, 1, 50) + 1;  // reach closed, then n x (open, closed)
      break;
    case 'Q': sweepLeft = 0; tgtUs = curUs; break;
    case 'U': unlocked = true; speedUs = 1; break;
    case 'L': unlocked = false; moveTo(tgtUs); break;
    case 'W': saveCal(); break;
    case 'X': calibrated = false; cal = {0, 0, 0}; saveCal(); tgtUs = clampUs(tgtUs); break;
    case 'R': sweepLeft = 0; tgtUs = curUs; if (attached) releaseServo(); break;
    case 'N':
      if (attached) { Serial.println(F("ERR release first")); return; }
      if (!pinAllowed(n)) { Serial.println(F("ERR that pin can't carry the signal")); return; }
      pin = n;
      break;
    case 'A':
      if (attached) { Serial.println(F("ERR already attached")); return; }
      if (*arg) curUs = clampUs(n);
      tgtUs = curUs;
      attachServo();
      Serial.print(F("OK attached on GPIO ")); Serial.print(pin); Serial.print(F(" at ")); Serial.println(curUs);
      return;
    default: Serial.println(F("ERR unknown command")); return;
  }
  Serial.print(F("OK ")); Serial.println(s);
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(sizeof(Cal));
  loadCal();
  curUs = tgtUs = (loUs() + hiUs()) / 2;
  Serial.println(F("READY gripper v4 (released - send A <us> to start)"));
  status();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) { line[lineLen] = 0; handle(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    }
  }

  if (millis() - lastTick < TICK_MS) return;
  lastTick = millis();
  if (!attached) return;
  if (curUs != tgtUs) {
    int step = constrain(tgtUs - curUs, -speedUs, speedUs);
    curUs += step;
    writeUs(curUs);
  } else if (sweepLeft > 0) {
    sweepLeft--;
    if (sweepLeft > 0) tgtUs = (tgtUs == cal.openUs) ? cal.closedUs : cal.openUs;
  }
}
