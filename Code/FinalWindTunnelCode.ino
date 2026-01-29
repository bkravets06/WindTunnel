#include <Arduino.h>
#include <U8g2lib.h>

// ----- Display wiring -----
#define PIN_SCLK 13
#define PIN_MOSI 11
#define PIN_DC 8    // LCD A0 (D/C)
#define PIN_RST 7   // LCD /RESET
#define PIN_CS 10   // LCD /CS

U8G2_ST7565_NHD_C12864_F_4W_SW_SPI u8g2(
  U8G2_R0,
  PIN_SCLK, PIN_MOSI, PIN_CS, PIN_DC, PIN_RST
);

// Airflow model (linear with RPM)
const float FAN_MAX_CFM = 86.5f; // at 800 RPM
const float FAN_MAX_RPM = 800.0f;

// Cross section: 2.6 in × 2.6 in
const float DUCT_W_IN = 2.381f;
const float DUCT_H_IN = 2.381f;

// Precomputed area in square feet
const float IN2_PER_FT2 = 144.0f;
const float ductArea_ft2 = (DUCT_W_IN * DUCT_H_IN) / IN2_PER_FT2;

// ----- FSRs -----
const int FSR1Pin = A0;
const int FSR2Pin = A1;
const int FSR3Pin = A2;

float FSR1Reading = 0, FSR2Reading = 0, FSR3Reading = 0;

// Baselines (tare) captured at test start (fan ON)
float fsr1Base = 0, fsr2Base = 0, fsr3Base = 0;

// Geometry parameters for force/moment calc
// theta in degrees (tilt angle), r is lever arm distance (units consistent with your force units)
float theta = 22.8f; // set your tilt if needed
float r = 1.0f;      // set your geometry (same arbitrary units as FSR-derived forces)

// Computed forces
float Fx_f = 0, Fy_f = 0, Fz_f = 0;

// Fog control (low-voltage DC control only)
const int fogGatePin = 6; // Arduino pin -> 100 Ω -> MOSFET Gate
const int fogSwPin = 5;   // Panel switch to GND; use INPUT_PULLUP

// ----- Fan control -----
const int buttonPin = 2;   // still used to toggle fan
const int fanPowerPin = 12; // moved off D8
const int potPin = A3;
const int PWMPin = 3;
const int tachPin = A4;
const int buttonPause = 1000;
const int logPause = 1000;

bool buttonState = HIGH;
bool fanState = false;
bool lastButtonState = HIGH;
bool fogCommand = false; // what the panel switch is requesting

unsigned long lastPressTime = 0;
unsigned long lastLogTime = 0;

int potRaw = 0;
int potPWM = 0;
int rpmValue = 0;

void forceClear() {
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  delay(10);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

// Map potPWM (0–255) to text bucket
const char* potLevelText(int pwm) {
  if (pwm == 0 && fanState == LOW) return "Off";
  if (pwm <= 85) return "Low";
  if (pwm <= 170) return "Med";
  if (pwm > 170) return "High";
}

void computeAirspeed(float rpm, float &v_fps, float &v_mps) {
  float Q_cfm = FAN_MAX_CFM * (rpm / FAN_MAX_RPM);
  if (Q_cfm < 0) Q_cfm = 0;
  float Q_cfs = Q_cfm / 60.0f;
  if (ductArea_ft2 > 0.0f) {
    v_fps = Q_cfs / ductArea_ft2;
  } else {
    v_fps = 0.0f;
  }
  v_mps = v_fps * 0.3048f;
}

// Compute Fx, Fy, Fz from FSR readings
void computeForces() {
  Fx_f = tan(theta * PI / 180) * (-FSR1Reading + 0.5f * (FSR2Reading + FSR3Reading)); // x-like axis
  Fy_f = (float)FSR1Reading + (float)FSR2Reading + (float)FSR3Reading;                // total
  Fz_f = tan(theta * PI / 180) * ((float)FSR2Reading - (float)FSR3Reading);           // z-like axis
}

void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf); // 8-px line height

  // Consistent baseline positions (8 px per row)
  const int L1 = 8;
  const int L2 = 16;
  const int L3 = 24;
  const int L4 = 32;
  const int L5 = 40;
  const int L6 = 48;
  const int L7 = 56;
  const int L8 = 63; // bottom line

  char line[32];
 
  // Row 1: Fx left
  int round1;
  char sign1;
  if (Fx_f < 0) {
    round1 = (int)(-Fx_f * 10000.0f + 0.5f);
    sign1 = '-';
  } else {
    round1 = (int)(Fx_f * 10000.0f + 0.5f);
    sign1 = ' ';
  }
  int ones1 = round1 / 10000;
  int tenths1 = (round1 / 1000) % 10;
  int hunds1 = (round1 / 100) % 10;
  int thous1 = (round1 / 10) % 10;
  int tenthous1 = round1 % 10;
  snprintf(line, sizeof(line), "Drag: %c%d.%d%d%d%d N", sign1, ones1, tenths1, hunds1, thous1, tenthous1);
  u8g2.drawStr(0, L1, line);

  // Row 2: Fy
  long round2;
  char sign2;
  if (Fy_f < 0) {
    round2 = (long)(-Fy_f * 10000.0f + 0.5f);
    sign2 = ' ';
  } else {
    round2 = (long)(Fy_f * 10000.0f + 0.5f);
    sign2 = '-';
  }
  int ones2 = round2 / 10000;
  int tenths2 = (round2 / 1000) % 10;
  int hunds2 = (round2 / 100) % 10;
  int thous2 = (round2 / 10) % 10;
  int tenthous2 = round2 % 10;
  snprintf(line, sizeof(line), "Lift: %c%d.%d%d%d%d N", sign2, ones2, tenths2, hunds2, thous2, tenthous2);
  u8g2.drawStr(0, L2, line);

  // Row 3: Fz
  int round3;
  char sign3;
  if (Fz_f < 0) {
    round3 = (int)(-Fz_f * 10000.0f + 0.5f);
    sign3 = '-';
  } else {
    round3 = (int)(Fz_f * 10000.0f + 0.5f);
    sign3 = ' ';
  }
  int ones3 = round3 / 10000;
  int tenths3 = (round3 / 1000) % 10;
  int hunds3 = (round3 / 100) % 10;
  int thous3 = (round3 / 10) % 10;
  int tenthous3 = round3 % 10;
  snprintf(line, sizeof(line), "Side: %c%d.%d%d%d%d N", sign3, ones3, tenths3, hunds3, thous3, tenthous3);
  u8g2.drawStr(0, L3, line);

  // Row 4: SPEED level
  snprintf(line, sizeof(line), "Speed: %-6s", potLevelText(potPWM));
  u8g2.drawStr(0, L5, line);

  // Row 5: RPM
  char rpmStr[20];
  snprintf(rpmStr, sizeof(rpmStr), "RPM:%4d", rpmValue);
  u8g2.drawStr(0, L6, rpmStr);

  // Row 6 (bottom): Airspeed with one decimal (ft/s)
  float v_fps_f, v_mps_f;
  computeAirspeed((float)rpmValue, v_fps_f, v_mps_f);
  int whole = (int)v_mps_f;
  int tenths = (int)((v_mps_f - whole) * 10.0f + 0.5f);
  if (tenths >= 10) {
    whole += 1;
    tenths = 0;
  }
  char vline[28];
  snprintf(vline, sizeof(vline), "Airspeed: %d.%d m/s", whole, tenths);
  u8g2.drawStr(0, L7, vline);

  // Row 8 left: Fog status (fixed width so label doesn't shift)
  const char* fogOnStr = "Fog: On";
  const char* fogOffStr = "Fog: Off";
  int fogW = u8g2.getStrWidth(fogOffStr);
  int fogX = 128 - fogW - 2;
  if (fogX < 0) fogX = 0;
  u8g2.drawStr(0, L8, isFogOn() ? fogOnStr : fogOffStr);

  //Hexagon LOGO:

  // Hexagon & GB
  u8g2.drawStr(98,L2+4,"/");
  u8g2.drawStr(102,L2+4,"\\");
  u8g2.drawStr(94,L3,"/");
  u8g2.drawStr(106,L3,"\\");
  u8g2.drawStr(90,L3+4,"/");
  u8g2.drawStr(110,L3+4,"\\");
  u8g2.drawStr(88,L4+2,"|");
  u8g2.drawStr(97,L4+1,"GB");
  u8g2.drawStr(111,L4+2,"|");
  u8g2.drawStr(90,L4+6,"\\");
  u8g2.drawStr(110,L4+6,"/");
  u8g2.drawStr(94,L5+2,"\\");
  u8g2.drawStr(106,L5+2,"/");
  u8g2.drawStr(98,L5+6,"\\");
  u8g2.drawStr(102,L5+6,"/");

  //Gusts
  u8g2.drawStr(109,L4,"~"); 
  u8g2.drawStr(108,L4,"-");
  u8g2.drawStr(107,L4+2,"-");
  u8g2.drawStr(109,L4+2,"-");
  u8g2.drawStr(113,L4,"~");
  u8g2.drawStr(112,L4+2,"~");
  u8g2.drawStr(114,L4+2,"-");
  u8g2.drawStr(113,L4-4,"\\");
  u8g2.drawStr(117,L4-2,"~");
  u8g2.drawStr(119,L4+3,"~");
  u8g2.drawStr(121,L4,"'");
  u8g2.drawStr(110,L4+7,"~");
  u8g2.drawStr(114,L3,"|");
  u8g2.drawStr(119,L3-3,"/");
  u8g2.drawStr(115,L2,"\\");
  u8g2.drawStr(117,L2+4,"~");
  u8g2.drawStr(108,L2+3,"`");
  u8g2.drawStr(110,L2+4,"`");
  u8g2.drawStr(113,L2+6,"`");



  u8g2.sendBuffer();
}

// -------- Fan logic (unchanged) --------
void fanPower() {
  unsigned long now = millis();
  if (now > lastPressTime + buttonPause) {
    buttonState = digitalRead(buttonPin); // LOW when pressed
    bool gateLevel = digitalRead(fanPowerPin);

    if (buttonState == LOW && buttonState != lastButtonState) {
      if (gateLevel == HIGH) {
        digitalWrite(fanPowerPin, LOW);
        fanState = false;
        pinMode(PWMPin, INPUT);
        Serial.println("Fan OFF");
        fsr1Base = 0;
        fsr2Base = 0;
        fsr3Base = 0;
      } else {
        digitalWrite(fanPowerPin, HIGH);
        fanState = true;
        pinMode(PWMPin, OUTPUT);
        Serial.println("Fan ON");
        fsr1Base = 0.08 * (exp(0.00464 * (analogRead(FSR1Pin))) - exp(-0.01 * (analogRead(FSR1Pin))));
        fsr2Base = 0.08 * (exp(0.00464 * (analogRead(FSR2Pin))) - exp(-0.01 * (analogRead(FSR2Pin))));
        fsr3Base = 0.08 * (exp(0.00464 * (analogRead(FSR3Pin))) - exp(-0.01 * (analogRead(FSR3Pin))));
      }
      lastButtonState = LOW;
      lastPressTime = now;
    } else if (buttonState == HIGH) {
      lastButtonState = HIGH;
    }
  }
}

void PWMControl() {
  potRaw = analogRead(potPin);
  potPWM = map(potRaw, 0, 1023, 0, 255);
  potPWM = constrain(potPWM, 0, 255);
  if (fanState) {
    analogWrite(PWMPin, potPWM);
  } else {
    analogWrite(PWMPin, 0);
    potPWM = 0;
  }
  Serial.println(potPWM);
}

void TachControl() {
  unsigned long now = millis();
  if (now > lastLogTime + logPause) {
    if (fanState) {
      int tachValue = analogRead(tachPin); // placeholder analog tach
      tachValue = map(tachValue, 0, 1023, 350, 800);
      rpmValue = constrain(tachValue, 325, 825);
      Serial.print("RPM: ");
      Serial.println(rpmValue);
    } else {
      rpmValue = 0;
      analogWrite(PWMPin, 0);
    }
    lastLogTime = now;
  }
}

void initFogIO() {
  pinMode(fogGatePin, OUTPUT);
  digitalWrite(fogGatePin, LOW); // MOSFET off by default
  pinMode(fogSwPin, INPUT_PULLUP); // switch to GND -> reads LOW when ON
}

// Read the panel switch and drive the MOSFET gate
void updateFogControl() {
  fogCommand = (digitalRead(fogSwPin) == LOW); // active-low switch
  digitalWrite(fogGatePin, fogCommand ? HIGH : LOW);
}

// For now, actual = commanded (no sense input yet)
bool isFogOn() {
  return fogCommand;
}

void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.setFlipMode(1);
  u8g2.setContrast(100);
  forceClear();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(fanPowerPin, OUTPUT);
  digitalWrite(fanPowerPin, LOW);
  fanState = false;

  pinMode(PWMPin, INPUT);
  pinMode(tachPin, INPUT);
  lastButtonState = HIGH;

  initFogIO();
  drawScreen();
}

void loop() {
  fanPower();
  PWMControl();
  TachControl();

  // Read FSRs (and scale to Newtons)
  FSR1Reading = 0.08 * (exp(0.00464 * (analogRead(FSR1Pin))) - exp(-0.01 * (analogRead(FSR1Pin))));
  FSR2Reading = 0.08 * (exp(0.00464 * (analogRead(FSR2Pin))) - exp(-0.01 * (analogRead(FSR2Pin))));
  FSR3Reading = 0.08 * (exp(0.00464 * (analogRead(FSR3Pin))) - exp(-0.01 * (analogRead(FSR3Pin))));

  // Apply tare baseline so we see wind-induced deltas only
  FSR1Reading = FSR1Reading - fsr1Base;
  FSR2Reading = FSR2Reading - fsr2Base;
  FSR3Reading = FSR3Reading - fsr3Base;

  // Compute Fx, Fy, Fz (and moments) from current FSR readings
  computeForces();

  // Fog control from panel switch
  updateFogControl();

  // Update display
static unsigned long lastScreen = 0;
if (millis() - lastScreen > 500) {
  drawScreen();
  lastScreen = millis();
}

  delay(50);
}