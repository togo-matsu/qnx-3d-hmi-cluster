#include <Arduino.h>

Uart SensorSerial(PA10, PA9); // D2(PA10), D8(PA9)

#define ARM_BUTTON_PIN PC13
#define ARM_LED_PIN    LED_BUILTIN

bool  isArmed = false;
bool  lastBtnState = HIGH;
unsigned long lastBtnMs = 0;

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, alt = 0.0f;
unsigned long lastImuMs = 0;
unsigned long lastTelemetryMs = 0;

const float BASE_THROTTLE = 1500.0f;
const float KP_ROLL_PITCH = 6.5f;

String rxLineUart = "";
String rxLineUsb  = "";

void parseImuLine(const String& line) {
    int idx = line.indexOf("$IMU,");
    if (idx < 0) return;
    String s = line.substring(idx);
    int p1 = s.indexOf(',', 5);
    int p2 = s.indexOf(',', p1 + 1);
    int p3 = s.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) return;

    roll  = s.substring(5, p1).toFloat();
    pitch = s.substring(p1 + 1, p2).toFloat();
    yaw   = s.substring(p2 + 1, p3).toFloat();
    alt   = s.substring(p3 + 1).toFloat();
    lastImuMs = millis();
}

uint16_t clampPwm(float val) {
    if (val < 1100.0f) return 1100;
    if (val > 1950.0f) return 1950;
    return (uint16_t)val;
}

void setup() {
    Serial.begin(115200);
    SensorSerial.begin(115200);
    pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ARM_LED_PIN, OUTPUT);
    digitalWrite(ARM_LED_PIN, LOW);
}

void loop() {
    // 1. 青ボタン (PC13) による Arm / Disarm トグル (常に有効)
    bool btn = digitalRead(ARM_BUTTON_PIN);
    if (btn == LOW && lastBtnState == HIGH && (millis() - lastBtnMs > 250)) {
        isArmed = !isArmed;
        digitalWrite(ARM_LED_PIN, isArmed ? HIGH : LOW);
        lastBtnMs = millis();
        Serial.println(isArmed ? ">>> [EVENT] B1 Pressed: ARMED! <<<" : ">>> [EVENT] B1 Pressed: DISARMED! <<<");
    }
    lastBtnState = btn;

    // 2A. ジャンパー線 (SensorSerial: D2/D8) からの受信
    while (SensorSerial.available()) {
        char c = (char)SensorSerial.read();
        if (c == '\n') {
            parseImuLine(rxLineUart);
            rxLineUart = "";
        } else if (c != '\r') {
            rxLineUart += c;
            if (rxLineUart.length() > 80) rxLineUart = "";
        }
    }

    // 2B. USB Serial からの受信 (ブリッジ中継時にもそのまま対応)
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            parseImuLine(rxLineUsb);
            rxLineUsb = "";
        } else if (c != '\r') {
            rxLineUsb += c;
            if (rxLineUsb.length() > 80) rxLineUsb = "";
        }
    }

    bool imuHealthy = (millis() - lastImuMs) < 500 && (lastImuMs > 0);

    // 3. クアッドコプター X型 4ch ミキサー計算
    uint16_t m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;
    if (isArmed) {
        float rollOut  = -roll  * KP_ROLL_PITCH;
        float pitchOut = -pitch * KP_ROLL_PITCH;

        m1 = clampPwm(BASE_THROTTLE + pitchOut - rollOut); // 右前
        m2 = clampPwm(BASE_THROTTLE - pitchOut + rollOut); // 左後
        m3 = clampPwm(BASE_THROTTLE + pitchOut + rollOut); // 左前
        m4 = clampPwm(BASE_THROTTLE - pitchOut - rollOut); // 右後
    }

    // 4. 20Hz テレメトリ出力 (snprintf %f を使わず Serial.print で確実に出力)
    if (millis() - lastTelemetryMs >= 50) {
        lastTelemetryMs = millis();
        Serial.print("FC STATE=");
        Serial.print(isArmed ? "ARMED   " : "DISARMED");
        Serial.print(" IMU=");
        Serial.print(imuHealthy ? "OK  " : "WAIT");
        Serial.print(" R=");   Serial.print(roll, 2);
        Serial.print(" P=");   Serial.print(pitch, 2);
        Serial.print(" Y=");   Serial.print(yaw, 2);
        Serial.print(" ALT="); Serial.print(alt, 2);
        Serial.print(" PWM=[M1:"); Serial.print(m1);
        Serial.print(" M2:");      Serial.print(m2);
        Serial.print(" M3:");      Serial.print(m3);
        Serial.print(" M4:");      Serial.print(m4);
        Serial.println("]");
    }
}
