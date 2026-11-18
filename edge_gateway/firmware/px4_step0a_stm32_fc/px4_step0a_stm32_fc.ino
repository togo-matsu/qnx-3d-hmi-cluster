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

// Offboard 目標値とウォッチドッグタイマー
float targetRoll = 0.0f;
float targetPitch = 0.0f;
float baseThrottle = 1500.0f;
unsigned long lastOffboardMs = 0;
bool wasOffboardActive = false;

const float KP_ROLL_PITCH = 6.5f;

String rxLineUart = "";
String rxLineUsb  = "";

void parseImuLine(const String& s) {
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

// 外部コマンド書式: $CMD,armCmd,targetRoll,targetPitch,throttle,estop
// armCmd: -1=維持, 1=ARM, 0=DISARM / estop: 1=最優先緊急遮断
void parseCmdLine(const String& s) {
    int p1 = s.indexOf(',', 5);
    int p2 = s.indexOf(',', p1 + 1);
    int p3 = s.indexOf(',', p2 + 1);
    int p4 = s.indexOf(',', p3 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) return;

    int   armCmd = s.substring(5, p1).toInt();
    float rSp    = s.substring(p1 + 1, p2).toFloat();
    float pSp    = s.substring(p2 + 1, p3).toFloat();
    float thrSp  = s.substring(p3 + 1, p4).toFloat();
    int   estop  = s.substring(p4 + 1).toInt();

    if (estop == 1) {
        isArmed = false;
        digitalWrite(ARM_LED_PIN, LOW);
        targetRoll = 0.0f;
        targetPitch = 0.0f;
        baseThrottle = 1500.0f;
        Serial.println(">>> [FAILSAFE] E-STOP TRIGGERED! FORCE DISARMED! <<<");
        return;
    }

    if (armCmd == 1 && !isArmed) {
        isArmed = true;
        digitalWrite(ARM_LED_PIN, HIGH);
        Serial.println(">>> [OFFBOARD] Remote ARM Accepted <<<");
    } else if (armCmd == 0 && isArmed) {
        isArmed = false;
        digitalWrite(ARM_LED_PIN, LOW);
        Serial.println(">>> [OFFBOARD] Remote DISARM Accepted <<<");
    }

    targetRoll   = constrain(rSp, -30.0f, 30.0f);
    targetPitch  = constrain(pSp, -30.0f, 30.0f);
    baseThrottle = constrain(thrSp, 1150.0f, 1850.0f);
    lastOffboardMs = millis();
}

void handleLine(const String& line) {
    int idxImu = line.indexOf("$IMU,");
    if (idxImu >= 0) {
        parseImuLine(line.substring(idxImu));
        return;
    }
    int idxCmd = line.indexOf("$CMD,");
    if (idxCmd >= 0) {
        parseCmdLine(line.substring(idxCmd));
        return;
    }
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
    // 1. 物理青ボタン (PC13) による最優先ローカル切替
    bool btn = digitalRead(ARM_BUTTON_PIN);
    if (btn == LOW && lastBtnState == HIGH && (millis() - lastBtnMs > 250)) {
        isArmed = !isArmed;
        digitalWrite(ARM_LED_PIN, isArmed ? HIGH : LOW);
        lastBtnMs = millis();
        Serial.println(isArmed ? ">>> [EVENT] B1 Pressed: ARMED! <<<" : ">>> [EVENT] B1 Pressed: DISARMED! <<<");
    }
    lastBtnState = btn;

    // 2. UART および USB シリアル受信
    while (SensorSerial.available()) {
        char c = (char)SensorSerial.read();
        if (c == '\n') { handleLine(rxLineUart); rxLineUart = ""; }
        else if (c != '\r') { rxLineUart += c; if (rxLineUart.length() > 96) rxLineUart = ""; }
    }
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') { handleLine(rxLineUsb); rxLineUsb = ""; }
        else if (c != '\r') { rxLineUsb += c; if (rxLineUsb.length() > 96) rxLineUsb = ""; }
    }

    // 3. PX4 Offboard ウォッチドッグ: 500ms 以上指令が途絶えたら自動で水平ホバリングへ復帰
    bool offboardActive = (millis() - lastOffboardMs) < 500 && (lastOffboardMs > 0);
    if (!offboardActive) {
        if (wasOffboardActive) {
            Serial.println(">>> [FAILSAFE] Offboard Heartbeat Lost (>500ms) -> Reverting to Level Hover <<<");
        }
        targetRoll = 0.0f;
        targetPitch = 0.0f;
        baseThrottle = 1500.0f;
    }
    wasOffboardActive = offboardActive;

    bool imuHealthy = (millis() - lastImuMs) < 500 && (lastImuMs > 0);

    // 4. 目標姿勢追従 PID ＋ クアッドコプター X型 4ch ミキサー計算
    uint16_t m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;
    if (isArmed) {
        float errRoll  = targetRoll  - roll;
        float errPitch = targetPitch - pitch;
        float rollOut  = errRoll  * KP_ROLL_PITCH;
        float pitchOut = errPitch * KP_ROLL_PITCH;

        m1 = clampPwm(baseThrottle + pitchOut - rollOut); // 右前
        m2 = clampPwm(baseThrottle - pitchOut + rollOut); // 左後
        m3 = clampPwm(baseThrottle + pitchOut + rollOut); // 左前
        m4 = clampPwm(baseThrottle - pitchOut - rollOut); // 右後
    }

    // 5. 20Hz テレメトリ出力
    if (millis() - lastTelemetryMs >= 50) {
        lastTelemetryMs = millis();
        Serial.print("FC STATE=");
        Serial.print(isArmed ? "ARMED   " : "DISARMED");
        Serial.print(" MODE=");
        Serial.print(offboardActive ? "OFFBOARD" : "STABILIZ");
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
