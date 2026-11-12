#include <Arduino.h>

// Nucleo-F446RE の青色 USER ボタンは PC13 (押すと LOW)
#define ACCEL_BUTTON_PIN PC13
#define LED_PIN          LED_BUILTIN // PA5 (緑色 LD2)

float targetSpeed = 0.0f;
float currentSpeed = 0.0f;
float currentRpm = 800.0f; // アイドリング 800 RPM

void setup() {
    Serial.begin(115200);
    pinMode(ACCEL_BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    bool isAccelerating = (digitalRead(ACCEL_BUTTON_PIN) == LOW);
    digitalWrite(LED_PIN, isAccelerating ? HIGH : LOW);

    // アクセル ON で急加速、OFF で自然減速
    if (isAccelerating) {
        currentSpeed += 2.2f;
        if (currentSpeed > 220.0f) currentSpeed = 220.0f;
    } else {
        currentSpeed -= 1.0f;
        if (currentSpeed < 0.0f) currentSpeed = 0.0f;
    }

    // 車速に応じたタコメーター (0 - 8000 RPM)
    currentRpm = (currentSpeed / 220.0f) * 7200.0f + 800.0f;

    // 車載 CAN 形式 (ID, DLC, Data[8]) をシリアルパケットとして出力 (50Hz / 20ms 周期)
    // CAN ID 0x0D0: 車速 (km/h * 100), CAN ID 0x0C0: エンジン回転数 (RPM)
    uint16_t speedRaw = (uint16_t)(currentSpeed * 100.0f);
    uint16_t rpmRaw   = (uint16_t)currentRpm;

    Serial.printf("CAN 0x0D0 %02X %02X\n", (speedRaw >> 8) & 0xFF, speedRaw & 0xFF);
    Serial.printf("CAN 0x0C0 %02X %02X\n", (rpmRaw >> 8) & 0xFF, rpmRaw & 0xFF);

    delay(20);
}
