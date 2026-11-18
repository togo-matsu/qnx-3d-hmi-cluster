#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_LPS22HB.h>
#include <math.h>

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float basePressure = 101.325f;
bool  baroOk = false;
unsigned long lastUs = 0;
uint8_t ledCnt = 0;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200); // TX1 (Pin 1) -> STM32 D2 (PA10)
    pinMode(LED_BUILTIN, OUTPUT);

    if (!IMU.begin()) {
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
    }
    if (BARO.begin()) {
        baroOk = true;
        delay(100);
        basePressure = BARO.readPressure();
    }
    lastUs = micros();
}

void loop() {
    unsigned long nowUs = micros();
    if (nowUs - lastUs < 10000) return; // 100Hz (10ms)
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    float ax, ay, az, gx, gy, gz;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        float accRoll  = atan2f(ay, az) * 57.29578f;
        float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;

        roll  = 0.96f * (roll  + gx * dt) + 0.04f * accRoll;
        pitch = 0.96f * (pitch + gy * dt) + 0.04f * accPitch;
        yaw  += gz * dt;
    }

    float relAlt = 0.0f;
    if (baroOk) {
        float p = BARO.readPressure();
        relAlt = (basePressure - p) * 84.3f;
    }

    // 20回(200ms)ごとにNano 33のLEDを点滅させて動作を可視化
    if (++ledCnt >= 20) {
        ledCnt = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    // ジャンパー線 (Serial1) と USB (Serial) の両方へ出力
    Serial1.print("$IMU,");
    Serial1.print(roll, 2);  Serial1.print(",");
    Serial1.print(pitch, 2); Serial1.print(",");
    Serial1.print(yaw, 2);   Serial1.print(",");
    Serial1.println(relAlt, 2);

    Serial.print("$IMU,");
    Serial.print(roll, 2);  Serial.print(",");
    Serial.print(pitch, 2); Serial.print(",");
    Serial.print(yaw, 2);   Serial.print(",");
    Serial.println(relAlt, 2);
}
