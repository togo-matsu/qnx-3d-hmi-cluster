#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>

const char* SERVICE_UUID   = "19B10000-E8F2-537E-4F6C-D104768A1214";
const char* CHAR_TELEMETRY = "19B10001-E8F2-537E-4F6C-D104768A1214";

BLEService sensorService(SERVICE_UUID);
BLECharacteristic telemetryChar(CHAR_TELEMETRY, BLERead | BLENotify, 20);

struct __attribute__((packed)) ImuHmiPacket {
    uint32_t timestamp_ms;
    float    angle_deg;     // 3D車両の回転角 (0.0 - 360.0)
    float    pitch_deg;     // 前後傾き角 (-90.0 - +90.0)
    float    rms_accel;     // 合成加速度 (G)
    uint8_t  mode;          // 0: Normal, 1: Warning, 2: Emergency
    uint8_t  reserved[3];   // reserved[0]: ターゲットID (1..9 = P1..P9)
};

float currentYaw = 0.0f;
unsigned long lastMicros = 0;
bool apdsAvailable = false;

// 列ラッチ状態: 0 = 左列(<-), 1 = 中央列(無), 2 = 右列(->)
uint8_t colLatch = 1;

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!BLE.begin()) {
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
    }
    BLE.setDeviceName("Nano33Sense-Rev2");
    BLE.setLocalName("Nano33Sense-Rev2");
    BLE.setAdvertisedService(sensorService);
    sensorService.addCharacteristic(telemetryChar);
    BLE.addService(sensorService);
    BLE.advertise();

    if (!IMU.begin()) {
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
    }

    // APDS-9960 光学ジェスチャー初期化（失敗時もIMU/BLEは継続）
    if (APDS.begin()) {
        APDS.setGestureSensitivity(80);
        apdsAvailable = true;
    }

    lastMicros = micros();
}

void loop() {
    BLEDevice central = BLE.central();
    if (central) {
        digitalWrite(LED_BUILTIN, HIGH);
        lastMicros = micros();

        while (central.connected()) {
            // 1. APDS-9960 左右スワイプによる列ラッチ更新
            if (apdsAvailable && APDS.gestureAvailable()) {
                int gesture = APDS.readGesture();
                if (gesture == GESTURE_LEFT) {
                    colLatch = 0; // 左列 (P1 / P4 / P7)
                } else if (gesture == GESTURE_RIGHT) {
                    colLatch = 2; // 右列 (P3 / P6 / P9)
                } else if (gesture == GESTURE_UP || gesture == GESTURE_DOWN) {
                    colLatch = 1; // 中央列にリセット (P2 / P5 / P8)
                }
            }

            // 2. IMU 傾き計測と 3x3 ターゲット番号算出
            float ax, ay, az, gx, gy, gz;
            if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
                IMU.readAcceleration(ax, y, az);
                IMU.readGyroscope(gx, gy, gz);

                unsigned long now = micros();
                float dt = (now - lastMicros) * 1e-6f;
                lastMicros = now;

                float roll  = atan2(ay, az) * 180.0f / M_PI;
                float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
                if (fabs(gz) > 1.5f) {
                    currentYaw += gz * dt;
                }

                float combinedAngle = fmod(currentYaw + roll * 2.0f + 3600.0f, 360.0f);
                float rms = sqrt(ax * ax + ay * ay + az * az);

                // 行判定: 0=上段(前傾 > +15°), 1=中段(水平), 2=下段(手前傾 < -15°)
                uint8_t row = 1;
                if (pitch > 15.0f) {
                    row = 0;
                } else if (pitch < -15.0f) {
                    row = 2;
                }

                // 3x3 マトリクス ID (1..9 = P1..P9)
                uint8_t targetId = row * 3 + colLatch + 1;

                uint8_t mode = 0;
                if (az < -0.5f) {
                    mode = 2;
                } else if (rms > 1.8f) {
                    mode = 1;
                }

                ImuHmiPacket pkt;
                pkt.timestamp_ms = millis();
                pkt.angle_deg    = combinedAngle;
                pkt.pitch_deg    = pitch;
                pkt.rms_accel    = rms;
                pkt.mode         = mode;
                pkt.reserved[0]  = targetId;
                pkt.reserved[1]  = colLatch;
                pkt.reserved[2]  = 0;

                telemetryChar.writeValue((uint8_t*)&pkt, sizeof(pkt));
                delay(40); // 25Hz
            }
        }
        digitalWrite(LED_BUILTIN, LOW);
        BLE.advertise();
    }
}
