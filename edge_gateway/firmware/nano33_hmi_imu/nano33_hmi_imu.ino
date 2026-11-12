#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
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
    uint8_t  mode;          // 0: Normal, 1: Warning(強振動), 2: Emergency(裏返し)
    uint8_t  reserved[3];
};

float currentYaw = 0.0f;
unsigned long lastMicros = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!BLE.begin()) {
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
    }
    BLE.setLocalName("Nano33Sense-Rev2");
    BLE.setAdvertisedService(sensorService);
    sensorService.addCharacteristic(telemetryChar);
    BLE.addService(sensorService);
    BLE.advertise();

    if (!IMU.begin()) {
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
    }
    lastMicros = micros();
}

void loop() {
    BLEDevice central = BLE.central();
    if (central) {
        digitalWrite(LED_BUILTIN, HIGH);
        lastMicros = micros();

        while (central.connected()) {
            float ax, ay, az, gx, gy, gz;
            if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
                IMU.readAcceleration(ax, ay, az);
                IMU.readGyroscope(gx, gy, gz);

                unsigned long now = micros();
                float dt = (now - lastMicros) * 1e-6f;
                lastMicros = now;

                // 1. 基板を水平に回した時のヨー角積分 + 傾けた時のロール角を合成
                float roll = atan2(ay, az) * 180.0f / M_PI;
                float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
                if (fabs(gz) > 1.5f) {
                    currentYaw += gz * dt;
                }

                // 基板の傾き(Roll)と回転(Yaw)を 0〜360度にマッピング
                float combinedAngle = fmod(currentYaw + roll * 2.0f + 3600.0f, 360.0f);
                float rms = sqrt(ax * ax + ay * ay + az * az);

                // 強い衝撃(>2.0G)で Mode 1(警告色)、基板を裏返す(az < -0.5G)と Mode 2(非常停止色)
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
                pkt.reserved[0]  = 0; pkt.reserved[1] = 0; pkt.reserved[2] = 0;

                telemetryChar.writeValue((uint8_t*)&pkt, sizeof(pkt));
                delay(40); // 25Hz 送信
            }
        }
        digitalWrite(LED_BUILTIN, LOW);
        BLE.advertise();
    }
}
