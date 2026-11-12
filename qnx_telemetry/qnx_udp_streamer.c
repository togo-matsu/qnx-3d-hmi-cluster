#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define HMI_TARGET_IP   "10.0.0.155"
#define HMI_TARGET_PORT 5005
#define EDGE_LISTEN_PORT 5006

#pragma pack(push, 1)
// Raspi 5 から受信する実機センサー＋CAN パケット (16 bytes)
typedef struct {
    float    angle;      // Nano 33 姿勢角 (0.0 - 360.0 deg)
    float    speed;      // CAN ID 0x0D0 車速 (km/h)
    float    rpm;        // CAN ID 0x0C0 エンジン回転数 (RPM)
    uint32_t mode;       // 0: Normal, 1: Warning, 2: Emergency
} EdgeInputPacket;

// Ubuntu 3D HMI へ送信する 60Hz パケット (16 bytes)
typedef struct {
    uint32_t seq;
    float    angle;
    float    speed;
    uint32_t mode;
} HmiTelemetryPacket;
#pragma pack(pop)

int main(void) {
    // 1. Ubuntu HMI への送信ソケット
    int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in hmi_addr;
    memset(&hmi_addr, 0, sizeof(hmi_addr));
    hmi_addr.sin_family = AF_INET;
    hmi_addr.sin_port = htons(HMI_TARGET_PORT);
    inet_pton(AF_INET, HMI_TARGET_IP, &hmi_addr.sin_addr);

    // 2. Raspi 5 からの受信ソケット (非ブロッキング)
    int rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(listen_addr));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(EDGE_LISTEN_PORT);
    bind(rx_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr));

    int flags = fcntl(rx_sock, F_GETFL, 0);
    fcntl(rx_sock, F_SETFL, flags | O_NONBLOCK);

    HmiTelemetryPacket out = {0, 0.0f, 0.0f, 0};
    EdgeInputPacket latest_in = {0.0f, 0.0f, 0.0f, 0};
    float smooth_angle = 0.0f;
    float smooth_speed = 0.0f;
    uint32_t timeout_frames = 0;

    struct timespec ts = {0, 16666667}; // 60Hz (16.66ms)
    printf("[QNX HIL Router] Listening Edge on UDP:%d -> Streaming 60Hz to %s:%d\n",
           EDGE_LISTEN_PORT, HMI_TARGET_IP, HMI_TARGET_PORT);

    while (1) {
        out.seq++;
        int received = 0;
        EdgeInputPacket temp_in;

        // 最新のエッジパケットをすべて読み出し
        while (recvfrom(rx_sock, &temp_in, sizeof(temp_in), 0, NULL, NULL) == sizeof(temp_in)) {
            latest_in = temp_in;
            received = 1;
        }

        if (received) {
            timeout_frames = 0;
        } else {
            timeout_frames++;
        }

        // 角度の最短経路補間 (BLE の 20Hz 更新を 60Hz に滑らかに補間)
        float diff = latest_in.angle - smooth_angle;
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        smooth_angle += diff * 0.25f;
        if (smooth_angle >= 360.0f) smooth_angle -= 360.0f;
        if (smooth_angle < 0.0f) smooth_angle += 360.0f;

        // 速度の一次遅れ補間
        smooth_speed += (latest_in.speed - smooth_speed) * 0.2f;

        out.angle = smooth_angle;
        out.speed = smooth_speed;
        out.mode  = latest_in.mode;

        sendto(tx_sock, &out, sizeof(out), 0, (struct sockaddr *)&hmi_addr, sizeof(hmi_addr));
        nanosleep(&ts, NULL);
    }

    close(rx_sock);
    close(tx_sock);
    return 0;
}
