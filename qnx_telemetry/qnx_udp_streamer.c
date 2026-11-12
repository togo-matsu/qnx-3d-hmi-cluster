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

#define HMI_TARGET_IP    "10.0.0.155"
#define HMI_TARGET_PORT  5005
#define EDGE_LISTEN_PORT 5006
#define ROS2_LISTEN_PORT 5008

#pragma pack(push, 1)
// Raspi 5 からの実機入力 (16 bytes)
typedef struct {
    float    angle;
    float    speed;
    float    rpm;
    uint32_t mode;
} EdgeInputPacket;

// ROS 2 ADAS ノードからの安全介入指令 (8 bytes)
typedef struct {
    uint32_t override_mode; // 0: 介入なし, 1: ADAS警告(減速), 2: 緊急ブレーキ(ESTOP)
    float    speed_limit;   // 制限速度 (km/h)
} Ros2AdasPacket;

// Ubuntu Qt6 3D HMI への 60Hz 出力 (16 bytes)
typedef struct {
    uint32_t seq;
    float    angle;
    float    speed;
    uint32_t mode;
} HmiTelemetryPacket;
#pragma pack(pop)

static int create_nonblock_udp(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    return sock;
}

int main(void) {
    int tx_sock   = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in hmi_addr;
    memset(&hmi_addr, 0, sizeof(hmi_addr));
    hmi_addr.sin_family = AF_INET;
    hmi_addr.sin_port = htons(HMI_TARGET_PORT);
    inet_pton(AF_INET, HMI_TARGET_IP, &hmi_addr.sin_addr);

    int edge_sock = create_nonblock_udp(EDGE_LISTEN_PORT);
    int ros2_sock = create_nonblock_udp(ROS2_LISTEN_PORT);

    HmiTelemetryPacket out = {0, 0.0f, 0.0f, 0};
    EdgeInputPacket latest_in = {0.0f, 0.0f, 0.0f, 0};
    Ros2AdasPacket  adas_cmd  = {0, 240.0f};
    uint32_t adas_ttl = 0; // ADAS 指令の有効フレーム数 (30フレーム=0.5秒)

    float smooth_angle = 0.0f;
    float smooth_speed = 0.0f;
    struct timespec ts = {0, 16666667}; // 60Hz (16.66ms)

    printf("[QNX Safety Island] Edge UDP:%d | ROS2 ADAS UDP:%d -> HMI %s:%d (60Hz)\n",
           EDGE_LISTEN_PORT, ROS2_LISTEN_PORT, HMI_TARGET_IP, HMI_TARGET_PORT);

    while (1) {
        out.seq++;
        EdgeInputPacket temp_in;
        while (recvfrom(edge_sock, &temp_in, sizeof(temp_in), 0, NULL, NULL) == sizeof(temp_in)) {
            latest_in = temp_in;
        }

        Ros2AdasPacket temp_adas;
        while (recvfrom(ros2_sock, &temp_adas, sizeof(temp_adas), 0, NULL, NULL) == sizeof(temp_adas)) {
            adas_cmd = temp_adas;
            adas_ttl = 30; // 受信から 0.5秒間有効
        }

        float target_speed = latest_in.speed;
        uint32_t final_mode = latest_in.mode;

        // ROS 2 ADAS 安全介入の調停 (QNX Safety Island)
        if (adas_ttl > 0) {
            adas_ttl--;
            if (adas_cmd.override_mode > final_mode) {
                final_mode = adas_cmd.override_mode;
            }
            if (target_speed > adas_cmd.speed_limit) {
                target_speed = adas_cmd.speed_limit;
            }
            if (final_mode == 2) {
                target_speed = 0.0f; // 緊急ブレーキ時は車速 0 へ強制減速
            }
        }

        float diff = latest_in.angle - smooth_angle;
        if (diff > 180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        smooth_angle += diff * 0.25f;
        if (smooth_angle >= 360.0f) smooth_angle -= 360.0f;
        if (smooth_angle < 0.0f) smooth_angle += 360.0f;

        smooth_speed += (target_speed - smooth_speed) * 0.2f;

        out.angle = smooth_angle;
        out.speed = smooth_speed;
        out.mode  = final_mode;

        sendto(tx_sock, &out, sizeof(out), 0, (struct sockaddr *)&hmi_addr, sizeof(hmi_addr));
        nanosleep(&ts, NULL);
    }
    return 0;
}
