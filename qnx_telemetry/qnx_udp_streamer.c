#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define TARGET_IP "10.0.0.155"
#define TARGET_PORT 5005

#pragma pack(push, 1)
typedef struct {
    uint32_t seq;
    float    angle;
    float    speed;
    uint32_t mode; // 0: Normal, 1: Warning, 2: Emergency Stop
} TelemetryData;
#pragma pack(pop)

int main(void) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(TARGET_PORT);
    inet_pton(AF_INET, TARGET_IP, &target_addr.sin_addr);

    TelemetryData data = {0, 0.0f, 60.0f, 0};
    struct timespec ts = {0, 16666667}; // 16.66ms (60Hz)

    printf("Starting Dynamic 60Hz Telemetry Streamer to %s:%d...\n", TARGET_IP, TARGET_PORT);

    while (1) {
        data.seq++;
        data.angle += 1.0f;
        if (data.angle >= 360.0f) data.angle -= 360.0f;

        // 300フレーム (約5秒) ごとにモードと速度を変動
        uint32_t cycle = (data.seq / 300) % 3;
        if (cycle == 0) {
            data.mode = 0;      // Normal
            data.speed = 60.0f;
        } else if (cycle == 1) {
            data.mode = 1;      // Warning
            data.speed = 85.5f;
        } else {
            data.mode = 2;      // Emergency Stop
            data.speed = 0.0f;
        }

        sendto(sock, &data, sizeof(data), 0, (struct sockaddr *)&target_addr, sizeof(target_addr));
        nanosleep(&ts, NULL);
    }

    close(sock);
    return 0;
}
