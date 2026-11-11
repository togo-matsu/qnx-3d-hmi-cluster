#include <QApplication>
#include <QMainWindow>
#include <QUdpSocket>
#include <QKeyEvent>
#include "glwidget.h"

#pragma pack(push, 1)
struct TelemetryPacket {
uint32_t seq;
float    angle;
float    speed;
uint32_t mode;
};
#pragma pack(pop)

class MainWindow : public QMainWindow {
public:
explicit MainWindow(QWidget *parent = nullptr) : QMainWindow(parent) {}
protected:
void keyPressEvent(QKeyEvent *event) override {
    if (event->key() == Qt::Key_Escape) {
        close();
    } else {
        QMainWindow::keyPressEvent(event);
    }
}
};

int main(int argc, char *argv[]) {
QApplication app(argc, argv);

MainWindow window;
window.setWindowTitle("QNX RTOS - Automotive 3D Cluster HMI");

// ウィンドウ枠を消去して最前面に設定
window.setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);

GLWidget *glWidget = new GLWidget(&window);
window.setCentralWidget(glWidget);

QUdpSocket udpSocket;
if (udpSocket.bind(QHostAddress::AnyIPv4, 5005, QUdpSocket::ShareAddress)) {
    QObject::connect(&udpSocket, &QUdpSocket::readyRead, [&]() {
        while (udpSocket.hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(udpSocket.pendingDatagramSize());
            udpSocket.readDatagram(datagram.data(), datagram.size());

            if (datagram.size() == sizeof(TelemetryPacket)) {
                TelemetryPacket packet;
                memcpy(&packet, datagram.constData(), sizeof(TelemetryPacket));
                glWidget->updateTelemetry(packet.seq, packet.angle, packet.speed, packet.mode);
            }
        }
    });
}

// GNOME Shell 正規フルスクリーン要求
window.showFullScreen();
return app.exec();
}
