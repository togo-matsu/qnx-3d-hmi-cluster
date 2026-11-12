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
    explicit MainWindow(GLWidget *gl, QWidget *parent = nullptr)
        : QMainWindow(parent), m_gl(gl) {}
protected:
    void keyPressEvent(QKeyEvent *event) override {
        if (event->key() == Qt::Key_Escape) {
            close();
        } else if (event->key() == Qt::Key_R && m_gl) {
            m_gl->resetTrajectory();
        } else {
            QMainWindow::keyPressEvent(event);
        }
    }
private:
    GLWidget *m_gl;
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    GLWidget *glWidget = new GLWidget();
    MainWindow window(glWidget);
    window.setWindowTitle("QNX RTOS - Automotive 3D & Navigation Cluster HMI");
    window.setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
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

    window.showFullScreen();
    return app.exec();
}
