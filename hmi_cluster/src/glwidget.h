#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QTimer>
#include <QElapsedTimer>
#include <QVector3D>
#include <QPointF>
#include <vector>
#include <deque>
#include <cstdint>

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit GLWidget(QWidget *parent = nullptr);
    ~GLWidget() override;

    void updateTelemetry(uint32_t seq, float angle, float speed, uint32_t mode);
    void updateOdom(double x, double y);
    void resetTrajectory();
    void loadTrajectoryHistory(const QString &csvPath = "/tmp/odom_history.csv");

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void onFrameTick();

private:
    void initShaders();
    void buildCarModel();
    void draw3DVehicleViewport(int w, int h);
    void drawCarNavigationMap(QPainter &p, const QRectF &navRect);
    void drawCompactGauge(QPainter &p, QPointF center, float radius,
                          float value, float maxVal, const QString &title,
                          const QString &unit, QColor accentColor);
    void drawHudBanner(QPainter &p, int w, int h);

    QTimer *frameTimer;
    QElapsedTimer dtTimer;
    QElapsedTimer odomSyncTimer;
    bool hasOdomSync = false;

    // OpenGL シェーダー & バッファ
    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo;
    int m_triVertexCount = 0;
    int m_lineVertexCount = 0;

    // テレメトリ状態
    uint32_t currentSeq = 0;
    float currentAngle  = 0.0f; // deg (0 - 360)
    float currentSpeed  = 0.0f; // km/h
    float currentRpm    = 800.0f;
    uint32_t currentMode = 0;   // 0: Normal, 1: Warning, 2: Emergency

    // オドメトリ & カーナビ軌跡状態
    double odomX = 0.0;
    double odomY = 0.0;
    double totalDistM = 0.0;
    std::deque<QPointF> trajectoryPoints;
    static const size_t MAX_TRAIL_POINTS = 1500;
};

#endif // GLWIDGET_H
