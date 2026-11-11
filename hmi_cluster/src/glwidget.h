#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPainter>
#include <vector>

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit GLWidget(QWidget *parent = nullptr);
    ~GLWidget() override;

    void updateTelemetry(uint32_t seq, float angle, float speed, uint32_t mode);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    void initShaders();
    void buildCarModel();

    void drawSpeedometer(QPainter &p, int cx, int cy, int radius);
    void drawTachometer(QPainter &p, int cx, int cy, int radius);
    void drawCenterHud(QPainter &p, int cx, int cy);
    void drawTopStatusBar(QPainter &p);

    QOpenGLShaderProgram m_program;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vbo;

    QMatrix4x4 m_projection;
    int m_triVertexCount = 0;
    int m_lineVertexCount = 0;

    uint32_t m_seq = 0;
    float m_angle = 0.0f;
    float m_speed = 0.0f;
    uint32_t m_mode = 0;
};
