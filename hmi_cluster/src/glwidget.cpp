#include "glwidget.h"
#include <QPainter>
#include <QPainterPath>
#include <QKeyEvent>
#include <QtMath>
#include <algorithm>
#include <cmath>

struct CarVertex {
    QVector3D position;
    QVector3D normal;
    QVector3D color;
    float     emissive;
};

static void addQuad(std::vector<CarVertex> &v,
                    const QVector3D &p1, const QVector3D &p2,
                    const QVector3D &p3, const QVector3D &p4,
                    const QVector3D &col, float emissive = 0.0f) {
    QVector3D n = QVector3D::crossProduct(p2 - p1, p3 - p1).normalized();
    v.push_back({p1, n, col, emissive});
    v.push_back({p2, n, col, emissive});
    v.push_back({p3, n, col, emissive});
    v.push_back({p1, n, col, emissive});
    v.push_back({p3, n, col, emissive});
    v.push_back({p4, n, col, emissive});
}

static void addEdgeQuad(std::vector<CarVertex> &lines,
                        const QVector3D &p1, const QVector3D &p2,
                        const QVector3D &p3, const QVector3D &p4,
                        const QVector3D &col) {
    QVector3D n(0, 1, 0);
    lines.push_back({p1, n, col, 1.0f}); lines.push_back({p2, n, col, 1.0f});
    lines.push_back({p2, n, col, 1.0f}); lines.push_back({p3, n, col, 1.0f});
    lines.push_back({p3, n, col, 1.0f}); lines.push_back({p4, n, col, 1.0f});
    lines.push_back({p4, n, col, 1.0f}); lines.push_back({p1, n, col, 1.0f});
}

GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      frameTimer(new QTimer(this)),
      m_vbo(QOpenGLBuffer::VertexBuffer)
{
    setMinimumSize(1280, 720);
    setFocusPolicy(Qt::StrongFocus);

    connect(frameTimer, &QTimer::timeout, this, &GLWidget::onFrameTick);
    frameTimer->start(16); // 60 FPS
    dtTimer.start();

    trajectoryPoints.push_back(QPointF(0.0, 0.0));
}

GLWidget::~GLWidget() {
    makeCurrent();
    m_vbo.destroy();
    m_vao.destroy();
    doneCurrent();
}

void GLWidget::updateTelemetry(uint32_t seq, float angle, float speed, uint32_t mode) {
    currentSeq   = seq;
    currentAngle = angle;
    currentSpeed = std::max(0.0f, speed);
    currentMode  = mode;
    currentRpm   = 800.0f + currentSpeed * 32.0f;
}

void GLWidget::resetTrajectory() {
    odomX = 0.0;
    odomY = 0.0;
    totalDistM = 0.0;
    trajectoryPoints.clear();
    trajectoryPoints.push_back(QPointF(0.0, 0.0));
    update();
}

void GLWidget::initializeGL() {
    initializeOpenGLFunctions();
    initShaders();
    buildCarModel();
}

void GLWidget::initShaders() {
    const char *vshader = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;
        layout(location = 2) in vec3 aColor;
        layout(location = 3) in float aEmissive;
        uniform mat4 uMvp;
        uniform mat4 uModel;
        out vec3 vNormal;
        out vec3 vFragPos;
        out vec3 vColor;
        out float vEmissive;
        void main() {
            vFragPos = vec3(uModel * vec4(aPos, 1.0));
            vNormal = mat3(uModel) * aNormal;
            vColor = aColor;
            vEmissive = aEmissive;
            gl_Position = uMvp * vec4(aPos, 1.0);
        }
    )";

    const char *fshader = R"(
        #version 330 core
        in vec3 vNormal;
        in vec3 vFragPos;
        in vec3 vColor;
        in float vEmissive;
        uniform int uMode;
        out vec4 FragColor;

        void main() {
            vec3 modeTint = vec3(0.0, 0.9, 1.0);
            if (uMode == 2) {
                modeTint = vec3(1.0, 0.15, 0.15);
            } else if (uMode == 1) {
                modeTint = vec3(1.0, 0.65, 0.0);
            }

            if (vEmissive > 0.5) {
                vec3 glowCol = (vColor.r > 0.9 && vColor.g < 0.3) ? vColor : modeTint;
                FragColor = vec4(glowCol, 1.0);
                return;
            }

            vec3 base = mix(vColor, modeTint * 0.45, 0.5);
            vec3 norm = normalize(vNormal);
            vec3 lightDir = normalize(vec3(2.0, 5.0, 3.0) - vFragPos);
            vec3 viewDir = normalize(-vFragPos);

            float diff = max(dot(norm, lightDir), 0.0);
            vec3 reflectDir = reflect(-lightDir, norm);
            float spec = pow(max(dot(viewDir, reflectDir), 0.0), 24.0);

            vec3 result = base * (0.28 + 0.65 * diff) + vec3(0.45) * spec;
            FragColor = vec4(result, 1.0);
        }
    )";

    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vshader);
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fshader);
    m_program.link();
}

void GLWidget::buildCarModel() {
    std::vector<CarVertex> tris;
    std::vector<CarVertex> lines;

    QVector3D fn_l(-0.62f, 0.12f, -1.75f), fn_r( 0.62f, 0.12f, -1.75f);
    QVector3D hood_l(-0.68f, 0.38f, -0.65f), hood_r( 0.68f, 0.38f, -0.65f);
    QVector3D roof_fl(-0.52f, 0.72f, -0.15f), roof_fr( 0.52f, 0.72f, -0.15f);
    QVector3D roof_rl(-0.52f, 0.72f,  0.55f), roof_rr( 0.52f, 0.72f,  0.55f);
    QVector3D trunk_l(-0.68f, 0.44f,  1.15f), trunk_r( 0.68f, 0.44f,  1.15f);
    QVector3D re_l(-0.62f, 0.28f,  1.75f), re_r( 0.62f, 0.28f,  1.75f);
    QVector3D b_fn_l(-0.62f, -0.08f, -1.75f), b_fn_r( 0.62f, -0.08f, -1.75f);
    QVector3D b_re_l(-0.62f, -0.08f,  1.75f), b_re_r( 0.62f, -0.08f,  1.75f);

    QVector3D bodyCol(0.20f, 0.42f, 0.62f);
    QVector3D glassCol(0.06f, 0.14f, 0.24f);
    QVector3D neonCol(0.0f, 1.0f, 1.0f);

    addQuad(tris, fn_l, fn_r, hood_r, hood_l, bodyCol);
    addQuad(tris, hood_l, hood_r, roof_fr, roof_fl, glassCol);
    addQuad(tris, roof_fl, roof_fr, roof_rr, roof_rl, bodyCol);
    addQuad(tris, roof_rl, roof_rr, trunk_r, trunk_l, glassCol);
    addQuad(tris, trunk_l, trunk_r, re_r, re_l, bodyCol);
    addQuad(tris, b_fn_l, b_fn_r, fn_r, fn_l, bodyCol);
    addQuad(tris, re_l, re_r, b_re_r, b_re_l, bodyCol);
    addQuad(tris, hood_l, roof_fl, roof_rl, trunk_l, glassCol);
    addQuad(tris, roof_fr, hood_r, trunk_r, roof_rr, glassCol);
    addQuad(tris, b_fn_l, fn_l, re_l, b_re_l, bodyCol);
    addQuad(tris, fn_r, b_fn_r, b_re_r, re_r, bodyCol);

    // ヘッドライト＆テールランプ
    addQuad(tris, QVector3D(-0.58f, 0.05f, -1.76f), QVector3D(-0.28f, 0.05f, -1.76f),
                  QVector3D(-0.28f, 0.14f, -1.76f), QVector3D(-0.58f, 0.14f, -1.76f),
                  QVector3D(0.8f, 1.0f, 1.0f), 1.0f);
    addQuad(tris, QVector3D( 0.28f, 0.05f, -1.76f), QVector3D( 0.58f, 0.05f, -1.76f),
                  QVector3D( 0.58f, 0.14f, -1.76f), QVector3D( 0.28f, 0.14f, -1.76f),
                  QVector3D(0.8f, 1.0f, 1.0f), 1.0f);
    addQuad(tris, QVector3D(-0.58f, 0.16f,  1.76f), QVector3D( 0.58f, 0.16f,  1.76f),
                  QVector3D( 0.58f, 0.24f,  1.76f), QVector3D(-0.58f, 0.24f,  1.76f),
                  QVector3D(1.0f, 0.1f, 0.1f), 1.0f);

    // 自発光ネオンエッジ
    addEdgeQuad(lines, fn_l, fn_r, hood_r, hood_l, neonCol);
    addEdgeQuad(lines, hood_l, hood_r, roof_fr, roof_fl, neonCol);
    addEdgeQuad(lines, roof_fl, roof_fr, roof_rr, roof_rl, neonCol);
    addEdgeQuad(lines, roof_rl, roof_rr, trunk_r, trunk_l, neonCol);
    addEdgeQuad(lines, trunk_l, trunk_r, re_r, re_l, neonCol);
    addEdgeQuad(lines, b_fn_l, fn_l, re_l, b_re_l, neonCol);
    addEdgeQuad(lines, fn_r, b_fn_r, b_re_r, re_r, neonCol);

    // 床面サイバーリング
    const int segs = 48;
    for (int r_idx = 0; r_idx < 2; ++r_idx) {
        float rad = (r_idx == 0) ? 2.05f : 2.30f;
        for (int i = 0; i < segs; ++i) {
            float a1 = float(i) / segs * 2.0f * float(M_PI);
            float a2 = float(i + 1) / segs * 2.0f * float(M_PI);
            lines.push_back({QVector3D(rad * qCos(a1), -0.12f, rad * qSin(a1)), QVector3D(0, 1, 0), neonCol, 1.0f});
            lines.push_back({QVector3D(rad * qCos(a2), -0.12f, rad * qSin(a2)), QVector3D(0, 1, 0), neonCol, 1.0f});
        }
    }

    m_triVertexCount  = static_cast<int>(tris.size());
    m_lineVertexCount = static_cast<int>(lines.size());

    std::vector<CarVertex> allVerts = tris;
    allVerts.insert(allVerts.end(), lines.begin(), lines.end());

    m_vao.create();
    m_vao.bind();
    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(allVerts.data(), int(allVerts.size() * sizeof(CarVertex)));

    m_program.bind();
    m_program.enableAttributeArray(0);
    m_program.setAttributeBuffer(0, GL_FLOAT, offsetof(CarVertex, position), 3, sizeof(CarVertex));
    m_program.enableAttributeArray(1);
    m_program.setAttributeBuffer(1, GL_FLOAT, offsetof(CarVertex, normal),   3, sizeof(CarVertex));
    m_program.enableAttributeArray(2);
    m_program.setAttributeBuffer(2, GL_FLOAT, offsetof(CarVertex, color),    3, sizeof(CarVertex));
    m_program.enableAttributeArray(3);
    m_program.setAttributeBuffer(3, GL_FLOAT, offsetof(CarVertex, emissive), 1, sizeof(CarVertex));
    m_program.release();

    m_vao.release();
    m_vbo.release();
}

void GLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void GLWidget::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_R) {
        resetTrajectory();
    }
    QOpenGLWidget::keyPressEvent(event);
}

void GLWidget::onFrameTick() {
    qint64 elapsedNs = dtTimer.nsecsElapsed();
    dtTimer.restart();
    double dt = double(elapsedNs) / 1e9;

    if (dt > 0.0 && dt < 0.5 && currentSpeed > 0.05f) {
        double v_ms = double(currentSpeed) / 3.6;
        double yawRad = qDegreesToRadians(double(currentAngle));
        double dx = v_ms * std::cos(yawRad) * dt;
        double dy = v_ms * std::sin(yawRad) * dt;

        odomX += dx;
        odomY += dy;
        totalDistM += std::hypot(dx, dy);

        QPointF newPt(odomX, odomY);
        if (trajectoryPoints.empty() ||
            QLineF(trajectoryPoints.back(), newPt).length() >= 0.15) {
            trajectoryPoints.push_back(newPt);
            if (trajectoryPoints.size() > MAX_TRAIL_POINTS) {
                trajectoryPoints.pop_front();
            }
        }
    }
    update();
}

void GLWidget::draw3DVehicleViewport(int w, int h) {
    if (m_triVertexCount <= 0) return;

    // 画面上部中央の 3D POSTURE 枠に合わせてビューポート設定 (OpenGLは左下原点)
    int vpX = int(w * 0.38f);
    int vpY = int(h * 0.72f);
    int vpW = int(w * 0.24f);
    int vpH = int(h * 0.20f);

    glViewport(vpX, vpY, vpW, vpH);
    glEnable(GL_SCISSOR_TEST);
    glScissor(vpX, vpY, vpW, vpH);
    glEnable(GL_DEPTH_TEST);

    QMatrix4x4 proj;
    proj.perspective(38.0f, float(vpW) / float(std::max(1, vpH)), 0.1f, 100.0f);

    QMatrix4x4 model;
    model.translate(0.0f, -0.20f, -4.3f);
    model.rotate(26.0f, 1.0f, 0.0f, 0.0f);
    model.rotate(currentAngle - 90.0f, 0.0f, 1.0f, 0.0f);

    m_program.bind();
    m_program.setUniformValue("uMvp", proj * model);
    m_program.setUniformValue("uModel", model);
    m_program.setUniformValue("uMode", static_cast<int>(currentMode));

    m_vao.bind();
    glDrawArrays(GL_TRIANGLES, 0, m_triVertexCount);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, m_triVertexCount, m_lineVertexCount);
    m_vao.release();
    m_program.release();

    // QPainter 描画前に OpenGL ステートを復元
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, w, h);
}

void GLWidget::drawCarNavigationMap(QPainter &p, const QRectF &navRect) {
    p.save();

    QPainterPath clipPath;
    clipPath.addRoundedRect(navRect, 18, 18);
    p.setClipPath(clipPath);

    QLinearGradient bgGrad(navRect.topLeft(), navRect.bottomLeft());
    bgGrad.setColorAt(0.0, QColor(8, 18, 30, 235));
    bgGrad.setColorAt(1.0, QColor(4, 10, 18, 245));
    p.fillRect(navRect, bgGrad);

    QPointF mapCenter = navRect.center();
    mapCenter.ry() += 15.0;

    // 動的ズーム計算
    double maxSpan = 25.0;
    for (const QPointF &pt : trajectoryPoints) {
        double dx = std::abs(pt.x() - odomX);
        double dy = std::abs(pt.y() - odomY);
        maxSpan = std::max({maxSpan, dx * 1.25, dy * 1.25});
    }
    maxSpan = std::min(maxSpan, 400.0);
    double scalePxPerMeter = (std::min(navRect.width(), navRect.height()) * 0.42) / maxSpan;

    double gridStepM = 10.0;
    if (maxSpan > 180.0) gridStepM = 50.0;
    else if (maxSpan > 80.0) gridStepM = 25.0;
    else if (maxSpan < 35.0) gridStepM = 5.0;

    // 自車追従グリッド
    p.setPen(QPen(QColor(0, 180, 220, 35), 1.0));
    double startX = std::floor((odomX - maxSpan * 1.5) / gridStepM) * gridStepM;
    double endX   = odomX + maxSpan * 1.5;
    for (double gx = startX; gx <= endX; gx += gridStepM) {
        double sx = mapCenter.x() + (gx - odomX) * scalePxPerMeter;
        p.drawLine(QPointF(sx, navRect.top()), QPointF(sx, navRect.bottom()));
    }

    double startY = std::floor((odomY - maxSpan * 1.5) / gridStepM) * gridStepM;
    double endY   = odomY + maxSpan * 1.5;
    for (double gy = startY; gy <= endY; gy += gridStepM) {
        double sy = mapCenter.y() - (gy - odomY) * scalePxPerMeter;
        p.drawLine(QPointF(navRect.left(), sy), QPointF(navRect.right(), sy));
    }

    // 距離リング
    p.setPen(QPen(QColor(0, 210, 255, 55), 1.2, Qt::DashLine));
    double ring1Px = (maxSpan * 0.45) * scalePxPerMeter;
    double ring2Px = (maxSpan * 0.90) * scalePxPerMeter;
    p.drawEllipse(mapCenter, ring1Px, ring1Px);
    p.drawEllipse(mapCenter, ring2Px, ring2Px);

    // 出発原点マーカー
    QPointF originScreen(mapCenter.x() + (0.0 - odomX) * scalePxPerMeter,
                         mapCenter.y() - (0.0 - odomY) * scalePxPerMeter);
    if (navRect.contains(originScreen)) {
        p.setPen(QPen(QColor(255, 200, 0, 180), 2.0));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(originScreen, 6.0, 6.0);
        p.setFont(QFont("Sans", 8, QFont::Bold));
        p.drawText(originScreen + QPointF(8, 4), "START (0,0)");
    }

    // 走行軌跡
    if (trajectoryPoints.size() >= 2) {
        QPainterPath path;
        for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
            QPointF pt = trajectoryPoints[i];
            QPointF sp(mapCenter.x() + (pt.x() - odomX) * scalePxPerMeter,
                       mapCenter.y() - (pt.y() - odomY) * scalePxPerMeter);
            if (i == 0) path.moveTo(sp);
            else path.lineTo(sp);
        }

        QColor glowColor = (currentMode == 2) ? QColor(255, 50, 50, 90)
                         : (currentMode == 1) ? QColor(255, 180, 0, 90)
                                              : QColor(0, 240, 255, 85);
        p.setPen(QPen(glowColor, 9.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.setBrush(Qt::NoBrush);
        p.drawPath(path);

        QColor coreColor = (currentMode == 2) ? QColor(255, 110, 110, 255)
                         : (currentMode == 1) ? QColor(255, 220, 60, 255)
                                              : QColor(80, 255, 180, 255);
        p.setPen(QPen(coreColor, 3.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPath(path);
    }

    // 自車位置マーカー
    p.save();
    p.translate(mapCenter);
    p.rotate(-currentAngle + 90.0f);

    QRadialGradient beamGrad(QPointF(0, -35), 45);
    beamGrad.setColorAt(0.0, QColor(0, 255, 200, 110));
    beamGrad.setColorAt(1.0, QColor(0, 255, 200, 0));
    p.setPen(Qt::NoPen);
    p.setBrush(beamGrad);
    QPolygonF beamPoly;
    beamPoly << QPointF(0, 0) << QPointF(-24, -65) << QPointF(24, -65);
    p.drawPolygon(beamPoly);

    QPolygonF navArrow;
    navArrow << QPointF(0, -16) << QPointF(12, 12) << QPointF(0, 6) << QPointF(-12, 12);
    p.setPen(QPen(Qt::white, 2.0));
    p.setBrush(QColor(0, 220, 255));
    p.drawPolygon(navArrow);
    p.restore();

    // オーバーレイ枠と数値表示
    p.setClipping(false);
    QColor borderColor = (currentMode == 2) ? QColor(255, 60, 60, 220)
                       : (currentMode == 1) ? QColor(255, 170, 0, 220)
                                            : QColor(0, 210, 255, 180);
    p.setPen(QPen(borderColor, 2.2));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(navRect, 18, 18);

    QRectF headerRect(navRect.left() + 14, navRect.top() + 10, navRect.width() - 28, 32);
    p.setPen(QColor(180, 240, 255));
    p.setFont(QFont("Sans", 11, QFont::Bold));
    p.drawText(headerRect, Qt::AlignLeft | Qt::AlignVCenter,
               "SDV CAR NAVIGATION  |  REAL-TIME ODOMETRY TRAJECTORY");
    p.drawText(headerRect, Qt::AlignRight | Qt::AlignVCenter,
               QString("GRID: %1m  [Press 'R' Reset]").arg(int(gridStepM)));

    QRectF footerBg(navRect.left() + 12, navRect.bottom() - 46, navRect.width() - 24, 36);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 25, 45, 200));
    p.drawRoundedRect(footerBg, 8, 8);

    p.setPen(QColor(100, 255, 210));
    p.setFont(QFont("Monospace", 11, QFont::Bold));
    QString navStats = QString("POS X: %1 m   |   POS Y: %2 m   |   HDG: %3°   |   TRIP: %4 m")
                           .arg(odomX, 6, 'f', 1)
                           .arg(odomY, 6, 'f', 1)
                           .arg(currentAngle, 5, 'f', 1)
                           .arg(totalDistM, 6, 'f', 1);
    p.drawText(footerBg, Qt::AlignCenter, navStats);

    p.restore();
}

void GLWidget::drawCompactGauge(QPainter &p, QPointF center, float radius,
                                float value, float maxVal, const QString &title,
                                const QString &unit, QColor accentColor) {
    p.save();

    QRadialGradient bg(center, radius);
    bg.setColorAt(0.0, QColor(12, 22, 36, 230));
    bg.setColorAt(0.85, QColor(6, 12, 22, 240));
    bg.setColorAt(1.0, QColor(0, 180, 255, 90));
    p.setPen(QPen(accentColor, 2.0));
    p.setBrush(bg);
    p.drawEllipse(center, radius, radius);

    float startAngleDeg = 225.0f;
    float totalSpanDeg  = 270.0f;
    int ticks = 9;
    p.setFont(QFont("Sans", 8, QFont::Bold));

    for (int i = 0; i <= ticks; ++i) {
        float ratio = float(i) / float(ticks);
        float deg = startAngleDeg - ratio * totalSpanDeg;
        float rad = qDegreesToRadians(deg);

        QPointF pOuter(center.x() + std::cos(rad) * (radius * 0.86f),
                       center.y() - std::sin(rad) * (radius * 0.86f));
        QPointF pInner(center.x() + std::cos(rad) * (radius * 0.74f),
                       center.y() - std::sin(rad) * (radius * 0.74f));
        p.setPen(QPen(Qt::white, 1.8));
        p.drawLine(pInner, pOuter);

        QPointF pText(center.x() + std::cos(rad) * (radius * 0.58f) - 14,
                      center.y() - std::sin(rad) * (radius * 0.58f) + 5);
        int labelVal = int(std::round((ratio * maxVal) / (maxVal > 1000 ? 1000.0f : 1.0f)));
        p.setPen(QColor(190, 220, 240));
        p.drawText(pText, QString::number(labelVal));
    }

    float clampedRatio = std::clamp(value / maxVal, 0.0f, 1.0f);
    float needleDeg = startAngleDeg - clampedRatio * totalSpanDeg;
    float needleRad = qDegreesToRadians(needleDeg);
    QPointF needleTip(center.x() + std::cos(needleRad) * (radius * 0.78f),
                      center.y() - std::sin(needleRad) * (radius * 0.78f));

    p.setPen(QPen(QColor(255, 70, 70), 3.2, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(center, needleTip);

    p.setPen(Qt::NoPen);
    p.setBrush(accentColor);
    p.drawEllipse(center, 7, 7);

    p.setPen(Qt::white);
    p.setFont(QFont("Sans", 16, QFont::Bold));
    QRectF valRect(center.x() - radius * 0.6f, center.y() + radius * 0.15f, radius * 1.2f, 26);
    p.drawText(valRect, Qt::AlignCenter, QString::number(int(value)));

    p.setPen(accentColor);
    p.setFont(QFont("Sans", 8, QFont::Bold));
    QRectF unitRect(center.x() - radius * 0.6f, center.y() + radius * 0.42f, radius * 1.2f, 18);
    p.drawText(unitRect, Qt::AlignCenter, title + " [" + unit + "]");

    p.restore();
}

void GLWidget::drawHudBanner(QPainter &p, int w, int h) {
    Q_UNUSED(h);
    QString statusText = QString("QNX SAFETY ISLAND : NORMAL  |  SEQ: %1  |  ROS 2 ODOMETRY LINKED").arg(currentSeq);
    QColor barColor(0, 170, 230, 190);

    if (currentMode == 1) {
        statusText = "ADAS WARNING : SPEED LIMITER ACTIVE (160 km/h)";
        barColor = QColor(240, 150, 0, 220);
    } else if (currentMode == 2) {
        statusText = "EMERGENCY STOP : ADAS AUTONOMOUS BRAKING INTERVENTION";
        barColor = QColor(230, 30, 30, 235);
    }

    QRectF topBanner(w * 0.18, 12, w * 0.64, 34);
    p.setPen(Qt::NoPen);
    p.setBrush(barColor);
    p.drawRoundedRect(topBanner, 8, 8);

    p.setPen(Qt::white);
    p.setFont(QFont("Sans", 11, QFont::Bold));
    p.drawText(topBanner, Qt::AlignCenter, statusText);
}

void GLWidget::paintGL() {
    int w = width();
    int h = height();

    if (currentMode == 2) glClearColor(0.22f, 0.03f, 0.03f, 1.0f);
    else if (currentMode == 1) glClearColor(0.18f, 0.11f, 0.02f, 1.0f);
    else glClearColor(0.03f, 0.06f, 0.10f, 1.0f);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 1. 画面上部ミニ枠に縮小 3D 車両を描画 (GLSLシェーダー + VBO)
    draw3DVehicleViewport(w, h);

    // 2. 2D オーバーレイ（中央カーナビ + 左右縮小メーター）
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawHudBanner(painter, w, h);

    // 3D車両ミニ枠のフレームとラベル
    QRectF mini3dBox(w * 0.38, h * 0.08, w * 0.24, h * 0.20);
    painter.setPen(QPen(QColor(0, 200, 255, 130), 1.8));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(mini3dBox, 10, 10);
    painter.setFont(QFont("Sans", 8, QFont::Bold));
    painter.setPen(QColor(160, 225, 255));
    painter.drawText(mini3dBox.adjusted(10, 5, -8, -4), Qt::AlignTop | Qt::AlignLeft, "3D POSTURE");

    // 中央大型カーナビゲーション領域
    QRectF navRect(w * 0.24, h * 0.30, w * 0.52, h * 0.65);
    drawCarNavigationMap(painter, navRect);

    // 左右の縮小メーター
    float compactRadius = std::min(w, h) * 0.155f;
    QPointF leftGaugeCenter(w * 0.12f, h * 0.62f);
    QPointF rightGaugeCenter(w * 0.88f, h * 0.62f);

    QColor accent = (currentMode == 2) ? QColor(255, 80, 80)
                  : (currentMode == 1) ? QColor(255, 190, 40)
                                       : QColor(0, 220, 255);

    drawCompactGauge(painter, leftGaugeCenter, compactRadius,
                     currentSpeed, 240.0f, "SPEED", "km/h", accent);

    drawCompactGauge(painter, rightGaugeCenter, compactRadius,
                     currentRpm, 8000.0f, "TACHO", "x1000 r/min", accent);

    painter.end();
}
