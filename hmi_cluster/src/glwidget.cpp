#include "glwidget.h"
#include <QtMath>
#include <vector>

struct CarVertex {
    QVector3D position;
    QVector3D normal;
    QVector3D color;
    float     emissive;
};

GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_vbo(QOpenGLBuffer::VertexBuffer) {
}

GLWidget::~GLWidget() {
    makeCurrent();
    m_vbo.destroy();
    m_vao.destroy();
    doneCurrent();
}

void GLWidget::updateTelemetry(uint32_t seq, float angle, float speed, uint32_t mode) {
    m_seq = seq;
    m_angle = angle;
    m_speed = speed;
    m_mode = mode;
    update();
}

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

    QVector3D bodyCol(0.20f, 0.40f, 0.58f);
    QVector3D glassCol(0.06f, 0.12f, 0.20f);
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
        float rad = (r_idx == 0) ? 2.1f : 2.35f;
        for (int i = 0; i < segs; ++i) {
            float a1 = (float)i / segs * 2.0f * M_PI;
            float a2 = (float)(i + 1) / segs * 2.0f * M_PI;
            lines.push_back({QVector3D(rad * qCos(a1), -0.12f, rad * qSin(a1)), QVector3D(0,1,0), neonCol, 1.0f});
            lines.push_back({QVector3D(rad * qCos(a2), -0.12f, rad * qSin(a2)), QVector3D(0,1,0), neonCol, 1.0f});
        }
    }

    m_triVertexCount = static_cast<int>(tris.size());
    m_lineVertexCount = static_cast<int>(lines.size());

    std::vector<CarVertex> allVerts = tris;
    allVerts.insert(allVerts.end(), lines.begin(), lines.end());

    m_vao.create();
    m_vao.bind();
    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(allVerts.data(), allVerts.size() * sizeof(CarVertex));

    m_program.enableAttributeArray(0);
    m_program.setAttributeBuffer(0, GL_FLOAT, offsetof(CarVertex, position), 3, sizeof(CarVertex));
    m_program.enableAttributeArray(1);
    m_program.setAttributeBuffer(1, GL_FLOAT, offsetof(CarVertex, normal), 3, sizeof(CarVertex));
    m_program.enableAttributeArray(2);
    m_program.setAttributeBuffer(2, GL_FLOAT, offsetof(CarVertex, color), 3, sizeof(CarVertex));
    m_program.enableAttributeArray(3);
    m_program.setAttributeBuffer(3, GL_FLOAT, offsetof(CarVertex, emissive), 1, sizeof(CarVertex));

    m_vao.release();
    m_vbo.release();
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

            vec3 result = base * (0.25 + 0.65 * diff) + vec3(0.45) * spec;
            FragColor = vec4(result, 1.0);
        }
    )";

    m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, vshader);
    m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, fshader);
    m_program.link();
}

void GLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    m_projection.setToIdentity();
    m_projection.perspective(42.0f, GLfloat(w) / GLfloat(h ? h : 1), 0.1f, 100.0f);
}

void GLWidget::paintGL() {
    // -------------------------------------------------------------------------
    // 1. 3D OpenGL 描画 (深度テスト有効)
    // -------------------------------------------------------------------------
    glEnable(GL_DEPTH_TEST);

    if (m_mode == 2) {
        glClearColor(0.08f, 0.01f, 0.01f, 1.0f);
    } else if (m_mode == 1) {
        glClearColor(0.08f, 0.05f, 0.01f, 1.0f);
    } else {
        glClearColor(0.02f, 0.03f, 0.05f, 1.0f);
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_triVertexCount > 0) {
        m_program.bind();

        QMatrix4x4 model;
        model.translate(0.0f, -0.15f, -5.8f);
        model.rotate(22.0f, 1.0f, 0.0f, 0.0f);
        model.rotate(m_angle, 0.0f, 1.0f, 0.0f);

        m_program.setUniformValue("uMvp", m_projection * model);
        m_program.setUniformValue("uModel", model);
        m_program.setUniformValue("uMode", static_cast<int>(m_mode));

        m_vao.bind();
        glDrawArrays(GL_TRIANGLES, 0, m_triVertexCount);
        glLineWidth(2.5f);
        glDrawArrays(GL_LINES, m_triVertexCount, m_lineVertexCount);
        m_vao.release();

        m_program.release();
    }

    glDisable(GL_DEPTH_TEST);

    // -------------------------------------------------------------------------
    // 2. 2D オーバーレイ描画 (QPainter を paintGL 内で確実に begin/end 管理)
    // -------------------------------------------------------------------------
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);

    int w = width();
    int h = height();

    drawTopStatusBar(p);

    int meterRadius = qMin(w / 5, h * 3 / 8);
    int leftCx = w * 0.20;
    int rightCx = w * 0.80;
    int meterCy = h * 0.54;

    drawSpeedometer(p, leftCx, meterCy, meterRadius);
    drawTachometer(p, rightCx, meterCy, meterRadius);
    drawCenterHud(p, w / 2, h * 0.88);

    // ★重要: メモリリーク防止のためペインターを明示的に解放
    p.end();
}

void GLWidget::drawTopStatusBar(QPainter &p) {
    int w = width();
    QRect barRect(30, 20, w - 60, 48);
    p.setPen(QColor(0, 220, 200, 100));
    p.setBrush(QColor(12, 18, 26, 230));
    p.drawRoundedRect(barRect, 8, 8);

    QFont font = p.font();
    font.setFamily("monospace");
    font.setPointSize(13);
    font.setBold(true);
    p.setFont(font);

    QString modeStr = (m_mode == 2) ? "EMERGENCY STOP" : ((m_mode == 1) ? "SYSTEM WARNING" : "NORMAL / READY");
    QColor modeColor = (m_mode == 2) ? QColor(255, 60, 60) : ((m_mode == 1) ? QColor(255, 180, 0) : QColor(0, 255, 200));

    p.setPen(QColor(200, 220, 240));
    p.drawText(barRect.adjusted(25, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft,
               QString("QNX RTOS 60Hz STREAM | SEQ: %1 | ANGLE: %2°").arg(m_seq, 6).arg(m_angle, 5, 'f', 1));

    p.setPen(modeColor);
    p.drawText(barRect.adjusted(0, 0, -25, 0), Qt::AlignVCenter | Qt::AlignRight,
               QString("[ MODE: %1 ]").arg(modeStr));
}

void GLWidget::drawSpeedometer(QPainter &p, int cx, int cy, int radius) {
    QRectF rect(cx - radius, cy - radius, radius * 2, radius * 2);
    QColor mainColor = (m_mode == 2) ? QColor(255, 50, 50) : ((m_mode == 1) ? QColor(255, 180, 0) : QColor(0, 230, 255));
    QColor glowColor(mainColor.red(), mainColor.green(), mainColor.blue(), 75);

    p.setPen(QPen(QColor(0, 180, 255, 70), 2));
    p.setBrush(QColor(12, 18, 26, 220));
    p.drawEllipse(rect);

    p.save();
    p.translate(cx, cy);
    for (int v = 0; v <= 220; v += 20) {
        float angleDeg = 135.0f + (float)v / 220.0f * 270.0f;
        p.save();
        p.rotate(angleDeg);
        p.setPen(QPen((v % 40 == 0) ? QColor(220, 240, 255) : QColor(90, 110, 130), (v % 40 == 0) ? 3 : 1));
        p.drawLine(0, -radius + 10, 0, -radius + ((v % 40 == 0) ? 26 : 16));
        p.restore();
    }
    p.restore();

    p.setPen(QPen(QColor(35, 50, 70, 160), 6));
    p.setBrush(Qt::NoBrush);
    p.drawArc(rect, -45 * 16, 270 * 16);

    float speedRatio = qBound(0.0f, m_speed / 220.0f, 1.0f);
    p.setPen(QPen(glowColor, 18, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(rect, 225 * 16, -int(speedRatio * 270 * 16));
    p.setPen(QPen(mainColor, 7, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(rect, 225 * 16, -int(speedRatio * 270 * 16));

    // 自発光レーザー指針
    p.save();
    p.translate(cx, cy);
    p.rotate(135.0f + speedRatio * 270.0f);
    p.setPen(Qt::NoPen);

    QPoint glowPoly[4] = { QPoint(-12, 15), QPoint(12, 15), QPoint(4, -radius + 14), QPoint(-4, -radius + 14) };
    p.setBrush(glowColor);
    p.drawConvexPolygon(glowPoly, 4);

    QPoint mainPoly[4] = { QPoint(-5, 10), QPoint(5, 10), QPoint(2, -radius + 12), QPoint(-2, -radius + 12) };
    p.setBrush(mainColor);
    p.drawConvexPolygon(mainPoly, 4);

    p.setPen(QPen(QColor(255, 255, 255, 240), 2));
    p.drawLine(0, 0, 0, -radius + 15);
    p.restore();

    p.setPen(QPen(mainColor, 2));
    p.setBrush(QColor(20, 28, 40));
    p.drawEllipse(QPoint(cx, cy), 14, 14);

    QFont font = p.font();
    font.setPointSize(radius / 5);
    font.setBold(true);
    p.setFont(font);
    p.setPen(QColor(245, 250, 255));
    p.drawText(QRect(cx - radius, cy + radius * 0.22, radius * 2, radius * 0.4),
               Qt::AlignCenter, QString::number(int(m_speed)));

    font.setPointSize(radius / 12);
    p.setFont(font);
    p.setPen(mainColor);
    p.drawText(QRect(cx - radius, cy + radius * 0.55, radius * 2, radius * 0.25),
               Qt::AlignCenter, "km/h");
}

void GLWidget::drawTachometer(QPainter &p, int cx, int cy, int radius) {
    QRectF rect(cx - radius, cy - radius, radius * 2, radius * 2);
    float rpm = (m_angle / 360.0f) * 8000.0f;
    float rpmRatio = qBound(0.0f, rpm / 8000.0f, 1.0f);

    QColor mainColor = (rpm > 6000.0f || m_mode == 2) ? QColor(255, 50, 50) : ((m_mode == 1) ? QColor(255, 180, 0) : QColor(150, 90, 255));
    QColor glowColor(mainColor.red(), mainColor.green(), mainColor.blue(), 75);

    p.setPen(QPen(QColor(140, 90, 255, 70), 2));
    p.setBrush(QColor(12, 18, 26, 220));
    p.drawEllipse(rect);

    p.save();
    p.translate(cx, cy);
    for (int r = 0; r <= 8; ++r) {
        float angleDeg = 135.0f + (float)r / 8.0f * 270.0f;
        p.save();
        p.rotate(angleDeg);
        p.setPen(QPen((r >= 6) ? QColor(255, 70, 70) : QColor(200, 220, 240), 3));
        p.drawLine(0, -radius + 10, 0, -radius + 26);
        p.restore();
    }
    p.restore();

    p.setPen(QPen(QColor(35, 50, 70, 160), 6));
    p.setBrush(Qt::NoBrush);
    p.drawArc(rect, -45 * 16, 270 * 16);

    p.setPen(QPen(glowColor, 18, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(rect, 225 * 16, -int(rpmRatio * 270 * 16));
    p.setPen(QPen(mainColor, 7, Qt::SolidLine, Qt::RoundCap));
    p.drawArc(rect, 225 * 16, -int(rpmRatio * 270 * 16));

    p.save();
    p.translate(cx, cy);
    p.rotate(135.0f + rpmRatio * 270.0f);
    p.setPen(Qt::NoPen);

    QPoint glowPoly[4] = { QPoint(-12, 15), QPoint(12, 15), QPoint(4, -radius + 14), QPoint(-4, -radius + 14) };
    p.setBrush(glowColor);
    p.drawConvexPolygon(glowPoly, 4);

    QPoint mainPoly[4] = { QPoint(-5, 10), QPoint(5, 10), QPoint(2, -radius + 12), QPoint(-2, -radius + 12) };
    p.setBrush(mainColor);
    p.drawConvexPolygon(mainPoly, 4);

    p.setPen(QPen(QColor(255, 255, 255, 240), 2));
    p.drawLine(0, 0, 0, -radius + 15);
    p.restore();

    p.setPen(QPen(mainColor, 2));
    p.setBrush(QColor(20, 28, 40));
    p.drawEllipse(QPoint(cx, cy), 14, 14);

    QFont font = p.font();
    font.setPointSize(radius / 5);
    font.setBold(true);
    p.setFont(font);
    p.setPen(QColor(245, 250, 255));
    p.drawText(QRect(cx - radius, cy + radius * 0.22, radius * 2, radius * 0.4),
               Qt::AlignCenter, QString::number(int(rpm)));

    font.setPointSize(radius / 12);
    p.setFont(font);
    p.setPen(mainColor);
    p.drawText(QRect(cx - radius, cy + radius * 0.55, radius * 2, radius * 0.25),
               Qt::AlignCenter, "RPM");
}

void GLWidget::drawCenterHud(QPainter &p, int cx, int cy) {
    QFont font = p.font();
    font.setPointSize(18);
    font.setBold(true);
    p.setFont(font);

    int boxW = 46;
    int spacing = 16;
    int startX = cx - (boxW * 4 + spacing * 3) / 2;

    const char *gears[] = {"P", "R", "N", "D"};
    for (int i = 0; i < 4; ++i) {
        QRect gRect(startX + i * (boxW + spacing), cy - 22, boxW, 44);
        bool isCurrent = (i == 3);

        if (isCurrent) {
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(0, 220, 255, 220));
            p.drawRoundedRect(gRect, 8, 8);
            p.setPen(QColor(10, 15, 20));
        } else {
            p.setPen(QColor(60, 80, 100));
            p.setBrush(QColor(18, 24, 32, 180));
            p.drawRoundedRect(gRect, 8, 8);
            p.setPen(QColor(110, 130, 150));
        }
        p.drawText(gRect, Qt::AlignCenter, gears[i]);
    }
}
