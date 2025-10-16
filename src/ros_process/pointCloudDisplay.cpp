#include "ros_process/pointCloudDisplay.h"
#include <QPainter>
#include <QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>

PointCloudDisplay::PointCloudDisplay(QWidget *parent)
    : QOpenGLWidget(parent)
    , rotY(0.0f)
    , rotX(0.0f)
    , distance(2.5f)
    , lastPos(QPoint())
{
    setMinimumSize(320, 240);
    setFocusPolicy(Qt::StrongFocus);
}

PointCloudDisplay::~PointCloudDisplay() = default;
// 接收点云数据槽函数
void PointCloudDisplay::onPointCloudReceived(const QList<QVector3D> &points)
{
    {
        QMutexLocker locker(&mtx_);
        m_points = points;
    }
    update();
}
// 清空点云数据
void PointCloudDisplay::clearPointCloud()
{
    {
        QMutexLocker locker(&mtx_);
        m_points.clear();
    }
    update();
}

void PointCloudDisplay::initializeGL()
{
    initializeOpenGLFunctions();                    // 初始化函数
    // white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glPointSize(1.2f);          // 显示点云尺寸
}

void PointCloudDisplay::resizeGL(int w, int h)
{
    glViewport(0,0,w,h);
}

void PointCloudDisplay::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QList<QVector3D> pts;
    {
        QMutexLocker locker(&mtx_);
        pts = m_points;
    }

    if (pts.isEmpty()) return;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = float(width()) / float(height() ? height() : 1);
    float fov = 45.0f;
    float znear = 0.01f;
    float zfar = 1000.0f;
    float top = tanf(fov * M_PI / 360.0f) * znear;
    glFrustum(-top*aspect, top*aspect, -top, top, znear, zfar);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // Apply camera transform: translate (pan), then rotate, then zoom
    glTranslatef(panX, panY, -distance);
    glRotatef(rotX, 1.0f, 0.0f, 0.0f);
    glRotatef(rotY, 0.0f, 1.0f, 0.0f);

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f); // red points
    for (const QVector3D &p : pts) {
        glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();
}

void PointCloudDisplay::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
    lastButton = event->button();
    event->accept();
}

void PointCloudDisplay::mouseMoveEvent(QMouseEvent *event)
{
    QPoint delta = event->pos() - lastPos;
    lastPos = event->pos();
    if (event->buttons() & Qt::LeftButton) {
        // pan
        panX += delta.x() * 0.002f; // scale to view
        panY -= delta.y() * 0.002f;
    } else if (event->buttons() & Qt::RightButton) {
        // rotate
        rotY += delta.x() * 0.5f;
        rotX += delta.y() * 0.5f;
    }
    update();
    event->accept();
}
void PointCloudDisplay::wheelEvent(QWheelEvent *event)
{
    // zoom in/out
    int delta = event->angleDelta().y();
    if (delta > 0) distance = qMax(0.1f, distance - 0.1f);
    else distance += 0.1f;
    update();
    event->accept();
}
