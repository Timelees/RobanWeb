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

    // continue even if main point cloud is empty because keyframe markers should still be drawn
    // if both are empty, nothing to draw
    QList<QVector3D> kpts_check;
    QList<QVector3D> klines_check;
    QList<QPair<QVector3D,QVector3D>> camposes_check;
    {
        QMutexLocker locker(&mtx_);
        kpts_check = kf_points;
        klines_check = kf_lines;
        camposes_check = camera_poses;
    }
    if (pts.isEmpty() && kpts_check.isEmpty() && klines_check.isEmpty() && camposes_check.isEmpty()) return;

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

    // If we have an external OpenGL matrix, multiply it here
    {
        QMutexLocker locker(&mtx_);
        if (camera_mat.size() == 16) {
            // assume incoming mat is row-major 4x4, OpenGL expects column-major
            GLfloat gmat[16];
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    gmat[c*4 + r] = static_cast<GLfloat>(camera_mat[r*4 + c]);
                }
            }
            glMultMatrixf(gmat);
        }
    }

    // 绘制点云
    drawPointCloud(pts);

    QList<QVector3D> kpts;
    QList<QVector3D> klines;
    {
        QMutexLocker locker(&mtx_);
        kpts = kf_points;
        klines = kf_lines;
    }
    // 绘制关键帧
    drawKeyFrames(kpts, klines);
    // 绘制单独发布的相机位姿
    drawCameraPoses();
}

// 鼠标事件
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

void PointCloudDisplay::onKeyFrameMarkers(const QList<QVector3D> &points, const QList<QVector3D> &lines)
{
    {
        QMutexLocker locker(&mtx_);
        kf_points = points;
        kf_lines = lines;
    }
    update();
}

void PointCloudDisplay::onCameraPoseReceived(const QVector3D &pos, const QVector3D &dir)
{
    {
        QMutexLocker locker(&mtx_);
        camera_poses.append(qMakePair(pos, dir));
    }
    // qDebug() << "PointCloudDisplay::onCameraPoseReceived -> pos=" << pos << " dir=" << dir;
    update();
}

void PointCloudDisplay::clearCameraPoses()
{
    {
        QMutexLocker locker(&mtx_);
        camera_poses.clear();
    }
    update();
}

void PointCloudDisplay::clearKeyFrames()
{
    {
        QMutexLocker locker(&mtx_);
        kf_points.clear();
        kf_lines.clear();
    }
    update();
}
// 更新相机矩阵
void PointCloudDisplay::onCameraMatrixReceived(const QList<double> &mat)
{
    QMutexLocker locker(&mtx_);
    camera_mat.clear();
    for (double v : mat) camera_mat.append(v);
    while (camera_mat.size() > 16) camera_mat.removeLast();
    update();
}
// 清空相机矩阵
void PointCloudDisplay::clearCameraMatrix()
{
    QMutexLocker locker(&mtx_);
    camera_mat.clear();
    update();
}
// 绘制点云数据
void PointCloudDisplay::drawPointCloud(const QList<QVector3D> &pts)
{
    if (pts.isEmpty()) return;
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f); // red points
    for (const QVector3D &p : pts) {
        glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();
}
// 绘制关键帧数据
void PointCloudDisplay::drawKeyFrames(const QList<QVector3D> &kpts, const QList<QVector3D> &klines)
{
    // Draw simple points for keyframes (small)
    if (!kpts.isEmpty()){
        const float kfPointSize = 2.0f;
        glPointSize(kfPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0f, 0.0f, 1.0f);
        for (const QVector3D &p : kpts) glVertex3f(p.x(), p.y(), p.z());
        glEnd();
        glPointSize(1.2f);
    }

    // Draw wireframe pyramid (MapDrawer-style) for keyframes using associated direction lines (if any)
    if (!kpts.isEmpty()){
        glColor3f(0.0f, 0.0f, 1.0f); // blue
        const float w = 0.05f;       // keyframe size (same default as ORB-SLAM2)
        const float h = w * 0.75f;
        const float z = w * 0.6f;
        glLineWidth(1.0f);

        for (const QVector3D &p : kpts) {
            // find matching line whose start is near p
            QVector3D dir(0,0,1);
            bool hasDir = false;
            for (int i=0;i+1<klines.size(); i+=2){
                const QVector3D &a = klines[i];
                const QVector3D &b = klines[i+1];
                if ((a - p).length() < 1e-4f || (a - p).length() < 0.01f) {
                    dir = (b - a);
                    if (dir.length() > 1e-6f) {
                        dir.normalize();
                        hasDir = true;
                    }
                    break;
                }
            }

            // compute rotation from +Z to dir (same method as before)
            QVector3D zAxis(0,0,1);
            QVector3D axis = QVector3D::crossProduct(zAxis, dir);
            float dot = QVector3D::dotProduct(zAxis, dir);
            float ang = 0.0f;
            if (hasDir) {
                float lenAxis = axis.length();
                if (lenAxis > 1e-6f) {
                    axis /= lenAxis;
                    ang = acosf(qBound(-1.0f, dot, 1.0f)) * 180.0f / M_PI; // degrees
                } else {
                    if (dot < 0) {
                        axis = QVector3D(1,0,0);
                        ang = 180.0f;
                    } else {
                        ang = 0.0f;
                    }
                }
            }

            glPushMatrix();
            // position and orientation
            glTranslatef(p.x(), p.y(), p.z());
            if (ang != 0.0f) glRotatef(ang, axis.x(), axis.y(), axis.z());

            // draw the wireframe pyramid with GL_LINES, matching MapDrawer layout
            glBegin(GL_LINES);
            // center to four corners
            glVertex3f(0,0,0); glVertex3f(w,h,z);
            glVertex3f(0,0,0); glVertex3f(w,-h,z);
            glVertex3f(0,0,0); glVertex3f(-w,-h,z);
            glVertex3f(0,0,0); glVertex3f(-w,h,z);

            // connect corners to form base rectangle
            glVertex3f(w,h,z); glVertex3f(w,-h,z);
            glVertex3f(-w,h,z); glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z); glVertex3f(w,h,z);
            glVertex3f(-w,-h,z); glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }
  
}

void PointCloudDisplay::drawCameraPoses(){
    // Draw camera poses (green, slightly larger pyramids)
    QList<QPair<QVector3D,QVector3D>> poses;
    QList<QVector3D> klines;
    {
        QMutexLocker locker(&mtx_);
        poses = camera_poses;
        klines = kf_lines;
    }

    if (poses.isEmpty()) return;
    // qDebug() << "PointCloudDisplay::drawCameraPoses count=" << poses.size();

    glColor3f(0.0f, 1.0f, 0.0f);
    const float w = 0.05f; // same base size as keyframes
    const float wcam = w * 1.5f; // slightly larger
    const float hcam = wcam * 0.75f;
    const float zcam = wcam * 0.6f;
    glLineWidth(1.0f);

    // also draw bright green points at camera centers for visibility
    glPointSize(6.0f);
    glBegin(GL_POINTS);
    for (const auto &pr : poses) {
        const QVector3D &pp = pr.first;
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(pp.x(), pp.y(), pp.z());
    }
    glEnd();
    glPointSize(1.2f);

    for (const auto &pr : poses) {
        const QVector3D &p = pr.first;
        QVector3D dir = pr.second;

        // if dir is near-zero, try to find a matching direction from klines (like keyframes)
        if (dir.length() < 1e-6f) {
            for (int i=0;i+1<klines.size(); i+=2){
                const QVector3D &a = klines[i];
                const QVector3D &b = klines[i+1];
                if ((a - p).length() < 0.01f) {
                    dir = (b - a);
                    if (dir.length() > 1e-6f) { dir.normalize(); break; }
                }
            }
        }

        if (dir.length() < 1e-6f) dir = QVector3D(0,0,1);
        else dir.normalize();

        // compute rotation from +Z to dir
        QVector3D zAxis(0,0,1);
        QVector3D axis = QVector3D::crossProduct(zAxis, dir);
        float dot = QVector3D::dotProduct(zAxis, dir);
        float ang = 0.0f;
        float lenAxis = axis.length();
        if (lenAxis > 1e-6f) { axis /= lenAxis; ang = acosf(qBound(-1.0f, dot, 1.0f)) * 180.0f / M_PI; }
        else { if (dot < 0) { axis = QVector3D(1,0,0); ang = 180.0f; } }

        glPushMatrix();
        glTranslatef(p.x(), p.y(), p.z());
        if (ang != 0.0f) glRotatef(ang, axis.x(), axis.y(), axis.z());

        glBegin(GL_LINES);
        glVertex3f(0,0,0); glVertex3f(wcam,hcam,zcam);
        glVertex3f(0,0,0); glVertex3f(wcam,-hcam,zcam);
        glVertex3f(0,0,0); glVertex3f(-wcam,-hcam,zcam);
        glVertex3f(0,0,0); glVertex3f(-wcam,hcam,zcam);

        glVertex3f(wcam,hcam,zcam); glVertex3f(wcam,-hcam,zcam);
        glVertex3f(-wcam,hcam,zcam); glVertex3f(-wcam,-hcam,zcam);

        glVertex3f(-wcam,hcam,zcam); glVertex3f(wcam,hcam,zcam);
        glVertex3f(-wcam,-hcam,zcam); glVertex3f(wcam,-hcam,zcam);
        glEnd();

        glPopMatrix();
    }
}
