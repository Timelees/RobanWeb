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
    if (parent) {
        // match the UI placeholder widget size and policy when embedded in shDialog
        setMinimumSize(parent->size());
        setSizePolicy(parent->sizePolicy());
        resize(parent->size());
    } else {
        setMinimumSize(320, 240);
    }
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

    // Try to compute ModelView = inverse(Twc) from the received Twc (camera->world) matrix.
    // computeModelViewFromTwc is thread-safe and reads camera_mat under lock.
    GLfloat modelView[16];
    if (computeModelViewFromTwc(modelView)) {
        // Compose final ModelView = I * B, where B = inverse(Twc) (modelView)
        // and I is the interactive transform built from pan/rotate/zoom. We do this by
        // loading identity, applying I, then multiplying by B (glMultMatrixf).
        glLoadIdentity();
        glTranslatef(panX, panY, -distance);
        glRotatef(rotX, 1.0f, 0.0f, 0.0f);
        glRotatef(rotY, 0.0f, 1.0f, 0.0f);
        glMultMatrixf(modelView);
    } else {
        // fallback: interactive camera (pan/rotate/zoom)
        glLoadIdentity();
        glTranslatef(panX, panY, -distance);
        glRotatef(rotX, 1.0f, 0.0f, 0.0f);
        glRotatef(rotY, 0.0f, 1.0f, 0.0f);
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

bool PointCloudDisplay::computeModelViewFromTwc(GLfloat out[16])
{
    QMutexLocker locker(&mtx_);
    if (camera_mat.size() != 16) return false;
    // camera_mat is row-major Twc: [ r00 r01 r02 tx; r10 r11 r12 ty; r20 r21 r22 tz; 0 0 0 1 ]
    // inverse for rigid transform: invT = [ R^T, -R^T * t; 0 1 ]
    double r00 = camera_mat[0]; double r01 = camera_mat[1]; double r02 = camera_mat[2]; double tx = camera_mat[3];
    double r10 = camera_mat[4]; double r11 = camera_mat[5]; double r12 = camera_mat[6]; double ty = camera_mat[7];
    double r20 = camera_mat[8]; double r21 = camera_mat[9]; double r22 = camera_mat[10]; double tz = camera_mat[11];
    // last row is assumed [0 0 0 1]

    // R^T
    double m00 = r00; double m01 = r10; double m02 = r20;
    double m10 = r01; double m11 = r11; double m12 = r21;
    double m20 = r02; double m21 = r12; double m22 = r22;

    // -R^T * t
    double itx = -(m00 * tx + m01 * ty + m02 * tz);
    double ity = -(m10 * tx + m11 * ty + m12 * tz);
    double itz = -(m20 * tx + m21 * ty + m22 * tz);

    // pack into column-major GLfloat out[16]
    out[0]  = static_cast<GLfloat>(m00); out[4]  = static_cast<GLfloat>(m01); out[8]  = static_cast<GLfloat>(m02); out[12] = static_cast<GLfloat>(itx);
    out[1]  = static_cast<GLfloat>(m10); out[5]  = static_cast<GLfloat>(m11); out[9]  = static_cast<GLfloat>(m12); out[13] = static_cast<GLfloat>(ity);
    out[2]  = static_cast<GLfloat>(m20); out[6]  = static_cast<GLfloat>(m21); out[10] = static_cast<GLfloat>(m22); out[14] = static_cast<GLfloat>(itz);
    out[3]  = 0.0f;                         out[7]  = 0.0f;                         out[11] = 0.0f;                         out[15] = 1.0f;

    // Apply a 180-degree rotation about world Z to match ORB-SLAM2 Viewer orientation (rotate initial view)
    // RotZ180 * out  (left-multiply)
    GLfloat rot[16] = {
        -1.0f, 0.0f, 0.0f, 0.0f,
         0.0f,-1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f
    };
    GLfloat tmp[16];
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += rot[c*4 + k] * out[k*4 + r];
            }
            tmp[c*4 + r] = sum;
        }
    }
    memcpy(out, tmp, sizeof(tmp));

    return true;
}

// 鼠标事件
void PointCloudDisplay::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
    lastButton = event->button();
    // ensure widget receives keyboard focus so key events and focus dependent behavior work
    setFocus();
    event->accept();
}

void PointCloudDisplay::mouseMoveEvent(QMouseEvent *event)
{
    QPoint delta = event->pos() - lastPos;
    lastPos = event->pos();
    // use stored lastButton to determine interaction rather than event->buttons() which may be unreliable across platforms
    if (lastButton == Qt::LeftButton) {
        // pan
        panX += delta.x() * 0.002f; // scale to view
        panY -= delta.y() * 0.002f;
    } else if (lastButton == Qt::RightButton) {
        // rotate
        rotY += delta.x() * 0.5f;
        rotX += delta.y() * 0.5f;
    }
    update();
    event->accept();
}

void PointCloudDisplay::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    lastButton = Qt::NoButton;
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
        camera_poses.clear();
        camera_poses.append(qMakePair(pos, dir));
    }
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
    // temporarily disable depth test so world-space camera markers are clearly visible on top
    glDisable(GL_DEPTH_TEST);
    glPointSize(8.0f);
    glBegin(GL_POINTS);
    for (const auto &pr : poses) {
        const QVector3D &pp = pr.first;
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(pp.x(), pp.y(), pp.z());
    }
    glEnd();
    glPointSize(1.2f);
    glEnable(GL_DEPTH_TEST);

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

    // draw wireframe pyramid; draw on top by briefly disabling depth test for visibility
    glDisable(GL_DEPTH_TEST);
    glLineWidth(2.0f);
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
    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
    }
    // In follow mode (haveView) we draw a centered HUD camera glyph and do not draw
    // the world-space camera pose (to avoid duplicate/confusing markers).
    return;
}
