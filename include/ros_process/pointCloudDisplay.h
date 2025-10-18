#ifndef POINTCLOUDDISPLAY_H
#define POINTCLOUDDISPLAY_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector3D>
#include <QMutex>
#include <QList>
#include <QPoint>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>

class PointCloudDisplay : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit PointCloudDisplay(QWidget *parent = nullptr);
    ~PointCloudDisplay() override;

public slots:
    void onPointCloudReceived(const QList<QVector3D> &points);
    void clearPointCloud();
    // receive keyframe marker data (points and lines)
    void onKeyFrameMarkers(const QList<QVector3D> &points, const QList<QVector3D> &lines);
    void clearKeyFrames();
    // receive camera poses (position + direction)
    void onCameraPoseReceived(const QVector3D &pos, const QVector3D &dir);
    void clearCameraPoses();
    // receive OpenGL camera matrix (16 doubles)
    void onCameraMatrixReceived(const QList<double> &mat);
    void clearCameraMatrix();

protected:
    // mouse interaction
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    // drawing helpers
    void drawPointCloud(const QList<QVector3D> &pts);
    void drawKeyFrames(const QList<QVector3D> &kpts, const QList<QVector3D> &klines);
    void drawCameraPoses();

private:
    QList<QVector3D> m_points;
    // keyframe markers
    QList<QVector3D> kf_points;
    QList<QVector3D> kf_lines; // stored as sequential pairs [p0,p1,p2,p3,...]
    // camera poses published separately (position, direction)
    QList<QPair<QVector3D,QVector3D>> camera_poses;
    QMutex mtx_;
    // optional external OpenGL camera matrix (row-major 4x4 values)
    QVector<double> camera_mat;
    // compute ModelView (column-major) from Twc (row-major list in camera_mat). Returns true if produced.
    bool computeModelViewFromTwc(GLfloat out[16]);
 
    float rotY = 0.0f;
    float rotX = 0.0f;
    float distance = 2.5f;
    bool initialViewAligned = false; // whether we've auto-aligned the view to camera on first Twc
    QPoint lastPos;
    float panX = 0.0f;
    float panY = 0.0f;
    Qt::MouseButton lastButton = Qt::NoButton;
};

#endif // POINTCLOUDDISPLAY_H
