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

class PointCloudDisplay : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit PointCloudDisplay(QWidget *parent = nullptr);
    ~PointCloudDisplay() override;

public slots:
    void onPointCloudReceived(const QList<QVector3D> &points);
    void clearPointCloud();

protected:
    // mouse interaction
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    QList<QVector3D> m_points;
    QMutex mtx_;
    float rotY = 0.0f;
    float rotX = 0.0f;
    float distance = 2.5f;
    QPoint lastPos;
    float panX = 0.0f;
    float panY = 0.0f;
    Qt::MouseButton lastButton = Qt::NoButton;
};

#endif // POINTCLOUDDISPLAY_H
