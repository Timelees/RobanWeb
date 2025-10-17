#ifndef SLAM_MAP_POINT_H
#define SLAM_MAP_POINT_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>
#include <QString>
#include <QList>
#include <QVector3D>
#include <QByteArray>
#include <QtGlobal>
#include <QJsonArray>
#include <QBuffer>
#include <QCryptographicHash>


class WebSocketWorker;

class SlamMapMonitor : public QObject {
    Q_OBJECT
public:
    explicit SlamMapMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~SlamMapMonitor();

public slots:
    void start();
    void stop();
    void onMessageReceived(const QString &message);

signals:
    void pointCloudReceived(const QList<QVector3D> &points);
    // Emitted when a keyframe message arrives (raw JSON object from rosbridge)
    void keyFrameReceived(const QJsonObject &msg);
    // parsed marker arrays: points and line segments (pairs in lines list should be interpreted sequentially)
    void keyFrameMarkers(const QList<QVector3D> &points, const QList<QVector3D> &lines);
    // Emitted when an OpenGL camera matrix (Float64MultiArray) is received
    void cameraMatrixReceived(const QList<double> &matrix);
    // Emitted when a geometry_msgs/PoseStamped camera pose is received.
    // pos: world position of camera center; dir: forward direction vector (unit) in world coords
    void cameraPoseReceived(const QVector3D &pos, const QVector3D &dir);

private:
    void loadTopicFromParams();

    QList<QVector3D> parsePointCloud(const QJsonObject &msgObj);    // 解析点云数据
    void parseKeyFrame(const QJsonObject &msg);                     // 解析关键帧数据
    void parseOpenGLMatrix(const QJsonObject &msg);                 // 解析OpenGL矩阵数据
    void parseCameraPose(const QJsonObject &msg);                   // 解析相机位置数据

private:
    WebSocketWorker *m_worker;
    QString slamPoint_topic_name;
    QString slamPoint_topic_type;
    QString slamKeyFrame_topic_name;
    QString slamKeyFrame_topic_type;
    QString cameraOpenGLMatrix_topic_name;
    QString cameraOpenGLMatrix_topic_type;
    QString cameraPose_topic_name;
    QString cameraPose_topic_type;

};


#endif // SLAM_MAP_POINT_H