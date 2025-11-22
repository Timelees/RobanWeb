#ifndef SLAMPOSE_H
#define SLAMPOSE_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>
#include <QVector3D>

class WebSocketWorker;

class PoseMonitor : public QObject{
    Q_OBJECT
public:
    explicit PoseMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~PoseMonitor();

public slots:
    void start(); // send subscribe request via worker
    QVector3D onMessageReceived(const QString &message);
    // If the monitor has a last-seen pose, emit it via poseUpdated.
    // This is useful for callers in other threads that attach later and want
    // to immediately receive the most recent pose.
    void emitLastPose();
    // Return the last known pose (x,y,yaw) as a QVector3D. This slot is safe to
    // invoke with Qt::BlockingQueuedConnection from other threads to synchronously
    // retrieve the last pose value.
    QVector3D lastPoseValue();
    bool hasLastPose();
signals:
    void poseUpdated(const QVector3D &pose);

private:
    WebSocketWorker *m_worker;
    QString pose_stamped_topic_name; // 位姿话题名称
    QString pose_stamped_topic_type; // 位姿话题类型
    double roll, pitch, yaw; // 欧拉角
    QVector3D x_y_yaw; // 存储x,y,yaw
    // 平滑/去抖参数：在连续收到位姿时对 x,y,yaw 做指数平滑，值范围 (0,1], 越小越平滑
    double m_poseSmoothingAlpha = 0.6;
    bool m_haveLastPose = false;
};



#endif // SLAMPOSE_H