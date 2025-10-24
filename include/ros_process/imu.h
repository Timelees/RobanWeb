#ifndef IMU_H
#define IMU_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>

class WebSocketWorker;

class ImuMonitor : public QObject {
    Q_OBJECT
public:
    explicit ImuMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~ImuMonitor();

public slots:
    void start(); // send subscribe request via worker
    void onMessageReceived(const QString &message);

signals:
    void orientationUpdated(double w, double x, double y, double z);
    void angularVelocityUpdated(double x, double y, double z);
    void linearAccelerationUpdated(double x, double y, double z);

private:
    WebSocketWorker *m_worker;
    QString imu_topic_name; // IMU话题名称
    QString imu_pose_topic_name; // IMU位姿话题名称
};


#endif // IMU_H