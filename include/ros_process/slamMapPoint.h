#ifndef SLAM_MAP_POINT_H
#define SLAM_MAP_POINT_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>
#include <QString>

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

private:
    WebSocketWorker *m_worker;
    QString slamPoint_topic_name;
    QString slamPoint_topic_type;
    QString slamKeyFrame_topic_name;
    QString slamKeyFrame_topic_type;

};


#endif // SLAM_MAP_POINT_H