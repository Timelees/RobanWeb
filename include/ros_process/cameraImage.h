#ifndef CAMERAIMAGE_H
#define CAMERAIMAGE_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>
#include <QImage>
#include <QElapsedTimer>
#include <QSize>
#include <QMutex>
#include <QImage>
#include <QBuffer>
#include <QByteArray>
#include <QJsonValue>
#include <QtGlobal>
#include <QElapsedTimer>
#include <QSize>

class WebSocketWorker;

class CameraImageMonitor : public QObject {
    Q_OBJECT
public:
explicit CameraImageMonitor(WebSocketWorker *worker, QObject *parent = nullptr, const QString &topic_name = QString());
    ~CameraImageMonitor();

public slots:
    void start(); // send subscribe request via worker
    void stop(); // send unsubscribe request via worker
    void onMessageReceived(const QString &message);
    void requestFrame(); // main thread requests the latest decoded frame (emitted back)
    void setTargetSize(const QSize &size); // desired display size (worker will scale to this)
    void setMaxFps(int fps); // throttle maximum frame rate emitted to UI

signals:
    void imageReceived(const QImage &image);

private:
    void topic_parse();
    void init();

private:
    WebSocketWorker *m_worker;
    QSize m_targetSize;
    int m_frameIntervalMs = 33; // default ~30 FPS
    QElapsedTimer m_lastDecodeTimer;
    QImage m_latestImage;
    QMutex m_latestMutex;
    QString *topic_name;
    QString cameraCompressed_topic_name;    // Camera话题名称(压缩)
    QString cameraCompressed_topic_type;    // Camera话题类型(压缩)

    QString cameraRaw_topic_name;           // Camera话题名称（未压缩）
    QString cameraRaw_topic_type;           // Camera话题类型（未压缩）

    QString featureImageCompressed_topic_name;    // SLAM FeaturePoint话题名称(压缩)
    QString featureImageCompressed_topic_type;    // SLAM FeaturePoint话题类型(压缩)

    QString featureImageRaw_topic_name;          // SLAM FeaturePoint话题名称
    QString featureImageRaw_topic_type;          // SLAM FeaturePoint话题类型

    QString act_topic_name;                 // 实际使用话题名称
    QString act_topic_type;                 // 实际使用话题类型
};


#endif // CAMERAIMAGE_H