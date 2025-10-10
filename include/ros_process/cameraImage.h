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

class WebSocketWorker;

class CameraImageMonitor : public QObject {
    Q_OBJECT
public:
    explicit CameraImageMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~CameraImageMonitor();

public slots:
    void start(); // send subscribe request via worker
    void onMessageReceived(const QString &message);
    void requestFrame(); // main thread requests the latest decoded frame (emitted back)
    void setTargetSize(const QSize &size); // desired display size (worker will scale to this)
    void setMaxFps(int fps); // throttle maximum frame rate emitted to UI

signals:
    void imageReceived(const QImage &image);

private:
    WebSocketWorker *m_worker;
    QSize m_targetSize;
    int m_frameIntervalMs = 33; // default ~30 FPS
    QElapsedTimer m_lastDecodeTimer;
    QImage m_latestImage;
    QMutex m_latestMutex;
};


#endif // CAMERAIMAGE_H