#include "ros_process/cameraImage.h"
#include "socket_process/websocketworker.h"
#include <QImage>
#include <QBuffer>
#include <QByteArray>
#include <QJsonValue>
#include <QtGlobal>
#include <QElapsedTimer>
#include <QSize>

CameraImageMonitor::CameraImageMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    m_lastDecodeTimer.start();
}

CameraImageMonitor::~CameraImageMonitor() {}

// 设置显示尺寸
void CameraImageMonitor::setTargetSize(const QSize &size) {
    m_targetSize = size;
}
// 设置最大帧率
void CameraImageMonitor::setMaxFps(int fps) {
    if (fps <= 0) return;
    m_frameIntervalMs = 1000 / fps;
}

// 订阅图像话题
void CameraImageMonitor::start(){
    if(!m_worker)   return;
    // send subscribe request for Image
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    // 订阅压缩图像话题
    subscribeMsg["topic"] = "/camera/color/image_raw/compressed";
    subscribeMsg["type"] = "sensor_msgs/CompressedImage";
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}

// 转换 JSON 为 QByteArray
static QByteArray jsonDataToByteArray(const QJsonValue &dataVal) {
    if (dataVal.isString()) {
        // base64 encoded string (could be compressed image like JPEG)
        QString s = dataVal.toString();
        return QByteArray::fromBase64(s.toUtf8());
    } else if (dataVal.isArray()) {
        QJsonArray arr = dataVal.toArray();
        QByteArray out;
        out.reserve(arr.size());
        for (const QJsonValue &v : arr) {
            int iv = v.toInt();
            out.append(static_cast<char>(iv & 0xFF));
        }
        return out;
    }
    return QByteArray();
}

// 处理接受数据
void CameraImageMonitor::onMessageReceived(const QString &message) {
   
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (!doc.isObject()) return;
    QJsonObject obj = doc.object();

    if (obj["op"].toString() == "publish"){
        QString topic = obj["topic"].toString();

        QJsonObject msgObj = obj["msg"].toObject();

        // compressed image path
        if (topic == "/camera/color/image_raw/compressed") {
            // sensor_msgs/CompressedImage: has fields 'format' and 'data'
            QString format = msgObj.value("format").toString();
            QJsonValue dataVal = msgObj.value("data");
            QByteArray bytes = jsonDataToByteArray(dataVal);
            if (bytes.isEmpty()) return;

            QImage img = QImage::fromData(bytes);
            if (img.isNull()) {
                qDebug() << "CameraImageMonitor: failed to decode compressed image, format=" << format << "bytes=" << bytes.size();
                return;
            }

            // throttle and store scaled image in cache (worker thread)
            qint64 elapsed = m_lastDecodeTimer.elapsed();
            if (elapsed < m_frameIntervalMs) return;
            m_lastDecodeTimer.restart();

            QImage toStore;
            if (!m_targetSize.isEmpty() && img.size() != m_targetSize) {
                toStore = img.scaled(m_targetSize, Qt::KeepAspectRatio, Qt::FastTransformation);
            } else {
                toStore = img;
            }
            {
                QMutexLocker locker(&m_latestMutex);
                m_latestImage = toStore;
            }
            return;
        }

        // 获取原图逻辑 raw sensor_msgs/Image path (fallback)
        if (topic != "/camera/color/image_raw") return;

        int width = msgObj.value("width").toInt();
        int height = msgObj.value("height").toInt();
        QString encoding = msgObj.value("encoding").toString();
        QJsonValue dataVal = msgObj.value("data");
        // 将图像数据转为 QByteArray
        QByteArray bytes = jsonDataToByteArray(dataVal);
        if (bytes.isEmpty() || width <= 0 || height <= 0) return;

        // First try to decode as compressed image (JPEG/PNG) even for raw topic payloads
        QImage img = QImage::fromData(bytes);
        if (!img.isNull()) {
            // throttle by max FPS (avoid excessive decoding)
            qint64 elapsed = m_lastDecodeTimer.elapsed();
            if (elapsed < m_frameIntervalMs) return;
            m_lastDecodeTimer.restart();

            // scale in worker thread if requested and store into latest cache
            QImage toStore;
            if (!m_targetSize.isEmpty() && img.size() != m_targetSize) {
                toStore = img.scaled(m_targetSize, Qt::KeepAspectRatio, Qt::FastTransformation);
            } else {
                toStore = img;
            }
            {
                QMutexLocker locker(&m_latestMutex);
                m_latestImage = toStore;
            }
            return;
        }

        // Otherwise interpret as raw pixel buffer. Determine bytes per pixel
        int bpp = 3;
        QImage::Format fmt = QImage::Format_Invalid;
        if (encoding == "mono8" || encoding == "gray" || encoding == "mono") {
            bpp = 1;
            fmt = QImage::Format_Grayscale8;
        } else if (encoding == "rgb8" || encoding == "rgb24") {
            bpp = 3;
            fmt = QImage::Format_RGB888;
        } else if (encoding == "bgr8") {
            bpp = 3;
            // we'll construct as BGR and swap
            fmt = QImage::Format_BGR888;
        } else if (encoding == "rgba8" || encoding == "rgba32") {
            bpp = 4;
            fmt = QImage::Format_RGBA8888;
        } else {
            // fallback assume RGB888
            bpp = 3;
            fmt = QImage::Format_RGB888;
        }

        int expected = width * height * bpp;
        if (bytes.size() < expected) {
            qDebug() << "CameraImageMonitor: raw buffer too small" << bytes.size() << "expected" << expected << "encoding:" << encoding;
            return;
        }

        int bytesPerLine = width * bpp;
        if (fmt == QImage::Format_BGR888) {
            QImage tmp(reinterpret_cast<const uchar*>(bytes.constData()), width, height, bytesPerLine, fmt);
            img = tmp.rgbSwapped().copy();
        } else {
            QImage tmp(reinterpret_cast<const uchar*>(bytes.constData()), width, height, bytesPerLine, fmt);
            img = tmp.copy();
        }

        if (!img.isNull()) {
            qint64 elapsed = m_lastDecodeTimer.elapsed();
            if (elapsed < m_frameIntervalMs) return;
            m_lastDecodeTimer.restart();

            QImage toStore;
            if (!m_targetSize.isEmpty() && img.size() != m_targetSize) {
                toStore = img.scaled(m_targetSize, Qt::KeepAspectRatio, Qt::FastTransformation);
            } else {
                toStore = img;
            }
            {
                QMutexLocker locker(&m_latestMutex);
                m_latestImage = toStore;
            }
        }
    }
    
}

void CameraImageMonitor::requestFrame()
{
    QImage snapshot;
    {
        QMutexLocker locker(&m_latestMutex);
        if (m_latestImage.isNull()) return;
        snapshot = m_latestImage;
        // clear to avoid re-sending same frame repeatedly
        m_latestImage = QImage();
    }
    // emit from whichever thread called requestFrame; UI will receive via queued connection
    emit imageReceived(snapshot);
}