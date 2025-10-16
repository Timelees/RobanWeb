#include "ros_process/slamMapPoint.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"
#include <QVector3D>
#include <QByteArray>
#include <QtGlobal>
#include <QJsonArray>
#include <QBuffer>
#include <QCryptographicHash>

SlamMapMonitor::SlamMapMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    slamPoint_topic_name = loadTopicFromConfig("slamPoint_topic");
    slamPoint_topic_type = loadTopicFromConfig("slamPoint_topic_type");
    slamKeyFrame_topic_name = loadTopicFromConfig("slamKeyFrame_topic");
    slamKeyFrame_topic_type = loadTopicFromConfig("slamKeyFrame_topic_type");
}

SlamMapMonitor::~SlamMapMonitor() {}

// 订阅SLAM点云话题
void SlamMapMonitor::start(){
    if(!m_worker)   return;
    // send subscribe request for PointCloud2
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = slamPoint_topic_name;
    subscribeMsg["type"] = slamPoint_topic_type;
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}
// 取消SLAM点云订阅
void SlamMapMonitor::stop(){
    if(!m_worker)   return;
    QJsonObject unsub;
    unsub["op"] = "unsubscribe";
    unsub["topic"] = slamPoint_topic_name;
    QJsonDocument doc(unsub);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}

// 接收到SLAM点云消息处理
void SlamMapMonitor::onMessageReceived(const QString &message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (!doc.isObject()) return;
    QJsonObject obj = doc.object();

    // rosbridge publish message format: {op:"publish", topic:"...", msg:{...}}
    if (obj["op"].toString() == "publish") {
        QString topic = obj["topic"].toString();
        if (topic != slamPoint_topic_name) return;

        QJsonObject msgObj = obj["msg"].toObject();

        // 解析PointCloud2数据结构
        QList<QVector3D> points;
        // fields: array of {name, offset, datatype, count}
        QJsonArray fields = msgObj["fields"].toArray();         // fields数组
        int x_offset=-1, y_offset=-1, z_offset=-1;
        int point_step = msgObj["point_step"].toInt(0);         // 每个点占用的字节数
        int data_length = msgObj["data"].toArray().size();
        // locate x,y,z offsets
        for (int i=0;i<fields.size();++i){
            QJsonObject f = fields[i].toObject();
            QString name = f["name"].toString();
            int offset = f["offset"].toInt();
            int datatype = f["datatype"].toInt();
            if (name=="x") x_offset = offset;
            if (name=="y") y_offset = offset;
            if (name=="z") z_offset = offset;
        }

        // data might be an array of numbers or a base64 string depending on rosbridge config
        if (msgObj.contains("data") && msgObj["data"].isArray()){
            QJsonArray dataArr = msgObj["data"].toArray();
            int total = dataArr.size();
            if (point_step<=0 && x_offset<0){
                // fallback: assume packed triples
                for (int i=0;i+2<total;i+=3){
                    float vx = float(dataArr[i].toDouble());
                    float vy = float(dataArr[i+1].toDouble());
                    float vz = float(dataArr[i+2].toDouble());
                    points.append(QVector3D(vx, vy, vz));
                }
            } else {
                int point_count = total / point_step;
                for (int p=0;p<point_count;++p){
                    int base = p*point_step;
                    float vx = x_offset>=0 ? float(dataArr[base + x_offset].toDouble()) : 0.0f;
                    float vy = y_offset>=0 ? float(dataArr[base + y_offset].toDouble()) : 0.0f;
                    float vz = z_offset>=0 ? float(dataArr[base + z_offset].toDouble()) : 0.0f;
                    points.append(QVector3D(vx, vy, vz));
                }
            }
        } else if (msgObj.contains("data") && msgObj["data"].isString()){
            // data is base64 string
            QString b64 = msgObj["data"].toString();
            QByteArray raw = QByteArray::fromBase64(b64.toUtf8());
            if (point_step<=0) point_step = 12; // 3 floats
            int point_count = raw.size() / point_step;
            for (int p=0;p<point_count;++p){
                int base = p*point_step;
                float vx = 0, vy = 0, vz = 0;
                if (x_offset>=0) memcpy(&vx, raw.constData()+base + x_offset, sizeof(float));
                if (y_offset>=0) memcpy(&vy, raw.constData()+base + y_offset, sizeof(float));
                if (z_offset>=0) memcpy(&vz, raw.constData()+base + z_offset, sizeof(float));
                    points.append(QVector3D(vx, vy, vz));
            }
        }

        if (!points.isEmpty()){         // 将解析成功的数据发送信号
            emit pointCloudReceived(points);
        } else {
            // fallback: print full msg for debugging
            QJsonDocument msgDoc(msgObj);
            QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
            qDebug() << "收到SLAM点云 JSON (topic:" << topic << ") :" << msgJson;
        }


    }
}