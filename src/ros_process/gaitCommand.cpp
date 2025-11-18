#include "ros_process/gaitCommand.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

GaitCommandMonitor::GaitCommandMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    gait_command_topic_name = loadTopicFromConfig("gait_command_topic");
    gait_command_topic_type = loadTopicFromConfig("gait_command_topic_type");

    if(gait_command_topic_name.isEmpty() || gait_command_topic_type.isEmpty()) {
        gait_command_topic_name = "/gaitCommand"; 
        gait_command_topic_type = "std_msgs/Float64MultiArray"; 
    }
    // 初始化默认值
    m_x = 0.0;
    m_y = 0.0;
    m_delta = 0.0;
}

GaitCommandMonitor::~GaitCommandMonitor() {}

void GaitCommandMonitor::start(){
    if(!m_worker) return;
    // 发送订阅请求给gait command话题
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = gait_command_topic_name;
    subscribeMsg["type"] = gait_command_topic_type;
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    // qDebug() << "GaitCommandMonitor subscribing to topic:" << payload;
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}

void GaitCommandMonitor::onMessageReceived(const QString &message){
    // 处理收到的消息
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if(doc.isNull() || !doc.isObject()) {
        qWarning() << "GaitCommandMonitor received invalid JSON message:" << message;
        return;
    }
    QJsonObject obj = doc.object();
    QString topic = obj["topic"].toString();
    // 处理步态命令消息
    if(obj["op"].toString() == "publish" && topic == gait_command_topic_name){
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "Received GaitCommand message: " << msgJson;

        // 简单解析：期望 msg.data 为数组，如 {"data":[x,y,delta], ...}
        if (msgObj.contains("data") && msgObj["data"].isArray()) {
            QJsonArray arr = msgObj["data"].toArray();
            double x = 0.0, y = 0.0, delta = 0.0;
            if (arr.size() > 0) {
                QJsonValue v = arr.at(0);
                if (v.isDouble()) x = v.toDouble();
                else if (v.isString()) x = v.toString().toDouble();
            }
            if (arr.size() > 1) {
                QJsonValue v = arr.at(1);
                if (v.isDouble()) y = v.toDouble();
                else if (v.isString()) y = v.toString().toDouble();
            }
            if (arr.size() > 2) {
                QJsonValue v = arr.at(2);
                if (v.isDouble()) delta = v.toDouble();
                else if (v.isString()) delta = v.toString().toDouble();
            }

            // 保存并发出信号
            {
                QMutexLocker locker(&m_mutex);
                m_x = x;
                m_y = y;
                m_delta = delta;
            }
            // qDebug() << "GaitCommand parsed x,y,delta:" << m_x << m_y << m_delta;
            emit gaitCommandUpdated();
        } else {
            qDebug() << "GaitCommandMonitor: msg.data is not array or missing:" << msgJson;
        }

    }
}

double GaitCommandMonitor::gaitX() const {
    QMutexLocker locker(&m_mutex);
    return m_x;
}

double GaitCommandMonitor::gaitY() const {
    QMutexLocker locker(&m_mutex);
    return m_y;
}

double GaitCommandMonitor::gaitDelta() const {
    QMutexLocker locker(&m_mutex);
    return m_delta;
}