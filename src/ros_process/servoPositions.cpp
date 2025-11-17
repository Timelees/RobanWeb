#include "ros_process/servoPositions.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

ServoPositionsMonitor::ServoPositionsMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    servo_positions_topic_name = loadTopicFromConfig("servo_positions_topic");
    servo_positions_topic_type = loadTopicFromConfig("servo_positions_topic_type");

    walking_status_topic_name = loadTopicFromConfig("walk_state_topic");
    walking_status_topic_type = loadTopicFromConfig("walk_state_topic_type");

    // qDebug() << "Loaded servo positions topic from config:"
    //          << servo_positions_topic_name << "(" << servo_positions_topic_type << ")";
    if(servo_positions_topic_name.isEmpty() || servo_positions_topic_type.isEmpty()) {
        servo_positions_topic_name = "/MediumSize/BodyHub/ServoPositions"; 
        servo_positions_topic_type = "bodyhub/ServoPositionAngle"; 
    }
    if(walking_status_topic_name.isEmpty() || walking_status_topic_type.isEmpty()) {
        walking_status_topic_name = "/MediumSize/BodyHub/WalkingStatus"; 
        walking_status_topic_type = "std_msgs/Float64";
        
    }
}

ServoPositionsMonitor::~ServoPositionsMonitor() {}



QVector<double> ServoPositionsMonitor::lastAngles() const {
    QMutexLocker locker(&m_mutex);
    return m_lastAngles;
}

void ServoPositionsMonitor::start(){
    if(!m_worker) return;
    // 发送订阅请求给servo positions话题
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = servo_positions_topic_name;
    subscribeMsg["type"] = servo_positions_topic_type;
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    // qDebug() << "ServoPositionsMonitor subscribing to topic:" << payload;
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));


    // 发送订阅请求给walking status话题
    QJsonObject subscribeMsg2;
    subscribeMsg2["op"] = "subscribe";
    subscribeMsg2["topic"] = walking_status_topic_name;
    subscribeMsg2["type"] = walking_status_topic_type;
    QJsonDocument doc2(subscribeMsg2);
    QString payload2 = QString::fromUtf8(doc2.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload2));

}

void ServoPositionsMonitor::onMessageReceived(const QString &message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if(!doc.isObject()) return;

    QJsonObject obj = doc.object();
    QString topic = obj["topic"].toString();
    // 处理伺服位置消息
    if(obj["op"].toString() == "publish" && topic == servo_positions_topic_name){
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "Received ServoPositions message: " << msgJson;

        // 保存原始消息并解析 angle 数组
        QVector<double> angles;
        if (msgObj.contains("angle") && msgObj["angle"].isArray()) {
            QJsonArray arr = msgObj["angle"].toArray();
            angles.reserve(arr.size());
            for (const QJsonValue &v : arr) {
                if (v.isDouble()) {
                    angles.append(v.toDouble());
                } else if (v.isString()) {
                    bool ok = false;
                    double d = v.toString().toDouble(&ok);
                    if (ok) angles.append(d);
                }
            }
        }

        bool shouldEmit = false;
        {
            QMutexLocker locker(&m_mutex);
                // 保存一份旧角度以供比较
                QVector<double> oldAngles = m_lastAngles;
                m_lastMsg = msgObj;
                m_lastAngles = angles;
                // 增加计数器，只有当累计到 m_batchSize 条消息时才发出更新信号，减少主线程负载
                m_msgCounter++;
                if (m_msgCounter >= m_batchSize) {
                    m_msgCounter = 0;
                    shouldEmit = true;
                }
                // 如果角度数组与上次相比发生显著变化，则立即发出更新（以降低行走时的延迟）
                const double SIGNIFICANT_ANGLE_DELTA = 0.5; // degrees
                if (!oldAngles.isEmpty() && oldAngles.size() == angles.size()) {
                    for (int i = 0; i < angles.size(); ++i) {
                        if (std::fabs(angles[i] - oldAngles[i]) > SIGNIFICANT_ANGLE_DELTA) {
                            shouldEmit = true;
                            break;
                        }
                    }
                }
        }
        if (shouldEmit) {
            // 通过信号通知（连接到 RobotManager 时使用 QueuedConnection）
            emit servoPositionsUpdated();
        }
    }
    // 处理walking status消息
    else if(obj["op"].toString() == "publish" && topic == walking_status_topic_name){
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "Received WalkingStatus message: " << msgJson;

        // 这里可以根据需要处理walking status消息内容
        if(msgObj.contains("data") && msgObj["data"].isDouble()) {
            double walkStatus = msgObj["data"].toDouble();
            qDebug() << "Current Walking Status:" << walkStatus;
            if(walkStatus == 1.0) {
                qDebug() << "Robot is walking.";
                // 先发出行走状态信号，告知上层播放行走动画
                emit walkingStatusUpdated();
                // 并且尽可能立即触发伺服/位置更新，减少行走动画与位置移动不同步的情况
                // 这里在 mutex 下读取最后的角度并立即发出 servoPositionsUpdated
                {
                    QMutexLocker locker(&m_mutex);
                    // 仅在已有角度数据时发出
                    if (!m_lastAngles.isEmpty()) {
                        // 直接发射更新信号以便上层尽快响应（通常为动画与位置相关逻辑）
                        emit servoPositionsUpdated();
                    }
                }
            } else {
                qDebug() << "Robot is standing.";
                // 当检测到非行走状态（例如 0.0）时，发送停止信号以便上层停止行走动画播放
                emit walkingStatusStopped();
            }
        }
    }

}