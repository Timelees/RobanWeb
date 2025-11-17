#include "ros_process/slamPose.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

#include "cmath"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


PoseMonitor::PoseMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    pose_stamped_topic_name = loadTopicFromConfig("pose_slam_topic");
    pose_stamped_topic_type = loadTopicFromConfig("pose_slam_topic_type");

    if(pose_stamped_topic_name.isEmpty()) {
        pose_stamped_topic_name = "/initialpose"; 
    }
    if(pose_stamped_topic_type.isEmpty()) {
        pose_stamped_topic_type = "geometry_msgs/PoseWithCovarianceStamped"; 
    }
}

PoseMonitor::~PoseMonitor() {}

// 订阅Pose话题
void PoseMonitor::start(){
    if(!m_worker) return;
    // 发送订阅请求给initialpose话题
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = pose_stamped_topic_name;
    subscribeMsg["type"] = pose_stamped_topic_type;
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));

}

// 处理收到的Pose消息
QVector3D PoseMonitor::onMessageReceived(const QString &message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if(!doc.isObject()) return QVector3D();
    
    QJsonObject obj = doc.object();
    QString topic = obj["topic"].toString();

    // 处理initialpose位姿数据
    if(obj["op"].toString() == "publish" && topic == pose_stamped_topic_name){
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "Received PoseStamped message:" << msgJson;
            // 帮助函数：从 QJsonValue 中安全提取 double（支持数字或字符串形式）
            auto getDouble = [&](const QJsonValue &v, bool &ok)->double{
                ok = false;
                if (v.isDouble()) { ok = true; return v.toDouble(); }
                if (v.isString()) { bool bok=false; double d = v.toString().toDouble(&bok); if (bok) { ok=true; return d; } }
                return 0.0;
            };

            double x = 0.0, y = 0.0, z = 0.0;
            double ox = 0.0, oy = 0.0, oz = 0.0, ow = 1.0;

            // 支持两种常见的消息嵌套结构：
            // 1) geometry_msgs/PoseStamped : msg.pose.position, msg.pose.orientation
            // 2) geometry_msgs/PoseWithCovarianceStamped : msg.pose.pose.position, msg.pose.pose.orientation
            QJsonObject poseContainer;
            if (msgObj.contains("pose") && msgObj["pose"].isObject()) {
                poseContainer = msgObj["pose"].toObject();
                // 如果是带协方差的消息，内部会再有一个 "pose" 字段
                if (poseContainer.contains("pose") && poseContainer["pose"].isObject())
                    poseContainer = poseContainer["pose"].toObject();
            } else if (msgObj.contains("position") && msgObj.contains("orientation")) {
                // 有些发送端直接把 position/orientation 放在 msg 顶层
                poseContainer = msgObj;
            }

            bool okx=false, oky=false, okz=false, okox=false, okoy=false, okoz=false, okut=false;
            if (!poseContainer.isEmpty()) {
                QJsonObject posObj = poseContainer.value("position").toObject();
                QJsonObject oriObj = poseContainer.value("orientation").toObject();
                x = getDouble(posObj.value("x"), okx);
                y = getDouble(posObj.value("y"), oky);
                z = getDouble(posObj.value("z"), okz);
                ox = getDouble(oriObj.value("x"), okox);
                oy = getDouble(oriObj.value("y"), okoy);
                oz = getDouble(oriObj.value("z"), okoz);
                ow = getDouble(oriObj.value("w"), okut);
                // qDebug() << "Parsed Pose - position:("<<x<<","<<y<<","<<z<<") \n orientation:("<<ox<<","<<oy<<","<<oz<<","<<ow<<")";
            } else {
                qDebug() << "PoseMonitor::onMessageReceived: pose field not found in msg";
                return x_y_yaw; // 返回上一次的值
            }

            // 如果不能成功解析 orientation 的四元数，则直接返回
            if (!(okox && okoy && okoz && okut)) {
                qDebug() << "PoseMonitor::onMessageReceived: invalid orientation data, skip";
                return x_y_yaw;
            }

            // 由四元数转为欧拉角（单位：弧度），使用稳定的 asin/clamp 方法以避免数值问题
            // roll (x-axis rotation)
            roll = atan2(2.0 * (ow * ox + oy * oz), 1.0 - 2.0 * (ox * ox + oy * oy));
            // pitch (y-axis rotation)
            pitch = asin(2.0 * (ow * oy - oz * ox));
            // yaw (z-axis rotation)
            yaw = atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz));
            // 转为度
            yaw = yaw * 180.0 / M_PI;
            x_y_yaw = QVector3D(float(x), float(y), float(yaw));
            // 发射信号以便 SceneManager 或其他监听器可以接收更新
            emit poseUpdated(x_y_yaw);
    }

    return x_y_yaw;
}