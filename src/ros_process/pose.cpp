#include "ros_process/pose.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

#include "cmath"


PoseMonitor::PoseMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    pose_stamped_topic_name = loadTopicFromConfig("pose_stamped_topic");
    pose_stamped_topic_type = loadTopicFromConfig("pose_stamped_topic_type");

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
        qDebug() << "Received PoseStamped message:" << msgJson;

        double x, y, z, ox, oy, oz, ow;
        if(msgObj.contains("pose") && msgObj["pose"].isObject()){
            QJsonObject poseObj = msgObj["pose"].toObject();
            if(poseObj.contains("position") && poseObj["position"].isObject()
               && poseObj.contains("orientation") && poseObj["orientation"].isObject()){
                QJsonObject positionObj = poseObj["position"].toObject();
                QJsonObject orientationObj = poseObj["orientation"].toObject();
                // 提取位置和朝向数据
                x = positionObj.value("x").toDouble();
                y = positionObj.value("y").toDouble();
                z = positionObj.value("z").toDouble();
                ox = orientationObj.value("x").toDouble();
                oy = orientationObj.value("y").toDouble();
                oz = orientationObj.value("z").toDouble();
                ow = orientationObj.value("w").toDouble();

                qDebug() << "Position - x:" << x << "y:" << y << "z:" << z;
                qDebug() << "Orientation - x:" << ox << "y:" << oy << "z:" << oz << "w:" << ow;
            }
        }
        // 提取位置的x,y坐标和将orientation转换为欧拉角
        roll = atan2(2.0 * (ow * ox + oy * oz), 1.0 - 2.0 * (ox * ox + oy * oy));
        pitch = atan2(2.0 * (ow * oy - oz * ox), 1.0 - 2.0 * (oy * oy + oz * oz));
        yaw = atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oz * oz + ox * ox));

        // TODO: 按照slam_map.py输出的点位格式[x,y,yaw],这里应该返回这三个值
        // 和三维场景中的 QVector3D(-2.84738, 2.38419e-07, 0.0739546) 去做对应
        // 这里的z值如何对应呢？  -----z值不做对应，直接对应x，y。yaw值发出来作为旋转角通过setModelRotation设置模型旋转角度
        x_y_yaw = QVector3D(x, y, yaw);
        // 发射信号以便 SceneManager 或其他监听器可以接收更新
        emit poseUpdated(x_y_yaw);
    }

    return x_y_yaw;
}