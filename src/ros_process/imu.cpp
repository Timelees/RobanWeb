#include "ros_process/imu.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

ImuMonitor::ImuMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    imu_topic_name = loadTopicFromConfig("imu_topic");
    imu_pose_topic_name = loadTopicFromConfig("imu_pose_topic");

    if(imu_pose_topic_name.isEmpty() || imu_topic_name.isEmpty()) {
        imu_pose_topic_name = "/MediumSize/SensorHub/ImuPose"; 
        imu_topic_name = "/MediumSize/SensorHub/Imu";           
    }

}

ImuMonitor::~ImuMonitor() {}

// 订阅IMU话题
void ImuMonitor::start(){
    if(!m_worker) return;
    // send subscribe request for IMU
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = imu_topic_name;
    subscribeMsg["type"] = "sensor_msgs/Imu";
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));

    // send subscribe request for IMU pose
    QJsonObject poseSubscribeMsg;
    poseSubscribeMsg["op"] = "subscribe";
    poseSubscribeMsg["topic"] = imu_pose_topic_name;
    poseSubscribeMsg["type"] = "geometry_msgs/PoseStamped";
    doc = QJsonDocument(poseSubscribeMsg);
    payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    // qDebug() << "ImuMonitor subscribing to IMU pose topic:" << payload;
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));

}

void ImuMonitor::onMessageReceived(const QString &message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if(!doc.isObject()) return;
    QJsonObject obj = doc.object();
    QString topic = obj["topic"].toString();

    // 处理imu加速度和线速度数据
    if (obj["op"].toString() == "publish" && topic == imu_topic_name) {
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
    
        // angular_velocity
        if (msgObj.contains("angular_velocity") && msgObj["angular_velocity"].isObject()) {
            QJsonObject ang = msgObj["angular_velocity"].toObject();
            double ax = ang.value("x").toDouble();
            double ay = ang.value("y").toDouble();
            double az = ang.value("z").toDouble();
            emit angularVelocityUpdated(ax, ay, az);
        }

        // linear_acceleration
        if (msgObj.contains("linear_acceleration") && msgObj["linear_acceleration"].isObject()) {
            QJsonObject lin = msgObj["linear_acceleration"].toObject();
            double lx = lin.value("x").toDouble();
            double ly = lin.value("y").toDouble();
            double lz = lin.value("z").toDouble();
            emit linearAccelerationUpdated(lx, ly, lz);
        }
    }
    // 处理imu的位姿信息
    if(obj["op"].toString() == "publish" &&  topic == imu_pose_topic_name){
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "ImuMonitor received IMU pose message:" << msgJson;
            // orientation: be defensive about possible wrapping (PoseStamped -> msg.pose.pose)
            if (msgObj.contains("pose") && msgObj["pose"].isObject()) {
                QJsonObject p = msgObj["pose"].toObject();
                QJsonObject poseObj;
                // Handle PoseStamped where msg.pose.pose exists
                if (p.contains("pose") && p["pose"].isObject()) poseObj = p["pose"].toObject();
                else poseObj = p;

                if (poseObj.contains("orientation") && poseObj["orientation"].isObject()){
                    QJsonObject ori = poseObj["orientation"].toObject();
                    double ow = ori.value("w").toDouble();
                    double ox = ori.value("x").toDouble();
                    double oy = ori.value("y").toDouble();
                    double oz = ori.value("z").toDouble();
                  
                    emit orientationUpdated(ow, ox, oy, oz);
                }
            } else if (msgObj.contains("data") && msgObj["data"].isObject()) {
                // fallback: some publishers wrap pose under msg.data
                QJsonObject dataObj = msgObj["data"].toObject();
                if (dataObj.contains("pose") && dataObj["pose"].isObject()) {
                    QJsonObject p = dataObj["pose"].toObject();
                    QJsonObject poseObj;
                    if (p.contains("pose") && p["pose"].isObject()) poseObj = p["pose"].toObject();
                    else poseObj = p;
                    if (poseObj.contains("orientation") && poseObj["orientation"].isObject()){
                        QJsonObject ori = poseObj["orientation"].toObject();
                        double ow = ori.value("w").toDouble();
                        double ox = ori.value("x").toDouble();
                        double oy = ori.value("y").toDouble();
                        double oz = ori.value("z").toDouble();
                
                        emit orientationUpdated(ow, ox, oy, oz);
                    }
                }
            }
    }


}