#include "ros_process/imu.h"
#include "socket_process/websocketworker.h"

ImuMonitor::ImuMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
}

ImuMonitor::~ImuMonitor() {}

// 订阅IMU话题
void ImuMonitor::start(){
    if(!m_worker) return;
    // send subscribe request for IMU
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = "/MediumSize/SensorHub/Imu";
    subscribeMsg["type"] = "sensor_msgs/Imu";
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}

void ImuMonitor::onMessageReceived(const QString &message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if(!doc.isObject()) return;
    QJsonObject obj = doc.object();

    // rosbridge publish message format: {op:"publish", topic:"...", msg:{...}}
    if (obj["op"].toString() == "publish") {
        QString topic = obj["topic"].toString();
        if (topic != "/MediumSize/SensorHub/Imu") return;

        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "收到 IMU msg JSON (topic:" << topic << ") :" << msgJson;

        // orientation
        if (msgObj.contains("orientation") && msgObj["orientation"].isObject()) {
            QJsonObject ori = msgObj["orientation"].toObject();
            double w = ori.value("w").toDouble();
            double x = ori.value("x").toDouble();
            double y = ori.value("y").toDouble();
            double z = ori.value("z").toDouble();
            emit orientationUpdated(w, x, y, z);
        }

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
}