#include "ros_process/battery.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"

// Map voltage to percent using a simple linear mapping as placeholder
// You can replace this with the more complex table from the python script if needed
static const double MIN_VOLTAGE = 10.0; // corresponding to 0%
static const double MAX_VOLTAGE = 12.46; // corresponding to 100%

BatteryMonitor::BatteryMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    battery_topic_name = loadTopicFromConfig("battery_topic");
    if(battery_topic_name.isEmpty()) {
        battery_topic_name = "/MediumSize/SensorHub/BatteryState"; // 话题解析失败，使用默认话题
    }
}
BatteryMonitor::~BatteryMonitor() {}

// 订阅电量话题
void BatteryMonitor::start()
{
    if (!m_worker) return;
    // send subscribe request for BatteryState
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = battery_topic_name;
    subscribeMsg["type"] = "sensor_msgs/BatteryState";
    QJsonDocument doc(subscribeMsg);
    QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}


void BatteryMonitor::onMessageReceived(const QString &message)
{
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (!doc.isObject()) return;
    QJsonObject obj = doc.object();

    // rosbridge publish message format: {op:"publish", topic:"...", msg:{...}}
    if (obj["op"].toString() == "publish") {
        QString topic = obj["topic"].toString();
        if (topic != battery_topic_name) return;

        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "收到电池 JSON (topic:" << topic << ") :" << msgJson;

        // Try to extract voltage field
        double voltage = 0.0;
        if (msgObj.contains("voltage")) {
            voltage = msgObj["voltage"].toDouble();
        } else {
            return;
        }

        int percent = voltageToPercent(voltage);
        emit batteryLevelChanged(percent);
    }
}

int BatteryMonitor::voltageToPercent(double voltage) const
{
    if (voltage <= MIN_VOLTAGE) return 0;
    if (voltage >= MAX_VOLTAGE) return 100;
    double t = (voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
    return static_cast<int>(t * 100.0 + 0.5);
}


