#include "ros_process/battery.h"
#include "socket_process/websocketworker.h"
#include <QMetaObject>

// Map voltage to percent using a simple linear mapping as placeholder
// You can replace this with the more complex table from the python script if needed
static const double MIN_VOLTAGE = 10.0; // corresponding to 0%
static const double MAX_VOLTAGE = 12.46; // corresponding to 100%

BatteryMonitor::BatteryMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
}

BatteryMonitor::~BatteryMonitor() {}

void BatteryMonitor::start()
{
    if (!m_worker) return;
    // send subscribe request for BatteryState
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = "/MediumSize/SensorHub/BatteryState";
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
    if (obj["op"].toString() == "publish" && obj["topic"].toString() == "/MediumSize/SensorHub/BatteryState") {
        QJsonObject msgObj = obj["msg"].toObject();
        // Try to extract voltage field
        double voltage = 0.0;
        if (msgObj.contains("voltage")) {
            voltage = msgObj["voltage"].toDouble();
        } else {
            // maybe nested or different naming; bail if not found
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
