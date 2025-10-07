#ifndef BATTERY_H
#define BATTERY_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

class WebSocketWorker;

class BatteryMonitor : public QObject {
    Q_OBJECT
public:
    explicit BatteryMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~BatteryMonitor();

public slots:
    void start(); // send subscribe request via worker
    void onMessageReceived(const QString &message);

signals:
    void batteryLevelChanged(int percent);

private:
    int voltageToPercent(double voltage) const;
    WebSocketWorker *m_worker;
};

#endif // BATTERY_H
