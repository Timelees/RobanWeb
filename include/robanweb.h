#pragma once
#include "ui_robanweb.h"
#include <QMainWindow>
#include <QThread>
#include <QCloseEvent>
#include <QTimer>
#include <QAction>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>
#include <QLabel>
#include <QJsonArray>
#include <QProgressBar>

#include "socket_process/websocketworker.h"
#include "ros_process/battery.h"

class robanweb : public QMainWindow {
    Q_OBJECT
    
public:
    robanweb(QWidget* parent = nullptr);
    ~robanweb();
    // 通过客户端发布话题
    void publishToTopic(const QString &topic, const QString &type, const QJsonObject &msg);
protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void onConnectSettingTriggered();       // 连接设置 槽函数
    void onWebSocketConnected();            // 连接webSocket
    void onWebSocketDisconnected();
    void onWebSocketError(const QString &error);
    void onWebSocketMessageReceived(const QString &message);
    void establishWebSocketConnection(const QString &url);
    void tryReconnect();

private:
    void settingMenuBar();          
    void settingStatusBar();
    void updateStatusLabel(const QString &status);  // 更新连接显示标签

private:
    Ui_robanweb* ui;
    WebSocketWorker *webSocketWorker;
    QThread *webSocketThread;
    QTimer *reconnectTimer;
    QString wsHost;
    QString wsPort;
    bool isReconnecting;
    int reconnectAttempts;
    static const int MAX_RECONNECT_ATTEMPTS = 10;
    QLabel *connect_label;                      // 连接状态标签
    QProgressBar *batteryProgressBar;           // 电量进度条
    BatteryMonitor *batteryMonitor = nullptr;   // 电量获取对象
};