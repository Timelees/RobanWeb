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
#include "ros_process/imu.h"
#include "ros_process/cameraImage.h"

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
    // void onWebSocketMessageReceived(const QString &message);
    void establishWebSocketConnection(const QString &url);
    void tryReconnect();

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    void settingMenuBar();          
    void settingStatusBar();
    void updateStatusLabel(const QString &status);  // 更新连接显示标签
    void bindSlots();                               // 绑定槽函数
    void init();
    void startSubscriptions();                     // 启动话题订阅

    private:
    Ui_robanweb* ui;
    WebSocketWorker *webSocketWorker;
    QThread *webSocketThread;
    QThread *imageThread;
    QTimer *reconnectTimer;
    QString wsHost;
    QString wsPort;
    // 菜单栏设置
    QAction *connectAction;
    
    bool isReconnecting;
    int reconnectAttempts;
    static const int MAX_RECONNECT_ATTEMPTS = 10;
    QLabel *connect_label;                      // 连接状态标签
    QProgressBar *batteryProgressBar;           // 电量进度条
    BatteryMonitor *batteryMonitor = nullptr;   // 电量获取对象
    ImuMonitor *imuMonitor = nullptr;           // IMU获取对象
    // Camera image display label overlaying the QOpenGLWidget
    QLabel *imageRawLabel = nullptr;
    CameraImageMonitor *cameraImageMonitor = nullptr;
    QTimer *imagePullTimer = nullptr; // timer to pull latest frame from camera monitor

};