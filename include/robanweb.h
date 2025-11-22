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
#include <QPropertyAnimation>
// layout helper for replacing placeholder widget
#include <QLayout>
#include <QBoxLayout>


#include "socket_process/websocketworker.h"
#include "ros_process/battery.h"
#include "ros_process/imu.h"
#include "ros_process/cameraImage.h"
#include "model_display/modelDisplay.h"
#include "manager/taskmanager.h"
#include "model_display/robotManager.h"
#include "model_display/sceneManager.h"
#include "ros_process/servoPositions.h"
#include "ros_process/slamPose.h"
#include "dialog/connectdialog.h"
#include "dialog/slamDialog.h"
#include "dialog/robotControlDialog.h"
#include "util/load_param.hpp"
#include "ros_process/gaitCommand.h"
#include "manager/multiRobotManager.h"


class robanweb : public QMainWindow {
    Q_OBJECT
    
public:
    robanweb(QWidget* parent = nullptr);
    ~robanweb();


protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void onConnectSettingButtonClicked();       // 连接设置 槽函数
    void onSlamControlButtonClicked();           // SLAM控制按钮 槽函数
    void onVoiceControlButtonClicked();         // 语音控制按钮 槽函数
    void onrobanControlButtonclicked();         // 机器人控制按钮 槽函数
    void onTaskExecuted(const QString& scriptPath); // 任务执行 槽函数
    void onTaskStopped(const QString& scriptPath);  // 任务停止 槽函数
    void onAddTask(Task* task);           // 添加任务 槽函数

    // 泛用的机器人事件槽，接收来自 MultiRobotManager 的 robotId
    void onRobotConnected(const QString &robotId);
    void onRobotDisconnected(const QString &robotId);
    void onRobotError(const QString &robotId, const QString &error);


protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:        
    void settingStatusBar();
    void updateStatusLabel(const QString &status);  // 更新连接显示标签
    void bindSlots();                               // 绑定槽函数
    void init();
    // helpers for unified connect/reconnect handling
    void startConnectFor(const QString &robotId, const QString &url);
    void doReconnectFor(const QString &robotId);
    void toggleRobot2Info(bool show);
    static QString resolveAssetPath(const QString &relPath);


private:
    Ui_robanweb* ui;
    WebSocketWorker *webSocketWorker;
    QThread *webSocketThread;       // webSocket 线程
    WebSocketWorker *webSocketWorker2; // 第二台机器人 worker
    QThread *webSocketThread2;          // 第二台机器人线程
    // per-robot websocket worker threads are created as needed
    QThread *imageThread;           // legacy placeholder (unused when MultiRobotManager used)
    
    QThread *sceneManagerThread;         // 场景处理线程
    QThread *robotManagerThread;   // 机器人管理线程


    QTimer *reconnectTimer;
    QTimer *reconnectTimer2;
    QString wsHost;
    QString wsPort;
    QString wsHost2;
    QString wsPort2;

    
    bool isReconnecting;
    int reconnectAttempts;
    bool isReconnecting2;
    int reconnectAttempts2;
    static const int MAX_RECONNECT_ATTEMPTS = 10;
    QLabel *connect_label1;                      // robot1连接状态标签
    QLabel *connect_label2;                      // robot2连接状态标签
    QProgressBar *batteryProgressBar;           // 电量进度条
    BatteryMonitor *batteryMonitor = nullptr;   // 电量获取对象
    QTimer *imagePullTimer = nullptr;           // 定时器，用于从相机监视器中获取最新帧

    ModelDisplay *modelDisplay = nullptr;        // 3D模型显示窗口
    RobotManager *robotManager = nullptr;        // 机器人管理器
    SceneManager *sceneManager = nullptr;        // 场景管理器

    // monitors (will be provided by MultiRobotManager instances)
    ImuMonitor *imuMonitor = nullptr;
    CameraImageMonitor *cameraImageMonitor = nullptr;
    ServoPositionsMonitor *servoPositionsMonitor = nullptr;
    PoseMonitor *poseMonitor = nullptr;
    GaitCommandMonitor *gaitCommandMonitor = nullptr;

    // legacy per-monitor threads (kept as placeholders; manager may own these)
    QThread *servoThread = nullptr;
    QThread *poseThread = nullptr;
    QThread *gaitCommandThread = nullptr;

    TaskManager *taskManager = nullptr;        // 任务管理器
    // per-robot managers (each manages worker + monitors for that robot)
    MultiRobotManager *multiRobot_mgr1 = nullptr;
    MultiRobotManager *multiRobot_mgr2 = nullptr;

    slamDialog *m_slamDialog = nullptr;        // SLAM对话框
    QPropertyAnimation *robot2Animation = nullptr;
};