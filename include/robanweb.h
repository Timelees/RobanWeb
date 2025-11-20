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
// layout helper for replacing placeholder widget
#include <QLayout>
#include <QBoxLayout>


#include "socket_process/websocketworker.h"
#include "ros_process/battery.h"
#include "ros_process/imu.h"
#include "ros_process/cameraImage.h"
#include "model_display/modelDisplay.h"
#include "task_manager/taskmanager.h"
#include "model_display/robotManager.h"
#include "model_display/sceneManager.h"
#include "ros_process/servoPositions.h"
#include "ros_process/slamPose.h"
#include "dialog/connectdialog.h"
#include "dialog/slamDialog.h"
#include "dialog/robotControlDialog.h"
#include "util/load_param.hpp"
#include "ros_process/gaitCommand.h"


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

    // webSocket 相关槽函数
    void onWebSocketConnected();            // 连接webSocket
    void onWebSocketDisconnected();
    void onWebSocketError(const QString &error);
    void establishWebSocketConnection(const QString &url);
    void tryReconnect();

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:        
    void settingStatusBar();
    void updateStatusLabel(const QString &status);  // 更新连接显示标签
    void bindSlots();                               // 绑定槽函数
    void init();
    void startSubscriptions();                     // 启动话题订阅

private:
    Ui_robanweb* ui;
    WebSocketWorker *webSocketWorker;
    QThread *webSocketThread;       // webSocket 线程
    QThread *imageThread;           // 图像处理线程
    QThread *servoThread;          // 伺服处理线程
    QThread *poseThread;          // 位置处理线程
    QThread *gaitCommandThread;   // 步态命令处理线程
    
    QThread *sceneManagerThread;         // 场景处理线程
    QThread *robotManagerThread;   // 机器人管理线程


    QTimer *reconnectTimer;
    QString wsHost;
    QString wsPort;

    
    bool isReconnecting;
    int reconnectAttempts;
    static const int MAX_RECONNECT_ATTEMPTS = 10;
    QLabel *connect_label;                      // 连接状态标签
    QProgressBar *batteryProgressBar;           // 电量进度条
    BatteryMonitor *batteryMonitor = nullptr;   // 电量获取对象
    ImuMonitor *imuMonitor = nullptr;           // IMU获取对象
    ServoPositionsMonitor *servoPositionsMonitor = nullptr; // 伺服位置获取对象
    PoseMonitor *poseMonitor = nullptr;         // 位置获取对象
    GaitCommandMonitor *gaitCommandMonitor = nullptr;         // 行走步态命令对象

    CameraImageMonitor *cameraImageMonitor = nullptr;
    QTimer *imagePullTimer = nullptr;           // 定时器，用于从相机监视器中获取最新帧

    ModelDisplay *modelDisplay = nullptr;        // 3D模型显示窗口
    RobotManager *robotManager = nullptr;        // 机器人管理器
    SceneManager *sceneManager = nullptr;        // 场景管理器

    TaskManager *taskManager = nullptr;        // 任务管理器

    slamDialog *m_slamDialog = nullptr;        // SLAM对话框
};