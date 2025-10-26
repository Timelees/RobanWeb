#include "robanweb.h"
#include "dialog/connectdialog.h"
#include "dialog/shDialog.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"
// layout helper for replacing placeholder widget
#include <QLayout>
#include <QBoxLayout>


robanweb::robanweb(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_robanweb)
    , webSocketWorker(nullptr)
    , webSocketThread(nullptr)
    , reconnectTimer(new QTimer(this))
    , isReconnecting(false)
    , reconnectAttempts(0)
{
    ui->setupUi(this);
    // 设置状态栏
    settingStatusBar();

    init(); 
    bindSlots();        // 绑定相关槽函数
    // 初始化状态标签
    updateStatusLabel("未连接");
    qDebug() << "robanweb run in thread:" << QThread::currentThread();
}


robanweb::~robanweb()
{
    // 在析构中，确保线程已停止并清理
    if (webSocketThread) {
        QMetaObject::invokeMethod(webSocketWorker, "closeConnection", Qt::QueuedConnection);
        webSocketThread->quit();
        webSocketThread->wait();
        // webSocketWorker 会在 thread finished 时 deleteLater 被调用
        delete webSocketThread;
        webSocketThread = nullptr;
        webSocketWorker = nullptr;
    }
    if(imageThread){
        imageThread->quit();
        imageThread->wait();
        delete imageThread;
        imageThread = nullptr;
        cameraImageMonitor = nullptr;
    }
    delete reconnectTimer;
    // delete imagePullTimer;
    delete ui; 
}
// 初始化
void robanweb::init(){
    // 创建 worker 和线程，把 WebSocket 操作放到子线程
    webSocketWorker = new WebSocketWorker();
    webSocketThread = new QThread(this);
    webSocketWorker->moveToThread(webSocketThread);
    webSocketThread->start();
    
    reconnectTimer->setInterval(5000); // 每5秒尝试重连
    // 图像拉取定时器（UI 拉取最新缓存帧，避免信号队列积压）
    imagePullTimer = new QTimer(this);
    imagePullTimer->setInterval(50); // 默认 20 FPS


    // ros话题接收对象
    batteryMonitor = new BatteryMonitor(webSocketWorker, this);         // 电池数据
    imuMonitor = new ImuMonitor(webSocketWorker, this);                 // IMU数据
   
    // 创建 cameraImageMonitor 时不指定父对象，并将其移动到 imageThread进行处理,订阅压缩图像
    QString camera_topic = loadTopicFromConfig("cameraCompressed_topic");
    cameraImageMonitor = new CameraImageMonitor(webSocketWorker, nullptr, camera_topic); // 图像数据（无父以便移动线程）
    imageThread = new QThread(this);
    cameraImageMonitor->moveToThread(imageThread);
    imageThread->start();
    connect(imageThread, &QThread::finished, cameraImageMonitor, &QObject::deleteLater);
    // Ensure image display label does not resize itself to the pixmap
    if (ui->imageRawDisplay) {
        // Let the worker scale to the desired target size and avoid QLabel auto-scaling to prevent blur
        ui->imageRawDisplay->setScaledContents(false);
        ui->imageRawDisplay->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    }
    // 设置目标显示尺寸和最大帧率（在 worker 线程中设置）
    if (ui->imageRawDisplay) {
        QSize target = ui->imageRawDisplay->size();
        QMetaObject::invokeMethod(cameraImageMonitor, "setTargetSize", Qt::QueuedConnection, Q_ARG(QSize, target));
    }
    // 将帧率限制到 20 FPS 默认以减少延迟和 CPU 负载
    QMetaObject::invokeMethod(cameraImageMonitor, "setMaxFps", Qt::QueuedConnection, Q_ARG(int, 20));
    // 安装事件过滤器以在 imageRawDisplay 尺寸变更时更新目标尺寸
    if (ui->imageRawDisplay) {
        ui->imageRawDisplay->installEventFilter(this);
    }
 
    // connect imagePullTimer once to cameraImageMonitor requestFrame (use queued connection)
    if (imagePullTimer && cameraImageMonitor) {
        connect(imagePullTimer, &QTimer::timeout, this, [this]() {
            QMetaObject::invokeMethod(cameraImageMonitor, "requestFrame", Qt::QueuedConnection);
        });
    }

    // 3D模型显示初始化
    QString modelPath = "..\\assets\\Roban.fbx"; // 默认模型路径
    if (ui->modelDisplay) {
        QWidget *placeholder = ui->modelDisplay; // UI 中的占位 widget
        QWidget *parent = placeholder->parentWidget();
        QRect geom = placeholder->geometry();
        QLayout *parentLayout = parent ? parent->layout() : nullptr;

        // 在同一父容器中创建 ModelDisplay，并在父 layout 中替换占位 widget（如果存在）
        modelDisplay = new ModelDisplay(modelPath, parent ? parent : placeholder);
        modelDisplay->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        if (parentLayout) {
            // 在 layout 中找到占位位置并替换
            int idx = -1;
            for (int i = 0; i < parentLayout->count(); ++i) {
                QLayoutItem *it = parentLayout->itemAt(i);
                if (it && it->widget() == placeholder) {
                    idx = i;
                    break;
                }
            }
            if (idx != -1) {
                parentLayout->removeWidget(placeholder);
                placeholder->hide();
                // If the parent layout is a box layout we can insert at index, otherwise append
                if (QBoxLayout *box = qobject_cast<QBoxLayout *>(parentLayout)) {
                    box->insertWidget(idx, modelDisplay);
                } else {
                    // fallback: add to layout (will appear at end)
                    parentLayout->addWidget(modelDisplay);
                }
            } else {
                // 找不到占位 index，回退为手动定位
                modelDisplay->setGeometry(geom);
            }
        } else {
            // 父容器没有 layout，直接按占位 geometry 放置
            modelDisplay->setGeometry(geom);
        }

        modelDisplay->show();
    }
    
}


// 设置状态栏组件
void robanweb::settingStatusBar(){
    // 连接状态
    connect_label = new QLabel("未连接");
    connect_label->setMinimumWidth(100);
    connect_label->setFont(QFont("Arial", 10, QFont::Bold));
    connect_label->setAlignment(Qt::AlignVCenter | Qt::AlignVCenter);
    ui->statusbar->addWidget(connect_label);
    // 电量显示
    batteryProgressBar = new QProgressBar();
    batteryProgressBar->setRange(0, 100);
    batteryProgressBar->setValue(0); // 初始值
    batteryProgressBar->setFixedWidth(100);
    ui->statusbar->addPermanentWidget(batteryProgressBar);
}

void robanweb::bindSlots(){
    // 连接信号槽
    connect(ui->connect_Button, &QPushButton::clicked, this, &robanweb::onConnectSettingButtonClicked);
    // SLAM和控制按钮槽
    connect(ui->SLAM_Control_Button, &QPushButton::clicked, this, &robanweb::onSlamControlButtonClicked);
    // 语音控制按钮槽
    connect(ui->voice_Button, &QPushButton::clicked, this, &robanweb::onVoiceControlButtonClicked);

    // 当线程启动时可做初始化
    connect(webSocketThread, &QThread::finished, webSocketWorker, &QObject::deleteLater);

    // 连接 worker 的信号到主线程槽
    connect(webSocketWorker, &WebSocketWorker::connected, this, &robanweb::onWebSocketConnected);
    connect(webSocketWorker, &WebSocketWorker::disconnected, this, &robanweb::onWebSocketDisconnected);
    // connect(webSocketWorker, &WebSocketWorker::messageReceived, this, &robanweb::onWebSocketMessageReceived);
    connect(webSocketWorker, &WebSocketWorker::errorOccurred, this, &robanweb::onWebSocketError);

    // 保留 UI 侧的重连策略触发器
    connect(reconnectTimer, &QTimer::timeout, this, &robanweb::tryReconnect);

    // 从ros话题获取电量信息
    connect(webSocketWorker, &WebSocketWorker::messageReceived, batteryMonitor, &BatteryMonitor::onMessageReceived, Qt::QueuedConnection);
    connect(batteryMonitor, &BatteryMonitor::batteryLevelChanged, this, [this](int pct){
        if (batteryProgressBar) batteryProgressBar->setValue(pct);
    }, Qt::QueuedConnection);
    
    // 从ros话题获取IMU信息
    connect(webSocketWorker, &WebSocketWorker::messageReceived, imuMonitor, &ImuMonitor::onMessageReceived, Qt::QueuedConnection);
    
    // 从ros话题获取图像信息
    connect(webSocketWorker, &WebSocketWorker::messageReceived, cameraImageMonitor, &CameraImageMonitor::onMessageReceived, Qt::QueuedConnection);
    connect(cameraImageMonitor, &CameraImageMonitor::imageReceived, this, [this](const QImage &img){
        if (ui->imageRawDisplay) {
            // Worker should already provide an image scaled to the target size; set directly to avoid resampling blur
            ui->imageRawDisplay->setPixmap(QPixmap::fromImage(img));
        }
    }, Qt::QueuedConnection);


    // 更新IMU数据显示
    connect(imuMonitor, &ImuMonitor::orientationUpdated, this, [this](double w, double x, double y, double z){
        if (ui) {
            // Update labels; wrapped in invokeMethod to be safe if called from other threads
            QMetaObject::invokeMethod(ui->ori_w, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(w, 'f', 2)));
            QMetaObject::invokeMethod(ui->ori_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(x, 'f', 2)));
            QMetaObject::invokeMethod(ui->ori_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(y, 'f', 2)));
            QMetaObject::invokeMethod(ui->ori_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(z, 'f', 2)));
        }
    }, Qt::QueuedConnection);
    connect(imuMonitor, &ImuMonitor::angularVelocityUpdated, this, [this](double x, double y, double z){
        if (ui) {
            ui->ang_x->setText(QString::number(x, 'f', 2));
            ui->ang_y->setText(QString::number(y, 'f', 2));
            ui->ang_z->setText(QString::number(z, 'f', 2));
        }
    }, Qt::QueuedConnection);
    connect(imuMonitor, &ImuMonitor::linearAccelerationUpdated, this, [this](double x, double y, double z){
        if (ui) {
            ui->lin_x->setText(QString::number(x, 'f', 2));
            ui->lin_y->setText(QString::number(y, 'f', 2));
            ui->lin_z->setText(QString::number(z, 'f', 2));
        }
    }, Qt::QueuedConnection);
    

}
// SLAM控制按钮槽函数
void robanweb::onSlamControlButtonClicked()
{
    ShDialog dialog(webSocketWorker, this);
    // connect dialog signal to main slot
    // connect(&dialog, &ShDialog::runScriptRequested, this, &robanweb::onRunScriptRequested, Qt::QueuedConnection);
    if(dialog.exec() == QDialog::Accepted){
        qDebug() << "脚本功能对话框开启";
    }
}


// 连接设置槽函数
void robanweb::onConnectSettingButtonClicked(){
    ConnectDialog dialog(this);
    connect(&dialog, &ConnectDialog::connectRequested, this, &robanweb::establishWebSocketConnection);
    if (dialog.exec() == QDialog::Accepted) {
        // 用户点击了连接按钮
        qDebug() << "连接确认";
    } else {
        // 用户点击了断开连接按钮
        if (webSocketWorker) {
            qDebug() << "请求 worker 关闭 WebSocket 连接...";
            QMetaObject::invokeMethod(webSocketWorker, "closeConnection", Qt::QueuedConnection);
            updateStatusLabel("未连接");
        }
    }
}

// 语音控制按钮槽函数
void robanweb::onVoiceControlButtonClicked(){
    QString cmd = loadCmdFromConfig("voiceControlScript");
    if(cmd.isEmpty()){
        cmd = "/home/lemon/largeModel.sh";
    }
    if (!cmd.isEmpty()) {
        QJsonObject pub;
        pub["op"] = "publish";
        pub["topic"] = "/robot/exec_sh";
        pub["type"] = "std_msgs/String";
        QJsonObject msg;
        msg["data"] = cmd;
        pub["msg"] = msg;
        QJsonDocument doc(pub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // 通过 worker 发送发布消息
        QMetaObject::invokeMethod(webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
        qDebug() << "Sent exec command to robot:" << cmd;
    }else{
        qDebug() << "Voice control script command is empty in config.";
    }
}

// 建立webSocket连接
void robanweb::establishWebSocketConnection(const QString &url)
{
    wsHost = url.split(":").value(1);
    wsPort = url.split(":").last();
    qDebug() << "尝试连接到 WebSocket:" << url;
    isReconnecting = true;
    reconnectAttempts = 0;
    updateStatusLabel("正在连接...");
    // 通过 worker 启动连接（跨线程异步调用startConnect方法）
    QMetaObject::invokeMethod(webSocketWorker, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
}

// 重连
void robanweb::tryReconnect()
{
    if (!wsHost.isEmpty() && !wsPort.isEmpty() && isReconnecting) {
        QString url = QString("ws://%1:%2").arg(wsHost).arg(wsPort);
        qDebug() << "尝试重新连接到 WebSocket:" << url << " (尝试次数:" << reconnectAttempts + 1 << ")";
        reconnectAttempts++;
        QMetaObject::invokeMethod(webSocketWorker, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
        updateStatusLabel(QString("正在重连... (尝试 %1/%2)").arg(reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
    }else if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        isReconnecting = false;
        reconnectTimer->stop();
        updateStatusLabel("连接失败：达到最大重试次数");
        qDebug() << "达到最大重试次数，停止重连";
    }
}

// WebSocket 连接成功处理
void robanweb::onWebSocketConnected()
{
    qDebug() << "WebSocket 连接成功";
    isReconnecting = false;
    reconnectTimer->stop();
    reconnectAttempts = 0;
    updateStatusLabel("已连接");
    // 连接成功后启动话题订阅
    startSubscriptions();  
    // 启动 image pull timer (already connected in init)
    if (imagePullTimer && cameraImageMonitor) {
        imagePullTimer->start();
    }
}

void robanweb::startSubscriptions(){
    // // 订阅 ROS 话题（例如 /robot_status）
    // QJsonObject subscribeMsg;
    // subscribeMsg["op"] = "subscribe";
    // subscribeMsg["topic"] = "/MediumSize/BodyHub/ServoPositions";    
    // subscribeMsg["type"] = "bodyhub/ServoPositionAngle"; 
    // QJsonDocument doc(subscribeMsg);
    // QString payload = QString(doc.toJson());
    // // 通过 worker 发送订阅消息
    // QMetaObject::invokeMethod(webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));


    // 启动电量订阅
    if (batteryMonitor) {
        QMetaObject::invokeMethod(batteryMonitor, "start", Qt::QueuedConnection);
    }
    // 启动IMU订阅
    if(imuMonitor){
        QMetaObject::invokeMethod(imuMonitor, "start", Qt::QueuedConnection);
    }

    // 启动图像订阅
    if(cameraImageMonitor){
        QMetaObject::invokeMethod(cameraImageMonitor, "start", Qt::QueuedConnection);
    }
}

void robanweb::onWebSocketDisconnected()
{
    qDebug() << "WebSocket 连接断开";
    if (isReconnecting) {
        reconnectTimer->start();
        updateStatusLabel("连接断开，正在重连...");
    }
    if (imagePullTimer) imagePullTimer->stop();
}

void robanweb::onWebSocketError(const QString &error)
{
    qDebug() << "WebSocket 错误:" << error;
    if (isReconnecting) {
        reconnectTimer->start();
        updateStatusLabel(QString("连接错误：%1，正在重连...").arg(error));
    }
}


void robanweb::updateStatusLabel(const QString &status)
{
    if (connect_label) {
        connect_label->setText(status);
    }
}

void robanweb::closeEvent(QCloseEvent *event)
{
    isReconnecting = false;
    reconnectTimer->stop();
    if (webSocketWorker) {
        QMetaObject::invokeMethod(webSocketWorker, "closeConnection", Qt::QueuedConnection);
        webSocketThread->quit();
        webSocketThread->wait();
        delete webSocketThread;
        webSocketThread = nullptr;
        webSocketWorker = nullptr; // worker deleted when thread finishes (connected earlier)
    }
    if (imageThread){
        imageThread->quit();
        imageThread->wait();
        delete imageThread;
        imageThread = nullptr;
        cameraImageMonitor = nullptr;
    }
    if (imagePullTimer) {
        imagePullTimer->stop();
        delete imagePullTimer;
        imagePullTimer = nullptr;
    }
    event->accept();
}

bool robanweb::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->imageRawDisplay && event->type() == QEvent::Resize) {
        QSize newSize = ui->imageRawDisplay->size();
        if (cameraImageMonitor) {
            QMetaObject::invokeMethod(cameraImageMonitor, "setTargetSize", Qt::QueuedConnection, Q_ARG(QSize, newSize));
        }
    }
    return QMainWindow::eventFilter(watched, event);
}