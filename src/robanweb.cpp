#include "robanweb.h"
#include "dialog/connectdialog.h"
#include "socket_process/websocketworker.h"


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
    // 设置菜单栏和状态栏
    settingMenuBar();
    settingStatusBar();

    // 为connectSetting菜单创建动作并连接信号槽
    QAction *connectAction = new QAction("连接设置", this);
    ui->connectSetting->addAction(connectAction);
    // 连接信号槽
    connect(connectAction, &QAction::triggered, this, &robanweb::onConnectSettingTriggered);

    // 创建 worker 和线程，把 WebSocket 操作放到子线程
    webSocketWorker = new WebSocketWorker();
    webSocketThread = new QThread(this);
    webSocketWorker->moveToThread(webSocketThread);

    // 当线程启动时可做初始化
    connect(webSocketThread, &QThread::finished, webSocketWorker, &QObject::deleteLater);

    // 连接 worker 的信号到主线程槽
    connect(webSocketWorker, &WebSocketWorker::connected, this, &robanweb::onWebSocketConnected);
    connect(webSocketWorker, &WebSocketWorker::disconnected, this, &robanweb::onWebSocketDisconnected);
    connect(webSocketWorker, &WebSocketWorker::messageReceived, this, &robanweb::onWebSocketMessageReceived);
    connect(webSocketWorker, &WebSocketWorker::errorOccurred, this, &robanweb::onWebSocketError);

    webSocketThread->start();

    // 保留 UI 侧的重连策略触发器
    connect(reconnectTimer, &QTimer::timeout, this, &robanweb::tryReconnect);
    reconnectTimer->setInterval(5000); // 每5秒尝试重连

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
    delete reconnectTimer;
    delete ui; 
}

// 设置菜单栏组件
void robanweb::settingMenuBar(){
    return;
}

// 设置状态栏组件
void robanweb::settingStatusBar(){
    connect_label = new QLabel("未连接");
    ui->statusbar->addWidget(connect_label);
}


// 菜单栏连接设置触发
void robanweb::onConnectSettingTriggered(){
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

    // 订阅 ROS 话题（例如 /robot_status）
    QJsonObject subscribeMsg;
    subscribeMsg["op"] = "subscribe";
    subscribeMsg["topic"] = "/MediumSize/BodyHub/ServoPositions";    
    subscribeMsg["type"] = "bodyhub/ServoPositionAngle"; 
    QJsonDocument doc(subscribeMsg);
    QString payload = QString(doc.toJson());
    // 通过 worker 发送订阅消息
    QMetaObject::invokeMethod(webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
}

void robanweb::onWebSocketDisconnected()
{
    qDebug() << "WebSocket 连接断开";
    if (isReconnecting) {
        reconnectTimer->start();
        updateStatusLabel("连接断开，正在重连...");
    }
}

void robanweb::onWebSocketError(const QString &error)
{
    qDebug() << "WebSocket 错误:" << error;
    if (isReconnecting) {
        reconnectTimer->start();
        updateStatusLabel(QString("连接错误：%1，正在重连...").arg(error));
    }
}

void robanweb::onWebSocketMessageReceived(const QString &message)
{
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject obj = doc.object();

    if (obj.contains("msg") && obj["msg"].isObject()) {
        QJsonObject msgObj = obj["msg"].toObject();
        QJsonDocument msgDoc(msgObj);
        QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
        // qDebug() << "收到 msg JSON:" << msgJson;
    }

    if (obj["op"].toString() == "publish"
        && obj["topic"].toString() == "/MediumSize/BodyHub/ServoPositions") {

        QJsonObject msgObj = obj["msg"].toObject();
        if (msgObj.contains("angle") && msgObj["angle"].isArray()) {
            QJsonArray angles = msgObj["angle"].toArray();
            QStringList parts;
            for (const QJsonValue &v : angles) {
                parts << QString::number(v.toDouble());
            }
            QString data = parts.join(", ");
            qDebug() << "收到话题 angle:" << data;
        } else {
            qDebug() << "未找到 angle 字段或类型不匹配，msg:" << QJsonDocument(msgObj).toJson(QJsonDocument::Compact);
        }
    }


}

// void robanweb::publishToTopic(const QString &topic, const QString &type, const QJsonObject &msg)
// {
//     QJsonObject publishMsg;
//     publishMsg["op"] = "publish";
//     publishMsg["topic"] = topic;
//     publishMsg["type"] = type;
//     publishMsg["msg"] = msg;

//     QJsonDocument doc(publishMsg);
//     webSocket->sendTextMessage(QString(doc.toJson()));
// }

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
    event->accept();
}