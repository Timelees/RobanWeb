#include "robanweb.h"
#include "connectdialog.h"  
#include <QAction>

robanweb::robanweb(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_robanweb)
    , webSocket(new QWebSocket)
    , reconnectTimer(new QTimer(this))
    , isReconnecting(false)
    , reconnectAttempts(0)
{
    ui->setupUi(this);
    // 设置菜单栏和状态栏
    settingMenuBar();
    settingStatusBar();
    // 禁用代理
    webSocket->setProxy(QNetworkProxy::NoProxy);

    // 为connectSetting菜单创建动作并连接信号槽
    QAction *connectAction = new QAction("连接设置", this);
    ui->connectSetting->addAction(connectAction);
    // 连接信号槽
    connect(connectAction, &QAction::triggered, this, &robanweb::onConnectSettingTriggered);

    // 连接 WebSocket 信号
    connect(webSocket, &QWebSocket::connected, this, &robanweb::onWebSocketConnected);
    connect(webSocket, &QWebSocket::disconnected, this, &robanweb::onWebSocketDisconnected);
    connect(webSocket, &QWebSocket::textMessageReceived, this, &robanweb::onWebSocketMessageReceived);
    connect(webSocket, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error), this, &robanweb::onWebSocketError);

    // 设置重连定时器
    connect(reconnectTimer, &QTimer::timeout, this, &robanweb::tryReconnect);
    reconnectTimer->setInterval(5000); // 每5秒尝试重连

    // 初始化状态标签
    updateStatusLabel("未连接");
}


robanweb::~robanweb()
{
    delete webSocket;
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
        qDebug() << "连接设置已确认";
    } else {
        // 用户点击了断开连接按钮
        if(webSocket){
            if(webSocket->state() == QAbstractSocket::ConnectedState){
                qDebug() << "关闭 WebSocket 连接...";
                webSocket->close();
                updateStatusLabel("未连接");
            }
            // webSocket->disconnect(this);
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
    webSocket->open(QUrl(url));
}

void robanweb::tryReconnect()
{
    if (!wsHost.isEmpty() && !wsPort.isEmpty() && isReconnecting) {
        QString url = QString("ws://%1:%2").arg(wsHost).arg(wsPort);
        qDebug() << "尝试重新连接到 WebSocket:" << url << " (尝试次数:" << reconnectAttempts + 1 << ")";
        reconnectAttempts++;
        webSocket->open(QUrl(url));
        updateStatusLabel(QString("正在重连... (尝试 %1/%2)").arg(reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
    }else if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        isReconnecting = false;
        reconnectTimer->stop();
        updateStatusLabel("连接失败：达到最大重试次数");
        qDebug() << "达到最大重试次数，停止重连";
    }
}

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
    webSocket->sendTextMessage(QString(doc.toJson()));
}

void robanweb::onWebSocketDisconnected()
{
    qDebug() << "WebSocket 连接断开";
    if (isReconnecting) {
        reconnectTimer->start();
        updateStatusLabel("连接断开，正在重连...");
    }
}

void robanweb::onWebSocketError(QAbstractSocket::SocketError error)
{
    qDebug() << "WebSocket 错误:" << webSocket->errorString();
    if (isReconnecting) {
        reconnectTimer->start();
        updateStatusLabel(QString("连接错误：%1，正在重连...").arg(webSocket->errorString()));
    }
}

void robanweb::onWebSocketMessageReceived(const QString &message)
{
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject obj = doc.object();
    if (obj["op"].toString() == "publish" && obj["topic"].toString() == "/MediumSize/BodyHub/ServoPositions") {
        QString data = obj["msg"].toObject()["data"].toString();
        qDebug() << "收到话题消息:" << data;
        // 在 UI 上显示消息，例如更新状态标签
        // ui->statusLabel->setText("状态: " + data);
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
    webSocket->close();
    event->accept();
}