#include "dialog/robotControlDialog.h"
#include "ui_robotControlDialog.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"
#include <QKeySequence>
#include <QDebug>

RobotControlDialog::RobotControlDialog(WebSocketWorker *webSocketWorker, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::RobotControlDialog)
    , m_worker(webSocketWorker)
{
    ui->setupUi(this);
    
    // 设置窗口属性
    setWindowTitle(tr("机器人控制"));
    setMinimumSize(600, 480);
    
    // 绑定按钮和快捷键
    bindSlots();
    
    // 按钮颜色已在UI中通过样式表设置
}

RobotControlDialog::~RobotControlDialog()
{
    delete ui;
}

// 按钮颜色和样式现在直接在UI文件的样式表中定义

void RobotControlDialog::bindSlots()
{
    // 连接启动控制和关闭控制按钮
    connect(ui->startControl_Button, &QPushButton::clicked, this, &RobotControlDialog::onStartControlButtonClicked);
    connect(ui->cancelControl_Button, &QPushButton::clicked, this, &RobotControlDialog::onCancelControlButtonClicked);

    // 设置所有控制按钮的快捷键和连接点击信号
    QList<QPushButton*> controlButtons = {
        ui->w_Button, ui->s_Button, ui->a_Button, ui->d_Button,
        ui->z_Button, ui->c_Button, ui->x_Button, ui->r_Button, ui->e_Button
    };
    
    QList<Qt::Key> keys = {
        Qt::Key_W, Qt::Key_S, Qt::Key_A, Qt::Key_D,
        Qt::Key_Z, Qt::Key_C, Qt::Key_X, Qt::Key_R, Qt::Key_E
    };
    
    for (int i = 0; i < controlButtons.size(); i++) {
        if (controlButtons[i]) {
            controlButtons[i]->setAutoRepeat(false);
            controlButtons[i]->setShortcut(QKeySequence(keys[i]));
            connect(controlButtons[i], &QPushButton::clicked, this, &RobotControlDialog::onControlButtonClicked);
        }
    }
}

QString RobotControlDialog::loadCmdFromConfig(const QString &key)
{
    // 使用工具函数从配置文件加载命令
    return ::loadCmdFromConfig(key);
}

// 启动控制
void RobotControlDialog::onStartControlButtonClicked()
{
    QString cmd = loadCmdFromConfig("start_control_bash");
    if(cmd.isEmpty()){
        cmd = "/home/lemon/move.sh";
    }
    if(!cmd.isEmpty()){
        qDebug() << "Run control script requested:" << cmd;
        // 通过webSocket发布启动脚本命令
        QJsonObject pub;
        pub["op"] = "publish";
        pub["topic"] = "/robot/exec_sh";
        pub["type"] = "std_msgs/String";
        QJsonObject msg;
        msg["data"] = cmd;
        pub["msg"] = msg;

        QJsonDocument doc(pub);
        QString jsonString = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        qDebug() << "Prepared exec command JSON:" << jsonString;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
        qDebug() << "Sent exec command to robot:" << cmd;
    }else{
        qDebug() << "Empty command, ignoring";
    }
}

// 关闭控制
void RobotControlDialog::onCancelControlButtonClicked()
{
    QJsonObject inner;
    inner["action"] = "stop";
    inner["which"] = "control";
    QJsonDocument innerDoc(inner);
    QString innerStr = QString::fromUtf8(innerDoc.toJson(QJsonDocument::Compact));

    // rosbridge publish message where msg.data is a string containing the JSON
    QJsonObject pub;
    pub["op"] = "publish";
    pub["topic"] = "/robot/exec_sh";
    pub["type"] = "std_msgs/String";
    QJsonObject msg;
    msg["data"] = innerStr;
    pub["msg"] = msg;
    QJsonDocument jsonDoc(pub);
    QString jsonString = QString::fromUtf8(jsonDoc.toJson(QJsonDocument::Compact));

    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
    qDebug() << "Sent stop control command to robot:" << jsonString;
}

// 接收控制按钮点击事件
void RobotControlDialog::onControlButtonClicked()
{
    QObject *s = sender();
    if(!s){
        qDebug() << "onControlButtonClicked: no sender";
        return;
    }
    QString key_cmd;        // 按下的按键对应的命令
    if(s == ui->w_Button){
        key_cmd = QStringLiteral("w");
    }else if (s == ui->s_Button){
        key_cmd = QStringLiteral("s");
    }else if(s == ui->a_Button){
        key_cmd = QStringLiteral("a");
    }else if(s == ui->d_Button){
        key_cmd = QStringLiteral("d");
    }else if(s == ui->z_Button){
        key_cmd = QStringLiteral("z");
    }else if(s == ui->c_Button){
        key_cmd = QStringLiteral("c");
    }else if(s == ui->x_Button){
        key_cmd = QStringLiteral("x");
    }else if(s == ui->r_Button){
        key_cmd = QStringLiteral("r");
    }else if(s == ui->e_Button){
        key_cmd = QStringLiteral("e");
    }else{
        qDebug() << "Unknown control source";
        return;
    }
    // 构建要发送的单字符控制命令（字符串）并通过 rosbridge 发布到 /robot/slam_cmd
    QString cmdStr = key_cmd; // single-letter command
    QJsonObject pub;
    pub["op"] = "publish";
    pub["topic"] = "/robot/slam_cmd";
    pub["type"] = "std_msgs/String";
    QJsonObject msg;
    msg["data"] = cmdStr;
    pub["msg"] = msg;

    QJsonDocument doc(pub);
    QString jsonString = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
    qDebug() << "Sent remote control command:" << cmdStr;
}