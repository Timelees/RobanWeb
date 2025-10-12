#include "dialog/shDialog.h"
#include "ui_shDialog.h"
#include "socket_process/websocketworker.h"

ShDialog::ShDialog(WebSocketWorker *webSocketWorker, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ShDialog)
    , m_worker(webSocketWorker)
{
    ui->setupUi(this);
    connect(ui->startSlam_Button, &QPushButton::clicked, this, &ShDialog::onRunButtonClicked);
    connect(ui->locationSlam_Button, &QPushButton::clicked, this, &ShDialog::onRunButtonClicked);
    connect(ui->closeSlam_Button, &QPushButton::clicked, this, &ShDialog::onCloseButtonClicked);
    connect(ui->closeLocationSlam_Button, &QPushButton::clicked, this, &ShDialog::onCloseButtonClicked);
}

ShDialog::~ShDialog()
{
    delete ui;
}

void ShDialog::onRunButtonClicked()
{
    QObject *s = sender();
    if (!s) return;
    QString cmd;
    if (s == ui->startSlam_Button) {
        cmd = ui->startSlamBash_Edit->text();
    } else if (s == ui->locationSlam_Button) {
        cmd = ui->locationSlamBash_Edit->text();
    } else {
        qDebug() << "Unknown run source";
        return;
    }

    if (!cmd.isEmpty()) {
        qDebug() << "Run script requested:" << cmd;
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
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
        qDebug() << "Sent exec command to robot:" << cmd;
    } else {
        qDebug() << "Empty command, ignoring";
    }
}

void ShDialog::onCloseButtonClicked()
{
    qDebug() << "Close script dialog requested";

    QObject *s = sender();
    if (!s) {
        qDebug() << "onCloseButtonClicked: no sender";
        return;
    }

  
    // Build the inner payload as JSON string: {"action":"stop","which":"slam"}
    QJsonObject inner;
    inner["action"] = "stop";
    inner["which"] = "slam";
    QJsonDocument innerDoc(inner);
    QString innerStr = QString::fromUtf8(innerDoc.toJson(QJsonDocument::Compact));

    // rosbridge publish message where msg.data is a string containing the JSON
    QJsonObject pub;
    pub["op"] = "publish";
    pub["topic"] = "/robot/exec_sh";
    pub["type"] = "std_msgs/String";
    QJsonObject msg;
    msg["data"] = innerStr; // NOTE: must be a string for std_msgs/String
    pub["msg"] = msg;

    QJsonDocument doc(pub);
    QString jsonString = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
    qDebug() << "Sent stop command to robot:" << innerStr;

}
