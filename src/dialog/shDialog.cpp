#include "dialog/shDialog.h"
#include "ui_shDialog.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"


ShDialog::ShDialog(WebSocketWorker *webSocketWorker, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ShDialog)
    , m_worker(webSocketWorker)
{
    ui->setupUi(this);

    // 初始化图像处理相关内容
    init();

    // 按钮槽函数
    connect(ui->startSlam_Button, &QPushButton::clicked, this, &ShDialog::onRunButtonClicked);
    connect(ui->locationSlam_Button, &QPushButton::clicked, this, &ShDialog::onRunButtonClicked);
    connect(ui->closeSlam_Button, &QPushButton::clicked, this, &ShDialog::onCloseButtonClicked);

    // 连接webSocket信号到特征点图像监视器槽函数
    connect(m_worker, &WebSocketWorker::messageReceived, featuredImageMonitor, &CameraImageMonitor::onMessageReceived, Qt::QueuedConnection);
    connect(featuredImageMonitor, &CameraImageMonitor::imageReceived, this, [this](const QImage &img){
        if (ui->featurePoint_Display) {
            // The worker already scales to the configured target size (SmoothTransformation), set pixmap directly
            ui->featurePoint_Display->setPixmap(QPixmap::fromImage(img));
        }
    }, Qt::QueuedConnection);
}

ShDialog::~ShDialog()
{
    // Stop pull timer
    if (featuredImagePullTimer) {
        featuredImagePullTimer->stop();
        delete featuredImagePullTimer;
        featuredImagePullTimer = nullptr;
    }

    // Request monitor to unsubscribe
    if (featuredImageMonitor) {
        QMetaObject::invokeMethod(featuredImageMonitor, "stop", Qt::BlockingQueuedConnection);
    }

    // Disconnect worker -> monitor connections to avoid messages during teardown
    if (m_worker && featuredImageMonitor) {
        QObject::disconnect(m_worker, nullptr, featuredImageMonitor, nullptr);
    }

    // Stop and delete thread cleanly
    if (featuredImageThread) {
        if (featuredImageThread->isRunning()) {
            featuredImageThread->quit();
            featuredImageThread->wait();
        }
        delete featuredImageThread;
        featuredImageThread = nullptr;
    }

    // Delete monitor (it lives in the thread; thread stopped above so it's safe)
    if (featuredImageMonitor) {
        delete featuredImageMonitor;
        featuredImageMonitor = nullptr;
    }

    delete ui;
}

void ShDialog::init()
{
    // 图像pull定时器
    featuredImagePullTimer = new QTimer(this);
    featuredImagePullTimer->setInterval(50); // 50ms间隔
    // 图像处理线程（不设 parent，析构由本类显式管理，以避免父对象自动删除时线程仍在运行）
    featuredImageThread = new QThread();
    m_featureTopic = loadTopicFromConfig("featureImageCompressed_topic");
    featuredImageMonitor = new CameraImageMonitor(m_worker, nullptr, m_featureTopic);
    featuredImageMonitor->moveToThread(featuredImageThread);
    featuredImageThread->start();


    // 避免图像显示标签在pixmap调整大小时自身也调整大小
    if (ui->featurePoint_Display) {
        // Use explicit scaling rather than letting the QLabel auto-scale the pixmap
        ui->featurePoint_Display->setScaledContents(false);
        ui->featurePoint_Display->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    }

    // 设置目标显示尺寸和最大帧率
    if(ui->featurePoint_Display){
        QSize targetSize = ui->featurePoint_Display->size();
        QMetaObject::invokeMethod(featuredImageMonitor, "setTargetSize", Qt::QueuedConnection, Q_ARG(QSize, targetSize));
    }
    QMetaObject::invokeMethod(featuredImageMonitor, "setMaxFps", Qt::QueuedConnection, Q_ARG(int, 20)); // 20 FPS
    // 安装事件过滤器以在 imageRawDisplay 尺寸变更时更新目标尺寸
    if (ui->featurePoint_Display) {
        ui->featurePoint_Display->installEventFilter(this);
    }

    // Connect the pull timer to requestFrame once
    if (featuredImagePullTimer && featuredImageMonitor) {
        connect(featuredImagePullTimer, &QTimer::timeout, this, [this]() {
            QMetaObject::invokeMethod(featuredImageMonitor, "requestFrame", Qt::QueuedConnection);
        });
    }

}

bool ShDialog::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->featurePoint_Display && event->type() == QEvent::Resize) {
        QSize newSize = ui->featurePoint_Display->size();
        if (featuredImageMonitor) {
            QMetaObject::invokeMethod(featuredImageMonitor, "setTargetSize", Qt::QueuedConnection, Q_ARG(QSize, newSize));
        }
    }
    return QDialog::eventFilter(watched, event);
}

void ShDialog::onRunButtonClicked()
{
  

    QObject *s = sender();
    if (!s) return;
    QString cmd;
    // 从config文件加载命令
    if (s == ui->startSlam_Button) {
        cmd = loadCmdFromConfig("start_slam_bash");
    } else if (s == ui->locationSlam_Button) {
        cmd = loadCmdFromConfig("start_slam_location_bash");
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

    // 启动特征点图像订阅
    if (featuredImageMonitor) {
        // If thread is not running (stopped previously), restart it
        if (featuredImageThread && !featuredImageThread->isRunning()) {
            featuredImageMonitor->moveToThread(featuredImageThread);
            featuredImageThread->start();
        }
        // ensure the worker->monitor connection exists (disconnect old then reconnect to avoid duplicates)
        QObject::disconnect(m_worker, nullptr, featuredImageMonitor, nullptr);
        connect(m_worker, &WebSocketWorker::messageReceived, featuredImageMonitor, &CameraImageMonitor::onMessageReceived, Qt::QueuedConnection);
        // Set the desired target size on the worker *right before* starting so it will scale to the shown widget size
        if (ui->featurePoint_Display) {
            QSize target = ui->featurePoint_Display->size();
            QMetaObject::invokeMethod(featuredImageMonitor, "setTargetSize", Qt::BlockingQueuedConnection, Q_ARG(QSize, target));
        }
        QMetaObject::invokeMethod(featuredImageMonitor, "start", Qt::QueuedConnection);
        // Request an immediate frame to refresh the display (queued)
        QMetaObject::invokeMethod(featuredImageMonitor, "requestFrame", Qt::QueuedConnection);
    }

    // Start image pull timer (connect once in init to avoid duplicate connects)
    if (featuredImagePullTimer && featuredImageMonitor) {
        // Connect once (already connected in init to avoid duplicate lambdas on repeated starts)
        if (!featuredImagePullTimer->isActive()) {
            featuredImagePullTimer->start();
        }
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

    // 停止特征点图像订阅
    if(featuredImagePullTimer){
        featuredImagePullTimer->stop();
    }
    if (featuredImageMonitor) {
        QMetaObject::invokeMethod(featuredImageMonitor, "stop", Qt::QueuedConnection);
    }

    // 图像显示关闭
    if (ui->featurePoint_Display) {
        ui->featurePoint_Display->clear();
    }

}
