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

    bindSlots();
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

    // 清理SLAM地图监视器和线程
    if(slamMapMonitor){
        QMetaObject::invokeMethod(slamMapMonitor, "stop", Qt::BlockingQueuedConnection);
    }
    if(slamMapThread){
        if(slamMapThread->isRunning()){
            slamMapThread->quit();
            slamMapThread->wait();
        }
        delete slamMapThread;
        slamMapThread = nullptr;
    }
    if(slamMapMonitor){
        delete slamMapMonitor;
        slamMapMonitor = nullptr;
    }
    // 清理点云显示对象
    if(pcd){
        delete pcd;
        pcd = nullptr;
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

    // 启动点云地图监视器和线程
    slamMapThread = new QThread();
    slamMapMonitor = new SlamMapMonitor(m_worker, nullptr);
    slamMapMonitor->moveToThread(slamMapThread);
    slamMapThread->start();

    // 启动点云显示对象
    if (ui->pointCloud_Display) {
        pcd = new PointCloudDisplay(ui->pointCloud_Display);
        connect(slamMapMonitor, &SlamMapMonitor::pointCloudReceived, pcd, &PointCloudDisplay::onPointCloudReceived, Qt::QueuedConnection);
        connect(slamMapMonitor, &SlamMapMonitor::keyFrameMarkers, pcd, &PointCloudDisplay::onKeyFrameMarkers, Qt::QueuedConnection);
        connect(slamMapMonitor, &SlamMapMonitor::cameraMatrixReceived, pcd, &PointCloudDisplay::onCameraMatrixReceived, Qt::QueuedConnection);
        connect(slamMapMonitor, &SlamMapMonitor::cameraPoseReceived, pcd, &PointCloudDisplay::onCameraPoseReceived, Qt::QueuedConnection);
        // ensure pcd matches placeholder size now and when resized by the UI (splitter)
        ui->pointCloud_Display->installEventFilter(this);
        // sync initial size
        pcd->resize(ui->pointCloud_Display->size());
    }

    // 设置点云显示和特征点显示区域比例
    if (ui->splitter_2) {
        // index 0 is the left widget (point cloud), index 1 is the right widget (feature points)
        ui->splitter_2->setStretchFactor(0, 2);
        ui->splitter_2->setStretchFactor(1, 1);
    }
    if(ui->splitter){
        // index 0 is the top widget, index 1 is the bottom widget
        ui->splitter->setStretchFactor(0, 3);
        ui->splitter->setStretchFactor(1, 1);
    }

    // 初始化时隐藏定位模式按钮
    if (ui->groupBox_2) {
        ui->groupBox_2->setVisible(false);
        ui->localization_checkBox->setChecked(false);   // 定位模式默认关闭
    }

}

void ShDialog::bindSlots(){
    // 按钮槽函数
    connect(ui->startSlam_Button, &QPushButton::clicked, this, &ShDialog::onRunSLAMButtonClicked);
    connect(ui->locationSlam_Button, &QPushButton::clicked, this, &ShDialog::onRunSLAMButtonClicked);
    connect(ui->closeSlam_Button, &QPushButton::clicked, this, &ShDialog::onCloseSLAMButtonClicked);
    connect(ui->cancelControl_Button, &QPushButton::clicked, this, &ShDialog::onCancelControlButtonClicked);
    connect(ui->startControl_Button, &QPushButton::clicked, this, &ShDialog::onRunControlButtonClicked);

    // 连接webSocket信号到特征点图像监视器槽函数
    connect(m_worker, &WebSocketWorker::messageReceived, featuredImageMonitor, &CameraImageMonitor::onMessageReceived, Qt::QueuedConnection);
    connect(featuredImageMonitor, &CameraImageMonitor::imageReceived, this, [this](const QImage &img){
        if (ui->featurePoint_Display) {
            // The worker already scales to the configured target size (SmoothTransformation), set pixmap directly
            ui->featurePoint_Display->setPixmap(QPixmap::fromImage(img));
        }
    }, Qt::QueuedConnection);


    // 连接webSocket信号到SLAM地图监视器槽函数
    connect(m_worker, &WebSocketWorker::messageReceived, slamMapMonitor, &SlamMapMonitor::onMessageReceived, Qt::QueuedConnection);


    // 控制按钮槽函数
    // Ensure buttons do not auto-repeat when held down (send only once per click)
    if (ui->w_Button) {
        ui->w_Button->setAutoRepeat(false);
        ui->w_Button->setCheckable(false);
        ui->w_Button->setShortcut(QKeySequence(Qt::Key_W));
        connect(ui->w_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->s_Button) {
        ui->s_Button->setAutoRepeat(false);
        ui->s_Button->setCheckable(false);
        ui->s_Button->setShortcut(QKeySequence(Qt::Key_S));
        connect(ui->s_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->a_Button) {
        ui->a_Button->setAutoRepeat(false);
        ui->a_Button->setCheckable(false);
        ui->a_Button->setShortcut(QKeySequence(Qt::Key_A));
        connect(ui->a_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->d_Button) {
        ui->d_Button->setAutoRepeat(false);
        ui->d_Button->setCheckable(false);
        ui->d_Button->setShortcut(QKeySequence(Qt::Key_D));
        connect(ui->d_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->z_Button) {
        ui->z_Button->setAutoRepeat(false);
        ui->z_Button->setCheckable(false);
        ui->z_Button->setShortcut(QKeySequence(Qt::Key_Z));
        connect(ui->z_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->c_Button) {
        ui->c_Button->setAutoRepeat(false);
        ui->c_Button->setCheckable(false);
        ui->c_Button->setShortcut(QKeySequence(Qt::Key_C));
        connect(ui->c_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->x_Button) {
        ui->x_Button->setAutoRepeat(false);
        ui->x_Button->setCheckable(false);
        ui->x_Button->setShortcut(QKeySequence(Qt::Key_X));
        connect(ui->x_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->r_Button) {
        ui->r_Button->setAutoRepeat(false);
        ui->r_Button->setCheckable(false);
        ui->r_Button->setShortcut(QKeySequence(Qt::Key_R));
        connect(ui->r_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }
    if (ui->e_Button) {
        ui->e_Button->setAutoRepeat(false);
        ui->e_Button->setCheckable(false);
        ui->e_Button->setShortcut(QKeySequence(Qt::Key_E));
        connect(ui->e_Button, &QPushButton::clicked, this, &ShDialog::onControlButtonClicked);
    }

    // 定位模式开关：发布模式变更到 rosbridge
    if (ui->localization_checkBox) {
        connect(ui->localization_checkBox, &QCheckBox::toggled, this, &ShDialog::onLocalizationModeToggled);
    }

}
// 事件过滤器，适应组件大小
bool ShDialog::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->featurePoint_Display && event->type() == QEvent::Resize) {
        QSize newSize = ui->featurePoint_Display->size();
        if (featuredImageMonitor) {
            QMetaObject::invokeMethod(featuredImageMonitor, "setTargetSize", Qt::QueuedConnection, Q_ARG(QSize, newSize));
        }
    }
    // forward resize events for pointCloud_Display to the embedded PointCloudDisplay widget
    if (watched == ui->pointCloud_Display && event->type() == QEvent::Resize) {
        QSize newSize = ui->pointCloud_Display->size();
        if (pcd) {
            // eventFilter runs in the GUI thread; call resize directly to avoid QMetaObject warnings
            pcd->resize(newSize);
        }
    }
    return QDialog::eventFilter(watched, event);
}

void ShDialog::onRunSLAMButtonClicked()
{
    QObject *s = sender();
    if (!s) return;
    QString cmd;
    // 从config文件加载命令
    if (s == ui->startSlam_Button) {
        cmd = loadCmdFromConfig("start_slam_bash");
        ui->localization_checkBox->setChecked(false);   // 非定位模式
    } else if (s == ui->locationSlam_Button) {
        cmd = loadCmdFromConfig("start_slam_location_bash");
        ui->localization_checkBox->setChecked(true);    // 定位模式
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

    // 启动或确保 SLAM 地图点云订阅（通过 slamMapMonitor 发送 subscribe 请求）
    if (slamMapMonitor) {
        // If thread is not running (stopped previously), restart it
        if (slamMapThread && !slamMapThread->isRunning()) {
            slamMapMonitor->moveToThread(slamMapThread);
            slamMapThread->start();
        }
        // ensure the worker->monitor connection exists (disconnect old then reconnect to avoid duplicates)
        QObject::disconnect(m_worker, nullptr, slamMapMonitor, nullptr);
        connect(m_worker, &WebSocketWorker::messageReceived, slamMapMonitor, &SlamMapMonitor::onMessageReceived, Qt::QueuedConnection);
        // Ask the monitor to send a rosbridge subscribe request
        QMetaObject::invokeMethod(slamMapMonitor, "start", Qt::QueuedConnection);
    }

    // 显示定位模式按钮
    if (ui->groupBox_2) {
        ui->groupBox_2->setVisible(true);
    }

    // Advertise /SLAM/localizationMode topic via rosbridge so subscribers can infer type
    if (!localizationAdvertised && m_worker) {
        QJsonObject adv;
        adv["op"] = "advertise";
        adv["topic"] = "/SLAM/localizationMode";
        adv["type"] = "std_msgs/Bool";
        QJsonDocument docAdv(adv);
        QString advStr = QString::fromUtf8(docAdv.toJson(QJsonDocument::Compact));
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, advStr));
        localizationAdvertised = true;
        qDebug() << "Advertised /SLAM/localizationMode";
    }

}



// 关闭SLAM建图
void ShDialog::onCloseSLAMButtonClicked()
{
    // qDebug() << "Close script dialog requested";

    QObject *s = sender();
    if (!s) {
        qDebug() << "onCloseSLAMButtonClicked: no sender";
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
    qDebug() << "Sent stop slam command to robot:" << innerStr;

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

    // Disconnect worker -> slamMapMonitor to stop receiving further messages immediately
    if (m_worker && slamMapMonitor) {
        QObject::disconnect(m_worker, nullptr, slamMapMonitor, nullptr);
    }

    // Ask slamMapMonitor to stop (unsubscribe) and wait for it to finish to avoid races
    if (slamMapMonitor) {
        QMetaObject::invokeMethod(slamMapMonitor, "stop", Qt::QueuedConnection);
    }

    // 清空点云和关键帧可视化
    if (pcd) {
        QMetaObject::invokeMethod(pcd, "clearPointCloud", Qt::QueuedConnection);
        QMetaObject::invokeMethod(pcd, "clearKeyFrames", Qt::QueuedConnection);
        QMetaObject::invokeMethod(pcd, "clearCameraPoses", Qt::QueuedConnection);
        QMetaObject::invokeMethod(pcd, "clearCameraMatrix", Qt::QueuedConnection);
    }

    // 隐藏定位模式按钮
    if (ui->groupBox_2) {
        ui->groupBox_2->setVisible(false);
        ui->localization_checkBox->setChecked(false);   // 定位模式关闭
    }

    // Unadvertise localization topic if we advertised it earlier
    if (localizationAdvertised && m_worker) {
        QJsonObject unadv;
        unadv["op"] = "unadvertise";
        unadv["topic"] = "/SLAM/localizationMode";
        QJsonDocument docUn(unadv);
        QString unStr = QString::fromUtf8(docUn.toJson(QJsonDocument::Compact));
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, unStr));
        localizationAdvertised = false;
        qDebug() << "Unadvertised /SLAM/localizationMode";
    }

}

// 启动遥控
void ShDialog::onRunControlButtonClicked()
{
    QObject *s = sender();
    if(!s){
        qDebug() << "onRunControlButtonClicked: no sender";
        return;
    }
    QString cmd = loadCmdFromConfig("start_control_bash");
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
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
        qDebug() << "Sent exec command to robot:" << cmd;
    }else{
        qDebug() << "Empty command, ignoring";
    }
}


// 取消遥控
void ShDialog::onCancelControlButtonClicked()
{
    QObject *s = sender();
    if(!s){
        qDebug() << "onCancelControlButtonClicked: no sender";
        return;
    }
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
    qDebug() << "Sent stop control command to robot:" << innerStr;
}

// 接收控制按钮点击事件
void ShDialog::onControlButtonClicked()
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
    qDebug() << "Sent remote SLAM control command:" << cmdStr;

}

// 当 localization_checkBox 状态变化时，通过 rosbridge 发布 /SLAM/localizationMode (std_msgs/Bool)
void ShDialog::onLocalizationModeToggled(bool checked)
{
    if (!m_worker) {
        qDebug() << "onLocalizationModeToggled: m_worker is null, cannot publish";
        return;
    }

    QJsonObject pub;
    pub["op"] = "publish";
    pub["topic"] = "/SLAM/localizationMode"; // use this topic name
    pub["type"] = "std_msgs/Bool";
    QJsonObject msg;
    msg["data"] = checked; // bool -> JSON true/false
    pub["msg"] = msg;

    QJsonDocument doc(pub);
    QString jsonString = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, jsonString));
    qDebug() << "Published /SLAM/localizationMode:" << checked;
}