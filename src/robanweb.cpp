#include "robanweb.h"
#include <QFileInfo>
#include <QDateTime>
#include <QCoreApplication>
#include <QDir>
#include <QFile>

robanweb::robanweb(QWidget *parent)
    : QMainWindow(parent), ui(new Ui_robanweb), webSocketWorker(nullptr), webSocketThread(nullptr), webSocketWorker2(nullptr), webSocketThread2(nullptr), reconnectTimer(new QTimer(this)), reconnectTimer2(nullptr), isReconnecting(false), isReconnecting2(false), reconnectAttempts(0), reconnectAttempts2(0)
{
    ui->setupUi(this);
    // 设置状态栏
    settingStatusBar();

    // 初始化 robot2 信息显示动画及初始状态
    if (ui->groupBox_robotInfor2) {
        robot2Animation = new QPropertyAnimation(ui->groupBox_robotInfor2, "maximumWidth", this);
        ui->groupBox_robotInfor2->setVisible(false); // 默认隐藏
    }

    init();
    bindSlots(); // 绑定相关槽函数
    // 初始化状态标签
    updateStatusLabel("未连接");
    // qDebug() << "robanweb run in thread:" << QThread::currentThread();
}

robanweb::~robanweb()
{

    if (m_slamDialog)
    {
        delete m_slamDialog;
        m_slamDialog = nullptr;
    }

    // 移除并删除多机器人管理器
    if (multiRobot_mgr1)
    {
        delete multiRobot_mgr1;
        multiRobot_mgr1 = nullptr;
    }
    if (multiRobot_mgr2)
    {
        delete multiRobot_mgr2;
        multiRobot_mgr2 = nullptr;
    }

    // 在析构中，确保线程已停止并清理
    if (webSocketThread)
    {
        QMetaObject::invokeMethod(webSocketWorker, "closeConnection", Qt::QueuedConnection);
        webSocketThread->quit();
        webSocketThread->wait();
        // webSocketWorker 会在 thread finished 时 deleteLater 被调用
        delete webSocketThread;
        webSocketThread = nullptr;
        webSocketWorker = nullptr;
    }
    // 清理第二个 webSocket 线程
    if (webSocketThread2)
    {
        if (webSocketWorker2)
        {
            QMetaObject::invokeMethod(webSocketWorker2, "closeConnection", Qt::QueuedConnection);
        }
        webSocketThread2->quit();
        webSocketThread2->wait();
        delete webSocketThread2;
        webSocketThread2 = nullptr;
        webSocketWorker2 = nullptr;
    }

    delete reconnectTimer;
    // delete imagePullTimer;
    delete ui;
}
// 初始化
void robanweb::init()
{
    // 创建 worker 和线程，把 WebSocket 操作放到子线程
    webSocketWorker = new WebSocketWorker();  // robot1的通信worker
    webSocketWorker2 = new WebSocketWorker(); // robot2的通信worker
    webSocketThread = new QThread(this);      // robot1的通信线程
    webSocketThread2 = new QThread(this);     // robot2的通信线程
    webSocketWorker->moveToThread(webSocketThread);
    webSocketWorker2->moveToThread(webSocketThread2);
    webSocketThread->start();
    webSocketThread2->start();
    // 通过multiRobotManager创建并管理robot1和robot2的连接
    multiRobot_mgr1 = new MultiRobotManager(webSocketWorker, QStringLiteral("robot1"), this);
    multiRobot_mgr2 = new MultiRobotManager(webSocketWorker2, QStringLiteral("robot2"), this);
    // multiRobot_mgr1/2 管理连接信号 -> 统一由 onRobot* 槽处理
    if (multiRobot_mgr1)
    {
        connect(multiRobot_mgr1, &MultiRobotManager::robotConnected, this, &robanweb::onRobotConnected);
        connect(multiRobot_mgr1, &MultiRobotManager::robotDisconnected, this, &robanweb::onRobotDisconnected);
        connect(multiRobot_mgr1, &MultiRobotManager::robotError, this, &robanweb::onRobotError);
    }
    if (multiRobot_mgr2)
    {
        connect(multiRobot_mgr2, &MultiRobotManager::robotConnected, this, &robanweb::onRobotConnected);
        connect(multiRobot_mgr2, &MultiRobotManager::robotDisconnected, this, &robanweb::onRobotDisconnected);
        connect(multiRobot_mgr2, &MultiRobotManager::robotError, this, &robanweb::onRobotError);
    }


    // reconnectTimer 用于主连接的重连逻辑。
    reconnectTimer->setInterval(5000); // 每5秒尝试重连
    // 将 reconnectTimer 的 timeout 连接到统一的 doReconnectFor("robot1")，
    // 以便定时触发统一重连逻辑（以前的连接可能被误删）。
    connect(reconnectTimer, &QTimer::timeout, this, [this]()
            { doReconnectFor(QLatin1String("robot1")); });

    // 使用 MultiRobotManager 管理图像拉取定时器
    if (multiRobot_mgr1)
    {
        QMetaObject::invokeMethod(multiRobot_mgr1, "setImagePullInterval", Qt::QueuedConnection, Q_ARG(int, 50));
    }
    if (multiRobot_mgr2)
    {
        QMetaObject::invokeMethod(multiRobot_mgr2, "setImagePullInterval", Qt::QueuedConnection, Q_ARG(int, 50));
    }

    // Ensure camera target size and FPS are applied via the manager (manager will forward to the camera monitor)
    if (ui->imageRawDisplay && multiRobot_mgr1)
    {
        ui->imageRawDisplay->setAlignment(Qt::AlignCenter);
        ui->imageRawDisplay->setScaledContents(false);
        ui->imageRawDisplay->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        QSize target = ui->imageRawDisplay->size();
        QMetaObject::invokeMethod(multiRobot_mgr1, "setCameraTargetSize", Qt::QueuedConnection, Q_ARG(QSize, target));
        QMetaObject::invokeMethod(multiRobot_mgr1, "setCameraMaxFps", Qt::QueuedConnection, Q_ARG(int, 20));
        // keep event filter installed on UI; eventFilter will update manager target size on resize
        ui->imageRawDisplay->installEventFilter(this);
    }
    if (ui->imageRawDisplay_2 && multiRobot_mgr2)
    {
        ui->imageRawDisplay_2->setAlignment(Qt::AlignCenter);
        ui->imageRawDisplay_2->setScaledContents(false);
        ui->imageRawDisplay_2->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        QSize target2 = ui->imageRawDisplay_2->size();
        QMetaObject::invokeMethod(multiRobot_mgr2, "setCameraTargetSize", Qt::QueuedConnection, Q_ARG(QSize, target2));
        QMetaObject::invokeMethod(multiRobot_mgr2, "setCameraMaxFps", Qt::QueuedConnection, Q_ARG(int, 20));
        // keep event filter installed on UI; eventFilter will update manager target size on resize
        ui->imageRawDisplay_2->installEventFilter(this);
    }

    // 3D模型显示初始化
    // 使用类的静态方法 resolveAssetPath() 来解析资源路径，便于在其它地方重复使用
    QString modelRobot = robanweb::resolveAssetPath("assets/Roban.fbx"); // 默认机器人模型路径
    QString modelScene = robanweb::resolveAssetPath("assets/scene.obj"); // 默认场景模型路径

    sceneManager = new SceneManager(poseMonitor, modelScene, this);           // 场景管理器
    robotManager = new RobotManager(modelRobot, sceneManager, nullptr, this); // 机器人管理器 (monitors injected by manager)
    // 将robotManager和sceneManager附加到multiRobot_mgr1，以便注入监视器指针
    if (multiRobot_mgr1)
    {
        multiRobot_mgr1->attachManagers(sceneManager, robotManager);
    }

    if (ui->modelDisplay)
    {
        QWidget *placeholder = ui->modelDisplay; // UI 中的占位 widget
        QWidget *parent = placeholder->parentWidget();
        QRect geom = placeholder->geometry();
        QLayout *parentLayout = parent ? parent->layout() : nullptr;

        // 在同一父容器中创建 ModelDisplay，并在父 layout 中替换占位 widget（如果存在）
        modelDisplay = new ModelDisplay(robotManager, sceneManager, parent ? parent : placeholder);
        modelDisplay->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        if (parentLayout)
        {
            // 在 layout 中找到占位位置并替换
            int idx = -1;
            for (int i = 0; i < parentLayout->count(); ++i)
            {
                QLayoutItem *it = parentLayout->itemAt(i);
                if (it && it->widget() == placeholder)
                {
                    idx = i;
                    break;
                }
            }
            if (idx != -1)
            {
                parentLayout->removeWidget(placeholder);
                placeholder->hide();
                // If the parent layout is a box layout we can insert at index, otherwise append
                if (QBoxLayout *box = qobject_cast<QBoxLayout *>(parentLayout))
                {
                    box->insertWidget(idx, modelDisplay);
                }
                else
                {
                    // fallback: add to layout (will appear at end)
                    parentLayout->addWidget(modelDisplay);
                }
            }
            else
            {
                // 找不到占位 index，回退为手动定位
                modelDisplay->setGeometry(geom);
            }
        }
        else
        {
            // 父容器没有 layout，直接按占位 geometry 放置
            modelDisplay->setGeometry(geom);
        }

        modelDisplay->show();
    }
}

// 设置状态栏组件
void robanweb::settingStatusBar()
{
    // 连接状态
    connect_label1 = new QLabel("【robot1】未连接");
    connect_label1->setMinimumWidth(200);
    connect_label1->setFont(QFont("Microsoft YaHei UI", 10, QFont::Bold));
    connect_label1->setAlignment(Qt::AlignVCenter | Qt::AlignVCenter);
    ui->statusbar->addWidget(connect_label1);

    connect_label2 = new QLabel("【robot2】未连接");
    connect_label2->setMinimumWidth(200);
    connect_label2->setFont(QFont("Microsoft YaHei UI", 10, QFont::Bold));
    connect_label2->setAlignment(Qt::AlignVCenter | Qt::AlignVCenter);
    ui->statusbar->addWidget(connect_label2);
}

void robanweb::bindSlots()
{
    // 连接信号槽
    connect(ui->connect_Button, &QPushButton::clicked, this, &robanweb::onConnectSettingButtonClicked);
    // SLAM和控制按钮槽
    connect(ui->SLAM_Control_Button, &QPushButton::clicked, this, &robanweb::onSlamControlButtonClicked);
    // 语音控制按钮槽
    connect(ui->voice_Button, &QPushButton::clicked, this, &robanweb::onVoiceControlButtonClicked);
    // 机器人控制按钮槽
    connect(ui->roban_Control_Button, &QPushButton::clicked, this, &robanweb::onrobanControlButtonclicked);

    // 连接任务管理器信号（由 MultiRobotManager 转发）
    if (multiRobot_mgr1)
    {
        connect(multiRobot_mgr1, &MultiRobotManager::taskExecuted,
                this, &robanweb::onTaskExecuted, Qt::QueuedConnection);
        connect(multiRobot_mgr1, &MultiRobotManager::taskStopped,
                this, &robanweb::onTaskStopped, Qt::QueuedConnection);
        connect(multiRobot_mgr1, &MultiRobotManager::taskAdded,
                this, &robanweb::onAddTask, Qt::QueuedConnection);
    }
    if (multiRobot_mgr2)
    {
        connect(multiRobot_mgr2, &MultiRobotManager::taskExecuted,
                this, &robanweb::onTaskExecuted, Qt::QueuedConnection);
        connect(multiRobot_mgr2, &MultiRobotManager::taskStopped,
                this, &robanweb::onTaskStopped, Qt::QueuedConnection);
        connect(multiRobot_mgr2, &MultiRobotManager::taskAdded,
                this, &robanweb::onAddTask, Qt::QueuedConnection);
    }
    // 当线程启动时可做初始化
    connect(webSocketThread, &QThread::finished, webSocketWorker, &QObject::deleteLater);
    connect(webSocketThread2, &QThread::finished, webSocketWorker2, &QObject::deleteLater);

    // Connect manager imageReceived -> UI
    if (multiRobot_mgr1)
    {
        connect(multiRobot_mgr1, &MultiRobotManager::imageReceived, this, [this](const QImage &img)
                {
                // only update UI if manager reports connected to avoid races where queued image
                // updates arrive after a disconnect and re-fill cleared UI
                if (multiRobot_mgr1 && multiRobot_mgr1->isConnected() && ui && ui->imageRawDisplay) {
                    QPixmap px = QPixmap::fromImage(img);
                    QSize lbl = ui->imageRawDisplay->size();
                    // scale to fit label keeping aspect ratio (allow upscaling so small frames are visible)
                    ui->imageRawDisplay->setPixmap(px.scaled(lbl, Qt::KeepAspectRatio, Qt::SmoothTransformation));
                }
                }, Qt::QueuedConnection);
    }
    if (multiRobot_mgr2)
    {
        connect(multiRobot_mgr2, &MultiRobotManager::imageReceived, this, [this](const QImage &img)
                {
                if (multiRobot_mgr2 && multiRobot_mgr2->isConnected() && ui && ui->imageRawDisplay_2) {
                    QPixmap px = QPixmap::fromImage(img);
                    QSize lbl = ui->imageRawDisplay_2->size();
                    // scale to fit label keeping aspect ratio (allow upscaling so small frames are visible)
                    ui->imageRawDisplay_2->setPixmap(px.scaled(lbl, Qt::KeepAspectRatio, Qt::SmoothTransformation));
                }
                }, Qt::QueuedConnection);
    }

    // 更新IMU数据显示  IMU 信号由 MultiRobotManager 转发并管理
    if (multiRobot_mgr1)
    {
        connect(multiRobot_mgr1, &MultiRobotManager::imuOrientationUpdated, this, [this](double w, double x, double y, double z)
                {
            if (multiRobot_mgr1 && multiRobot_mgr1->isConnected() && ui) {
                QMetaObject::invokeMethod(ui->ori_w, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(w, 'f', 2)));
                QMetaObject::invokeMethod(ui->ori_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(x, 'f', 2)));
                QMetaObject::invokeMethod(ui->ori_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(y, 'f', 2)));
                QMetaObject::invokeMethod(ui->ori_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(z, 'f', 2)));
            } }, Qt::QueuedConnection);
        connect(multiRobot_mgr1, &MultiRobotManager::imuAngularVelocityUpdated, this, [this](double x, double y, double z)
                {
            if (multiRobot_mgr1 && multiRobot_mgr1->isConnected() && ui) {
                ui->ang_x->setText(QString::number(x, 'f', 2));
                ui->ang_y->setText(QString::number(y, 'f', 2));
                ui->ang_z->setText(QString::number(z, 'f', 2));
            } }, Qt::QueuedConnection);
        connect(multiRobot_mgr1, &MultiRobotManager::imuLinearAccelerationUpdated, this, [this](double x, double y, double z)
                {
            if (multiRobot_mgr1 && multiRobot_mgr1->isConnected() && ui) {
                ui->lin_x->setText(QString::number(x, 'f', 2));
                ui->lin_y->setText(QString::number(y, 'f', 2));
                ui->lin_z->setText(QString::number(z, 'f', 2));
            } }, Qt::QueuedConnection);
    }

    if (multiRobot_mgr2)
    {
        connect(multiRobot_mgr2, &MultiRobotManager::imuOrientationUpdated, this, [this](double w, double x, double y, double z)
                {
            if (multiRobot_mgr2 && multiRobot_mgr2->isConnected() && ui) {
                QMetaObject::invokeMethod(ui->ori_w_2, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(w, 'f', 2)));
                QMetaObject::invokeMethod(ui->ori_x_2, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(x, 'f', 2)));
                QMetaObject::invokeMethod(ui->ori_y_2, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(y, 'f', 2)));
                QMetaObject::invokeMethod(ui->ori_z_2, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(z, 'f', 2)));
            } }, Qt::QueuedConnection);
        connect(multiRobot_mgr2, &MultiRobotManager::imuAngularVelocityUpdated, this, [this](double x, double y, double z)
                {
            if (multiRobot_mgr2 && multiRobot_mgr2->isConnected() && ui) {
                ui->ang_x_2->setText(QString::number(x, 'f', 2));
                ui->ang_y_2->setText(QString::number(y, 'f', 2));
                ui->ang_z_2->setText(QString::number(z, 'f', 2));
            } }, Qt::QueuedConnection);
        connect(multiRobot_mgr2, &MultiRobotManager::imuLinearAccelerationUpdated, this, [this](double x, double y, double z)
                {
            if (multiRobot_mgr2 && multiRobot_mgr2->isConnected() && ui) {
                ui->lin_x_2->setText(QString::number(x, 'f', 2));
                ui->lin_y_2->setText(QString::number(y, 'f', 2));
                ui->lin_z_2->setText(QString::number(z, 'f', 2));
            } }, Qt::QueuedConnection);
    }
}

// 任务执行槽函数
void robanweb::onTaskExecuted(const QString &scriptPath)
{
    QString cmd = scriptPath;
    // 使用脚本文件名作为简单的 task 标识（可改为 UUID 或其它规则）
    QString taskId = QFileInfo(scriptPath).fileName();
    if (taskId.isEmpty())
    {
        taskId = QString::number(QDateTime::currentMSecsSinceEpoch());
    }

    if (!cmd.isEmpty())
    {
        // 直接把脚本路径字符串放入 msg.data，cmd_executor.py 对纯字符串也能正确处理。
        QJsonObject pub;
        pub["op"] = "publish";
        pub["topic"] = "/robot/exec_sh";
        pub["type"] = "std_msgs/String";
        QJsonObject msg;
        // 直接发送脚本路径（非 JSON 字符串），可确保机器人端按原样将其当作命令执行
        msg["data"] = cmd;
        pub["msg"] = msg;
        QJsonDocument doc(pub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // 通过 worker 发送发布消息
        QMetaObject::invokeMethod(webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));

        QMessageBox::information(this, "任务执行", "已发送任务执行命令:\n" + cmd + "\n任务ID:" + taskId);
        qDebug() << "Sent exec command to robot (data):" << cmd << " task:" << taskId;
    }
    else
    {
        QMessageBox::warning(this, tr("连接错误"), tr("WebSocket连接未建立，无法执行任务"));
    }
}

// 添加任务槽函数
void robanweb::onAddTask(Task *task)
{
    QString codePath = task->getTaskCodePath();
    QString sourceCmd = "source ~/robot_ros_application/catkin_ws/devel/setup.bash";
    QString execCmd = QString("python %1").arg(codePath);
    QString fullCmd = QString("%1 && %2").arg(sourceCmd).arg(execCmd);
    if (!fullCmd.isEmpty())
    {
        // 1. 构建完整的 payload 对象，包含 action, sh_path 和 data
        QJsonObject payloadObj;
        payloadObj["action"] = "add";
        payloadObj["sh_path"] = task->getScriptPath();
        payloadObj["data"] = fullCmd; // 将实际命令放入 data 字段

        // 2. 将 payload 对象序列化为 JSON 字符串
        QJsonDocument payloadDoc(payloadObj);
        QString payloadStr = QString::fromUtf8(payloadDoc.toJson(QJsonDocument::Compact));

        // 3. 构建 rosbridge 发布消息
        QJsonObject pub;
        pub["op"] = "publish";
        pub["topic"] = "/robot/exec_sh";
        pub["type"] = "std_msgs/String";

        // 4. 将序列化后的 JSON 字符串作为 ROS 消息的内容
        QJsonObject msg;
        msg["data"] = payloadStr;
        pub["msg"] = msg;

        QJsonDocument doc(pub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));

        // 通过 worker 发送发布消息
        QMetaObject::invokeMethod(webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
        qDebug() << "Sent add task command to robot:" << payload;
    }
}

// 任务停止槽函数
void robanweb::onTaskStopped(const QString &scriptPath)
{
    if (!scriptPath.isEmpty())
    {
        // 使用与 onTaskExecuted 相同的 task 标识约定（脚本文件名），便于机器人端根据 task 停止对应进程
        QString taskId = QFileInfo(scriptPath).fileName();
        if (taskId.isEmpty())
            taskId = scriptPath;

        QJsonObject payloadObj;
        payloadObj["action"] = "stop";
        // 向机器人端指明要停止的是由本地脚本启动的控制类型进程（cmd_executor.py 在启动本地脚本时会把 which 标记为 'control'）
        payloadObj["which"] = "control";
        payloadObj["task"] = taskId;
        payloadObj["script_path"] = scriptPath;

        QJsonDocument payloadDoc(payloadObj);
        QString payloadStr = QString::fromUtf8(payloadDoc.toJson(QJsonDocument::Compact));

        QJsonObject pub;
        pub["op"] = "publish";
        pub["topic"] = "/robot/exec_sh";
        pub["type"] = "std_msgs/String";
        QJsonObject msg;
        msg["data"] = payloadStr;
        pub["msg"] = msg;
        QJsonDocument doc(pub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));

        // 通过 worker 发送发布消息
        QMetaObject::invokeMethod(webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
        qDebug() << "Sent stop task command to robot:" << payload;
    }
}

// SLAM控制按钮槽函数
void robanweb::onSlamControlButtonClicked()
{
    if (!m_slamDialog)
    {
        m_slamDialog = new slamDialog(webSocketWorker, this);
    }
    m_slamDialog->show();
    m_slamDialog->raise();
    m_slamDialog->activateWindow();
}

// 机器人控制按钮槽函数
void robanweb::onrobanControlButtonclicked()
{
    RobotControlDialog dialog(webSocketWorker, this);
    if (dialog.exec() == QDialog::Accepted)
    {
        qDebug() << "机器人控制对话框开启";
    }
}

// 连接设置槽函数
void robanweb::onConnectSettingButtonClicked()
{
    ConnectDialog dialog(this);
    // 建立通信连接信号槽
    connect(&dialog, &ConnectDialog::connectRequested, this, [&](const QString &url)
            { startConnectFor(QLatin1String("robot1"), url); });
    connect(&dialog, &ConnectDialog::connectRequested2, this, [&](const QString &url)
            { startConnectFor(QLatin1String("robot2"), url); });

    // 只在对应取消按钮按下时关闭对应机器人的连接。
    connect(&dialog, &ConnectDialog::cancelRequested, this, [this]() {
        qDebug() << "ConnectDialog: robot1 cancel requested";
        if (webSocketWorker) {
            qDebug() << "请求 worker 关闭 WebSocket 连接 (robot1)...";
            QMetaObject::invokeMethod(webSocketWorker, "closeConnection", Qt::QueuedConnection);
        }
        if(ui->imageRawDisplay) {
            ui->imageRawDisplay->clear(); // 清除显示的图像
        }
        // 清除IMU数据显示
        if(ui->ori_x && ui->ori_y && ui->ori_z && ui->ori_w && ui->lin_x && ui->lin_y && ui->lin_z && ui->ang_x && ui->ang_y && ui->ang_z) {
            ui->ori_x->clear();
            ui->ori_y->clear();
            ui->ori_z->clear();
            ui->ori_w->clear();
            ui->lin_x->clear();
            ui->lin_y->clear();
            ui->lin_z->clear();
            ui->ang_x->clear();
            ui->ang_y->clear();
            ui->ang_z->clear();
        }
        // 更新状态标签以反映 robot1 未连接
        updateStatusLabel("未连接");
    });
    connect(&dialog, &ConnectDialog::cancelRequested2, this, [this]() {
        qDebug() << "ConnectDialog: robot2 cancel requested";
        if (webSocketWorker2) {
            qDebug() << "请求 worker 关闭 WebSocket 连接 (robot2)...";
            QMetaObject::invokeMethod(webSocketWorker2, "closeConnection", Qt::QueuedConnection);
        }
        if(ui->imageRawDisplay_2) {
            ui->imageRawDisplay_2->clear(); // 清除显示的图像
        }
        // 清除IMU数据显示
        if(ui->ori_x_2 && ui->ori_y_2 && ui->ori_z_2 && ui->ori_w_2 && ui->lin_x_2 && ui->lin_y_2 && ui->lin_z_2 && ui->ang_x_2 && ui->ang_y_2 && ui->ang_z_2) {
            ui->ori_x_2->clear();
            ui->ori_y_2->clear();
            ui->ori_z_2->clear();
            ui->ori_w_2->clear();
            ui->lin_x_2->clear();
            ui->lin_y_2->clear();           
            ui->lin_z_2->clear();
            ui->ang_x_2->clear();
            ui->ang_y_2->clear();
            ui->ang_z_2->clear();
        }
        // 更新状态标签以反映 robot2 未连接
        updateStatusLabel("未连接");
        toggleRobot2Info(false);
    });

    if (dialog.exec() == QDialog::Accepted)
    {
        // 用户点击了连接按钮
        qDebug() << "连接确认";
    }
}

// 语音控制按钮槽函数
void robanweb::onVoiceControlButtonClicked()
{
    QString cmd = loadCmdFromConfig("voiceControlScript");
    if (cmd.isEmpty())
    {
        cmd = "/home/lemon/largeModel.sh";
    }
    if (!cmd.isEmpty())
    {
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
    }
    else
    {
        qDebug() << "Voice control script command is empty in config.";
    }
}

// --- Unified helpers for connect/reconnect ---
void robanweb::startConnectFor(const QString &robotId, const QString &url)
{
    qDebug() << "startConnectFor" << robotId << url;
    if (robotId == QLatin1String("robot1"))
    {
        wsHost = url.split(QLatin1Char(':')).value(1);
        wsPort = url.split(QLatin1Char(':')).last();
        isReconnecting = true;
        reconnectAttempts = 0;
        updateStatusLabel("正在连接...");
        if (multiRobot_mgr1)
        {
            multiRobot_mgr1->startConnect(url);
        }
        else if (webSocketWorker)
        {
            QMetaObject::invokeMethod(webSocketWorker, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
        }
    }
    else if (robotId == QLatin1String("robot2"))
    {
        wsHost2 = url.split(QLatin1Char(':')).value(1);
        wsPort2 = url.split(QLatin1Char(':')).last();
        isReconnecting2 = true;
        reconnectAttempts2 = 0;
        // ensure reconnectTimer2 exists, is connected and started so doReconnectFor() will be invoked on timeout
        if (!reconnectTimer2)
        {
            reconnectTimer2 = new QTimer(this);
            reconnectTimer2->setInterval(5000);
            connect(reconnectTimer2, &QTimer::timeout, this, [this]()
                    { doReconnectFor(QLatin1String("robot2")); });
        }
        // 启动重连定时器（若已经在运行则不会重复启动）
        if (!reconnectTimer2->isActive())
        {
            reconnectTimer2->start();
        }
        updateStatusLabel("正在连接(机器人2)...");
        if (multiRobot_mgr2)
        {
            multiRobot_mgr2->startConnect(url);
        }
        else if (webSocketWorker2)
        {
            QMetaObject::invokeMethod(webSocketWorker2, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
        }
    }
}

void robanweb::doReconnectFor(const QString &robotId)
{
    if (robotId == QLatin1String("robot1"))
    {
        if (!wsHost.isEmpty() && !wsPort.isEmpty() && isReconnecting)
        {
            QString url = QString("ws://%1:%2").arg(wsHost).arg(wsPort);
            qDebug() << "尝试重新连接到 WebSocket:" << url << " (尝试次数:" << reconnectAttempts + 1 << ")";
            reconnectAttempts++;
            if (multiRobot_mgr1)
            {
                multiRobot_mgr1->startConnect(url);
            }
            else if (webSocketWorker)
            {
                QMetaObject::invokeMethod(webSocketWorker, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
            }
            updateStatusLabel(QString("正在重连... (尝试 %1/%2)").arg(reconnectAttempts).arg(MAX_RECONNECT_ATTEMPTS));
        }
        else if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS)
        {
            isReconnecting = false;
            if (reconnectTimer)
                reconnectTimer->stop();
            updateStatusLabel("连接失败：达到最大重试次数");
            qDebug() << "达到最大重试次数，停止重连";
        }
    }
    else if (robotId == QLatin1String("robot2"))
    {
        if (!wsHost2.isEmpty() && !wsPort2.isEmpty() && isReconnecting2)
        {
            QString url = QString("ws://%1:%2").arg(wsHost2).arg(wsPort2);
            qDebug() << "尝试由 MultiRobotManager 重新连接到 第二台 WebSocket:" << url << " (尝试次数:" << reconnectAttempts2 + 1 << ")";
            reconnectAttempts2++;
            if (multiRobot_mgr2)
            {
                multiRobot_mgr2->startConnect(url);
            }
            else if (webSocketWorker2)
            {
                QMetaObject::invokeMethod(webSocketWorker2, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
            }
            updateStatusLabel(QString("正在重连(机器人2)... (尝试 %1/%2)").arg(reconnectAttempts2).arg(MAX_RECONNECT_ATTEMPTS));
        }
        else if (reconnectAttempts2 >= MAX_RECONNECT_ATTEMPTS)
        {
            isReconnecting2 = false;
            if (reconnectTimer2)
                reconnectTimer2->stop();
            updateStatusLabel("机器人2连接失败");
            qDebug() << "机器人2达到最大重试次数，停止重连";
        }
    }
}

// 统一处理来自 MultiRobotManager 的连接成功事件（带 robotId）
void robanweb::onRobotConnected(const QString &robotId)
{
    qDebug() << "Robot connected:" << robotId;
    if (robotId == QLatin1String("robot1"))
    {
        isReconnecting = false;
        if (reconnectTimer)
            reconnectTimer->stop();
        if (multiRobot_mgr1)
            multiRobot_mgr1->startMonitors();
            // 创建 TaskManager（仅当尚未创建且 UI 控件存在时）
            if (multiRobot_mgr1) {
                // 若 manager 暴露 taskManager() 可用来检查是否已创建
                bool needCreate = true;
                #if 1
                // 尝试以安全方式检查是否存在 taskManager() 方法
                if (multiRobot_mgr1->taskManager() != nullptr) {
                    needCreate = false;
                }
                #endif
                if (needCreate) {
                    if (ui->taskListWidget && ui->addTaskButton && ui->runTaskButton && ui->stopTaskButton) {
                        multiRobot_mgr1->createTaskManager(ui->taskListWidget,
                                                           ui->addTaskButton,
                                                           ui->runTaskButton,
                                                           ui->stopTaskButton,
                                                           this);
                    } else {
                        qDebug() << "任务管理器UI组件未找到，未创建 TaskManager (robot1)";
                    }
                }
            }
    }
    else if (robotId == QLatin1String("robot2"))
    {
        isReconnecting2 = false;
        if (reconnectTimer2)
            reconnectTimer2->stop();
        if (multiRobot_mgr2)
            multiRobot_mgr2->startMonitors();
            // 创建 TaskManager（仅当尚未创建且 UI 控件存在时）
            if (multiRobot_mgr2) {
                bool needCreate = true;
                #if 1
                if (multiRobot_mgr2->taskManager() != nullptr) {
                    needCreate = false;
                }
                #endif
                if (needCreate) {
                    if (ui->taskListWidget_2 && ui->addTaskButton_2 && ui->runTaskButton_2 && ui->stopTaskButton_2) {
                        multiRobot_mgr2->createTaskManager(ui->taskListWidget_2,
                                                           ui->addTaskButton_2,
                                                           ui->runTaskButton_2,
                                                           ui->stopTaskButton_2,
                                                           this);
                    } else {
                        qDebug() << "任务管理器UI组件未找到，未创建 TaskManager (robot2)";
                    }
                }
            }
        toggleRobot2Info(true);
    }

    // 每台机器人由其各自的 MultiRobotManager 管理图像拉取，允许同时显示多台机器人画面。
    if (robotId == QLatin1String("robot1")) {
        if (multiRobot_mgr1) multiRobot_mgr1->startImagePulling();
    } else if (robotId == QLatin1String("robot2")) {
        if (multiRobot_mgr2) multiRobot_mgr2->startImagePulling();
    }
    else if (multiRobot_mgr2 && multiRobot_mgr2->isConnected())
    {
        multiRobot_mgr2->startImagePulling();
    }

    updateStatusLabel(QString());
}

// 统一处理断线事件
void robanweb::onRobotDisconnected(const QString &robotId)
{
    qDebug() << "Robot disconnected:" << robotId;
    if (robotId == QLatin1String("robot1"))
    {
        if (isReconnecting)
        {
            if (reconnectTimer)
                reconnectTimer->start();
        }
        if (multiRobot_mgr1)
            multiRobot_mgr1->stopImagePulling();
        // 清除 robot1 的图像与 IMU 显示
        if (ui->imageRawDisplay) {
            ui->imageRawDisplay->clear();
        }
        if(ui->ori_x && ui->ori_y && ui->ori_z && ui->ori_w && ui->lin_x && ui->lin_y && ui->lin_z && ui->ang_x && ui->ang_y && ui->ang_z) {
            ui->ori_x->clear();
            ui->ori_y->clear();
            ui->ori_z->clear();
            ui->ori_w->clear();
            ui->lin_x->clear();
            ui->lin_y->clear();
            ui->lin_z->clear();
            ui->ang_x->clear();
            ui->ang_y->clear();
            ui->ang_z->clear();
        }
    }
    else if (robotId == QLatin1String("robot2"))
    {
        if (isReconnecting2)
        {
            if (reconnectTimer2)
                reconnectTimer2->start();
        }
        if (multiRobot_mgr2)
            multiRobot_mgr2->stopImagePulling();
        // 清除 robot2 的图像与 IMU 显示
        if (ui->imageRawDisplay_2) {
            ui->imageRawDisplay_2->clear();
        }
        if(ui->ori_x_2 && ui->ori_y_2 && ui->ori_z_2 && ui->ori_w_2 && ui->lin_x_2 && ui->lin_y_2 && ui->lin_z_2 && ui->ang_x_2 && ui->ang_y_2 && ui->ang_z_2) {
            ui->ori_x_2->clear();
            ui->ori_y_2->clear();
            ui->ori_z_2->clear();
            ui->ori_w_2->clear();
            ui->lin_x_2->clear();
            ui->lin_y_2->clear();
            ui->lin_z_2->clear();
            ui->ang_x_2->clear();
            ui->ang_y_2->clear();
            ui->ang_z_2->clear();
        }
        toggleRobot2Info(false);
    }
    // 断开连接时同时清理 TaskManager 与 UI 任务列表（仅清空对应机器人的列表）
    if (robotId == QLatin1String("robot1")) {
        if (multiRobot_mgr1) {
            multiRobot_mgr1->closeTaskManager();
        }
        if (ui->taskListWidget) ui->taskListWidget->clear();
    } else if (robotId == QLatin1String("robot2")) {
        if (multiRobot_mgr2) {
            multiRobot_mgr2->closeTaskManager();
        }
        if (ui->taskListWidget_2) ui->taskListWidget_2->clear();
    }
    updateStatusLabel(QString());
}

// 统一处理错误事件
void robanweb::onRobotError(const QString &robotId, const QString &error)
{
    qDebug() << "Robot error(" << robotId << "):" << error;
    if (robotId == QLatin1String("robot1"))
    {
        if (isReconnecting && reconnectTimer)
            reconnectTimer->start();
        updateStatusLabel(QString("机器人1连接错误：%1").arg(error));
    }
    else if (robotId == QLatin1String("robot2"))
    {
        if (isReconnecting2 && reconnectTimer2)
            reconnectTimer2->start();
        updateStatusLabel(QString("机器人2连接错误：%1").arg(error));
    }
}

void robanweb::updateStatusLabel(const QString &status)
{
    Q_UNUSED(status);
    // 更新每个机器人对应的状态标签（支持同时显示两台机器人的状态）
    if (connect_label1)
    {
        QString text;
        if (multiRobot_mgr1 && multiRobot_mgr1->isConnected())
        {
            text = QStringLiteral("【robot1】已连接");
        }
        else if (isReconnecting)
        {
            text = QStringLiteral("【robot1】连接断开，正在重连...");
        }
        else
        {
            text = QStringLiteral("【robot1】未连接");
        }
        connect_label1->setText(text);
    }

    if (connect_label2)
    {
        QString text;
        if (multiRobot_mgr2 && multiRobot_mgr2->isConnected())
        {
            text = QStringLiteral("【robot2】已连接");
        }
        else if (isReconnecting2)
        {
            text = QStringLiteral("【robot2】连接断开，正在重连...");
        }
        else
        {
            text = QStringLiteral("【robot2】未连接");
        }
        connect_label2->setText(text);
    }
}

void robanweb::closeEvent(QCloseEvent *event)
{
    isReconnecting = false;
    reconnectTimer->stop();
    // 确保在关闭应用前停止 SLAM
    if (m_slamDialog)
    {
        // 跨线程调用停止 SLAM 方法
        if (m_slamDialog->thread() == QThread::currentThread())
        {
            m_slamDialog->stopSLAM();
        }
        else
        {
            QMetaObject::invokeMethod(m_slamDialog, "stopSLAM", Qt::BlockingQueuedConnection);
        }
    }
    if (webSocketWorker)
    {
        QMetaObject::invokeMethod(webSocketWorker, "closeConnection", Qt::QueuedConnection);
        webSocketThread->quit();
        webSocketThread->wait();
        delete webSocketThread;
        webSocketThread = nullptr;
        webSocketWorker = nullptr; // worker deleted when thread finishes (connected earlier)
    }
    if (webSocketWorker2)
    {
        QMetaObject::invokeMethod(webSocketWorker2, "closeConnection", Qt::QueuedConnection);
        webSocketThread2->quit();
        webSocketThread2->wait();
        delete webSocketThread2;
        webSocketThread2 = nullptr;
        webSocketWorker2 = nullptr; // worker deleted when thread finishes (connected earlier)
    }

    // Close both managers (they will close their workers if needed)
    if (multiRobot_mgr1)
    {
        multiRobot_mgr1->close();
    }
    if (multiRobot_mgr2)
    {
        multiRobot_mgr2->close();
    }

    if (imagePullTimer)
    {
        imagePullTimer->stop();
        delete imagePullTimer;
        imagePullTimer = nullptr;
    }

    event->accept();
}

bool robanweb::eventFilter(QObject *watched, QEvent *event)
{
    if (event->type() == QEvent::Resize) {
        if (watched == ui->imageRawDisplay) {
            QSize newSize = ui->imageRawDisplay->size();
            if (multiRobot_mgr1) {
                QMetaObject::invokeMethod(multiRobot_mgr1, "setCameraTargetSize", Qt::QueuedConnection, Q_ARG(QSize, newSize));
            }
        } else if (watched == ui->imageRawDisplay_2) {
            QSize newSize = ui->imageRawDisplay_2->size();
            if (multiRobot_mgr2) {
                QMetaObject::invokeMethod(multiRobot_mgr2, "setCameraTargetSize", Qt::QueuedConnection, Q_ARG(QSize, newSize));
            }
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

void robanweb::toggleRobot2Info(bool show)
{
    if (!ui->groupBox_robotInfor2 || !robot2Animation) return;

    if (show) {
        // 如果已经在显示或动画正在进行（且是显示方向），则忽略
        if (ui->groupBox_robotInfor2->isVisible() && ui->groupBox_robotInfor2->maximumWidth() > 0 && robot2Animation->state() == QAbstractAnimation::Stopped) {
             return;
        }
        
        robot2Animation->stop();
        
        // 如果当前是隐藏的，先设置最大宽度为0并显示
        if (!ui->groupBox_robotInfor2->isVisible()) {
             ui->groupBox_robotInfor2->setMaximumWidth(0);
             ui->groupBox_robotInfor2->setVisible(true);
        }
        
        int startWidth = ui->groupBox_robotInfor2->width();
        // 目标宽度：参考 robot1 的宽度，或者给一个默认值
        int targetWidth = (ui->groupBox_robotInfor1 && ui->groupBox_robotInfor1->isVisible()) ? ui->groupBox_robotInfor1->width() : 350;
        if (targetWidth < 100) targetWidth = 350;

        robot2Animation->setDuration(500);
        robot2Animation->setStartValue(startWidth);
        robot2Animation->setEndValue(targetWidth);
        robot2Animation->setEasingCurve(QEasingCurve::OutCubic);
        
        disconnect(robot2Animation, &QPropertyAnimation::finished, nullptr, nullptr);
        connect(robot2Animation, &QPropertyAnimation::finished, this, [this](){
            // 动画结束后恢复最大宽度限制，以便布局自动调整
            ui->groupBox_robotInfor2->setMaximumWidth(QWIDGETSIZE_MAX);
        });
        
        robot2Animation->start();
    } else {
        if (!ui->groupBox_robotInfor2->isVisible()) return;

        robot2Animation->stop();
        robot2Animation->setDuration(500);
        robot2Animation->setStartValue(ui->groupBox_robotInfor2->width());
        robot2Animation->setEndValue(0);
        robot2Animation->setEasingCurve(QEasingCurve::OutCubic);
        
        disconnect(robot2Animation, &QPropertyAnimation::finished, nullptr, nullptr);
        connect(robot2Animation, &QPropertyAnimation::finished, this, [this](){
            ui->groupBox_robotInfor2->setVisible(false);
            ui->groupBox_robotInfor2->setMaximumWidth(QWIDGETSIZE_MAX); 
        });
        
        robot2Animation->start();
    }
}

QString robanweb::resolveAssetPath(const QString &relPath)
{
    QString appDir = QCoreApplication::applicationDirPath();
    QString curDir = QDir::currentPath();
    QStringList candidates;
    // common locations relative to executable
    candidates << QDir::cleanPath(appDir + QDir::separator() + relPath);
    candidates << QDir::cleanPath(appDir + QDir::separator() + ".." + QDir::separator() + relPath);
    candidates << QDir::cleanPath(appDir + QDir::separator() + ".." + QDir::separator() + ".." + QDir::separator() + relPath);
    // current working directory options (useful when running from IDE)
    candidates << QDir::cleanPath(curDir + QDir::separator() + relPath);
    candidates << QDir::cleanPath(curDir + QDir::separator() + ".." + QDir::separator() + relPath);

    for (const QString &p : candidates) {
        if (QFile::exists(p)) {
            qDebug() << "Found asset:" << p;
            return p;
        }
    }
    // 如果没有找到，返回第一个候选（可用于观察失败路径）
    qWarning() << "Asset not found in candidate locations, using fallback:" << candidates.first();
    return candidates.first();
}
