#include "manager/multiRobotManager.h"
#include "socket_process/websocketworker.h"
#include "ros_process/imu.h"
#include "ros_process/cameraImage.h"
#include "ros_process/servoPositions.h"
#include "ros_process/slamPose.h"
#include "ros_process/gaitCommand.h"
#include "util/load_param.hpp"
#include <QMetaObject>
#include <QUrl>
#include <QDebug>

#include "model_display/sceneManager.h"
#include "model_display/robotManager.h"
#include "manager/taskmanager.h"

MultiRobotManager::MultiRobotManager(WebSocketWorker *worker, const QString &robotId, QObject *parent)
    : QObject(parent), m_worker(nullptr), m_thread(nullptr), m_reconnectTimer(nullptr), m_robotId(robotId), m_ownsWorker(false),
      m_imuMonitor(nullptr), m_cameraImageMonitor(nullptr), m_servoPositionsMonitor(nullptr), m_poseMonitor(nullptr), m_gaitCommandMonitor(nullptr)
{
    // 设置robanweb中创建传入的worker
    if (worker) {
        m_worker = worker;
        qDebug() << robotId << ": Using provided WebSocketWorker: " << worker << "in Thread " << m_worker->thread();
        m_ownsWorker = false;
    } else {
        // 传入的websocketWorker为空，则创建新的worker和线程
        m_worker = new WebSocketWorker();
        m_thread = new QThread(this);
        m_worker->moveToThread(m_thread);
        m_thread->start();
        connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
        m_ownsWorker = true;
    }

    // 通过worker发送通信连接相关的信号
    connect(m_worker, &WebSocketWorker::connected, this, [this]() { emit robotConnected(m_robotId); });
    connect(m_worker, &WebSocketWorker::disconnected, this, [this]() { emit robotDisconnected(m_robotId); });
    connect(m_worker, &WebSocketWorker::errorOccurred, this, [this](const QString &err){ emit robotError(m_robotId, err); });

    // 创建绑定到此worker的监视器
    // parent设置为this，以便由管理器管理生命周期
    m_imuMonitor = new ImuMonitor(m_worker, this);
    m_servoPositionsMonitor = new ServoPositionsMonitor(m_worker, this);
    m_poseMonitor = new PoseMonitor(m_worker, this);
    m_gaitCommandMonitor = new GaitCommandMonitor(m_worker, this);

    // CameraImageMonitor 需要单独线程处理图像解码等工作，避免阻塞 websocket 线程
    QString camera_topic = loadTopicFromConfig("cameraCompressed_topic");
    m_cameraImageMonitor = new CameraImageMonitor(m_worker, nullptr, camera_topic);
    m_cameraThread = new QThread(this);
    m_cameraImageMonitor->moveToThread(m_cameraThread);
    m_cameraThread->start();
    connect(m_cameraThread, &QThread::finished, m_cameraImageMonitor, &QObject::deleteLater);

    // image pull timer controlled by the manager (UI need not own it)
    m_imagePullTimer = new QTimer(this);
    m_imagePullTimer->setInterval(50); // default ~20 FPS
    connect(m_imagePullTimer, &QTimer::timeout, this, [this]() { requestFrame(); });

    // 转发监视器信号到上层
    connect(m_cameraImageMonitor, &CameraImageMonitor::imageReceived, this, &MultiRobotManager::imageReceived, Qt::QueuedConnection);
    connect(m_imuMonitor, &ImuMonitor::orientationUpdated, this, &MultiRobotManager::imuOrientationUpdated, Qt::QueuedConnection);
    connect(m_imuMonitor, &ImuMonitor::angularVelocityUpdated, this, &MultiRobotManager::imuAngularVelocityUpdated, Qt::QueuedConnection);
    connect(m_imuMonitor, &ImuMonitor::linearAccelerationUpdated, this, &MultiRobotManager::imuLinearAccelerationUpdated, Qt::QueuedConnection);

    // TaskManager will be created on demand by createTaskManager();
    m_taskManager = nullptr;

    // 连接 worker 的 messageReceived 信号到各个 monitor 的 onMessageReceived 槽，处理数据
    if (m_worker) {
        connect(m_worker, &WebSocketWorker::messageReceived, m_imuMonitor, &ImuMonitor::onMessageReceived, Qt::QueuedConnection);
        connect(m_worker, &WebSocketWorker::messageReceived, m_servoPositionsMonitor, &ServoPositionsMonitor::onMessageReceived, Qt::QueuedConnection);
        connect(m_worker, &WebSocketWorker::messageReceived, m_poseMonitor, &PoseMonitor::onMessageReceived, Qt::QueuedConnection);
        connect(m_worker, &WebSocketWorker::messageReceived, m_gaitCommandMonitor, &GaitCommandMonitor::onMessageReceived, Qt::QueuedConnection);
        connect(m_worker, &WebSocketWorker::messageReceived, m_cameraImageMonitor, &CameraImageMonitor::onMessageReceived, Qt::QueuedConnection);
    }
}

MultiRobotManager::~MultiRobotManager()
{
    QMutexLocker locker(&m_mutex);
    if (m_worker) {
        QMetaObject::invokeMethod(m_worker, "closeConnection", Qt::QueuedConnection);
    }
    if (m_thread) {
        m_thread->quit();
        m_thread->wait();
        delete m_thread;
        m_thread = nullptr;
    }
    if (m_cameraThread) {
        m_cameraThread->quit();
        m_cameraThread->wait();
        delete m_cameraThread;
        m_cameraThread = nullptr;
    }
    if (m_reconnectTimer) {
        m_reconnectTimer->stop();
        delete m_reconnectTimer;
        m_reconnectTimer = nullptr;
    }
    if (m_imagePullTimer) {
        m_imagePullTimer->stop();
        delete m_imagePullTimer;
        m_imagePullTimer = nullptr;
    }
    // monitors are children and will be deleted automatically
}
// 开始为指定 robotId 建立连接
void MultiRobotManager::startConnect(const QString &url)
{
    if (!m_worker) return;
    QMetaObject::invokeMethod(m_worker, "startConnect", Qt::QueuedConnection, Q_ARG(QString, url));
}
// 关闭该 robot 的连接
void MultiRobotManager::close()
{
    if (m_worker) {
        QMetaObject::invokeMethod(m_worker, "closeConnection", Qt::QueuedConnection);
    }
}
// 请求相机抓取一帧（UI 的 imagePullTimer 调用）
void MultiRobotManager::requestFrame()
{
    if (m_cameraImageMonitor) {
        QMetaObject::invokeMethod(m_cameraImageMonitor, "requestFrame", Qt::QueuedConnection);
    }
}

// 设置相机目标尺寸
void MultiRobotManager::setCameraTargetSize(const QSize &sz)
{
    if (m_cameraImageMonitor) {
        QMetaObject::invokeMethod(m_cameraImageMonitor, "setTargetSize", Qt::QueuedConnection, Q_ARG(QSize, sz));
    }
}
// 设置相机最大帧率
void MultiRobotManager::setCameraMaxFps(int fps)
{
    if (m_cameraImageMonitor) {
        QMetaObject::invokeMethod(m_cameraImageMonitor, "setMaxFps", Qt::QueuedConnection, Q_ARG(int, fps));
    }
}
//  启动内部 monitors（订阅 topic）
void MultiRobotManager::startMonitors()
{
    if (m_imuMonitor) QMetaObject::invokeMethod(m_imuMonitor, "start", Qt::QueuedConnection);
    if (m_cameraImageMonitor) QMetaObject::invokeMethod(m_cameraImageMonitor, "start", Qt::QueuedConnection);
    if (m_servoPositionsMonitor) QMetaObject::invokeMethod(m_servoPositionsMonitor, "start", Qt::QueuedConnection);
    if (m_poseMonitor) QMetaObject::invokeMethod(m_poseMonitor, "start", Qt::QueuedConnection);
    if (m_gaitCommandMonitor) QMetaObject::invokeMethod(m_gaitCommandMonitor, "start", Qt::QueuedConnection);
    // start manager-owned image pull timer so UI need not create one
    if (m_imagePullTimer && !m_imagePullTimer->isActive()) {
        m_imagePullTimer->start();
    }
}

void MultiRobotManager::attachManagers(SceneManager *sceneMgr, RobotManager *robotMgr)
{
    QMutexLocker locker(&m_mutex);
    m_sceneManager = sceneMgr;
    m_robotManager = robotMgr;
    // inject monitors into provided managers (connect and set pointers)
    if (m_sceneManager && m_poseMonitor) {
        m_sceneManager->setPoseMonitor(m_poseMonitor);
    }
    if (m_robotManager) {
        if (m_servoPositionsMonitor) m_robotManager->setServoPositionsMonitor(m_servoPositionsMonitor);
        if (m_gaitCommandMonitor) m_robotManager->setGaitCommandMonitor(m_gaitCommandMonitor);
    }
}

void MultiRobotManager::setImagePullInterval(int ms)
{
    if (m_imagePullTimer) m_imagePullTimer->setInterval(ms);
}

void MultiRobotManager::startImagePulling()
{
    if (m_imagePullTimer && !m_imagePullTimer->isActive()) m_imagePullTimer->start();
}

void MultiRobotManager::stopImagePulling()
{
    if (m_imagePullTimer && m_imagePullTimer->isActive()) m_imagePullTimer->stop();
}

bool MultiRobotManager::isConnected() const
{
    QMutexLocker locker(&m_mutex);
    if (!m_worker) return false;
    // WebSocketWorker::isConnected() is a lightweight query; it's safe to call here.
    return m_worker->isConnected();
}

void MultiRobotManager::createTaskManager(QListWidget* taskListWidget,
                                         QPushButton* addTaskButton,
                                         QPushButton* runTaskButton,
                                         QPushButton* stopTaskButton,
                                         QObject* parent)
{
    QMutexLocker locker(&m_mutex);
    if (m_taskManager) {
        // already created
        return;
    }
    // create TaskManager as child of this manager so it will be deleted with the manager
    m_taskManager = new TaskManager(taskListWidget, addTaskButton, runTaskButton, stopTaskButton, m_worker, parent ? parent : this);
    // forward TaskManager signals
    connect(m_taskManager, SIGNAL(taskAdded(Task*)), this, SIGNAL(taskAdded(Task*)), Qt::QueuedConnection);
    connect(m_taskManager, SIGNAL(taskRemoved(int)), this, SIGNAL(taskRemoved(int)), Qt::QueuedConnection);
    connect(m_taskManager, SIGNAL(taskSelected(Task*)), this, SIGNAL(taskSelected(Task*)), Qt::QueuedConnection);
    connect(m_taskManager, SIGNAL(taskExecuted(const QString&)), this, SIGNAL(taskExecuted(const QString&)), Qt::QueuedConnection);
    connect(m_taskManager, SIGNAL(taskStopped(const QString&)), this, SIGNAL(taskStopped(const QString&)), Qt::QueuedConnection);
    // optionally load saved tasks
    m_taskManager->loadTasksFromConfig();
}

void MultiRobotManager::closeTaskManager()
{
    QMutexLocker locker(&m_mutex);
    if (m_taskManager) {
        delete m_taskManager;
        m_taskManager = nullptr;
    }
}

TaskManager* MultiRobotManager::taskManager() const
{
    return m_taskManager;
}
