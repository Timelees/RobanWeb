#pragma once

#include <QObject>
#include <QMap>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QString>
#include <QImage>
#include <QSize>
#include <QListWidget>
#include <QPushButton>
// forward declare monitor types
class ImuMonitor;
class CameraImageMonitor;
class ServoPositionsMonitor;
class PoseMonitor;
class GaitCommandMonitor;

class WebSocketWorker;
class TaskManager;
class Task; // Forward declaration of Task class

class MultiRobotManager : public QObject {
    Q_OBJECT
public:
    // 构造函数：可传入已创建并移动到线程的 WebSocketWorker（非拥有权），
    // 或传入 nullptr 让 manager 自行创建 worker/thread
    explicit MultiRobotManager(WebSocketWorker *worker, const QString &robotId, QObject *parent = nullptr);
    ~MultiRobotManager();

    // 开始为指定 robotId 建立连接（如果不存在会创建 worker/thread）
    // 如果 manager 未拥有 worker（即通过构造传入的 worker），可通过此接口发起连接
    void startConnect(const QString &url);
    // 关闭该 robot 的连接
    void close();
    // 请求相机抓取一帧（UI 的 imagePullTimer 调用）
    Q_INVOKABLE void requestFrame();
    // 摄像头参数控制
    Q_INVOKABLE void setCameraTargetSize(const QSize &sz);
    Q_INVOKABLE void setCameraMaxFps(int fps);
    // 启动内部 monitors（订阅 topic）
    Q_INVOKABLE void startMonitors();

    // 添加 SceneManager 和 RobotManager，以便 manager 在准备好时注入监视器指针。
    void attachManagers(class SceneManager *sceneMgr, class RobotManager *robotMgr);

    // Image pull control: manager can own a periodic timer to call
    // requestFrame(); callers can adjust interval or enable/disable pulling.
    Q_INVOKABLE void setImagePullInterval(int ms);
    Q_INVOKABLE void startImagePulling();
    Q_INVOKABLE void stopImagePulling();

    // 查询当前连接状态（线程安全）
    bool isConnected() const;

    // TaskManager 管理接口：由 MultiRobotManager 创建并管理 TaskManager 的生命周期
    // 将 UI 控件交给 manager，让其创建任务管理器并负责与 worker 的连接
    void createTaskManager(QListWidget* taskListWidget,
                           QPushButton* addTaskButton,
                           QPushButton* runTaskButton,
                           QPushButton* stopTaskButton,
                           QObject* parent = nullptr);
    void closeTaskManager();
    TaskManager* taskManager() const;

    // getters for monitors so caller can attach UI or pass to other subsystems
    ServoPositionsMonitor* servoPositionsMonitor() const { return m_servoPositionsMonitor; }
    PoseMonitor* poseMonitor() const { return m_poseMonitor; }
    GaitCommandMonitor* gaitCommandMonitor() const { return m_gaitCommandMonitor; }
    ImuMonitor* imuMonitor() const { return m_imuMonitor; }
    CameraImageMonitor* cameraImageMonitor() const { return m_cameraImageMonitor; }

signals:
    void robotConnected(const QString &robotId);
    void robotDisconnected(const QString &robotId);
    void robotError(const QString &robotId, const QString &error);
    // 转发监视器信号给上层 UI
    void imageReceived(const QImage &img);
    void imuOrientationUpdated(double w, double x, double y, double z);
    void imuAngularVelocityUpdated(double x, double y, double z);
    void imuLinearAccelerationUpdated(double x, double y, double z);
    // 转发 TaskManager 的信号，便于上层 UI 直接连接到 manager
    void taskAdded(Task* task);
    void taskRemoved(int index);
    void taskSelected(Task* task);
    void taskExecuted(const QString &scriptPath);
    void taskStopped(const QString &scriptPath);

public:
    // 单 robot 管理结构（每个实例管理一台机器人）
    WebSocketWorker *m_worker = nullptr; // 非拥有者或拥有者取决于构造参数
    QThread *m_thread = nullptr;         // 若 manager 创建 worker，则创建 thread 并拥有
    QTimer *m_reconnectTimer = nullptr;
    QString m_robotId;
    bool m_ownsWorker = false;

private:
    

    // Monitors
    ImuMonitor *m_imuMonitor = nullptr;
    CameraImageMonitor *m_cameraImageMonitor = nullptr;
    QThread *m_cameraThread = nullptr;
    ServoPositionsMonitor *m_servoPositionsMonitor = nullptr;
    PoseMonitor *m_poseMonitor = nullptr;
    GaitCommandMonitor *m_gaitCommandMonitor = nullptr;

    // Optional pointers to scene/robot managers so the MultiRobotManager can
    // inject monitors after those managers are constructed.
    class SceneManager *m_sceneManager = nullptr;
    class RobotManager *m_robotManager = nullptr;

    // Manager-owned image pull timer (periodically calls requestFrame())
    QTimer *m_imagePullTimer = nullptr;
    
        // Optional TaskManager owned by this manager when created via createTaskManager
        TaskManager *m_taskManager = nullptr;

    mutable QMutex m_mutex;
};
