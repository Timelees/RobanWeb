#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H


#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QFileInfo>
#include <QImage>
#include <QDebug>
#include <QMatrix4x4>
#include <QVector>
#include <QVector3D>
#include <QVector2D>
#include <QColor>
#include <QDir>
#include <QByteArray>
#include <QPainter>
#include <QThread>
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "model_display/meshes.h"
#include "socket_process/webSocketWorker.h"
#include "ros_process/slamPose.h"

class WebSocketWorker;

// ---------- SceneManager ----------
// 负责加载场景模型（静态），提供 meshes 给视图渲染
class SceneManager : public QObject
{
    Q_OBJECT
public:
    explicit SceneManager(PoseMonitor *poseMonitor, const QString &modelPath, QObject *parent = nullptr);

    ~SceneManager() override;

    void init();                             // 初始化

    bool loadModel(const std::string &file);        // 加载模型
    void createGridMesh(float size = 20.0f, int divisions = 20, float yOffset = 0.0f);      // 创建网格地面平面

    std::vector<SimpleMesh> &meshes() { return m_meshes; }
    const std::vector<SimpleMesh> &meshes() const { return m_meshes; }
    bool loaded() const { return m_loaded; }

    // Ray pick against loaded meshes. rayOrigin and rayDir are in model space.
    // Returns true and sets outHit (model-space) if an intersection was found.
    bool pickIntersect(const QVector3D &rayOrigin, const QVector3D &rayDir, QVector3D &outHit) const;

    // 标记类型
    enum MarkerType { Marker_Map = 0, Marker_Pickup = 1, Marker_Place = 2 };

    // 标记数据结构（由 SceneManager 管理）
    struct Marker
    {
        int id = -1;               // 唯一 id
        MarkerType type = Marker_Map;
        QVector3D pos;            // 模型空间坐标
        float radius = 0.05f;     // 可视半径（模型单位）
        int meshIndex = -1;       // 对应 m_meshes 中的索引
        QColor color;             // 显示颜色
        // 记录添加该标记时采集到的机器人位姿 (x,y,yaw)
        QVector3D robotPoseAtCapture;
    };

    // 添加一个标记（会在内部创建球体 mesh 并返回分配的 marker id）
    int addCalibrationMarker(MarkerType type, const QVector3D &pos, float radius = 0.05f, const QColor &color = QColor(0, 255, 0));

    // 删除指定 id 的标记（返回是否成功）
    bool removeMarkerById(int id);

    // 根据射线拾取已存在的标记，找到最近的一个，返回 marker id 或 -1
    int pickMarkerByRay(const QVector3D &rayOrigin, const QVector3D &rayDir, QVector3D &outHit) const;

    // 保存/加载标记到磁盘（默认保存在模型目录下的 calib_points.json）
    bool saveMarkers(const QString &path = QString());
    bool loadMarkers(const QString &path = QString());

    // 删除所有标记（移除所有标记数据，并清空对应 mesh 的几何数据）
    void clearAllMarkers();

    void SceneMapping(); // 场景映射函数


    // 测试用：从 CSV 读取一组机器人位姿并解析（保留为成员以便测试）
    QVector<QVector3D> test_sceneCornerMapping();

    // 将场景空间坐标（模型空间）映射到机器人坐标（x,y）。
    // 返回 true 表示已有映射并成功计算 outRobot。
    bool mapSceneToRobot(const QVector3D &scenePt, QVector2D &outRobot) const;

    // 将机器人坐标 (x,y) 映射回场景坐标 (模型空间)。
    // 返回 true 表示已有映射且计算成功。outScene 的 y 分量为场景高度（目前设置为 0，场景为平面，不用适配）。
    bool mapRobotToScene(const QVector2D &robotPt, QVector3D &outScene) const;

    // Convenience: map the current robot pose (as received from PoseMonitor)
    // directly to scene coordinates. Returns false if mapping is not available
    // or robot pose is not valid.
    bool mapCurrentRobotPoseToScene(QVector3D &outScene) const;

    // Allow setting or changing the PoseMonitor after construction. When set,
    // SceneManager will subscribe to pose updates from the provided monitor.
    void setPoseMonitor(PoseMonitor *poseMonitor);

    // // Configuration: adjust display smoothing and throttle at runtime
    // Q_INVOKABLE void setDisplayIntervalMs(int ms) { m_displayIntervalMs = ms; if (m_displayTimer) m_displayTimer->setInterval(ms); }
    // int displayIntervalMs() const { return m_displayIntervalMs; }
    // Q_INVOKABLE void setDisplayLerpFactor(float f) { m_displayLerpFactor = f; }
    // float displayLerpFactor() const { return m_displayLerpFactor; }
    // Q_INVOKABLE void setPoseUpdateThrottle(int n) { m_poseUpdateThrottle = n; }
    // int poseUpdateThrottle() const { return m_poseUpdateThrottle; }

    // Position acceptance thresholds: when walking stops, only accept the
    // most-recent SLAM pose as the new stable robot pose if the difference
    // vs the previously-stable pose is within these thresholds.
    // Q_INVOKABLE void setAcceptPositionThresholdMeters(float m) { m_acceptPositionThresholdMeters = m; }
    // float acceptPositionThresholdMeters() const { return m_acceptPositionThresholdMeters; }
    // Q_INVOKABLE void setAcceptYawThresholdDegrees(float d) { m_acceptYawThresholdDeg = d; }
    // float acceptYawThresholdDegrees() const { return m_acceptYawThresholdDeg; }
    // Q_INVOKABLE void setAcceptPoseOnStopEnabled(bool e) { m_acceptPoseOnStopEnabled = e; }
    // bool acceptPoseOnStopEnabled() const { return m_acceptPoseOnStopEnabled; }

public slots:
    // 通知 SceneManager 当前机器人是否处于行走状态（true=行走，false=静止）
    // 当从行走切换到静止（false）时，SceneManager 会把最近收到的稳定位姿应用到 robotPose 并发出 robotPoseUpdated
    // void setRobotWalking(bool walking);


    const std::vector<Marker> &markers() const { return m_markers; }

signals:
    // 当 SceneManager 收到新的机器人位姿时发出（x,y,yaw）
    // void robotPoseUpdated(const QVector3D &pose);
    // 每次收到原始位姿（未节流）时发出。动画子系统应监听此信号以保持动画与实际运动同步。
    // 例如：腿部步态/骨骼动画可以使用此频繁的信号驱动，而不受位置更新节流影响。
    // void robotPoseAnimationUpdated(const QVector3D &pose);
    // 用于渲染层的平滑显示位姿：SceneManager 内部会把节流后的目标位姿平滑插值到一个显示位姿并
    // 周期性发出此信号以驱动视图位置过渡（频率可配置，默认 ~60Hz）。渲染层应监听此信号以获得平滑位置。
    // void robotPoseDisplayUpdated(const QVector3D &pose);

private:
    // 内部函数：创建球体 mesh 并返回 mesh 索引（仅 SceneManager 内部使用）
    int addMarkerSphere(const QVector3D &pos, float radius = 0.05f, const QColor &color = QColor(0, 255, 0));
    // 加载场景与机器人坐标仿射参数
    void loadSceneMappingParameters();

private:
    QString m_modelPath;
    std::vector<SimpleMesh> m_meshes;
    bool loadSucceeded = false;
    bool m_loaded = false;
    // 内部标记存储
    std::vector<Marker> m_markers;
    int m_nextMarkerId = 1;



    PoseMonitor *m_poseMonitor;       // 位姿监视器
    QVector3D robotPose;            // 机器人当前位姿
    // 当前用于渲染/显示的插值位姿（在内部用定时器从 m_displayRobotPose 向 robotPose 平滑过渡）
    // QVector3D m_displayRobotPose;
    // QTimer *m_displayTimer = nullptr; // 用于推进平滑插值的定时器 (已注释)
    // int m_displayIntervalMs = 16;     // 默认 16ms -> ~60Hz 的显示更新 (已注释)
    // float m_displayLerpFactor = 0.15f; // 每帧插值因子(0..1)，越大过渡越快 (已注释)
    // 节流：接收到外部位姿更新时不必每次都通知渲染/其他模块
    // m_poseUpdateCounter 累计收到的更新次数，达到 m_poseUpdateThrottle 时才把收到的位姿写入 robotPose 并发出信号
    int m_poseUpdateCounter = 0;
    int m_poseUpdateThrottle = 20; // 默认每 20 条更新一次（可按需调整）
    QVector3D m_lastReceivedPose;   // 最近一次收到但尚未转发的位姿
    // 是否正在行走（来自 ServoPositionsMonitor walking status）
    bool m_isWalking = false;
    // 在非行走状态下用于记录稳定位置的副本（当 walking 停止时用于立即应用）
    QVector3D m_lastStablePose;
    // 接收slam位置数据作为稳定位置的阈值，避免位置因为slam位置的突变导致模型位置突变
    float m_acceptPositionThresholdMeters = 0.10f; // default 30 cm
    float m_acceptYawThresholdDeg = 10.0f;         // default 30 degrees
    bool m_acceptPoseOnStopEnabled = true;         // enable acceptance check by default

    // 场景映射测试代码
    QVector<QVector3D>  cornerPoints;   // 场景的四个角点
    QVector<QVector3D>  posePoints;     // 角点对应的机器人位姿

    // 由 SceneMapping 计算并保留的仿射映射参数: scene (X,Z) -> robot (x,y)
    bool m_hasMapping = false;
    double m_map_a = 1.0, m_map_b = 0.0, m_map_tx = 0.0;
    double m_map_c = 0.0, m_map_d = 1.0, m_map_ty = 0.0;

    
};


#endif // SCENEMANAGER_H