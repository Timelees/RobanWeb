#ifndef MODEL_DISPLAY_ROBOTMANAGER_H
#define MODEL_DISPLAY_ROBOTMANAGER_H

#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QObject>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QPointer>
#include <QMutex>
#include <QFileInfo>
#include <QImage>
#include <QDebug>
#include <QMatrix4x4>
#include <QDir>
#include <QByteArray>
#include <QPainter>
#include <QVector3D>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "model_display/meshes.h"
#include "ros_process/servoPositions.h"
#include "socket_process/webSocketWorker.h"
#include "model_display/sceneManager.h"
#include "ros_process/imu.h"
#include "ros_process/gaitCommand.h"

class WebSocketWorker;

// ---------- RobotManager ----------
// 负责加载机器人模型（带骨骼动画），提供 meshes 给视图
class RobotManager : public QObject
{
    Q_OBJECT
public:
    explicit RobotManager(const QString &modelPath, SceneManager *sceneManager, ServoPositionsMonitor *servoPositionsMonitor, QObject *parent = nullptr);
    ~RobotManager() override = default;

    // 绑定 IMU 数据源：外部可创建 ImuMonitor 实例并通过该方法连接到 RobotManager
    void attachImuMonitor(ImuMonitor *imu);


    bool loadBoneJointMapping(const QString &csvPath);
    bool loadActionCsv(const QString &csvPath, int intervalMs = 100, bool loop = true);
    

    // 从 CSV 加载位置数据（每行 x,y,z），该函数仅负责解析并把数据保存到内部缓存
    // csvPath: CSV 路径，格式示例见 src/test/test_config/location_test.csv
    bool loadLocationCsv(const QString &csvPath);

    // 直接从一组场景坐标（scene-space）加载位置点，用于把外部计算出的场景坐标序列交由
    // RobotManager 播放/应用（例如由 SceneManager 的 mapRobotToScene 得到的一系列点）
    void loadLocationRowsFromVector(const QVector<QVector3D> &rows);


    // 根据已解析的 location rows，应用第 row 行位置（等效于 applyLocation(m_locationRows[row])）
    void applyLocationRow(int row);

    
    // 直接将模型移动到给定的世界坐标位置（基于 m_originalMeshVertices 做顶点平移，非重建 meshes）
    // target: 目标世界坐标 (x,y,z)
    void applyLocation(const QVector3D &target);

    void refreshRobotPositionsFromScene();      // 根据接收到的实机位置刷新模型在场景中的位置
    void resetRobotPositions();                  // 重置模型位置到场景中心

    // 设定模型整体的旋转（以度为单位的欧拉角），旋转围绕已计算的模型中心 m_modelCenterX/Y/Z。
    // 参数 eulerDeg 表示绕 X、Y、Z 轴的角度（单位：度）。内部按 Z * Y * X 的顺序复合旋转。
    // 调用后会重新应用当前关节角度并触发 frameAdvanced()，以便视图刷新展示新的姿态。
    void setModelRotation(const QVector3D &eulerDeg);
    // 返回当前模型的欧拉角（度），便于外部查询并在需要时恢复模型朝向
    QVector3D modelRotationDeg() const { return m_modelRotationDeg; }

    // 构建并返回用于绘制机器人移动轨迹的简单网格（line strip 格式，顶点顺序即为轨迹点的时间顺序）
    // 颜色与渲染方式由调用者（视图）决定；
    SimpleMesh buildTrajectoryMesh() const;


    // 启动按间隔播放已加载位置的计时器（intervalMs 毫秒），返回是否成功启动
    // 若 loop=false 则播放到最后一帧后停止并发出 locationPlaybackFinished()
    bool startLocationPlayback(int intervalMs, bool loop = true);
    // 启动按间隔播放由机器人位置转换来的场景位置target_pos的计时器（intervalMs 毫秒），返回是否成功启动
    bool startLocationPlayback(QVector3D &target_pos, int intervalMs);


    // 停止位置播放计时器（若存在）
    void stopLocationPlayback();
    // 停止 place CSV 的计时器（若存在），用于外部中断动作播放
    void stopPlacePlayback();

    // expose computed bounds for viewers
    std::vector<SimpleMesh> &meshes() { return m_meshes; }
    const std::vector<SimpleMesh> &meshes() const { return m_meshes; }
    bool loaded() const { return loadSucceeded; }
    float modelCenterX() const { return m_modelCenterX; }
    float modelCenterY() const { return m_modelCenterY; }
    float modelCenterZ() const { return m_modelCenterZ; }
    float modelScale() const { return m_modelScale; }
    float cameraDistance() const { return m_cameraDistance; }
    void setCameraDistance(float d) { m_cameraDistance = d; }
    bool cameraDistanceLocked() const { return m_cameraDistanceLockedDuringPlayback; }

signals:
    void frameAdvanced();
    void animationStarted();
    // 当 place CSV 播放到最后一帧并停止（非循环模式）时发出该信号
    void placePlaybackFinished();
    // 当按非循环模式播放位置并到达最后一行时发出该信号
    void locationPlaybackFinished();

public slots:
    // 应用关节角度到模型
    void applyJointAngles(const std::map<int, double> &jointAngles);
    // 响应 ServoPositionsMonitor 的更新通知，允许 RobotManager 读取最新伺服消息
    void onServoPositionsUpdated();

    // 响应ServoPositionsMonitor的腿部关节角度更新
    void onLegJointAnglesUpdated();

    // 辅助函数：融合伺服数据和腿部数据并更新 pendingServo
    void updateFusedServo();

    // 把伺服角度值应用到模型（复用 advancePlaceFrame 的映射规则）
    void applyServoAnglesRow(const QVector<double> &row);
    // 当 SceneManager 收到新的机器人位姿时调用，该槽会触发场景坐标映射并刷新模型位置
    // void onSceneRobotPoseUpdated(const QVector3D &pose);


    // 响应步态命令更新（来自 GaitCommandMonitor）
    void onGaitCommandUpdated();
    // 设置 gait command 数据源，RobotManager 会订阅其 gaitCommandUpdated 信号
    void setGaitCommandMonitor(GaitCommandMonitor *monitor);

    // 设置/更改 ServoPositionsMonitor 的数据源，允许在运行时注入 monitor
    void setServoPositionsMonitor(ServoPositionsMonitor *monitor);


private:
    bool loadModel(const std::string &file);
    void advancePlaceFrame();
    void computeBounds();
    void initialize();


private:
    QString m_modelPath;
    QString jointConfigPath;
    std::vector<SimpleMesh> m_meshes;
    bool loadSucceeded = false;
    std::vector<std::vector<std::vector<std::pair<int, float>>>> m_meshesInfluences; // 每个 mesh 的每个顶点的骨骼影响
    std::vector<std::vector<float>> m_originalMeshVertices;     // 保存了未变形（原始）的顶点数组
    std::vector<std::vector<CompactInfluence>> m_compactInfluences; // per-mesh compacted influences
    std::vector<BoneInfo> m_bones;  // 骨骼列表
    std::vector<NodeInfo> m_nodeInfos;  // 拷贝了 Assimp 的节点层级到 nodeInfos,不再依赖 Importer 的生命周期
    std::map<QString, int> m_nodeNameToIndex;
    std::vector<float> m_cachedFlatBoneM;   // nbones * 12 floats
    std::map<QString, std::vector<std::pair<int, char>>> m_boneToJoint; // 骨骼名称到关节ID和旋转轴的映射（允许多个）
    std::vector<QVector<double>> m_placeRows;   // 对应动作数据的22个关节角度
    int m_currentRow = 0;
    QTimer *m_animTimer = nullptr;
    std::map<int, double> m_currentJointAngles;     // 当前使用的关节角度
    // 计算包围盒缩放
    float m_modelCenterX = 0, m_modelCenterY = 0, m_modelCenterZ = 0, m_modelScale = 1.0f;
    bool m_cameraDistanceLockedDuringPlayback = false;
    float m_cameraDistance = 3.0f;
    float m_initialCameraDistance = 3.0f;
    bool m_initialCameraDistanceSet = false;
    
    // 世界坐标系下应用于整个模型的平移（由 applyLocation 设置）
    QVector3D m_worldTranslation = QVector3D(0.0f, 0.0f, 0.0f);

    // world-space 旋转矩阵：由 setModelRotation 设置，表示在模型本地坐标系上绕模型中心的整体旋转
    Mat4 m_worldRotation; // 默认在构造函数中初始化为 identity
    // 以度为单位保存的欧拉角（x_deg, y_deg, z_deg），便于查询或 UI 反馈
    QVector3D m_modelRotationDeg = QVector3D(0.0f, 0.0f, 0.0f);
    // 保存程序启动或模型加载时的初始模型欧拉角（用于 reset 恢复）
    QVector3D m_initialModelRotationDeg = QVector3D(0.0f, 0.0f, 0.0f);
    // 基础 yaw 偏置（度）：用于把来自 ROS/Scene 的 yaw（相对于机器人前向）合并到模型的初始朝向上
    // 在 initialize() 中根据首次 setModelRotation 的结果初始化为当前 m_modelRotationDeg.y()
    float m_modelYawOffsetDeg = 0.0f;
    // 额外的可调补偿（度），用于在实机 yaw 与模型 yaw 之间做人工偏移/校准
    float m_modelYawCompensationDeg = 35.0f;

    // --- 位置信息播放相关 ---
    QVector<QVector3D> m_locationRows; // 从 CSV 读取的 x,y,z 列表
    // per-location timestamps in seconds (monotonic), aligned with m_locationRows
    QVector<double> m_locationTimes;
    // 轨迹持续时长
    double m_trajectoryFadeSeconds = 5.0;
    QTimer *m_locationTimer = nullptr; // 用于逐行播放位置（若 startLocationPlayback 被调用）
    int m_currentLocationRow = 0;
    // 已经被播放/应用的轨迹点数量（用于逐步绘制轨迹，初始为0，启动播放时重置）
    int m_playedLocationCount = 0;
    // 位置播放是否循环（true）或在到最后一行保持并结束（false）
    bool m_locationLooping = true;
    // 是否在播放 place CSV 时循环（true）或在到最后一行保持不再循环（false）
    bool m_placeLooping = true;

    QPointer<ServoPositionsMonitor> m_servoPositionsMonitor = nullptr;
    QPointer<SceneManager> m_sceneManager = nullptr;
    QPointer<GaitCommandMonitor> m_gaitCommandMonitor = nullptr;
    // 步态命令位移缩放（用于调节 gait 命令对模型移动的视觉表现）；默认 10.0
    float m_gaitScale = 5.0f;

public:
    // 设置/获取模型 yaw 的补偿（度）。补偿会在从 gait/外部 yaw 计算出模型 Euler 并应用时加入。
    void setModelYawCompensationDeg(float deg) { m_modelYawCompensationDeg = deg; }
    float modelYawCompensationDeg() const { return m_modelYawCompensationDeg; }
    // 设置 gait 缩放因子（例如 100 可把 0.06m 放大到 6m，用于调试可见性)
    void setGaitScale(float s) { m_gaitScale = s; }

    // runtime toggles for smoothing/behavior (useful to debug regressions)
    void setEnableTimeBasedSmoothing(bool v) { m_enableTimeBasedSmoothing = v; }
    void setEnableImmediateYaw(bool v) { m_enableImmediateYaw = v; }
    void setServoStaleMs(int ms) { m_servoStaleMs = ms; }

    // 设置应用 pending servo/gait 更新的定时器间隔（毫秒），用于调节合并更新频率
    void setServoApplyIntervalMs(int ms) { m_servoApplyIntervalMs = ms; if (m_servoApplyTimer) m_servoApplyTimer->setInterval(ms); }

    // 可配置的局部轴映射策略：有些模型的“前向”在本地坐标系上是 X 或 Z
    enum GaitAxisMapping {
        LocalXForward = 0, // local X = forward, local Z = left
        LocalZForward = 1, // local Z = forward, local X = left
        SwapAxes = 2        // swapped interpretation
    };

private:
    GaitAxisMapping m_gaitAxisMapping = LocalZForward;

public:
    void setGaitAxisMapping(GaitAxisMapping m) { m_gaitAxisMapping = m; }


    // --- 行走动画融合相关 ---
    // walk.csv 仅用于提供前 12 列的动画帧数据（预处理在 initialize() 中完成）
    QVector<QVector<double>> m_walkFirst12Rows; // 只保存每行的前12列，用于walking状态的骨骼融合
    QTimer *m_walkTimer = nullptr;              // 行走时播放该CSV的定时器
    int m_walkCurrentRow = 0;
    int m_walkIntervalMs = 50;                 // 默认行走动画帧率（ms），可调整
    bool m_walkLooping = true;
    bool m_isWalking = false;                  // 标记是否处于行走状态
    QVector<double> m_cachedLegAngles;         // 缓存最新的腿部关节数据

    // --- Servo/Gait coalescing to reduce update frequency and avoid UI stalls ---
private:
    QTimer *m_servoApplyTimer = nullptr; // timer to apply pending servo/gait updates at a steady rate
    int m_servoApplyIntervalMs = 33;     // default to ~30Hz
    QMutex m_pendingServoMutex;
    // pending servo frame with timestamp so we can drop stale frames and prefer latest
    struct PendingServoFrame {
        qint64 tsMs = 0; // milliseconds since epoch
        QVector<double> angles;
    };
    PendingServoFrame m_pendingServo;
    // drop servo frames older than this (ms) when applying
    int m_servoStaleMs = 1000; // larger default to avoid accidental dropping

    // feature toggles (allow quick rollback of time-based smoothing / immediate yaw)
    bool m_enableTimeBasedSmoothing = false; // keep default off to preserve previous behavior
    bool m_enableImmediateYaw = false;       // keep default off to avoid unexpected jumps

    QMutex m_pendingGaitMutex;
    double m_pendingGx = 0.0;
    double m_pendingGy = 0.0;
    double m_pendingGdelta = 0.0;
    bool m_hasPendingGait = false;

    // accumulated gait targets (world-space translation and absolute yaw degrees)
    QVector3D m_gaitTargetWorldTranslation = QVector3D(0.0f, 0.0f, 0.0f);
    float m_gaitTargetYawDeg = 0.0f;
    // how quickly to follow gait target (0..1), yaw typically higher for responsiveness
    float m_gaitFollowFactorTranslation = 0.5f;
    float m_gaitFollowFactorYaw = 1.0f;

    // --- quick-yaw assist when switching from translation to in-place rotation ---
    // If the local translation magnitude is below this threshold and a non-trivial
    // gdelta is received, temporarily boost yaw following to avoid visible lag.
    float m_yawBoostTranslationThreshold = 0.02f; // local units (before m_gaitScale)
    float m_yawActivationGdeltaThreshold = 0.5f; // degrees (minimum gdelta to consider a turn)
    int m_yawBoostFrames = 3;                      // how many apply ticks to keep boosted yaw
    int m_yawBoostRemaining = 0;                   // internal counter

    // --- smoothing / blending state ---
    // 0..1 lerp factors: 1.0 = immediate, 0.0 = no movement
    float m_servoLerpFactor = 0.7f; // smooth servo angle changes (higher -> faster)
    float m_gaitLerpFactor = 0.7f;  // smooth gait-driven translation/rotation

    // last-applied servo angles for exponential/linear smoothing
    std::vector<double> m_lastAppliedServoAngles;

    // last apply timestamp (ms) used to compute dt for time-based smoothing
    qint64 m_lastApplyMs = 0;

    // last applied world translation / yaw used to lerp gait movement
    QVector3D m_lastAppliedWorldTranslation = QVector3D(0.0f, 0.0f, 0.0f);
    float m_lastAppliedYawDeg = 0.0f;
    // whether last-applied caches were initialized from current state
    bool m_hasInitializedApplyState = false;

private slots:
    void applyPendingServoAndGait();
};


#endif // MODEL_DISPLAY_ROBOTMANAGER_H

