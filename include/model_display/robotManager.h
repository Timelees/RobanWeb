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
#include <QFileInfo>
#include <QImage>
#include <QDebug>
#include <QMatrix4x4>
#include <QDir>
#include <QByteArray>
#include <QPainter>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "model_display/meshes.h"
#include <QVector3D>

// ---------- RobotManager ----------
// 负责加载机器人模型（带骨骼动画），提供 meshes 给视图
class RobotManager : public QObject
{
    Q_OBJECT
public:
    explicit RobotManager(const QString &modelPath, QObject *parent = nullptr);
    ~RobotManager() override = default;


    bool loadBoneJointMapping(const QString &csvPath);
    bool loadPlaceCsv(const QString &csvPath, int intervalMs = 100, bool loop = true);
    

    // 从 CSV 加载位置数据（每行 x,y,z），该函数仅负责解析并把数据保存到内部缓存
    // csvPath: CSV 路径，格式示例见 src/test/test_config/location_test.csv
    bool loadLocationCsv(const QString &csvPath);


    // 根据已解析的 location rows，应用第 row 行位置（等效于 applyLocation(m_locationRows[row])）
    void applyLocationRow(int row);

    
    // 直接将模型移动到给定的世界坐标位置（基于 m_originalMeshVertices 做顶点平移，非重建 meshes）
    // target: 目标世界坐标 (x,y,z)
    void applyLocation(const QVector3D &target);

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
    void applyJointAngles(const std::map<int, double> &jointAngles);

private:
    bool loadModel(const std::string &file);
    void advancePlaceFrame();
    void computeBounds();


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
    
    // world-space translation applied to the whole model (set by applyLocation)
    QVector3D m_worldTranslation = QVector3D(0.0f, 0.0f, 0.0f);

    // world-space 旋转矩阵：由 setModelRotation 设置，表示在模型本地坐标系上绕模型中心的整体旋转
    Mat4 m_worldRotation; // 默认在构造函数中初始化为 identity
    // 以度为单位保存的欧拉角（x_deg, y_deg, z_deg），便于查询或 UI 反馈
    QVector3D m_modelRotationDeg = QVector3D(0.0f, 0.0f, 0.0f);

    // --- 位置信息播放相关 ---
    QVector<QVector3D> m_locationRows; // 从 CSV 读取的 x,y,z 列表
    QTimer *m_locationTimer = nullptr; // 用于逐行播放位置（若 startLocationPlayback 被调用）
    int m_currentLocationRow = 0;
    // 已经被播放/应用的轨迹点数量（用于逐步绘制轨迹，初始为0，启动播放时重置）
    int m_playedLocationCount = 0;
    // 位置播放是否循环（true）或在到最后一行保持并结束（false）
    bool m_locationLooping = true;
    // 是否在播放 place CSV 时循环（true）或在到最后一行保持不再循环（false）
    bool m_placeLooping = true;

};

#endif // MODEL_DISPLAY_ROBOTMANAGER_H

