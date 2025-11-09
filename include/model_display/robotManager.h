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

// ---------- RobotManager ----------
// 负责加载机器人模型（带骨骼动画），提供 meshes 给视图
class RobotManager : public QObject
{
    Q_OBJECT
public:
    explicit RobotManager(const QString &modelPath, QObject *parent = nullptr);
    ~RobotManager() override = default;


    bool loadBoneJointMapping(const QString &csvPath);
    bool loadPlaceCsv(const QString &csvPath, int intervalMs = 100);

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

public slots:
    void applyJointAngles(const std::map<int, double> &jointAngles);

private:
    bool loadModel(const std::string &file);
    void advancePlaceFrame();
    void computeBounds();


private:
    QString m_modelPath;
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
    // computed bounds
    float m_modelCenterX = 0, m_modelCenterY = 0, m_modelCenterZ = 0, m_modelScale = 1.0f;
    bool m_cameraDistanceLockedDuringPlayback = false;
    float m_cameraDistance = 3.0f;
    float m_initialCameraDistance = 3.0f;
    bool m_initialCameraDistanceSet = false;

};
