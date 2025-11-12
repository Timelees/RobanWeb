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
#include <QVector3D>
#include <QColor>
#include <QDir>
#include <QByteArray>
#include <QPainter>
#include <QThread>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "model_display/meshes.h"
#include "socket_process/webSocketWorker.h"
#include "ros_process/pose.h"

class WebSocketWorker;

// ---------- SceneManager ----------
// 负责加载场景模型（静态），提供 meshes 给视图渲染
class SceneManager : public QObject
{
    Q_OBJECT
public:
    explicit SceneManager(WebSocketWorker *webSocketWorker, const QString &modelPath, QObject *parent = nullptr);
    // compatibility ctor used by tests/tools that don't provide a WebSocketWorker
    explicit SceneManager(const QString &modelPath, QObject *parent = nullptr);
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


    const std::vector<Marker> &markers() const { return m_markers; }

private:
    // 内部函数：创建球体 mesh 并返回 mesh 索引（仅 SceneManager 内部使用）
    int addMarkerSphere(const QVector3D &pos, float radius = 0.05f, const QColor &color = QColor(0, 255, 0));

private:
    QString m_modelPath;
    std::vector<SimpleMesh> m_meshes;
    bool loadSucceeded = false;
    bool m_loaded = false;
    // 内部标记存储
    std::vector<Marker> m_markers;
    int m_nextMarkerId = 1;

    WebSocketWorker *m_worker;

    PoseMonitor *poseMonitor;       // 位姿监视器
    QVector3D robotPose;            // 机器人当前位姿

    
};


#endif // SCENEMANAGER_H