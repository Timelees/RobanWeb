#ifndef MODEL_DISPLAY_H
#define MODEL_DISPLAY_H

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
#include <QCoreApplication>

#include <QVector3D>
#include <QColor>
#include <QMenu>
#include <QAction>
#include <QToolTip>
#include <QLabel>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <memory>

#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "model_display/meshes.h"
#include "model_display/robotManager.h"
#include "model_display/sceneManager.h"

class ModelDisplay: public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit ModelDisplay(RobotManager *robot, SceneManager *scene, QWidget *parent = nullptr);
    ~ModelDisplay() override;

    void test_showAction();    // 测试显示动作
    void test_showByRobotPoseToMove(); // 测试通过机器人位姿移动模型

protected:
    // 窗口绘制
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    // 鼠标事件
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;


private:
    void setDisplayWindow();         // 设置窗口界面
    static QString resolveTexturePathRuntime(const QString &path);
    bool screenPosToRay(const QPoint &p, QVector3D &outOrigin, QVector3D &outDir);     // 捕获鼠标射线
    void onRobotAnimationStarted();         // 机器人动画开始回调
    void createMeshes(std::vector<SimpleMesh> &meshList);    // 创建网格数据
    void drawMeshes(const std::vector<SimpleMesh> &mes);    // 绘制网格数据

public slots:
    // 菜单动作：在之前点击的位置添加不同颜色的标记
    void onAddMapMarker();          // 添加地图标记
    void onAddPickupMarker();       // 添加取货点标记
    void onAddPlaceMarker();        // 添加放置点标记
    void onSaveMarkers();        // 保存标记到磁盘
    void onDeleteSelected();     // 删除当前选中的标记
    void onClearAllMarkers();   // 删除所有标记
    void onSceneMapping();      // 场景映射函数
    void onRefreshPositions();  // 刷新位置函数
    void onResetPositions();    // 位置重置函数(重置到地图中心)
private:
    RobotManager *m_robot = nullptr;
    SceneManager *m_scene = nullptr;
    float m_rotX = 20.0f, m_rotY = 30.0f;
    float m_cameraDistance = 11.6f;
    QPoint m_lastPos;
    // model centering/scale captured from meshes
    float m_modelCenterX = 0.0f, m_modelCenterY = 0.0f, m_modelCenterZ = 0.0f;
    float m_modelScale = 1.0f;
    // 初始相机距离
    float m_initialCameraDistance = 11.6f;
    bool m_initialCameraDistanceSet = true;
    // when true, prevent non-interactive code from changing cameraDistance while animation is playing
    bool m_cameraDistanceLockedDuringPlayback = false;
    // 鼠标点击判断
    QPoint m_pressPos;
    bool m_pressed = false;
    // 待添加点（由点击场景后菜单触发创建）
    bool m_hasPendingPick = false;
    QVector3D m_pendingPickPos;
    // 当前选中的标记 id（用于删除）
    int m_selectedMarkerId = -1;
    // 顶部工具栏（按钮行）
    QWidget *m_topBar = nullptr;
    // 右键点击浮动显示信息标签
    QLabel *m_tooltipLabel = nullptr;

    // 位置播放状态 （非必要，后面实际使用通过接收slam位置的更新来更新模型位置）
    bool m_locationPlaying = false;

    // connection for demo sequence (used to disconnect when stopping demo)
    std::shared_ptr<QMetaObject::Connection> m_demoConn;
    // 当 place 动画播放完成后触发旋转的连接句柄
    std::shared_ptr<QMetaObject::Connection> m_demoPlaceConn;

    // rotation animation for demo 旋转动画测试参数
    QTimer *m_demoRotateTimer = nullptr;
    float m_demoRotateStartY = 0.0f;
    float m_demoRotateEndY = 0.0f;
    int m_demoRotateSteps = 0;
    int m_demoRotateCurrentStep = 0;

};

#endif // MODEL_DISPLAY_H