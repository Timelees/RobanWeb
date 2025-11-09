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

// ---------- SceneManager ----------
// 负责加载场景模型（静态），提供 meshes 给视图渲染
class SceneManager : public QObject
{
    Q_OBJECT
public:
    explicit SceneManager(const QString &modelPath, QObject *parent = nullptr);
    ~SceneManager() override;

    bool loadModel(const std::string &file);        // 加载模型
    void createGridMesh(float size = 20.0f, int divisions = 20, float yOffset = 0.0f);      // 创建网格地面平面

    std::vector<SimpleMesh> &meshes() { return m_meshes; }
    const std::vector<SimpleMesh> &meshes() const { return m_meshes; }
    bool loaded() const { return m_loaded; }

private:
    QString m_modelPath;
    std::vector<SimpleMesh> m_meshes;
    bool loadSucceeded = false;
    bool m_loaded = false;
};  