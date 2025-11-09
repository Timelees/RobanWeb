#ifndef MODEL_DISPLAY_H
#define MODEL_DISPLAY_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QOpenGLShaderProgram>
#include <QImage>
#include <QFile>
#include <QFileInfo>
#include <QPoint>
#include <QDir>
#include <QDebug>

#include <vector>
#include <algorithm>
#include <cmath>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// reuse shared mesh/helpers definitions
#include "model_display/meshes.h"

class ModelDisplay: public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit ModelDisplay(const QString &modelPath, QWidget *parent = nullptr);
    ~ModelDisplay() override;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;


private:
    bool loadModel(const std::string &file);
    void computeBounds();

private:
    QString m_modelPath;                // 模型路径
    std::vector<SimpleMesh> m_meshes;   // 网格数据
    bool loadSucceeded = false;          // 模型加载是否成功
    std::vector<QString> matTexPaths;           // 材质文件路径
    std::vector<QImage> matEmbeddedImages;      // 嵌入式纹理图像


    float rotY = -20.0f;
    float rotX = 30.0f;
    float distance = 3.0f;
    float panX = 0.0f;
    float panY = 0.0f;
    QPoint lastPos;          // 上次鼠标位置
    Qt::MouseButton lastButton = Qt::NoButton;

};

#endif // MODEL_DISPLAY_H