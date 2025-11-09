// test_model_scene.cpp
// 封装 RobotManager (处理 Roban.fbx) 和 SceneManager (处理 Scene.fbx)
// ModelViewer 调用它们以完成渲染与简单动画播放

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

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "model_display/meshes.h"
#include "model_display/robotManager.h"
#include "model_display/sceneManager.h"

// helper: try to resolve a texture path by testing several candidate locations
static QString resolveTexturePathRuntime(const QString &path)
{
    if (path.isEmpty())
        return QString();
    // if absolute or already exists, return normalized
    if (QFile::exists(path))
        return QDir::cleanPath(path);
    // try application dir
    QString appdir = QCoreApplication::applicationDirPath();
    QString fname = QFileInfo(path).fileName();
    QString cand;
    cand = QDir::cleanPath(appdir + QDir::separator() + path);
    if (QFile::exists(cand)) return cand;
    cand = QDir::cleanPath(appdir + QDir::separator() + fname);
    if (QFile::exists(cand)) return cand;
    // try cwd
    QString cwd = QDir::currentPath();
    cand = QDir::cleanPath(cwd + QDir::separator() + path);
    if (QFile::exists(cand)) return cand;
    cand = QDir::cleanPath(cwd + QDir::separator() + fname);
    if (QFile::exists(cand)) return cand;
    // try appdir/assets and cwd/assets
    cand = QDir::cleanPath(appdir + QDir::separator() + "assets" + QDir::separator() + fname);
    if (QFile::exists(cand)) return cand;
    cand = QDir::cleanPath(cwd + QDir::separator() + "assets" + QDir::separator() + fname);
    if (QFile::exists(cand)) return cand;
    // walk up from cwd and appdir looking for assets/<fname>
    auto searchUp = [&](const QString &start) -> QString {
        QDir d(start);
        for (int i = 0; i < 6; ++i) {
            if (!d.exists()) break;
            QString check = QDir::cleanPath(d.absolutePath() + QDir::separator() + "assets" + QDir::separator() + fname);
            if (QFile::exists(check)) return check;
            if (!d.cdUp()) break;
        }
        return QString();
    };
    QString r = searchUp(appdir);
    if (!r.isEmpty()) return r;
    r = searchUp(cwd);
    if (!r.isEmpty()) return r;
    return QString();
}

// ---------- ModelViewer ----------
class ModelViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    ModelViewer(RobotManager *robot, SceneManager *scene, QWidget *parent = nullptr)
        : QOpenGLWidget(parent), m_robot(robot), m_scene(scene)
    {
        setMinimumSize(800, 600);
        setFocusPolicy(Qt::StrongFocus);
        if (m_robot)
            connect(m_robot, &RobotManager::frameAdvanced, this, QOverload<>::of(&ModelViewer::update));
        // connect to robot animation started so viewer can restore initial camera and lock programmatic zoom
        if (m_robot)
        {
            connect(m_robot, &RobotManager::animationStarted, this, &ModelViewer::onRobotAnimationStarted);
            // copy computed model bounds so viewer applies same centering/scale as RobotManager
            if (m_robot->loaded())
            {
                m_modelCenterX = m_robot->modelCenterX();
                m_modelCenterY = m_robot->modelCenterY();
                m_modelCenterZ = m_robot->modelCenterZ();
                m_modelScale = m_robot->modelScale();
                m_cameraDistance = m_robot->cameraDistance();
                m_cameraDistanceLockedDuringPlayback = m_robot->cameraDistanceLocked();
                qDebug() << "ModelViewer: copied robot bounds center=(" << m_modelCenterX << m_modelCenterY << m_modelCenterZ << ") scale=" << m_modelScale;
            }
        }
    }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);

        if (m_scene)
            createMeshes(m_scene->meshes());
        if (m_robot)
            createMeshes(m_robot->meshes());
    }

    void resizeGL(int w, int h) override
    {
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = float(w) / float(qMax(1, h));
        const float fovDeg = 45.0f;
        const float fov = fovDeg * 3.14159265358979323846f / 180.0f;
        const float nearv = 0.1f, farv = 1000.0f;

        const float top = tanf(fov * 0.5f) * nearv;
        const float bottom = -top;
        const float right = top * aspect;
        const float left = -right;
        glFrustum(left, right, bottom, top, nearv, farv);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    void paintGL() override
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        // apply model-space centering + scale so vertices and bone transforms share same space
        if (fabs(m_modelCenterX) > 1e-9f || fabs(m_modelCenterY) > 1e-9f || fabs(m_modelCenterZ) > 1e-9f)
            glTranslatef(-m_modelCenterX, -m_modelCenterY, -m_modelCenterZ);
        if (fabs(m_modelScale - 1.0f) > 1e-9f)
            glScalef(m_modelScale, m_modelScale, m_modelScale);
        // simple camera (translate back then rotate view)
        glTranslatef(0, 0, -m_cameraDistance);
        glRotatef(m_rotX, 1, 0, 0);
        glRotatef(m_rotY, 0, 1, 0);

        // capture the initial camera distance at first paint (the actual shown view)
        if (!m_initialCameraDistanceSet)
        {
            m_initialCameraDistance = m_cameraDistance;
            m_initialCameraDistanceSet = true;
            qDebug() << "Captured initialCameraDistance:" << m_initialCameraDistance;
        }

        if (!m_robot->loaded())
        {
            // 若模型加载失败，使用 QPainter 在窗口上绘制错误文本（QPainter 在 paintGL 中使用需在 gl 绘制后切换到 2D 绘制）
            QPainter p(this);
            p.setPen(Qt::black);
            p.drawText(rect(), Qt::AlignCenter, "Failed to load robot model\n");
            p.end();
            return;
        }

        // if (!m_scene->loaded()){
        //     QPainter p(this);
        //     p.setPen(Qt::black);
        //     p.drawText(rect(), Qt::AlignCenter, "Failed to load scene model\n");
        //     p.end();
        //     return;
        // }

        // draw scene meshes first (static)
        if (m_scene && m_scene->loaded())
            drawMeshes(m_scene->meshes());

        // draw robot meshes
        if (m_robot)
            drawMeshes(m_robot->meshes());
    }

    void mousePressEvent(QMouseEvent *e) override { m_lastPos = e->pos(); }
    void mouseMoveEvent(QMouseEvent *e) override
    {
        QPoint delta = e->pos() - m_lastPos;
        if (e->buttons() & Qt::LeftButton)
        {
            m_rotX += delta.y() * 0.5f;
            m_rotY += delta.x() * 0.5f;
            update();
        }
        m_lastPos = e->pos();
    }
    void wheelEvent(QWheelEvent *e) override
    {
        int d = e->angleDelta().y();
        m_cameraDistance -= d * 0.01f;
        m_cameraDistance = qMax(0.1f, m_cameraDistance);
        // qDebug() << "Mouse wheel: cameraDistance=" << m_cameraDistance;
        update();
    }

private:
private:
    void onRobotAnimationStarted();

    void createMeshes(std::vector<SimpleMesh> &meshList)
    {
        for (auto &m : meshList)
        {
            if (m.texId == 0)
            {
                QImage img;
                if (!m.diffuseImage.isNull())
                {
                    img = m.diffuseImage;
                }
                else if (!m.diffuseTexPath.isEmpty())
                {
                    img = QImage(m.diffuseTexPath);
                    // runtime fallback: if direct load failed, try to resolve via common candidate locations
                    if (img.isNull()) {
                        QString resolved = resolveTexturePathRuntime(m.diffuseTexPath);
                        if (!resolved.isEmpty()) {
                            qDebug() << "createMeshes: runtime resolved texture" << m.diffuseTexPath << "->" << resolved;
                            img = QImage(resolved);
                            if (!img.isNull()) {
                                // update diffuseTexPath to resolved absolute path so later checks reflect actual file
                                m.diffuseTexPath = resolved;
                            }
                        }
                    }
                }
                if (!img.isNull())
                {
                    // Convert image to a well-defined RGBA layout that matches GL_RGBA upload
                    // QImage::Format_RGBA8888 gives 4 bytes per pixel in R,G,B,A order (unpremultiplied)
                    QImage tex = img.convertToFormat(QImage::Format_RGBA8888);
                    qDebug() << "createMeshes: uploading texture size=" << tex.width() << "x" << tex.height() << " format=" << tex.format();
                    glGenTextures(1, &m.texId);
                    glBindTexture(GL_TEXTURE_2D, m.texId);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    // prevent alignment issues when width is not multiple of 4
                    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

                    // Upload using GL_RGBA / GL_UNSIGNED_BYTE because Format_RGBA8888 is RGBA order
                    GLenum uploadFormat = GL_RGBA;
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.width(), tex.height(), 0, uploadFormat, GL_UNSIGNED_BYTE, tex.constBits());
                    GLenum gerr = glGetError();
                    if (gerr != GL_NO_ERROR)
                        qDebug() << "createMeshes: glTexImage2D error=" << gerr;

                    // print first bytes of pixel data for quick channel-order inspection
                    int totalBytes = tex.height() * tex.bytesPerLine();
                    if (totalBytes >= 4) {
                        const unsigned char *p = tex.constBits();
                        qDebug() << "createMeshes: first pixel bytes (R,G,B,A)=" << (int)p[0] << (int)p[1] << (int)p[2] << (int)p[3];
                    }

                    glGenerateMipmap(GL_TEXTURE_2D);
                    glBindTexture(GL_TEXTURE_2D, 0);
                }
                else if (!m.diffuseTexPath.isEmpty() || !m.diffuseImage.isNull())
                {
                    bool exists = false;
                    if (!m.diffuseTexPath.isEmpty())
                        exists = QFile::exists(m.diffuseTexPath);
                    qDebug() << "Failed to load texture image for mesh; path:" << m.diffuseTexPath << " exists?" << exists << " embedded?" << !m.diffuseImage.isNull();
                }
            }
        }
    }

    void drawMeshes(const std::vector<SimpleMesh> &mes)
    {
        for (const SimpleMesh &m : mes)
        {
            bool useTex = (m.texId != 0) && (!m.texcoords.empty());
            if (useTex)
            {
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, m.texId);
            }
            glBegin(GL_TRIANGLES);
            for (size_t i = 0; i < m.indices.size(); i += 3)
            {
                unsigned int ia = m.indices[i];
                unsigned int ib = m.indices[i + 1];
                unsigned int ic = m.indices[i + 2];
                // A: 若存在法线则先提交法线，若使用纹理则先提交纹理坐标，再提交顶点位置
                // 注意：这里假设 texcoords 的每个顶点有一对 (u,v)，并且 v 不需要翻转（若贴图上下颠倒，可改为 1-v）
                if (!m.normals.empty())
                    glNormal3f(m.normals[3 * ia], m.normals[3 * ia + 1], m.normals[3 * ia + 2]);
                if (useTex)
                    glTexCoord2f(m.texcoords[2 * ia], 1.0f - m.texcoords[2 * ia + 1]);
                glVertex3f(m.vertices[3 * ia], m.vertices[3 * ia + 1], m.vertices[3 * ia + 2]);
                // B
                if (!m.normals.empty())
                    glNormal3f(m.normals[3 * ib], m.normals[3 * ib + 1], m.normals[3 * ib + 2]);
                if (useTex)
                    glTexCoord2f(m.texcoords[2 * ib], 1.0f - m.texcoords[2 * ib + 1]);
                glVertex3f(m.vertices[3 * ib], m.vertices[3 * ib + 1], m.vertices[3 * ib + 2]);
                // C
                if (!m.normals.empty())
                    glNormal3f(m.normals[3 * ic], m.normals[3 * ic + 1], m.normals[3 * ic + 2]);
                if (useTex)
                    glTexCoord2f(m.texcoords[2 * ic], 1.0f - m.texcoords[2 * ic + 1]);
                glVertex3f(m.vertices[3 * ic], m.vertices[3 * ic + 1], m.vertices[3 * ic + 2]);
            }
            glEnd();
            if (useTex)
            {
                glBindTexture(GL_TEXTURE_2D, 0);
                glDisable(GL_TEXTURE_2D);
            }
        }
    }

    RobotManager *m_robot = nullptr;
    SceneManager *m_scene = nullptr;
    float m_rotX = 20.0f, m_rotY = 30.0f;
    float m_cameraDistance = 11.6f;
    QPoint m_lastPos;
    // model centering/scale captured from meshes
    float m_modelCenterX = 0.0f, m_modelCenterY = 0.0f, m_modelCenterZ = 0.0f;
    float m_modelScale = 1.0f;
    // initial camera distance capture
    float m_initialCameraDistance = 11.6f;
    bool m_initialCameraDistanceSet = true;
    // when true, prevent non-interactive code from changing cameraDistance while animation is playing
    bool m_cameraDistanceLockedDuringPlayback = false;
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QString modelRobot = QString::fromUtf8((argc > 1) ? argv[1] : "..\\assets\\Roban.fbx");
    QString modelScene = QString::fromUtf8((argc > 2) ? argv[2] : "..\\assets\\scene.obj");

    RobotManager *robot = new RobotManager(modelRobot);
    SceneManager *scene = new SceneManager(modelScene);

    ModelViewer viewer(robot, scene);
    viewer.show();

    // 尝试常见路径加载 CSV（如果存在）
    QStringList candMap = {"test_config/boneToJoint.csv", "src/test/test_config/boneToJoint.csv", "..\\src\\test\\test_config\\boneToJoint.csv", "./test_config/boneToJoint.csv"};
    for (const QString &p : candMap)
    {
        if (QFile::exists(p))
        {
            robot->loadBoneJointMapping(p);
            break;
        }
    }
    QStringList candPlace = {"test_config/place.csv", "src/test/test_config/place.csv", "..\\src\\test\\test_config\\place.csv", "./test_config/place.csv"};
    for (const QString &p : candPlace)
    {
        if (QFile::exists(p))
        {
            robot->loadPlaceCsv(p, 80);
            break;
        }
    }

    return app.exec();
}

// out-of-class slot implementation (declared inside ModelViewer)
void ModelViewer::onRobotAnimationStarted()
{
    if (m_initialCameraDistanceSet)
    {
        m_cameraDistance = m_initialCameraDistance;
        qDebug() << "ModelViewer: animation started, restored cameraDistance to" << m_cameraDistance;
    }
    m_cameraDistanceLockedDuringPlayback = true;
}

#include "test_model_scene.moc"
