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

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <QWidget>
#include <QPushButton>
#include <QHBoxLayout>

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
    if (QFile::exists(cand))
        return cand;
    cand = QDir::cleanPath(appdir + QDir::separator() + fname);
    if (QFile::exists(cand))
        return cand;
    // try cwd
    QString cwd = QDir::currentPath();
    cand = QDir::cleanPath(cwd + QDir::separator() + path);
    if (QFile::exists(cand))
        return cand;
    cand = QDir::cleanPath(cwd + QDir::separator() + fname);
    if (QFile::exists(cand))
        return cand;
    // try appdir/assets and cwd/assets
    cand = QDir::cleanPath(appdir + QDir::separator() + "assets" + QDir::separator() + fname);
    if (QFile::exists(cand))
        return cand;
    cand = QDir::cleanPath(cwd + QDir::separator() + "assets" + QDir::separator() + fname);
    if (QFile::exists(cand))
        return cand;
    // walk up from cwd and appdir looking for assets/<fname>
    auto searchUp = [&](const QString &start) -> QString
    {
        QDir d(start);
        for (int i = 0; i < 6; ++i)
        {
            if (!d.exists())
                break;
            QString check = QDir::cleanPath(d.absolutePath() + QDir::separator() + "assets" + QDir::separator() + fname);
            if (QFile::exists(check))
                return check;
            if (!d.cdUp())
                break;
        }
        return QString();
    };
    QString r = searchUp(appdir);
    if (!r.isEmpty())
        return r;
    r = searchUp(cwd);
    if (!r.isEmpty())
        return r;
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
        // 在 widget 内部创建水平工具栏（按钮行），放在窗口顶部
        m_topBar = new QWidget(this);
        m_topBar->setObjectName("calibTopBar");
        m_topBar->setAutoFillBackground(true);
        QHBoxLayout *hb = new QHBoxLayout(m_topBar);
        hb->setContentsMargins(4, 4, 4, 4);
        hb->setSpacing(6);
        // 创建五个按钮，按需连接到已有槽
        QPushButton *btnMap = new QPushButton(QString::fromUtf8("地图标定"), m_topBar);
        QPushButton *btnPickup = new QPushButton(QString::fromUtf8("取货点标定"), m_topBar);
        QPushButton *btnPlace = new QPushButton(QString::fromUtf8("放置点标定"), m_topBar);
        QPushButton *btnSave = new QPushButton(QString::fromUtf8("标定保存"), m_topBar);
        QPushButton *btnDelete = new QPushButton(QString::fromUtf8("标定删除"), m_topBar);
        QPushButton *btnClearAll = new QPushButton(QString::fromUtf8("全部删除"), m_topBar);
        // 将按钮加入布局
        hb->addWidget(btnMap);
        hb->addWidget(btnPickup);
        hb->addWidget(btnPlace);
        hb->addWidget(btnSave);
        hb->addWidget(btnDelete);
        hb->addWidget(btnClearAll);
        // ----------------------------------
        // 播放位置（路径）按钮
        QPushButton *btnPlayLocations = new QPushButton(QString::fromUtf8("播放位置"), m_topBar);
        hb->addWidget(btnPlayLocations);
        // ----------------------------------

        hb->addStretch();
        // 连接信号
        connect(btnMap, &QPushButton::clicked, this, &ModelViewer::onAddMapMarker);
        connect(btnPickup, &QPushButton::clicked, this, &ModelViewer::onAddPickupMarker);
        connect(btnPlace, &QPushButton::clicked, this, &ModelViewer::onAddPlaceMarker);
        connect(btnSave, &QPushButton::clicked, this, &ModelViewer::onSaveMarkers);
        connect(btnDelete, &QPushButton::clicked, this, &ModelViewer::onDeleteSelected);
        connect(btnClearAll, &QPushButton::clicked, this, &ModelViewer::onClearAllMarkers);
        // ---------------------------------------
        // 非必要，后面实际使用通过接收slam位置的更新来更新模型位置
        connect(btnPlayLocations, &QPushButton::clicked, this, [this, btnPlayLocations]()
                {
        // 切换播放/停止
        if (!m_locationPlaying) {
            // 默认 500ms 间隔播放位置
            if (m_robot && m_robot->startLocationPlayback(500)) {
                m_locationPlaying = true;
                btnPlayLocations->setText(QString::fromUtf8("停止播放"));
            }
        } else {
            if (m_robot) m_robot->stopLocationPlayback();
            m_locationPlaying = false;
            btnPlayLocations->setText(QString::fromUtf8("播放位置"));
        } });

        // ---------------------------------------
        // 初始布局（位置和高度）
        int th = btnMap->sizeHint().height() + 8;
        m_topBar->setGeometry(6, 6, width() - 12, th);
    }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);

        // 在 GL 上下文准备好后，先让 SceneManager 加载此前保存的标记数据（如果存在），
        // 然后创建/上传所有 meshes（包括场景、机器人和标记的贴图）。
        if (m_scene)
            m_scene->loadMarkers();
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
        // 调整顶部工具栏宽度/位置
        if (m_topBar)
        {
            int th = m_topBar->sizeHint().height();
            m_topBar->setGeometry(6, 6, w - 12, th);
        }
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

        if (!m_scene->loaded())
        {
            QPainter p(this);
            p.setPen(Qt::black);
            p.drawText(rect(), Qt::AlignCenter, "Failed to load scene model\n");
            p.end();
            return;
        }

        // draw scene meshes first (static)
        if (m_scene && m_scene->loaded())
            drawMeshes(m_scene->meshes());

        // draw robot meshes
        if (m_robot)
            drawMeshes(m_robot->meshes());

        // 如果存在位置点，绘制轨迹线（绿色），使用 GL_LINE_STRIP 连续连接位置点
        if (m_robot)
        {
            SimpleMesh traj = m_robot->buildTrajectoryMesh();
            if (!traj.vertices.empty())
            {
                // 关闭纹理，设置线宽与颜色
                glDisable(GL_TEXTURE_2D);
                glLineWidth(2.0f);
                glColor3f(0.0f, 1.0f, 0.0f); // 绿色
                glBegin(GL_LINE_STRIP);
                for (size_t vi = 0; vi + 2 < traj.vertices.size(); vi += 3)
                {
                    glVertex3f(traj.vertices[vi + 0], traj.vertices[vi + 1], traj.vertices[vi + 2]);
                }
                glEnd();
                // 恢复默认颜色（白色），线宽和纹理状态由后续绘制自行调整
                glColor3f(1.0f, 1.0f, 1.0f);
            }
        }
    }

    void mousePressEvent(QMouseEvent *e) override
    {
        // 记录按下位置用于判断是否为点击（非拖拽）
        m_lastPos = e->pos();
        // 记录按下位置（用于左键判断拖拽/点击，也用于右键信息显示阈值）
        m_pressPos = e->pos();
        if (e->button() == Qt::LeftButton)
        {
            m_pressed = true;
        }
        QOpenGLWidget::mousePressEvent(e);
    }

    void mouseReleaseEvent(QMouseEvent *e) override
    {
        // 左键短点击表示选择/拾取位置或标记
        if (e->button() == Qt::LeftButton && m_pressed)
        {
            QPoint delta = e->pos() - m_pressPos;
            if (delta.manhattanLength() < 6) // 认为是点击
            {
                QVector3D origin, dir;
                if (screenPosToRay(e->pos(), origin, dir))
                {
                    // 优先尝试拾取已有标记
                    QVector3D hitMarkerPt;
                    int mid = -1;
                    if (m_scene)
                        mid = m_scene->pickMarkerByRay(origin, dir, hitMarkerPt);
                    if (mid >= 0)
                    {
                        // 选中了某个标记
                        m_selectedMarkerId = mid;
                        qDebug() << "Selected marker id=" << mid << " at " << hitMarkerPt;
                    }
                    else
                    {
                        // 记录场景中的点位作为待添加点（当点击菜单项时生效）
                        QVector3D hit; // x y z
                        if (m_scene && m_scene->pickIntersect(origin, dir, hit))
                        {
                            m_pendingPickPos = hit;
                            m_hasPendingPick = true;
                            qDebug() << "Pending pick pos set to" << hit;
                        }
                    }
                }
            }
        }
        // 右键点击标记点显示标记信息提示框
        if (e->button() == Qt::RightButton)
        {
            QPoint delta = e->pos() - m_pressPos; // m_pressPos set on press only for left button; allow small movement anyway
            if (delta.manhattanLength() < 12)
            {
                QVector3D origin, dir, hitMarkerPt;
                if (screenPosToRay(e->pos(), origin, dir) && m_scene)
                {
                    int mid = m_scene->pickMarkerByRay(origin, dir, hitMarkerPt);
                    if (mid >= 0)
                    {
                        QString txt = QString("id=%1\nx=%2\ny=%3\nz=%4")
                                          .arg(mid)
                                          .arg(hitMarkerPt.x(), 0, 'f', 3)
                                          .arg(hitMarkerPt.y(), 0, 'f', 3)
                                          .arg(hitMarkerPt.z(), 0, 'f', 3);
                        QToolTip::showText(mapToGlobal(e->pos()), txt, this);
                        qDebug() << "Right-click marker id=" << mid << " at " << hitMarkerPt;
                    }
                }
            }
        }

        m_pressed = false;
        QOpenGLWidget::mouseReleaseEvent(e);
    }
    void mouseMoveEvent(QMouseEvent *e) override
    {
        QPoint delta = e->pos() - m_lastPos;
        // 左键按住：旋转视角（保留原先行为）
        if (e->buttons() & Qt::LeftButton)
        {
            m_rotX += delta.y() * 0.5f;
            m_rotY += delta.x() * 0.5f;
            update();
        }
        // 右键按住：平移视角（左右/上下），通过修改模型中心实现
        else if (e->buttons() & Qt::RightButton)
        {
            // pan speed 与相机距离相关，调节感受
            float panFactor = 0.0025f * m_cameraDistance;
            m_modelCenterX += delta.x() * panFactor; // 鼠标右移 -> 视点右移
            m_modelCenterY -= delta.y() * panFactor; // 鼠标上移 -> 视点上移
            update();
        }

        // 记录鼠标当前位置
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
    // convert screen position (widget coordinates) to a ray in model space
    bool screenPosToRay(const QPoint &p, QVector3D &outOrigin, QVector3D &outDir)
    {
        int w = width();
        int h = height();
        if (w <= 0 || h <= 0)
            return false;
        // build projection matching resizeGL
        QMatrix4x4 proj;
        float aspect = float(w) / float(qMax(1, h));
        const float fovDeg = 45.0f;
        const float fov = fovDeg * 3.14159265358979323846f / 180.0f;
        const float nearv = 0.1f, farv = 1000.0f;
        const float top = tanf(fov * 0.5f) * nearv;
        const float bottom = -top;
        const float right = top * aspect;
        const float left = -right;
        proj.frustum(left, right, bottom, top, nearv, farv);

        // build modelview matching paintGL order
        QMatrix4x4 mv;
        mv.setToIdentity();
        if (fabs(m_modelCenterX) > 1e-9f || fabs(m_modelCenterY) > 1e-9f || fabs(m_modelCenterZ) > 1e-9f)
            mv.translate(-m_modelCenterX, -m_modelCenterY, -m_modelCenterZ);
        if (fabs(m_modelScale - 1.0f) > 1e-9f)
            mv.scale(m_modelScale);
        mv.translate(0, 0, -m_cameraDistance);
        mv.rotate(m_rotX, 1, 0, 0);
        mv.rotate(m_rotY, 0, 1, 0);

        QMatrix4x4 inv = (proj * mv).inverted();
        // normalized device coords
        float nx = (2.0f * p.x()) / float(w) - 1.0f;
        float ny = 1.0f - (2.0f * p.y()) / float(h);
        QVector4D nearN(nx, ny, -1.0f, 1.0f);
        QVector4D farN(nx, ny, 1.0f, 1.0f);
        QVector4D nearW = inv * nearN;
        QVector4D farW = inv * farN;
        if (nearW.w() == 0.0f || farW.w() == 0.0f)
            return false;
        QVector3D nearWorld(nearW.x() / nearW.w(), nearW.y() / nearW.w(), nearW.z() / nearW.w());
        QVector3D farWorld(farW.x() / farW.w(), farW.y() / farW.w(), farW.z() / farW.w());
        outOrigin = nearWorld;
        outDir = (farWorld - nearWorld).normalized();
        return true;
    }

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
                    if (img.isNull())
                    {
                        QString resolved = resolveTexturePathRuntime(m.diffuseTexPath);
                        if (!resolved.isEmpty())
                        {
                            qDebug() << "createMeshes: runtime resolved texture" << m.diffuseTexPath << "->" << resolved;
                            img = QImage(resolved);
                            if (!img.isNull())
                            {
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
                    // qDebug() << "createMeshes: uploading texture size=" << tex.width() << "x" << tex.height() << " format=" << tex.format();
                    glGenTextures(1, &m.texId);
                    glBindTexture(GL_TEXTURE_2D, m.texId);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    // prevent alignment issues when width is not multiple of 4
                    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

                    // Upload using GL_RGBA / GL_UNSIGNED_BYTE because Format_RGBA8888 is RGBA order
                    GLenum uploadFormat = GL_RGBA;
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.width(), tex.height(), 0, uploadFormat, GL_UNSIGNED_BYTE, tex.constBits());
                    // GLenum gerr = glGetError();
                    // if (gerr != GL_NO_ERROR)
                    //     qDebug() << "createMeshes: glTexImage2D error=" << gerr;

                    // print first bytes of pixel data for quick channel-order inspection
                    // int totalBytes = tex.height() * tex.bytesPerLine();
                    // if (totalBytes >= 4) {
                    //     const unsigned char *p = tex.constBits();
                    //     qDebug() << "createMeshes: first pixel bytes (R,G,B,A)=" << (int)p[0] << (int)p[1] << (int)p[2] << (int)p[3];
                    // }

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

    // 位置播放状态 （非必要，后面实际使用通过接收slam位置的更新来更新模型位置）
    bool m_locationPlaying = false;

public slots:
    // 菜单动作：在之前点击的位置添加不同颜色的标记
    void onAddMapMarker()
    {
        if (!m_hasPendingPick || !m_scene)
            return;
        m_scene->addCalibrationMarker(SceneManager::Marker_Map, m_pendingPickPos, 0.08f, QColor(0, 200, 0));
        // 确保在 GL 上下文中上传纹理
        makeCurrent();
        createMeshes(m_scene->meshes());
        doneCurrent();
        update();
        m_hasPendingPick = false;
    }
    void onAddPickupMarker()
    {
        if (!m_hasPendingPick || !m_scene)
            return;
        m_scene->addCalibrationMarker(SceneManager::Marker_Pickup, m_pendingPickPos, 0.08f, QColor(200, 0, 0));
        makeCurrent();
        createMeshes(m_scene->meshes());
        doneCurrent();
        update();
        m_hasPendingPick = false;
    }
    void onAddPlaceMarker()
    {
        if (!m_hasPendingPick || !m_scene)
            return;
        m_scene->addCalibrationMarker(SceneManager::Marker_Place, m_pendingPickPos, 0.08f, QColor(0, 0, 200));
        makeCurrent();
        createMeshes(m_scene->meshes());
        doneCurrent();
        update();
        m_hasPendingPick = false;
    }
    // 保存当前标定点到磁盘
    void onSaveMarkers()
    {
        if (!m_scene)
            return;
        bool ok = m_scene->saveMarkers();
        qDebug() << "saveMarkers ->" << ok;
    }
    // 删除当前选中的标记
    void onDeleteSelected()
    {
        if (!m_scene)
            return;
        if (m_selectedMarkerId >= 0)
        {
            bool ok = m_scene->removeMarkerById(m_selectedMarkerId);
            qDebug() << "removeMarkerById(" << m_selectedMarkerId << ") ->" << ok;
            m_selectedMarkerId = -1;
            makeCurrent();
            createMeshes(m_scene->meshes());
            doneCurrent();
            update();
        }
    }
    // 删除所有标记
    void onClearAllMarkers()
    {
        if (!m_scene)
            return;
        m_scene->clearAllMarkers();
        makeCurrent();
        createMeshes(m_scene->meshes());
        doneCurrent();
        update();
    }
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

    // 试着加载位置数据（location_test.csv），用于把机器人模型移动到场景中事先定义的位置
    QStringList candLoc = {"test_config/location_test.csv", "src/test/test_config/location_test.csv", "..\\src\\test\\test_config\\location_test.csv", "./test_config/location_test.csv"};
    for (const QString &p : candLoc)
    {
        if (QFile::exists(p))
        {
            // 仅解析 CSV（loadLocationCsv 现在仅解析数据），随后显式应用第一行位置
            if (robot->loadLocationCsv(p))
            {
                // robot->applyLocationRow(0); // 直接把机器人放到 CSV 的第一行位置
                qDebug() << "Parsed and applied first location row from:" << p;
            }
            else
            {
                qDebug() << "Failed to parse location CSV:" << p;
            }
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

#include "test_scene_loc.moc"
