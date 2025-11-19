#include "model_display/modelDisplay.h"
#include "util/load_csv.hpp"
#include <memory>

ModelDisplay::ModelDisplay(RobotManager *robot, SceneManager *scene, QWidget *parent)
    : QOpenGLWidget(parent), m_robot(robot), m_scene(scene)
{
    if (parent)
    {
        // 匹配控件的尺寸
        setMinimumSize(parent->size());
        setSizePolicy(parent->sizePolicy());
        resize(parent->size());
    }
    else
    {
        qDebug() << "ModelDisplay: No parent widget provided, using default size 320x240.";
        setMinimumSize(577, 429);
    }
    setFocusPolicy(Qt::StrongFocus);
   
   

    if (m_robot)
    {
        connect(m_robot, &RobotManager::frameAdvanced, this, QOverload<>::of(&ModelDisplay::update));
        connect(m_robot, &RobotManager::animationStarted, this, &ModelDisplay::onRobotAnimationStarted);
        // 拷贝模型居中显示的相关参数
        if (m_robot->loaded())
        {
            m_modelCenterX = m_robot->modelCenterX();
            m_modelCenterY = m_robot->modelCenterY();
            m_modelCenterZ = m_robot->modelCenterZ();
            m_modelScale = m_robot->modelScale();
            m_cameraDistance = m_robot->cameraDistance();
            m_cameraDistanceLockedDuringPlayback = m_robot->cameraDistanceLocked();
            // qDebug() << "ModelViewer: copied robot bounds center=(" << m_modelCenterX << m_modelCenterY << m_modelCenterZ << ") scale=" << m_modelScale;
        }
    }



    setDisplayWindow(); // 设置窗口界面
}

ModelDisplay::~ModelDisplay() = default;


// 设置工具栏
void ModelDisplay::setDisplayWindow()
{
    // 在 widget 内部创建水平工具栏（按钮行），放在窗口顶部
    m_topBar = new QWidget(this);
    m_topBar->setObjectName("calibTopBar");
    m_topBar->setAutoFillBackground(true);
    m_topBar->setStyleSheet("background-color: rgb(255, 255, 255); border:2px solid rgb(255, 255, 255); border-radius:15px");
    QHBoxLayout *hb = new QHBoxLayout(m_topBar);
    hb->setContentsMargins(4, 4, 4, 4);
    hb->setSpacing(6);

    // 定义按钮样式表，参考 robanweb.ui 中的按钮样式
    QString buttonStyle = "QPushButton {"
                          "    background-color: rgba(61, 112, 175, 100);"
                          "    color: rgb(255, 255, 255);"
                          "    font: 8pt \"微软雅黑\";"
                          "    border: 2px solid rgb(255, 255, 255);"
                          "    border-radius: 10px;"
                          "}"
                          "QPushButton:hover {"
                          "    background-color: rgb(0, 126, 185);"
                          "}"
                          "QPushButton:pressed {"
                          "    background-color: rgb(0, 126, 185);"
                          "}";


    // 创建按钮，按需连接到已有槽
    QPushButton *btnMap = new QPushButton(QString::fromUtf8("地图标定"), m_topBar);
    btnMap->setMinimumSize(75, 25);
    btnMap->setStyleSheet(buttonStyle);
    
    // QPushButton *btnPickup = new QPushButton(QString::fromUtf8("取货点标定"), m_topBar);
    // btnPickup->setMinimumSize(75, 25);
    // btnPickup->setStyleSheet(buttonStyle);
    
    // QPushButton *btnPlace = new QPushButton(QString::fromUtf8("放置点标定"), m_topBar);
    // btnPlace->setMinimumSize(75, 25);
    // btnPlace->setStyleSheet(buttonStyle);
    
    QPushButton *btnSave = new QPushButton(QString::fromUtf8("标定保存"), m_topBar);
    btnSave->setMinimumSize(75, 25);
    btnSave->setStyleSheet(buttonStyle);
    
    QPushButton *btnDelete = new QPushButton(QString::fromUtf8("标定删除"), m_topBar);
    btnDelete->setMinimumSize(75, 25);
    btnDelete->setStyleSheet(buttonStyle);
    
    QPushButton *btnClearAll = new QPushButton(QString::fromUtf8("全部删除"), m_topBar);
    btnClearAll->setMinimumSize(75, 25);
    btnClearAll->setStyleSheet(buttonStyle);
    
    QPushButton *btnSceneMapping = new QPushButton(QString::fromUtf8("场景映射"), m_topBar);
    btnSceneMapping->setMinimumSize(75, 25);
    btnSceneMapping->setStyleSheet(buttonStyle);

    QPushButton *btnPositonRefresh = new QPushButton(QString::fromUtf8("位置刷新"), m_topBar);
    btnPositonRefresh->setMinimumSize(75, 25);
    btnPositonRefresh->setStyleSheet(buttonStyle);

    QPushButton *btnPostionReset = new QPushButton(QString::fromUtf8("位置重置"), m_topBar);
    btnPostionReset->setMinimumSize(75, 25);
    btnPostionReset->setStyleSheet(buttonStyle);

    // 将按钮加入布局
    hb->addWidget(btnMap);
    // hb->addWidget(btnPickup);
    // hb->addWidget(btnPlace);
    hb->addWidget(btnSave);
    hb->addWidget(btnDelete);
    hb->addWidget(btnClearAll);
    hb->addWidget(btnSceneMapping);
    hb->addWidget(btnPositonRefresh);
    hb->addWidget(btnPostionReset);



    hb->addStretch();
    // 连接信号
    connect(btnMap, &QPushButton::clicked, this, &ModelDisplay::onAddMapMarker);
    // connect(btnPickup, &QPushButton::clicked, this, &ModelDisplay::onAddPickupMarker);
    // connect(btnPlace, &QPushButton::clicked, this, &ModelDisplay::onAddPlaceMarker);
    connect(btnSave, &QPushButton::clicked, this, &ModelDisplay::onSaveMarkers);
    connect(btnDelete, &QPushButton::clicked, this, &ModelDisplay::onDeleteSelected);
    connect(btnClearAll, &QPushButton::clicked, this, &ModelDisplay::onClearAllMarkers);
    connect(btnSceneMapping, &QPushButton::clicked, this,  &ModelDisplay::onSceneMapping);
    connect(btnPositonRefresh, &QPushButton::clicked, this, &ModelDisplay::onRefreshPositions);
    connect(btnPostionReset, &QPushButton::clicked, this, &ModelDisplay::onResetPositions);
    

    // 设置浮动信息标签
    m_tooltipLabel = new QLabel(this);
    m_tooltipLabel->setWindowFlags(Qt::ToolTip);
    m_tooltipLabel->setStyleSheet("background-color: rgba(240,248,255,1); border: 1px solid #888; border-radius:15px; padding: 4px; font-size: 11px;");
    m_tooltipLabel->hide();
    m_tooltipLabel->setAttribute(Qt::WA_TransparentForMouseEvents);

    // 初始布局（位置和高度）
    int th = btnMap->sizeHint().height() + 8;
    m_topBar->setGeometry(6, 6, width() - 12, th);
}

// 初始化OpenGL窗口
void ModelDisplay::initializeGL()
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

// 调整视口大小
void ModelDisplay::resizeGL(int w, int h)
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

// 渲染场景
void ModelDisplay::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    // 应用模型空间中心 + 缩放，使顶点和骨骼变换共享相同空间
    if (fabs(m_modelCenterX) > 1e-9f || fabs(m_modelCenterY) > 1e-9f || fabs(m_modelCenterZ) > 1e-9f)
        glTranslatef(-m_modelCenterX, -m_modelCenterY, -m_modelCenterZ);
    if (fabs(m_modelScale - 1.0f) > 1e-9f)
        glScalef(m_modelScale, m_modelScale, m_modelScale);
    // 简单相机（先平移再旋转视图）
    glTranslatef(0, 0, -m_cameraDistance);
    glRotatef(m_rotX, 1, 0, 0);
    glRotatef(m_rotY, 0, 1, 0);

    // 初次绘制时捕获初始相机距离
    if (!m_initialCameraDistanceSet)
    {
        m_initialCameraDistance = m_cameraDistance;
        m_initialCameraDistanceSet = true;
        qDebug() << "Captured initialCameraDistance:" << m_initialCameraDistance;
    }
    // 加载异常处理
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

    // 绘制场景网格（静态）
    if (m_scene && m_scene->loaded())
        drawMeshes(m_scene->meshes());

    // 绘制机器人网格
    if (m_robot)
        drawMeshes(m_robot->meshes());

    // 如果存在位置点，绘制轨迹线（绿色），使用 GL_LINE_STRIP 连续连接位置点
    if (m_robot)
    {
        SimpleMesh traj = m_robot->buildTrajectoryMesh();
        if (!traj.vertices.empty())
        {
            // 使用顶点颜色绘制轨迹，启用混合以支持 alpha
            glDisable(GL_TEXTURE_2D);
            glLineWidth(2.0f);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glBegin(GL_LINE_STRIP);
            size_t vcount = traj.vertices.size() / 3;
            bool hasColor = (traj.colors.size() / 4) >= vcount;
            for (size_t vi = 0; vi < vcount; ++vi)
            {
                if (hasColor) {
                    float r = traj.colors[4 * vi + 0];
                    float g = traj.colors[4 * vi + 1];
                    float b = traj.colors[4 * vi + 2];
                    float a = traj.colors[4 * vi + 3];
                    glColor4f(r, g, b, a);
                } else {
                    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
                }
                glVertex3f(traj.vertices[3 * vi + 0], traj.vertices[3 * vi + 1], traj.vertices[3 * vi + 2]);
            }
            glEnd();
            // 恢复状态
            glDisable(GL_BLEND);
            glColor3f(1.0f, 1.0f, 1.0f);
        }
    }
}

// 鼠标事件
void ModelDisplay::mousePressEvent(QMouseEvent *e)
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

void ModelDisplay::mouseMoveEvent(QMouseEvent *e)
{
    QPoint delta = e->pos() - m_lastPos;
    // 左键按住：旋转视角
    if (e->buttons() & Qt::LeftButton)
    {
        m_rotX += delta.y() * 0.5f;
        m_rotY += delta.x() * 0.5f;
        update();
    }
    // 右键按住：平移视角（左右/上下），通过修改模型中心实现
    else if (e->buttons() & Qt::RightButton)
    {
        // pan speed 与相机距离相关
        float panFactor = 0.0025f * m_cameraDistance;
        m_modelCenterX += delta.x() * panFactor; // 鼠标右移 -> 视点右移
        m_modelCenterY -= delta.y() * panFactor; // 鼠标上移 -> 视点上移
        update();
    }

    // 记录鼠标当前位置
    m_lastPos = e->pos();
}

void ModelDisplay::mouseReleaseEvent(QMouseEvent *event)
{
    // 左键短点击表示选择/拾取位置或标记
    if (event->button() == Qt::LeftButton && m_pressed)
    {
        QPoint delta = event->pos() - m_pressPos;
        if (delta.manhattanLength() < 6) // 认为是点击
        {
            QVector3D origin, dir;
            if (screenPosToRay(event->pos(), origin, dir))
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
    if (event->button() == Qt::RightButton)
    {
        QPoint delta = event->pos() - m_pressPos; 
        if (delta.manhattanLength() < 6)
        {
            QVector3D origin, dir;
            if (screenPosToRay(event->pos(), origin, dir))
            {
                if (!m_scene) {
                    qDebug() << "RightClick: m_scene is null, cannot pick.";
                } else {
                    // First try to intersect with scene geometry to get a scene-space point
                    QVector3D hit; // model-space coordinate
                    bool hitScene = m_scene->pickIntersect(origin, dir, hit);

                    // Also check if we hit a marker (for id)
                    QVector3D hitMarkerPt;
                    int mid = m_scene->pickMarkerByRay(origin, dir, hitMarkerPt);

                    if (hitScene)
                    {
                        // build tooltip text with scene-space coordinates and mapped robot coords (if available)
                        QString txt = QString("场景坐标:[x=%1,y=%2,z=%3]")
                                          .arg(hit.x(), 0, 'f', 4)
                                          .arg(hit.z(), 0, 'f', 4)
                                          .arg(hit.y(), 0, 'f', 4);

                        // map scene point to robot coords using SceneManager mapping (if available)
                        QVector2D robot;
                        if (m_scene->mapSceneToRobot(hit, robot)) {
                            txt += QString("\n机器人位置:[x=%1,y=%2]")
                                       .arg(robot.x(), 0, 'f', 4)
                                       .arg(robot.y(), 0, 'f', 4);
                        } else {
                            txt += "\nmapping: unavailable";
                        }

                        if (mid >= 0) {
                            txt = QString("id=%1\n").arg(mid) + txt;
                        }

                        QPoint pos = event->pos();
                        m_tooltipLabel->setText(txt);
                        m_tooltipLabel->adjustSize();
                        m_tooltipLabel->move(mapToGlobal(pos) + QPoint(10, -m_tooltipLabel->height() - 10));
                        m_tooltipLabel->show();
                        QTimer::singleShot(3000, m_tooltipLabel, &QLabel::hide);

                        qDebug() << "Right-click scene hit at" << hit << "markerId=" << mid;
                        // qDebug() << "Right-click robot loc at " <<  robot;
                    }
                    else if (mid >= 0)
                    {
                        // 场景平面的y对应空间坐标的z
                        QString txt = QString("id=%1\nx=%2\ny=%3\nz=%4")
                                          .arg(mid)
                                          .arg(hitMarkerPt.x(), 0, 'f', 3)
                                          .arg(hitMarkerPt.z(), 0, 'f', 3)
                                          .arg(hitMarkerPt.y(), 0, 'f', 3);
                        // 显示浮动标签
                        QPoint pos = event->pos();
                        m_tooltipLabel->setText(txt);
                        m_tooltipLabel->adjustSize();
                        m_tooltipLabel->move(mapToGlobal(pos) + QPoint(10, -m_tooltipLabel->height() - 10));
                        m_tooltipLabel->show();

                        // 3秒后隐藏
                        QTimer::singleShot(3000, m_tooltipLabel, &QLabel::hide);

                        qDebug() << "Right-click marker id=" << mid << " at " << hitMarkerPt;
                    }
                }
            }
        }
    }

    m_pressed = false;
    QOpenGLWidget::mouseReleaseEvent(event);
}

void ModelDisplay::wheelEvent(QWheelEvent *event)
{
    int d = event->angleDelta().y();
    m_cameraDistance -= d * 0.01f;
    m_cameraDistance = qMax(0.1f, m_cameraDistance);
    // qDebug() << "Mouse wheel: cameraDistance=" << m_cameraDistance;
    update();
}

// 运行时解析纹理路径
QString ModelDisplay::resolveTexturePathRuntime(const QString &path)
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
// 捕获鼠标射线
bool ModelDisplay::screenPosToRay(const QPoint &p, QVector3D &outOrigin, QVector3D &outDir)
{
    int w = width();
    int h = height();
    if (w <= 0 || h <= 0)
        return false;
    // 构建匹配 resizeGL 的投影矩阵
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

    // 构建匹配 paintGL 顺序的模型视图矩阵
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
    // 归一化设备坐标
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

// 机器人动画开始回调
void ModelDisplay::onRobotAnimationStarted()
{
    if (m_initialCameraDistanceSet)
    {
        m_cameraDistance = m_initialCameraDistance;
        // qDebug() << "ModelViewer: animation started, restored cameraDistance to" << m_cameraDistance;
    }
    m_cameraDistanceLockedDuringPlayback = true;
}

// 创建网格数据
void ModelDisplay::createMeshes(std::vector<SimpleMesh> &meshList)
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

// 绘制网格数据
void ModelDisplay::drawMeshes(const std::vector<SimpleMesh> &mes)
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

// 添加地图标记
void ModelDisplay::onAddMapMarker()
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

// 添加取货点标记
void ModelDisplay::onAddPickupMarker()
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

// 添加放置点标记
void ModelDisplay::onAddPlaceMarker()
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
// 保存标记到磁盘
void ModelDisplay::onSaveMarkers()
{
    if (!m_scene)
        return;
    bool ok = m_scene->saveMarkers();
    qDebug() << "saveMarkers ->" << ok;
}
// 删除当前选中的标记
void ModelDisplay::onDeleteSelected()
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
void ModelDisplay::onClearAllMarkers()
{
    if (!m_scene)
        return;
    m_scene->clearAllMarkers();
    makeCurrent();
    createMeshes(m_scene->meshes());
    doneCurrent();
    update();
}

// 场景映射按钮回调
void ModelDisplay::onSceneMapping()
{
    if (m_scene)
    {
        m_scene->SceneMapping();
    }
}

// 位置刷新按钮回调
void ModelDisplay::onRefreshPositions()
{
    if (m_robot)
    {
        m_robot->refreshRobotPositionsFromScene();
    }
}

void ModelDisplay::onResetPositions()
{
    if (m_robot)
    {
        m_robot->resetRobotPositions();
    }
}