#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QFileInfo>
#include <QPainter>
#include <QDebug>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <QDir>

// 简单网格结构，保存从 Assimp 导入后的顶点、法线、UV、索引以及贴图信息
struct SimpleMesh
{
    std::vector<float> vertices;       // x,y,z
    std::vector<float> normals;        // x,y,z
    std::vector<float> texcoords;      // u,v
    std::vector<unsigned int> indices; // triangle indices
    QString diffuseTexPath;
    QImage diffuseImage; // for embedded textures
    unsigned int texId = 0;
};

// ModelViewer: 继承 QOpenGLWidget，负责加载模型、解析贴图并用固定管线渲染
class ModelViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    ModelViewer(const QString &path, QWidget *parent = nullptr) : QOpenGLWidget(parent), modelPath(path)
    {
        setMinimumSize(800, 600);
        setFocusPolicy(Qt::StrongFocus);
        loadSucceeded = loadModel(path.toStdString());
    }

    // load a mapping CSV that maps bone name -> joint id and axis 映射骨骼和关节ID以及对应的旋转轴
    bool loadBoneJointMapping(const QString &csvPath)
    {
        QFile f(csvPath);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qDebug() << "Failed open mapping" << csvPath;
            return false;
        }
        QTextStream ts(&f);
        QString header = ts.readLine(); // skip header
        while (!ts.atEnd())
        {
            QString line = ts.readLine().trimmed();
            if (line.isEmpty())
                continue;
            QStringList parts = line.split(',');
            if (parts.size() >= 2)
            {
                QString bone = parts[0].trimmed();    // 获取骨骼名称
                int jid = parts[1].trimmed().toInt(); // 获取关节ID
                char axis = 'x';
                if (parts.size() >= 3)
                    axis = parts[2].trimmed().isEmpty() ? 'x' : parts[2].trimmed().at(0).toLatin1(); // 获取旋转轴
                boneToJointMap[bone].push_back({jid, axis});
                // qDebug() << "Mapping bone to joint (append):" << bone << "->" << jid << "axis:" << axis;
            }
        }
        qDebug() << "Loaded bone-joint mapping entries:" << boneToJointMap.size();
        // apply mapping to bones array if already loaded
        for (auto &b : bones)
        {
            auto it = boneToJointMap.find(b.name);
            if (it != boneToJointMap.end())
            {
                b.jointMappings = it->second;
            }
        }
        return true;
    }

    // load place CSV (rows of angle1..angleN) and start simple playback timer 加载动作数据，这个数据后面从ros话题获取
    bool loadPlaceCsv(const QString &csvPath, int intervalMs = 100)
    {
        QFile f(csvPath);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qDebug() << "Failed open place.csv" << csvPath;
            return false;
        }
        QTextStream ts(&f);
        QString header = ts.readLine(); // assume header
        placeRows.clear();
        while (!ts.atEnd())
        {
            QString line = ts.readLine().trimmed();
            if (line.isEmpty())
                continue;
            // support comma or other separators
            QStringList parts = line.split(',');
            QVector<double> row;
            for (const QString &p : parts)
            {
                bool ok = false;
                double v = p.trimmed().toDouble(&ok);
                if (ok)
                    row.append(v);
                else
                    row.append(0.0);
            }
            placeRows.push_back(row);
            // qDebug() << "Loaded place row:" << row;
        }
        qDebug() << "Loaded place.csv rows:" << placeRows.size() << "cols(first):" << (placeRows.empty() ? 0 : placeRows[0].size());
        if (placeRows.empty())
            return false;
        // start timer
        if (!animTimer)
            animTimer = new QTimer(this);
        connect(animTimer, &QTimer::timeout, this, [this]()
                { advancePlaceFrame(); });
        animTimer->start(intervalMs);
            qDebug() << "loadPlaceCsv: starting animTimer; initialCameraDistanceSet=" << initialCameraDistanceSet << " initialCameraDistance=" << initialCameraDistance << " current cameraDistance=" << cameraDistance;
            // when animation playback starts, restore the initial camera distance captured at startup
            if (initialCameraDistanceSet) {
                cameraDistance = initialCameraDistance;
                qDebug() << "loadPlaceCsv: restored cameraDistance to initialCameraDistance=" << cameraDistance;
            }
            // lock cameraDistance from programmatic changes while playback runs (wheel zoom still allowed)
            cameraDistanceLockedDuringPlayback = true;
            qDebug() << "loadPlaceCsv: cameraDistanceLockedDuringPlayback=" << cameraDistanceLockedDuringPlayback;
        currentPlaceRow = 0;
        return true;
    }

protected:
    // 初始化OpenGL窗口
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);

        // 为每个 mesh 创建 OpenGL 纹理：
        // 优先使用嵌入式的 QImage（diffuseImage），否则尝试从解析到的 diffuseTexPath 加载文件。
        // 说明：QImage 的像素行起点通常在左上角，而 OpenGL 期望纹理原点在左下角，
        // 因此这里对 QImage 使用 mirrored() 做垂直翻转，使纹理方向与 UV 坐标一致。
        for (auto &m : meshes)
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
                }
                if (!img.isNull())
                {
                    QImage tex = img.convertToFormat(QImage::Format_RGBA8888).mirrored();
                    glGenTextures(1, &m.texId);
                    glBindTexture(GL_TEXTURE_2D, m.texId);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.width(), tex.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
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

    void resizeGL(int w, int h) override
    {
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = float(w) / float(qMax(1, h));
        // replace gluPerspective which may not be available: compute frustum
        const float fovDeg = 45.0f;
        const float fovRad = fovDeg * (3.14159265358979323846f / 180.0f);
        const float nearVal = 0.1f;
        const float farVal = 1000.0f;
        const float top = tanf(fovRad * 0.5f) * nearVal;
        const float bottom = -top;
        const float right = top * aspect;
        const float left = -right;
        glFrustum(left, right, bottom, top, nearVal, farVal);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    void paintGL() override
    {
    
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();
        // apply model-space centering + scale so vertices and bone transforms share same space
        if (fabs(modelCenterX) > 1e-9f || fabs(modelCenterY) > 1e-9f || fabs(modelCenterZ) > 1e-9f)
            glTranslatef(-modelCenterX, -modelCenterY, -modelCenterZ);
        if (fabs(modelScale - 1.0f) > 1e-9f)
            glScalef(modelScale, modelScale, modelScale);
        // simple camera (translate back then rotate view)
        glTranslatef(0, 0, -cameraDistance);
        glRotatef(rotX, 1, 0, 0);
        glRotatef(rotY, 0, 1, 0);

        // capture the initial camera distance at first paint (this is the actual view shown to user)
        if (!initialCameraDistanceSet) {
            initialCameraDistance = cameraDistance;
            initialCameraDistanceSet = true;
            qDebug() << "Captured initialCameraDistance:" << initialCameraDistance << " addr:" << (void*)&cameraDistance;
        }

        if (!loadSucceeded)
        {
            // 若模型加载失败，使用 QPainter 在窗口上绘制错误文本（QPainter 在 paintGL 中使用需在 gl 绘制后切换到 2D 绘制）
            QPainter p(this);
            p.setPen(Qt::black);
            p.drawText(rect(), Qt::AlignCenter, "Failed to load model:\n" + modelPath);
            p.end();
            return;
        }

        // draw meshes
        for (const SimpleMesh &m : meshes)
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
                    glTexCoord2f(m.texcoords[2 * ia], m.texcoords[2 * ia + 1]);
                glVertex3f(m.vertices[3 * ia], m.vertices[3 * ia + 1], m.vertices[3 * ia + 2]);
                // B
                if (!m.normals.empty())
                    glNormal3f(m.normals[3 * ib], m.normals[3 * ib + 1], m.normals[3 * ib + 2]);
                if (useTex)
                    glTexCoord2f(m.texcoords[2 * ib], m.texcoords[2 * ib + 1]);
                glVertex3f(m.vertices[3 * ib], m.vertices[3 * ib + 1], m.vertices[3 * ib + 2]);
                // C
                if (!m.normals.empty())
                    glNormal3f(m.normals[3 * ic], m.normals[3 * ic + 1], m.normals[3 * ic + 2]);
                if (useTex)
                    glTexCoord2f(m.texcoords[2 * ic], m.texcoords[2 * ic + 1]);
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

    // 鼠标事件
    void mousePressEvent(QMouseEvent *e) override
    {
        lastPos = e->pos();
    }
    void mouseMoveEvent(QMouseEvent *e) override
    {
        QPoint delta = e->pos() - lastPos;
        if (e->buttons() & Qt::LeftButton)
        {
            rotX += delta.y() * 0.5f;
            rotY += delta.x() * 0.5f;
            update();
        }
        lastPos = e->pos();
    }
    void wheelEvent(QWheelEvent *e) override
    {
        int delta = e->angleDelta().y();
        cameraDistance -= delta * 0.01f;
        cameraDistance = qMax(0.1f, cameraDistance);
        update();
    }

    // void keyPressEvent(QKeyEvent *e) override
    // {
    //     if (e->key() == Qt::Key_P)
    //     {
    //         printCurrentBoneAngles();

    //         // 调试内容

    //         QStringList candPlace = {
    //             QStringLiteral("test_config/place.csv"),
    //             QStringLiteral("src/test/test_config/place.csv"),
    //             QStringLiteral("..\\src\\test\\test_config\\place.csv"),
    //             QStringLiteral("./test_config/place.csv")};
    //         for (const QString &p : candPlace)
    //         {
    //             if (QFile::exists(p))
    //             {
    //                 loadPlaceCsv(p, 80);
    //                 qDebug() << "Loaded place.csv from" << p;
    //                 break;
    //             }
    //         }


    //         QStringList candMap = {
    //             QStringLiteral("test_config/boneToJoint.csv"),
    //             QStringLiteral("src/test/test_config/boneToJoint.csv"),
    //             QStringLiteral("..\\src\\test\\test_config\\boneToJoint.csv"),
    //             QStringLiteral("./test_config/boneToJoint.csv")};
    //         for (const QString &p : candMap)
    //         {
    //             if (QFile::exists(p))
    //             {
    //                 loadBoneJointMapping(p);
    //                 qDebug() << "Loaded mapping from" << p;
    //                 break;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         QOpenGLWidget::keyPressEvent(e);
    //     }
    // }

private:
    // 拷贝和计算节点变换、骨骼矩阵、点变换、轴旋转矩阵
    struct Mat4
    {
        float m[4][4];
        Mat4() { memset(m, 0, sizeof(m)); }
        static Mat4 identity()
        {
            Mat4 I;
            I.m[0][0] = I.m[1][1] = I.m[2][2] = I.m[3][3] = 1.0f;
            return I;
        }
        static Mat4 fromAi(const aiMatrix4x4 &a)
        {
            Mat4 r;
            r.m[0][0] = a.a1;
            r.m[0][1] = a.a2;
            r.m[0][2] = a.a3;
            r.m[0][3] = a.a4;
            r.m[1][0] = a.b1;
            r.m[1][1] = a.b2;
            r.m[1][2] = a.b3;
            r.m[1][3] = a.b4;
            r.m[2][0] = a.c1;
            r.m[2][1] = a.c2;
            r.m[2][2] = a.c3;
            r.m[2][3] = a.c4;
            r.m[3][0] = a.d1;
            r.m[3][1] = a.d2;
            r.m[3][2] = a.d3;
            r.m[3][3] = a.d4;
            return r;
        }
        Mat4 operator*(const Mat4 &o) const
        {
            Mat4 r;
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                {
                    r.m[i][j] = 0;
                    for (int k = 0; k < 4; k++)
                        r.m[i][j] += m[i][k] * o.m[k][j];
                }
            return r;
        }
        QVector3D transformPoint(const QVector3D &v) const
        {
            QVector3D r;
            r.setX(v.x() * m[0][0] + v.y() * m[0][1] + v.z() * m[0][2] + m[0][3]);
            r.setY(v.x() * m[1][0] + v.y() * m[1][1] + v.z() * m[1][2] + m[1][3]);
            r.setZ(v.x() * m[2][0] + v.y() * m[2][1] + v.z() * m[2][2] + m[2][3]);
            return r;
        }
        static Mat4 rotationX(float rad)
        {
            Mat4 r = Mat4::identity();
            float c = cosf(rad), s = sinf(rad);
            r.m[1][1] = c;
            r.m[1][2] = -s;
            r.m[2][1] = s;
            r.m[2][2] = c;
            return r;
        }
        static Mat4 rotationY(float rad)
        {
            Mat4 r = Mat4::identity();
            float c = cosf(rad), s = sinf(rad);
            r.m[0][0] = c;
            r.m[0][2] = s;
            r.m[2][0] = -s;
            r.m[2][2] = c;
            return r;
        }
        static Mat4 rotationZ(float rad)
        {
            Mat4 r = Mat4::identity();
            float c = cosf(rad), s = sinf(rad);
            r.m[0][0] = c;
            r.m[0][1] = -s;
            r.m[1][0] = s;
            r.m[1][1] = c;
            return r;
        }
        // compute determinant of top-left 3x3
        float det3() const
        {
            const float (&a)[4][4] = m;
            float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                      - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                      + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
            return det;
        }
        // return max row norm of top-left 3x3
        float maxRowNorm3() const
        {
            float maxn = 0.0f;
            for (int r = 0; r < 3; ++r) {
                float v0 = m[r][0], v1 = m[r][1], v2 = m[r][2];
                float n = sqrtf(v0*v0 + v1*v1 + v2*v2);
                if (n > maxn) maxn = n;
            }
            return maxn;
        }
        // build an orthonormal rotation matrix from the 3x3 part of src,
        // keep translation components from src. Uses Gram-Schmidt on columns.
        static Mat4 orthonormalizeRotationKeepTranslation(const Mat4 &src)
        {
            Mat4 out = src;
            // extract columns
            float c0x = src.m[0][0], c0y = src.m[1][0], c0z = src.m[2][0];
            float c1x = src.m[0][1], c1y = src.m[1][1], c1z = src.m[2][1];
            float c2x = src.m[0][2], c2y = src.m[1][2], c2z = src.m[2][2];

            auto norm = [](float x, float y, float z){ return sqrtf(x*x + y*y + z*z); };
            float n0 = norm(c0x,c0y,c0z);
            if (n0 < 1e-8f) {
                // fallback to identity rotation
                out = Mat4::identity();
                out.m[0][3] = src.m[0][3]; out.m[1][3] = src.m[1][3]; out.m[2][3] = src.m[2][3];
                return out;
            }
            float u0x = c0x / n0, u0y = c0y / n0, u0z = c0z / n0;
            // subtract projection of c1 onto u0
            float dot1 = c1x*u0x + c1y*u0y + c1z*u0z;
            float t1x = c1x - dot1*u0x, t1y = c1y - dot1*u0y, t1z = c1z - dot1*u0z;
            float n1 = norm(t1x,t1y,t1z);
            if (n1 < 1e-8f) {
                // create an arbitrary perpendicular vector
                if (fabs(u0x) < fabs(u0y)) {
                    t1x = 0; t1y = -u0z; t1z = u0y;
                } else {
                    t1x = -u0z; t1y = 0; t1z = u0x;
                }
                n1 = norm(t1x,t1y,t1z);
            }
            float u1x = t1x / n1, u1y = t1y / n1, u1z = t1z / n1;
            // u2 = cross(u0, u1)
            float u2x = u0y * u1z - u0z * u1y;
            float u2y = u0z * u1x - u0x * u1z;
            float u2z = u0x * u1y - u0y * u1x;
            // write back as columns
            out.m[0][0] = u0x; out.m[1][0] = u0y; out.m[2][0] = u0z;
            out.m[0][1] = u1x; out.m[1][1] = u1y; out.m[2][1] = u1z;
            out.m[0][2] = u2x; out.m[1][2] = u2y; out.m[2][2] = u2z;
            // keep translation
            out.m[0][3] = src.m[0][3]; out.m[1][3] = src.m[1][3]; out.m[2][3] = src.m[2][3];
            out.m[3][3] = 1.0f;
            return out;
        }
    };

    struct NodeInfo
    {
        QString name;
        Mat4 local; // local transform
        Mat4 originalLocal;
        int parent = -1;
        std::vector<int> children;
    };

    struct BoneInfo
    {
        QString name;
        Mat4 offset;        // offset matrix
        int nodeIndex = -1; // index into nodeInfos
        // allow multiple joint mappings per bone (bone can be driven by several joint IDs on different axes)
        std::vector<std::pair<int, char>> jointMappings; // (jointId, axis)
    };

    std::vector<NodeInfo> nodeInfos; // 拷贝了 Assimp 的节点层级到 nodeInfos,不再依赖 Importer 的生命周期
    std::map<QString, int> nodeNameToIndex;
    std::vector<BoneInfo> bones; // 骨骼列表
    // per-mesh per-vertex influences
    std::vector<std::vector<std::vector<std::pair<int, float>>>> meshesInfluences; // 每个 mesh 的每个顶点的骨骼影响
    // original unskinned vertex positions per mesh
    std::vector<std::vector<float>> originalMeshVertices; // 保存了未变形（原始）的顶点数组

    // mapping from bone name to one-or-more (joint id, axis) entries loaded from CSV
    std::map<QString, std::vector<std::pair<int, char>>> boneToJointMap; // 骨骼名称到关节ID和旋转轴的映射（允许多个）
    // place rows loaded from CSV
    std::vector<QVector<double>> placeRows;         // 对应动作数据的22个关节角度，从csv文件中解析的每一行，后面从话题获取需要单独处理
    int currentPlaceRow = 0;
    QTimer *animTimer = nullptr;
    // last applied joint angles (jointId -> degrees)
    std::map<int, double> currentJointAngles;       // 当前使用的关节角度

    // loadModel: 使用 Assimp 读取模型文件，提取网格数据和材质贴图（支持嵌入式和外链）
    bool loadModel(const std::string &file)
    {
        Assimp::Importer importer;
        // 导入模型数据
        const aiScene *scene = importer.ReadFile(file, aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
        if (!scene)
        {
            std::cerr << "Assimp load error: " << importer.GetErrorString() << std::endl;
            return false;
        }
        // 清理旧的网格数组，准备加载新模型的数据
        meshes.clear();
        // qDebug() << "Loading model:" << QString::fromStdString(file) << "\n"
        //     << "materials:" << scene->mNumMaterials << "\n"
        //     << "embedded_textures:" << scene->mNumTextures << "\n"
        //     << "meshes:" << scene->mNumMeshes;

        // -----------------------------
        // 解析材质贴图（per-material）
        // 步骤说明：
        // 1) 尝试读取材质的 DIFFUSE 贴图槽（常见位置）
        // 2) 若字符串以 '*' 开头，表示引用了 scene->mTextures 中的嵌入式纹理（格式可能为压缩数据或原始 RGBA）
        //    - 压缩数据用 QImage::fromData 解码
        //    - 原始 RGBA 数据直接构造 QImage 并 copy 一份，避免依赖 Assimp 内存
        // 3) 否则将路径视为外部文件引用：若为相对路径则相对于模型目录解析；若为绝对路径则清理后当作候选
        // 4) 所有解析结果保存在 matEmbeddedImages 或 matTexPaths，用于后续将材质贴图分配给 mesh
        // prepare material texture paths and embedded images
        std::vector<QString> matTexPaths; // 材质文件路径
        matTexPaths.resize(scene->mNumMaterials);
        std::vector<QImage> matEmbeddedImages; // 嵌入式纹理图像
        matEmbeddedImages.resize(scene->mNumMaterials);
        QFileInfo modelInfo(QString::fromStdString(file));
        QString modelDir = modelInfo.absolutePath();
        for (unsigned int mi = 0; mi < scene->mNumMaterials; ++mi)
        {
            aiString path;
            if (AI_SUCCESS == scene->mMaterials[mi]->GetTexture(aiTextureType_DIFFUSE, 0, &path))
            {
                std::string p = path.C_Str();
                if (!p.empty())
                {
                    // embedded textures are referenced as "*<index>" by Assimp
                    if (p.size() > 0 && p[0] == '*')
                    {
                        int texIdx = atoi(p.c_str() + 1);
                        if (texIdx >= 0 && texIdx < (int)scene->mNumTextures)
                        {
                            aiTexture *tex = scene->mTextures[texIdx];
                            QImage img;
                            if (tex->mHeight == 0)
                            {
                                // 压缩的纹理数据（png/jpg 等）以二进制形式存放在 pcData，长度在 mWidth 字段
                                // 使用 QImage::fromData 解码为 QImage
                                img = QImage::fromData(QByteArray(reinterpret_cast<const char *>(tex->pcData), (int)tex->mWidth));
                            }
                            else
                            {
                                // 未压缩的原始像素数据：Assimp 提供宽度 mWidth 和高度 mHeight，像素格式假定为 RGBA
                                // 直接用 QImage 包装后复制一份到本地内存，避免依赖 Assimp 的内存生命周期
                                QImage tmp(reinterpret_cast<const uchar *>(tex->pcData), tex->mWidth, tex->mHeight, QImage::Format_RGBA8888);
                                img = tmp.copy(); // copy to own storage
                            }
                            if (!img.isNull())
                                matEmbeddedImages[mi] = img;
                        }
                    }
                    else
                    {
                        // if path is a relative filename, resolve relative to model dir
                        if (p.size() > 0 && p[0] != '/' && p[0] != '\\' && p.find(":") == std::string::npos)
                        {
                            // 相对路径：通常相对于模型文件所在目录，构造 modelDir + 相对路径
                            QString full = modelDir + QDir::separator() + QString::fromStdString(p);
                            matTexPaths[mi] = full;
                            // qDebug() << "Material" << mi << "relative path ->" << matTexPaths[mi];
                        }
                        else
                        {
                            // 绝对路径或带盘符的路径：先做路径规范化（替换/\等），指向模型导出者的机器路径
                            QString orig = QString::fromStdString(p);
                            // normalize separators and clean path
                            orig = QDir::cleanPath(orig);
                            matTexPaths[mi] = orig;
                            // qDebug() << "Material" << mi << "orig path ->" << matTexPaths[mi];
                        }
                    }
                }
            }
        }

        // 处理网格
        for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
        {
            const aiMesh *mesh = scene->mMeshes[mi];
            // qDebug() << "Mesh" << mi << "information: "<< "\n------" << "verts:" << mesh->mNumVertices
            //                         << "\n------" << "faces:" << mesh->mNumFaces
            //                         << "\n------" << "hasTex:" << mesh->HasTextureCoords(0)
            //                         << "\n------" << "matIdx:" << mesh->mMaterialIndex;
            SimpleMesh sm;
            sm.vertices.reserve(mesh->mNumVertices * 3);
            if (mesh->HasNormals())
                sm.normals.reserve(mesh->mNumVertices * 3);
            bool hasTex = mesh->HasTextureCoords(0);
            if (hasTex)
                sm.texcoords.reserve(mesh->mNumVertices * 2);
            for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
            {
                aiVector3D vert = mesh->mVertices[v];
                sm.vertices.push_back(vert.x);
                sm.vertices.push_back(vert.y);
                sm.vertices.push_back(vert.z);
                if (mesh->HasNormals())
                {
                    aiVector3D n = mesh->mNormals[v];
                    sm.normals.push_back(n.x);
                    sm.normals.push_back(n.y);
                    sm.normals.push_back(n.z);
                }
                if (hasTex)
                {
                    aiVector3D uv = mesh->mTextureCoords[0][v];
                    sm.texcoords.push_back(uv.x);
                    sm.texcoords.push_back(uv.y);
                }
            }
            for (unsigned int f = 0; f < mesh->mNumFaces; ++f)
            {
                const aiFace &face = mesh->mFaces[f];
                if (face.mNumIndices == 3)
                {
                    sm.indices.push_back(face.mIndices[0]);
                    sm.indices.push_back(face.mIndices[1]);
                    sm.indices.push_back(face.mIndices[2]);
                }
            }
            meshes.push_back(std::move(sm));
        }
        // 处理材质
        // assign texture (embedded image or path) per mesh based on material index
        for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
        {
            const aiMesh *mesh = scene->mMeshes[mi];
            unsigned int matIdx = mesh->mMaterialIndex;
            if (matIdx < matEmbeddedImages.size() && !matEmbeddedImages[matIdx].isNull())
            {
                meshes[mi].diffuseImage = matEmbeddedImages[matIdx];
            }
            else if (matIdx < matTexPaths.size())
            {
                if (!matTexPaths[matIdx].isEmpty())
                {
                    QString p = matTexPaths[matIdx];
                    // 创建所有材质的候选路径并尝试解析实际文件
                    QVector<QString> candidates;
                    candidates.append(QDir::cleanPath(p));
                    // If p is relative (no drive or leading slash), try relative to modelDir
                    QFileInfo pi(p);
                    if (pi.isRelative())
                    {
                        candidates.append(QDir::cleanPath(modelDir + QDir::separator() + p));
                        candidates.append(QDir::cleanPath(modelDir + QDir::separator() + QDir("textures").filePath(pi.fileName())));
                        candidates.append(QDir::cleanPath(modelDir + QDir::separator() + QDir("assets").filePath(pi.fileName())));
                    }
                    else
                    {
                        // absolute: also try cleaning native separators
                        candidates.append(QDir::fromNativeSeparators(p));
                    }
                    // also try just the filename in model dir
                    candidates.append(QDir::cleanPath(modelDir + QDir::separator() + pi.fileName()));

                    QString chosen;
                    // qDebug() << "Material" << matIdx << "candidates path:" << candidates;
                    for (const QString &c : candidates)
                    {
                        if (QFile::exists(c))
                        {
                            chosen = c;
                            break;
                        }
                    }
                    if (!chosen.isEmpty())
                    {
                        meshes[mi].diffuseTexPath = chosen;
                        // qDebug() << "Mesh" << mi << "texture chosen actual path:" << chosen;
                    }
                    else
                    {
                        // keep original (cleaned) as last resort and log for debugging
                        meshes[mi].diffuseTexPath = QDir::cleanPath(p);
                        // qDebug() << "Texture file not found for material" << matIdx << "; tried candidates:" << candidates << "; using" << meshes[mi].diffuseTexPath;
                    }
                }
            }
        }
        // 居中缩放显示模型
        computeBounds();

        // 拷贝节点树；构建 bones 列表并把 weights 填入 meshesInfluences
        nodeInfos.clear();
        nodeNameToIndex.clear();
        std::function<void(aiNode *, int)> copyNode = [&](aiNode *node, int parentIdx)
        {
            int idx = (int)nodeInfos.size();
            nodeNameToIndex[QString::fromUtf8(node->mName.C_Str())] = idx;
            NodeInfo ninfo;
            ninfo.name = QString::fromUtf8(node->mName.C_Str());
            ninfo.parent = parentIdx;
            ninfo.local = Mat4::fromAi(node->mTransformation);
            // keep original local but orthonormalize rotation part to remove scale/shear from node transforms
            ninfo.originalLocal = Mat4::orthonormalizeRotationKeepTranslation(ninfo.local);
            nodeInfos.push_back(ninfo);
            if (parentIdx >= 0)
                nodeInfos[parentIdx].children.push_back(idx);
            for (unsigned int i = 0; i < node->mNumChildren; ++i)
                copyNode(node->mChildren[i], idx);
        };
        if (scene->mRootNode)
            copyNode(scene->mRootNode, -1);

        // prepare influences and bone list
        meshesInfluences.resize(scene->mNumMeshes);
        for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
            meshesInfluences[mi].resize(scene->mMeshes[mi]->mNumVertices);

        std::map<QString, int> boneNameToIndex;
        for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
        {
            const aiMesh *mesh = scene->mMeshes[mi];
            for (unsigned int bi = 0; bi < mesh->mNumBones; ++bi)
            {
                const aiBone *bone = mesh->mBones[bi];
                QString bname = QString::fromUtf8(bone->mName.C_Str());
                int bidx = -1;
                auto it = boneNameToIndex.find(bname);
                if (it == boneNameToIndex.end())
                {
                    bidx = (int)bones.size();
                    boneNameToIndex[bname] = bidx;
                    BoneInfo binfo;
                    binfo.name = bname;
                    binfo.offset = Mat4::fromAi(bone->mOffsetMatrix);
                    // NOTE: previously we orthonormalized the bone offset here to remove scale:
                    // binfo.offset = Mat4::orthonormalizeRotationKeepTranslation(binfo.offset);
                    // That can alter the inverse-bind (offset) matrix and cause the skinned mesh
                    // to no longer line up with the bone joint. Disable the orthonormalization by
                    // default to verify whether this was the cause of mesh offset. If needed,
                    // re-enable after investigation.
                    // map to node index if exists
                    auto nit = nodeNameToIndex.find(bname);
                    if (nit != nodeNameToIndex.end())
                        binfo.nodeIndex = nit->second;
                    // apply mapping if available
                    auto mit = boneToJointMap.find(bname);
                    if (mit != boneToJointMap.end())
                    {
                        binfo.jointMappings = mit->second;
                    }
                    bones.push_back(binfo);
                }
                else
                    bidx = it->second;
                // weights
                for (unsigned int wi = 0; wi < mesh->mNumBones; ++wi)
                {
                    // (we cannot iterate by bone->mNumWeights here, use bone->mNumWeights)
                }
                // actually register weights properly:
                for (unsigned int wi = 0; wi < bone->mNumWeights; ++wi)
                {
                    const aiVertexWeight &w = bone->mWeights[wi];
                    if (w.mVertexId < meshesInfluences[mi].size())
                        meshesInfluences[mi][w.mVertexId].push_back({bidx, (float)w.mWeight});
                }
            }
        }

        // save original vertices
        originalMeshVertices.clear();
        originalMeshVertices.resize(meshes.size());
        for (size_t i = 0; i < meshes.size(); ++i)
            originalMeshVertices[i] = meshes[i].vertices;

        // print bones info for debug: list bones, mapped node index, offset translation,
        // and how many vertices are influenced by each bone (quick sanity check)
        // qDebug() << "--- Bones summary after loadModel ---";
        for (size_t bi = 0; bi < bones.size(); ++bi) {
            const BoneInfo &b = bones[bi];
            // offset translation components
            float ox = b.offset.m[0][3];
            float oy = b.offset.m[1][3];
            float oz = b.offset.m[2][3];
            // count influenced vertices across all meshes
            size_t infCount = 0;
            for (size_t mi = 0; mi < meshesInfluences.size(); ++mi) {
                for (size_t vi = 0; vi < meshesInfluences[mi].size(); ++vi) {
                    const auto &inf = meshesInfluences[mi][vi];
                    for (const auto &p : inf) if (p.first == (int)bi) { ++infCount; break; }
                }
            }
            // qDebug() << "Bone" << (int)bi << "name=" << b.name << " nodeIndex=" << b.nodeIndex
            //          << " offsetTrans=(" << ox << "," << oy << "," << oz << ")" << " influencedVerts=" << (int)infCount;
        }
        // qDebug() << "--- end bones summary ---";

        return true;
    }

    // advance one frame from placeRows and apply to skeleton 提取关节数据，应用到骨骼
    void advancePlaceFrame()
    {
        if (placeRows.empty())
            return;
        const QVector<double> &row = placeRows[currentPlaceRow];        
        std::map<int, double> jointAngles;
        // 获取关节角度到jointAngles
        for (int i = 0; i < row.size(); ++i)
        {
            int jointId = i + 1; // angle1 -> joint 1
            jointAngles[jointId] = row[i];  
            // qDebug() << "Joint" << jointId << "angle(deg):" << row[i];    
        }
        applyJointAngles(jointAngles);
        currentPlaceRow = (currentPlaceRow + 1) % (int)placeRows.size();        // 导致循环播放动画
        // debug: report cameraDistance each frame to see if it changes during playback
        // qDebug() << "advancePlaceFrame: frame" << currentPlaceRow << " cameraDistance=" << cameraDistance;
    }

    // 应用关节角度到骨骼并更新网格顶点，显示动画
    // applyJointAngles: jointId->angle (degrees). This updates nodeInfos.local and performs CPU skinning to update mesh vertices, then triggers repaint
    void applyJointAngles(const std::map<int, double> &jointAngles)
    {
        // store current joint angles for debug/inspection
        currentJointAngles = jointAngles;
        // reset local to original
        for (auto &n : nodeInfos)
            n.local = n.originalLocal;
        // for each bone, if has joint mapping and node index, apply rotation about specified axis
        auto applyRotationToNodeLocal = [&](int nodeIdx, const Mat4 &Radd)
        {
            if (nodeIdx < 0 || nodeIdx >= (int)nodeInfos.size())
                return;
            const Mat4 &orig = nodeInfos[nodeIdx].originalLocal;
            // diagnostics: check if original local contains scale/non-orthogonality
            {
                float det = orig.det3();
                // compute row norms and check deviation from 1.0
                float maxRow = orig.maxRowNorm3();
                float minRow = 1e30f;
                for (int r=0;r<3;++r) {
                    float n = sqrtf(orig.m[r][0]*orig.m[r][0] + orig.m[r][1]*orig.m[r][1] + orig.m[r][2]*orig.m[r][2]);
                    if (n < minRow) minRow = n;
                }
                if (fabs(det - 1.0f) > 1e-3f || fabs(maxRow - 1.0f) > 1e-3f || fabs(minRow - 1.0f) > 1e-3f) {
                    QString nodeName = (nodeIdx >=0 && nodeIdx < (int)nodeInfos.size()) ? nodeInfos[nodeIdx].name : QString("<unknown>");
                    // qDebug() << "applyRotationToNodeLocal: Node" << nodeIdx << nodeName << "originalLocal DET=" << det << " rowNorms(min,max)=" << minRow << maxRow;
                }
            }
            // extract translation from original local (assume stored in m[0][3], m[1][3], m[2][3])
            float tx = orig.m[0][3];
            float ty = orig.m[1][3];
            float tz = orig.m[2][3];
            // extract original rotation 3x3
            Mat4 Rorig = Mat4::identity();
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    Rorig.m[r][c] = orig.m[r][c];
            // diagnostics on Rorig and incoming Radd
            {
                float detR = Rorig.det3();
                float detAdd = Radd.det3();
                float maxR = Rorig.maxRowNorm3();
                float maxAdd = Radd.maxRowNorm3();
                // if (fabs(detR - 1.0f) > 1e-3f || fabs(maxR - 1.0f) > 1e-3f) {
                //     QString nodeName = (nodeIdx >=0 && nodeIdx < (int)nodeInfos.size()) ? nodeInfos[nodeIdx].name : QString("<unknown>");
                //     // qDebug() << "applyRotationToNodeLocal: Rorig non-orthonormal for node" << nodeIdx << nodeName << "DET=" << detR << " maxRow=" << maxR;
                // }
                // if (fabs(detAdd - 1.0f) > 1e-3f || fabs(maxAdd - 1.0f) > 1e-3f) {
                //     // qDebug() << "applyRotationToNodeLocal: Radd unusual DET=" << detAdd << " maxRow=" << maxAdd;
                // }
            }
            // new rotation = Radd * Rorig (apply additional rotation in node-local coordinate before translation)
            Mat4 Rnew = Radd * Rorig;
            // compose local = T * Rnew
            Mat4 local = Mat4::identity();
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    local.m[r][c] = Rnew.m[r][c];
            local.m[0][3] = tx;
            local.m[1][3] = ty;
            local.m[2][3] = tz;
            local.m[3][3] = 1.0f;
            nodeInfos[nodeIdx].local = local;
        };

        for (size_t bi = 0; bi < bones.size(); ++bi)
        {
            const BoneInfo &b = bones[bi];
            // debug: show all mappings for this bone (jointId/axis pairs)
            // QStringList mapRepr;
            // for (const auto &jm : b.jointMappings)
            //     mapRepr.push_back(QString("%1/%2").arg(jm.first).arg(QChar(jm.second)));
            // qDebug() << "Processing bone:" << b.name << "mappings:" << (mapRepr.empty() ? QString("none") : mapRepr.join(";"));
            // if this bone has no joint mappings, skip
            if (b.jointMappings.empty())
                continue;
            // build an additive rotation from all mapped joint angles (order follows jointMappings vector)
            Mat4 Radd = Mat4::identity();
            bool anyApplied = false;
            for (const auto &jm : b.jointMappings)
            {
                int jid = jm.first;
                char axis = jm.second;
                auto it = jointAngles.find(jid);
                if (it == jointAngles.end())
                    continue;
                double angleDeg = it->second;
                float rad = (float)(angleDeg * (3.14159265358979323846 / 180.0));
                Mat4 Rm = Mat4::identity();
                if (axis == 'x' || axis == 'X')
                    Rm = Mat4::rotationX(rad);
                else if (axis == 'y' || axis == 'Y')
                    Rm = Mat4::rotationY(rad);
                else if (axis == 'z' || axis == 'Z')
                    Rm = Mat4::rotationZ(rad);
                // compose: new Radd = Rm * Radd so that the first mapping in vector is applied first
                Radd = Rm * Radd;
                anyApplied = true;
            }
            if (anyApplied && b.nodeIndex >= 0)
                applyRotationToNodeLocal(b.nodeIndex, Radd);
        }

        // compute global transforms (iterative traversal from roots to avoid recursion overhead)
        std::vector<Mat4> globalT(nodeInfos.size());
        // initialize
        for (size_t i = 0; i < nodeInfos.size(); ++i)
            globalT[i] = Mat4::identity();
        // process roots with simple queue
        std::vector<int> stack;
        for (int i = 0; i < (int)nodeInfos.size(); ++i)
            if (nodeInfos[i].parent == -1)
                stack.push_back(i);
        while (!stack.empty())
        {
            int idx = stack.back();
            stack.pop_back();
            int p = nodeInfos[idx].parent;
            if (p == -1)
                globalT[idx] = nodeInfos[idx].local;
            else
                globalT[idx] = globalT[p] * nodeInfos[idx].local;
            for (int c : nodeInfos[idx].children)
                stack.push_back(c);
        }

        // final bone matrices = global(node) * offset
        size_t nbones = bones.size();
        std::vector<float> flatBoneM(nbones * 12, 0.0f); // store only 3x4 rows [r0 r1 r2 r3; r4..] as 12 floats per bone (row-major 3x4)
        for (size_t bi = 0; bi < nbones; ++bi)
        {
            const BoneInfo &b = bones[bi];
            Mat4 F = Mat4::identity();
            if (b.nodeIndex >= 0 && b.nodeIndex < (int)globalT.size())
                F = globalT[b.nodeIndex] * b.offset;
            // diagnostics: check bone offset for scale and the resulting final matrix
            {
                float detOffset = b.offset.det3();
                float detF = F.det3();
                float maxOff = b.offset.maxRowNorm3();
                if (fabs(detOffset - 1.0f) > 1e-3f || fabs(maxOff - 1.0f) > 1e-3f) {
                    qDebug() << "Bone" << bi << b.name << "offset DET=" << detOffset << " maxRow=" << maxOff << " resulting F.DET=" << detF;
                }
                // also warn if resulting final matrix is non-orthonormal
                if (fabs(detF - 1.0f) > 1e-2f) {
                    qDebug() << "Warning: final bone matrix F for bone" << bi << b.name << "has DET=" << detF;
                }
            }
            // flatten first 3 rows and 4 cols -> 12 floats
            size_t off = bi * 12;
            flatBoneM[off + 0] = F.m[0][0];
            flatBoneM[off + 1] = F.m[0][1];
            flatBoneM[off + 2] = F.m[0][2];
            flatBoneM[off + 3] = F.m[0][3];
            flatBoneM[off + 4] = F.m[1][0];
            flatBoneM[off + 5] = F.m[1][1];
            flatBoneM[off + 6] = F.m[1][2];
            flatBoneM[off + 7] = F.m[1][3];
            flatBoneM[off + 8] = F.m[2][0];
            flatBoneM[off + 9] = F.m[2][1];
            flatBoneM[off + 10] = F.m[2][2];
            flatBoneM[off + 11] = F.m[2][3];

            // Debug diagnostics for bone 0 (help to verify alignment between bone transform and influenced mesh)
            if (bi == 0) {
                // qDebug() << "DEBUG: bone[0] name=" << b.name << " nodeIndex=" << b.nodeIndex;
                if (b.nodeIndex >= 0 && b.nodeIndex < (int)globalT.size()) {
                    const Mat4 &G = globalT[b.nodeIndex];
                    // qDebug() << " DEBUG: globalT translation =" << G.m[0][3] << G.m[1][3] << G.m[2][3];
                }
                // find a sample vertex influenced by this bone and print original->transformed by F
                bool foundSample = false;
                for (size_t smi = 0; smi < meshesInfluences.size() && !foundSample; ++smi) {
                    for (size_t svi = 0; svi < meshesInfluences[smi].size() && !foundSample; ++svi) {
                        const auto &inf = meshesInfluences[smi][svi];
                        for (size_t ik = 0; ik < inf.size(); ++ik) {
                            if (inf[ik].first == (int)bi) {
                                if (smi < originalMeshVertices.size()) {
                                    const auto &origV = originalMeshVertices[smi];
                                    if (origV.size() >= (svi * 3 + 3)) {
                                        float ox = origV[3 * svi + 0];
                                        float oy = origV[3 * svi + 1];
                                        float oz = origV[3 * svi + 2];
                                        float tx = ox * flatBoneM[off + 0] + oy * flatBoneM[off + 1] + oz * flatBoneM[off + 2] + flatBoneM[off + 3];
                                        float ty = ox * flatBoneM[off + 4] + oy * flatBoneM[off + 5] + oz * flatBoneM[off + 6] + flatBoneM[off + 7];
                                        float tz = ox * flatBoneM[off + 8] + oy * flatBoneM[off + 9] + oz * flatBoneM[off + 10] + flatBoneM[off + 11];
                                        // qDebug() << " DEBUG: sample mesh" << (int)smi << "vert" << (int)svi << "orig(" << ox << "," << oy << "," << oz << ") -> F(" << tx << "," << ty << "," << tz << ")";
                                    }
                                }
                                foundSample = true;
                                break;
                            }
                        }
                    }
                }
            }
        }

        // CPU skinning: for each mesh and vertex
        for (size_t mi = 0; mi < meshes.size(); ++mi)
        {
            SimpleMesh &m = meshes[mi];
            if (mi >= originalMeshVertices.size())
                continue;
            const std::vector<float> &orig = originalMeshVertices[mi];
            if (orig.size() / 3 != meshesInfluences[mi].size())
                continue;
            const size_t vcount = meshesInfluences[mi].size();
            float *out = m.vertices.data();
            const float *inp = orig.data();
            for (size_t vi = 0; vi < vcount; ++vi)
            {
                // safety: ensure we won't write out of bounds into mesh vertex buffer
                size_t outIdx = 3 * vi + 2;
                if (outIdx >= m.vertices.size()) {
                    qDebug() << "CPU skinning: out-of-bounds write prevented: mesh" << mi << "vi" << vi << "out.size" << m.vertices.size();
                    return; // abort applying this frame to avoid memory corruption
                }
                const float x = inp[3 * vi + 0];
                const float y = inp[3 * vi + 1];
                const float z = inp[3 * vi + 2];
                float ax = 0.0f, ay = 0.0f, az = 0.0f;
                float totalW = 0.0f;
                const auto &inf = meshesInfluences[mi][vi];
                for (size_t k = 0; k < inf.size(); ++k)
                {
                    int bidx = inf[k].first;
                    float w = inf[k].second;
                    if (bidx < 0 || (size_t)bidx >= nbones)
                        continue;
                    size_t off = (size_t)bidx * 12;
                    // transform point by 3x4 matrix
                    float tx = x * flatBoneM[off + 0] + y * flatBoneM[off + 1] + z * flatBoneM[off + 2] + flatBoneM[off + 3];
                    float ty = x * flatBoneM[off + 4] + y * flatBoneM[off + 5] + z * flatBoneM[off + 6] + flatBoneM[off + 7];
                    float tz = x * flatBoneM[off + 8] + y * flatBoneM[off + 9] + z * flatBoneM[off + 10] + flatBoneM[off + 11];
                    ax += tx * w;
                    ay += ty * w;
                    az += tz * w;
                    totalW += w;
                }
                if (totalW <= 1e-6f)
                {
                    ax = x;
                    ay = y;
                    az = z;
                }
                else if (fabs(totalW - 1.0f) > 1e-5f)
                {
                    ax /= totalW;
                    ay /= totalW;
                    az /= totalW;
                }
                out[3 * vi + 0] = ax;
                out[3 * vi + 1] = ay;
                out[3 * vi + 2] = az;
            }
        }

        // trigger repaint
        update();
    }

    // print current bone angles (uses currentJointAngles when available; otherwise estimates from local->original)
    void printCurrentBoneAngles()
    {
        qDebug() << "--- Current bone angles (deg) ---";
        const double RAD2DEG = 180.0 / 3.14159265358979323846;
        for (size_t bi = 0; bi < bones.size(); ++bi)
        {
            const BoneInfo &b = bones[bi];
            QString name = b.name;
            // if there are mapped joint entries, print each mapped joint's current angle (if available)
            if (!b.jointMappings.empty())
            {
                for (const auto &jm : b.jointMappings)
                {
                    int jid = jm.first;
                    char axis = jm.second;
                    auto it = currentJointAngles.find(jid);
                    if (it != currentJointAngles.end())
                    {
                        qDebug() << name << "mapped-> jointId=" << jid << "axis=" << axis << " angle=" << it->second;
                    }
                    else
                    {
                        qDebug() << name << "mapped-> jointId=" << jid << "axis=" << axis << " angle=N/A";
                    }
                }
                continue;
            }
            // fallback: estimate a single-angle approximation from node local vs originalLocal (old behavior)
            double angleDeg = NAN;
            char axis = 'x';
            if (b.nodeIndex >= 0 && b.nodeIndex < (int)nodeInfos.size())
            {
                const Mat4 &L = nodeInfos[b.nodeIndex].local;
                const Mat4 &O = nodeInfos[b.nodeIndex].originalLocal;
                // Rrel = L_rot * O_rot^T (rotation parts)
                float Rrel[3][3];
                // compute O_rot^T
                float Ot[3][3];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        Ot[r][c] = O.m[c][r];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                    {
                        Rrel[r][c] = 0.0f;
                        for (int k = 0; k < 3; ++k)
                            Rrel[r][c] += L.m[r][k] * Ot[k][c];
                    }
                // angle from trace
                float tr = Rrel[0][0] + Rrel[1][1] + Rrel[2][2];
                float cosang = (tr - 1.0f) * 0.5f;
                if (cosang > 1.0f)
                    cosang = 1.0f;
                if (cosang < -1.0f)
                    cosang = -1.0f;
                float ang = acosf(cosang);
                if (fabs(ang) < 1e-6f)
                    angleDeg = 0.0;
                else
                {
                    float denom = 2.0f * sinf(ang);
                    float ax = (Rrel[2][1] - Rrel[1][2]) / denom;
                    float ay = (Rrel[0][2] - Rrel[2][0]) / denom;
                    float az = (Rrel[1][0] - Rrel[0][1]) / denom;
                    float sign = 1.0f;
                    if (axis == 'x' || axis == 'X')
                        sign = (ax < 0.0f) ? -1.0f : 1.0f;
                    else if (axis == 'y' || axis == 'Y')
                        sign = (ay < 0.0f) ? -1.0f : 1.0f;
                    else if (axis == 'z' || axis == 'Z')
                        sign = (az < 0.0f) ? -1.0f : 1.0f;
                    angleDeg = ang * sign * RAD2DEG;
                }
            }
            if (std::isnan(angleDeg))
                qDebug() << name << " angle=N/A";
            else
                qDebug() << name << " angle=" << angleDeg;
        }
        qDebug() << "--- end angles ---";
    }

    // computeBounds: 计算模型包围盒并将模型居中归一化到一个合适的缩放范围
    void computeBounds()
    {
        bool first = true;
        float minx=0, miny=0, minz=0, maxx=0, maxy=0, maxz=0;
        for (const auto &m : meshes)
        {
            for (size_t i = 0; i < m.vertices.size(); i += 3)
            {
                float x = m.vertices[i], y = m.vertices[i + 1], z = m.vertices[i + 2];
                if (first)
                {
                    minx = maxx = x;
                    miny = maxy = y;
                    minz = maxz = z;
                    first = false;
                }
                minx = std::min(minx, x);
                maxx = std::max(maxx, x);
                miny = std::min(miny, y);
                maxy = std::max(maxy, y);
                minz = std::min(minz, z);
                maxz = std::max(maxz, z);
            }
        }
        if (!first)
        {
            float cx = (minx + maxx) / 2.0f;
            float cy = (miny + maxy) / 2.0f;
            float cz = (minz + maxz) / 2.0f;
            float diag = sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny) + (maxz - minz) * (maxz - minz));
            float scale = (diag > 0.0001f) ? (1.0f / diag) : 1.0f;
            // qDebug() << "computeBounds: center=(" << cx << "," << cy << "," << cz << ") diag=" << diag << " scale=" << scale;
            // store model centering and scale but DO NOT mutate mesh vertex buffers.
            modelCenterX = cx;
            modelCenterY = cy;
            modelCenterZ = cz;
            modelScale = scale;
            // default zoom computed after centering/scaling; only set on first compute to preserve user view when animation starts
            if (!initialCameraDistanceSet) {
                if (!cameraDistanceLockedDuringPlayback) {
                    cameraDistance = 2.0f; // default zoom
                    initialCameraDistance = cameraDistance;
                    initialCameraDistanceSet = true;
                    qDebug() << "computeBounds: set initialCameraDistance=" << initialCameraDistance;
                } else {
                    // if locked, keep previously captured initialCameraDistance
                    qDebug() << "computeBounds: cameraDistance locked during playback, skipping default zoom set";
                }
            }
        }
    }

    void getBonesInfo(const aiScene *scene)
    {
        // ---------- 打印骨骼信息（per-mesh bones） ----------
        qDebug() << "--- Bones per mesh ---";
        for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
        {
            const aiMesh *mesh = scene->mMeshes[mi];
            if (mesh->mNumBones > 0)
            {
                qDebug() << "Mesh" << mi << "has bones:" << mesh->mNumBones;
                for (unsigned int bi = 0; bi < mesh->mNumBones; ++bi)
                {
                    const aiBone *bone = mesh->mBones[bi];
                    qDebug() << "  Bone" << bi << ": name=" << bone->mName.C_Str() << " weights=" << bone->mNumWeights;
                    // 打印偏移矩阵
                    aiMatrix4x4 m = bone->mOffsetMatrix;
                    // qDebug() << "    offset matrix:";
                    // qDebug() << "      " << m.a1 << m.a2 << m.a3 << m.a4;
                    // qDebug() << "      " << m.b1 << m.b2 << m.b3 << m.b4;
                    // qDebug() << "      " << m.c1 << m.c2 << m.c3 << m.c4;
                    // qDebug() << "      " << m.d1 << m.d2 << m.d3 << m.d4;
                    // 打印若干权重信息（顶点索引和权重值）
                    // for (unsigned int wi = 0; wi < bone->mNumWeights; ++wi) {
                    //     const aiVertexWeight &w = bone->mWeights[wi];
                    //     // qDebug() << "      weight" << wi << ": vertexId=" << w.mVertexId << " w=" << w.mWeight;
                    // }
                }
            }
        }
        // ---------- 打印节点树（用于查看骨骼父子关系） ----------
        std::function<void(const aiNode *, int)> printNode = [&](const aiNode *node, int depth)
        {
            QString indent(depth * 2, ' ');
            qDebug() << indent + "node:" << node->mName.C_Str() << " children=" << node->mNumChildren << " meshes=" << node->mNumMeshes;
            // 如果该节点名称与任何 bone 名称匹配，标注出来
            for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
            {
                const aiMesh *mesh = scene->mMeshes[mi];
                for (unsigned int bi = 0; bi < mesh->mNumBones; ++bi)
                {
                    const aiBone *bone = mesh->mBones[bi];
                    if (QString(bone->mName.C_Str()) == QString(node->mName.C_Str()))
                    {
                        qDebug() << indent + "  (matches bone)";
                    }
                }
            }
            for (unsigned int i = 0; i < node->mNumChildren; ++i)
                printNode(node->mChildren[i], depth + 1);
        };
        qDebug() << "--- Scene node hierarchy ---";
        if (scene->mRootNode)
            printNode(scene->mRootNode, 0);
    }

    QString modelPath;
    std::vector<SimpleMesh> meshes;
    bool loadSucceeded = false;
    float rotX = -20.0f, rotY = 30.0f;
    float cameraDistance = 3.0f;
    // remember initial camera distance (the view scale when program first shows the model)
    float initialCameraDistance = 3.0f;
    bool initialCameraDistanceSet = false;
    // model-space centering and scale computed from computeBounds()
    float modelCenterX = 0.0f, modelCenterY = 0.0f, modelCenterZ = 0.0f;
    float modelScale = 1.0f;
    // when true, prevent non-interactive code from changing cameraDistance while animation is playing
    bool cameraDistanceLockedDuringPlayback = false;
    QPoint lastPos;
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QString model = QString::fromUtf8((argc > 1) ? argv[1] : "..\\assets\\Roban.fbx");
    QFile f(model);
    if (!f.exists())
    {
        std::cerr << "Model file not found: " << model.toStdString() << std::endl;
        std::cerr << "Please place the model in assets/Roban.fbx or pass full path as argument." << std::endl;
    }
    ModelViewer viewer(model);
    viewer.show();
    // Temporarily disabled: do not load bone->joint mapping or place.csv animation
    // (keeps model in its initial, static imported pose)
#if 1
    // try load mapping and place.csv from common candidate locations
    QStringList candMap = {
        QStringLiteral("test_config/boneToJoint.csv"),
        QStringLiteral("src/test/test_config/boneToJoint.csv"),
        QStringLiteral("..\\src\\test\\test_config\\boneToJoint.csv"),
        QStringLiteral("./test_config/boneToJoint.csv")
    };
    for (const QString &p : candMap) {
        if (QFile::exists(p)) { viewer.loadBoneJointMapping(p); qDebug() << "Loaded mapping from" << p; break; }
    }
    QStringList candPlace = {
        QStringLiteral("test_config/place.csv"),
        QStringLiteral("src/test/test_config/place.csv"),
        QStringLiteral("..\\src\\test\\test_config\\place.csv"),
        QStringLiteral("./test_config/place.csv")
    };
    for (const QString &p : candPlace) {
        if (QFile::exists(p)) { viewer.loadPlaceCsv(p, 80); qDebug() << "Loaded place.csv from" << p; break; }
    }
#endif
    return app.exec();
}

#include "test_model_bone.moc"
