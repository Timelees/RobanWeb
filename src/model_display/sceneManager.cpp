#include "model_display/sceneManager.h"
#include "util/load_csv.hpp"
#include "ros_process/slamPose.h"

SceneManager::SceneManager(PoseMonitor *poseMonitor, const QString &modelPath, QObject *parent)
    : QObject(parent), m_modelPath(modelPath), m_poseMonitor(poseMonitor)
{
    init();

    if(m_poseMonitor){
        // 从ros话题获取位置消息
        connect(m_poseMonitor, &PoseMonitor::poseUpdated, this, [this](const QVector3D &p) {
            // 每次收到原始位姿时，先保存并立即发出动画专用信号。
            // animation subsystem (legs/bones) should follow high-rate updates.
            this->robotPose = p;
            
        });
    }
}


SceneManager::~SceneManager() {}



void SceneManager::init(){
    loadSucceeded = loadModel(m_modelPath.toStdString());   // 加载模型

    // 加载场景映射参数
    loadSceneMappingParameters();

 
}

void SceneManager::setPoseMonitor(PoseMonitor *poseMonitor)
{
    if (m_poseMonitor == poseMonitor)
        return;
    if (m_poseMonitor) {
        // best-effort: disconnect any existing connections (use QObject::disconnect)
        disconnect(m_poseMonitor, nullptr, this, nullptr);
    }
    m_poseMonitor = poseMonitor;
    if (m_poseMonitor) {
        connect(m_poseMonitor, &PoseMonitor::poseUpdated, this, [this](const QVector3D &p) {
            this->robotPose = p;
        });
    }
}

// 场景映射测试 (成员函数)，从csv解析数据
QVector<QVector3D> SceneManager::test_sceneCornerMapping(){
    QString poseCsvPath = resolveConfigPath("test_robot_cornerPose.csv");
    // 解析文件
    QFile f(poseCsvPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "SceneManager: fail open location csv" << poseCsvPath;
        return QVector<QVector3D>();
    }
    QTextStream ts(&f);
    // 临时容器用于保存解析到的角点/位姿
    QVector<QVector3D> corners;
    corners.clear();
    QString firstLine = ts.readLine(); // 读取并忽略标题行
    // 解析每一行位姿数据
    auto tryParseLine = [&](const QString &line){
        QString s = line.trimmed(); 
        if (s.isEmpty())
            return;
        QStringList parts = s.split(',');
        if(parts.size() < 3)    return;
        bool okx, oky, okyaw;
        double x = parts[0].trimmed().toDouble(&okx);
        double y = parts[1].trimmed().toDouble(&oky);
        double yaw = parts[2].trimmed().toDouble(&okyaw);
        if (okx && oky && okyaw){
            // qDebug() << "Parsed pose:" << x << y << yaw;
            // 在这里可以使用解析出的 x, y, yaw 值
            corners.append(QVector3D(float(x), float(y), float(yaw)));
        
        }
    };

    // 如果第一行看起来像数据则解析
    if (!firstLine.isEmpty() && (firstLine.at(0).isDigit() || firstLine.at(0) == QChar('-') || firstLine.startsWith("0")))  
        tryParseLine(firstLine);

    while (!ts.atEnd())
    {
        QString line = ts.readLine();
        tryParseLine(line);
    }
    f.close();

    // if(corners.size() !=4 ){
    //     qDebug() << "SceneManager: corner pose count !=4 , actual count:" << corners.size();
    //     return QVector<QVector3D>();
    // }
    return corners;
}


void SceneManager::loadSceneMappingParameters(){
    // 尝试从 config/SceneMapping.json 恢复已保存的仿射映射参数
    QDir appdir(QCoreApplication::applicationDirPath());
    QString cand = QDir::cleanPath(appdir.filePath(QString("../config/SceneMapping.json")));
    if (QFile::exists(cand)) {
        QFile f(cand);
        if (f.open(QIODevice::ReadOnly)) {
            QByteArray data = f.readAll();
            f.close();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            if (doc.isObject()) {
                QJsonObject root = doc.object();
                // 支持两种格式：root.params.{a,b,tx,c,d,ty} 或 root.{a,b,tx,c,d,ty}
                QJsonObject params;
                if (root.contains("params") && root["params"].isObject()) {
                    params = root["params"].toObject();
                } else {
                    // try top-level keys
                    params = QJsonObject();
                    QString keys[] = {"a","b","tx","c","d","ty"};
                    bool hasAny = false;
                    for (const QString &k : keys) {
                        if (root.contains(k)) { params[k] = root[k]; hasAny = true; }
                    }
                    if (!hasAny) params = QJsonObject();
                }
                if (!params.isEmpty()) {
                    // Ensure all required keys exist
                    QString keys[] = {"a","b","tx","c","d","ty"};
                    bool hasAll = true;
                    for (const QString &k : keys) {
                        if (!params.contains(k)) { hasAll = false; break; }
                    }
                    if (hasAll) {
                        double a = params.value("a").toDouble();
                        double b = params.value("b").toDouble();
                        double tx = params.value("tx").toDouble();
                        double c = params.value("c").toDouble();
                        double d = params.value("d").toDouble();
                        double ty = params.value("ty").toDouble();
                        m_map_a = a; m_map_b = b; m_map_tx = tx;
                        m_map_c = c; m_map_d = d; m_map_ty = ty;
                        m_hasMapping = true;
                        qDebug() << "SceneManager::init: loaded mapping params from" << cand;
                    }
                }
            }
        }
    }
}


bool SceneManager::loadModel(const std::string &file)
{

    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(file, aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
    if (!scene)
    {
        qDebug() << "SceneManager: Assimp load error:" << importer.GetErrorString();
        m_loaded = false;
        return false;
    }
    m_meshes.clear();

    QFileInfo modelInfo(QString::fromStdString(file));
    QString modelDir = modelInfo.absolutePath();

    // load per-material diffuse path / embedded images
    std::vector<QString> matTexPaths(scene->mNumMaterials);
    std::vector<QImage> matEmbeddedImages(scene->mNumMaterials);
    for (unsigned int mi = 0; mi < scene->mNumMaterials; ++mi)
    {
        aiString texPath;
        std::string rawP;
        if (AI_SUCCESS == scene->mMaterials[mi]->GetTexture(aiTextureType_DIFFUSE, 0, &texPath))
        {
            rawP = texPath.C_Str();
            std::string p = rawP;
            if (!p.empty())
            {
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
                        // 相对路径：通常相对于模型文件所在目录，但在不同运行目录/导出工具链下可能找不到。
                        // 我们尝试多个候选位置：modelDir/p、程序运行目录、当前工作目录，以及向上查找的 assets 子目录。
                        QString rel = QString::fromStdString(p);
                        auto tryPath = [&](const QString &base) -> QString {
                            if (base.isEmpty())
                                return QString();
                            QString cand = QDir::cleanPath(base + QDir::separator() + rel);
                            if (QFile::exists(cand))
                                return cand;
                            // also try base/assets/rel
                            QString cand2 = QDir::cleanPath(base + QDir::separator() + "assets" + QDir::separator() + rel);
                            if (QFile::exists(cand2))
                                return cand2;
                            return QString();
                        };

                        QString found;
                        // 1) try model directory directly
                        found = tryPath(modelDir);
                        // 2) try application directory
                        if (found.isEmpty())
                            found = tryPath(QCoreApplication::applicationDirPath());
                        // 3) try current working directory
                        if (found.isEmpty())
                            found = tryPath(QDir::currentPath());

                        // 4) as a last resort, walk up from modelDir and current dir looking for an "assets" folder
                        auto searchAssetsUp = [&](const QString &start) -> QString {
                            QDir d(start);
                            for (int depth = 0; depth < 6; ++depth)
                            {
                                if (!d.exists())
                                    break;
                                QString cand = QDir::cleanPath(d.absolutePath() + QDir::separator() + "assets" + QDir::separator() + rel);
                                if (QFile::exists(cand))
                                    return cand;
                                if (!d.cdUp())
                                    break;
                            }
                            return QString();
                        };

                        if (found.isEmpty())
                            found = searchAssetsUp(modelDir);
                        if (found.isEmpty())
                            found = searchAssetsUp(QDir::currentPath());

                        if (!found.isEmpty())
                        {
                            matTexPaths[mi] = QDir::cleanPath(found);
                            // qDebug() << "Material" << mi << "resolved relative path ->" << matTexPaths[mi];
                        }
                        else
                        {
                            // fallback: keep modelDir + rel (may be missing) so downstream code can report it
                            QString full = QDir::cleanPath(modelDir + QDir::separator() + rel);
                            matTexPaths[mi] = full;
                            // qDebug() << "Material" << mi << "relative path (fallback) ->" << matTexPaths[mi];
                        }
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
        // debug summary for this material: raw string, embedded? resolved path and whether file exists
        {
            bool embedded = (matEmbeddedImages[mi].isNull() == false);
            bool hasPath = (matTexPaths[mi].isEmpty() == false);
            bool exists = false;
            if (hasPath)
                exists = QFile::exists(matTexPaths[mi]);
            // qDebug() << "Material" << mi << "summary: raw='" << QString::fromStdString(rawP) << "' embedded=" << embedded << " path='" << matTexPaths[mi] << "' exists=" << exists;
        }
    }

    // iterate meshes
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
    {
        const aiMesh *mesh = scene->mMeshes[mi];
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
                // Assimp provides UV with origin at bottom-left in many exporters; we'll keep as-is
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

        // assign texture (material)
        unsigned int matIdx = mesh->mMaterialIndex;
        if (matIdx < matEmbeddedImages.size() && !matEmbeddedImages[matIdx].isNull())
        {
            sm.diffuseImage = matEmbeddedImages[matIdx];
        }
        else if (matIdx < matTexPaths.size() && !matTexPaths[matIdx].isEmpty())
        {
            sm.diffuseTexPath = matTexPaths[matIdx];
        }

        m_meshes.push_back(std::move(sm));
    }

    m_loaded = !m_meshes.empty();
    // always add a ground grid mesh under the scene for reference (Blender-like)
    // create after loading so it overlays (or underlays) scene geometry as desired
    createGridMesh(50.0f, 80, -0.001f);
    // qDebug() << "SceneManager: loaded meshes=" << m_meshes.size();
    return m_loaded;
}

// create a simple textured ground plane with a grid drawn into a QImage
// size: 地面在世界空间（模型单位）上的边长（正方形）, divisions: 把边长分成多少个格子（每轴的格子数）
// yOffset: 平面在 Y 方向上的微小偏移
void SceneManager::createGridMesh(float size, int divisions, float yOffset)
{
    if (divisions <= 0)
        divisions = 10;
    SimpleMesh sm;
    // plane centered at origin on XZ plane, y = yOffset
    float half = size * 0.5f;
    // 4 vertices
    // v0: (-half, y, -half)
    // v1: ( half, y, -half)
    // v2: ( half, y,  half)
    // v3: (-half, y,  half)
    sm.vertices = {
        -half, yOffset, -half,
        half, yOffset, -half,
        half, yOffset, half,
        -half, yOffset, half};
    // normals up
    sm.normals = {0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0};
    // simple UVs covering [0,1]
    sm.texcoords = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f};
    // two triangles
    sm.indices = {0, 1, 2, 2, 3, 0};

    // create a procedural grid texture
    const int texSize = 2048; // 纹理像素尺寸
    QImage img(texSize, texSize, QImage::Format_RGBA8888);
    img.fill(QColor(220, 220, 220, 255)); // 纹理背景色（浅灰）
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, false); // 关闭抗锯齿以获得更“像素对齐”的清晰直线
    // grid line colors
    QColor lineCol(140, 140, 140, 255);
    QColor majorCol(100, 100, 100, 255);
    // compute pixel step
    float step = float(texSize) / float(divisions); // 在像素空间中每格的像素间隔
    for (int i = 0; i <= divisions; ++i)
    {
        int x = int(i * step + 0.5f);
        int y = int(i * step + 0.5f);
        // 判断是否为“主”线：每 5 格作为粗一点/深一点的主线
        bool major = (i % 5 == 0) || (i == divisions / 2);
        p.setPen(QPen(major ? majorCol : lineCol, major ? 1 : 1));
        // vertical
        p.drawLine(x, 0, x, texSize);
        // horizontal
        p.drawLine(0, y, texSize, y);
    }
    p.end();

    sm.diffuseImage = img;

    // push grid as the first mesh so it's drawn before other scene meshes
    m_meshes.insert(m_meshes.begin(), std::move(sm));
}

// Moller-Trumbore ray-triangle intersection Möller–Trumbore 射线三角形相交函数
static bool rayTriangleIntersect(const QVector3D &orig, const QVector3D &dir,
                                 const QVector3D &v0, const QVector3D &v1, const QVector3D &v2,
                                 float &outT)
{
    const float EPS = 1e-6f;
    QVector3D edge1 = v1 - v0;
    QVector3D edge2 = v2 - v0;
    QVector3D pvec = QVector3D::crossProduct(dir, edge2);
    float det = QVector3D::dotProduct(edge1, pvec);
    if (det > -EPS && det < EPS)
        return false; // parallel
    float invDet = 1.0f / det;
    QVector3D tvec = orig - v0;
    float u = QVector3D::dotProduct(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f)
        return false;
    QVector3D qvec = QVector3D::crossProduct(tvec, edge1);
    float v = QVector3D::dotProduct(dir, qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f)
        return false;
    float t = QVector3D::dotProduct(edge2, qvec) * invDet;
    if (t <= EPS)
        return false;
    outT = t;
    return true;
}
// 对场景中三角形进行逐三角形检测（跳过插入的网格地面），返回最近交点（模型坐标）
bool SceneManager::pickIntersect(const QVector3D &rayOrigin, const QVector3D &rayDir, QVector3D &outHit) const
{
    bool found = false;
    float bestT = 1e30f;
    QVector3D bestHit;
    // skip the first mesh if it's the grid (we insert grid at begin)
    size_t startIdx = 0;
    if (!m_meshes.empty())
    {
        // heuristic: grid has 4 vertices and 6 indices
        if (m_meshes[0].vertices.size() == 12 && m_meshes[0].indices.size() == 6)
            startIdx = 1;
    }
    for (size_t mi = startIdx; mi < m_meshes.size(); ++mi)
    {
        const SimpleMesh &m = m_meshes[mi];
        const std::vector<float> &v = m.vertices;
        const std::vector<unsigned int> &idx = m.indices;
        for (size_t i = 0; i + 2 < idx.size(); i += 3)
        {
            unsigned int ia = idx[i];
            unsigned int ib = idx[i + 1];
            unsigned int ic = idx[i + 2];
            if (ia * 3 + 2 >= v.size() || ib * 3 + 2 >= v.size() || ic * 3 + 2 >= v.size())
                continue;
            QVector3D va(v[3 * ia], v[3 * ia + 1], v[3 * ia + 2]);
            QVector3D vb(v[3 * ib], v[3 * ib + 1], v[3 * ib + 2]);
            QVector3D vc(v[3 * ic], v[3 * ic + 1], v[3 * ic + 2]);
            float t;
            if (rayTriangleIntersect(rayOrigin, rayDir, va, vb, vc, t))
            {
                if (t < bestT)
                {
                    bestT = t;
                    bestHit = rayOrigin + rayDir * t;
                    found = true;
                }
            }
        }
    }
    if (found)
    {
        outHit = bestHit;
        return true;
    }
    return false;
}

// 生成简单 UV 球体网格并用一个小的纯色 QImage 作为漫反射贴图（绿色），将该球体加入场景网格列表中
// create a simple UV sphere mesh centered at pos with given radius and solid color texture
int SceneManager::addMarkerSphere(const QVector3D &pos, float radius, const QColor &color)
{
    SimpleMesh sm;
    const int lat = 10;
    const int lon = 12;
    for (int j = 0; j <= lat; ++j)
    {
        float v = float(j) / float(lat);
        float phi = v * M_PI; // 0..pi
        float sinPhi = sinf(phi);
        float cosPhi = cosf(phi);
        for (int i = 0; i <= lon; ++i)
        {
            float u = float(i) / float(lon);
            float theta = u * 2.0f * M_PI; // 0..2pi
            float sinTheta = sinf(theta);
            float cosTheta = cosf(theta);
            QVector3D p = QVector3D(cosTheta * sinPhi, cosPhi, sinTheta * sinPhi) * radius + pos;
            sm.vertices.push_back(p.x());
            sm.vertices.push_back(p.y());
            sm.vertices.push_back(p.z());
            // normals
            QVector3D n = (p - pos).normalized();
            sm.normals.push_back(n.x());
            sm.normals.push_back(n.y());
            sm.normals.push_back(n.z());
            // texcoords simple
            sm.texcoords.push_back(u);
            sm.texcoords.push_back(v);
        }
    }
    // indices
    for (int j = 0; j < lat; ++j)
    {
        for (int i = 0; i < lon; ++i)
        {
            int a = j * (lon + 1) + i;
            int b = a + lon + 1;
            int c = a + 1;
            int d = b + 1;
            sm.indices.push_back(a);
            sm.indices.push_back(b);
            sm.indices.push_back(c);
            sm.indices.push_back(c);
            sm.indices.push_back(b);
            sm.indices.push_back(d);
        }
    }

    // create a tiny solid-color texture so the sphere appears green
    QImage tex(4, 4, QImage::Format_RGBA8888);
    tex.fill(color);
    sm.diffuseImage = tex;

    m_meshes.push_back(std::move(sm));
    return int(m_meshes.size() - 1);
}

// --- 标记点管理实现 ---
// 内部：添加一个标注记录并生成对应 mesh
int SceneManager::addCalibrationMarker(MarkerType type, const QVector3D &pos, float radius, const QColor &color)
{
    // 创建球体 mesh，返回 mesh 索引
    int meshIdx = addMarkerSphere(pos, radius, color);
    Marker mk;
    mk.id = m_nextMarkerId++;
    mk.type = type;
    mk.pos = pos;
    mk.radius = radius;
    mk.meshIndex = meshIdx;
    mk.color = color;
    // 获取当前机器人位姿作为捕获时的位姿
    mk.robotPoseAtCapture = robotPose;
    m_markers.push_back(mk);
    return mk.id;
}

// 从磁盘读取 JSON 并创建标记（如果 path 为空则放在模型目录下的 calib_points.json）
bool SceneManager::loadMarkers(const QString &path)
{
    QString p = path;
    if (p.isEmpty())
    {
        qDebug() << "SceneManager::loadMarkers: no path provided, using config dir";
      
        QDir appdir(QCoreApplication::applicationDirPath());
        QString cand  = QDir::cleanPath(appdir.filePath(QString("../config/calib_points.json")));
        p = cand;
    }
    // qDebug() << "SceneManager::loadMarkers path=" << p;
    if (!QFile::exists(p))
    {
        qDebug() << "SceneManager::loadMarkers: file does not exist:" << p;
        return false;
    }
    QFile f(p);
    if (!f.open(QIODevice::ReadOnly))
        return false;
    QByteArray data = f.readAll();
    f.close();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray())
        return false;
    QJsonArray arr = doc.array();
    // clear existing markers (do not remove meshes to avoid reindex complexity)
    // we will still create new meshes for loaded markers
    for (const QJsonValue &v : arr)
    {
        if (!v.isObject()) continue;
        QJsonObject o = v.toObject();
        int t = o.value("type").toInt(0);
        double x = o.value("x").toDouble();
        double y = o.value("y").toDouble();
        double z = o.value("z").toDouble();
        double r = o.value("r").toDouble(0.05);
        QColor c(o.value("colorR").toInt(0), o.value("colorG").toInt(255), o.value("colorB").toInt(0));
        addCalibrationMarker(static_cast<MarkerType>(t), QVector3D(float(x), float(y), float(z)), float(r), c);
    }
    return true;
}

bool SceneManager::saveMarkers(const QString &path)
{
    QString p = path;
    if (p.isEmpty())
    {
       QDir appdir(QCoreApplication::applicationDirPath());
        QString cand  = QDir::cleanPath(appdir.filePath(QString("../config/calib_points.json")));
        p = cand;
    }
    qDebug() << "SceneManager::saveMarkers path=" << p;
    QJsonArray arr;
    for (const Marker &m : m_markers)
    {
        QJsonObject o;
        o["id"] = m.id;
        o["type"] = int(m.type);
        o["x"] = m.pos.x();
        o["y"] = m.pos.y();
        o["z"] = m.pos.z();
        o["r"] = m.radius;
        o["colorR"] = m.color.red();
        o["colorG"] = m.color.green();
        o["colorB"] = m.color.blue();
        arr.append(o);
    }
    QJsonDocument doc(arr);
    QFile f(p);
    QDir d = QFileInfo(p).absoluteDir();
    if (!d.exists())
    {
        if (!d.mkpath("."))
        {
            qDebug() << "SceneManager::saveMarkers: failed to create dir" << d.absolutePath();
            return false;
        }
    }
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        qDebug() << "SceneManager::saveMarkers: failed to open file for write" << p << "error:" << f.errorString();
        return false;
    }
    qint64 written = f.write(doc.toJson(QJsonDocument::Indented));
    f.close();
    qDebug() << "SceneManager::saveMarkers: wrote bytes=" << written << " to " << p;
    return written > 0;
}

bool SceneManager::removeMarkerById(int id)
{
    for (size_t i = 0; i < m_markers.size(); ++i)
    {
        if (m_markers[i].id == id)
        {
            int midx = m_markers[i].meshIndex;
            if (midx >= 0 && midx < (int)m_meshes.size())
            {
                // clear geometry so it won't draw
                m_meshes[midx].vertices.clear();
                m_meshes[midx].indices.clear();
                m_meshes[midx].normals.clear();
                m_meshes[midx].texcoords.clear();
                // free GL texture if any (defer to caller's GL context if needed)
                if (m_meshes[midx].texId != 0)
                {
                    // Note: glDeleteTextures must be called in GL context; here we just reset id
                    m_meshes[midx].texId = 0;
                }
            }
            m_markers.erase(m_markers.begin() + i);
            return true;
        }
    }
    return false;
}

// 使用简单点-射线距离判定拾取标记（更快且与球体形状一致）
int SceneManager::pickMarkerByRay(const QVector3D &rayOrigin, const QVector3D &rayDir, QVector3D &outHit) const
{
    int bestId = -1;
    float bestT = 1e30f;
    // qDebug() << "pickMarkerByRay: markers.count=" << m_markers.size() << " rayOrigin=" << rayOrigin << " rayDir=" << rayDir;
    for (const Marker &m : m_markers)
    {
        // 项目：计算射线到点的最近点参数 t
        QVector3D oc = m.pos - rayOrigin;
        float t = QVector3D::dotProduct(oc, rayDir);
        if (t <= 0) continue; // 在射线后方
        QVector3D closest = rayOrigin + rayDir * t;
        float dist2 = (closest - m.pos).lengthSquared();
        // qDebug() << "  marker id=" << m.id << " pos=" << m.pos << " radius=" << m.radius << " t=" << t << " dist2=" << dist2;
        if (dist2 <= m.radius * m.radius)
        {
            if (t < bestT)
            {
                bestT = t;
                bestId = m.id;
                outHit = closest;
            }
        }
    }
    // qDebug() << "  pick result bestId=" << bestId << " bestT=" << bestT << " outHit=" << outHit;
    return bestId;
}

void SceneManager::clearAllMarkers()
{
    // 清空所有与标记相关的 mesh 内容并释放记录
    for (const Marker &m : m_markers)
    {
        int midx = m.meshIndex;
        if (midx >= 0 && midx < (int)m_meshes.size())
        {
            m_meshes[midx].vertices.clear();
            m_meshes[midx].indices.clear();
            m_meshes[midx].normals.clear();
            m_meshes[midx].texcoords.clear();
            if (m_meshes[midx].texId != 0)
            {
                m_meshes[midx].texId = 0; // 实际 glDeleteTextures 由调用者在 GL 上下文中执行
            }
        }
    }
    m_markers.clear();
    // 同步保存到磁盘：将标记文件更新为当前（空）状态，确保 JSON 中不再包含已删除的标定点
    bool ok = saveMarkers();
    qDebug() << "SceneManager::clearAllMarkers: saved empty markers ->" << ok;
}


// 使用场景中的地图标记点计算从场景坐标到机器人坐标的仿射映射
void SceneManager::SceneMapping(){
    // 收集地图标记点
    std::vector<Marker> mapMarkers;
    for (const Marker &m : m_markers){
        if (m.type == Marker_Map)
            mapMarkers.push_back(m);
    }
    if (mapMarkers.size() < 4){
        qDebug() << "SceneManager::SceneMapping: need at least 4 map markers (have)" << mapMarkers.size();
        return;
    }

    // 使用前四个标记点（可以扩展为选择最佳四个）
    std::vector<Marker> sel(mapMarkers.begin(), mapMarkers.begin() + 4);

    // 构建从场景 (sx, sy) 到机器人 (rx, ry) 的最小二乘仿射映射
    // 模型: [rx]   [a b tx] [sx]
    //        [ry] = [c d ty] [sy]
    // For rx and ry separately solve p = argmin ||M * p - r||, where M = [sx sy 1]

    int N = 4;
    // compute M^T * M (3x3) and M^T * r for rx and ry
    double MtM[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double Mtrx[3] = {0,0,0};
    double Mtry[3] = {0,0,0};
    for (int i=0;i<N;++i){
    
        // 场景坐标系使用射线模型捕获的X和Z作为x和y坐标
        double sx = sel[i].pos.x();
        double sy = sel[i].pos.z();
        double rx = sel[i].robotPoseAtCapture.x();
        double ry = sel[i].robotPoseAtCapture.y();

        double row[3] = {sx, sy, 1.0};
        for (int r=0;r<3;++r){
            for (int c=0;c<3;++c){
                MtM[r][c] += row[r] * row[c];
            }
            Mtrx[r] += row[r] * rx;
            Mtry[r] += row[r] * ry;
        }
    }

    // invert 3x3 MtM
    auto invert3 = [&](double in[3][3], double out[3][3]) -> bool {
        // compute determinant and adjugate
        double a = in[0][0], b = in[0][1], c = in[0][2];
        double d = in[1][0], e = in[1][1], f = in[1][2];
        double g = in[2][0], h = in[2][1], i = in[2][2];
        double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        if (fabs(det) < 1e-12) return false;
        double invdet = 1.0 / det;
        out[0][0] =  (e*i - f*h) * invdet;
        out[0][1] = -(b*i - c*h) * invdet;
        out[0][2] =  (b*f - c*e) * invdet;
        out[1][0] = -(d*i - f*g) * invdet;
        out[1][1] =  (a*i - c*g) * invdet;
        out[1][2] = -(a*f - c*d) * invdet;
        out[2][0] =  (d*h - e*g) * invdet;
        out[2][1] = -(a*h - b*g) * invdet;
        out[2][2] =  (a*e - b*d) * invdet;
        return true;
    };

    double inv[3][3];
    if (!invert3(MtM, inv)){
        qDebug() << "SceneManager::SceneMapping: failed to invert MtM matrix (degenerate points)";
        return;
    }

    // compute params p = inv * M^T * r for rx and ry
    double px[3] = {0,0,0};
    double py[3] = {0,0,0};
    for (int r=0;r<3;++r){
        for (int c=0;c<3;++c){
            px[r] += inv[r][c] * Mtrx[c];
            py[r] += inv[r][c] * Mtry[c];
        }
    }
    double a = px[0], b = px[1], tx = px[2];
    double c_ = py[0], d = py[1], ty = py[2];

    // store mapping parameters in member fields so other code can map arbitrary scene points
    this->m_map_a = a;
    this->m_map_b = b;
    this->m_map_tx = tx;
    this->m_map_c = c_;
    this->m_map_d = d;
    this->m_map_ty = ty;
    this->m_hasMapping = true;

    // compute bounding rectangle in scene coordinates
    // compute bounding rectangle in scene XZ plane
    double minx = sel[0].pos.x(), maxx = sel[0].pos.x();
    double miny = sel[0].pos.z(), maxy = sel[0].pos.z();
    for (int i=1;i<N;++i){
        double sx = sel[i].pos.x();
        double sy = sel[i].pos.z();         // 场景的平面y对应模型空间的Z坐标
        minx = std::min(minx, sx); maxx = std::max(maxx, sx);
        miny = std::min(miny, sy); maxy = std::max(maxy, sy);
    }

    // grid step (meters) — 可根据需要调整或暴露为参数
    const double step = 0.05; // 5cm resolution

    // normalize start/end to step grid boundaries
    int ix0 = int(floor(minx / step));
    int ix1 = int(ceil(maxx / step));
    int iy0 = int(floor(miny / step));
    int iy1 = int(ceil(maxy / step));

    QJsonArray arr;
    for (int ix = ix0; ix <= ix1; ++ix){
        for (int iy = iy0; iy <= iy1; ++iy){
            double sx = ix * step;
            double sy = iy * step;                  // 场景的平面y对应模型空间的Z坐标
            double rx = a * sx + b * sy + tx;
            double ry = c_ * sx + d * sy + ty;
            QJsonObject o;
            o["scene_x"] = sx;
            // scene_y field stores the second scene-plane coordinate (Z in model space)
            o["scene_y"] = sy;
            o["robot_x"] = rx;
            o["robot_y"] = ry;
            arr.append(o);
        }
    }

    QJsonObject root;
    root["mapping"] = arr;
    QJsonArray corners;
    for (int i=0;i<N;++i){
        QJsonObject c;
        c["scene_x"] = sel[i].pos.x();
        c["scene_y"] = sel[i].pos.z();                      // 场景的平面y对应模型空间的Z坐标
        c["robot_x"] = sel[i].robotPoseAtCapture.x();
        c["robot_y"] = sel[i].robotPoseAtCapture.y();
        corners.append(c);
    }
    root["corners"] = corners;

    // 保存用于从 scene (X,Z) 到 robot (x,y) 的仿射参数，方便程序重启后恢复映射
    QJsonObject params;
    params["a"] = a;
    params["b"] = b;
    params["tx"] = tx;
    params["c"] = c_;
    params["d"] = d;
    params["ty"] = ty;
    // 还保存使用的网格步长与边界，便于外部工具参考
    params["grid_step"] = step;
    params["min_scene_x"] = minx;
    params["max_scene_x"] = maxx;
    params["min_scene_y"] = miny; // 注意: scene_y 对应模型空间的 Z
    params["max_scene_y"] = maxy;
    root["params"] = params;

    QJsonDocument doc(root);

    // write to config/SceneMapping.json next to application config
    QString outPath;
    QDir appdir(QCoreApplication::applicationDirPath());
    QString cand = QDir::cleanPath(appdir.filePath(QString("../config/SceneMapping.json")));
    QDir cfgdir = QFileInfo(cand).absoluteDir();
    if (!cfgdir.exists()){
        if (!cfgdir.mkpath(".")){
            qDebug() << "SceneManager::SceneMapping: failed to create config dir" << cfgdir.absolutePath();
            return;
        }
    }
    QFile f(cand);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        qDebug() << "SceneManager::SceneMapping: failed to open file for write" << cand << f.errorString();
        return;
    }
    qint64 written = f.write(doc.toJson(QJsonDocument::Indented));
    f.close();
    qDebug() << "SceneManager::SceneMapping: wrote" << written << "bytes to" << cand << "grid points=" << arr.size();
}

// 通过SceneMapping计算的参数将场景坐标映射到机器人坐标系
bool SceneManager::mapSceneToRobot(const QVector3D &scenePt, QVector2D &outRobot) const
{
    if (!m_hasMapping)
        return false;
    double sx = scenePt.x();
    double sz = scenePt.z();                            // 场景的平面y对应模型空间的Z坐标
    double rx = m_map_a * sx + m_map_b * sz + m_map_tx;
    double ry = m_map_c * sx + m_map_d * sz + m_map_ty;
    outRobot = QVector2D(float(rx), float(ry));
    return true;
}


// 将机器人坐标 (x,y) 映射回场景坐标 (模型空间)。
// 使用 SceneMapping 中保存的仿射参数的逆，返回场景平面上的 (X,Z)。
bool SceneManager::mapRobotToScene(const QVector2D &robotPt, QVector3D &outScene) const
{
    if (!m_hasMapping)
        return false;

    // A = [a b; c d], s = [sx; sz], r = [rx; ry] = A * s + t
    // => s = A^{-1} * (r - t)
    double a = m_map_a, b = m_map_b, c = m_map_c, d = m_map_d;
    double tx = m_map_tx, ty = m_map_ty;

    double det = a * d - b * c;
    if (fabs(det) < 1e-12)
        return false; // singular mapping

    double inv00 =  d / det;
    double inv01 = -b / det;
    double inv10 = -c / det;
    double inv11 =  a / det;

    double rx = robotPt.x();
    double ry = robotPt.y();

    double dx = rx - tx;
    double dy = ry - ty;

    double sx = inv00 * dx + inv01 * dy;
    double sz = inv10 * dx + inv11 * dy;

    // 将映射点放在场景平面上，y 分量（高度）设为 0.0f（可根据需要改为平均地面高度）
    outScene = QVector3D(float(sx), 0.0f, float(sz));
    return true;
}

// Map the currently stored robot pose (robotPose: x,y,yaw) to scene coords.
bool SceneManager::mapCurrentRobotPoseToScene(QVector3D &outScene) const
{
    // robotPose stores x,y,yaw; only x,y are used for planar mapping
    if (robotPose.isNull())
        return false;
    QVector2D rxy(robotPose.x(), robotPose.y());
    return mapRobotToScene(rxy, outScene);
}

