#include "model_display/sceneManager.h"
#include <QCoreApplication>

SceneManager::SceneManager(const QString &modelPath, QObject *parent)
    : QObject(parent), m_modelPath(modelPath)
{
    loadSucceeded = loadModel(modelPath.toStdString());
}

// non-inline destructor to ensure vtable is emitted in this TU
SceneManager::~SceneManager() {}

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
    qDebug() << "Loading model:" << QString::fromStdString(file) << "\n"
             << "materials:" << scene->mNumMaterials << "\n"
             << "embedded_textures:" << scene->mNumTextures << "\n"
             << "meshes:" << scene->mNumMeshes;

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
                            qDebug() << "Material" << mi << "resolved relative path ->" << matTexPaths[mi];
                        }
                        else
                        {
                            // fallback: keep modelDir + rel (may be missing) so downstream code can report it
                            QString full = QDir::cleanPath(modelDir + QDir::separator() + rel);
                            matTexPaths[mi] = full;
                            qDebug() << "Material" << mi << "relative path (fallback) ->" << matTexPaths[mi];
                        }
                    }
                    else
                    {
                        // 绝对路径或带盘符的路径：先做路径规范化（替换/\等），指向模型导出者的机器路径
                        QString orig = QString::fromStdString(p);
                        // normalize separators and clean path
                        orig = QDir::cleanPath(orig);
                        matTexPaths[mi] = orig;
                        qDebug() << "Material" << mi << "orig path ->" << matTexPaths[mi];
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
            qDebug() << "Material" << mi << "summary: raw='" << QString::fromStdString(rawP) << "' embedded=" << embedded << " path='" << matTexPaths[mi] << "' exists=" << exists;
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
    qDebug() << "SceneManager: loaded meshes=" << m_meshes.size();
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
