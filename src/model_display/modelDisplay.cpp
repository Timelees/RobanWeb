#include "model_display/modelDisplay.h"

ModelDisplay::ModelDisplay(const QString &modelPath, QWidget *parent)
    : QOpenGLWidget(parent), m_modelPath(modelPath), rotY(30.0f), rotX(-20.0f), distance(3.0f)
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
    
    // 记录加载尝试并保存结果，便于运行时诊断
    qDebug() << "ModelDisplay: attempting to load model:" << modelPath;
    loadSucceeded = loadModel(modelPath.toStdString());     // 加载模型
    qDebug() << "ModelDisplay: loadSucceeded=" << loadSucceeded << " meshes=" << m_meshes.size();
  
}

ModelDisplay::~ModelDisplay() = default;

// 初始化OpenGL窗口
void ModelDisplay::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    qDebug() << "ModelDisplay::initializeGL called; meshes=" << m_meshes.size();

    // 给meshes创建OpenGL纹理
    for (auto &m : m_meshes)
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
                QImage tex = img.convertToFormat(QImage::Format_RGBA8888).mirrored(); // 垂直翻转，对齐纹理方向和UV坐标
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
                qDebug() << "ModelDisplay: texture missing for mesh; path:" << m.diffuseTexPath << " exists?" << exists << " embedded?" << !m.diffuseImage.isNull();
            }
        }
    }
}

// 调整视口大小
void ModelDisplay::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = float(w) / float(qMax(1,h));
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

// 渲染场景
void ModelDisplay::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
   
    // 设置相机视角
    glTranslatef(0, 0, -distance);
    glRotatef(rotX, 1, 0, 0);
    glRotatef(rotY, 0, 1, 0);

    if (!loadSucceeded)
    {
        // 若模型加载失败，使用 QPainter 在窗口上绘制错误文本（QPainter 在 paintGL 中使用需在 gl 绘制后切换到 2D 绘制）
        QPainter p(this);
        p.setPen(Qt::black);
        p.drawText(rect(), Qt::AlignCenter, "Failed to load model:\n" + m_modelPath);
        p.end();
        return;
    }

    // 绘制网格meshes
    for (const SimpleMesh &m : m_meshes)
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
void ModelDisplay::mousePressEvent(QMouseEvent *e)
{
    lastPos = e->pos();
    lastButton = e->button();
    // ensure widget receives keyboard focus so key events and focus dependent behavior work
    setFocus();
    e->accept();
}
void ModelDisplay::mouseMoveEvent(QMouseEvent *e)
{
    QPoint delta = e->pos() - lastPos;
    lastPos = e->pos();
    if (lastButton == Qt::LeftButton)
    {
        // pan
        panX += delta.x() * 0.002f; // scale to view
        panY -= delta.y() * 0.002f;
    }
    else if (lastButton == Qt::RightButton)
    {
        // rotate
        rotY += delta.x() * 0.5f;
        rotX += delta.y() * 0.5f;
    }
    update();
    e->accept();
}
void ModelDisplay::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    lastButton = Qt::NoButton;
    event->accept();
}
void ModelDisplay::wheelEvent(QWheelEvent *e)
{
    // zoom in/out
    int delta = e->angleDelta().y();
    if (delta > 0)
        distance = qMax(0.1f, distance - 0.1f);
    else
        distance += 0.1f;
    update();
    e->accept();
}

bool ModelDisplay::loadModel(const std::string &file)
{
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(file, aiProcess_CalcTangentSpace|aiProcess_Triangulate
                                                |aiProcess_JoinIdenticalVertices|aiProcess_SortByPType);

    if (!scene)
    {
        qDebug() << "Assimp failed to load model:" << importer.GetErrorString();
        return false;
    }

    // 清楚旧的网格数组
    m_meshes.clear();

    // 解析材质
    matTexPaths.resize(scene->mNumMaterials);
    matEmbeddedImages.resize(scene->mNumMaterials);
    QFileInfo modelInfo(QString::fromStdString(file));
    QString modelDir = modelInfo.absolutePath(); // 模型文件所在目录
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
                        qDebug() << "Material" << mi << "relative path ->" << matTexPaths[mi];
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
    }

    // 解析网格
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
    {
        const aiMesh *mesh = scene->mMeshes[mi];
        // qDebug() << "Mesh" << mi << "information: " << "\n------" << "verts:" << mesh->mNumVertices
        //          << "\n------" << "faces:" << mesh->mNumFaces
        //          << "\n------" << "hasTex:" << mesh->HasTextureCoords(0)
        //          << "\n------" << "matIdx:" << mesh->mMaterialIndex;
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
        m_meshes.push_back(std::move(sm));
    }

    // 处理材质
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
    {
        const aiMesh *mesh = scene->mMeshes[mi];
        unsigned int matIdx = mesh->mMaterialIndex;
        if (matIdx < matEmbeddedImages.size() && !matEmbeddedImages[matIdx].isNull())
        {
            m_meshes[mi].diffuseImage = matEmbeddedImages[matIdx];
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
                qDebug() << "Material" << matIdx << "candidates path:" << candidates;
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
                    m_meshes[mi].diffuseTexPath = chosen;
                    qDebug() << "Mesh" << mi << "texture chosen actual path:" << chosen;
                }
                else
                {
                    // keep original (cleaned) as last resort and log for debugging
                    m_meshes[mi].diffuseTexPath = QDir::cleanPath(p);
                    qDebug() << "Texture file not found for material" << matIdx << "; tried candidates:" << candidates << "; using" << m_meshes[mi].diffuseTexPath;
                }
            }
        }
    }
    // 居中缩放模型
    computeBounds();
    return true;
}

void ModelDisplay::computeBounds()
{
    bool first = true;
    float minx, miny, minz, maxx, maxy, maxz;
    for (const auto &m : m_meshes)
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
        // apply centering and scaling
        for (auto &m : m_meshes)
        {
            for (size_t i = 0; i < m.vertices.size(); i += 3)
            {
                m.vertices[i + 0] = (m.vertices[i + 0] - cx) * scale;
                m.vertices[i + 1] = (m.vertices[i + 1] - cy) * scale;
                m.vertices[i + 2] = (m.vertices[i + 2] - cz) * scale;
            }
        }
        distance = 2.0f; // default zoom
    }
}
