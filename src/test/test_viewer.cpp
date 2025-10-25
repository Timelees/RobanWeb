#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QFile>
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
struct SimpleMesh {
    std::vector<float> vertices; // x,y,z
    std::vector<float> normals;  // x,y,z
    std::vector<float> texcoords; // u,v
    std::vector<unsigned int> indices; // triangle indices
    QString diffuseTexPath;
    QImage diffuseImage; // for embedded textures
    unsigned int texId = 0;
};

// ModelViewer: 继承 QOpenGLWidget，负责加载模型、解析贴图并用固定管线渲染
class ModelViewer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    ModelViewer(const QString &path, QWidget *parent = nullptr) : QOpenGLWidget(parent), modelPath(path) {
        setMinimumSize(800,600);
        setFocusPolicy(Qt::StrongFocus);
        loadSucceeded = loadModel(path.toStdString());
    }

protected:
    // 初始化OpenGL窗口
    void initializeGL() override {
        initializeOpenGLFunctions();
        glClearColor(0.9f, 0.9f, 0.9f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);

    // 为每个 mesh 创建 OpenGL 纹理：
    // 优先使用嵌入式的 QImage（diffuseImage），否则尝试从解析到的 diffuseTexPath 加载文件。
    // 说明：QImage 的像素行起点通常在左上角，而 OpenGL 期望纹理原点在左下角，
    // 因此这里对 QImage 使用 mirrored() 做垂直翻转，使纹理方向与 UV 坐标一致。
        for (auto &m : meshes) {
            if (m.texId == 0) {
                QImage img;
                if (!m.diffuseImage.isNull()) {
                    img = m.diffuseImage;
                } else if (!m.diffuseTexPath.isEmpty()) {
                    img = QImage(m.diffuseTexPath);
                }
                if (!img.isNull()) {
                    QImage tex = img.convertToFormat(QImage::Format_RGBA8888).mirrored();
                    glGenTextures(1, &m.texId);
                    glBindTexture(GL_TEXTURE_2D, m.texId);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.width(), tex.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
                    glGenerateMipmap(GL_TEXTURE_2D);
                    glBindTexture(GL_TEXTURE_2D, 0);
                } else if (!m.diffuseTexPath.isEmpty() || !m.diffuseImage.isNull()) {
                    bool exists = false;
                    if (!m.diffuseTexPath.isEmpty()) exists = QFile::exists(m.diffuseTexPath);
                    qDebug() << "Failed to load texture image for mesh; path:" << m.diffuseTexPath << " exists?" << exists << " embedded?" << !m.diffuseImage.isNull();
                }
            }
        }
    }

    void resizeGL(int w, int h) override {
        glViewport(0,0,w,h);
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

    void paintGL() override {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        // simple camera
        glTranslatef(0, 0, -cameraDistance);
        glRotatef(rotX, 1, 0, 0);
        glRotatef(rotY, 0, 1, 0);

        if (!loadSucceeded) {
            // 若模型加载失败，使用 QPainter 在窗口上绘制错误文本（QPainter 在 paintGL 中使用需在 gl 绘制后切换到 2D 绘制）
            QPainter p(this);
            p.setPen(Qt::black);
            p.drawText(rect(), Qt::AlignCenter, "Failed to load model:\n" + modelPath);
            p.end();
            return;
        }

        // draw meshes
        for (const SimpleMesh &m : meshes) {
            bool useTex = (m.texId != 0) && (!m.texcoords.empty());
            if (useTex) {
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, m.texId);
            }
            glBegin(GL_TRIANGLES);
            for (size_t i = 0; i < m.indices.size(); i += 3) {
                unsigned int ia = m.indices[i];
                unsigned int ib = m.indices[i+1];
                unsigned int ic = m.indices[i+2];
                // A: 若存在法线则先提交法线，若使用纹理则先提交纹理坐标，再提交顶点位置
                // 注意：这里假设 texcoords 的每个顶点有一对 (u,v)，并且 v 不需要翻转（若贴图上下颠倒，可改为 1-v）
                if (!m.normals.empty()) glNormal3f(m.normals[3*ia], m.normals[3*ia+1], m.normals[3*ia+2]);
                if (useTex) glTexCoord2f(m.texcoords[2*ia], m.texcoords[2*ia+1]);
                glVertex3f(m.vertices[3*ia], m.vertices[3*ia+1], m.vertices[3*ia+2]);
                // B
                if (!m.normals.empty()) glNormal3f(m.normals[3*ib], m.normals[3*ib+1], m.normals[3*ib+2]);
                if (useTex) glTexCoord2f(m.texcoords[2*ib], m.texcoords[2*ib+1]);
                glVertex3f(m.vertices[3*ib], m.vertices[3*ib+1], m.vertices[3*ib+2]);
                // C
                if (!m.normals.empty()) glNormal3f(m.normals[3*ic], m.normals[3*ic+1], m.normals[3*ic+2]);
                if (useTex) glTexCoord2f(m.texcoords[2*ic], m.texcoords[2*ic+1]);
                glVertex3f(m.vertices[3*ic], m.vertices[3*ic+1], m.vertices[3*ic+2]);
            }
            glEnd();
            if (useTex) {
                glBindTexture(GL_TEXTURE_2D, 0);
                glDisable(GL_TEXTURE_2D);
            }
        }
    }

    // 鼠标事件
    void mousePressEvent(QMouseEvent *e) override {
        lastPos = e->pos();
    }
    void mouseMoveEvent(QMouseEvent *e) override {
        QPoint delta = e->pos() - lastPos;
        if (e->buttons() & Qt::LeftButton) {
            rotX += delta.y() * 0.5f;
            rotY += delta.x() * 0.5f;
            update();
        }
        lastPos = e->pos();
    }
    void wheelEvent(QWheelEvent *e) override {
        int delta = e->angleDelta().y();
        cameraDistance -= delta * 0.01f;
        cameraDistance = qMax(0.1f, cameraDistance);
        update();
    }

private:
    // loadModel: 使用 Assimp 读取模型文件，提取网格数据和材质贴图（支持嵌入式和外链）
    bool loadModel(const std::string &file) {
        Assimp::Importer importer;
        // 导入模型数据
        const aiScene* scene = importer.ReadFile(file,  aiProcess_CalcTangentSpace|aiProcess_Triangulate
                                                        |aiProcess_JoinIdenticalVertices|aiProcess_SortByPType);
        if (!scene) {
            std::cerr << "Assimp load error: " << importer.GetErrorString() << std::endl;
            return false;
        }
        // 清理旧的网格数组，准备加载新模型的数据
        meshes.clear();
        qDebug() << "Loading model:" << QString::fromStdString(file) << "\n"
            << "materials:" << scene->mNumMaterials << "\n"
            << "embedded_textures:" << scene->mNumTextures << "\n"
            << "meshes:" << scene->mNumMeshes;

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
        std::vector<QString> matTexPaths;           // 材质文件路径
        matTexPaths.resize(scene->mNumMaterials);
        std::vector<QImage> matEmbeddedImages;      // 嵌入式纹理图像
        matEmbeddedImages.resize(scene->mNumMaterials);
        QFileInfo modelInfo(QString::fromStdString(file));
        QString modelDir = modelInfo.absolutePath();
        for (unsigned int mi = 0; mi < scene->mNumMaterials; ++mi) {
            aiString path;
            if (AI_SUCCESS == scene->mMaterials[mi]->GetTexture(aiTextureType_DIFFUSE, 0, &path)) {
                std::string p = path.C_Str();
                if (!p.empty()) {
                    // embedded textures are referenced as "*<index>" by Assimp
                    if (p.size() > 0 && p[0] == '*') {
                        int texIdx = atoi(p.c_str()+1);
                        if (texIdx >= 0 && texIdx < (int)scene->mNumTextures) {
                            aiTexture* tex = scene->mTextures[texIdx];
                            QImage img;
                            if (tex->mHeight == 0) {
                                // 压缩的纹理数据（png/jpg 等）以二进制形式存放在 pcData，长度在 mWidth 字段
                                // 使用 QImage::fromData 解码为 QImage
                                img = QImage::fromData(QByteArray(reinterpret_cast<const char*>(tex->pcData), (int)tex->mWidth));
                            } else {
                                // 未压缩的原始像素数据：Assimp 提供宽度 mWidth 和高度 mHeight，像素格式假定为 RGBA
                                // 直接用 QImage 包装后复制一份到本地内存，避免依赖 Assimp 的内存生命周期
                                QImage tmp(reinterpret_cast<const uchar*>(tex->pcData), tex->mWidth, tex->mHeight, QImage::Format_RGBA8888);
                                img = tmp.copy(); // copy to own storage
                            }
                            if (!img.isNull()) matEmbeddedImages[mi] = img;
                        }
                    } else {
                        // if path is a relative filename, resolve relative to model dir
                        if (p.size() > 0 && p[0] != '/' && p[0] != '\\' && p.find(":") == std::string::npos) {
                                // 相对路径：通常相对于模型文件所在目录，构造 modelDir + 相对路径
                                QString full = modelDir + QDir::separator() + QString::fromStdString(p);
                                matTexPaths[mi] = full;
                                qDebug() << "Material" << mi << "relative path ->" << matTexPaths[mi];
                        } else {
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

        // 处理网格
        for (unsigned int mi=0; mi<scene->mNumMeshes; ++mi) {
            const aiMesh* mesh = scene->mMeshes[mi];
            qDebug() << "Mesh" << mi << "information: "<< "\n------" << "verts:" << mesh->mNumVertices 
                                    << "\n------" << "faces:" << mesh->mNumFaces 
                                    << "\n------" << "hasTex:" << mesh->HasTextureCoords(0) 
                                    << "\n------" << "matIdx:" << mesh->mMaterialIndex;
            SimpleMesh sm;
            sm.vertices.reserve(mesh->mNumVertices * 3);
            if (mesh->HasNormals()) sm.normals.reserve(mesh->mNumVertices * 3);
            bool hasTex = mesh->HasTextureCoords(0);
            if (hasTex) sm.texcoords.reserve(mesh->mNumVertices * 2);
            for (unsigned int v=0; v<mesh->mNumVertices; ++v) {
                aiVector3D vert = mesh->mVertices[v];
                sm.vertices.push_back(vert.x);
                sm.vertices.push_back(vert.y);
                sm.vertices.push_back(vert.z);
                if (mesh->HasNormals()) {
                    aiVector3D n = mesh->mNormals[v];
                    sm.normals.push_back(n.x);
                    sm.normals.push_back(n.y);
                    sm.normals.push_back(n.z);
                }
                if (hasTex) {
                    aiVector3D uv = mesh->mTextureCoords[0][v];
                    sm.texcoords.push_back(uv.x);
                    sm.texcoords.push_back(uv.y);
                }
            }
            for (unsigned int f=0; f<mesh->mNumFaces; ++f) {
                const aiFace &face = mesh->mFaces[f];
                if (face.mNumIndices == 3) {
                    sm.indices.push_back(face.mIndices[0]);
                    sm.indices.push_back(face.mIndices[1]);
                    sm.indices.push_back(face.mIndices[2]);
                }
            }
            meshes.push_back(std::move(sm));
        }
        // 处理材质
        // assign texture (embedded image or path) per mesh based on material index
        for (unsigned int mi=0; mi<scene->mNumMeshes; ++mi) {
            const aiMesh* mesh = scene->mMeshes[mi];
            unsigned int matIdx = mesh->mMaterialIndex;
            if (matIdx < matEmbeddedImages.size() && !matEmbeddedImages[matIdx].isNull()) {
                meshes[mi].diffuseImage = matEmbeddedImages[matIdx];
            } else if (matIdx < matTexPaths.size()) {
                if (!matTexPaths[matIdx].isEmpty()) {
                    QString p = matTexPaths[matIdx];
                    // 创建所有材质的候选路径并尝试解析实际文件
                    QVector<QString> candidates;
                    candidates.append(QDir::cleanPath(p));
                    // If p is relative (no drive or leading slash), try relative to modelDir
                    QFileInfo pi(p);
                    if (pi.isRelative()) {
                        candidates.append(QDir::cleanPath(modelDir + QDir::separator() + p));
                        candidates.append(QDir::cleanPath(modelDir + QDir::separator() + QDir("textures").filePath(pi.fileName())));
                        candidates.append(QDir::cleanPath(modelDir + QDir::separator() + QDir("assets").filePath(pi.fileName())));
                    } else {
                        // absolute: also try cleaning native separators
                        candidates.append(QDir::fromNativeSeparators(p));
                    }
                    // also try just the filename in model dir
                    candidates.append(QDir::cleanPath(modelDir + QDir::separator() + pi.fileName()));

                    QString chosen;
                    qDebug() << "Material" << matIdx << "candidates path:" << candidates;
                    for (const QString &c : candidates) {
                        if (QFile::exists(c)) { chosen = c; break; }
                    }
                    if (!chosen.isEmpty()) {
                        meshes[mi].diffuseTexPath = chosen;
                        qDebug() << "Mesh" << mi << "texture chosen actual path:" << chosen;
                    } else {
                        // keep original (cleaned) as last resort and log for debugging
                        meshes[mi].diffuseTexPath = QDir::cleanPath(p);
                        qDebug() << "Texture file not found for material" << matIdx << "; tried candidates:" << candidates << "; using" << meshes[mi].diffuseTexPath;
                    }
                }
            }
        }
        // center/scale: compute bounding box and center model
        computeBounds();
        return true;
    }

    // computeBounds: 计算模型包围盒并将模型居中归一化到一个合适的缩放范围
    void computeBounds() {
        bool first = true;
        float minx, miny, minz, maxx, maxy, maxz;
        for (const auto &m : meshes) {
            for (size_t i=0;i<m.vertices.size(); i+=3) {
                float x = m.vertices[i], y = m.vertices[i+1], z = m.vertices[i+2];
                if (first) { minx=maxx=x; miny=maxy=y; minz=maxz=z; first=false; }
                minx = std::min(minx, x); maxx = std::max(maxx, x);
                miny = std::min(miny, y); maxy = std::max(maxy, y);
                minz = std::min(minz, z); maxz = std::max(maxz, z);
            }
        }
        if (!first) {
            float cx = (minx+maxx)/2.0f;
            float cy = (miny+maxy)/2.0f;
            float cz = (minz+maxz)/2.0f;
            float diag = sqrt((maxx-minx)*(maxx-minx) + (maxy-miny)*(maxy-miny) + (maxz-minz)*(maxz-minz));
            float scale = (diag > 0.0001f) ? (1.0f/diag) : 1.0f;
            // apply centering and scaling
            for (auto &m : meshes) {
                for (size_t i=0;i<m.vertices.size(); i+=3) {
                    m.vertices[i+0] = (m.vertices[i+0] - cx) * scale;
                    m.vertices[i+1] = (m.vertices[i+1] - cy) * scale;
                    m.vertices[i+2] = (m.vertices[i+2] - cz) * scale;
                }
            }
            cameraDistance = 2.0f; // default zoom
        }
    }

    QString modelPath;
    std::vector<SimpleMesh> meshes;
    bool loadSucceeded = false;
    float rotX = -20.0f, rotY = 30.0f;
    float cameraDistance = 3.0f;
    QPoint lastPos;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QString model = QString::fromUtf8((argc>1)?argv[1]:"..\\assets\\Roban.fbx");
    QFile f(model);
    if (!f.exists()) {
        std::cerr << "Model file not found: " << model.toStdString() << std::endl;
        std::cerr << "Please place the model in assets/Roban.fbx or pass full path as argument." << std::endl;
    }
    ModelViewer viewer(model);
    viewer.show();
    return app.exec();
}

#include "test_viewer.moc"
