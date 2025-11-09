#include "model_display/robotManager.h"

RobotManager::RobotManager(const QString &modelPath, QObject *parent)
    : QObject(parent), m_modelPath(modelPath)
{
    loadSucceeded = loadModel(modelPath.toStdString());
}

bool RobotManager::loadBoneJointMapping(const QString &csvPath)
{
    QFile f(csvPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "RobotManager: fail open mapping" << csvPath;
        return false;
    }
    QTextStream ts(&f);
    QString header = ts.readLine(); // skip possible header
    while (!ts.atEnd())
    {
        QString line = ts.readLine().trimmed();
        if (line.isEmpty())
            continue;
        QStringList p = line.split(',');
        if (p.size() >= 2)
        {
            QString bone = p[0].trimmed();    // 获取骨骼名称
            int jid = p[1].trimmed().toInt(); // 获取关节ID
            char axis = 'x';
            if (p.size() >= 3 && !p[2].trimmed().isEmpty()) // 提取旋转轴
                axis = p[2].trimmed().at(0).toLatin1();
            m_boneToJoint[bone].push_back({jid, axis});
        }
    }
    qDebug() << "RobotManager: loaded mapping entries" << m_boneToJoint.size();
    // apply mapping to bones if already loaded
    for (auto &b : m_bones)
    {
        auto it = m_boneToJoint.find(b.name);
        if (it != m_boneToJoint.end())
            b.jointMappings = it->second;
    }
    return true;
}

bool RobotManager::loadPlaceCsv(const QString &csvPath, int intervalMs)
{
    QFile f(csvPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "RobotManager: fail open place.csv" << csvPath;
        return false;
    }
    QTextStream ts(&f);
    QString header = ts.readLine();
    m_placeRows.clear();
    while (!ts.atEnd())
    {
        QString line = ts.readLine().trimmed();
        if (line.isEmpty())
            continue;
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
        m_placeRows.push_back(row);
    }
    if (m_placeRows.empty())
        return false;

    if (!m_animTimer)
    {
        m_animTimer = new QTimer(this);
        connect(m_animTimer, &QTimer::timeout, this, &RobotManager::advancePlaceFrame);
    }
    m_animTimer->start(intervalMs);
    // mark that playback is active and notify viewers; viewers will decide how to adjust camera
    m_cameraDistanceLockedDuringPlayback = true;
    qDebug() << "RobotManager: started animation, rows=" << m_placeRows.size() << " cameraDistanceLockedDuringPlayback=" << m_cameraDistanceLockedDuringPlayback;

    emit animationStarted();
    m_currentRow = 0;

    return true;
}

void RobotManager::applyJointAngles(const std::map<int, double> &jointAngles)
{
    m_currentJointAngles = jointAngles;
    // diagnostic: print some nodeInfos (helpful to see root transforms)
    // qDebug() << "RobotManager: nodeInfos count=" << m_nodeInfos.size();
    // for (int i = 0; i < (int)m_nodeInfos.size() && i < 12; ++i) {
    //     const NodeInfo &ni = m_nodeInfos[i];
    //     qDebug() << " node[" << i << "] name=" << ni.name << " parent=" << ni.parent
    //              << " trans=(" << ni.local.m[0][3] << "," << ni.local.m[1][3] << "," << ni.local.m[2][3] << ")";
    // }

    // reset local to original
    for (auto &n : m_nodeInfos)
        n.local = n.originalLocal;

    // helper to apply rotation matrix to a node's local transform
    auto applyRotationToNodeLocal = [&](int nodeIdx, const Mat4 &Radd)
    {
        if (nodeIdx < 0 || nodeIdx >= (int)m_nodeInfos.size())
            return;
        const Mat4 &orig = m_nodeInfos[nodeIdx].originalLocal;

        // {
        //     float det = orig.det3();

        //     float maxRow = orig.maxRowNorm3();
        //     float minRow = 1e30f;
        //     for(int r = 0; r < 3; ++r){
        //         float n = sqrtf(orig.m[r][0]*orig.m[r][0] + orig.m[r][1]*orig.m[r][1] + orig.m[r][2]*orig.m[r][2]);
        //         if (n < minRow) minRow = n;
        //     }
        //     if (fabs(det - 1.0f) > 1e-3f || fabs(maxRow - 1.0f) > 1e-3f || fabs(minRow - 1.0f) > 1e-3f) {
        //         QString nodeName = (nodeIdx >=0 && nodeIdx < (int)m_nodeInfos.size()) ? m_nodeInfos[nodeIdx].name : QString("<unknown>");
        //         // qDebug() << "applyRotationToNodeLocal: Node" << nodeIdx << nodeName << "originalLocal DET=" << det << " rowNorms(min,max)=" << minRow << maxRow;
        //     }

        // }

        // extract translation
        float tx = orig.m[0][3];
        float ty = orig.m[1][3];
        float tz = orig.m[2][3];
        // extract rotation (3x3)
        Mat4 Rorig = Mat4::identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Rorig.m[r][c] = orig.m[r][c];

        // {
        // float detR = Rorig.det3();
        // float detAdd = Radd.det3();
        // float maxR = Rorig.maxRowNorm3();
        // float maxAdd = Radd.maxRowNorm3();
        // if (fabs(detR - 1.0f) > 1e-3f || fabs(maxR - 1.0f) > 1e-3f) {
        //     QString nodeName = (nodeIdx >=0 && nodeIdx < (int)nodeInfos.size()) ? nodeInfos[nodeIdx].name : QString("<unknown>");
        //     // qDebug() << "applyRotationToNodeLocal: Rorig non-orthonormal for node" << nodeIdx << nodeName << "DET=" << detR << " maxRow=" << maxR;
        // }
        // if (fabs(detAdd - 1.0f) > 1e-3f || fabs(maxAdd - 1.0f) > 1e-3f) {
        //     // qDebug() << "applyRotationToNodeLocal: Radd unusual DET=" << detAdd << " maxRow=" << maxAdd;
        // }
        // }

        Mat4 Rnew = Radd * Rorig;
        Mat4 local = Mat4::identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                local.m[r][c] = Rnew.m[r][c];
        local.m[0][3] = tx;
        local.m[1][3] = ty;
        local.m[2][3] = tz;
        local.m[3][3] = 1.0f;
        m_nodeInfos[nodeIdx].local = local;
    };

    // for each bone, build additive rotation from mapped joint angles
    for (size_t bi = 0; bi < m_bones.size(); ++bi)
    {
        const BoneInfo &b = m_bones[bi];
        if (b.jointMappings.empty())
            continue;
        Mat4 Radd = Mat4::identity();
        bool anyApplied = false;
        for (const auto &jm : b.jointMappings)
        {
            int jid = jm.first;
            char axis = jm.second;
            auto it = m_currentJointAngles.find(jid);
            if (it == m_currentJointAngles.end())
                continue;
            double deg = it->second;
            float rad = (float)(deg * (3.14159265358979323846 / 180.0)); // 角度旋转
            Mat4 Raxis = Mat4::identity();
            if (axis == 'x' || axis == 'X')
                Raxis = Mat4::rotationX(rad);
            else if (axis == 'y' || axis == 'Y')
                Raxis = Mat4::rotationY(rad);
            else if (axis == 'z' || axis == 'Z')
                Raxis = Mat4::rotationZ(rad);
            else
                Raxis = Mat4::rotationZ(rad);
            Radd = Raxis * Radd; // apply in order
            anyApplied = true;
        }
        if (anyApplied && b.nodeIndex >= 0)
            applyRotationToNodeLocal(b.nodeIndex, Radd);
    }

    // compute global transforms
    std::vector<Mat4> globalT(m_nodeInfos.size());
    // initialize
    for (size_t i = 0; i < m_nodeInfos.size(); ++i)
        globalT[i] = Mat4::identity();
    // collect roots
    std::vector<int> stack;
    for (int i = 0; i < (int)m_nodeInfos.size(); ++i)
        if (m_nodeInfos[i].parent == -1)
            stack.push_back(i);
    while (!stack.empty())
    {
        int idx = stack.back();
        stack.pop_back();
        int p = m_nodeInfos[idx].parent;
        if (p == -1)
            globalT[idx] = m_nodeInfos[idx].local;
        else
            globalT[idx] = globalT[p] * m_nodeInfos[idx].local;
        for (int c : m_nodeInfos[idx].children)
            stack.push_back(c);
    }

    // final bone matrices = global(node) * offset
    size_t nbones = m_bones.size();
    m_cachedFlatBoneM.clear();
    m_cachedFlatBoneM.resize(nbones * 12);
    float *flat = m_cachedFlatBoneM.data();
    for (size_t bi = 0; bi < nbones; ++bi)
    {
        Mat4 F = Mat4::identity();
        const BoneInfo &b = m_bones[bi];
        if (b.nodeIndex >= 0 && b.nodeIndex < (int)globalT.size())
            F = globalT[b.nodeIndex] * b.offset;
        size_t off = bi * 12;
        flat[off + 0] = F.m[0][0];
        flat[off + 1] = F.m[0][1];
        flat[off + 2] = F.m[0][2];
        flat[off + 3] = F.m[0][3];
        flat[off + 4] = F.m[1][0];
        flat[off + 5] = F.m[1][1];
        flat[off + 6] = F.m[1][2];
        flat[off + 7] = F.m[1][3];
        flat[off + 8] = F.m[2][0];
        flat[off + 9] = F.m[2][1];
        flat[off + 10] = F.m[2][2];
        flat[off + 11] = F.m[2][3];
    }

    // diagnostic: print bone final transforms for first few bones
    // for (size_t bi = 0; bi < nbones && bi < 12; ++bi)
    // {
    //     const BoneInfo &b = m_bones[bi];
    //     Mat4 F = Mat4::identity();
    //     if (b.nodeIndex >= 0 && b.nodeIndex < (int)globalT.size())
    //         F = globalT[b.nodeIndex] * b.offset;
    //     qDebug() << "Bone" << (int)bi << "name=" << b.name << " nodeIndex=" << b.nodeIndex
    //              << " trans=(" << F.m[0][3] << "," << F.m[1][3] << "," << F.m[2][3] << ") det3=" << F.det3()
    //              << " maxRowNorm3=" << F.maxRowNorm3();
    // }

    // CPU skinning using compactInfluences
    for (size_t mi = 0; mi < m_meshes.size(); ++mi)
    {
        SimpleMesh &m = m_meshes[mi];
        if (mi >= m_originalMeshVertices.size())
            continue;
        const std::vector<float> &orig = m_originalMeshVertices[mi];
        if (orig.size() / 3 != m_compactInfluences[mi].size())
            continue;
        size_t vcount = m_compactInfluences[mi].size();
        float *out = m.vertices.data();
        const float *inp = orig.data();
        for (size_t vi = 0; vi < vcount; ++vi)
        {
            // safety: ensure we won't write out of bounds into mesh vertex buffer
            size_t outIdx = 3 * vi + 2;
            if (outIdx >= m.vertices.size())
            {
                qDebug() << "CPU skinning: out-of-bounds write prevented: mesh" << mi << "vi" << vi << "out.size" << m.vertices.size();
                return; // abort applying this frame to avoid memory corruption
            }
            const float x = inp[3 * vi + 0];
            const float y = inp[3 * vi + 1];
            const float z = inp[3 * vi + 2];
            float ax = 0.0f, ay = 0.0f, az = 0.0f;

            const CompactInfluence &ci = m_compactInfluences[mi][vi];
            if (ci.count == 0)
            {
                out[3 * vi + 0] = x;
                out[3 * vi + 1] = y;
                out[3 * vi + 2] = z;
                continue;
            }

            // unroll up to 4 influences for speed
            for (unsigned char k = 0; k < ci.count; ++k)
            {
                int bidx = ci.boneIdx[k];
                float w = ci.weight[k];
                if (bidx < 0 || (size_t)bidx >= nbones)
                    continue;
                size_t off = (size_t)bidx * 12;
                // transform point by 3x4 matrix
                float tx = x * flat[off + 0] + y * flat[off + 1] + z * flat[off + 2] + flat[off + 3];
                float ty = x * flat[off + 4] + y * flat[off + 5] + z * flat[off + 6] + flat[off + 7];
                float tz = x * flat[off + 8] + y * flat[off + 9] + z * flat[off + 10] + flat[off + 11];
                ax += tx * w;
                ay += ty * w;
                az += tz * w;
            }

            m.vertices[vi * 3 + 0] = ax;
            m.vertices[vi * 3 + 1] = ay;
            m.vertices[vi * 3 + 2] = az;
        }
    }

    // // diagnostic: sample original vs skinned vertex for mesh 0
    // if (!m_originalMeshVertices.empty() && !m_meshes.empty()) {
    //     const auto &orig = m_originalMeshVertices[0];
    //     const auto &cur = m_meshes[0].vertices;
    //     if (orig.size() >= 3 && cur.size() >= 3) {
    //         qDebug() << "Sample vertex[0] orig=(" << orig[0] << "," << orig[1] << "," << orig[2] << ")"
    //                  << " skinned=(" << cur[0] << "," << cur[1] << "," << cur[2] << ")";
    //     }
    // }

    emit frameAdvanced();
}

// loadModel: 使用 Assimp 读取模型文件，提取网格数据和材质贴图（支持嵌入式和外链）
bool RobotManager::loadModel(const std::string &file)
{
    Assimp::Importer importer;
    // 导入模型数据
    const aiScene *scene = importer.ReadFile(file, aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
    if (!scene)
    {
        qDebug() << "RobotManager: Assimp load error:" << importer.GetErrorString();
        return false;
    }
    // 清理旧的网格数组，准备加载新模型的数据
    m_meshes.clear();

    QFileInfo modelInfo(QString::fromStdString(file));
    QString modelDir = modelInfo.absolutePath();

    // prepare materials解析材质贴图（per-material）
    std::vector<QString> matTexPaths(scene->mNumMaterials);      // 材质文件路径
    std::vector<QImage> matEmbeddedImages(scene->mNumMaterials); // 嵌入式纹理图像

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

    // 网格meshes
    // m_meshes.reserve(scene->mNumMeshes);
    // m_meshesInfluences.clear();
    // m_meshesInfluences.resize(scene->mNumMeshes);
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
        m_meshes.push_back(std::move(sm));
    }

    // 处理材质
    // assign texture (embedded image or path) per mesh based on material index
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
                    m_meshes[mi].diffuseTexPath = chosen;
                    // qDebug() << "Mesh" << mi << "texture chosen actual path:" << chosen;
                }
                else
                {
                    // keep original (cleaned) as last resort and log for debugging
                    m_meshes[mi].diffuseTexPath = QDir::cleanPath(p);
                    // qDebug() << "Texture file not found for material" << matIdx << "; tried candidates:" << candidates << "; using" << meshes[mi].diffuseTexPath;
                }
            }
        }
    }

    // compute bounds (simple)
    computeBounds();

    // 拷贝节点树；构建 bones 列表并把 weights 填入 meshesInfluences
    m_nodeInfos.clear();
    m_nodeNameToIndex.clear();
    std::function<void(aiNode *, int)> copyNode = [&](aiNode *node, int parentIdx)
    {
        int idx = (int)m_nodeInfos.size();
        m_nodeNameToIndex[QString::fromUtf8(node->mName.C_Str())] = idx;
        NodeInfo n;
        n.name = QString::fromUtf8(node->mName.C_Str());
        n.parent = parentIdx;
        n.local = Mat4::fromAi(node->mTransformation);
        n.originalLocal = Mat4::orthonormalizeRotationKeepTranslation(n.local);
        m_nodeInfos.push_back(n);
        if (parentIdx >= 0)
            m_nodeInfos[parentIdx].children.push_back(idx);
        for (unsigned int i = 0; i < node->mNumChildren; ++i)
            copyNode(node->mChildren[i], idx);
    };
    if (scene->mRootNode)
        copyNode(scene->mRootNode, -1);

    // prepare influences and bone list
    m_meshesInfluences.resize(scene->mNumMeshes);
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
    {
        m_meshesInfluences[mi].resize(scene->mMeshes[mi]->mNumVertices);
    }

    std::map<QString, int> boneNameToIndex;
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
    {
        const aiMesh *mesh = scene->mMeshes[mi];
        for (unsigned int bi = 0; bi < mesh->mNumBones; ++bi)
        {
            const aiBone *bone = mesh->mBones[bi];
            QString bname = QString::fromUtf8(bone->mName.C_Str());
            int biIdx = -1;
            auto it = boneNameToIndex.find(bname);
            if (it == boneNameToIndex.end())
            {
                biIdx = (int)m_bones.size();
                boneNameToIndex[bname] = biIdx;
                BoneInfo bn;
                bn.name = bname;
                bn.offset = Mat4::fromAi(bone->mOffsetMatrix);
                // map node name to index if present
                auto nit = m_nodeNameToIndex.find(bname);
                if (nit != m_nodeNameToIndex.end())
                    bn.nodeIndex = nit->second;

                auto mit = m_boneToJoint.find(bname);
                if (mit != m_boneToJoint.end())
                    bn.jointMappings = mit->second;

                m_bones.push_back(bn);
            }
            else
                biIdx = it->second;
            // add influences
            for (unsigned int wi = 0; wi < bone->mNumWeights; ++wi)
            {
                const aiVertexWeight &vw = bone->mWeights[wi];
                if (vw.mVertexId < m_meshesInfluences[mi].size())
                    m_meshesInfluences[mi][vw.mVertexId].push_back({biIdx, (float)vw.mWeight});
            }
        }
    }

    // save original vertices
    m_originalMeshVertices.clear();
    m_originalMeshVertices.resize(m_meshes.size());
    for (size_t i = 0; i < m_meshes.size(); ++i)
        m_originalMeshVertices[i] = m_meshes[i].vertices;

    // build compactInfluences (top4 normalization)
    m_compactInfluences.clear();
    m_compactInfluences.resize(m_meshesInfluences.size());
    for (size_t mi = 0; mi < m_meshesInfluences.size(); ++mi)
    {
        size_t vcount = m_meshesInfluences[mi].size();
        m_compactInfluences[mi].resize(vcount);
        for (size_t vi = 0; vi < vcount; ++vi)
        {
            const auto &inf = m_meshesInfluences[mi][vi];
            // copy into local vector and sort by weight desc to keep top 4
            std::vector<std::pair<int, float>> tmp = inf;
            if (tmp.size() > 1)
            {
                std::sort(tmp.begin(), tmp.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b)
                          { return a.second > b.second; });
            }
            CompactInfluence ci;
            ci.count = 0;
            for (int k = 0; k < 4; ++k)
            {
                ci.boneIdx[k] = -1;
                ci.weight[k] = 0.0f;
            }
            float sumw = 0.0f;
            for (size_t k = 0; k < tmp.size() && k < 4; ++k)
            {
                ci.boneIdx[k] = tmp[k].first;
                ci.weight[k] = tmp[k].second;
                sumw += ci.weight[k];
                ci.count++;
            }
            // normalize weights if sum > 0
            if (sumw > 1e-8f)
            {
                for (unsigned char k = 0; k < ci.count; ++k)
                    ci.weight[k] /= sumw;
            }
            else
            {
                // fallback: no influences -> identity (count=0)
                ci.count = 0;
            }
            // fill remaining slots to keep deterministic memory content
            for (unsigned char k = ci.count; k < 4; ++k)
            {
                ci.boneIdx[k] = -1;
                ci.weight[k] = 0.0f;
            }
            m_compactInfluences[mi][vi] = ci;
        }
    }

    // print bones info for debug: list bones, mapped node index, offset translation,
    // qDebug() << "--- Bones summary after loadModel ---";
    // for (size_t bi = 0; bi < m_bones.size(); ++bi) {
    //     const BoneInfo &b = m_bones[bi];
    //     // offset translation components
    //     float ox = b.offset.m[0][3];
    //     float oy = b.offset.m[1][3];
    //     float oz = b.offset.m[2][3];
    //     // count influenced vertices across all meshes
    //     size_t infCount = 0;
    //     for (size_t mi = 0; mi < m_meshesInfluences.size(); ++mi) {
    //         for (size_t vi = 0; vi < m_meshesInfluences[mi].size(); ++vi) {
    //             const auto &inf = m_meshesInfluences[mi][vi];
    //             for (const auto &p : inf) if (p.first == (int)bi) { ++infCount; break; }
    //         }
    //     }
    //     qDebug() << "Bone" << (int)bi << "name=" << b.name << " nodeIndex=" << b.nodeIndex
    //              << " offsetTrans=(" << ox << "," << oy << "," << oz << ")" << " influencedVerts=" << (int)infCount;
    // }
    // qDebug() << "--- end bones summary ---";

    qDebug() << "RobotManager: loaded model meshes=" << m_meshes.size() << " bones=" << m_bones.size();
    return true;
}

void RobotManager::advancePlaceFrame()
{
    if (m_placeRows.empty())
        return;
    const QVector<double> &row = m_placeRows[m_currentRow];
    std::map<int, double> ja;
    // follow original file mapping: some joints need sign inversion or offsets
    for (int i = 0; i < row.size(); ++i)
    {
        int jointId = i + 1; // angle1 -> joint 1
        // joints that require negation to match coordinate system
        if (jointId == 2 || jointId == 3 || jointId == 4 || jointId == 6 || jointId == 11 || jointId == 12 || jointId == 13 || jointId == 16 || jointId == 22)
        {
            ja[jointId] = -row[i];
        }
        else if (jointId == 14)
        {
            ja[jointId] = row[i] + 90.0;
        }
        else if (jointId == 17)
        {
            ja[jointId] = row[i] - 90.0;
        }
        else
        {
            ja[jointId] = row[i];
        }
    }
    applyJointAngles(ja);
    m_currentRow = (m_currentRow + 1) % (int)m_placeRows.size();
}

// computeBounds: 计算模型包围盒并将模型居中归一化到一个合适的缩放范围
void RobotManager::computeBounds()
{
    bool first = true;
    float minx = 0, miny = 0, minz = 0, maxx = 0, maxy = 0, maxz = 0;
    for (const auto &m : m_meshes)
    {
        for (size_t i = 0; i < m.vertices.size(); i += 3)
        {
            float x = m.vertices[i + 0], y = m.vertices[i + 1], z = m.vertices[i + 2];
            if (first)
            {
                minx = maxx = x;
                miny = maxy = y;
                minz = maxz = z;
                first = false;
            }
            minx = std::min(minx, x);
            miny = std::min(miny, y);
            minz = std::min(minz, z);
            maxx = std::max(maxx, x);
            maxy = std::max(maxy, y);
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

        // store model centering and scale but DO NOT mutate mesh vertex buffers.
        m_modelCenterX = cx;
        m_modelCenterY = cy;
        m_modelCenterZ = cz;
        m_modelScale = scale;

        if (!m_initialCameraDistanceSet)
        {
            if (!m_cameraDistanceLockedDuringPlayback)
            {
                m_cameraDistance = 2.0f; // default zoom
                m_initialCameraDistance = m_cameraDistance;
                m_initialCameraDistanceSet = true;
                qDebug() << "computeBounds: set initialCameraDistance=" << m_initialCameraDistance;
            }
            else
            {
                // if locked, keep previously captured initialCameraDistance
                qDebug() << "computeBounds: cameraDistance locked during playback, skipping default zoom set";
            }
        }
    }
}