#include "model_display/robotManager.h"
#include "util/load_csv.hpp"
#include <QDateTime>
#include <QCoreApplication>
#include <QDir>
#include <QStandardPaths>
#include <cmath>
#include <QMutexLocker>
#include "model_display/sceneManager.h"
#include "ros_process/gaitCommand.h"
RobotManager::RobotManager(const QString &modelPath, SceneManager *sceneManager, ServoPositionsMonitor *servoPositionsMonitor, QObject *parent)
    : QObject(parent), m_modelPath(modelPath), m_sceneManager(sceneManager), m_servoPositionsMonitor(servoPositionsMonitor)
{
    initialize();
}

void RobotManager::setServoPositionsMonitor(ServoPositionsMonitor *monitor)
{
    if (m_servoPositionsMonitor == monitor)
        return;
    if (m_servoPositionsMonitor) {
        disconnect(m_servoPositionsMonitor, nullptr, this, nullptr);
    }
    m_servoPositionsMonitor = monitor;
    if (m_servoPositionsMonitor)
    {
        bool connected1 = connect(m_servoPositionsMonitor, &ServoPositionsMonitor::servoPositionsUpdated,
                this, &RobotManager::onServoPositionsUpdated, Qt::QueuedConnection);
        bool connected2 = connect(m_servoPositionsMonitor, &ServoPositionsMonitor::legJointAnglesUpdated,
                this, &RobotManager::onLegJointAnglesUpdated, Qt::QueuedConnection);
        // qDebug() << "RobotManager: setServoPositionsMonitor - monitor=" << monitor 
        //          << " servoPositionsUpdated connected=" << connected1
        //          << " legJointAnglesUpdated connected=" << connected2;
    }
    else
    {
        qDebug() << "RobotManager: setServoPositionsMonitor - monitor is nullptr!";
    }
}

void RobotManager::initialize()
{
    // 初始化整体旋转矩阵与欧拉角（默认无旋转）
    m_worldRotation = Mat4::identity();
    m_modelRotationDeg = QVector3D(0.0f, 0.0f, 0.0f);

    loadSucceeded = loadModel(m_modelPath.toStdString());

    // 默认在模型加载成功后将模型整体绕 Y 轴旋转 90 度，便于视图初始展示。
    if (loadSucceeded)
    {
        setModelRotation(QVector3D(0.0f, -90.0f, 0.0f));
    }
    // 记录初始模型朝向以便以后 reset 时恢复
    m_initialModelRotationDeg = m_modelRotationDeg;
    // 将当前模型朝向的 Y 分量作为基础偏置记录下来，后续来自 Scene 的 yaw 将在此基础上叠加
    // m_modelYawOffsetDeg = m_modelRotationDeg.y();

    // 设置关节映射：使用成员函数 resolveConfigAsset() 查找配置文件路径（在类中单独封装以便复用和测试）
    jointConfigPath = resolveConfigAsset("boneToJoint.csv");
    if (!jointConfigPath.isEmpty() && QFile::exists(jointConfigPath)) {
        bool loaded = loadBoneJointMapping(jointConfigPath);
        if (loaded) {
            qDebug() << "RobotManager::initialize: successfully loaded bone-joint mapping from" << jointConfigPath
                     << "entries=" << m_boneToJoint.size() << "bones=" << m_bones.size();
        } else {
            qWarning() << "RobotManager::initialize: failed to load bone-joint mapping from" << jointConfigPath;
        }
    } else {
        qWarning() << "RobotManager::initialize: boneToJoint.csv not found or unreadable! Joint animation will not work.";
    }


    // 如果有 ServoPositionsMonitor，则连接更新信号（queued connection），使用信号驱动直接应用角度，避免轮询
    if (m_servoPositionsMonitor)
    {
        // 连接伺服位置更新信号
        connect(m_servoPositionsMonitor, &ServoPositionsMonitor::servoPositionsUpdated,
                this, &RobotManager::onServoPositionsUpdated, Qt::QueuedConnection);
        // // 连接行走状态更新信号
        // connect(m_servoPositionsMonitor, &ServoPositionsMonitor::walkingStatusUpdated,
        //         this, &RobotManager::onWalkingStatusUpdated, Qt::QueuedConnection);
  
        // // 连接行走停止信号，停止行走动画播放
        // connect(m_servoPositionsMonitor, &ServoPositionsMonitor::walkingStatusStopped,
        //         this, &RobotManager::stopWalkingPlayback, Qt::QueuedConnection);

        connect(m_servoPositionsMonitor, &ServoPositionsMonitor::legJointAnglesUpdated,
                this, &RobotManager::onLegJointAnglesUpdated, Qt::QueuedConnection);
    }

    // 创建并启动用于合并并以固定频率应用伺服与步态更新的定时器（减少高频更新引起的卡顿）
    m_servoApplyTimer = new QTimer(this);
    m_servoApplyTimer->setInterval(m_servoApplyIntervalMs);
    connect(m_servoApplyTimer, &QTimer::timeout, this, &RobotManager::applyPendingServoAndGait, Qt::QueuedConnection);
    m_servoApplyTimer->start();
    // qDebug() << "RobotManager::initialize: servoApplyTimer started, interval=" << m_servoApplyIntervalMs << "ms";


};

// 设定模型整体旋转：传入度为单位的欧拉角 (x_deg, y_deg, z_deg)
void RobotManager::setModelRotation(const QVector3D &eulerDeg)
{
    // 保存欧拉角
    m_modelRotationDeg = eulerDeg;
    const double PI = 3.14159265358979323846;
    float rx = float(eulerDeg.x() * (PI / 180.0));
    float ry = float(eulerDeg.y() * (PI / 180.0));
    float rz = float(eulerDeg.z() * (PI / 180.0));

    // 构造绕各轴的旋转矩阵（使用 Mat4 的静态方法）
    Mat4 Rx = Mat4::rotationX(rx);
    Mat4 Ry = Mat4::rotationY(ry);
    Mat4 Rz = Mat4::rotationZ(rz);

    // 内部按 Z * Y * X 的顺序复合旋转（与注释一致）
    m_worldRotation = Rz * Ry * Rx;

    // 重新应用当前关节角度以更新顶点（applyJointAngles 会触发 frameAdvanced）
    applyJointAngles(m_currentJointAngles);
}

// 封装的路径解析函数实现：按更广泛的策略搜索配置文件并返回第一个存在的路径：
// 1) 环境变量覆盖（ROBANWEB_CONFIG_DIR）
// 2) 直接提供的路径
// 3) 应用目录 / 当前目录 及其向上回溯中可能的 config/ 子目录
// 4) QStandardPaths 的 AppConfigLocation 作为补充
// 若没有找到，返回第一个候选路径以便运行时调试输出（保持与历史行为兼容）
QString RobotManager::resolveConfigAsset(const QString &filename) const
{
    if (filename.isEmpty()) return QString();

    // 1) environment override
    QByteArray env = qgetenv("ROBANWEB_CONFIG_DIR");
    if (!env.isEmpty()) {
        QString envDir = QString::fromLocal8Bit(env);
        QString p = QDir::cleanPath(envDir + QDir::separator() + filename);
        if (QFile::exists(p)) {
            qDebug() << "RobotManager::resolveConfigAsset: using ROBANWEB_CONFIG_DIR ->" << p;
            return p;
        }
    }

    // 2) if caller passed an existing path, accept it
    if (QFile::exists(filename)) {
        return QDir::cleanPath(filename);
    }

    QString appDir = QCoreApplication::applicationDirPath();
    QString curDir = QDir::currentPath();
    QStringList candidates;

    // helper: try to append candidate and avoid duplicates
    auto pushCandidate = [&candidates](const QString &s){ QString c = QDir::cleanPath(s); if (!c.isEmpty() && !candidates.contains(c)) candidates << c; };

    // common direct candidates
    pushCandidate(appDir + QDir::separator() + "config" + QDir::separator() + filename);
    pushCandidate(appDir + QDir::separator() + filename);
    pushCandidate(curDir + QDir::separator() + "config" + QDir::separator() + filename);
    pushCandidate(curDir + QDir::separator() + filename);

    // also try a few well-known relative guesses (exe in build/bin etc.)
    pushCandidate(appDir + QDir::separator() + ".." + QDir::separator() + "config" + QDir::separator() + filename);
    pushCandidate(appDir + QDir::separator() + ".." + QDir::separator() + ".." + QDir::separator() + "config" + QDir::separator() + filename);
    pushCandidate(appDir + QDir::separator() + ".." + QDir::separator() + "RobanWeb" + QDir::separator() + "config" + QDir::separator() + filename);

    // 3) walk upwards from appDir and curDir and look for a config/ directory at each level
    auto walkUpAndCollect = [&](const QString &start){
        QFileInfo fi(start);
        QString dir = fi.absolutePath();
        int levels = 0;
        while (!dir.isEmpty() && levels < 12) {
            pushCandidate(dir + QDir::separator() + "config" + QDir::separator() + filename);
            pushCandidate(dir + QDir::separator() + filename);
            QFileInfo pdir(dir);
            QString parent = pdir.dir().absolutePath();
            if (parent == dir) break; // reached root
            dir = parent;
            levels++;
        }
    };
    walkUpAndCollect(appDir);
    if (curDir != appDir) walkUpAndCollect(curDir);

    // 4) try QStandardPaths AppConfigLocation (useful on some platforms)
    QString stdCfg = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (!stdCfg.isEmpty()) pushCandidate(stdCfg + QDir::separator() + filename);

    // search candidates for existence
    for (const QString &p : candidates) {
        if (QFile::exists(p)) {
            qDebug() << "RobotManager::resolveConfigAsset: found" << filename << "->" << p;
            return p;
        }
    }

    // nothing found: log attempted candidates to help debugging and return a sensible first candidate
    if (!candidates.isEmpty()) {
        qWarning() << "RobotManager::resolveConfigAsset: did not find" << filename << "; tried candidates:" << candidates;
        return candidates.first();
    }

    qWarning() << "RobotManager::resolveConfigAsset: no candidates generated for" << filename;
    return QString();
}

bool RobotManager::loadBoneJointMapping(const QString &csvPath)
{
    if (csvPath.isEmpty()) {
        qWarning() << "RobotManager::loadBoneJointMapping: empty path provided";
        return false;
    }
    QFile f(csvPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "RobotManager::loadBoneJointMapping: fail open mapping" << csvPath;
        return false;
    }
    m_boneToJoint.clear(); // 清空旧映射
    QTextStream ts(&f);
    QString header = ts.readLine(); // skip possible header
    int mappingCount = 0;
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
            mappingCount++;
        }
    }
    // apply mapping to bones if already loaded
    int appliedCount = 0;
    for (auto &b : m_bones)
    {
        auto it = m_boneToJoint.find(b.name);
        if (it != m_boneToJoint.end()) {
            b.jointMappings = it->second;
            appliedCount++;
        }
    }
    // qDebug() << "RobotManager::loadBoneJointMapping: loaded" << mappingCount << "mappings, applied to" 
    //          << appliedCount << "out of" << m_bones.size() << "bones";
    return true;
}

bool RobotManager::loadActionCsv(const QString &csvPath, int intervalMs, bool loop)
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

    // set looping behavior according to caller (默认或调用者指定)
    m_placeLooping = loop;

    if (!m_animTimer)
    {
        m_animTimer = new QTimer(this);
        connect(m_animTimer, &QTimer::timeout, this, &RobotManager::advancePlaceFrame);
    }
    m_animTimer->start(intervalMs);
    // mark that playback is active and notify viewers; viewers will decide how to adjust camera
    m_cameraDistanceLockedDuringPlayback = true;
    // qDebug() << "RobotManager: started animation, rows=" << m_placeRows.size() << " cameraDistanceLockedDuringPlayback=" << m_cameraDistanceLockedDuringPlayback;

    emit animationStarted(); // 连接ModelViewer, 设置视角
    m_currentRow = 0;

    return true;
}

void RobotManager::applyJointAngles(const std::map<int, double> &jointAngles)
{
    // lock meshes while we update vertex buffers to avoid concurrent read from GL thread
    QMutexLocker meshLocker(&m_meshesMutex);
    m_currentJointAngles = jointAngles;

    // reset local to original
    for (auto &n : m_nodeInfos)
        n.local = n.originalLocal;

    // helper to apply rotation matrix to a node's local transform
    auto applyRotationToNodeLocal = [&](int nodeIdx, const Mat4 &Radd)
    {
        if (nodeIdx < 0 || nodeIdx >= (int)m_nodeInfos.size())
            return;
        const Mat4 &orig = m_nodeInfos[nodeIdx].originalLocal;

        // extract translation
        float tx = orig.m[0][3];
        float ty = orig.m[1][3];
        float tz = orig.m[2][3];
        // extract rotation (3x3)
        Mat4 Rorig = Mat4::identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Rorig.m[r][c] = orig.m[r][c];

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
    int bonesWithMapping = 0;
    int bonesApplied = 0;
    static int jointApplyCount = 0;
    for (size_t bi = 0; bi < m_bones.size(); ++bi)
    {
        const BoneInfo &b = m_bones[bi];
        if (b.jointMappings.empty())
            continue;
        bonesWithMapping++;
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
        if (anyApplied && b.nodeIndex >= 0) {
            applyRotationToNodeLocal(b.nodeIndex, Radd);
            bonesApplied++;
        }
    }
    if (++jointApplyCount % 200 == 0) {
        qDebug() << "RobotManager::applyJointAngles: joints=" << jointAngles.size() 
                 << " bonesWithMapping=" << bonesWithMapping << " bonesApplied=" << bonesApplied
                 << " totalBones=" << m_bones.size();
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

            // 先围绕模型中心应用整体旋转（由 setModelRotation 设置），再应用世界平移
            // 旋转在模型局部空间进行：把点平移到以模型中心为原点 -> 旋转 -> 平移回中心
            float px = ax - m_modelCenterX;
            float py = ay - m_modelCenterY;
            float pz = az - m_modelCenterZ;
            float rx = px * m_worldRotation.m[0][0] + py * m_worldRotation.m[0][1] + pz * m_worldRotation.m[0][2];
            float ry = px * m_worldRotation.m[1][0] + py * m_worldRotation.m[1][1] + pz * m_worldRotation.m[1][2];
            float rz = px * m_worldRotation.m[2][0] + py * m_worldRotation.m[2][1] + pz * m_worldRotation.m[2][2];

            ax = rx + m_modelCenterX + m_worldTranslation.x();
            ay = ry + m_modelCenterY + m_worldTranslation.y();
            az = rz + m_modelCenterZ + m_worldTranslation.z();

            m.vertices[vi * 3 + 0] = ax;
            m.vertices[vi * 3 + 1] = ay;
            m.vertices[vi * 3 + 2] = az;
        }
    }

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
    // protect mesh replacement and population from concurrent GL reads
    QMutexLocker meshLocker(&m_meshesMutex);
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

    // m_currentRow = (m_currentRow + 1) % (int)m_placeRows.size();

    // ---------------测试代码，从csv读取时是否循环读取---------------
    if (m_placeLooping)
    {
        m_currentRow = (m_currentRow + 1) % (int)m_placeRows.size();
    }
    else
    {
        // 非循环模式：到最后一行后保持不再回到开头，且停止计时器以节省资源
        if (m_currentRow + 1 < (int)m_placeRows.size())
        {
            m_currentRow += 1;
        }
        else
        {
            // 已到最后一帧，停止计时器但保持当前行（最后一行）不变
            if (m_animTimer)
            {
                m_animTimer->stop();
                // don't delete timer here; keep for potential future reuse
            }
            // notify listeners that the place (action) playback reached its end
            emit placePlaybackFinished(); // 当读取到csv最后一行时发出信号表示动画播放结束
        }
    }
    // ---------------测试代码，从csv读取时是否循环读取---------------
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

        // 模型中心和缩放， 对Y轴和Z轴做微调以适应视图位置
        m_modelCenterX = cx;
        m_modelCenterY = cy + -0.0419408;
        m_modelCenterZ = cz - 1.2664;
        m_modelScale = scale;

        // qDebug() << "computeBounds: model center=(" << m_modelCenterX << "," << m_modelCenterY << "," << m_modelCenterZ << ")"
        //          << " scale=" << m_modelScale;

        if (!m_initialCameraDistanceSet)
        {
            if (!m_cameraDistanceLockedDuringPlayback)
            {
                m_cameraDistance = 11.6f; // default zoom
                m_initialCameraDistance = m_cameraDistance;
                m_initialCameraDistanceSet = true;
                // qDebug() << "computeBounds: set initialCameraDistance=" << m_initialCameraDistance;
            }
            else
            {
                // if locked, keep previously captured initialCameraDistance
                qDebug() << "computeBounds: cameraDistance locked during playback, skipping default zoom set";
            }
        }
    }
}



// 将模型移动到给定的世界位置（直接修改 mesh 顶点为原始顶点 + delta）
void RobotManager::applyLocation(const QVector3D &target)
{
    // store world translation (target relative to computed model center)
    float tx = target.x() - m_modelCenterX;
    float ty = target.y() - m_modelCenterY;
    float tz = target.z() - m_modelCenterZ;
    m_worldTranslation = QVector3D(tx, ty, tz);

    // Recompute current displayed vertices using existing joint angles so
    // the translation is applied on top of the current pose (avoid reverting
    // to bind/original vertices). applyJointAngles will add m_worldTranslation
    // into the skinned vertex results.
    applyJointAngles(m_currentJointAngles);
}

void RobotManager::refreshRobotPositionsFromScene()
{
    QVector3D scenePt;
    if (!m_sceneManager->mapCurrentRobotPoseToScene(scenePt))
    {
        qDebug() << "RobotManager::refreshRobotPositionsFromScene: no valid mapping or robot pose";
        return;
    }

    // qDebug() << "RobotManager::refreshRobotPositionsFromScene: mapped robot -> scene:" << scenePt;
    applyLocation(scenePt);
}

void RobotManager::resetRobotPositions()
{
    // 恢复为初始朝向与初始位置，确保模型回到加载时的姿态
    setModelRotation(m_initialModelRotationDeg);
    QVector3D origin(0.0f, 0.0f, 0.0f);
    applyLocation(origin);

    // Also reset smoothing / gait caches to avoid jumps when subsequent gait commands arrive.
    // Clear pending gait accumulators under lock and reset last-applied / target caches
    {
        QMutexLocker locker(&m_pendingGaitMutex);
        m_pendingGx = 0.0;
        m_pendingGy = 0.0;
        m_pendingGdelta = 0.0;
        m_hasPendingGait = false;
    }

    // Reset last-applied and target values so smoothing starts from the reset state
    m_lastAppliedWorldTranslation = m_worldTranslation;
    m_gaitTargetWorldTranslation = m_worldTranslation;
    m_lastAppliedYawDeg = m_modelRotationDeg.y();
    m_gaitTargetYawDeg = m_modelRotationDeg.y();
    m_hasInitializedApplyState = true;

    // Clear pending servo angles (so smoothing won't lerp from stale values)
    {
        QMutexLocker locker(&m_pendingServoMutex);
        m_pendingServo.tsMs = 0;
        m_pendingServo.angles.clear();
    }
    m_lastAppliedServoAngles.clear();
    // reset last apply time to avoid large dt on next apply
    m_lastApplyMs = QDateTime::currentMSecsSinceEpoch();
}


// 使用从机器人位置转换来的场景坐标，启动定时器执行模型移动
bool RobotManager::startLocationPlayback(QVector3D &target_pos, int intervalMs)
{
    stopLocationPlayback();

    m_locationTimer = new QTimer(this);
    connect(m_locationTimer, &QTimer::timeout, this, [this, &target_pos]()
            {
                applyLocation(target_pos);

                if (m_locationTimer)
                {
                    m_locationTimer->stop();
                    delete m_locationTimer;
                    m_locationTimer = nullptr;
                }
                emit locationPlaybackFinished(); // 发送位置更新结束的信号
            });

    m_locationTimer->start(intervalMs);

    return true;
}

void RobotManager::stopLocationPlayback()
{
    if (m_locationTimer)
    {
        m_locationTimer->stop();
        delete m_locationTimer;
        m_locationTimer = nullptr;
    }
}

// 构建轨迹 mesh：把已解析的 m_locationRows 转为 SimpleMesh 的顶点数组，顶点顺序即为时间顺序。
// 该函数只生成顶点数据（positions），渲染（GL_LINE_STRIP、颜色等）由调用者在 GL 上进行。
SimpleMesh RobotManager::buildTrajectoryMesh() const
{
    SimpleMesh out;
    if (m_locationRows.empty())
        return out;

    // 只返回已经播放/应用的点，以便视图逐步绘制轨迹
    int count = qMin((int)m_locationRows.size(), m_playedLocationCount);
    if (count <= 0)
        return out;

    out.vertices.reserve(count * 3);
    out.colors.reserve(count * 4); // 颜色设置
    for (int i = 0; i < count; ++i)
    {
        const QVector3D &p = m_locationRows[i];
        out.vertices.push_back(p.x());
        out.vertices.push_back(p.y());
        out.vertices.push_back(p.z());
        // 轨迹颜色设置
        // default color green
        float r = 0.0f, g = 1.0f, b = 0.0f;
        float a = 1.0f;
        // compute alpha based on time if available
        if (i < m_locationTimes.size() && m_locationTimes[i] > 0 && m_trajectoryFadeSeconds > 0.0)
        {
            double now = QDateTime::currentDateTimeUtc().toMSecsSinceEpoch() / 1000.0;
            double playedT = m_locationTimes[i];
            double age = now - playedT; // seconds since played
            if (age >= m_trajectoryFadeSeconds)
            {
                a = 0.0f;
            }
            else if (age <= 0.0)
            {
                a = 1.0f;
            }
            else
            {
                a = float(1.0 - (age / m_trajectoryFadeSeconds));
            }
        }
        out.colors.push_back(r);
        out.colors.push_back(g);
        out.colors.push_back(b);
        out.colors.push_back(a);
    }

    // 不填充 indices：ModelViewer 会使用无索引的 GL_LINE_STRIP 绘制或按需要读取 vertices
    return out;
}

// 停止播放位置点的计时器
void RobotManager::stopPlacePlayback()
{
    if (m_animTimer)
    {
        m_animTimer->stop();
    }
    // Unlock camera distance when stopping playback
    m_cameraDistanceLockedDuringPlayback = false;
    qDebug() << "RobotManager::stopPlacePlayback: stopped place animation";
}

// 停止行走动画播放：停止并删除行走播放定时器，重置播放索引
// void RobotManager::stopWalkingPlayback()
// {
//     m_isWalking = false;
 
// }

void RobotManager::onServoPositionsUpdated()
{
    if (!m_servoPositionsMonitor)
    {
        qDebug() << "RobotManager::onServoPositionsUpdated: monitor is nullptr!";
        return;
    }
    
    static int servoUpdateCount = 0;
    if (++servoUpdateCount % 100 == 0) {
        qDebug() << "RobotManager::onServoPositionsUpdated: called (count=" << servoUpdateCount << ")";
    }
    
    // 调用融合更新逻辑
    updateFusedServo();
}

void RobotManager::applyServoAnglesRow(const QVector<double> &row)
{
    // row 中的每一项对应 angle1..angleN，映射到 jointId = index+1
    std::map<int, double> ja;
    for (int i = 0; i < row.size(); ++i)
    {
        int jointId = i + 1;
        double val = row[i];
        // 使用与 advancePlaceFrame 相同的坐标系/符号调整规则
        if ( jointId == 2 || jointId == 3 || jointId == 4 || jointId == 6 || jointId == 11 || jointId == 12 || jointId == 13 || jointId == 16 || jointId == 22)
        {
            ja[jointId] = -val;
        }
        else if (jointId == 14)
        {
            ja[jointId] = val + 90.0;
        }
        else if (jointId == 17)
        {
            ja[jointId] = val - 90.0;
        }
        else
        {
            ja[jointId] = val;
        }
    }
    
    static int applyCount = 0;
    static QVector<double> lastRow;
    bool hasChange = (lastRow.size() != row.size());
    if (!hasChange) {
        for (int i = 0; i < row.size() && i < 5; ++i) { // 只检查前5个角度
            if (qAbs(row[i] - lastRow[i]) > 0.1) {
                hasChange = true;
                break;
            }
        }
    }
    // if (hasChange && ++applyCount % 100 == 0) {
    //     QString anglesStr;
    //     for (int i = 0; i < qMin(5, row.size()); ++i) {
    //         anglesStr += QString::number(row[i], 'f', 1) + " ";
    //     }
    //     qDebug() << "RobotManager::applyServoAnglesRow: applying angles (count=" << applyCount 
    //              << " size=" << row.size() << " first5=" << anglesStr << "...)";
    // }
    lastRow = row;
    
    // 直接应用到模型（会触发 frameAdvanced）
    applyJointAngles(ja);
}

void RobotManager::onLegJointAnglesUpdated(){
    if (!m_servoPositionsMonitor) {
        qDebug() << "RobotManager::onLegJointAnglesUpdated: monitor is nullptr!";
        return;
    }
    // 获取最新的腿部关节数据并缓存
    m_cachedLegAngles = m_servoPositionsMonitor->lastLegAngles();
    static int legUpdateCount = 0;
    if (++legUpdateCount % 100 == 0) {
        qDebug() << "RobotManager::onLegJointAnglesUpdated: cached leg angles, size=" << m_cachedLegAngles.size();
    }
    // 总是触发融合更新，不再依赖行走状态
    updateFusedServo();
}

void RobotManager::updateFusedServo() {
    if (!m_servoPositionsMonitor) {
        qDebug() << "RobotManager::updateFusedServo: monitor is nullptr!";
        return;
    }

    // 获取最新的全身伺服数据
    QVector<double> servo = m_servoPositionsMonitor->lastAngles();
    
    // 如果有腿部数据，进行融合（直接覆盖前12维）
    if (!m_cachedLegAngles.isEmpty()) {
        // 目标尺寸：优先使用 servo 的长度以保留实时数据；若 servo 不足，则按 22 个关节长度扩展
        int targetSize = qMax<int>(servo.size(), 22);
        QVector<double> fused;
        fused.resize(targetSize);
        
        // 先用 servo 的数据填充（若存在），否则填 0
        for (int i = 0; i < targetSize; ++i) {
            if (i < servo.size()) fused[i] = servo[i];
            else fused[i] = 0.0;
        }

        // 用 legJointAngles 的数据覆盖 fused 的前 N 项 (通常腿部关节在前)
        // 假设 legJointAngles 包含 12 个关节数据
        int take = qMin(12, m_cachedLegAngles.size());
        for (int i = 0; i < take; ++i) {
            fused[i] = m_cachedLegAngles[i];
        }
        
        // 使用融合后的数据
        servo = fused;
        static int fusionCount = 0;
        if (++fusionCount % 100 == 0) {
            qDebug() << "RobotManager::updateFusedServo: fused leg angles (count=" << fusionCount 
                     << " servoSize=" << servo.size() << " legSize=" << m_cachedLegAngles.size() << ")";
        }
    } else {
        static int noLegCount = 0;
        if (++noLegCount % 100 == 0) {
            qDebug() << "RobotManager::updateFusedServo: no leg angles cached (count=" << noLegCount 
                     << " servoSize=" << servo.size() << ")";
        }
    }

    if (servo.isEmpty()) {
        static int emptyCount = 0;
        if (++emptyCount % 100 == 0) {  // 每100次打印一次，避免日志过多
            qDebug() << "RobotManager::updateFusedServo: servo data is empty (count=" << emptyCount << ")";
        }
        return;
    }

    {
        QMutexLocker locker(&m_pendingServoMutex);
        // store as timestamped pending frame (replace any older pending)
        m_pendingServo.tsMs = QDateTime::currentMSecsSinceEpoch();
        m_pendingServo.angles = std::move(servo);
        static int updateCount = 0;
        if (++updateCount % 100 == 0) {  // 每100次打印一次
            qDebug() << "RobotManager::updateFusedServo: updated pending servo data (count=" << updateCount 
                     << " angles=" << m_pendingServo.angles.size() << ")";
        }
    }
}


// 设置 gait command 数据源：保存指针并订阅其更新信号（使用队列连接以线程安全）
void RobotManager::setGaitCommandMonitor(GaitCommandMonitor *monitor)
{
    if (m_gaitCommandMonitor == monitor)
        return;
    if (m_gaitCommandMonitor)
    {
        // disconnect previous if any
        disconnect(m_gaitCommandMonitor, &GaitCommandMonitor::gaitCommandUpdated, this, &RobotManager::onGaitCommandUpdated);
    }
    m_gaitCommandMonitor = monitor;
    if (m_gaitCommandMonitor)
    {
        connect(m_gaitCommandMonitor, &GaitCommandMonitor::gaitCommandUpdated, this, &RobotManager::onGaitCommandUpdated, Qt::QueuedConnection);
    }
}

// 当收到步态命令更新时调用：读取 gaitCommand 中的 x,y,delta 并把模型移动/旋转到新的位置
void RobotManager::onGaitCommandUpdated()
{
    if (!m_gaitCommandMonitor)
        return;

    // 线程安全的访问 getters
    double gx = m_gaitCommandMonitor->gaitX();
    double gy = m_gaitCommandMonitor->gaitY();
    double gdelta = m_gaitCommandMonitor->gaitDelta();

    // Coalesce gait updates: accumulate small increments into pending sums so that
    // fast frequent updates are integrated and applied by the periodic timer.
    {
        QMutexLocker locker(&m_pendingGaitMutex);
        m_pendingGx += gx;
        m_pendingGy += gy;
        m_pendingGdelta += gdelta;
        m_hasPendingGait = true;

        // compute local translation magnitude (pre-scale) to detect in-place turn intent
        float lx = 0.0f, lz = 0.0f;
        switch (m_gaitAxisMapping)
        {
        case RobotManager::LocalXForward:
            lx = float(gx);
            lz = float(gy);
            break;
        case RobotManager::LocalZForward:
            lx = float(gy);
            lz = float(gx);
            break;
        case RobotManager::SwapAxes:
            lx = float(gy);
            lz = -float(gx);
            break;
        default:
            lx = float(gx);
            lz = float(gy);
            break;
        }
        float localMoveMag = std::sqrt(lx * lx + lz * lz);

        // If translation is tiny but we received a meaningful gdelta, apply yaw immediately
        if (localMoveMag < m_yawBoostTranslationThreshold && std::fabs(gdelta) >= m_yawActivationGdeltaThreshold)
        {
            // consume the pending gdelta we just stored so applyPending won't double-apply it
            double immediateGdelta = m_pendingGdelta;
            m_pendingGdelta -= immediateGdelta;
            if (m_pendingGx == 0.0 && m_pendingGy == 0.0 && m_pendingGdelta == 0.0)
                m_hasPendingGait = false;

            // ensure apply caches initialized
            if (!m_hasInitializedApplyState)
            {
                m_lastAppliedWorldTranslation = m_worldTranslation;
                m_lastAppliedYawDeg = m_modelRotationDeg.y();
                m_gaitTargetWorldTranslation = m_worldTranslation;
                m_gaitTargetYawDeg = m_modelRotationDeg.y();
                m_hasInitializedApplyState = true;
            }

            // update target and last-applied yaw and set model rotation immediately
            m_gaitTargetYawDeg += float(immediateGdelta);
            m_lastAppliedYawDeg = m_gaitTargetYawDeg;
            QVector3D newEuler = m_modelRotationDeg;
            // apply user-configurable compensation between real yaw and model yaw
            newEuler.setY(m_lastAppliedYawDeg + m_modelYawCompensationDeg);
            setModelRotation(newEuler);
            // update last apply timestamp to avoid a large dt
            m_lastApplyMs = QDateTime::currentMSecsSinceEpoch();
        }
    }
}

void RobotManager::applyPendingServoAndGait()
{
    // Extract pending servo angles and gait under locks with minimal hold time
    QVector<double> servoAngles;
    qint64 servoTs = 0;
    {
        QMutexLocker locker(&m_pendingServoMutex);
        if (!m_pendingServo.angles.isEmpty()) {
            servoAngles = m_pendingServo.angles;
            servoTs = m_pendingServo.tsMs;
        }
        // clear pending (we replaced it with last snapshot)
        m_pendingServo.tsMs = 0;
        m_pendingServo.angles.clear();
    }
    
    static int applyCount = 0;
    // if (!servoAngles.isEmpty()) {
    //     if (++applyCount % 100 == 0) {
    //         qDebug() << "RobotManager::applyPendingServoAndGait: applying servo angles (count=" << applyCount 
    //                  << " size=" << servoAngles.size() << ")";
    //     }
    // }

    qint64 nowMs = QDateTime::currentMSecsSinceEpoch();

    double gx = 0.0, gy = 0.0, gdelta = 0.0;
    bool hasGait = false;
    {
        QMutexLocker locker(&m_pendingGaitMutex);
        if (m_hasPendingGait)
        {
            gx = m_pendingGx;
            gy = m_pendingGy;
            gdelta = m_pendingGdelta;
            hasGait = true;
            m_hasPendingGait = false;
            // clear accumulated pending after taking a snapshot
            m_pendingGx = 0.0;
            m_pendingGy = 0.0;
            m_pendingGdelta = 0.0;
        }
    }

    // Apply servo angles (if any)
    if (!servoAngles.isEmpty())
    {
        // drop stale frames that are too old
        if (servoTs > 0 && (nowMs - servoTs) > m_servoStaleMs) {
            // stale -> ignore
            servoAngles.clear();
        }
    }

    if (!servoAngles.isEmpty())
    {
        // smoothing: lerp between lastApplied and new servoAngles to reduce jitter
        std::vector<double> toApply;
        toApply.resize(servoAngles.size());
        if (m_lastAppliedServoAngles.empty())
        {
            // first time: initialize lastApplied to current values
            m_lastAppliedServoAngles = std::vector<double>(servoAngles.begin(), servoAngles.end());
        }
        // ensure lastApplied has same size
        if (m_lastAppliedServoAngles.size() < servoAngles.size())
            m_lastAppliedServoAngles.resize(servoAngles.size(), 0.0);

        // compute effective lerp factor: either time-based smoothing or fixed per-frame lerp
        double effF = std::min(std::max<double>(m_servoLerpFactor, 0.0), 1.0);
        if (m_enableTimeBasedSmoothing)
        {
            double dtMs = (m_lastApplyMs == 0) ? double(m_servoApplyIntervalMs) : double(nowMs - m_lastApplyMs);
            double baseMs = double(qMax(1, m_servoApplyIntervalMs));
            double baseF = std::min(std::max<double>(m_servoLerpFactor, 0.0), 0.999);
            effF = 1.0 - std::pow(1.0 - baseF, dtMs / baseMs);
            if (!(effF >= 0.0 && effF <= 1.0)) effF = baseF; // fallback
        }
        for (int i = 0; i < servoAngles.size(); ++i)
        {
            double last = (i < m_lastAppliedServoAngles.size()) ? m_lastAppliedServoAngles[i] : 0.0;
            double cur = servoAngles[i];
            double v = last + (cur - last) * effF;
            toApply[i] = v;
        }
        // apply smoothed angles (construct QVector<double> for applyServoAnglesRow)
        QVector<double> qToApply;
        qToApply.reserve((int)toApply.size());
        for (size_t i = 0; i < toApply.size(); ++i)
            qToApply.append(toApply[i]);
        applyServoAnglesRow(qToApply);
        // store last applied
        m_lastAppliedServoAngles = toApply;
        // update last apply time
        m_lastApplyMs = nowMs;
    }

    // Apply gait-driven translation/rotation (if any)
    if (hasGait)
    {
    Mat4 R = m_worldRotation;
    float lx = 0.0f, ly = 0.0f, lz = 0.0f;
        switch (m_gaitAxisMapping)
        {
        case RobotManager::LocalXForward:
            lx = float(gx) * m_gaitScale;
            lz = float(gy) * m_gaitScale;
            break;
        case RobotManager::LocalZForward:
            lx = float(gy) * m_gaitScale;
            lz = float(gx) * m_gaitScale;
            break;
        case RobotManager::SwapAxes:
            lx = float(gy) * m_gaitScale;
            lz = -float(gx) * m_gaitScale;
            break;
        default:
            lx = float(gx) * m_gaitScale;
            lz = float(gy) * m_gaitScale;
            break;
        }
        float world_dx = lx * R.m[0][0] + ly * R.m[0][1] + lz * R.m[0][2];
        float world_dy = lx * R.m[1][0] + ly * R.m[1][1] + lz * R.m[1][2];
        float world_dz = lx * R.m[2][0] + ly * R.m[2][1] + lz * R.m[2][2];
        // accumulate into gait target (world-space translation and absolute yaw)
        if (!m_hasInitializedApplyState)
        {
            m_lastAppliedWorldTranslation = m_worldTranslation;
            m_lastAppliedYawDeg = m_modelRotationDeg.y();
            // initialize targets to current state
            m_gaitTargetWorldTranslation = m_worldTranslation;
            m_gaitTargetYawDeg = m_modelRotationDeg.y();
            m_hasInitializedApplyState = true;
        }

        // add incremental world delta into gait target
        m_gaitTargetWorldTranslation += QVector3D(world_dx, world_dy, world_dz);
        // add delta yaw (degrees) to target yaw
        m_gaitTargetYawDeg += float(gdelta);

        // compute local translation magnitude (before rotating into world) to
        // detect when the robot is essentially turning in-place
        float localMoveMag = std::sqrt(lx * lx + lz * lz);
        // trigger yaw-boost if we detect small translation but a significant gdelta
        if (localMoveMag < m_yawBoostTranslationThreshold && std::fabs(gdelta) >= m_yawActivationGdeltaThreshold)
        {
            m_yawBoostRemaining = m_yawBoostFrames;
        }

        // lerp from last applied toward target using configurable follow factors
        // compute effective follow factors based on elapsed time for consistent behavior
        double effTf = double(m_gaitFollowFactorTranslation);
        double effYf = double(m_gaitFollowFactorYaw);
        if (m_enableTimeBasedSmoothing)
        {
            double dtMs = (m_lastApplyMs == 0) ? double(m_servoApplyIntervalMs) : double(nowMs - m_lastApplyMs);
            double baseMs = double(qMax(1, m_servoApplyIntervalMs));
            double tf_base = std::min(std::max<double>(m_gaitFollowFactorTranslation, 0.0), 0.999);
            double yf_base = std::min(std::max<double>(m_gaitFollowFactorYaw, 0.0), 0.999);
            effTf = 1.0 - std::pow(1.0 - tf_base, dtMs / baseMs);
            effYf = 1.0 - std::pow(1.0 - yf_base, dtMs / baseMs);
        }

        // if we have an active yaw-boost window, apply yaw immediately (or with full follow)
        bool applyImmediateYaw = false;
        if (m_yawBoostRemaining > 0)
        {
            applyImmediateYaw = true;
            m_yawBoostRemaining -= 1;
        }
        QVector3D lerped = m_lastAppliedWorldTranslation + (m_gaitTargetWorldTranslation - m_lastAppliedWorldTranslation) * float(effTf);
        float lerpedYaw;
        if (applyImmediateYaw && m_enableImmediateYaw)
        {
            // snap yaw to target to avoid perceptible lag when switching to turning-in-place
            lerpedYaw = m_gaitTargetYawDeg;
        }
        else
        {
            lerpedYaw = m_lastAppliedYawDeg + (m_gaitTargetYawDeg - m_lastAppliedYawDeg) * float(effYf);
        }

        // apply rotation & translation
    QVector3D newEuler = m_modelRotationDeg;
    // 仅在确实存在 yaw 变化（目标 yaw 与上次应用 yaw 不同）或处于 yaw-boost/立即应用的窗口时
    // 才把补偿角加入到模型 yaw 中。这样可以避免在纯平移时引入补偿导致的误旋转。
    const float YAW_COMPENSATION_THRESHOLD = 1e-3f; // degrees
    bool hasYawChange = (std::fabs(m_gaitTargetYawDeg - m_lastAppliedYawDeg) > YAW_COMPENSATION_THRESHOLD) || applyImmediateYaw;
    float finalYaw = lerpedYaw + (hasYawChange ? m_modelYawCompensationDeg : 0.0f);
    newEuler.setY(finalYaw);
    setModelRotation(newEuler);

        QVector3D applyTarget = lerped + QVector3D(m_modelCenterX, m_modelCenterY, m_modelCenterZ);
        applyLocation(applyTarget);

        // update last applied cache
        m_lastAppliedWorldTranslation = lerped;
        m_lastAppliedYawDeg = lerpedYaw;
        // update last apply time
        m_lastApplyMs = nowMs;
    }
}



