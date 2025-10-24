#include "ros_process/slamMapPoint.h"
#include "socket_process/websocketworker.h"
#include "util/load_param.hpp"


SlamMapMonitor::SlamMapMonitor(WebSocketWorker *worker, QObject *parent)
    : QObject(parent), m_worker(worker)
{
    loadTopicFromParams();
}

SlamMapMonitor::~SlamMapMonitor() {}

void SlamMapMonitor::loadTopicFromParams(){
    slamPoint_topic_name = loadTopicFromConfig("slamPoint_topic");
    slamPoint_topic_type = loadTopicFromConfig("slamPoint_topic_type");
    slamKeyFrame_topic_name = loadTopicFromConfig("slamKeyFrame_topic");
    slamKeyFrame_topic_type = loadTopicFromConfig("slamKeyFrame_topic_type");
    cameraOpenGLMatrix_topic_name = loadTopicFromConfig("openGLMatrix_topic");
    cameraOpenGLMatrix_topic_type = loadTopicFromConfig("openGLMatrix_topic_type");
    cameraPose_topic_name = loadTopicFromConfig("cameraPose_topic");
    cameraPose_topic_type = loadTopicFromConfig("cameraPose_topic_type");

    // 话题解析异常处理
    if(slamPoint_topic_name.isEmpty() || slamKeyFrame_topic_name.isEmpty() || cameraOpenGLMatrix_topic_name.isEmpty() || cameraPose_topic_name.isEmpty()){
        slamPoint_topic_name = "/SLAM/MapPoints"; 
        slamPoint_topic_type = "sensor_msgs/PointCloud2";

        slamKeyFrame_topic_name = "/SLAM/KeyFrames";
        slamKeyFrame_topic_type = "visualization_msgs/MarkerArray";

        cameraOpenGLMatrix_topic_name = "/SLAM/CameraOpenGLMatrix";
        cameraOpenGLMatrix_topic_type = "std_msgs/Float64MultiArray";

        cameraPose_topic_name = "/SLAM/CameraPoint";
        cameraPose_topic_type = "geometry_msgs/PoseStamped";
    }

}

// 订阅SLAM点云话题
void SlamMapMonitor::start()
{
    if (!m_worker)
        return;
    // 订阅地图点云PointCloud2数据
    if(!slamPoint_topic_name.isEmpty())
    {
        QJsonObject subscribeMsg;
        subscribeMsg["op"] = "subscribe";
        subscribeMsg["topic"] = slamPoint_topic_name;
        subscribeMsg["type"] = slamPoint_topic_type;
        QJsonDocument doc(subscribeMsg);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        // qDebug() << "SlamMapMonitor::start subscribing:" << payload;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
    }

    // 订阅keyframe topic
    if (!slamKeyFrame_topic_name.isEmpty())
    {
        QJsonObject sub2;
        sub2["op"] = "subscribe";
        sub2["topic"] = slamKeyFrame_topic_name;
        sub2["type"] = slamKeyFrame_topic_type;
        QJsonDocument d2(sub2);
        QString p2 = QString::fromUtf8(d2.toJson(QJsonDocument::Compact));
        // qDebug() << "SlamMapMonitor::start subscribing:" << p2;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, p2));
    }

    // 订阅相机OpenGL矩阵topic
    if(!cameraOpenGLMatrix_topic_name.isEmpty())
    {
        QJsonObject sub3;
        sub3["op"] = "subscribe";
        sub3["topic"] = cameraOpenGLMatrix_topic_name;
        sub3["type"] = cameraOpenGLMatrix_topic_type;
        QJsonDocument d3(sub3);
        QString p3 = QString::fromUtf8(d3.toJson(QJsonDocument::Compact));
        // qDebug() << "SlamMapMonitor::start subscribing:" << p3;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, p3));
    }   

    // 订阅相机位置
    if(!cameraPose_topic_name.isEmpty())
    {
        QJsonObject sub4;
        sub4["op"] = "subscribe";
        sub4["topic"] = cameraPose_topic_name;
        QJsonDocument d4(sub4);
        QString p4 = QString::fromUtf8(d4.toJson(QJsonDocument::Compact));
        // qDebug() << "SlamMapMonitor::start subscribing:" << p4;
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, p4));
    }
}

// 取消SLAM数据订阅
void SlamMapMonitor::stop()
{
    if (!m_worker)
        return;
    // 取消点云数据订阅
    if (slamPoint_topic_name.isEmpty())
    {
        QJsonObject unsub;
        unsub["op"] = "unsubscribe";
        unsub["topic"] = slamPoint_topic_name;
        QJsonDocument doc(unsub);
        QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
    }
    
    // 取消keyframe订阅
    if (!slamKeyFrame_topic_name.isEmpty())
    {
        QJsonObject unsub2;
        unsub2["op"] = "unsubscribe";
        unsub2["topic"] = slamKeyFrame_topic_name;
        QJsonDocument d2(unsub2);
        QString p2 = QString::fromUtf8(d2.toJson(QJsonDocument::Compact));
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, p2));
    }

    // 取消相机OpenGL矩阵订阅
    if (!cameraOpenGLMatrix_topic_name.isEmpty())
    {
        QJsonObject unsub3;
        unsub3["op"] = "unsubscribe";
        unsub3["topic"] = cameraOpenGLMatrix_topic_name;
        QJsonDocument d3(unsub3);
        QString p3 = QString::fromUtf8(d3.toJson(QJsonDocument::Compact));
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, p3));
    }

    // 取消相机位置订阅
    if (!cameraPose_topic_name.isEmpty())
    {
        QJsonObject unsub4;
        unsub4["op"] = "unsubscribe";
        unsub4["topic"] = cameraPose_topic_name;
        QJsonDocument d4(unsub4);
        QString p4 = QString::fromUtf8(d4.toJson(QJsonDocument::Compact));
        QMetaObject::invokeMethod(m_worker, "sendText", Qt::QueuedConnection, Q_ARG(QString, p4));
    }
}

// 接收到SLAM点云消息处理
void SlamMapMonitor::onMessageReceived(const QString &message)
{
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (!doc.isObject())
        return;
    QJsonObject obj = doc.object();

    // rosbridge publish message format: {op:"publish", topic:"...", msg:{...}}
    if (obj["op"].toString() == "publish")
    {
        QString topic = obj["topic"].toString();
        QJsonObject msgObj = obj["msg"].toObject();
        
        // 解析PointCloud2数据结构
        if(topic == slamPoint_topic_name){ 
            QList<QVector3D> points;
            points = parsePointCloud(msgObj);
            if (!points.isEmpty()) {            // 将解析成功的数据发送信号
                emit pointCloudReceived(points);
            }
            else{
                // fallback: print full msg for debugging
                QJsonDocument msgDoc(msgObj);
                QString msgJson = QString::fromUtf8(msgDoc.toJson(QJsonDocument::Compact));
                // qDebug() << "收到SLAM点云 JSON (topic:" << topic << ") :" << msgJson;
            }
        }  else if (topic == slamKeyFrame_topic_name){
            // 收到 keyframe 消息，打印调试信息并解析
            int markersCount = -1;
            if (msgObj.contains("markers") && msgObj["markers"].isArray()) 
            parseKeyFrame(msgObj);
        }   else if(topic == cameraOpenGLMatrix_topic_name){
            // 收到相机OpenGL矩阵消息
            parseOpenGLMatrix(msgObj);
        } else if(topic == cameraPose_topic_name){
            // 收到相机位置消息
            parseCameraPose(msgObj);
        }
    }
}

// 解析PointCloud2数据
QList<QVector3D> SlamMapMonitor::parsePointCloud(const QJsonObject &msgObj)
{
    // 解析PointCloud2数据结构
    QList<QVector3D> points;
    // fields: array of {name, offset, datatype, count}
    QJsonArray fields = msgObj["fields"].toArray(); // fields数组
    int x_offset = -1, y_offset = -1, z_offset = -1;
    int point_step = msgObj["point_step"].toInt(0); // 每个点占用的字节数
    int data_length = msgObj["data"].toArray().size();
    // locate x,y,z offsets
    for (int i = 0; i < fields.size(); ++i)
    {
        QJsonObject f = fields[i].toObject();
        QString name = f["name"].toString();
        int offset = f["offset"].toInt();
        int datatype = f["datatype"].toInt();
        if (name == "x")
            x_offset = offset;
        if (name == "y")
            y_offset = offset;
        if (name == "z")
            z_offset = offset;
    }

    // data might be an array of numbers or a base64 string depending on rosbridge config
    if (msgObj.contains("data") && msgObj["data"].isArray())
    {
        QJsonArray dataArr = msgObj["data"].toArray();
        int total = dataArr.size();
        if (point_step <= 0 && x_offset < 0)
        {
            // fallback: assume packed triples
            for (int i = 0; i + 2 < total; i += 3)
            {
                float vx = float(dataArr[i].toDouble());
                float vy = float(dataArr[i + 1].toDouble());
                float vz = float(dataArr[i + 2].toDouble());
                points.append(QVector3D(vx, vy, vz));
            }
        }
        else
        {
            int point_count = total / point_step;
            for (int p = 0; p < point_count; ++p)
            {
                int base = p * point_step;
                float vx = x_offset >= 0 ? float(dataArr[base + x_offset].toDouble()) : 0.0f;
                float vy = y_offset >= 0 ? float(dataArr[base + y_offset].toDouble()) : 0.0f;
                float vz = z_offset >= 0 ? float(dataArr[base + z_offset].toDouble()) : 0.0f;
                points.append(QVector3D(vx, vy, vz));
            }
        }
    }
    else if (msgObj.contains("data") && msgObj["data"].isString())
    {
        // data is base64 string
        QString b64 = msgObj["data"].toString();
        QByteArray raw = QByteArray::fromBase64(b64.toUtf8());
        if (point_step <= 0)
            point_step = 12; // 3 floats
        int point_count = raw.size() / point_step;
        for (int p = 0; p < point_count; ++p)
        {
            int base = p * point_step;
            float vx = 0, vy = 0, vz = 0;
            if (x_offset >= 0)
                memcpy(&vx, raw.constData() + base + x_offset, sizeof(float));
            if (y_offset >= 0)
                memcpy(&vy, raw.constData() + base + y_offset, sizeof(float));
            if (z_offset >= 0)
                memcpy(&vz, raw.constData() + base + z_offset, sizeof(float));
            points.append(QVector3D(vx, vy, vz));
        }
    }
    return points;
}

// 解析 MarkerArray 消息并提取 POINTS 与 LINE_LIST/LINE_STRIP 标记
void SlamMapMonitor::parseKeyFrame(const QJsonObject &msgObj)
{
    // msgObj 是一个 visualization_msgs/MarkerArray 类型，即 { markers: [ { type, points:[{x,y,z},...], ... }, ... ] }
    if (!msgObj.contains("markers") || !msgObj["markers"].isArray()) return;
    QJsonArray markers = msgObj["markers"].toArray();

    // qDebug() << "SlamMapMonitor::parseKeyFrame: received markers size=" << markers.size();

    QList<QVector3D> points;
    QList<QVector3D> lines; // sequential pairs

    for (int i = 0; i < markers.size(); ++i) {
        QJsonObject m = markers[i].toObject();
        int type = m["type"].toInt(-1);
        // geometry may be in m["points"] as array of {x,y,z}
        if (m.contains("points") && m["points"].isArray()) {
            QJsonArray parr = m["points"].toArray();
            if (type == 8) { // visualization_msgs::Marker::POINTS == 8
                for (int j = 0; j < parr.size(); ++j) {
                    QJsonObject pv = parr[j].toObject();
                    float x = float(pv.value("x").toDouble());
                    float y = float(pv.value("y").toDouble());
                    float z = float(pv.value("z").toDouble());
                    points.append(QVector3D(x, y, z));
                }
            } else if (type == 5 || type == 10) { // LINE_STRIP(5) or LINE_LIST(10)
                // for LINE_LIST, points are pairs; for LINE_STRIP, connect sequentially
                if (type == 10) {
                    // LINE_LIST: every pair defines a segment
                    for (int j = 0; j + 1 < parr.size(); j += 2) {
                        QJsonObject p0 = parr[j].toObject();
                        QJsonObject p1 = parr[j + 1].toObject();
                        QVector3D a(float(p0.value("x").toDouble()), float(p0.value("y").toDouble()), float(p0.value("z").toDouble()));
                        QVector3D b(float(p1.value("x").toDouble()), float(p1.value("y").toDouble()), float(p1.value("z").toDouble()));
                        lines.append(a);
                        lines.append(b);
                    }
                } else {
                    // LINE_STRIP: connect sequential points
                    QJsonObject prev;
                    bool hasPrev = false;
                    for (int j = 0; j < parr.size(); ++j) {
                        QJsonObject pv = parr[j].toObject();
                        QVector3D cur(float(pv.value("x").toDouble()), float(pv.value("y").toDouble()), float(pv.value("z").toDouble()));
                        if (hasPrev) {
                            QVector3D pre(float(prev.value("x").toDouble()), float(prev.value("y").toDouble()), float(prev.value("z").toDouble()));
                            lines.append(pre);
                            lines.append(cur);
                        }
                        prev = pv;
                        hasPrev = true;
                    }
                }
            }
        }

        // always check pose.position (some markers include a pose even if points exists or is empty)
        if (m.contains("pose") && m["pose"].isObject()) {
            QJsonObject pose = m["pose"].toObject();
            QJsonObject pos = pose.value("position").toObject();
            if (pos.contains("x") || pos.contains("y") || pos.contains("z")) {
                float x = float(pos.value("x").toDouble());
                float y = float(pos.value("y").toDouble());
                float z = float(pos.value("z").toDouble());
                points.append(QVector3D(x, y, z));
            }
        }
    }

    if (!points.isEmpty() || !lines.isEmpty()) {
        // qDebug() << "SlamMapMonitor::parseKeyFrame parsed" << points.size() << "points," << lines.size() << "line endpoints";
        emit keyFrameMarkers(points, lines);
    }
}

// 解析相机OpenGL矩阵消息
void SlamMapMonitor::parseOpenGLMatrix(const QJsonObject &msgObj){
    // Expecting std_msgs/Float64MultiArray-like JSON: { layout:..., data: [ ... ] }
    if (!msgObj.contains("data")) return;
    QList<double> mat;
    if (msgObj["data"].isArray()){
        QJsonArray arr = msgObj["data"].toArray();
        mat.reserve(arr.size());
        for (int i=0;i<arr.size();++i){
            mat.append(arr[i].toDouble());
        }
    } else if (msgObj["data"].isString()){
        // if data is encoded as base64 of doubles (unlikely), attempt to decode
        QByteArray b = QByteArray::fromBase64(msgObj["data"].toString().toUtf8());
        int n = b.size()/sizeof(double);
        mat.reserve(n);
        const double *ptr = reinterpret_cast<const double*>(b.constData());
        for (int i=0;i<n;++i) mat.append(ptr[i]);
    }

    if (!mat.isEmpty()){
        emit cameraMatrixReceived(mat);
    }
}

// 解析相机位置消息
void SlamMapMonitor::parseCameraPose(const QJsonObject &msgObj){
    QJsonObject poseObj;
    if (msgObj.contains("pose") && msgObj["pose"].isObject()) {
        // could be PoseStamped: msgObj["pose"]["pose"] or Pose geometry directly
        QJsonObject p = msgObj["pose"].toObject();
        if (p.contains("pose") && p["pose"].isObject()) poseObj = p["pose"].toObject();
        else poseObj = p;
    } else if (msgObj.contains("data") && msgObj["data"].isObject()) {
        poseObj = msgObj["data"].toObject();
    } else {
        // fallback: maybe msgObj itself is a Pose
        poseObj = msgObj;
    }

    // Determine position object: try pose.position (PoseStamped), then msg.point (PointStamped),
    // then poseObj itself if it directly contains x/y/z
    QJsonObject posObj;
    QString usedField;
    if (poseObj.contains("position") && poseObj["position"].isObject()) {
        posObj = poseObj["position"].toObject();
        usedField = "pose.position";
    } else if (msgObj.contains("point") && msgObj["point"].isObject()) {
        posObj = msgObj["point"].toObject();
        usedField = "point";
    } else if (poseObj.contains("x") || poseObj.contains("y") || poseObj.contains("z")) {
        posObj = poseObj;
        usedField = "direct";
    } else {
        return;
    }

    double px = posObj.value("x").toDouble(0.0);
    double py = posObj.value("y").toDouble(0.0);
    double pz = posObj.value("z").toDouble(0.0);

    // default forward vector (+Z in camera local coordinates)
    QVector3D dir(0,0,1);

    // if (poseObj.contains("orientation") && poseObj["orientation"].isObject()) {
    //     QJsonObject ori = poseObj["orientation"].toObject();
    //     double ox = ori.value("x").toDouble(0.0);
    //     double oy = ori.value("y").toDouble(0.0);
    //     double oz = ori.value("z").toDouble(0.0);
    //     double ow = ori.value("w").toDouble(1.0);

    //     // rotate local +Z by quaternion (ox,oy,oz,ow)
    //     // q * v * q^{-1}
    //     // compute q * (0,0,1) * q^{-1} analytically
    //     // t = 2 * cross(q.xyz, v)
    //     // v' = v + q.w * t + cross(q.xyz, t)

    //     QVector3D qv(ox, oy, oz);
    //     QVector3D v(0,0,1);
    //     QVector3D t = 2.0 * QVector3D::crossProduct(qv, v);
    //     QVector3D vprime = v + static_cast<float>(ow) * t + QVector3D::crossProduct(qv, t);
    //     if (vprime.length() > 1e-6f) {
    //         vprime.normalize();
    //         dir = vprime;
    //     }
    // }

    QVector3D posf{static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz)};
    emit cameraPoseReceived(posf, dir);
}