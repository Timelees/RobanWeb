#pragma once

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
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

struct SimpleMesh
{
    std::vector<float> vertices;  // x,y,z
    std::vector<float> normals;   // x,y,z
    std::vector<float> texcoords; // u,v
    std::vector<unsigned int> indices;
    // optional per-vertex colors (r,g,b,a) matching vertices count (4 floats per vertex)
    std::vector<float> colors;    // r,g,b,a
    QString diffuseTexPath;
    QImage diffuseImage;
    unsigned int texId = 0;
};

// simple 4x4 matrix helper (minimal)
struct Mat4
{
    float m[4][4];
    Mat4() { memset(m, 0, sizeof(m)); }
    static Mat4 identity()
    {
        Mat4 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
        return r;
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
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                r.m[i][j] = 0;
                for (int k = 0; k < 4; ++k)
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
        float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        return det;
    }

    // return max row norm of top-left 3x3
    float maxRowNorm3() const
    {
        float maxn = 0.0f;
        for (int r = 0; r < 3; ++r)
        {
            float v0 = m[r][0], v1 = m[r][1], v2 = m[r][2];
            float n = sqrtf(v0 * v0 + v1 * v1 + v2 * v2);
            if (n > maxn)
                maxn = n;
        }
        return maxn;
    }

    static Mat4 orthonormalizeRotationKeepTranslation(const Mat4 &src)
    {
        Mat4 out = src;
        // extract columns
        float c0x = src.m[0][0], c0y = src.m[1][0], c0z = src.m[2][0];
        float c1x = src.m[0][1], c1y = src.m[1][1], c1z = src.m[2][1];
        float c2x = src.m[0][2], c2y = src.m[1][2], c2z = src.m[2][2];

        auto norm = [](float x, float y, float z)
        { return sqrtf(x * x + y * y + z * z); };
        float n0 = norm(c0x, c0y, c0z);
        if (n0 < 1e-8f)
        {
            // fallback to identity rotation
            out = Mat4::identity();
            out.m[0][3] = src.m[0][3];
            out.m[1][3] = src.m[1][3];
            out.m[2][3] = src.m[2][3];
            return out;
        }
        float u0x = c0x / n0, u0y = c0y / n0, u0z = c0z / n0;
        // subtract projection of c1 onto u0
        float dot1 = c1x * u0x + c1y * u0y + c1z * u0z;
        float t1x = c1x - dot1 * u0x, t1y = c1y - dot1 * u0y, t1z = c1z - dot1 * u0z;
        float n1 = norm(t1x, t1y, t1z);
        if (n1 < 1e-8f)
        {
            // create an arbitrary perpendicular vector
            if (fabs(u0x) < fabs(u0y))
            {
                t1x = 0;
                t1y = -u0z;
                t1z = u0y;
            }
            else
            {
                t1x = -u0z;
                t1y = 0;
                t1z = u0x;
            }
            n1 = norm(t1x, t1y, t1z);
        }
        float u1x = t1x / n1, u1y = t1y / n1, u1z = t1z / n1;
        // u2 = cross(u0, u1)
        float u2x = u0y * u1z - u0z * u1y;
        float u2y = u0z * u1x - u0x * u1z;
        float u2z = u0x * u1y - u0y * u1x;
        // write back as columns
        out.m[0][0] = u0x;
        out.m[1][0] = u0y;
        out.m[2][0] = u0z;
        out.m[0][1] = u1x;
        out.m[1][1] = u1y;
        out.m[2][1] = u1z;
        out.m[0][2] = u2x;
        out.m[1][2] = u2y;
        out.m[2][2] = u2z;
        // keep translation
        out.m[0][3] = src.m[0][3];
        out.m[1][3] = src.m[1][3];
        out.m[2][3] = src.m[2][3];
        out.m[3][3] = 1.0f;
        return out;
    }
};

struct NodeInfo
{
    QString name;
    Mat4 local;
    Mat4 originalLocal;
    int parent = -1;
    std::vector<int> children;
};

struct BoneInfo
{
    QString name;
    Mat4 offset;        // offset matrix 偏移矩阵
    int nodeIndex = -1; // index into nodeInfos 节点信息索引
    // allow multiple joint mappings per bone (bone can be driven by several joint IDs on different axes)
    std::vector<std::pair<int, char>> jointMappings; // (jointId, axis) 关节映射
};

// Compact per-vertex influence
struct CompactInfluence
{
    unsigned char count;
    int boneIdx[4];
    float weight[4];
};