#pragma once

#include <QString>
#include <QDir>
#include <QFile>
#include <QCoreApplication>
#include <QTextStream>
#include <QVector>
#include <QStringList>
#include <QStandardPaths>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

// 简单 CSV 加载器，针对 config 目录下的 csv 文件。
// 用法：
// QVector<QStringList> rows;
// bool ok = loadCsv("place.csv", rows);
// 首先会尝试在应用程序目录的 config/、上一级的 config/、以及当前工作目录的 config/ 中查找文件。

// 更鲁棒的 resolveConfigPath：
// - 支持环境变量覆盖：ROBANWEB_CONFIG_DIR
// - 支持直接传入的路径
// - 在 applicationDir、currentDir 及其向上父目录中查找 config/<filename>
// - 尝试 QStandardPaths::AppConfigLocation
// 返回第一个存在的路径；若未找到，返回第一个候选以便用于保存或调试（与历史兼容）
inline QString resolveConfigPath(const QString &filename)
{
    if (filename.isEmpty()) return QString();

    // 1) environment override
    QByteArray env = qgetenv("ROBANWEB_CONFIG_DIR");
    if (!env.isEmpty()) {
        QString envDir = QString::fromLocal8Bit(env);
        QString p = QDir::cleanPath(envDir + QDir::separator() + filename);
        if (QFile::exists(p)) return p;
    }

    // 2) caller-provided path (absolute or relative)
    if (QFile::exists(filename)) return QDir::cleanPath(filename);

    QString appDir = QCoreApplication::applicationDirPath();
    QString curDir = QDir::currentPath();

    QStringList candidates;
    auto push = [&candidates](const QString &s){ QString c = QDir::cleanPath(s); if (!c.isEmpty() && !candidates.contains(c)) candidates << c; };

    push(appDir + QDir::separator() + "config" + QDir::separator() + filename);
    push(appDir + QDir::separator() + filename);
    push(curDir + QDir::separator() + "config" + QDir::separator() + filename);
    push(curDir + QDir::separator() + filename);
    push(appDir + QDir::separator() + ".." + QDir::separator() + "config" + QDir::separator() + filename);

    // walk up to find project-level config directories
    auto walkUp = [&](const QString &start){
        QFileInfo fi(start);
        QString dir = fi.absolutePath();
        int levels = 0;
        while (!dir.isEmpty() && levels < 12) {
            push(dir + QDir::separator() + "config" + QDir::separator() + filename);
            push(dir + QDir::separator() + filename);
            QFileInfo pdir(dir);
            QString parent = pdir.dir().absolutePath();
            if (parent == dir) break;
            dir = parent;
            levels++;
        }
    };
    walkUp(appDir);
    if (curDir != appDir) walkUp(curDir);

    // QStandardPaths fallback
    QString stdCfg = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (!stdCfg.isEmpty()) push(stdCfg + QDir::separator() + filename);

    // return first existing candidate
    for (const QString &p : candidates) if (QFile::exists(p)) return p;

    // nothing exists: return first candidate (useful when saving into config)
    if (!candidates.isEmpty()) return candidates.first();
    return QString();
}

// 保存 CSV 文件到 config 中的首选候选位置（如果目标目录不存在会尝试创建）
inline bool saveCsvToConfig(const QString &filename, const QVector<QStringList> &rows, QChar sep = ',')
{
    if (filename.isEmpty()) return false;
    QString path = resolveConfigPath(filename);
    if (path.isEmpty()) return false;
    QFileInfo fi(path);
    QDir dir = fi.dir();
    if (!dir.exists()) {
        if (!QDir().mkpath(dir.absolutePath())) return false;
    }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream out(&f);
    for (const QStringList &cols : rows) {
        QStringList esc;
        for (const QString &c : cols) {
            QString s = c;
            if (s.contains(sep) || s.contains('"') || s.contains('\n')) {
                s.replace('"', "\"");
                s = '"' + s + '"';
            }
            esc << s;
        }
        out << esc.join(sep) << "\n";
    }
    f.close();
    return true;
}

// Save arbitrary JSON object text to config/<filename> (filename typically ends with .json)
inline bool saveJsonToConfig(const QString &filename, const QJsonDocument &doc)
{
    if (filename.isEmpty()) return false;
    QString path = resolveConfigPath(filename);
    if (path.isEmpty()) return false;
    QFileInfo fi(path);
    QDir dir = fi.dir();
    if (!dir.exists()) {
        if (!QDir().mkpath(dir.absolutePath())) return false;
    }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    f.write(doc.toJson(QJsonDocument::Indented));
    f.close();
    return true;
}

// 解析单行 CSV（支持用双引号包裹并允许在引号中包含分隔符和双引号转义"
inline QStringList parseCsvLine(const QString &line, QChar sep = ',')
{
    QStringList fields;
    QString cur;
    bool inQuotes = false;
    int i = 0;
    while (i < line.size()) {
        QChar c = line.at(i);
        if (!inQuotes) {
            if (c == '"') {
                inQuotes = true;
                // 开始引号，不把引号本身加入字段
            } else if (c == sep) {
                fields.append(cur.trimmed());
                cur.clear();
            } else {
                cur.append(c);
            }
            ++i;
        } else {
            // in quotes
            if (c == '"') {
                // 看下一个字符是否也是引号（转义）
                if (i + 1 < line.size() && line.at(i + 1) == '"') {
                    cur.append('"');
                    i += 2;
                } else {
                    // 结束引号
                    inQuotes = false;
                    ++i;
                }
            } else {
                cur.append(c);
                ++i;
            }
        }
    }
    // append last field
    fields.append(cur.trimmed());
    return fields;
}

// 从 config 中加载 CSV 文件到行字段集合（每行为 QStringList），返回是否成功找到并解析文件
inline bool loadCsv(const QString &filename, QVector<QStringList> &outRows, QChar sep = ',')
{
    outRows.clear();
    QString path = resolveConfigPath(filename);
    if (path.isEmpty()) {
        // 也允许传入带相对路径的 filename，比如 "../config/place.csv" 之前 resolve 会已处理
        if (!QFile::exists(filename))
            return false;
        path = QDir::cleanPath(filename);
    }

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QTextStream in(&f);
    while (!in.atEnd()) {
        QString line = in.readLine();
        // 跳过空行与以 # 或 // 开头的注释行
        QString t = line.trimmed();
        if (t.isEmpty()) continue;
        if (t.startsWith('#') || t.startsWith("//")) continue;

        QStringList cols = parseCsvLine(line, sep);
        outRows.append(cols);
    }
    f.close();
    return !outRows.isEmpty();
}

// 便捷载入：返回 vector of rows（空表示失败或空文件）
inline QVector<QStringList> loadCsvToRows(const QString &filename, QChar sep = ',')
{
    QVector<QStringList> rows;
    loadCsv(filename, rows, sep);
    return rows;
}
