#pragma once

#include <QString>
#include <QDir>
#include <QFile>
#include <QCoreApplication>
#include <QTextStream>
#include <QVector>
#include <QStringList>

// 简单 CSV 加载器，针对 config 目录下的 csv 文件。
// 用法：
// QVector<QStringList> rows;
// bool ok = loadCsv("place.csv", rows);
// 首先会尝试在应用程序目录的 config/、上一级的 config/、以及当前工作目录的 config/ 中查找文件。

inline QString resolveConfigPath(const QString &filename)
{
    if (filename.isEmpty())
        return QString();
    // 如果传入的是绝对路径或相对到当前工作目录且存在，则直接返回
    if (QFile::exists(filename))
        return QDir::cleanPath(filename);

    QDir appdir(QCoreApplication::applicationDirPath());
    QStringList candidates = {
        appdir.filePath(QString("../config/%1").arg(filename)),
        appdir.filePath(QString("config/%1").arg(filename)),
        QDir::current().filePath(QString("../config/%1").arg(filename)),
        QDir::current().filePath(QString("config/%1").arg(filename))
    };
    for (const QString &p : candidates) {
        if (QFile::exists(p)) {
            // qDebug() << "resolveConfigPath: resolved " << filename << " to " << p;
            return QDir::cleanPath(p);
        }
    }
    return QString();
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
