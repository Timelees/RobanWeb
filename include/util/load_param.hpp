#pragma once
#include <QString>
#include <QDir>
#include <QFile>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QCoreApplication>
#include "load_csv.hpp"

// 从config/topic_config.yaml加载命令（使用 resolveConfigPath 来解析位置）
inline QString loadTopicFromConfig(const QString &key)
{
    QString path = resolveConfigPath("topic_config.yaml");
    if (path.isEmpty()) return QString();
    QFile f(path);
    if (!f.exists() || !f.open(QIODevice::ReadOnly | QIODevice::Text)) return QString();
    QString content = QString::fromUtf8(f.readAll());
    f.close();
    if (content.isEmpty()) return QString();
    QRegularExpression re(QStringLiteral("^%1:\\s*\"?(.*)\"?$").arg(QRegularExpression::escape(key)));
    re.setPatternOptions(QRegularExpression::MultilineOption);
    QRegularExpressionMatch m = re.match(content);
    if (m.hasMatch()) {
        QString val = m.captured(1).trimmed();
        // Normalize and unescape
        if (val.size() >= 2) {
            QChar first = val.front();
            QChar last = val.back();
            if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
                val = val.mid(1, val.size() - 2).trimmed();
            }
        }
        while (!val.isEmpty() && (val.front() == '"' || val.front() == '\'')) val.remove(0,1);
        while (!val.isEmpty() && (val.back() == '"' || val.back() == '\'')) val.chop(1);
        val.replace("\\\"", "\"");
        val.replace("\\\\", "\\");
        return val;
    }
    return QString();
}

// 从config/bash_config.yaml加载命令
inline QString loadCmdFromConfig(const QString &key)
{
    QString path = resolveConfigPath("bash_config.yaml");
    if (path.isEmpty()) return QString();
    QFile f(path);
    if (!f.exists() || !f.open(QIODevice::ReadOnly | QIODevice::Text)) return QString();
    QString content = QString::fromUtf8(f.readAll());
    f.close();
    if (content.isEmpty()) return QString();
    QRegularExpression re(QStringLiteral("^%1:\\s*\"?(.*)\"?$").arg(QRegularExpression::escape(key)));
    re.setPatternOptions(QRegularExpression::MultilineOption);
    QRegularExpressionMatch m = re.match(content);
    if (m.hasMatch()) {
        QString val = m.captured(1).trimmed();
        if (val.size() >= 2) {
            QChar first = val.front();
            QChar last = val.back();
            if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
                val = val.mid(1, val.size() - 2).trimmed();
            }
        }
        while (!val.isEmpty() && (val.front() == '"' || val.front() == '\'')) val.remove(0,1);
        while (!val.isEmpty() && (val.back() == '"' || val.back() == '\'')) val.chop(1);
        val.replace("\\\"", "\"");
        val.replace("\\\\", "\\");
        return val;
    }
    return QString();
}