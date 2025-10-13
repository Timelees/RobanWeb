#include <QString>
#include <QDir>
#include <QFile>    
#include <QRegularExpression>
#include <QRegularExpressionMatch>  
#include <QCoreApplication>

// 从config/topic_config.yaml加载命令
inline QString loadTopicFromConfig(const QString &key)
{
    // config file path relative to application root
    QDir d(QCoreApplication::applicationDirPath());
    // try a few likely locations: ../config, ./config
    QStringList candidates = {
        d.filePath("config/topic_config.yaml"),
        d.filePath("../config/topic_config.yaml"),
        QDir::current().filePath("config/topic_config.yaml")
    };
    QString content;
    for (const QString &path : candidates) {
        QFile f(path);
        if (f.exists() && f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            content = QString::fromUtf8(f.readAll());
            f.close();
            break;
        }
    }
    if (content.isEmpty()) return QString();
    // Very small YAML: key: "value"
    QRegularExpression re(QStringLiteral("^%1:\\s*\"?(.*)\"?$").arg(QRegularExpression::escape(key))); 
    re.setPatternOptions(QRegularExpression::MultilineOption);
    QRegularExpressionMatch m = re.match(content);
    if (m.hasMatch()) {
        QString val = m.captured(1).trimmed();
        // Normalize/remove surrounding quotes: ASCII " or ' and common Unicode “ ” ‘ ’
        if (val.size() >= 2) {
            QChar first = val.front();
            QChar last = val.back();
            if ((first == '"' && last == '"') || (first == '\'' && last == '\'')
                || (first == QChar(0x201C) && last == QChar(0x201D))
                || (first == QChar(0x2018) && last == QChar(0x2019))) {
                val = val.mid(1, val.size() - 2).trimmed();
            }
        }
        // Strip any stray leading/trailing quote characters that may remain
        while (!val.isEmpty() && (val.front() == '"' || val.front() == '\'' || val.front() == QChar(0x201C) || val.front() == QChar(0x2018))) {
            val.remove(0, 1);
        }
        while (!val.isEmpty() && (val.back() == '"' || val.back() == '\'' || val.back() == QChar(0x201D) || val.back() == QChar(0x2019))) {
            val.chop(1);
        }
        // Unescape simple sequences (e.g. \" -> ") and double-backslashes
        val.replace("\\\"", "\"");
        val.replace("\\\\", "\\");
        return val;
    }
    return QString();
}


// 从config/bash_config.yaml加载命令
inline QString loadCmdFromConfig(const QString &key)
{
    // config file path relative to application root
    QDir d(QCoreApplication::applicationDirPath());
    // try a few likely locations: ../config, ./config
    QStringList candidates = {
        d.filePath("config/bash_config.yaml"),
        d.filePath("../config/bash_config.yaml"),
        QDir::current().filePath("config/bash_config.yaml")
    };
    QString content;
    for (const QString &path : candidates) {
        QFile f(path);
        if (f.exists() && f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            content = QString::fromUtf8(f.readAll());
            f.close();
            break;
        }
    }
    if (content.isEmpty()) return QString();
    // Very small YAML: key: "value"
    QRegularExpression re(QStringLiteral("^%1:\\s*\"?(.*)\"?$").arg(QRegularExpression::escape(key))); 
    re.setPatternOptions(QRegularExpression::MultilineOption);
    QRegularExpressionMatch m = re.match(content);
    if (m.hasMatch()) {
        QString val = m.captured(1).trimmed();
        // Normalize/remove surrounding quotes: ASCII " or ' and common Unicode “ ” ‘ ’
        if (val.size() >= 2) {
            QChar first = val.front();
            QChar last = val.back();
            if ((first == '"' && last == '"') || (first == '\'' && last == '\'')
                || (first == QChar(0x201C) && last == QChar(0x201D))
                || (first == QChar(0x2018) && last == QChar(0x2019))) {
                val = val.mid(1, val.size() - 2).trimmed();
            }
        }
        // Strip any stray leading/trailing quote characters that may remain
        while (!val.isEmpty() && (val.front() == '"' || val.front() == '\'' || val.front() == QChar(0x201C) || val.front() == QChar(0x2018))) {
            val.remove(0, 1);
        }
        while (!val.isEmpty() && (val.back() == '"' || val.back() == '\'' || val.back() == QChar(0x201D) || val.back() == QChar(0x2019))) {
            val.chop(1);
        }
        // Unescape simple sequences (e.g. \" -> ") and double-backslashes
        val.replace("\\\"", "\"");
        val.replace("\\\\", "\\");
        return val;
    }
    return QString();
}