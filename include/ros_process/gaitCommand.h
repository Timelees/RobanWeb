#ifndef GAITCOMMAND_H
#define GAITCOMMAND_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>
#include <QMutex>
#include <QVector3D>

class WebSocketWorker;

class GaitCommandMonitor: public QObject{
    Q_OBJECT
public:
    explicit GaitCommandMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~GaitCommandMonitor();

public slots:
    void start(); // send subscribe request via worker
    void onMessageReceived(const QString &message);  

public:
    WebSocketWorker *m_worker;
    
private:
    QString gait_command_topic_name; // 步态命令话题名称
    QString gait_command_topic_type; // 步态命令话题类型

public:
    // 线程安全访问最近解析到的步态命令分量
    double gaitX() const;
    double gaitY() const;
    double gaitDelta() const;

signals:
    // 当解析到新的步态命令数据时发出
    void gaitCommandUpdated();

private:
    mutable QMutex m_mutex;
    double m_x = 0.0;
    double m_y = 0.0;
    double m_delta = 0.0;

};


#endif // GAITCOMMAND_H