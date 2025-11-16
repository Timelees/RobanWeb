#ifndef SERVOPOSITIONS_H
#define SERVOPOSITIONS_H

#include <QObject>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QMetaObject>
#include <QMutex>
#include <QVector3D>

class WebSocketWorker;

class ServoPositionsMonitor: public QObject{
    Q_OBJECT
public:
    explicit ServoPositionsMonitor(WebSocketWorker *worker, QObject *parent = nullptr);
    ~ServoPositionsMonitor();

public slots:
    void start(); // send subscribe request via worker
    void onMessageReceived(const QString &message);  
  
    
public:
    WebSocketWorker *m_worker;

private:
    
    QString servo_positions_topic_name; // 伺服位置话题名称
    QString servo_positions_topic_type; // 伺服位置话题类型
    QString walking_status_topic_name; // 机器人行走状态话题名称
    QString walking_status_topic_type; // 机器人行走状态话题类型

    // 保存最近一次接收到的消息对象（publish->msg）
    mutable QMutex m_mutex;
    QJsonObject m_lastMsg;
    // 保存最近解析出的角度数组
    QVector<double> m_lastAngles;
    // 批处理设置：累积 N 条消息后再发出一次更新以减少主线程负载
    int m_msgCounter = 0;
    int m_batchSize = 15; // 默认每15条消息触发一次更新

signals:
    // 当收到新的伺服位置消息时发出（仅表示已接收，新数据存储在 lastMessage()）
    void servoPositionsUpdated();
    // 当收到新的行走状态消息时发出
    void walkingStatusUpdated();
    // 当收到行走状态恢复为非行走（0）时发出
    void walkingStatusStopped();

public:
    // 返回最近一次解析得到的角度数组（线程安全）
    QVector<double> lastAngles() const;
    // 设置批大小（>=1）
    void setBatchSize(int n) { if (n > 0) m_batchSize = n; }

};



#endif // SERVOPOSITIONS_H