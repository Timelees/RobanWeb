#ifndef WEBSOCKETWORKER_H
#define WEBSOCKETWORKER_H

#include <QObject>
#include <QWebSocket>
#include <QTimer>
#include <QDebug>
#include <QThread>
#include <QUrl>
#include <QNetworkProxy>


class WebSocketWorker : public QObject
{
    Q_OBJECT
public:
    explicit WebSocketWorker(QObject *parent = nullptr);
    ~WebSocketWorker();

public slots:
    void init(); // create QWebSocket/QTimer in worker thread
    void startConnect(const QString &url);
    void closeConnection();
    void sendText(const QString &text);

signals:
    void connected();
    void disconnected();
    void errorOccurred(const QString &error);
    void messageReceived(const QString &message);

private slots:
    void onConnected();
    void onDisconnected();
    void onTextMessageReceived(const QString &message);
    void onError(QAbstractSocket::SocketError error);

private:
    QWebSocket *m_webSocket;
    QTimer *m_reconnectTimer;
    QString m_url;
    bool m_isReconnecting;
    int m_reconnectAttempts;
};

#endif // WEBSOCKETWORKER_H
