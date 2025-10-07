#include "socket_process/websocketworker.h"

WebSocketWorker::WebSocketWorker(QObject *parent)
    : QObject(parent), m_webSocket(nullptr), m_reconnectTimer(nullptr), m_isReconnecting(false), m_reconnectAttempts(0)
{

}

WebSocketWorker::~WebSocketWorker()
{
    if (m_webSocket) {
        // ensure deletion happens in the thread that owns the object
        m_webSocket->deleteLater();
        m_webSocket = nullptr;
    }
    if (m_reconnectTimer) {
        m_reconnectTimer->deleteLater();
        m_reconnectTimer = nullptr;
    }
}
// url 规范化处理
static QUrl normalizeUrl(const QString &in)
{
    QUrl url = QUrl::fromUserInput(in);
    if (url.scheme().isEmpty()) {
        url.setScheme(QStringLiteral("ws"));
    }
    return url;
}
// 初始化设置websocket和定时器
void WebSocketWorker::init()
{
    if (!m_webSocket) {
        m_webSocket = new QWebSocket();
        m_webSocket->setProxy(QNetworkProxy::NoProxy);
        connect(m_webSocket, &QWebSocket::connected, this, &WebSocketWorker::onConnected);
        connect(m_webSocket, &QWebSocket::disconnected, this, &WebSocketWorker::onDisconnected);
        connect(m_webSocket, &QWebSocket::textMessageReceived, this, &WebSocketWorker::onTextMessageReceived);
        connect(m_webSocket, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error), this, &WebSocketWorker::onError);
    }

    if (!m_reconnectTimer) {
        m_reconnectTimer = new QTimer(this);
        m_reconnectTimer->setInterval(3000);
        m_reconnectTimer->setSingleShot(true);
        connect(m_reconnectTimer, &QTimer::timeout, this, [this]() {
            if (!m_url.isEmpty() && m_webSocket) {
                QUrl q = normalizeUrl(m_url);
                qDebug() << "WebSocketWorker: reconnecting to" << q.toString();
                m_webSocket->open(q);
                m_reconnectAttempts++;
            }
        });
    }
}
// 从主线程调用，启动连接
void WebSocketWorker::startConnect(const QString &url)
{
    m_url = url;
    m_isReconnecting = true;
    m_reconnectAttempts = 0;
    QUrl q = normalizeUrl(url);
    qDebug() << "WebSocketWorker: startConnect" << q.toString() << " on thread " << QThread::currentThread();
    if (!m_webSocket) init();
    if (m_webSocket) {
        m_webSocket->open(q);
    }
}
// 断开连接
void WebSocketWorker::closeConnection()
{
    m_isReconnecting = false;
    if (m_reconnectTimer) {
        m_reconnectTimer->stop();
    }
    if (m_webSocket) {
        m_webSocket->close();
    }
}

void WebSocketWorker::sendText(const QString &text)
{
    if (m_webSocket && m_webSocket->state() == QAbstractSocket::ConnectedState) {
        m_webSocket->sendTextMessage(text);
    } else {
        qDebug() << "WebSocketWorker: cannot send, socket not connected";
    }
}
// 连接成功
void WebSocketWorker::onConnected()
{
    qDebug() << "WebSocketWorker: onConnected on thread" << QThread::currentThread();
    m_isReconnecting = false;
    if (m_reconnectTimer) {
        m_reconnectTimer->stop();
    }
    emit connected();
}
// 连接断开
void WebSocketWorker::onDisconnected()
{
    qDebug() << "WebSocketWorker: onDisconnected";
    emit disconnected();
    if (m_isReconnecting) {
        m_reconnectTimer->start();
    }
}
// 从QWebSocket接收到消息时，直接把原始消息发回主线程，由主线程处理 JSON/UI 更新
void WebSocketWorker::onTextMessageReceived(const QString &message)
{
    emit messageReceived(message);
}

void WebSocketWorker::onError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error)
    QString err = m_webSocket ? m_webSocket->errorString() : QString("unknown");
    qDebug() << "WebSocketWorker: error:" << err;
    emit errorOccurred(err);
}
