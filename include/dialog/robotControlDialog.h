#ifndef ROBOTCONTROLDIALOG_H
#define ROBOTCONTROLDIALOG_H

#include <QDialog>
#include <QDebug>
#include <QPushButton>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>

// 前向声明 UI 类
namespace Ui
{
    class RobotControlDialog;
} // namespace Ui

class WebSocketWorker;

class RobotControlDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RobotControlDialog(WebSocketWorker *webSocketWorker, QWidget *parent = nullptr);
    ~RobotControlDialog();

private slots:
    void onStartControlButtonClicked();
    void onCancelControlButtonClicked();
    void onControlButtonClicked();

private:
    void bindSlots();
    QString loadCmdFromConfig(const QString &key);

private:
    Ui::RobotControlDialog *ui;
    WebSocketWorker *m_worker;
};

#endif // ROBOTCONTROLDIALOG_H