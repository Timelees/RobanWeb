#ifndef SHDIALOG_H
#define SHDIALOG_H

#include <QDialog>
#include <QDebug>
#include <QPushButton>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>

namespace Ui
{
    class ShDialog;
} // namespace Ui

class WebSocketWorker;

class ShDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ShDialog(WebSocketWorker *webSocketWorker, QWidget *parent = nullptr);
    ~ShDialog();

private slots:
    void onRunButtonClicked();
    void onCloseButtonClicked();

signals:
    void runScriptRequested(const QString &cmd);
    void closeScriptRequested();

private:
    Ui::ShDialog *ui;
    WebSocketWorker *m_worker;
};

#endif // SHDIALOG_H