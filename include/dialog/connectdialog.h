#ifndef CONNECTDIALOG_H
#define CONNECTDIALOG_H

#include <QDialog>
#include <QDebug>
#include <QTableWidgetItem>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QStandardPaths>
#include <QDir>

namespace Ui
{
    class ConnectDialog;
}

class ConnectDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConnectDialog(QWidget *parent = nullptr);
    ~ConnectDialog();

signals:
    void connectRequested(const QString &url);    // 建立连接请求
    void connectRequested2(const QString &url);   // 建立机器人2连接请求
    // 取消连接请求（分别用于机器人1和机器人2的取消按钮）
    void cancelRequested();
    void cancelRequested2();

private slots:
    void onAddButtonClicked();
    void onDeleteButtonClicked();
    void onConnectButtonClicked();
    void onCancelButtonClicked();

    void onRobot1ToggleButtonClicked();
    void onRobot2ToggleButtonClicked();
    void onRobot2AddButtonClicked();
    void onRobot2DeleteButtonClicked();
    void onRobot2ConnectButtonClicked();
    void onRobot2CancelButtonClicked();
    
private:
    void setTableWidget();  // 设置tableWidget
    void setRobot2TableWidget();
    void updateTableSelection();    // 更新table选项

    static QString resolveDatabasePath(const QString &relPath);
    
    // 数据库函数
    void setupDatabase();
    void loadConnectionsFromDatabase();
    void saveConnectionToDatabase(const QString &protocol, const QString &host, const QString &port);
    void deleteConnectionFromDatabase(const QString &host, const QString &port);
    QString getDatabasePath();
    void loadRobot2ConnectionsFromDatabase();
    void saveRobot2ConnectionToDatabase(const QString &protocol, const QString &host, const QString &port);
    void deleteRobot2ConnectionFromDatabase(const QString &host, const QString &port);


private:
    Ui::ConnectDialog *ui;
    QSqlDatabase db;
    
};

#endif // CONNECTDIALOG_H