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

private slots:
    void onAddButtonClicked();
    void onDeleteButtonClicked();
    void onConnectButtonClicked();
    void onCancelButtonClicked();
    
private:
    void setTableWidget();  // 设置tableWidget
    void updateTableSelection();    // 更新table选项
    // 数据库函数
    void setupDatabase();
    void loadConnectionsFromDatabase();
    void saveConnectionToDatabase(const QString &protocol, const QString &host, const QString &port);
    void deleteConnectionFromDatabase(int row);
    QString getDatabasePath();


private:
    Ui::ConnectDialog *ui;
    QSqlDatabase db;
    
};

#endif // CONNECTDIALOG_H