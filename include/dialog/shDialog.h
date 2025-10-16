#ifndef SHDIALOG_H
#define SHDIALOG_H

#include <QDialog>
#include <QDebug>
#include <QPushButton>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QFile>
#include <QRegularExpression>
#include <QDir>
#include <QThread>
#include <QTimer>


#include "ros_process/cameraImage.h"
#include "ros_process/slamMapPoint.h"
#include "ros_process/pointclouddisplay.h"

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
    void onRunSLAMButtonClicked();
    void onCloseSLAMButtonClicked();
    void onRunControlButtonClicked();
    void onCancelControlButtonClicked();
    // 控制按键
    void onControlButtonClicked();      
    

signals:
    void runScriptRequested(const QString &cmd);
    void closeScriptRequested();

private:
    void init();
    void bindSlots();
    bool eventFilter(QObject *watched, QEvent *event) override;


private:
    Ui::ShDialog *ui;
    WebSocketWorker *m_worker;
    QThread *featuredImageThread;       // 特征点图像处理线程
    CameraImageMonitor *featuredImageMonitor; // 特征点图像监视器
    QTimer *featuredImagePullTimer;           // 定时器，用于从特征点图像监视器中获取最新帧
    CameraImageMonitor *cameraImageMonitor = nullptr; // 相机图像监视器
    QString m_featureTopic;

    SlamMapMonitor *slamMapMonitor = nullptr; // SLAM地图点云监视器
    QThread *slamMapThread = nullptr;       // SLAM地图点云处理线程
    
    PointCloudDisplay *pcd = nullptr; // QOpenGL点云显示
};

#endif // SHDIALOG_H