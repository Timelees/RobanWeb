#ifndef SLAMDIALOG_H
#define SLAMDIALOG_H

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
#include <QPointer>


#include "ros_process/cameraImage.h"
#include "ros_process/slamMapPoint.h"
#include "ros_process/pointclouddisplay.h"

namespace Ui
{
    class slamDialog;
} // namespace Ui

class WebSocketWorker;

class slamDialog : public QDialog
{
    Q_OBJECT

public:
    explicit slamDialog(WebSocketWorker *webSocketWorker, QWidget *parent = nullptr);
    ~slamDialog();

public slots:
    // Request SLAM stop from outside (safe to call from other objects)
    void stopSLAM();

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void onRunSLAMButtonClicked();
    void onCloseSLAMButtonClicked();
    void onRunControlButtonClicked();
    void onCancelControlButtonClicked();
    // localization checkbox toggled
    void onLocalizationModeToggled(bool checked);
    // 控制按键
    void onControlButtonClicked();      
    

signals:
    void runScriptRequested(const QString &cmd);
    void closeScriptRequested();

private:
    void init();
    void bindSlots();
    bool eventFilter(QObject *watched, QEvent *event) override;
    // 更新 localization_checkBox 的图标（根据是否选中）
    void updateLocalizationIcon(bool checked);



private:
    Ui::slamDialog *ui;
    QPointer<WebSocketWorker> m_worker;
    QThread *featuredImageThread;                       // 特征点图像处理线程
    CameraImageMonitor *featuredImageMonitor;           // 特征点图像监视器
    QTimer *featuredImagePullTimer;                     // 定时器，用于从特征点图像监视器中获取最新帧
    CameraImageMonitor *cameraImageMonitor = nullptr;   // 相机图像监视器
    QString m_featureTopic;

    SlamMapMonitor *slamMapMonitor = nullptr;       // SLAM地图点云监视器
    QThread *slamMapThread = nullptr;               // SLAM地图点云处理线程
    
    PointCloudDisplay *pcd = nullptr;           // QOpenGL点云显示
    bool localizationAdvertised = false;        // 是否已发布定位模式话题
};

#endif // SLAMDIALOG_H