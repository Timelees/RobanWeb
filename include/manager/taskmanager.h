#pragma once

#include <QObject>
#include <QListWidget>
#include <QPushButton>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <QMap>
#include <QVector>
#include <QFile>
#include <QDir>
#include <QMessageBox>
#include <QInputDialog>
#include <QMenu>
#include <QAction>
#include <QCoreApplication>

#include "socket_process/websocketworker.h"

// 任务类定义
class Task : public QObject {
    Q_OBJECT

public:
    Task(QObject* parent = nullptr) : QObject(parent) {}

        Task(const QString& Taskname, const QString& scriptName, const QString& TaskCodePath, QObject* parent = nullptr)
            : QObject(parent), m_taskName(Taskname), m_TaskCodePath(TaskCodePath)
        {
            setScriptName(scriptName);
            m_scriptPath = QString("%1/%2").arg("/home/lemon/exec_scripts").arg(m_scriptName);
        }

    // 提取task信息
    QString getTaskName() const { return m_taskName; }
    QString getScriptName() const { return m_scriptName; }
    QString getScriptPath() const { return m_scriptPath; }
    QString getTaskCodePath() const { return m_TaskCodePath; }
    
    // 设置Task信息
    void setTaskName(const QString& name) { m_taskName = name; }
    void setScriptName(const QString& name) {
            if (name.isEmpty()) {
                m_scriptName = name;
                return;
            }
            QString n = name;
            // 如果脚本名不以 .sh 结尾，则追加 .sh（不区分大小写）
            if (!n.endsWith(".sh", Qt::CaseInsensitive)) {
                n += ".sh";
            }
            m_scriptName = n;
        }
    void setScriptPath(const QString& path) { m_scriptPath = path; }
    void setTaskCodePath(const QString& path) { m_TaskCodePath = path; }
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["name"] = m_taskName;
        obj["scriptName"] = m_scriptName;
        obj["scriptPath"] = m_scriptPath;
        obj["TaskCodePath"] = m_TaskCodePath;
        return obj;
    }
    
    // 从JSON对象创建
    static Task* fromJson(const QJsonObject& obj, QObject* parent = nullptr) {
        return new Task(
            obj["name"].toString(),
            obj["scriptName"].toString(),
            obj["TaskCodePath"].toString(),
            parent
        );
    }
    
private:
    QString m_taskName;
    QString m_scriptName;
    QString m_TaskCodePath;
    QString m_scriptPath;
};

// 任务管理器类定义
class TaskManager : public QObject {
    Q_OBJECT

public:
    TaskManager(QListWidget* taskListWidget, 
                QPushButton* addTaskButton,
                QPushButton* runTaskButton,
                QPushButton* stopTaskButton,
                WebSocketWorker* webSocketWorker,
                QObject* parent = nullptr);
    ~TaskManager();

    // 加载保存的任务列表
    void loadTasksFromConfig();
    
    // 保存任务列表到配置文件
    void saveTasksToConfig();
    
    // 添加新任务
    void addTask(Task* task);
    
    // 删除任务
    void removeTask(int index);
    
    // 获取任务列表
    QVector<Task*> tasks() const { return m_tasks; }

public slots:
    // UI按钮点击处理
    void onAddTaskClicked();
    void onRunTaskClicked();
    void onStopTaskClicked();
    // 任务列表项双击处理
    void onTaskItemDoubleClicked(QListWidgetItem* item);
    
    // 任务列表项右键菜单处理
    void showTaskContextMenu(const QPoint& pos);
    
signals:
    void taskAdded(Task* task);
    void taskRemoved(int index);
    void taskSelected(Task* task);
    void taskExecuted(const QString& TaskCodePath);
    void taskStopped(const QString& scriptPath);

private:
    // 更新UI显示
    void updateTaskListUI();
    
    // 创建自定义任务对话框
    Task* showAddTaskDialog();
    

private:
    QListWidget* m_taskListWidget;
    QPushButton* m_addTaskButton;
    QPushButton* m_runTaskButton;
    QPushButton* m_stopTaskButton;
    WebSocketWorker* m_webSocketWorker;
    QVector<Task*> m_tasks;
    QString m_taskConfigDir;
};