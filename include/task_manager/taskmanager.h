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

#include "../socket_process/websocketworker.h"

// 任务类定义
class Task : public QObject {
    Q_OBJECT

public:
    Task(QObject* parent = nullptr) : QObject(parent) {}
    
    Task(const QString& name, const QString& scriptPath, 
         QObject* parent = nullptr) : QObject(parent),
         m_name(name), m_scriptPath(scriptPath) {}
    
    // Getters
    QString name() const { return m_name; }
    QString scriptPath() const { return m_scriptPath; }
    
    // Setters
    void setName(const QString& name) { m_name = name; }
    void setScriptPath(const QString& path) { m_scriptPath = path; }
    
    // 转换为JSON对象
    QJsonObject toJson() const {
        QJsonObject obj;
        obj["name"] = m_name;
        obj["scriptPath"] = m_scriptPath;
        return obj;
    }
    
    // 从JSON对象创建
    static Task* fromJson(const QJsonObject& obj, QObject* parent = nullptr) {
        return new Task(
            obj["name"].toString(),
            obj["scriptPath"].toString(),
            parent
        );
    }
    
private:
    QString m_name;
    QString m_scriptPath;
};

// 任务管理器类定义
class TaskManager : public QObject {
    Q_OBJECT

public:
    TaskManager(QListWidget* taskListWidget, 
                QPushButton* addTaskButton,
                QPushButton* importTaskButton,
                QPushButton* runTaskButton,
                WebSocketWorker* webSocketWorker,
                QObject* parent = nullptr);
    ~TaskManager();

    // 加载保存的任务列表
    void loadTasks();
    
    // 保存任务列表到配置文件
    void saveTasks();
    
    // 添加新任务
    void addTask(Task* task);
    
    // 删除任务
    void removeTask(int index);
    
    // 获取任务列表
    QVector<Task*> tasks() const { return m_tasks; }

public slots:
    // UI按钮点击处理
    void onAddTaskClicked();
    void onImportTaskClicked();
    void onRunTaskClicked();
    
    // 任务列表项双击处理
    void onTaskItemDoubleClicked(QListWidgetItem* item);
    
    // 任务列表项右键菜单处理
    void showTaskContextMenu(const QPoint& pos);
    
signals:
    void taskAdded(Task* task);
    void taskRemoved(int index);
    void taskSelected(Task* task);
    void taskExecuted(const QString& scriptPath);

private:
    // 更新UI显示
    void updateTaskListUI();
    
    // 创建自定义任务对话框
    Task* showAddTaskDialog();
    
    // 任务配置文件路径
    QString getTaskConfigPath() const;

private:
    QListWidget* m_taskListWidget;
    QPushButton* m_addTaskButton;
    QPushButton* m_importTaskButton;
    QPushButton* m_runTaskButton;
    WebSocketWorker* m_webSocketWorker;
    QVector<Task*> m_tasks;
    QString m_taskConfigDir;
};