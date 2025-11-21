#include "manager/taskmanager.h"

TaskManager::TaskManager(QListWidget* taskListWidget, 
                         QPushButton* addTaskButton,
                         QPushButton* runTaskButton,
                         QPushButton* stopTaskButton,
                         WebSocketWorker* webSocketWorker,
                         QObject* parent)
    : QObject(parent),
      m_taskListWidget(taskListWidget),
      m_addTaskButton(addTaskButton),
      m_runTaskButton(runTaskButton),
      m_stopTaskButton(stopTaskButton),
      m_webSocketWorker(webSocketWorker)
{
    // 绑定按钮点击事件
    connect(m_addTaskButton, &QPushButton::clicked, this, &TaskManager::onAddTaskClicked);
    connect(m_runTaskButton, &QPushButton::clicked, this, &TaskManager::onRunTaskClicked);
    connect(m_stopTaskButton, &QPushButton::clicked, this, &TaskManager::onStopTaskClicked);
    // 绑定列表项双击事件
    connect(m_taskListWidget, &QListWidget::itemDoubleClicked, this, &TaskManager::onTaskItemDoubleClicked);
    
    // 绑定列表项右键菜单
    m_taskListWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_taskListWidget, &QListWidget::customContextMenuRequested, this, &TaskManager::showTaskContextMenu);
    
    // 加载任务列表
    loadTasksFromConfig();
}

TaskManager::~TaskManager()
{
    saveTasksToConfig();   
    // 清理任务对象
    for (Task* task : m_tasks) {
        delete task;
    }
    m_tasks.clear();
}


void TaskManager::loadTasksFromConfig()
{
    QDir appdir(QCoreApplication::applicationDirPath());
    QString cand = QDir::cleanPath(appdir.filePath(QString("../config/tasks_config.json")));
    QFile file(cand);

    // 确保config目录存在
    QDir configDir = QFileInfo(file).absoluteDir();
    if (!configDir.exists()) {
        configDir.mkpath(".");
    }

    if (!file.exists()) {
        // 配置文件不存在，创建默认任务
        Task* defaultTask1 = new Task(tr("Say-yeah舞蹈"), "say_yeah.sh", 
                    "/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/Say-yeah舞蹈案例.py", this);
        Task* defaultTask2 = new Task(tr("货物搬运"), "Task_carry.sh", 
                    "/home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/game/2022/caai_roban_challenge/colleges/scripts/Task_carry_box.py", this);

        m_tasks.append(defaultTask1);
        m_tasks.append(defaultTask2);
        
        saveTasksToConfig();
        updateTaskListUI();
        return;
    }
    
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开任务配置文件:" << file.fileName() << "，错误：" << file.errorString();
        return;
    }
    
    QByteArray jsonData = file.readAll();
    file.close();
    
    QJsonDocument doc = QJsonDocument::fromJson(jsonData);
    if (doc.isNull() || !doc.isArray()) {
        qWarning() << "任务配置文件格式错误";
        return;
    }
    
    // 清除现有任务
    for (Task* task : m_tasks) {
        delete task;
    }
    m_tasks.clear();
    
    // 从JSON加载任务
    QJsonArray taskArray = doc.array();
    for (const QJsonValue& value : taskArray) {
        if (value.isObject()) {
            QJsonObject obj = value.toObject();
            Task* task = Task::fromJson(obj, this);
            m_tasks.append(task);
        }
    }
    
    updateTaskListUI();
}
// 保存Task列表到配置文件
void TaskManager::saveTasksToConfig()
{
    QDir appdir(QCoreApplication::applicationDirPath());
    QString cand = QDir::cleanPath(appdir.filePath(QString("../config/tasks_config.json")));
    QFile file(cand);

    // 确保config目录存在
    QDir configDir = QFileInfo(file).absoluteDir();
    if (!configDir.exists()) {
        if (!configDir.mkpath(".")) {
            qWarning() << "无法创建配置目录:" << configDir.absolutePath();
            return;
        }
    }

    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        qWarning() << "无法保存任务配置文件:" << file.fileName() << "，错误：" << file.errorString();
        return;
    }
    
    QJsonArray taskArray;
    for (const Task* task : m_tasks) {
        taskArray.append(task->toJson());
    }
    
    QJsonDocument doc(taskArray);
    file.write(doc.toJson(QJsonDocument::Indented));
    file.close();
}
// 添加任务
void TaskManager::addTask(Task* task)
{
    m_tasks.append(task);
    updateTaskListUI();
    emit taskAdded(task);
    saveTasksToConfig();
}
// 删除任务
void TaskManager::removeTask(int index)
{
    if (index >= 0 && index < m_tasks.size()) {
        Task* task = m_tasks.at(index);
        m_tasks.remove(index);
        delete task;
        updateTaskListUI();
        emit taskRemoved(index);
        saveTasksToConfig();
    }
}
// 更新任务列表
void TaskManager::updateTaskListUI()
{
    m_taskListWidget->clear();
    
    for (const Task* task : m_tasks) {
        QListWidgetItem* item = new QListWidgetItem(task->getTaskName());
        item->setToolTip(task->getScriptName());  // 使用脚本名称作为工具提示
        m_taskListWidget->addItem(item);
    }
}

Task* TaskManager::showAddTaskDialog()
{
    // 获取任务名称
    QString taskName = QInputDialog::getText(nullptr, tr("添加任务"), 
                                            tr("请输入任务名称:"), 
                                            QLineEdit::Normal);
    if (taskName.isEmpty()) {
        return nullptr;
    }

    // 获取脚本名称
    QString scriptName = QInputDialog::getText(nullptr, tr("添加任务"), 
                                              tr("请输入任务脚本名称:"), 
                                              QLineEdit::Normal);
    if (scriptName.isEmpty()) {
        return nullptr;
    }

    // 获取任务代码路径
    QString TaskCodePath = QInputDialog::getText(nullptr, tr("添加任务"), 
                                              tr("请输入任务代码路径:"), 
                                              QLineEdit::Normal);
    if (TaskCodePath.isEmpty()) {
        return nullptr;
    }
    
    return new Task(taskName, scriptName, TaskCodePath, this);
}

void TaskManager::onAddTaskClicked()
{
    Task* newTask = showAddTaskDialog();
    if (newTask) {
        addTask(newTask);
    }
}


void TaskManager::onRunTaskClicked()
{
    int currentRow = m_taskListWidget->currentRow();
    if (currentRow >= 0 && currentRow < m_tasks.size()) {
        Task* selectedTask = m_tasks.at(currentRow);
        
        // 通过WebSocket发送执行脚本的命令（先检查连接状态）
        bool wsConnected = false;
        if (m_webSocketWorker) {
            // 调用 worker 的 isConnected()，在 worker 所在线程中执行以获知真实状态
            QMetaObject::invokeMethod(m_webSocketWorker, "isConnected", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, wsConnected));
        }

        if (!m_webSocketWorker || !wsConnected) {
            QMessageBox::warning(nullptr, tr("错误"),
                                 tr("WebSocket 未连接，无法执行任务，请先建立连接。"));
            return;
        }

        
        // 发出任务执行信号（仅在已成功发送时）
        emit taskExecuted(selectedTask->getScriptPath());

    } else {
        QMessageBox::warning(nullptr, tr("提示"),
                           tr("请先选择要执行的任务"));
    }
}
void TaskManager::onStopTaskClicked()
{
    int currentRow = m_taskListWidget->currentRow();
    if (currentRow >= 0 && currentRow < m_tasks.size()) {
        Task* selectedTask = m_tasks.at(currentRow);
        
        // 通过WebSocket发送停止脚本的命令（先检查连接状态）
        bool wsConnected = false;
        if (m_webSocketWorker) {
            QMetaObject::invokeMethod(m_webSocketWorker, "isConnected", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, wsConnected));
        }
        if (!m_webSocketWorker || !wsConnected) {
            QMessageBox::warning(nullptr, tr("错误"),
                                 tr("WebSocket 未连接，无法停止任务，请先建立连接。"));
            return;
        }

        // 发出任务停止信号
        emit taskStopped(selectedTask->getScriptPath());

    } else {
        QMessageBox::warning(nullptr, tr("提示"),
                           tr("请先选择要停止的任务"));
    }
}

void TaskManager::onTaskItemDoubleClicked(QListWidgetItem* item)
{
    int row = m_taskListWidget->row(item);
    if (row >= 0 && row < m_tasks.size()) {
        Task* selectedTask = m_tasks.at(row);
        emit taskSelected(selectedTask);
        
        // 显示任务详情
        QMessageBox::information(nullptr, tr("任务详情"),
                               tr("名称: %1\n脚本名称: %2\n脚本路径: %3\n代码路径: %4")
                               .arg(selectedTask->getTaskName())
                               .arg(selectedTask->getScriptName())
                               .arg(selectedTask->getScriptPath())
                               .arg(selectedTask->getTaskCodePath()));
    }
}

void TaskManager::showTaskContextMenu(const QPoint& pos)
{
    QListWidgetItem* item = m_taskListWidget->itemAt(pos);
    if (!item) {
        return;
    }
    
    int row = m_taskListWidget->row(item);
    
    QMenu contextMenu;
    QAction* runAction = contextMenu.addAction(tr("执行"));
    QAction* stopAction = contextMenu.addAction(tr("停止"));
    QAction* editAction = contextMenu.addAction(tr("编辑"));
    QAction* deleteAction = contextMenu.addAction(tr("删除"));
    
    QAction* selectedAction = contextMenu.exec(m_taskListWidget->viewport()->mapToGlobal(pos));
    
    if (selectedAction == runAction) {
        // 执行任务
        if (row >= 0 && row < m_tasks.size()) {
            Task* selectedTask = m_tasks.at(row);
            
            // 通过WebSocket发送执行脚本的命令（先检查连接状态）
            bool wsConnected = false;
            if (m_webSocketWorker) {
                QMetaObject::invokeMethod(m_webSocketWorker, "isConnected", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, wsConnected));
            }
            if (!m_webSocketWorker || !wsConnected) {
                QMessageBox::warning(nullptr, tr("错误"),
                                     tr("WebSocket 未连接，无法执行任务，请先建立连接。"));
            } else {
                // 发出任务执行信号（仅在已连接时）
                emit taskExecuted(selectedTask->getScriptPath());
            }
        }
    }else if(selectedAction == stopAction) {
        // 停止任务
        if (row >= 0 && row < m_tasks.size()) {
            Task* selectedTask = m_tasks.at(row);
            
            // 通过WebSocket发送停止脚本的命令（先检查连接状态）
            bool wsConnected = false;
            if (m_webSocketWorker) {
                QMetaObject::invokeMethod(m_webSocketWorker, "isConnected", Qt::BlockingQueuedConnection, Q_RETURN_ARG(bool, wsConnected));
            }
            if (!m_webSocketWorker || !wsConnected) {
                QMessageBox::warning(nullptr, tr("错误"),
                                     tr("WebSocket 未连接，无法停止任务，请先建立连接。"));
            } else {
                // 发出任务停止信号
                emit taskStopped(selectedTask->getScriptPath());
            }
        }
    }
    
    else if (selectedAction == editAction) {
        // 编辑任务
        if (row >= 0 && row < m_tasks.size()) {
            Task* task = m_tasks.at(row);
            
            // 编辑名称
            QString newName = QInputDialog::getText(nullptr, tr("编辑任务"), 
                                                  tr("任务名称:"), 
                                                  QLineEdit::Normal, 
                                                  task->getTaskName());
            if (!newName.isEmpty()) {
                task->setTaskName(newName);
            }

            // 编辑脚本名称
            QString newScriptName = QInputDialog::getText(nullptr, tr("编辑任务"), 
                                                  tr("脚本名称:"), 
                                                  QLineEdit::Normal, 
                                                  task->getScriptName());
            if (!newScriptName.isEmpty()) {
                task->setScriptName(newScriptName);
            }

            // 编辑代码路径
            QString newPath = QInputDialog::getText(nullptr, tr("编辑任务"), 
                                                  tr("代码路径:"), 
                                                  QLineEdit::Normal, 
                                                  task->getTaskCodePath());
            if (!newPath.isEmpty()) {
                task->setTaskCodePath(newPath);
            }

            updateTaskListUI();
            saveTasksToConfig();
        }
    } else if (selectedAction == deleteAction) {
        // 删除任务
        if (row >= 0 && row < m_tasks.size()) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(nullptr, tr("确认删除"),
                                        tr("是否删除任务 '%1' 吗?").arg(item->text()),
                                        QMessageBox::Yes | QMessageBox::No);
            
            if (reply == QMessageBox::Yes) {
                removeTask(row);
            }
        }
    }
}