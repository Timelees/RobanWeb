#include "task_manager/taskmanager.h"

TaskManager::TaskManager(QListWidget* taskListWidget, 
                         QPushButton* addTaskButton,
                         QPushButton* importTaskButton,
                         QPushButton* runTaskButton,
                         WebSocketWorker* webSocketWorker,
                         QObject* parent)
    : QObject(parent),
      m_taskListWidget(taskListWidget),
      m_addTaskButton(addTaskButton),
      m_importTaskButton(importTaskButton),
      m_runTaskButton(runTaskButton),
      m_webSocketWorker(webSocketWorker)
{
    // 设置任务配置目录
    m_taskConfigDir = QDir::current().filePath("config");
    QDir dir(m_taskConfigDir);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    // 绑定按钮点击事件
    connect(m_addTaskButton, &QPushButton::clicked, this, &TaskManager::onAddTaskClicked);
    connect(m_importTaskButton, &QPushButton::clicked, this, &TaskManager::onImportTaskClicked);
    connect(m_runTaskButton, &QPushButton::clicked, this, &TaskManager::onRunTaskClicked);
    
    // 绑定列表项双击事件
    connect(m_taskListWidget, &QListWidget::itemDoubleClicked, this, &TaskManager::onTaskItemDoubleClicked);
    
    // 绑定列表项右键菜单
    m_taskListWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_taskListWidget, &QListWidget::customContextMenuRequested, this, &TaskManager::showTaskContextMenu);
    
    // 加载任务列表
    loadTasks();
}

TaskManager::~TaskManager()
{
    saveTasks();
    
    // 清理任务对象
    for (Task* task : m_tasks) {
        delete task;
    }
    m_tasks.clear();
}

QString TaskManager::getTaskConfigPath() const
{
    return QDir(m_taskConfigDir).filePath("tasks.json");
}

void TaskManager::loadTasks()
{
    QString configPath = getTaskConfigPath();
    QFile file(configPath);
    
    if (!file.exists()) {
        // 配置文件不存在，创建默认任务
        Task* defaultTask2 = new Task(tr("跳舞"), "identify_numbers.py", this);
        Task* defaultTask3 = new Task(tr("搬运箱子"), "carry_box.py", this);
        
        m_tasks.append(defaultTask2);
        m_tasks.append(defaultTask3);
        
        saveTasks();
        updateTaskListUI();
        return;
    }
    
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开任务配置文件:" << configPath;
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

void TaskManager::saveTasks()
{
    QString configPath = getTaskConfigPath();
    QFile file(configPath);
    
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "无法保存任务配置文件:" << configPath;
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

void TaskManager::addTask(Task* task)
{
    m_tasks.append(task);
    updateTaskListUI();
    emit taskAdded(task);
    saveTasks();
}

void TaskManager::removeTask(int index)
{
    if (index >= 0 && index < m_tasks.size()) {
        Task* task = m_tasks.at(index);
        m_tasks.remove(index);
        delete task;
        updateTaskListUI();
        emit taskRemoved(index);
        saveTasks();
    }
}

void TaskManager::updateTaskListUI()
{
    m_taskListWidget->clear();
    
    for (const Task* task : m_tasks) {
        QListWidgetItem* item = new QListWidgetItem(task->name());
        item->setToolTip(task->scriptPath());  // 使用脚本路径作为工具提示
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
    
    // 获取脚本路径
    QString scriptPath = QInputDialog::getText(nullptr, tr("添加任务"), 
                                              tr("请输入脚本路径:"), 
                                              QLineEdit::Normal);
    if (scriptPath.isEmpty()) {
        return nullptr;
    }
    
    return new Task(taskName, scriptPath, this);
}

void TaskManager::onAddTaskClicked()
{
    Task* newTask = showAddTaskDialog();
    if (newTask) {
        addTask(newTask);
    }
}

void TaskManager::onImportTaskClicked()
{
    // 打开文件对话框选择Python脚本文件
    QString filePath = QFileDialog::getOpenFileName(nullptr, 
                                                   tr("导入任务脚本"), 
                                                   QDir::currentPath(), 
                                                   tr("Python脚本 (*.py);;所有文件 (*.*)"));
    if (filePath.isEmpty()) {
        return;
    }
    
    // 从文件名提取默认任务名
    QFileInfo fileInfo(filePath);
    QString defaultTaskName = fileInfo.baseName();
    
    // 获取任务名称
    QString taskName = QInputDialog::getText(nullptr, tr("导入任务"), 
                                            tr("请输入任务名称:"), 
                                            QLineEdit::Normal, 
                                            defaultTaskName);
    if (taskName.isEmpty()) {
        return;
    }
    
    // 创建并添加任务
    Task* newTask = new Task(taskName, filePath, this);
    addTask(newTask);
}

void TaskManager::onRunTaskClicked()
{
    int currentRow = m_taskListWidget->currentRow();
    if (currentRow >= 0 && currentRow < m_tasks.size()) {
        Task* selectedTask = m_tasks.at(currentRow);
        
        // 发出任务执行信号
        emit taskExecuted(selectedTask->scriptPath());
        
        // 通过WebSocket发送执行脚本的命令
        if (m_webSocketWorker) {
            QJsonObject pub;
            pub["op"] = "publish";
            pub["topic"] = "/robot/exec_py";
            pub["type"] = "std_msgs/String";
            QJsonObject msg;
            msg["data"] = selectedTask->scriptPath();
            pub["msg"] = msg;
            QJsonDocument doc(pub);
            QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
            
            // 通过 worker 发送发布消息
            QMetaObject::invokeMethod(m_webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
            
            QMessageBox::information(nullptr, tr("执行任务"),
                                    tr("任务 '%1' 已发送执行命令").arg(selectedTask->name()));
        } else {
            QMessageBox::warning(nullptr, tr("错误"),
                               tr("WebSocket连接未建立，无法执行任务"));
        }
    } else {
        QMessageBox::warning(nullptr, tr("提示"),
                           tr("请先选择要执行的任务"));
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
                               tr("名称: %1\n脚本: %2")
                               .arg(selectedTask->name())
                               .arg(selectedTask->scriptPath()));
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
    QAction* editAction = contextMenu.addAction(tr("编辑"));
    QAction* deleteAction = contextMenu.addAction(tr("删除"));
    
    QAction* selectedAction = contextMenu.exec(m_taskListWidget->viewport()->mapToGlobal(pos));
    
    if (selectedAction == runAction) {
        // 执行任务
        if (row >= 0 && row < m_tasks.size()) {
            Task* selectedTask = m_tasks.at(row);
            
            // 发出任务执行信号
            emit taskExecuted(selectedTask->scriptPath());
            
            // 通过WebSocket发送执行脚本的命令
            if (m_webSocketWorker) {
                QJsonObject pub;
                pub["op"] = "publish";
                pub["topic"] = "/robot/exec_py";
                pub["type"] = "std_msgs/String";
                QJsonObject msg;
                msg["data"] = selectedTask->scriptPath();
                pub["msg"] = msg;
                QJsonDocument doc(pub);
                QString payload = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
                
                // 通过 worker 发送发布消息
                QMetaObject::invokeMethod(m_webSocketWorker, "sendText", Qt::QueuedConnection, Q_ARG(QString, payload));
            }
        }
    } else if (selectedAction == editAction) {
        // 编辑任务
        if (row >= 0 && row < m_tasks.size()) {
            Task* task = m_tasks.at(row);
            
            // 编辑名称
            QString newName = QInputDialog::getText(nullptr, tr("编辑任务"), 
                                                  tr("任务名称:"), 
                                                  QLineEdit::Normal, 
                                                  task->name());
            if (!newName.isEmpty()) {
                task->setName(newName);
            }
            
            // 编辑脚本路径
            QString newPath = QInputDialog::getText(nullptr, tr("编辑任务"), 
                                                  tr("脚本路径:"), 
                                                  QLineEdit::Normal, 
                                                  task->scriptPath());
            if (!newPath.isEmpty()) {
                task->setScriptPath(newPath);
            }
            

            
            updateTaskListUI();
            saveTasks();
        }
    } else if (selectedAction == deleteAction) {
        // 删除任务
        if (row >= 0 && row < m_tasks.size()) {
            QMessageBox::StandardButton reply;
            reply = QMessageBox::question(nullptr, tr("确认删除"),
                                        tr("确定要删除任务 '%1' 吗?").arg(item->text()),
                                        QMessageBox::Yes | QMessageBox::No);
            
            if (reply == QMessageBox::Yes) {
                removeTask(row);
            }
        }
    }
}