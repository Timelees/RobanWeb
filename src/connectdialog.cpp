#include "connectdialog.h"
#include "ui_connectdialog.h"


ConnectDialog::ConnectDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectDialog)
{
    ui->setupUi(this);
    setWindowTitle("连接设置");
    setTableWidget();

    // 连接按钮信号槽
    connect(ui->connectButton, &QPushButton::clicked, this, &ConnectDialog::onConnectButtonClicked);
    connect(ui->cancelButton, &QPushButton::clicked, this, &ConnectDialog::onCancelButtonClicked);
    connect(ui->addPB, &QPushButton::clicked, this, &ConnectDialog::onAddButtonClicked);
    connect(ui->delPB, &QPushButton::clicked, this, &ConnectDialog::onDeleteButtonClicked);

    // 设置数据库
    setupDatabase();
    loadConnectionsFromDatabase();
}

ConnectDialog::~ConnectDialog()
{
    if (db.isOpen()) {
        db.close();
    }
    delete ui;
}
// 设置tableWidget
void ConnectDialog::setTableWidget(){
    // 设置 tableWidget 列数和标题
    ui->tableWidget->setColumnCount(3);
    QStringList headers;
    headers << "选择" << "主机IP" << "端口";
    ui->tableWidget->setHorizontalHeaderLabels(headers);

    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers); // 禁止编辑

    // 设置列宽比例 (1:3:2)
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed); // 复选框列固定宽度
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch); // 主机IP列自适应
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents); // 端口列按内容调整
    ui->tableWidget->setColumnWidth(0, 50); // 复选框列宽度设为50像素
}

// 添加数据库
void ConnectDialog::setupDatabase(){
    db = QSqlDatabase::addDatabase("QSQLITE");
    QString dbPath = getDatabasePath();
    qDebug() << "数据库路径:" << dbPath; // 调试输出路径
    db.setDatabaseName(dbPath);

    if (!db.open()) {
        qDebug() << "数据库打开失败:" << db.lastError().text();
        return;
    }

    QSqlQuery query;
    if (!query.exec("CREATE TABLE IF NOT EXISTS connections ("
                    "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                    "protocol TEXT, host TEXT, port TEXT)")) {
        qDebug() << "创建表失败:" << query.lastError().text();
    }
}

// 获取数据库路径
QString ConnectDialog::getDatabasePath(){
    // 获取项目根目录
    QString basePath = QDir::currentPath();
    QDir dir(basePath);
    if(!dir.cdUp()){    // 移动到上级目录
        qDebug() << "无法定位源代码目录，使用当前路径";
        basePath = QDir::currentPath();
    }
    else{
        basePath = dir.absolutePath();
    }
    // 构造 database 目录路径
    QString dbDirPath = basePath + "/database";
    QDir dbDir(dbDirPath);
    if (!dbDir.exists()) {
        if (!dbDir.mkpath(".")) {
            qDebug() << "无法创建 database 目录，使用当前路径作为备用";
            return QDir::currentPath() + "/connections.db";
        }
    }
    return dbDirPath + "/connections.db";
}

// 从数据库加载连接配置
void ConnectDialog::loadConnectionsFromDatabase()
{
    if (!db.isOpen()) {
        qDebug() << "数据库未打开，无法加载连接";
        return;
    }

    ui->tableWidget->setRowCount(0); // 清空表格
    QSqlQuery query("SELECT protocol, host, port FROM connections");
    while (query.next()) {
        int row = ui->tableWidget->rowCount();
        ui->tableWidget->insertRow(row);

        // 第一列: 复选框
        QTableWidgetItem *checkItem = new QTableWidgetItem();
        checkItem->setCheckState(Qt::Unchecked);
        checkItem->setFlags(checkItem->flags() | Qt::ItemIsUserCheckable);
        ui->tableWidget->setItem(row, 0, checkItem);

        // 第二列: 主机IP
        QTableWidgetItem *hostItem = new QTableWidgetItem(query.value("host").toString());
        ui->tableWidget->setItem(row, 1, hostItem);

        // 第三列: 端口
        QTableWidgetItem *portItem = new QTableWidgetItem(query.value("port").toString());
        ui->tableWidget->setItem(row, 2, portItem);
    }
}

void ConnectDialog::saveConnectionToDatabase(const QString &protocol, const QString &host, const QString &port)
{
    if (!db.isOpen()) {
        qDebug() << "数据库未打开，无法保存连接";
        return;
    }

    QSqlQuery query;
    query.prepare("INSERT INTO connections (protocol, host, port) VALUES (:protocol, :host, :port)");
    query.bindValue(":protocol", protocol);
    query.bindValue(":host", host);
    query.bindValue(":port", port);
    if (!query.exec()) {
        qDebug() << "保存连接失败:" << query.lastError().text();
    } 
    // else {
    //     loadConnectionsFromDatabase(); // 刷新表格
    // }
}
// 从数据库删除配置
void ConnectDialog::deleteConnectionFromDatabase(const QString &host, const QString &port)
{
    if (!db.isOpen()) {
        qDebug() << "数据库未打开，无法删除连接";
        return;
    }

    if (host.isEmpty() || port.isEmpty()) {
        qDebug() << "主机或端口为空，跳过数据库删除";
        return;
    }

    QSqlQuery query;
    query.prepare("DELETE FROM connections WHERE host = :host AND port = :port");
    query.bindValue(":host", host);
    query.bindValue(":port", port);
    if (!query.exec()) {
        qDebug() << "删除连接失败:" << query.lastError().text();
    }
}

// 添加按钮
void ConnectDialog::onAddButtonClicked()
{
    QString protocol = "ws://";
    QString host = ui->hostEdit->text().trimmed();
    QString port = ui->portEdit->text().trimmed();
    bool ok;
    int portNum = port.toInt(&ok);
    if (host.isEmpty() || !ok || portNum < 0 || portNum > 65535) {
        qDebug() << "无效的输入: 主机或端口无效";
        return;
    }

    // 添加到 tableWidget
    int rowCount = ui->tableWidget->rowCount();
    int curRow = ui->tableWidget->currentRow();
    int row = (curRow >= 0) ? curRow + 1 : rowCount;    // 未选中表格时，添加到末尾
    ui->tableWidget->insertRow(row);

    // 第一列: 复选框
    QTableWidgetItem *checkItem = new QTableWidgetItem();
    checkItem->setCheckState(Qt::Unchecked);
    checkItem->setFlags(checkItem->flags() | Qt::ItemIsUserCheckable); // 允许勾选
    ui->tableWidget->setItem(row, 0, checkItem);

    // 第二列: 主机IP
    QTableWidgetItem *hostItem = new QTableWidgetItem(host);
    ui->tableWidget->setItem(row, 1, hostItem);

    // 第三列: 端口
    QTableWidgetItem *portItem = new QTableWidgetItem(port);
    ui->tableWidget->setItem(row, 2, portItem);

    saveConnectionToDatabase(protocol, host, port);
    // ui->hostEdit->clear();
    // ui->portEdit->clear();

    qDebug() << "已添加:" << protocol << host << ":" << port;
}
// 删除
void ConnectDialog::onDeleteButtonClicked()
{
    // 删除选中的行
    QList<QTableWidgetItem*> selectedItems = ui->tableWidget->selectedItems();
   
    if (selectedItems.isEmpty()) {
        qDebug() << "未选择要删除的行";
        return;
    }

    // 获取所有选中的行号（可能有多选）
    QList<int> rowsToDelete;
    for (QTableWidgetItem *item : selectedItems) {
        int row = item->row();
        if (!rowsToDelete.contains(row)) {
            rowsToDelete.append(row);
        }
    }
    // 按行号降序排序，确保删除时不影响后续行号
    std::sort(rowsToDelete.begin(), rowsToDelete.end(), std::greater<int>());

    for (int row : rowsToDelete) {
        // 读取要删除行的 host 和 port，确保在 removeRow 之前访问
        QTableWidgetItem *hostItem = ui->tableWidget->item(row, 1);
        QTableWidgetItem *portItem = ui->tableWidget->item(row, 2);
        QString host = hostItem ? hostItem->text() : QString();
        QString port = portItem ? portItem->text() : QString();

        // 先删除数据库记录
        deleteConnectionFromDatabase(host, port);

        // 然后从表格中移除行
        ui->tableWidget->removeRow(row);
        qDebug() << "已删除选中的行: " << row << " (" << host << ":" << port << ")";
    }

}



// 连接按钮connectButton  槽函数
void ConnectDialog::onConnectButtonClicked()
{
  // 查找选中的行
    for (int row = 0; row < ui->tableWidget->rowCount(); ++row) {
        QTableWidgetItem *checkItem = ui->tableWidget->item(row, 0);
        if (checkItem && checkItem->checkState() == Qt::Checked) {
            QString host = ui->tableWidget->item(row, 1)->text();
            QString port = ui->tableWidget->item(row, 2)->text();
            QString protocol = "ws://";
            QString url = protocol + host + ":" + port;
            qDebug() << "请求连接到:" << url;
            emit connectRequested(url);
            accept();
            return;
        }
    }
    qDebug() << "未选择任何连接";
}
// 断开连接按钮cancelButton  槽函数
void ConnectDialog::onCancelButtonClicked()
{
    reject();
}
