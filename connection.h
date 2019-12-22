#ifndef CONNECTION_H
#define CONNECTION_H

#include <QMessageBox>
#include <QSqlDatabase>

// 创建链接
static bool createConnection()
{

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("my.db");
    if (!db.open()) {
        QMessageBox::critical(0, "Cannot open database1",
            "Unable to establish a database connection.", QMessageBox::Cancel);
        return false;
    }
    return true;
}

class Connection
{
public:
    void createRelationalTables(std::vector<std::string>);  // 声明函数：创建表
//    void createRelationalTables();
};
//定义创建表函数
//void createRelationalTables();

#endif // CONNECTION_H



