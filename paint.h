#ifndef PAINT_H
#define PAINT_H

#include <QWidget>

namespace Ui {
class paint;
}

class paint : public QWidget
{
    Q_OBJECT

public:
    explicit paint(QWidget *parent = nullptr);
    ~paint();

    void txtsplit(int);//分割函数
    void paint_v(std::vector<int>,std::vector<double>,int,int);//作图函数
    int num_line(QString);
    void select_id();
    void channel();

private slots:
    void on_btn_query_ID_clicked();

    void on_btn_clear_clicked();

private:
    Ui::paint *ui;
    QList<QString> txt_line;
    int ID;
    int num;//文件行数
    //int num_channel;//通道编号
    QString txt_name;//要打开的txt文件名字
};

#endif // PAINT_H
