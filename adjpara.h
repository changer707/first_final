#ifndef ADJPARA_H
#define ADJPARA_H

#include <QDialog>

namespace Ui {
class adjpara;
}

class adjpara : public QDialog
{
    Q_OBJECT

public:
    explicit adjpara(QWidget *parent = 0);
    ~adjpara();

private slots:
    void on_btn_ensure_clicked();

    void on_btn_cancel_clicked();

private:
    Ui::adjpara *ui;

};

#endif // ADJPARA_H
