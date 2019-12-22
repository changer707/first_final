#ifndef POPVIDEO_H
#define POPVIDEO_H

#include <QDialog>

namespace Ui {
class Popvideo;
}

class Popvideo : public QDialog
{
    Q_OBJECT

public:
    explicit Popvideo(QWidget *parent = nullptr);
    ~Popvideo();
    void Getduration(qint64);
    void resizeEvent(QResizeEvent *event);

private slots:
    void on_ProgressSlider_valueChanged(int value);

    void on_pushButton_Play_clicked();

    void on_pushButton_Pause_clicked();

    void on_pushButton_Stop_clicked();

private:
    Ui::Popvideo *ui;
};

#endif // POPVIDEO_H
