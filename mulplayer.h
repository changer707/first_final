#ifndef MULPLAYER_H
#define MULPLAYER_H
//#pragma once

#include <QMainWindow>
#include <QVideoWidget>
#include <QLabel>
#include <QMediaPlayer>
#include <QFileDialog>
#include <QMediaPlaylist>
#include <QString>
#include <opencv2/opencv.hpp>
#include <QStandardItem>
#include <QStandardItemModel>

#include "paint.h"
#include "adjpara.h"


class QSqlTableModel;              // 添加类的前置声明
namespace Ui {
class MulPlayer;
}

class MulPlayer : public QMainWindow
{
    Q_OBJECT

public:
    explicit MulPlayer(QWidget *parent = nullptr);
    ~MulPlayer();
    void addToPlaylist(const QStringList& fileNames);
    //void display();
    std::string video_path;// send video path massage
//    void readFromfile(std::vector<std::string>);          // 声明函数：读取文件
//    void readFromfile();
    //void addinfo();               // 声明函数：添加信息
    void setmode();
    //int set_num_of_color;
    void printmsg(std::string a);
//    void show_data();
    void video_opencv(cv::Mat);
    int channel_num;//多通道__编号
    QPixmap fitPixmap;
//    bool on_btn_stop_pressed();


private slots:
    //void on_btn_readcam_clicked();

    void on_btn_submit_clicked();

    void on_btn_revert_clicked();

    void on_btn_add_clicked();

    void on_btn_deletedelete_clicked();

    void on_pushButton_clicked();

    //void on_btn_value_exe_clicked();
    //void resizeEvent(QResizeEvent *event);

    void on_action_2_triggered();

    void on_action_3_triggered();

    //void on_comboBox_ch_activated(int index);

    //void on_btn_open_original_2_clicked();

    void on_pushButton_playvideo_clicked();
    void on_btn_readfile_clicked();

public slots:
    void showdata();
private:
    Ui::MulPlayer *ui;
    QMediaPlayer *player;
    QMediaPlayer *player_2;
    QMediaPlayer *player_3;
    QMediaPlayer *player_4;
    /*QVideoWidget *videoWidget_1;
    QVideoWidget *videoWidget_2;
    QVideoWidget *videoWidget_3;
    QVideoWidget *videoWidget_4;*/
    //QLabel *label_1;
    QMediaPlaylist *Playlist;
    QString m_fileName;
    QStringList fileNames;
    QMediaPlayer::State m_playerState;

    QSqlTableModel *model;         // 定义私有对象
 //   QList<QString> info_lines;     // 定义容器
    paint a;
    adjpara p;                     // 调参界面



};

#endif // MULPLAYER_H
