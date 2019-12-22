#include "popvideo.h"
#include "ui_popvideo.h"
#include "mulplayer.h"

//#include <QSlider>
#include <QResizeEvent>


QMediaPlayer *player10;
QVideoWidget *videoWidget10;
Popvideo::Popvideo(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Popvideo)
{
    ui->setupUi(this);
    player10 = new QMediaPlayer;
    videoWidget10 = new QVideoWidget(this);       //创建播放窗口
    player10->setVideoOutput(videoWidget10);        //将播放显示在videowidget窗口
//    videoWidget10->setGeometry(20,10,541,341);
    connect(player10,&QMediaPlayer::durationChanged,ui->ProgressSlider,&QSlider::setMaximum);
    connect(player10,&QMediaPlayer::positionChanged,ui->ProgressSlider,&QSlider::setValue);
    connect(ui->ProgressSlider,&QSlider::sliderMoved,player10,&QMediaPlayer::setPosition);
    connect(player10,SIGNAL(durationChanged(qint64)),this,SLOT(Getduration(qint64)));
}

Popvideo::~Popvideo()
{
    delete ui;
}

//视频总时间获取槽函数
void Popvideo::Getduration(qint64){
    long long playtime;
    playtime = player10->duration();
    qDebug()<<playtime;
    playtime /= 1000;  //获得的时间是以毫秒为单位的,变成秒
    //时间单位转化
    long long h,m,s;
    h = playtime/3600;
    m = (playtime-h*3600)/60;
    s = playtime-h*3600-m*60;
    //qDebug()<<h<<m<<s;
    if(h == 0){
        ui->lineEdit_Time->setText(QString("%1:%2").arg(m).arg(s));
    }
    else  ui->lineEdit_Time->setText(QString("%1:%2:%3").arg(h).arg(m).arg(s));
}

void Popvideo::on_ProgressSlider_valueChanged(int value)
{
    long long currenttime;
        currenttime = ui->ProgressSlider->value();
        qDebug()<<currenttime;
        long long h,m,s;
        currenttime /= 1000;
        h = currenttime /3600;
        m = (currenttime-h*3600)/60;
        s = currenttime-h*3600-m*60;
        if(h == 0){
            ui->lineEdit_Currenttime->setText(QString("%1:%2").arg(m).arg(s));
        }
        else  ui->lineEdit_Currenttime->setText(QString("%1:%2:%3").arg(h).arg(m).arg(s));
}

void Popvideo::on_pushButton_Play_clicked()
{
    player10->play();
}

void Popvideo::on_pushButton_Pause_clicked()
{
    player10->pause();
}

void Popvideo::on_pushButton_Stop_clicked()
{
    player10->stop();
}

void Popvideo::resizeEvent(QResizeEvent *event){
    int w;
    int h;
    w = this->width();
    w = w - 40;
    h = this->height();
    h = h - 100;
    qDebug()<<w<<h;
    videoWidget10->setGeometry(20,10,w,h);
//    videoWidget_2->setGeometry(w+15,50,w,h);
//    videoWidget_3->setGeometry(10,h+55,w,h);
//    videoWidget_4->setGeometry(w+15,h+55,w,h);
    //ui->label->setGeometry(10,20,600,300);
    //ui->label_video->setGeometry(w+15,3,w,h);
}
