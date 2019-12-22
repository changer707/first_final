#include "adjpara.h"
#include "ui_adjpara.h"
#include "kalman_filter.h"
#include "tracker.h"
#include <QDebug>


//数据关联所需变量
float iou_threshold;
//卡尔曼滤波所需变量
float noise_rx=10,noise_ry=10,noise_rw=50,noise_rh=50,noise_ax =100,noise_ay=100,noise_aw=100,noise_ah=100;
//事故判定所需变量
/*float kmove;
int like_check_thre;
int noacci_count_thre;
int neigh_check_thre;
float vfilter_thre;
float vfilter_lowthre;
float vy_filter_thre;
float vyf_gain;
float vsum_staythre;
float slide_staythre;
int stay_checkthre;*/

adjpara::adjpara(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::adjpara)
{
    ui->setupUi(this);
}

adjpara::~adjpara()
{
    delete ui;
}
//确定按钮
void adjpara::on_btn_ensure_clicked()
{

    //获取数据关联所需变量
    iou_threshold=this->ui->lineEdit_datass->text().toFloat();
    iou_threshold/=100;
    //获取卡尔曼滤波所需变量
    noise_rx=this->ui->lineEdit_kalman1->text().toFloat();
    noise_ry=this->ui->lineEdit_kalman2->text().toFloat();
    noise_rw=this->ui->lineEdit_kalman3->text().toFloat();
    noise_rh=this->ui->lineEdit_kalman4->text().toFloat();
    noise_ax=this->ui->lineEdit_kalman5->text().toFloat();
    noise_ay=this->ui->lineEdit_kalman6->text().toFloat();
    noise_aw=this->ui->lineEdit_kalman7->text().toFloat();
    noise_ah=this->ui->lineEdit_kalman8->text().toFloat();
    //获取事故判定所需变量
  /*  kmove=this->ui->lineEdit_judge1->text().toFloat();
    like_check_thre=this->ui->lineEdit_judge2->text().toInt();
    noacci_count_thre=this->ui->lineEdit_judge3->text().toInt();
    neigh_check_thre=this->ui->lineEdit_judge4->text().toInt();
    vfilter_thre=this->ui->lineEdit_judge5->text().toFloat();
    vfilter_lowthre=this->ui->lineEdit_judge6->text().toFloat();
    vy_filter_thre=this->ui->lineEdit_judge7->text().toFloat();
    vyf_gain=this->ui->lineEdit_judge8->text().toFloat();
    vsum_staythre=this->ui->lineEdit_judge9->text().toFloat();
    slide_staythre=this->ui->lineEdit_judge10->text().toFloat();
    stay_checkthre=this->ui->lineEdit_judge11->text().toInt();*/
    this->close();

}
//取消按钮
void adjpara::on_btn_cancel_clicked()
{
    this->close();
}










