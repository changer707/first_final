#include <QChart>
#include <QChartView>
#include <QPen>
#include <QPainter>
#include <QSplineSeries>
#include <QLineSeries>
#include <QHBoxLayout>
#include <QValueAxis>
using namespace QtCharts;

#include "paint.h"
#include "ui_paint.h"

#include <QString>
#include <QTextStream>
#include <QFile>
#include <QDebug>
#include <qmath.h>
#include <iostream>
#include <vector>
#include <QMessageBox>
using namespace std;

paint::paint(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::paint)
{
    ui->setupUi(this);

    ui->comboBox_channel->setEnabled(false);
    channel();
    num = num_line(txt_name);
    //qDebug()<<num_line(txt_name);//246,right!
    txtsplit(num);
}

paint::~paint()
{
    delete ui;
}

void paint::channel(){
    QString num_channel;
    num_channel = QString::number(ui->comboBox_channel->currentIndex()+1);
    //QString alp = "v_"+num_channel+".txt";
    QString alp = "v_1.txt";
    txt_name = alp;
}

void paint::on_btn_query_ID_clicked()
{
    channel();
    //qDebug()<<txt_name;
    select_id();
    ui->btn_query_ID->setEnabled(false);

}

void paint::paint_v(std::vector<int> frames,std::vector<double> other,int j,int selector){
    QLineSeries *line = new QLineSeries();
    //paint line_v

    for (int i =0;i < j;i++) {
        line->append(frames[i],other[i]);
        //line->replace( (frames[i],other[i]);
    }

    QValueAxis *axisX = new QValueAxis();
    QValueAxis *axisY = new QValueAxis();
    //axisX->setRange(0,10);
    axisX->setLabelFormat("%d");
    axisX->setTitleText("时间（帧数）");
    axisX->setTickCount(20);
    axisX->setMinorTickCount(5);
    axisY->setMinorTickCount(5);
    axisX->setLinePenColor(Qt::darkBlue);
    if (selector ==1){
        axisY->setTitleText("速度");
    }
    else if (selector == 2) {
        axisY->setTitleText("速度角度");
    }
    else if (selector == 3) {
        axisY->setTitleText("加速度");
    }
    else if (selector == 4) {
        axisY->setTitleText("加速度角度");
    }
    QChart *chart = new QChart();
    chart->addAxis(axisX,Qt::AlignBottom);
    chart->addAxis(axisY,Qt::AlignLeft);
    chart->setAxisX(axisX,line);
    chart->setAxisY(axisY,line);
    chart->legend()->hide();
    chart->addSeries(line);
    line->attachAxis(axisX);
    line->attachAxis(axisY);
    //chart->createDefaultAxes();
    if (selector ==1){
        chart->setTitle("速度曲线");
        ui->widget_v->setChart(chart);
    }
    else if (selector == 2) {
        chart->setTitle("速度角度曲线");
        ui->widget_v_angle->setChart(chart);
    }
    else if (selector == 3) {
        chart->setTitle("加速度曲线");
        ui->widget_a->setChart(chart);
    }
    else if (selector == 4) {
        chart->setTitle("加速度角度曲线");
        ui->widget_a_angle->setChart(chart);
    }
}

//读取数据行数
int paint::num_line(QString filename){
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug()<<"paint.txt 平line文件打开失败";
    }
    QTextStream in (&file);
    while (! in.atEnd() ){
        QString line = in.readLine();
        txt_line.append(line);
    }
    file.close();
    int num =  txt_line.length();
    return num;
}

vector<double>vdata, adata;
vector<int>frames_index,name_id;
vector<double>v_angle_data, a_angle_data;

void paint::txtsplit(int num){

    QFile file(txt_name);//V是全部的车辆信息
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug()<<"paint.txt文件打开失败";
    }
    int i = 0;
    while (i < num) {
        QString line = file.readLine();
        line = line.trimmed();
        QString txt[num];//行,150应该替换为行数，怎么获取行数？
        txt[i] = line;//第i行赋值
        //qDebug()<<txt[i];
        //————————————————此处可以正常输出文本——————————————
        QString strvalue[num][6];//num = 246
        for (int j = 0;j<6;j++) {
            //qDebug()<<txt[i].split(",").at(j);
            strvalue[i][j] = txt[i].split(",").at(j);
            //qDebug()<<strvalue[i][j];
        }
        frames_index.push_back(strvalue[i][0].toInt());
        name_id.push_back(strvalue[i][1].toInt());
        vdata.push_back(strvalue[i][2].toDouble());
        adata.push_back(strvalue[i][3].toDouble());
        v_angle_data.push_back(strvalue[i][4].toDouble());
        a_angle_data.push_back(strvalue[i][5].toDouble());

        //qDebug()<<a_angle_data[i];
        //——————————————可以读出各项数值并和保存至相应数组——————————————
        i++;
    }//while done.
}

vector<double>vdata_2, adata_2;
vector<int>frames_index_2,name_id_2;
vector<double>v_angle_data_2, a_angle_data_2;

void paint::select_id(){
    ID = ui->lineEdit_ID->text().toInt();
    int i;
    int j = 0;
    for (i = 0;i < num;i++) {
        if(name_id[i] == ID){
            frames_index_2.push_back(frames_index[i]);
            vdata_2.push_back(vdata[i]);
            v_angle_data_2.push_back(v_angle_data[i]);
            adata_2.push_back(adata[i]);
            a_angle_data_2.push_back(a_angle_data[i]);
            j=j+1;

            /*for (j = 0;j < 20; j++) {
                qDebug()<<vdata_2[j];
            }*/

        }//if done
    }//for done

    paint_v(frames_index_2,vdata_2,j,1);
    paint_v(frames_index_2,v_angle_data_2,j,2);
    paint_v(frames_index_2,adata_2,j,3);
    paint_v(frames_index_2,a_angle_data_2,j,4);

}

void paint::on_btn_clear_clicked()
{

    vdata.erase(vdata.begin(),vdata.end());
    vdata_2.clear();
    v_angle_data.clear();
    v_angle_data_2.clear();
    adata.clear();
    adata_2.clear();
    a_angle_data.clear();
    a_angle_data_2.clear();
    frames_index.clear();
    frames_index_2.clear();
    name_id.clear();
    name_id_2.clear();

    /*int i;
    for (i=0;i<10;i++) {
        qDebug()<<vdata[i];
    }*/
    ui->btn_query_ID->setEnabled(true);

}
