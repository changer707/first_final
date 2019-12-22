#include "mulplayer.h"
#include "ui_mulplayer.h"
#include <QBoxLayout>
#include <QDebug>
#include <QStringList>
#include <QVector>
#include <QList>
#include <QSlider>
#include <QStringList>
#include <QStandardItemModel>
#include <QListWidget>
#include <QVideoWidget>
#include <QMessageBox>
#include <QProcess>
#include "dec_match.h"
#include "connection.h"
#include "header.h"
#include "popvideo.h"
#include <QDebug>
#include <QFile>
#include <QSqlQueryModel>
#include <QSqlTableModel>
#include <QSqlRelationalTableModel>
#include <QSqlError>
#include <QSqlQuery>
#include <QFileDialog>
#include <QSqlDriver>
#include <QSqlRecord>
#include <QSqlField>
#include <QTextStream>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <iostream>
#include <QThread>

#include <opencv2/opencv.hpp>
#include <QTimer>
#include <QLabel>

//extern  std::string output_path;
    // 定义全局变量
    QString a,b,c,d,e,f,g,h,i;

std::string accident_path;
extern QMediaPlayer *player10;
extern QVideoWidget *videoWidget10;
MulPlayer::MulPlayer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MulPlayer)
{
    ui->setupUi(this);

    setmode();

    player = new QMediaPlayer;      //创建播放器
    player_2 = new QMediaPlayer;
    player_3 = new QMediaPlayer;
    player_4 = new QMediaPlayer;

    Playlist = new QMediaPlaylist();        //创建播放列表，提供给QMediaPlayer::player
    QMediaPlaylist *Playlist_2 = new QMediaPlaylist();
    player->setPlaylist(Playlist);
    player_2->setPlaylist(Playlist_2);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(showdata()));
    timer->start(10000);
}

MulPlayer::~MulPlayer()
{
    delete ui;
}

//——————————————————————————————分割线————————————————————————————————
//———————————————————————————以下都是函数定义———————————————————————————


/*int  MulPlayer::on_btn_open_original_clicked()
{
    QString fileNames;
    //设置文件打开窗口，dialog类，标题“打开文件”，输入框为空，文件类型设置（非真实）
    fileNames = QFileDialog::getOpenFileName(this,tr("选择原始视频文件"),"",tr("Files (*.mp4 *.avi *.mkv)"));
    video_path = fileNames.toStdString();
    std::cout<<video_path;
    auto str_view = this;
    accident_path= dec_and_match(video_path,str_view,8);  //通过此函数调用检测及之后的程序,关联其他程序的重要语句!    !!!!!!!!!!!!!!    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    qDebug()<<"diyigetongdaojieshu"<<endl;
    std::cout<<accident_path;
    return 0;
}*/

void MulPlayer::on_btn_readfile_clicked()
{
    QString fileNames;
    //设置文件打开窗口，dialog类，标题“打开文件”，输入框为空，文件类型设置（非真实）
    fileNames = QFileDialog::getOpenFileName(this,tr("选择原始视频文件"),"",tr("Files (*.mp4 *.avi *.mkv)"));
    video_path = fileNames.toStdString();
    std::cout<<video_path;
    auto str_view = this;

    accident_path= dec_and_match(video_path,str_view,8);  //通过此函数调用检测及之后的程序,关联其他程序的重要语句!    !!!!!!!!!!!!!!    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //qDebug()<<"diyigetongdaojieshu"<<endl;
    std::cout<<accident_path;
}


/*
bool MulPlayer::on_btn_stop_pressed()
{
    //ui->btn_closecam->hide();
    bool a;
    if (ui->btn_stop->isHidden()){
        a = 0;
    }
    else {
        a = 1;
    }
    return a;
}
*/

void MulPlayer::setmode(){
    model = new QSqlTableModel(this);
    model->setTable("traffic");
    model->select();
    // 设置编辑策略
    model->setEditStrategy(QSqlTableModel::OnManualSubmit);
    ui->tableView->setModel(model);        // 将表显示到tableview中
    // 设置表头
    model->setHeaderData(0, Qt::Horizontal, tr("时间"));
    model->setHeaderData(1, Qt::Horizontal, tr("摄像头编号"));
    model->setHeaderData(2, Qt::Horizontal, tr("文件名"));
    model->setHeaderData(3, Qt::Horizontal, tr("帧数"));
    model->setHeaderData(4, Qt::Horizontal, tr("事故类型"));
    model->setHeaderData(5, Qt::Horizontal, tr("参与方1"));
    model->setHeaderData(6, Qt::Horizontal, tr("速度(像素/秒)"));
    model->setHeaderData(7, Qt::Horizontal, tr("参与方2"));
    model->setHeaderData(8, Qt::Horizontal, tr("速度(像素/秒)"));
    this->ui->tableView->setColumnWidth(0,200);
    this->ui->tableView->setColumnWidth(1,100);
    this->ui->tableView->setColumnWidth(2,100);
    this->ui->tableView->setColumnWidth(3,100);
    this->ui->tableView->setColumnWidth(4,100);
    this->ui->tableView->setColumnWidth(5,100);
    this->ui->tableView->setColumnWidth(6,100);
    this->ui->tableView->setColumnWidth(7,100);
    this->ui->tableView->setColumnWidth(8,100);
}

// 读取变量
//...........................................................................................................................................................................
//添加信息函数
void addinfo()
{
    QSqlQuery query;
    // 添加信息
    query.prepare("insert into traffic (currentime, cameranum, filename, frame, type, participant1, speed1, participant2,speed2) values (:currentime, :cameranum,:filename, :frame, :type, :participant1, :speed1, :participant2,:speed2)");
    // 传递变量
    QString currentimeValue=a;
    QString filenameValue=b;
    QString cameranumValue=c;
    QString frameValue=d;
    QString typeValue=e;
    QString participant1Value=f;
    QString speed1Value=g;
    QString participant2Value=h;
    QString speed2Value=i;
    // 添加信息
    query.bindValue(":currentime", currentimeValue);
    query.bindValue(":cameranum", cameranumValue);
    query.bindValue(":filename", filenameValue);
    query.bindValue(":frame", frameValue);
    query.bindValue(":type", typeValue);
    query.bindValue(":participant1", participant1Value);
    query.bindValue(":speed1", speed1Value);
    query.bindValue(":participant2", participant2Value);
    query.bindValue(":speed2", speed2Value);

    query.exec();

}
// 读取文件函数
void readFromfile(std::vector<std::string> accident){

    int j=0;

    for(auto it=std::begin(accident);it!=std::end(accident);++it){         // 从容器中读取
        QString line=QString::fromStdString(*it);

       // line = line.trimmed();                  // 去掉每行开头和结尾的空格
        QStringList subs = line.split(",");    // 分割
        a=subs.at(0);                          // 赋值给变量
        b=subs.at(1);
        c=subs.at(2);
        d=subs.at(3);
        e=subs.at(4);
        f=subs.at(5);
        g=subs.at(6);
        h=subs.at(7);
        i=subs.at(8);
        addinfo();            // 添加信息函数嵌套在其中

   }
       return ;
}
/*
// 创建表函数
void createRelationalTables()
//void Connection::createRelationalTables()

{
    QSqlQuery query;
    //创建表
    query.exec(QString("create table traffic (currentime varchar,""cameranum varchar,""filename varchar, ""frame varchar, ""type varchar, ""participant1 varchar, ""speed1 varchar,""participant2 varchar,""speed2 varchar)"));
    //readFromfile(judge_txt);                // 读取文件函数嵌套在其中

}*/
//...................................................................................................................................................................................

//读取txt文件
//添加信息函数
/*
void addinfo()
{
    QSqlQuery query;
    // 添加信息
    query.prepare("insert into traffic (currentime, cameranum, filename, frame, type, participant1, speed1, participant2,speed2) values (:currentime,:cameranum,:filename, :frame, :type, :participant1, :speed1, :participant2,:speed2)");
    // 传递变量
    QString currentimeValue=a;
    QString cameranumValue=b;
    QString filenameValue=c;
    QString frameValue=d;
    QString typeValue=e;
    QString participant1Value=f;
    QString speed1Value=g;
    QString participant2Value=h;
    QString speed2Value=i;
    // 添加信息
    query.bindValue(":currentime", currentimeValue);
    query.bindValue(":cameranum", cameranumValue);
    query.bindValue(":filename", filenameValue);
    query.bindValue(":frame", frameValue);
    query.bindValue(":type", typeValue);
    query.bindValue(":participant1", participant1Value);
    query.bindValue(":speed1", speed1Value);
    query.bindValue(":participant2", participant2Value);
    query.bindValue(":speed2", speed2Value);

    query.exec();

}*/
// 读取文件函数
/*
void readFromfile(){
    QList<QString> info_lines;     // 定义容器
    QFile file("/home/ps/zhouning/info1.txt");        // 读取文件路径
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){

       }
        QTextStream in(&file);
        while (!in.atEnd()) {
            QString line = in.readLine();           // 按行读取
            info_lines.append(line);                // 添加到容器中
         }

        file.close();
//        return;
        int j=0;
            for(j=0;j<info_lines.length();j++){         // 从容器中读取
                QString line=info_lines.at(j);
                line = line.trimmed();                  // 去掉每行开头和结尾的空格
                QStringList subs = line.split("  ");    // 分割
                a=subs.at(0);                          // 赋值给变量
                b=subs.at(1);
                c=subs.at(2);
                d=subs.at(3);
                e=subs.at(4);
                f=subs.at(5);
                g=subs.at(6);
                h=subs.at(7);
                i=subs.at(8);
                addinfo();                             // 添加信息函数嵌套在其中
           }

              return ;*/
/*    int j=0;

    for(auto it=std::begin(accident);it!=std::end(accident);++it){         // 从容器中读取
        QString line=QString::fromStdString(*it);

       // line = line.trimmed();                  // 去掉每行开头和结尾的空格
        QStringList subs = line.split(",");    // 分割
        a=subs.at(0);                          // 赋值给变量
        b=subs.at(1);
        c=subs.at(2);
        d=subs.at(3);
        e=subs.at(4);
        f=subs.at(5);
        g=subs.at(6);
        h=subs.at(7);
        i=subs.at(8);
        addinfo();            // 添加信息函数嵌套在其中

   }
       return ;
}*/

// 创建表函数
void createRelationalTables(std::vector<std::string> judge_txt)
//void Connection::createRelationalTables()

{
    QSqlQuery query;
    //创建表
    query.exec(QString("create table traffic (currentime varchar,""cameranum varchar,""filename varchar, ""frame varchar, ""type varchar, ""participant1 varchar, ""speed1 varchar,""participant2 varchar,""speed2 varchar)"));
//    query.exec(QString("create table traffic (filename varchar, ""frame int, ""type int, ""participant1 int, ""speed1 int,""participant2 int,""speed2 int)"));
    readFromfile(judge_txt);                // 读取文件函数嵌套在其中
 //   readFromfile();                // 读取文件函数嵌套在其中

}
// 提交修改按钮
void MulPlayer::on_btn_submit_clicked()
{
    int ok = QMessageBox::warning(this,tr("提交修改!"),
                            tr("你确定提交修改吗？"),QMessageBox::Yes, QMessageBox::No);
        if(ok == QMessageBox::No)
        { // 如果不确定，则撤销
            model->revertAll();
        } else { // 否则提交
            model->submitAll();
        }
}
// 撤销修改按钮
// 注：已经点击“提交修改”按钮后，再点击“撤销修改”按钮无法撤销
void MulPlayer::on_btn_revert_clicked()
{
   model->revertAll();        // 恢复
}
// 添加记录按钮
void MulPlayer::on_btn_add_clicked()
{
    // 获得表的行数
        int rowNum = model->rowCount();
        // 添加一行
        model->insertRow(rowNum);
        // 可以直接提交
        //model->submitAll();
}
// 删除记录按钮(并删除对应视频)
void MulPlayer::on_btn_deletedelete_clicked()
{
    // 获取选中的行
        int curRow = ui->tableView->currentIndex().row();
        int ok = QMessageBox::warning(this,tr("删除当前行!"),
                            tr("你确定删除当前行吗？"),QMessageBox::Yes, QMessageBox::No);
        if(ok == QMessageBox::No)
        { // 如果不删除，则撤销
            model->revertAll();
        } else { // 否则提交，在数据库中删除该行
            // 视频名为选中行第一列的内容
            QString videoname = model->data(model->index(curRow, 2)).toString();
     //       QString videopath="/home/zhx/Qtcode/ver03/";       // 待删除视频的路径
    //        QString extension=".mp4";                           // 扩展名（.mp4）
     //       videopath.append(videoname);         // videopath字符串后添加videoname字符串
    //        videopath.append(extension);         // videopath字符串后添加extension字符串
            QFile::remove(videoname);            // 删除视频文件
            model->removeRow(curRow);            // 删除该行记录
            model->submitAll();
        }
}

//显示全表按钮
void MulPlayer::on_pushButton_clicked()
{
        createRelationalTables(mainjudge(accident_path));// 执行创建表函数
        model->setTable("traffic");
        model->select();
        ui->tableView->setModel(model);        // 将表显示到tableview中
        model->setHeaderData(0, Qt::Horizontal, tr("时间"));
        model->setHeaderData(1, Qt::Horizontal, tr("摄像头编号"));
        model->setHeaderData(2, Qt::Horizontal, tr("文件名"));
        model->setHeaderData(3, Qt::Horizontal, tr("帧数"));
        model->setHeaderData(4, Qt::Horizontal, tr("事故类型"));
        model->setHeaderData(5, Qt::Horizontal, tr("参与方1"));
        model->setHeaderData(6, Qt::Horizontal, tr("速度(像素/秒)"));
        model->setHeaderData(7, Qt::Horizontal, tr("参与方2"));
        model->setHeaderData(8, Qt::Horizontal, tr("速度(像素/秒)"));
        this->ui->tableView->setColumnWidth(0,200);
        this->ui->tableView->setColumnWidth(1,100);
        this->ui->tableView->setColumnWidth(2,100);
        this->ui->tableView->setColumnWidth(3,100);
        this->ui->tableView->setColumnWidth(4,100);
        this->ui->tableView->setColumnWidth(5,100);
        this->ui->tableView->setColumnWidth(6,100);
        this->ui->tableView->setColumnWidth(7,100);
        this->ui->tableView->setColumnWidth(8,100);
//        qDebug()<<"123";
}
//刷新表函数
void MulPlayer::showdata()
{
//        createRelationalTables(mainjudge(accident_path));// 执行创建表函数
        model->setTable("traffic");
        model->select();
//        ui->tableView->setModel(model);        // 将表显示到tableview中
        model->setHeaderData(0, Qt::Horizontal, tr("时间"));
        model->setHeaderData(1, Qt::Horizontal, tr("摄像头编号"));
        model->setHeaderData(2, Qt::Horizontal, tr("文件名"));
        model->setHeaderData(3, Qt::Horizontal, tr("帧数"));
        model->setHeaderData(4, Qt::Horizontal, tr("事故类型"));
        model->setHeaderData(5, Qt::Horizontal, tr("参与方1"));
        model->setHeaderData(6, Qt::Horizontal, tr("速度(像素/秒)"));
        model->setHeaderData(7, Qt::Horizontal, tr("参与方2"));
        model->setHeaderData(8, Qt::Horizontal, tr("速度(像素/秒)"));
        this->ui->tableView->setColumnWidth(0,200);
        this->ui->tableView->setColumnWidth(1,100);
        this->ui->tableView->setColumnWidth(2,100);
        this->ui->tableView->setColumnWidth(3,100);
        this->ui->tableView->setColumnWidth(4,100);
        this->ui->tableView->setColumnWidth(5,100);
        this->ui->tableView->setColumnWidth(6,100);
        this->ui->tableView->setColumnWidth(7,100);
        this->ui->tableView->setColumnWidth(8,100);
        qDebug()<<"123";
}



/*void MulPlayer::on_btn_value_exe_clicked()
{
    set_num_of_color = ui->lineEdit_num->text().toInt();
    //qDebug()<<"set_num_of_color"<<set_num_of_color<<endl;
}*/

void MulPlayer::printmsg(std::string str){
    QString a = QString::fromStdString( str);
    this->ui->textEdit->append(a);
    //this->ui->textEdit->append("读取视频成功. . .");
}


void MulPlayer::on_action_2_triggered()
{
    a.show();
}

void MulPlayer::on_action_3_triggered()
{
    p.show();
}


int w2 = 0;
int h2 = 0;
void MulPlayer::video_opencv(cv::Mat frame){

    cv::Mat result_frame;
    cvtColor(frame, result_frame, CV_BGR2RGB);     //  OpenCV中Mat读入的图像是BGR格式，要转换为RGB格式
    cv::resize(result_frame,result_frame,cv::Size(640,480));// 将抓取到的帧，转换为QImage格式。QImage::Format_RGB888不同的摄像头用不同的格式。
    QImage image((const uchar*)result_frame.data, result_frame.cols, result_frame.rows, QImage::Format_RGB888);

    QPixmap pix;
    pix = pix.fromImage(image);
    int w,h;

    w = ui->label->frameGeometry().width();
    h = ui->label->frameGeometry().height();

    //通过IF:减缓摄像头晃动引起的画面抖动
    if( h-h2 < 30){
        h = h2;
    }
    if( w-w2 < 30){
        w = w2;
    }
    w2 = w;
    h2 = h;

    fitPixmap= pix.scaled(w,h,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->label->setPixmap(fitPixmap);//输出画面
}


void MulPlayer::on_pushButton_playvideo_clicked()
{
    int curRow = ui->tableView->currentIndex().row();
        QString openvideo = model->data(model->index(curRow, 2)).toString();
       // QString openvideo="/home/ps/zhouning/video1.mp4";
        qDebug()<<openvideo;
        qDebug()<<curRow;
//        if(openvideo!="")
//        {
        Popvideo *v=new Popvideo();
        v->show();
        player10->setMedia(QUrl::fromLocalFile(openvideo));

        player10->play();

//        }
}





