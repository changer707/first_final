#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <random>
#include <chrono>
#include <initializer_list>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <typeinfo>
#include <iterator>
#include "cmath"


#include "header.h"
#include "matrix.h"
#include "munkres.h"
#include "utils.h"
#include "dection.h"
#include "tracker.h"
#include "kalman_filter.h"
#include "darknet.h"
#include "improcess.h"
#include "dec_match.h"
#include "my_pthread.h"

#include "mulplayer.h"
#include <QApplication>
#include <QDebug>
//#include "ui_mulplayer.h"
#include "connection.h"
#include <QTextCodec>
#include <QFile>

#include <vector>
#include <QSqlQuery>

#define self 1//output class
#define judge 0//draw bbox

using namespace std;
//using namespace cv;
//output typeid(i).name();

int main(int argc, char *argv[])
{

    //std::string video_path="/home/zykzyh/视频/one.mp4";
    /*std::string name=video_path.substr(video_path.find_last_of("/")+1,video_path.find_last_of(".")-video_path.find_last_of("/")-1);
    std::string folder=video_path.substr(0,video_path.find_last_of("/"));
    std::cout<<folder;
    return 0;*/
    //cout<<dec_and_match(video_path,8);
    //return 0;
   //mainjudge("/home/ps/zk/darknet-master-1/sh-car-bicycleb.txt");
   //return 0;
   // std::vector<int> aa;
   // std::cout<<aa.size();
   //   return 0;
     /*int aaa=5;
       pthread_parallel((void*)&aaa);
       return 0;*/

        QApplication a(argc,argv);
        if (!createConnection()) return 1;     // 判断是否链接

        MulPlayer w;
//        Connection c;
//        c.createRelationalTables();


/*        createRelationalTables();
        QSqlQuery query;
        // 添加信息
        query.prepare("insert into traffic (filename, frame, type, participant1, speed1, participant2,speed2) values (:filename, :frame, :type, :participant1, :speed1, :participant2,:speed2)");
        // 传递变量
        QString filenameValue=QString::fromStdString("zzzzz");
        int frameValue=2;
        int typeValue=3;
        int participant1Value=4;
       int speed1Value=5;
        int participant2Value=6;
       int speed2Value=7;
        // 添加信息
        query.bindValue(":filename", filenameValue);
        query.bindValue(":frame", frameValue);
        query.bindValue(":type", typeValue);
        query.bindValue(":participant1", participant1Value);
        query.bindValue(":speed1", speed1Value);
        query.bindValue(":participant2", participant2Value);
        query.bindValue(":speed2", speed2Value);
        query.exec();*/

        w.show();
//        for(int i=0;i<2;i++)
//        w.show_data();

        //a.exec();
        return a.exec();
}
