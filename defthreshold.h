//#ifndef DEFTHRESHOLD_H
//#define DEFTHRESHOLD_H
// //一帧经过的时间
//#define delta_T 0.033333f
// //正无穷大
//#define infinity std::numeric_limits<double>::infinity()
// //对象的面积阈值
//constexpr float person_area=1;
//constexpr float bicycle_area=1.5;
//constexpr float tricycle_area=3;
//constexpr float car_area=4;
//constexpr float suv_area=4.5;
//constexpr float truck_area=6;
//constexpr float heavytruck_area=7;
//constexpr float coach_area=7;

// //视频的像素
//   //图像的大小
//constexpr float img_wideth=850;//手动更改
//constexpr float img_height=480;

//constexpr float edge_left=img_wideth/10;
//constexpr float edge_right=(img_wideth-img_wideth/10);
//constexpr float edge_top=img_height/5;
//constexpr float edge_below=(img_height-img_height/10);

// //范围r,可大一些,尽量使相撞物体包含在内
//constexpr float r_threshold=img_height/5;

// //碰撞距离阈值
//constexpr float crash_distance=img_height/20;

// //加速度阈值
//constexpr float asum_threshold=1.412;//没用,因为不同类型的加速度阈值不同

// //疑似碰撞对象的坐标移动阈值
//constexpr float r_move=img_height/15;

// //疑似对象的检查帧数
//constexpr int like_check_thre=11;
//#endif // DEFTHRESHOLD_H



#ifndef DEFTHRESHOLD_H
#define DEFTHRESHOLD_H
#include "cmath"
//
//#define AREA1 (1920*1080)
//#define W1      (1920)
//#define H1      (1080)
//一帧经过的时间
#define delta_T 0.033333f
//正无穷大
#define infinity_double std::numeric_limits<double>::infinity()
//对象的面积阈值
static float person_area;
static float bicycle_area;
static float tricycle_area;
static float car_area;
static float suv_area;
static float truck_area;
static float heavytruck_area;
static float coach_area;

 //图像的大小
static float img_wideth;
static float img_height;
static float img_area;
 //图像边缘区域
static float edge_left;
static float edge_right;
static float edge_top;
static float edge_below;

 //范围r,可大一些,尽量使相撞物体包含在内
static  float r_threshold;
static  float r_big_threshold;//truck,heavytruck,coach

  //碰撞距离阈值
static  float crash_distance;

  //疑似碰撞对象的坐标移动阈值
static  float r_move;//not used


  //v filter
static  float v_bic=sqrt(25*25+10*10);//bicycle
static  float v_car=sqrt(30*30+30*30);//car,suv
static  float v_truck=sqrt(30*30+30*30);//truck,heavytruck,coach


/*
  功能：初始化阈值
  输入：视频的分辨率
  输出：无
*/
static void RXY_threinit(float& img_wideth,float& img_height){
     float W1=1920.0;float H1=1080.0;
     float AREA1=W1*H1;

      edge_left=img_wideth*(94.5/W1);
      edge_right=img_wideth*(1750/W1);
      edge_top=img_height*(300/H1);
      edge_below=img_height*(886/H1);

      r_threshold=img_wideth*(400/W1);//
      r_big_threshold=img_wideth*(440/W1);

      crash_distance=img_wideth*(150/W1);

      img_area=img_height*img_wideth;

      person_area=img_area*(786/AREA1);
      bicycle_area=img_area*(786/AREA1);
      tricycle_area=img_area*(1475/AREA1);
      car_area=img_area*(999/AREA1);
      suv_area=img_area*(1338/AREA1);
      truck_area=img_area*(3067/AREA1);
      heavytruck_area=img_area*(4180/AREA1);
      coach_area=img_area*(3143/AREA1);
 }




//加速度阈值
// float asum_threshold=1.412;//没用,因为不同类型的加速度阈值不同

//移动因数:检测物体移动大于kmove*wideth时为移动
constexpr float kmove=0.1;

constexpr float nmove=0.05;

//疑似对象的检查帧数
constexpr int like_check_thre=11;
//未出事故的消除对象检查帧数
constexpr int noacci_count_thre=1;
//事故对象的相撞对象的检查帧数
constexpr int neigh_check_thre=30;
//速度滤波
constexpr float vfilter_thre=300;
constexpr float vfilter_lowthre=20;// 低速下限
constexpr float vfstay_lowstay=15;
constexpr float vy_filter_thre=3.8;// y向行驶相撞物体的vyf下限
constexpr float vsumfilter_forstay_thre=30;
constexpr float vyf_gain=1;// vy_filter的放大增益

constexpr float vsum_staythre=10;//速度
constexpr float slide_staythre=5;//坐标
constexpr float nomiss_vsum_staythre=25;
constexpr float nomiss_slide_staythre=15;
constexpr int stay_checkthre=200;

//加速度阈值
constexpr float asum_thre=900;
//速度阈值(未消畸变)
constexpr float vsum_thre=500;
#endif // DEFTHRESHOLD_H


