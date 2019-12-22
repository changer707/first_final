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
#include "pthread.h"
#include <unistd.h>
#include <ctime>


#include "header.h"
#include <matrix.h>
#include <munkres.h>
#include "utils.h"
#include "dection.h"
#include <tracker.h>
#include <kalman_filter.h>
#include <darknet.h>
#include <improcess.h>

#include "mulplayer.h"
#include <QApplication>
#include "ui_mulplayer.h"



#define judge 1//draw bbox
using namespace std;

static float demo_thresh = 0;
static float demo_hier = .5;
static cv::Mat buff[3];
static int buff_index=0;
float* zyk_resize_data[3];
cv::VideoCapture cap;
static int running = 0;
float thresh=0.5;//参数设置
float nms=0.35;
int classes=8;
network* net;
std::vector<string> classNamesVec;
bool demo_done;
static int demo_frame = 3;
static int demo_index = 0;
static float **predictions;
static float *avg;

static int demo_total = 0;


int frame_index=1;
std::map<int, Tracker> tracks;
bool enable_display_flag=true;

int current_ID = 0;

std::vector<cv::Mat> image_vector;

int choose;
void* pthread_parallel(void* ptr);
pthread_mutex_t lock_det,lock_read;

int size_network(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            count += l.outputs;
        }
    }
    return count;
}
void remember_network(network *net)
{
    int i;
    int count = 0;
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(predictions[demo_index] + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
}
detection *avg_predictions(network *net, int *nboxes)
{

    int i, j;
    int count = 0;
    fill_cpu(demo_total, 0, avg, 1);
    for(j = 0; j < demo_frame; ++j){
        axpy_cpu(demo_total, 1./demo_frame, predictions[j], 1, avg, 1);
    }
    for(i = 0; i < net->n; ++i){
        layer l = net->layers[i];
        if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
            memcpy(l.output, avg + count, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
    detection *dets = get_network_boxes(net, buff[0].cols, buff[0].rows, demo_thresh, demo_hier, 0, 1, nboxes);
    return dets;
}

void* process(void* ptr){
  //  pthread_mutex_lock(&lock_det);
    int index;
    //if(!ptr)
        index=*((int*)ptr);
    running = 1;
    network_predict(net,zyk_resize_data[(buff_index+2)%3]);//网络推理
    free(zyk_resize_data[(buff_index+2)%3]);
   // remember_network(net);
    int nboxes=0;
 //   detection* dets = avg_predictions(net, &nboxes);
    detection *dets=get_network_boxes(net,buff[0].cols,buff[0].rows,thresh,0.5,0,1,&nboxes);//a struct pull
    if(nms){//nms is yuzhi??????????
        do_nms_obj(dets,nboxes,classes,nms);//no return
    }

    constexpr float dt = 0.03333333333333f;//predit of kalman
    for (auto &track : tracks) {
      track.second.Predict(dt);
    }
     cv::Mat img_tracking = buff[(buff_index+2)%3].clone();
     vector<Rect_with_class>one_frame_dec;
     Rect_with_class one_dec;
     for (int i = 0; i < nboxes; i++){//nboxes represent the number of dection
         bool flag=0;
         int className;
         for(int j=1;j<=classes;j++){
             if(dets[i].prob[j-1]>thresh){
                 if(!flag){
                     flag=1;
                     className=j;

                 }
             }
         }//


         if(flag){//read all bbox in the same frame
             int left = (dets[i].bbox.x - dets[i].bbox.w / 2.)*buff[(buff_index+2)%3].cols;
             int right = (dets[i].bbox.x + dets[i].bbox.w / 2.)*buff[(buff_index+2)%3].cols;
             int top = (dets[i].bbox.y - dets[i].bbox.h / 2.)*buff[(buff_index+2)%3].rows;
             int bot = (dets[i].bbox.y + dets[i].bbox.h / 2.)*buff[(buff_index+2)%3].rows;

             if (left < 0) left = 0;
             if (right > buff[(buff_index+2)%3].cols - 1) right = buff[(buff_index+2)%3].cols - 1;
             if (top < 0) top = 0;
             if (bot > buff[(buff_index+2)%3].rows - 1) bot = buff[(buff_index+2)%3].rows - 1;

             one_dec.init(left, top, fabs(left - right), fabs(top - bot),className);//is it right????????????
             one_frame_dec.push_back(one_dec);
         }
     }//for end one frame end,begin match
     free_detections(dets, nboxes);

    for (auto &trk : tracks) {//draw prediction box and to txt
         if (judge||(trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits)) {
             const auto &bbox = trk.second.GetStateAsBbox();
             cv::Rect myRect_to_cv(bbox.block.x,bbox.block.y,bbox.block.width,bbox.block.height);
             std::string display_str=std::to_string(frame_index);
             display_str+=std::to_string(trk.first);
             cv::putText(buff[(buff_index+2)%3], display_str, cv::Point(bbox.block.tl().x, bbox.block.tl().y - 5), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 1);
             cv::rectangle(buff[(buff_index+2)%3], myRect_to_cv, cv::Scalar(0, 255, 0), 3);
         }
     }

     std::map<int,Rect_with_class > matched;
     std::vector<Rect_with_class> unmatched_det;


     AssociateDetectionsToTrackers(one_frame_dec, tracks, matched, unmatched_det,0.45);
                  //相应的输入变量类型 std::vector<cv::Rect>& detection,std::map<int, Tracker>& tracks,std::map<int, cv::Rect>& matched,std::vector<cv::Rect>& unmatched_det,
                 /*** Update tracks with associated bbox ***/
                 for (const auto &match : matched) {
                     const auto &ID = match.first;
                     tracks[ID].Update(match.second);

                     const auto &bbox = tracks[ID].GetStateAsBbox();//draw trk
                     cv::Rect myRect_to_cv(bbox.block.x,bbox.block.y,bbox.block.width,bbox.block.height);
                     //std::string display_str=std::to_string(ID)+" "+std::to_string((int)(CalculateIou(match.second,tracks[ID])*100))+" "+std::to_string(frame_index);
                     std::string display_str=std::to_string(ID)+" "+std::to_string(frame_index);
                     //cv::putText(img_tracking, display_str, cv::Point(bbox.block.tl().x, bbox.block.tl().y - 10),cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 1);
                   //  cv::rectangle(img_tracking, myRect_to_cv,cv::Scalar(255, 0, 0), 1);//blue

                     Rect_with_class decbbox = match.second;//draw dec
                     //cv::Rect decRect_to_cv(decbbox.block.x,decbbox.block.y,decbbox.block.width,decbbox.block.height);
                   // cv::rectangle(img_tracking, decRect_to_cv,cv::Scalar(0, 255, 0), 1);//green
                 }
                 /*** Create new tracks for unmatched detections ***/
                 for (const auto &det : unmatched_det) {
                     Tracker tracker;
                     tracker.Init(det);
                     // Create new track and generate new ID
                     tracks[current_ID++] = tracker;
                 }
                 /*** Delete lose tracked tracks ***/
                 for (auto it = tracks.begin(); it != tracks.end();) {
                     if (it->second.coast_cycles_ > kMaxCoastCycles) {
                         it = tracks.erase(it);
                     } else {
                         it++;
                     }
                 }
                 //output_file << frame_index<<",";
                 for (auto &trk : tracks) { //tracker to  txt
                     const auto &bbox = trk.second.GetStateAsBbox();
                     if (trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits) {
                     // Print to terminal for debugging
                         std::cout << frame_index << "," << trk.first << "," << bbox.block.tl().x << "," << bbox.block.tl().y
                                   << "," << bbox.block.width << "," << bbox.block.height << ",1,-1,-1,-1"
                                   << " Hit Streak = " << trk.second.hit_streak_
                                   << " Coast Cycles = " << trk.second.coast_cycles_ << std::endl;

                         // Export to text file for metrics evaluation
                         //central point!
                      //   output_file << frame_index << "," << trk.first << "," <<"1,"<<trk.second.category<<","
                     //                 << bbox.block.tl().x+bbox.block.width*0.5<< "," << bbox.block.tl().y+bbox.block.height*0.5<< "," << bbox.block.width << "," << bbox.block.height<<","
                      //                <<trk.second.GetMotionState()(4)<<","<<trk.second.GetMotionState()(5)<<","<<trk.second.GetMotionState()(0)<<","<<trk.second.GetMotionState()(1)<<endl;
                         /*output_file   << trk.first << "," <<"1,"<<trk.second.category<<","
                                      << bbox.block.tl().x << "," << bbox.block.tl().y<< "," << bbox.block.width << "," << bbox.block.height<<","
                                      <<trk.second.GetMotionState()(3)<<","<<trk.second.GetMotionState()(5)<<","<<trk.second.GetMotionState()(0)<<","<<trk.second.GetMotionState()(1);*/
                         //Export to text file for velocity and acceleration of an ID
                         /*if (trk.first == ID_NUM) {

                             float v = sqrt(trk.second.GetMotionState()(4)*trk.second.GetMotionState()(4) + trk.second.GetMotionState()(5)*trk.second.GetMotionState()(5));
                             float a = sqrt(trk.second.GetMotionState()(0)*trk.second.GetMotionState()(0) + trk.second.GetMotionState()(1)*trk.second.GetMotionState()(1));
                             double v_angle = atan2(trk.second.GetMotionState()(5), trk.second.GetMotionState()(4));
                             double a_angle = atan2(trk.second.GetMotionState()(1), trk.second.GetMotionState()(0));

                             foutput_v << frame_index << "," << v << "," << a << "," << v_angle << "," << a_angle << endl;
                            }*/

                     }
                 }
                 //output_file<<endl;

                 if (enable_display_flag) {
                                 if (buff[(buff_index+2)%3].empty()) {
                                     std::cerr << "Could not open or find the image!!!" << std::endl;
                                     //return -1;
                                 }

                                 for (const auto &det : one_frame_dec) {
                                     // Draw detected bounding boxes in red
                                     cv::Rect myRect_to_cv(det.block.x,det.block.y,det.block.width,det.block.height);
                                     std::string display_str=std::to_string(frame_index);
                                     cv::rectangle(buff[(buff_index+2)%3], myRect_to_cv, cv::Scalar(0, 0, 255), 3);
                                 }

                                 for (auto &trk : tracks) {//draw box after matching

                                     if ((trk.second.hit_streak_ >= kMinHits || frame_index < kMinHits)) {
                                         const auto &bbox = trk.second.GetStateAsBbox();
                                         cv::Rect myRect_to_cv(bbox.block.x,bbox.block.y,bbox.block.width,bbox.block.height);
                                         std::string display_str=std::to_string(frame_index);
                                         display_str+=" ";
                                        // display_str+=std::to_string(trk.second.category);
                                       //  display_str+=" ";
                                         display_str+=std::to_string(trk.first);
                                         cv::putText(img_tracking, display_str, cv::Point(bbox.block.tl().x, bbox.block.tl().y - 10),cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 255, 255), 1);
                                         //cv::rectangle(img_tracking, myRect_to_cv, colors[trk.first % num_of_colors], 3);
                                         //trk.second.drawstateline(img_tracking,colors[trk.first % num_of_colors]);
                                         cv::rectangle(img_tracking, myRect_to_cv, cv::Scalar(255,0,0), 3);
                                         trk.second.drawstateline(img_tracking, cv::Scalar(255,0,0));
                                     }

                                 }
                                 std::string namewindow_det=std::to_string(index)+"det";
                                 std::string namewindow_tra=std::to_string(index)+"tra";
                                cv::namedWindow(namewindow_det, CV_WINDOW_NORMAL);
                                cv::namedWindow(namewindow_tra, CV_WINDOW_NORMAL);

                                cv::imshow(namewindow_det, buff[(buff_index+2)%3]);
                                // writer_dec<<frame;

                                 cv::imshow(namewindow_tra, img_tracking);
                                 // writer_match<<img_tracking;
                                 /*if(image_vector.size()<200)
                                 image_vector.insert(begin(image_vector),img_tracking);
                                 else{
                                     image_vector.pop_back();
                                     image_vector.insert(begin(image_vector),img_tracking);
                                 }*/
                                 int a=cv::waitKey(1);
                                 if(a==27){
                                     choose=27;
                                     pthread_t new_pthread;
                                     pthread_create(&new_pthread, 0, pthread_parallel,(void*)&choose);
                                 }
                 }
     ++frame_index;
    demo_index = (demo_index + 1)%demo_frame;
    //pthread_mutex_unlock(&lock_det);
    running=0;

}
void* display(void* ptr){
    //cv::namedWindow("Detecing", CV_WINDOW_NORMAL);
    //cv::imshow("Detecing",buff[(buff_index + 1)%3]);//opencv的窗口名字是opencv
   // int c=cv::waitKey(1);

}
/*void display_save(){
    for(auto img:image_vector){
        cv::namedWindow("tracking", CV_WINDOW_NORMAL);
        cv::imshow("tracking",img);
        cv::waitKey(1000);
    }
    cout<<"\nsize is "<<image_vector.size()<<endl;

}*/
void* read_frame(void* ptr){
    //free() free who????????????
  //pthread_mutex_lock(&lock_read);
  if (!cap.read(buff[buff_index])){//read one frame photo
  printf("demo done.\n");
  //return 0;
  demo_done=true;
   }
  cv::Mat rgbImg;
   cvtColor(buff[buff_index], rgbImg, cv::COLOR_BGR2RGB);
   float* srcImg;
   size_t srcSize=rgbImg.rows*rgbImg.cols*3*sizeof(float);
   srcImg=(float*)malloc(srcSize);
   imgConvert(rgbImg,srcImg);//将图像转为yolo形式

   size_t resizeSize=net->w*net->h*3*sizeof(float);
   zyk_resize_data[buff_index]=(float*)malloc(resizeSize);
   imgResize(srcImg,zyk_resize_data[buff_index],buff[(buff_index+2)%3].cols,buff[(buff_index+2)%3].rows,net->w,net->h);//缩放图像
   free(srcImg);
   //free(zyk_resize_data[buff_index]);
   //pthread_mutex_unlock(&lock_read);
   return 0;
}
void init_resize_data(cv::Mat image){
     cv::Mat rgbImg;
     cvtColor(image, rgbImg, cv::COLOR_BGR2RGB);
     float* srcImg;
     size_t srcSize=rgbImg.rows*rgbImg.cols*3*sizeof(float);
     srcImg=(float*)malloc(srcSize);
     imgConvert(rgbImg,srcImg);//将图像转为yolo形式

     size_t resizeSize=net->w*net->h*3*sizeof(float);
     zyk_resize_data[0]=(float*)malloc(resizeSize);
     zyk_resize_data[1]=(float*)malloc(resizeSize);
     zyk_resize_data[2]=(float*)malloc(resizeSize);
     imgResize(srcImg,zyk_resize_data[0],buff[(buff_index+2)%3].cols,buff[(buff_index+2)%3].rows,net->w,net->h);//缩放图像
     imgResize(srcImg,zyk_resize_data[1],buff[(buff_index+2)%3].cols,buff[(buff_index+2)%3].rows,net->w,net->h);//缩放图像
     imgResize(srcImg,zyk_resize_data[2],buff[(buff_index+2)%3].cols,buff[(buff_index+2)%3].rows,net->w,net->h);//缩放图像
     free(srcImg);

}

void* pthread_parallel(void* ptr){//to open camera,ptr point to the index of the camera

    pthread_mutex_init(&lock_det,NULL);
    pthread_mutex_init(&lock_read,NULL);

    std::string darknet_path="/home/ps/zk/darknet-master-1";
    std::string cfgfile =darknet_path+"/cfg/yolov3.cfg";//读取模型文件，请自行修改相应路径
    std::string weightfile =darknet_path+"/yolov3.weights";
    pthread_t process_thread;
    pthread_t read_thread;
    net=load_network((char*)cfgfile.c_str(),(char*)weightfile.c_str(),0);//加载网络模型
    set_batch_network(net, 1);//no return

    demo_total = size_network(net);
     predictions = (float**) calloc(demo_frame, sizeof(float*));
     for (int i = 0; i < demo_frame; ++i){
            predictions[i] = (float*) calloc(demo_total, sizeof(float));
        }
        avg =(float*) calloc(demo_total, sizeof(float));

    std::ifstream classNamesFile(darknet_path+"/data/coco.names");//标签文件coco有80类
    if (classNamesFile.is_open()){
        string className = "";
        while (getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }


    std::string video_path;
    if(choose==27) video_path="/home/ps/zk/darknet-master-1/sh-car-bicycle.mp4";
    else video_path=          "/home/ps/zk/darknet-master-1/sh-car-car-2.mp4";

    if(!cap.open(video_path)) cout<<"fail to open video file"<<video_path<<endl<<endl<<endl;
    cap.read(buff[0]);
    buff[1]=buff[0].clone();
    buff[2]=buff[0].clone();
    init_resize_data(buff[0]);
  //  process(ptr);
     while(!demo_done){
   // while(frame_index<60){
        buff_index = (buff_index + 1)%3;
        if(pthread_create(&process_thread, 0, process,ptr)) error("Thread creation failed");//????????????
        if(pthread_create(&read_thread, 0, read_frame, 0)) error("Thread creation failed");

        pthread_join(process_thread, 0);
        pthread_join(read_thread, 0);

    }
    return 0;
}

void* pthread_parallel(std::string video_path){//to open local video
    pthread_mutex_init(&lock_det,NULL);
    pthread_mutex_init(&lock_read,NULL);

    std::string darknet_path="/home/ps/zk/darknet-master-1";
    std::string cfgfile =darknet_path+"/cfg/yolov3.cfg";//读取模型文件，请自行修改相应路径
    std::string weightfile =darknet_path+"/yolov3.weights";
    pthread_t process_thread;
    pthread_t read_thread;
    net=load_network((char*)cfgfile.c_str(),(char*)weightfile.c_str(),0);//加载网络模型
    set_batch_network(net, 1);//no return
    std::ifstream classNamesFile(darknet_path+"/data/coco.names");//标签文件coco有80类
    if (classNamesFile.is_open()){
        string className = "";
        while (getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }

    if(choose==27) video_path="/home/ps/zk/darknet-master-1/car-bicycle.mp4";
    else video_path="/home/ps/zk/darknet-master-1/car-car-2.mp4";

    if(!cap.open(video_path)) cout<<"fail to open video file"<<video_path<<endl;
    cap.read(buff[0]);
    buff[1]=buff[0].clone();
    buff[2]=buff[0].clone();
    init_resize_data(buff[0]);
  //  process(ptr);
     while(!demo_done){
   // while(frame_index<60){
        buff_index = (buff_index + 1)%3;
        if(pthread_create(&process_thread, 0, process,0)) error("Thread creation failed");//????????????
        if(pthread_create(&read_thread, 0, read_frame, 0)) error("Thread creation failed");

        display(0);

        pthread_join(process_thread, 0);
        pthread_join(read_thread, 0);

    }
    return 0;
}

