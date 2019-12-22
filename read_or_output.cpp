#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "dection.h"
#include "cmath"
#include "utils.h"
using namespace cv;
//1person  4car  2bicycle   6truck  3tricycle  5suv  7heavetruck   8coach
std::vector<std::vector<Rect_with_class>> ProcessLabel(std::ifstream& label_file) {
    //需要加一步转换，将高宽变成xy坐标，注意y的正负方向
    // Process labels - group bounding boxes by frame index
    std::vector<std::vector<Rect_with_class>> bbox;
    std::vector<Rect_with_class> bbox_per_frame;
    // Label index starts from 1
    int current_frame_index = 1;
    std::string line;
    int tlx,tly,w,h;
    while (std::getline(label_file, line)&&bbox.size()<900) {
        std::stringstream ss(line);
        // Label format 1,-1,car,0.999999,413.671021,346.401642,54.901081,62.437252,-1,-1,-1
        std::vector<float> label;
        std::string data;
        int zyk_count=0;

        while (std::getline(ss , data, ',')) {//here need to right!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            if(zyk_count!=2)
            {label.push_back(std::stof(data));
                //std::cout<<data<<"  ";
                //if(zyk_count==10) std::cout<<std::endl;
            }
            else{
                 if(data=="person")  label.push_back(1);
                 else if(data=="car") label.push_back(2);
                 else if(data=="bicycle") label.push_back(3);
                 else if(data=="truck") label.push_back(4);
                 else if(data=="tricycle") label.push_back(5);
                 else if(data=="suv") label.push_back(6);
                 else if(data=="heavetruck") label.push_back(7);
                 else if(data=="coach") label.push_back(8);
                 else std::cout<<"error input"<<std::endl;
            }

            zyk_count++;
        }//得到第一行的data

        if (label[0] != current_frame_index) {
            current_frame_index = static_cast<int>(label[0]);
            bbox.push_back(bbox_per_frame);
            //std::cout<<bbox.size()<<"size"<<std::endl;
            bbox_per_frame.clear();
        }

        // Ignore low confidence detections
        if (label[3] > kMinConfidence){//need to right!!!!!!!!!!!!!!!!!!!!
            tlx=label[4]-0.5*label[6];
            tly=label[5]-0.5*label[7];
            w=label[6];
            h=label[7];
           Rect_with_class rect;
           rect.init(tlx,tly,w,h,label[2]);
            bbox_per_frame.emplace_back(rect);
            //bbox_per_frame.emplace_back(label[1], label[2], label[3], label[4]);
        }
    }//while end
    // Add bounding boxes from last frame
    bbox.push_back(bbox_per_frame);
    return bbox;
}
int outputdata(){
   std::string label_path = "/media/zykzyh/study/forth/my _hu/a.txt";
    std::ifstream label_file(label_path);
    if (!label_file.is_open()) {
       std::cout << "Could not open or find the label!!!" << std::endl;
       return -1;
       }
    std::string output_path = "/media/zykzyh/study/forth/my _hu/c.txt";//有输出文件
    std::ofstream output_file(output_path);
     std::vector<std::vector<Rect_with_class>> all_detections = ProcessLabel(label_file);//读取数据，所有帧的所有目标，每帧在同一行，不含帧数，
     for(auto itt=begin(all_detections);itt!=end(all_detections);++itt)
         for(auto it=begin(*itt);it!=end(*itt);++it)

     output_file << (*it).block.x << ","  <<(*it).block.y<< "," << (*it).block.width << "," << (*it).block.height<<","
                 <<(*it).category<< ",1,-1,-1,-1\n";
     label_file.close();
    return 0;
}
