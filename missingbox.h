#ifndef MISSINGBOX_H
#define MISSINGBOX_H
#include<vector>
#include "defthreshold.h"
#include "mybox.h"
class missingbox
{
public:
    missingbox() {}
    int missing_frame;
    int missing_ID;
    int missing_type;

    int neigh_change=0;
    float missing_area;
    float x;
    float y;
    float wideth;
    float height;
    float dx;//有正负
    float dy;//有正负
    float k;//绝对值
    float vx;//有正负
    float vy;//有正负
    float v_sum;
    //画当量线段
    Rnode A;//中心点
    Rnode B;//由速度方向确定点
    bool flag_runaway;//驶出画面或驶远：true
    bool flag_accident=false;//事故发生flag
    bool flag_cout=false;//统计疑似对象个数用
    bool flag_merge=false;//疑似事故对象是否在后续几帧中出现
    bool flag_merge_acci=false;
    bool flag_merge_stay1=false;//crashone静止检测中是否出现
    bool flag_output=false;//是否已打印
    bool flag_foutput=false;
    bool flag_vout=false;
    bool flag_fvout=false;
    bool flag_sout=false;
    bool flag_tout=false;
    bool flag_ooutput=false;
    bool flag_stay=false;

    int like_check=0;//记录疑似对象的二次检查帧数
    int noacci_count=0;//用来消除没有事故的like_accident对象
    int stay_check=0;

    mybox crashone;//表示相撞的另一个对象
//    std::vector<int> missing_frame;
//    std::vector<int> missing_ID;
//    std::vector<int> missing_type;
//    std::vector<float> missing_area;
    //速度修正
    float k_vfilter;
    float b_vfilter;
    float l_vfilter;
    float vy_filter;
    float vx_filter;

    float vsum_filter;

    std::vector<float> dist_slide;//记录crashone相撞后已滑行的总距离(per frame),从而判断是否静止




    float missbox_areathod(const int&missing_type){
        float areathod;
        switch (missing_type) {
        case 1:
             areathod=person_area;
            break;
        case 2:
            areathod=bicycle_area;
            break;
        case 3:
            areathod=tricycle_area;
            break;
        case 4:
            areathod=car_area;
            break;
        case 5:
            areathod=suv_area;
            break;
        case 6:
            areathod=truck_area;
            break;
        case 7:
            areathod=heavytruck_area;
            break;
        default:
            areathod=coach_area;
            break;
        }
        return areathod;
    }



};






#endif // MISSINGBOX_H

