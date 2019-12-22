#include <cmath>
#include <algorithm>
#include <vector>
#include "getline.h"
//class node
//{
//public:
//    node() {}
//    float x;
//    float y;
//};
//向量叉积ABXAP
float RXY_cross(Rnode A,Rnode B,Rnode P){
    Rnode AB;
    AB.x=B.x-A.x;AB.y=B.y-A.y;
    Rnode AP;
    AP.x=P.x-A.x;AP.y=P.y-A.y;
    return AB.x*AP.y-AB.y*AP.x;
}
//向量点积AB.AP
float RXY_dot(Rnode A,Rnode B,Rnode P){
    Rnode AB;
    AB.x=B.x-A.x;AB.y=B.y-A.y;
    Rnode AP;
    AP.x=P.x-A.x;AP.y=P.y-A.y;
    return AB.x*AP.x+AB.y*AP.y;
}
//向量AB的模
float RXY_mol(Rnode A,Rnode B){
    return sqrt((B.x-A.x)*(B.x-A.x)+(B.y-A.y)*(B.y-A.y));
}

//判断P和AB的位置关系
int RXY_dir(Rnode A,Rnode B,Rnode P){
    if(RXY_cross(A,B,P)>0){return -1;}//逆时针方向上
    else if(RXY_cross(A,B,P)<0){return 1;}//顺时针方向上
    else{
        if(RXY_dot(A,B,P)<0){return -2;}//在AB反向延长线上
        else {
            if(RXY_mol(A,P)<=RXY_mol(A,B)){return 0;}//在线段AB上
            else{return 2;}//在AB延长线上
        }
    }
}

//求点P到线段AB的最短距离
float RXY_distmin_dotline(Rnode A,Rnode B,Rnode P){
//    node AB;
//    AB.x=B.x-A.x;AB.y=B.y-A.y;
//    node AP;
//    AP.x=P.x-A.x;AP.y=P.y-A.y;
    float r=RXY_dot(A,B,P)/(RXY_mol(A,B)*RXY_mol(A,B));
    if(r<0){
        return RXY_mol(A,P);
    }
    else if(r>1){
        return RXY_mol(B,P);
    }
    else {
        float distAC=r*RXY_mol(A,B);
        return sqrt(RXY_mol(A,P)*RXY_mol(A,P)-distAC*distAC);
    }
}


//求两条线段之间的最短距离
float RXY_distmin_twoline(Rnode A1,Rnode A2,Rnode B1,Rnode B2){
    std::vector<float> dist;
    std::vector<float>::iterator it_dist;
    if(RXY_dir(A1,A2,B1)*RXY_dir(A1,A2,B2)<=0&&RXY_dir(B1,B2,A1)*RXY_dir(B1,B2,A2)<=0){
        return 0;
    }
    else {
       dist.push_back(RXY_distmin_dotline(B1,B2,A1));
       dist.push_back(RXY_distmin_dotline(B1,B2,A2));
       dist.push_back(RXY_distmin_dotline(A1,A2,B1));
       dist.push_back(RXY_distmin_dotline(A1,A2,B2));
   float distmin=dist[0];
   for(int ia=0;ia<dist.size();ia++){
       if(dist[ia]<distmin){
           distmin=dist[ia];
       }
   }
       return distmin;
    }
}

//求neighbourboxes中所有的当量线段与消失ID当量线段的最小值
float RXY_distmin_neighbour(missingbox& miss_box,const std::vector<mybox>& neighbourboxes){
    std::vector<float> dist_neighbourboxes;//存放每条相邻线段与消失ID线段的最小距离
//    std::vector<mybox>::iterator it_neighbourboxes;
    int mark=0;
    for(int i=0;i<neighbourboxes.size();i++){
    float dist= RXY_distmin_twoline(miss_box.A,miss_box.B,neighbourboxes.at(i).A,neighbourboxes.at(i).B);
        dist_neighbourboxes.push_back(dist);
//        dist_neighbourboxes.push_back(RXY_distmin_twoline(miss_box.A,miss_box.B,(*it_neighbourboxes).A,(*it_neighbourboxes).B));
    }

    //找出所有最小距离中的最小值
float mindist=dist_neighbourboxes.at(0);
for(int j=0;j<dist_neighbourboxes.size();j++){
    if(dist_neighbourboxes.at(j)<mindist){
        mindist=dist_neighbourboxes.at(j);
        mark=j;
    }
}

//记录相撞对象的ID,class,x,y,w,h,vx,vy
miss_box.crashone.ID_number=neighbourboxes.at(mark).ID_number;
miss_box.crashone.typenumber=neighbourboxes.at(mark).typenumber;
miss_box.crashone.boxings.x=neighbourboxes.at(mark).boxings.x;
miss_box.crashone.boxings.y=neighbourboxes.at(mark).boxings.y;
miss_box.crashone.boxings.width=neighbourboxes.at(mark).boxings.width;
miss_box.crashone.boxings.height=neighbourboxes.at(mark).boxings.height;
miss_box.crashone.vx=neighbourboxes.at(mark).vx;
miss_box.crashone.vy=neighbourboxes.at(mark).vy;
miss_box.crashone.v_sum=neighbourboxes.at(mark).v_sum;

//    std::vector<float>::iterator it_dist_neighbourboxes;
//    it_dist_neighbourboxes=std::min_element(dist_neighbourboxes.begin(),dist_neighbourboxes.end());
//    float mindist=*it_dist_neighbourboxes;
    return mindist;
}
