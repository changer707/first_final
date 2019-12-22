#ifndef DISTANCE_H
#define DISTANCE_H
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
float RXY_cross(Rnode A,Rnode B,Rnode P);
//向量点积AB.AP
float RXY_dot(Rnode A,Rnode B,Rnode P);
//向量AB的模
float RXY_mol(Rnode A,Rnode B);

//判断P和AB的位置关系
int RXY_dir(Rnode A,Rnode B,Rnode P);

//求点P到线段AB的最短距离
float RXY_distmin_dotline(Rnode A,Rnode B,Rnode P);


//求两条线段之间的最短距离
float RXY_distmin_twoline(Rnode A1,Rnode A2,Rnode B1,Rnode B2);

//求neighbourboxes中所有的当量线段与消失ID当量线段的最小值
float RXY_distmin_neighbour(missingbox& miss_box,const std::vector<mybox>& neighbourboxes);


#endif // DISTANCE_H
