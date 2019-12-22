#ifndef GETLINE_H
#define GETLINE_H
#include<vector>
#include "missingbox.h"

//对于单个物体，根据他的运动方向，获取代表他的线段
void RXY_getline_fromdirection(mybox& box);

//获取附近的当量线段
void RXY_getline_forneighbour(std::vector<mybox>& neighbourboxes);

//对于消失ID，根据速度方向获取B点，来获取当量线段
void RXY_getline_formissbox(missingbox& miss_box);


#endif // GETLINE_H
