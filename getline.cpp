#include<vector>
#include "missingbox.h"

//对于单个物体，根据他的运动方向，获取代表他的线段
void RXY_getline_fromdirection(mybox& box){
    float W=box.boxings.width;
    float H=box.boxings.height;
    box.A.x=box.boxings.x;
    box.A.y=box.boxings.y;
    float K=box.k;
    if(K==infinity_double){
        if(box.dx==0&&box.dy<0){
            //向上
            box.B.x=box.A.x;
            box.B.y=box.A.y-H*0.5;
        }
        else
        {   //向下
            box.B.x=box.A.x;
            box.B.y=box.A.y+H*0.5;
        }
    }
    else if(W*K<H){
        if(box.dx<0&&box.dy>=0){
            //向左下和左
            box.B.x=box.A.x-0.5*W;
            box.B.y=box.A.y+K*0.5*W;
        }
        else if(box.dx>0&&box.dy>=0){
            //向右下和右
            box.B.x=box.A.x+0.5*W;
            box.B.y=box.A.y+K*0.5*W;
        }
        else if(box.dx>0&&box.dy<0){
            //向右上
            box.B.x=box.A.x+0.5*W;
            box.B.y=box.A.y-K*0.5*W;
        }
        else {
            //向左上
            box.B.x=box.A.x-0.5*W;
            box.B.y=box.A.y-K*0.5*W;
        }
    }
    else{
        if(box.dx<0&&box.dy>0){
            box.B.x=box.A.x-0.5*H/K;
            box.B.y=box.A.y+0.5*H;
        }
        else if(box.dx>0&&box.dy>0){
            box.B.x=box.A.x+0.5*H/K;
            box.B.y=box.A.y+0.5*H;
        }
        else if(box.dx>0&&box.dy<0){
            box.B.x=box.A.x+0.5*H/K;
            box.B.y=box.A.y-0.5*H;
        }
        else{
            box.B.x=box.A.x-0.5*H/K;
            box.B.y=box.A.y-0.5*H;
        }
    }
}




//获取附近的当量线段
void RXY_getline_forneighbour(std::vector<mybox>& neighbourboxes){
    std::vector<mybox>::iterator it_neighbour;
    for(it_neighbour=neighbourboxes.begin();it_neighbour<neighbourboxes.end();++it_neighbour){
        RXY_getline_fromdirection(*it_neighbour);
    }
}



//对于消失ID，根据速度方向获取B点，来获取当量线段
void RXY_getline_formissbox(missingbox& miss_box){
    float W=miss_box.wideth;
    float H=miss_box.height;
    float K=miss_box.k;
    miss_box.A.x=miss_box.x;
    miss_box.A.y=miss_box.y;
//    miss_box.B.x=miss_box.x;
//    miss_box.B.y=miss_box.y+0.5*H;
    if(miss_box.k==infinity_double){
        //向下
        if(miss_box.dy>0){miss_box.B.x=miss_box.x;miss_box.B.y=miss_box.y+0.5*H;}
        //向上
        else{miss_box.B.x=miss_box.x;miss_box.B.y=miss_box.y-0.5*H;}
    }
    else if(K*W<H)
    {
        if(miss_box.dx*miss_box.dy>0){
            //右下
            if(miss_box.dx>0){miss_box.B.x=miss_box.x+0.5*W;miss_box.B.y=miss_box.y+K*0.5*W;}
            //左上
            else{miss_box.B.x=miss_box.x-0.5*W;miss_box.B.y=miss_box.y-K*0.5*W;}
        }
        else{
            //右上
            if(miss_box.dx>0){miss_box.B.x=miss_box.x+0.5*W;miss_box.B.y=miss_box.y-K*0.5*W;}
            //左下
            else{miss_box.B.x=miss_box.x-0.5*W;miss_box.B.y=miss_box.y+K*0.5*W;}
        }
    }
    else
    {
        if(miss_box.dx*miss_box.dy>0){
            //右下
            if(miss_box.dx>0){miss_box.B.y=miss_box.y+0.5*H;miss_box.B.x=miss_box.x+0.5*H/K;}
            //左上
            else{miss_box.B.y=miss_box.y-0.5*H;miss_box.B.x=miss_box.x-0.5*H/K;}
        }
        else{
            //右上
            if(miss_box.dx>0){miss_box.B.y=miss_box.y-0.5*H;miss_box.B.x=miss_box.x+0.5*H/K;}
            //左下
            else{miss_box.B.y=miss_box.y+0.5*H;miss_box.B.x=miss_box.x-0.5*H/K;}
        }
    }
}
