#ifndef DECTION_H
#define DECTION_H
#include <opencv2/core.hpp>
//template <typename T>
class Rect_with_class{

    public:
        cv::Rect block;
        int category;
        void init(int a,int b,int c,int d,int e){
            block.x=a;
            block.y=b;
            block.width=c;
            block.height=d;
            category=e;
        }
        void init(int a,int b,int c,int d){
            block.x=a;
            block.y=b;
            block.width=c;
            block.height=d;

        }
};

#endif // DECTION_H
