#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
int test(){
    std::vector<cv::Rect> box;
    box.emplace_back(1,2,6,4);
    std::cout<<box[0].x<<std::endl;
    std::cout<<box[0].y<<std::endl;
    std::cout<<box[0].width<<std::endl;
    std::cout<<box[0].height<<std::endl;
    std::cout<<box[0].br().x<<std::endl;
    std::cout<<box[0].tl().x<<std::endl;
    std::cout<<box[0].tl().y<<std::endl;
    std::vector<std::vector<float>>iou_matrix;
    std::vector<std::vector<float>>association;
    float a[4][4]={{3,4,5,2},{6,5,7,6},{9,6,4,5},{5,3,6,6}};

    iou_matrix.resize(4, std::vector<float>(4));
    association.resize(4, std::vector<float>(4));

    for(int rowcount = 0;rowcount < 4;rowcount++)
        for(int colcount = 0;colcount < 4;colcount++)
            iou_matrix[rowcount][colcount]=a[rowcount][colcount];

    //HungarianMatching(iou_matrix,4,4,association);

    /*for(int rowcount = 0;rowcount < 4;rowcount++){
        for(int colcount = 0;colcount < 4;colcount++){
        std::cout<<association[rowcount][colcount];
        if(colcount==3)
            std::cout<<std::endl;
        }

    }*/
    return 0;
}
