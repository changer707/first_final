#include <iostream>
#include <fstream>
#include <matrix.h>
#include <munkres.h>
#include <vector>
#include <map>
#include <string>
#include <initializer_list>
#include <improcess.h>
#include <tracker.h>
#include <kalman_filter.h>
#include "utils.h"
#include "dection.h"
#include "header.h"
#include <cstdlib>

std::ofstream iou_txt("/home/ps/zk/darknet-master-1/iou.txt",std::ios_base::out|std::ios::app);
float CalculateIou(const Rect_with_class& det, const Tracker& track) {//how to konw track's class????????????????
    auto trk = track.GetStateAsBbox();
    float iou;
    // get min/max points
    //if(abs(det.category-track.category)>1)
    if(0)
             iou = 0 ;
    else{
    auto xx1 = std::max(det.block.tl().x, trk.block.tl().x);
    auto yy1 = std::max(det.block.tl().y, trk.block.tl().y);
    auto xx2 = std::min(det.block.br().x, trk.block.br().x);
    auto yy2 = std::min(det.block.br().y, trk.block.br().y);
    auto w = std::max(0, xx2 - xx1);
    auto h = std::max(0, yy2 - yy1);

    // calculate area of intersection and union
    float det_area = det.block.area();
    float trk_area = trk.block.area();
    auto intersection_area = w * h;
    float union_area = det_area + trk_area - intersection_area;
    iou = intersection_area / union_area;
}
    return iou;
}


void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,size_t nrows, size_t ncols,std::vector<std::vector<float>>& association) {
    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values



    iou_txt<<"****new frame****"<<nrows<<"      "<<ncols<<"\n";



    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            iou_txt<<iou_matrix[i][j]<<",";
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = 1-iou_matrix[i][j];
            }
            else {
                // TODO: figure out why we have to assign value to get correct result
                matrix(i, j) = 1.0f;
                //matrix(i, j) = 0.0f;
                //why to be 1
            }
        }
        iou_txt<<std::endl;
    }
        // Display begin matrix state.
        /*for (size_t row = 0 ; row < nrows ; row++) {
            for (size_t col = 0 ; col < ncols ; col++) {
                std::cout.width(10);
             std::cout << matrix(row,col) << ",";
           }
            std::cout << std::endl;
        }
        std::cout << std::endl;*/

    Munkres<float> m;//Munkres是一个class muban
    m.solve(matrix);
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);          
        }
    }
}
void AssociateDetectionsToTrackers(const std::vector<Rect_with_class>& detection,
        std::map<int, Tracker>& tracks,
        std::map<int, Rect_with_class>& matched,
        std::vector<Rect_with_class>& unmatched_det,
        float iou_threshold ) {

    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }
    if (detection.size()==0) {

        return;
    }


    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    // resize association matrix based on number of detection and tracks
    association.resize(detection.size(), std::vector<float>(tracks.size()));


    // row - detection, column - tracks
    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {

            iou_matrix[i][j] = CalculateIou(detection[i], trk.second);

            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= iou_threshold) {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}
