#ifndef MYHEADER_H
#define MYHEADER_H
#include <vector>
#include <dection.h>
#include "tracker.h"
#include <map>
std::vector<std::vector<Rect_with_class>> ProcessLabel(std::ifstream& label_file) ;
float CalculateIou(const Rect_with_class& det, const Tracker& track);
void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,size_t nrows, size_t ncols,std::vector<std::vector<float>>& association) ;
void AssociateDetectionsToTrackers(const std::vector<Rect_with_class>& detection,
        std::map<int, Tracker>& tracks,
        std::map<int, Rect_with_class>& matched,
        std::vector<Rect_with_class>& unmatched_det,
        float iou_threshold);
int outputdata();
std::vector<std::string>  mainjudge(std::string);
#endif // MYHEADER_H
