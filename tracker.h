#ifndef TRACKER_H
#define TRACKER_H
#include <opencv2/core.hpp>
#include "kalman_filter.h"
#include "dection.h"
#include  <eigen3/Eigen/Dense>

class Tracker {
public:
    // Constructor
    Tracker();

    void Init(const Rect_with_class& bbox);
    void Predict(float dt);
    void Update(const Rect_with_class& bbox);
    Rect_with_class GetStateAsBbox() const;
    Eigen::VectorXd GetMotionState() const;
    double generateGaussianNoise(double mu, double sigma);
    void drawstateline(cv::Mat frame,cv::Scalar color);
    float GetNIS() const;
    int coast_cycles_ = 0;
    int hit_streak_ = 0;
    int category;
    std::vector<cv::Point2d>state_save;
private:
    Eigen::VectorXd ConvertBboxToObservation(const Rect_with_class& bbox) const;
    Rect_with_class ConvertStateToBbox(const Eigen::VectorXd &state) const;

    KalmanFilter kf_;
};

#endif // TRACKER_H
