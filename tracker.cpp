#include "tracker.h"
#include <opencv2/opencv.hpp>
#include <QDebug>

extern float noise_rx,noise_ry,noise_rw,noise_rh,noise_ax,noise_ay,noise_aw,noise_ah;
Tracker::Tracker() : kf_(10, 4) {

    /*** Define constant velocity model ***/
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height,ddx,ddy   10

   /* kf_.x_(4,0)=10;
    kf_.x_(5,0)=10;
    kf_.x_(6,0)=1;
    kf_.x_(7,0)=1;
    kf_.x_(8,0)=50;
    kf_.x_(9,0)=50;*/

    kf_.F_ <<

              1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
              0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    kf_.P_ <<
              50, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 50, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0,10, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 10, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 2000, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 2000, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 20, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 20, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 30000, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 30000;

    kf_.H_ <<
              1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

//    float noise_rx,noise_ry;
//    float noise_rw,noise_rh;

//    noise_rx = noise_ry = 10;
//    noise_rx = noise_ry = 50;

    kf_.R_ <<
              noise_rx, 0, 0, 0,
              0, noise_ry, 0, 0,
              0, 0, noise_rw, 0,
              0, 0, 0, noise_rh;



}


// Get predicted locations from existing trackers
// dt is time elapsed between the current and previous measurements
void Tracker::Predict(float dt) {
    kf_.F_(0, 4) = dt;
    kf_.F_(1, 5) = dt;
    kf_.F_(4, 8) = dt;
    kf_.F_(5, 9) = dt;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;


    // Variance
//    float noise_ax,noise_ay;
//    float noise_aw,noise_ah;
//    noise_ax = noise_ay = 100.0;
//    noise_aw = noise_ah = 100.0;
    kf_.Q_ <<
        dt_4/4*noise_ax, 0, 0, 0, 0, dt_3/2*noise_ax, 0, 0, 0, 0,
        0, dt_4/4*noise_ay, 0, 0, 0, 0, dt_3/2*noise_ay, 0, 0, 0,
        0, 0, dt*dt_4/4*noise_aw, 0, 0, 0, 0, dt_3/2*noise_aw, 0, 0,
        0, 0, 0, dt*dt_4/4*noise_ah, 0, 0, 0, 0, dt_3/2*noise_ah, 0,
        0, 0, 0, 0, dt_2*noise_ax, 0, 0, 0, 0, dt_3/2*noise_ah,
        dt_3/2*noise_ax, 0, 0, 0, 0, dt_2*noise_ay, 0, 0, 0, 0,
        0, dt_3/2*noise_ay, 0, 0, 0, 0, dt_2*noise_aw, 0, 0, 0,
        0, 0, dt_3/2*noise_aw, 0, 0, 0, 0, dt_2*noise_ah, 0, 0,
        0, 0, 0, dt_3/2*noise_ah, 0, 0, 0, 0, 25*dt*noise_ah,0 ,
        0, 0, 0, 0, dt_3/2*noise_ah, 0, 0, 0, 0, 25*dt*noise_ah;

    /*kf_.Q_ <<0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1;*/
    //qDebug()<<noise_ax;

    kf_.Predict();

    // hit streak count will be reset
    if (coast_cycles_ > 0) {
        hit_streak_ = 0;
    }
    // accumulate coast cycle count
    coast_cycles_++;
}


// Update matched trackers with assigned detections
void Tracker::Update(const Rect_with_class& bbox) {

    // get measurement update, reset coast cycle count
    coast_cycles_ = 0;
    // accumulate hit streak count
    hit_streak_++;

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
    kf_.Update(observation);


}


// Create and initialize new trackers for unmatched detections, with initial bounding box
void Tracker::Init(const Rect_with_class &bbox) {
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
    hit_streak_++;
    category=bbox.category;
}


/**
 * Returns the current bounding box estimate
 * @return
 */
Rect_with_class Tracker::GetStateAsBbox() const {
    //return ConvertStateToBbox(kf_.x_);
    return ConvertStateToBbox(kf_.x_predict_);
}
Eigen::VectorXd Tracker::GetMotionState() const {
    return kf_.x_.tail(6);
}


float Tracker::GetNIS() const {
    return kf_.NIS_;
}
void Tracker::drawstateline(cv::Mat frame,cv::Scalar color){
    cv::Point2i point(kf_.x_(0),kf_.x_(1)+0.5*kf_.x_(3));
    if(state_save.size()<30)
    state_save.insert(std::begin(state_save),point);
    else {
        state_save.pop_back();
        state_save.insert(std::begin(state_save),point);
    }
    for (auto it=std::begin(state_save);it!=std::end(state_save);++it) {
       cv::circle(frame,*it , 2,color,-1);
    }

}

/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Tracker::ConvertBboxToObservation(const Rect_with_class& bbox) const{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);
    auto width = static_cast<float>(bbox.block.width);
    auto height = static_cast<float>(bbox.block.height);
    float center_x = bbox.block.x + width / 2;
    float center_y = bbox.block.y + height / 2;
    observation << center_x, center_y, width, height;
    return observation;
}


/**
 * Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
 * [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 *
 * @param state
 * @return
 */
Rect_with_class Tracker::ConvertStateToBbox(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    auto width = static_cast<int>(state[2]);
    auto height = static_cast<int>(state[3]);
    auto tl_x = static_cast<int>(state[0] - width / 2.0);
    auto tl_y = static_cast<int>(state[1] - height / 2.0);
    //cv::Rect rect(cv::Point(tl_x, tl_y), cv::Size(width, height));
    Rect_with_class rect;
    rect.init(tl_x,tl_y,width,height);//need to right
    return rect;
}
