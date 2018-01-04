//
//  car.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//

#ifndef path_planner_h
#define path_planner_h

#include <vector>
#include "car.h"
#include "sensor_fusion.h"
using namespace std;

static int const STATE_KEEP_LANE = 0;
static int const STATE_CHANGE_LEFT = 1;
static int const STATE_CHANGE_RIGHT = 2;
static int const STATE_PREPARE_CHANGE_LEFT = 3;
static int const STATE_PREPARE_CHANGE_RIGHT = 4;

class PathPlanner {
private:
    const vector<double> maps_s_;
    const vector<double> maps_x_;
    const vector<double> maps_y_;
    const vector<double> maps_d_x_;
    const vector<double> maps_d_y_;

    int state_;
    vector<double> previous_path_x_;
    vector<double> previous_path_y_;

    // void predict_state(){

    // };

public:
    // Car my_car_;
    double target_vel_;
    double ref_vel_;
    double lane_;
    double delta_t_;
    SensorFusion sensor_fusion_;

    PathPlanner(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_d_x,const vector<double> &maps_d_y):
        maps_s_(maps_s),
        maps_x_(maps_x),
        maps_y_(maps_y),
        maps_d_x_(maps_d_x),
        maps_d_y_(maps_d_y),
        state_(STATE_KEEP_LANE),
        target_vel_(0),
        ref_vel_(0),
        lane_(1),
        delta_t_(0)
    {
    };

    void new_sense(const Car &my_car, const vector<vector<double> > &raw_sensor_fusion, vector<double> &previous_path_x, vector<double> &previous_path_y){
        delta_t_=previous_path_x_.size()*.02;
        sensor_fusion_.new_sense(my_car, raw_sensor_fusion, delta_t_);
        previous_path_x_=previous_path_x;
        previous_path_y_=previous_path_y;
        // initialize
        if (previous_path_x_.size()<2){
            lane_=my_car.lane_;
            ref_vel_=my_car.speed_;
        }
    };
    void next_state();
    vector<vector<double> > next_path();
};


class StateInfo{
public:
    int state_;
    int current_lane_;
    int target_lane_;
    double s_, speed_;

    StateInfo(int state, int current_lane, double s, double speed):
    state_(state),
    current_lane_(current_lane),
    s_(s),
    speed_(speed)
    // delta_t_(delta_t)
    {
        if (state_==STATE_KEEP_LANE){
            target_lane_=current_lane;
        }
        else if (state_ == STATE_CHANGE_LEFT && target_lane_ > 0) {
          target_lane_=current_lane-1;
        }
        else if (state_ == STATE_CHANGE_RIGHT && target_lane_ < LANES) {
          target_lane_=current_lane+1;
        }
    }
};

#endif /* path_planner_h */
