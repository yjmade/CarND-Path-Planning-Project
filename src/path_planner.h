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

class PathPlanner {
private:
    const vector<double> maps_s_;
    const vector<double> maps_x_;
    const vector<double> maps_y_;
    const vector<double> maps_d_x_;
    const vector<double> maps_d_y_;

    int current_state;
    vector<double> previous_path_x_;
    vector<double> previous_path_y_;

public:
    // Car my_car_;
    double ref_vel_;
    double lane_;
    SensorFusion sensor_fusion_;

    PathPlanner(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_d_x,const vector<double> &maps_d_y):
        maps_s_(maps_s),
        maps_x_(maps_x),
        maps_y_(maps_y),
        maps_d_x_(maps_d_x),
        maps_d_y_(maps_d_y),
        current_state(0),
        ref_vel_(0),
        lane_(1)
    {
    };

    void new_sense(const Car &my_car, const vector<vector<double> > &raw_sensor_fusion, vector<double> previous_path_x, vector<double> previous_path_y){
        sensor_fusion_.new_sense(my_car, raw_sensor_fusion, previous_path_x_.size()*.02);
        previous_path_x_=previous_path_x;
        previous_path_y_=previous_path_y;
        // initialize
        if (previous_path_x_.size()<2){
            lane_=my_car.lane_;
            ref_vel_=my_car.speed_;
        }
    };
    void next_state(){
        Car closest_car=sensor_fusion_.closest_car();
        if(closest_car.id_>0 && closest_car.speed_<ref_vel_+1.){
          ref_vel_-=0.3/2.24;
          lane_=0;
        }else if(ref_vel_<49.5/2.24){
          ref_vel_+=0.3/2.24;
        }

        if(ref_vel_<1){
            ref_vel_=1;
        }
        cout<<"closest_speed "<<closest_car.speed_<<"\tvel "<<ref_vel_<<"\tlane "<<lane_<<endl;

    };
    vector<vector<double> > next_path();
};

#endif /* path_planner_h */
