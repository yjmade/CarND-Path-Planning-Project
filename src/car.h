//
//  car.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//

#ifndef car_h
#define car_h

#include <vector>
#include <math.h>
using namespace std;

const int MYSELF_ID=-1;

class Car {


public:
    int id_;
    double  x_, y_, vx_, vy_, s_, d_;
    int lane_;
    double speed_;
    double yaw_;
    double distance_;
    double end_s_;

    Car(){};
    Car(vector<double> &sensor_fusion, Car my_car, double future_time):
        id_((int) round(sensor_fusion[0])),
        x_(sensor_fusion[1]),
        y_(sensor_fusion[2]),
        vx_(sensor_fusion[3]),
        vy_(sensor_fusion[4]),
        s_(sensor_fusion[5]),
        d_(sensor_fusion[6])
    {
        lane_=int (d_/4.);
        speed_=sqrt(vx_*vx_+vy_*vy_);
        yaw_=atan2(vy_, vx_);
        distance_=s_-my_car.s_;
        end_s_=s_+future_time*speed_;
    };

    Car(double x,double y, double s, double d, double speed, double yaw, double end_s):
        x_(x),
        y_(y),
        s_(s),
        d_(d),
        speed_(speed),
        yaw_(yaw),
        distance_(0),
        end_s_(end_s)
    {
        id_=MYSELF_ID;
        lane_=int (d_/4.);
        vx_=cos(yaw_)*speed;
        vy_=sin(yaw_)*speed;
    };

    // int get_lane(){
    //     return int (d_/4.);
    // };
    // double get_speed(){
    //     return
    // }


};

#endif /* car_h */
