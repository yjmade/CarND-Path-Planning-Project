//
//  car.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//

#ifndef car_h
#define car_h

#include <vector>
using namespace std;

const int MYSELF_ID=-1;

class Car {


public:
    int id_;
    double  x_, y_, vx_, vy_, s_, d_;
    int lane_;
    double speed_;
    double yaw_;
    Car(vector<double> &sensor_fusion):
        id_((int) std::round(sensor_fusion[0])),
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
    };

    Car(double x,double y, double s, double d, double speed, double yaw):
        x_(x),
        y_(y),
        s_(s),
        d_(d),
        speed_(speed),
        yaw_(yaw)
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
