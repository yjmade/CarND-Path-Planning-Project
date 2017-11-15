//
//  car.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//

#ifndef sensor_fusion_h
#define sensor_fusion_h

#include <vector>
#include "car.h"
using namespace std;


class SensorFusion {


public:
    vector<Car> cars_;

    void new_sense(vector<vector<double>> raw_sensor_fusion){
        vector<Car> new_cars;

        for (auto sensor_fusion_item: raw_sensor_fusion){
            Car car=Car(sensor_fusion_item);
            new_cars.push_back(sensor_fusion_item);
        }

        // TODO
        cars_=new_cars;
    };

};

#endif /* sensor_fusion_h */
