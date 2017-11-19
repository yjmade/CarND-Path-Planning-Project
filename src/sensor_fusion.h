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
#include <iostream>
using namespace std;


class SensorFusion {


public:
    vector<Car> cars_;

    Car my_car_;
    SensorFusion(){};

    void new_sense(const Car &my_car,const vector<vector<double> > &raw_sensor_fusion, double future_time){
        my_car_=my_car;
        vector<Car> new_cars;

        for (auto sensor_fusion_item: raw_sensor_fusion){
            Car car=Car(sensor_fusion_item, my_car_, future_time);
            new_cars.push_back(car);
        }
        // TODO
        cout<<"cars"<<cars_.size()<<" news_cars"<<new_cars.size();
        cars_=new_cars;
        cout<<"cars_"<<cars_.size()<<endl;
    };

    Car closest_car(){
      cout<<"name\tid\tlane\ts\td\tspeed\tdis\tsamel\n";
      cout<<"var\t"<<my_car_.id_<<"\t"<<my_car_.lane_<<"\t"<<my_car_.s_<<"\t"<<my_car_.d_<<"\t"<<my_car_.speed_<<"\t"<<my_car_.distance_<<"\t"<<true<<endl;

      // double closest_speed;
      double shortest_distance=99999;
      Car the_closest_car;
      for(auto car: cars_){
        if (car.lane_==my_car_.lane_){
          // double check_car_s = car.s_+((double)prev_size*.02*car.speed_);
          double distance=car.end_s_-my_car_.end_s_;
          double real_distance=car.s_-my_car_.s_;
          cout<<"car\t"<<car.id_<<"\t"<<car.lane_<<"\t"<<car.s_<<"\t"<<car.d_<<"\t"<<car.speed_<<"\t"<<real_distance<<"\t"<<distance<<endl;
          if(distance<0||distance>shortest_distance){
            continue;
          }
          shortest_distance=distance;
          the_closest_car=car;
        }
      }
      if (shortest_distance<30){
        return the_closest_car;
      }
      return my_car_;
    };

};

#endif /* sensor_fusion_h */
