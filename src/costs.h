//
//  car.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//

#ifndef costs_h
#define costs_h

#include "consts.h"
#include "path_planner.h"
#include "utils.h"

double cost_efficiency(SensorFusion &sensor_fusion, const StateInfo &state){
    double min_speed=sensor_fusion.get_lane_min_speed_in_front(state.target_lane_);
    return (MAX_SPEED-min_speed)/MAX_SPEED;
}

double cost_comfort(SensorFusion &sensor_fusion, const StateInfo &state){
    if(state.state_==STATE_KEEP_LANE)return 0;
    return 1;
}

double cost_safety(SensorFusion &sensor_fusion, const StateInfo &state){
    // get safe buffer distance
    vector<double> distances=sensor_fusion.get_buffer_distances(state.target_lane_);
    double distance=distances[0];//+distances[1];
    double cost=(MAX_BUFFER_DISTANCE-distance)/MAX_BUFFER_DISTANCE;
    if(cost<0)cost=0;
    return cost;

}

double cost_legality(SensorFusion &sensor_fusion, const StateInfo &state){
    //stay on road
    if(state.target_lane_<0)return 1;
    if(state.target_lane_>MAX_LANE)return 1;
    return 0;
}

double cost_feasibility(SensorFusion &sensor_fusion, const StateInfo &state){
    // //avoid colission
    // auto cars_in_target_lane=sensor_fusion.get_cars_in_lane(state.target_lane_);
    // // find vehicles that are pretty close to us
    // vector<Car> close_cars;
    // std::copy_if(cars_in_target_lane.begin(), cars_in_target_lane.end(), std::back_inserter(close_cars),
    //                 [&state](Car &car) {
    //                     return abs(s_distance(state.s_, car.end_s_)) < NO_COLLISION_LIMIT;
    //                 });

    // for (Car &close_car : close_cars) {
    //     double distance = s_distance(state.s_, close_car.end_s_);
    //     if (distance > 0) {
    //         // car is close to front
    //         if (distance < MIN_DISTANCE){
    //             return MIN_DISTANCE/distance;
    //         }
    //         //check if it is slowing down
    //         if (close_car.speed_ < state.speed_) {
    //             return 0.5;
    //         }
    //     } else {
    //         // car is close to rear
    //         if (distance > -MIN_DISTANCE){
    //             return MIN_DISTANCE/-distance;
    //         }
    //         //check if it is speeding up
    //         if (close_car.speed_ > state.speed_) {
    //             return 0.5;
    //         }
    //     }
    // }
    // return 0.;
    Car* close_car_front_p = sensor_fusion.closest_car_in_front(state.target_lane_);
    Car* close_car_rear_p = sensor_fusion.closest_car_in_rear(state.target_lane_);

    vector<double> costs;

    if(close_car_front_p!=nullptr){
        double distance_front = s_distance(state.s_, close_car_front_p->end_s_);
        print "distance_front", distance_front;

        if (distance_front < NO_COLLISION_LIMIT){
            costs.push_back(MIN_DISTANCE/distance_front);
        }else if(distance_front<MIN_DISTANCE){
            costs.push_back(close_car_front_p->speed_ > state.speed_?0:0.5);
        }
    }

    if(close_car_rear_p!=nullptr){
        double distance_rear = -s_distance(state.s_, close_car_rear_p->end_s_);
        print "distance_rear", distance_rear;
        if (distance_rear < NO_COLLISION_LIMIT){
            costs.push_back(MIN_DISTANCE/distance_rear);
        }else if(distance_rear<MIN_DISTANCE){
            costs.push_back(close_car_rear_p->speed_ < state.speed_?0:0.5);
        }
    }

    return sum(costs);
}


double get_cost(SensorFusion &sensor_fusion, const StateInfo &state){
    vector<double> costs={
        cost_efficiency(sensor_fusion, state)*COST_EFFICIENCY_WEIGHT,
        cost_comfort(sensor_fusion, state)*COST_COMFORT_WEIGHT,
        cost_safety(sensor_fusion, state)*COST_SAFETY_WEIGHT,
        cost_feasibility(sensor_fusion, state)*COST_FEASIBILITY_WEIGHT,
        cost_legality(sensor_fusion, state)*COST_LEGALITY_WEIGHT, // with possible state, this should be always 0
    };
    print "costs", costs, "state", state.target_lane_, state.current_lane_;
    return sum(costs);
}


#endif // costs_h