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
    auto distances=sensor_fusion.get_buffer_distances(state.target_lane_);
    double distance=distances[0]+distances[1];
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
    //avoid colission
    return 0;
}


double get_cost(SensorFusion &sensor_fusion, const StateInfo &state){
    double cost=0;
    cost+=cost_efficiency(sensor_fusion, state)*COST_EFFICIENCY_WEIGHT;
    cost+=cost_comfort(sensor_fusion, state)*COST_COMFORT_WEIGHT;
    cost+=cost_safety(sensor_fusion, state)*COST_SAFETY_WEIGHT;
    cost+=cost_legality(sensor_fusion, state)*COST_LEGALITY_WEIGHT;
    cost+=cost_feasibility(sensor_fusion, state)*COST_FEASIBILITY_WEIGHT;
    return cost;
}


#endif // costs_h