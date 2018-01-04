//
//  car.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//
#ifndef const_h
#define const_h

#include "utils.h"
#include <chrono>

const int LANES = 3;
const int MAX_LANE=LANES-1;
const double LANE_WIDTH=4.;
const double MAX_SPEED=mph2mps(49.5);
const double MIN_SPEED=mph2mps(0.1);

const double MAX_D=LANES*LANE_WIDTH;

const double MAX_SPEED_CHANGE = mph2mps(0.27);

const double MAX_BUFFER_DISTANCE=100;
const double MIN_DISTANCE=20;
const double NO_COLLISION_LIMIT= 10;

const double COST_EFFICIENCY_WEIGHT=8e0;
const double COST_COMFORT_WEIGHT=3e0;
const double COST_SAFETY_WEIGHT=8e0;
const double COST_LEGALITY_WEIGHT=1e6;
const double COST_FEASIBILITY_WEIGHT=1e1;

const auto PLAN_INTERVAL=chrono::seconds(1);
const double MAX_S = 6945.554;


#endif // const_h