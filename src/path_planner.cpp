#include "path_planner.h"
#include "spline.h"
#include "utils.h"
#include "costs.h"


void PathPlanner::next_state(){
        // int current_state=state_;

  vector<int> possible_states={
    STATE_KEEP_LANE,
    STATE_CHANGE_LEFT,
    STATE_CHANGE_RIGHT
  };

  double lowest_cost=numeric_limits<double>::max();
  double lowest_cost_state;

  for(int state: possible_states){
    StateInfo state_info(
      state,
      sensor_fusion_.my_car_.lane_,
      sensor_fusion_.my_car_.end_s_,
      sensor_fusion_.my_car_.speed_
    );
    double cost=get_cost(sensor_fusion_, state_info);
    if (cost<lowest_cost){
      lowest_cost=cost;
      lowest_cost_state=state;
    }
  }

  // lane_=lowest_cost_state->target_lane_;
  state_=lowest_cost_state;
  Car* carp=sensor_fusion_.closest_car_in_front(lane_);


  // Car closest_car=sensor_fusion_.closest_car();



  if (state_ == STATE_CHANGE_LEFT && lane_ > 0) {
    lane_--;
    cout<<"STATE_CHANGE_LEFT"<<lane_<<"\n";
  }
  if (state_ == STATE_CHANGE_RIGHT && lane_ < LANES) {
    lane_++;
    cout<<"STATE_CHANGE_RIGHT"<<lane_<<"\n";
  }
  if(carp==nullptr){
    target_vel_=MAX_SPEED;
    cout<<"closest_speed "<<"null"<<"\tvel "<<ref_vel_<<"\tlane "<<lane_<<endl;
  }else{
    cout<<"closest_speed "<<carp->speed_<<"distance() "<<carp->distance_<<"\tlane "<<lane_<<endl;
    if(carp->distance_<MAX_BUFFER_DISTANCE)
      {target_vel_=carp->speed_;}
    else{
      target_vel_=MAX_SPEED;
    }
  }
};



vector<vector<double> > PathPlanner::next_path(){
    if(ref_vel_>target_vel_-1.){
      ref_vel_-=0.5/2.24;
    }else if(ref_vel_<MAX_SPEED){
      ref_vel_+=0.5/2.24;
    }
    // ref_vel_=target_speed;
    if(ref_vel_<0)ref_vel_=0;
    cout<<"ref_vel_ "<<ref_vel_<<"\ttarget_speed "<<target_vel_<<"\n";
    vector<double> ptsx;
    vector<double> ptsy;
    int prev_size=previous_path_x_.size();
    // cout<<prev_size<<"\n";
    // if(prev_size>0)cout<<"previous_path_x_\t"<<previous_path_x_<<"\n";
    double ref_x=sensor_fusion_.my_car_.x_;
    double ref_y=sensor_fusion_.my_car_.y_;
    double ref_yaw=deg2rad(sensor_fusion_.my_car_.yaw_);

    if (prev_size<2){
      double prev_car_x=sensor_fusion_.my_car_.x_-cos(sensor_fusion_.my_car_.yaw_);
      double prev_car_y=sensor_fusion_.my_car_.y_-sin(sensor_fusion_.my_car_.yaw_);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(sensor_fusion_.my_car_.x_);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(sensor_fusion_.my_car_.y_);

    }else{
      ref_x=previous_path_x_[prev_size-1];
      ref_y=previous_path_y_[prev_size-1];
      double ref_x_prev=previous_path_x_[prev_size-2];
      double ref_y_prev=previous_path_y_[prev_size-2];
      ref_yaw=atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);


    }
    double target_switch_dist=ref_vel_*3;
    vector<double> next_wp0=getXY(sensor_fusion_.my_car_.s_+target_switch_dist,(2+4*lane_),maps_s_,maps_x_,maps_y_);
    vector<double> next_wp1=getXY(sensor_fusion_.my_car_.s_+target_switch_dist+30,(2+4*lane_),maps_s_,maps_x_,maps_y_);
    vector<double> next_wp2=getXY(sensor_fusion_.my_car_.s_+target_switch_dist+60,(2+4*lane_),maps_s_,maps_x_,maps_y_);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // cout<<"start ptsx\t"<<ptsx<<"\tptsy\t"<<ptsy<<"\n";
    for(int i=0;i<ptsx.size();i++){
      double shift_x=ptsx[i]-ref_x;
      double shift_y=ptsy[i]-ref_y;

      ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
      ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

    }

    // cout<<"ptsx\t"<<ptsx<<"\tptsy\t"<<ptsy<<"\n";
    tk::spline spline;
    spline.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for(int i=0; i< previous_path_x_.size();i++){
      next_x_vals.push_back(previous_path_x_[i]);
      next_y_vals.push_back(previous_path_y_[i]);
    }


    double target_x=30. ;
    double target_y=spline(target_x);
    double target_dist=sqrt(target_x*target_x+target_y*target_y);

    double x_add_on=0;

    for(int i =1; i<=50-previous_path_x_.size();i++){
      double N = (target_dist/(.02*ref_vel_)); // 0.02 sec/frame , 2.24 for convert mph to m/s
      double x_point=x_add_on+(target_x)/N;
      double y_point=spline(x_point);

      x_add_on=x_point;

      double x_ref=x_point;
      double y_ref=y_point;

      x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
      y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
      x_point+=ref_x;
      y_point+=ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);


    }
    return {next_x_vals, next_y_vals};
}