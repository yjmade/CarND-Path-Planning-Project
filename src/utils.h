//
//  utils.h
//  Path_Planning
//
//  Created by Yang Jian on 14/11/2017.
//

#ifndef utils_h
#define utils_h

#include <vector>
#include <iostream>
#include <cmath>
#include "consts.h"


using namespace std;


constexpr double pi() { return M_PI; }

inline double deg2rad(double x){ return x * pi() / 180; };
inline double rad2deg(double x){ return x * 180 / pi(); };

inline double mph2mps(double mph) { return mph * 0.44704; };

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

inline double distance(double x1, double y1, double x2, double y2){
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
};


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

double getAccelation(double current_speed, double target_speed);

template<typename T>
ostream& operator<< (ostream& out, const vector<T>& v) {
  out << "[";
  size_t last = v.size() - 1;
  for(size_t i = 0; i < v.size(); ++i) {
    out << v[i];
    if (i != last)
      out << ", ";
  }
  out << "]";
  return out;
}

#endif /* utils_h */
