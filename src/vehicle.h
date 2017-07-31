//
// Created by josh on 7/29/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


class Vehicle {
public:
  Vehicle(double id, double x, double y, double vx, double vy, double s, double d){
    id_ = id;
    x_ = x;
    y_ = y;
    vx_ = vx;
    vy_ = vy;
    s_ = s;
    d_ = d;

  }

  // Trivial destructor
  ~Vehicle() = default;

  // Public Data
  double id_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double s_;
  double d_;
};


#endif //PATH_PLANNING_VEHICLE_H
