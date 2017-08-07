//
// Created by josh on 7/29/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


class Vehicle {
public:
  Vehicle(double id, double x, double y, double vx, double vy, double s, double d){
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->v = std::sqrt(vx*vx + vy*vy);
  }

  // Trivial destructor
  ~Vehicle() = default;

  // Public Data
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double v;
  double s;
  double d;

  double predict(double t){
    // assuming s'', d' and d'' = 0
    return s + v*t;
  }
};


#endif //PATH_PLANNING_VEHICLE_H
