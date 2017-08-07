//
// Created by josh on 8/6/17.
//


#include <vector>

#include "jmt.h"
#include "Eigen-3.3/Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


/*
 * Initialize and calculate 1-D trajectory
 * start
 *   Starting state: (pos, vel, accel)
 * end
 *   Ending State: (pos, vel, accel)
 * time
 *   Time of transition
 */

void JMT::set(std::vector<double> start, std::vector<double> end, double time){
  T = time;
  T2 = T*T;
  T3 = T2*T;
  T4 = T3*T;
  T5 = T4*T;

  MatrixXd A = MatrixXd(3, 3);
  A <<  T3,    T4,    T5,
      3*T2,  4*T3,  5*T4,
      6*T , 12*T2, 20*T3;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T2),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

  // Initial condition
  trajectory_coeff = {start[0], start[1], .5*start[2]};
  // end condition
  for(int i = 0; i < C.size(); i++)
  {
    trajectory_coeff.push_back(C.data()[i]);
  }
};

double JMT::operator() (double t) const{
  // Calculate position at time, t
  double result = 0;
  for(int i=0; i < trajectory_coeff.size(); i++){
    result += trajectory_coeff[i] * std::pow(t, i);
  }

  return result;
};