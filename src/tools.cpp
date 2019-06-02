#include "tools.h"
#include <iostream>
#include <string>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::string;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /* TODO: Calculate the RMSE here. */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  //check the validity of the inputs
  try {
    if (estimations.size() == 0){
      throw string("The estimation vector size should not be zero!!");
    }
    else if (estimations.size() != ground_truth.size()){
      throw string("The estimation vector size should equal ground truth vector size!!");
    }
  }
  catch (string &e){
    cout << e << endl;
    return rmse;
  }

  // compute RMSE
  if (estimations.size() == 1){
    total_residual = VectorXd(4);
    total_residual << 0, 0, 0, 0;
  }

  int last_index = estimations.size() - 1;
  VectorXd residual = estimations[last_index] - ground_truth[last_index];
  residual = residual.array() * residual.array();
  total_residual += residual;
  rmse = total_residual / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /* TODO: Calculate a Jacobian here. */
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float s = px * px + py * py;

  // check division by zeros
  try {
    if (s<0.0001){
      throw string("Error! Division by Zero. Check x_state.");
    }
  }
  catch (string &e){
    cout << e << endl;
    return Hj;
  }
  // compute the Jacobian matrix
  Hj << px/sqrt(s), py/sqrt(s), 0, 0,
        -(py/s), px/s, 0, 0,
        py*(vx*py-vy*px)/pow(s, 1.5), px*(vy*px-vx*py)/pow(s, 1.5), px/sqrt(s), py/sqrt(s);
  return Hj;
}
