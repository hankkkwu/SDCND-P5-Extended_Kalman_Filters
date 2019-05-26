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

  for (int i = 0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse /= estimations.size();
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
    if (s==0){
      throw string("Error! Division by Zero. Check x_state.");
    }
  }
  catch (string &e){
    cout << e << endl;
    return Hj;
  }
  // compute the Jacobian matrix
  Hj << px/sqrt(s), py/sqrt(s), 0, 0,
        -py/s, px/s, 0, 0,
        py*(vx*py-vy*px)/pow(s, 3/2), px*(vy*px-vx*py)/pow(s, 3/2), px/sqrt(s), py/sqrt(s);
  return Hj;
}
