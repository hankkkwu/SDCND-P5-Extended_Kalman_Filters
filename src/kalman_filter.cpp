#include "kalman_filter.h"
#include <iostream>
#include <string>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::string;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /* TODO: predict the state */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /* TODO: update the state by using Kalman Filter equations */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  // new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /* TODO: update the state by using Extended Kalman Filter equations */
  float px = x_(0);   // predicted x-position
  float py = x_(1);   // predicted y-position
  float vx = x_(2);   // predicted speed-x
  float vy = x_(3);   // predicted speed-y
  VectorXd h_x(3);
  float s = px * px + py * py;
  // Check division by zeros
  try{
    if (s < 0.0001){
      throw string("Error! Division by Zero. Check x_state.");
    }
    h_x << sqrt(s),
           atan2(py, px),
           (px*vx + py*vy) / sqrt(s);
  }
  catch(string &e){
    cout << e << endl;
  }

  VectorXd y = z - h_x;   // y = 3x1 - 3x1 = 3x1

  // Normalizing Angles : make sure phi between -pi and pi.
  float y_phi = y(1);
  while (y_phi < -M_PI){
    y_phi += 2 * M_PI;
  }
  while (y_phi > M_PI){
    y_phi -= 2 * M_PI;
  }
  y(1) = y_phi;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;   // S = 3x4 * 4x4 * 4x3 + 3x3 = 3x3
  MatrixXd K = P_ * Ht * S.inverse();   // K = 4x4 * 4x3 * 3x3 = 4x3
  // new estimate
  x_ = x_ + (K * y);   // x_ = 4x1 + (4x3 * 3x1) = 4x1
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;   // P_ = (4x4 - (4x3 * 3x4)) * 4x4 = 4x4
}
