#include "kalman_filter.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  // new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /* TODO: update the state by using Extended Kalman Filter equations */
  double px = x_(0);   // predicted position - x
  double py = x_(1);   // predicted position - y
  double vx = x_(2);   // predicted speed - x
  double vy = x_(3);   // predicted speed - y
  VectorXd h_x(3, 1);
  double s = px * px + py * py;
  // check division by zeros
  try {
    if (s==0){
      throw string("Error! Division by Zero. Check x_state.");
    }
    h_x << sqrt(s),
           atan2(py, px),
           (px*vx + py*vy) / sqrt(s);
  }
  catch (string &e){
    cout << e << endl;
  }

  VectorXd y = z - h_x;   // y = 3x1 - 3x1 = 3x1
  // Normalizing Angles
  double phi = y(1);
  while (phi < -M_PI){
    phi += M_PI;
  }
  while (phi > M_PI){
    phi -= M_PI;
  }
  y(1) = phi;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;   // S = 3x4 * 4x4 * 4x3 + 3x3 = 3x3
  MatrixXd K = P_ * H_.transpose() * S.inverse();   // K = 4x4 * 4x3 * 3x3 = 4x3
  // new estimate
  x_ = x_ + (K * y);   // x_ = 4x1 + (4x3 * 3x1) = 4x1
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;   // P_ = (4x4 - (4x3 * 3x4)) * 4x4 = 4x4
}
