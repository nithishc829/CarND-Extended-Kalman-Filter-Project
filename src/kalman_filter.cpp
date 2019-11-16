#include "kalman_filter.h"
#include "tools.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

VectorXd convert_to_polar(VectorXd &x) {
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);
  VectorXd res(3);
  res(0) = sqrtf(px * px + py * py);
  res(1) = atan2(py, px);
  res(2) = (px * vx + py * vy) / res(0);
  return res;
}

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
  /**
   * TODO: predict the state
   */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  /*
    std::cout << " cart ";
    print_vec(x_);
    std::cout << " polar ";
    
  */

  VectorXd z_pred = convert_to_polar(x_);

  // print_vec(z_pred);

  VectorXd y = z - z_pred;
  // angle difference should be taken care of
  float theta_diff = z(1) - z_pred(1);
  y(1) = atan2(sinf(theta_diff), cosf(theta_diff));

  // std::cout << " z ";
  // print_vec(z);
  // std::cout << " zpr ";
  // print_vec(z_pred);

  // std::cout << " y ";
  // print_vec(y);

  // std::cout << "\n";

  MatrixXd Ht = H_.transpose();

  MatrixXd S = H_ * P_ * Ht + R_;

  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;

  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);

  long x_size = x_.size();

  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}