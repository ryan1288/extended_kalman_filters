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
  // Predict the state 
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Apply Kalman Filter equations to obtain new state estimates
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // New estimate
  x_ = x_ + (K * y);
  long x_length = x_.size();
  MatrixXd I = MatrixXd::Identity(x_length, x_length);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Convert predicted state into cartesian coordinates
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // Calculate the corresponding h(x) output
  VectorXd h_x(3);
  float sum_squares = pow(px,2) + pow(py,2);
  float rho = pow(sum_squares, 0.5);
  float phi = atan2(py, px);
  float rho_dot;
  if (fabs(rho) < 0.0001)
    rho_dot = 0;
  else
    rho_dot = (px*vx + py*vy)/rho;
  
  h_x << rho, phi, rho_dot;
  
  // Apply Kalman Filter equations to obtain new state estimates
  VectorXd y = z - h_x;
  
  // Normalize angle to [-pi,pi]
  if (y[1] > M_PI)
    y[1] -= 2*M_PI;
  else if (y[1] < -M_PI)
    y[1] += 2*M_PI;
  
  // Calculate remaining matrices
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // New estimate
  x_ = x_ + (K * y);
  long x_length = x_.size();
  MatrixXd I = MatrixXd::Identity(x_length, x_length);
  P_ = (I - K * H_) * P_;
}
