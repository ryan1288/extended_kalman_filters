#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
  // Initialize rmse vector with zeros
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  
  // Check the validity of the following inputs:
  //  * the estimation vector size is not zero
  //  * the estimation vector size = ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    
    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse = rmse/estimations.size(); // Calculate the mean
  rmse = rmse.array().sqrt(); // Calculate the squared root
  return rmse; // return the result
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Initialize Jacobian matrix
  MatrixXd Hj(3,4);
  
  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = pow(px,2) + pow(py,2);
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // Check division by zero
  if (fabs(c1) < 0.0001)
  {
    std::cout << "CalculateJacobian() - Error - Division by Zero" << std::endl;  
  }
  else
  {
    Hj << px/c2, py/c2, 0, 0,
            -py/c1, px/c1, 0, 0,
            py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3,
            px/c2, py/c2;
  }

  return Hj;
}