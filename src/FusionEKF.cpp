#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
  // Reset flag for initialization
  is_initialized_ = false;

  // Initialize timestamp
  previous_timestamp_ = 0;

  // Initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Initialize laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;
  
  // Initialize to 0 for Jacobian matrix
  Hj_ << 0, 0, 0, 0,
  		 0, 0, 0, 0,
  		 0, 0, 0, 0;  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // Using Kalman Filter class' Init() function, set _in variables
	
    cout << "EKF: " << endl;
    
    // first measurement
    VectorXd x_init = VectorXd(4);
    x_init << 1, 1, 1, 1;
    
    // State covariance matrix P
    MatrixXd P_init = MatrixXd(4, 4);
    P_init << 	1, 0, 0, 0,
             	0, 1, 0, 0,
             	0, 0, 1000, 0,
             	0, 0, 0, 1000;
  
  	// Initial transition matrix F
    MatrixXd F_init = MatrixXd(4, 4);
    F_init << 	1, 0, 1, 0,
             	0, 1, 0, 1,
             	0, 0, 1, 0,
             	0, 0, 0, 1;
    
    // Initial process covariance matrix Q
    MatrixXd Q_init = MatrixXd(4, 4);
    Q_init << 	1, 0, 1, 0,
             	0, 1, 0, 1,
             	1, 0, 1, 0,
             	0, 1, 0, 1;
    
    // Pre-initialize matrices
    MatrixXd H_init;
    MatrixXd R_init;
    
    // Set timestamp to have a reference for the first prediction
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // H_init and R_init depends on the sensor type
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      H_init = Hj_;
      R_init = R_radar_;
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      x_init << rho * cos(phi), rho * sin(phi), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state
      H_init = H_laser_;
      R_init = R_laser_;
      x_init << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

 	// Use Init() to initialize all variables
    ekf_.Init(x_init, P_init, F_init, H_init, R_init, Q_init);
    
    // Set flag for completed initialization
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  // Pre-calculate dt powers
  float dt_2 = pow(dt, 2);
  float dt_3 = pow(dt, 3);
  float dt_4 = pow(dt, 4);
  
  // Initialize measurement noises for x and y
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  
  // Update the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
