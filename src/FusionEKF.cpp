#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  // transition
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  bool using_radar = true;
  bool using_laser = true;  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.Q_ = MatrixXd(4, 4);
    previous_timestamp_ = measurement_pack.timestamp_;
    if (using_radar && (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "First measurement is RADAR, set location & velocity " << endl;     
      float range = measurement_pack.raw_measurements_[0];
      float heading = measurement_pack.raw_measurements_[1];
      float velocity = measurement_pack.raw_measurements_[2];
      
      float px = range * std::cos(heading);
      float py = range * std::sin(heading);
      float vx = velocity * std::cos(heading);
      float vy = velocity * std::sin(heading);

      // initilize
      ekf_.x_ << px, py, vx, vy;
    }
    else if (using_laser && (measurement_pack.sensor_type_ == MeasurementPackage::LASER)) 
    {
      cout << "First measurement is RADAR, set location, velocity set to zero " << endl;     
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];

      // initilize, speed unknown
      ekf_.x_ << px, py, 0, 0;
    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "EKF Initialized " << endl;

    return;
  }

  if (!using_radar && (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)) {
    return;
  }
  else if (!using_laser && (measurement_pack.sensor_type_ == MeasurementPackage::LASER)) {
    return;
  }


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	//1. Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	//2. Set the process covariance matrix Q
  float noise_ax = 9;
  float noise_ay = 9;
	MatrixXd Qv = MatrixXd(2, 2);
	Qv << noise_ax, 0,
	      0, noise_ay;

	MatrixXd G = MatrixXd(4, 2);
	float dt2 = 0.5 * dt * dt;
	G << dt2, 0,
	     0, dt2,
	     dt, 0, 
	     0, dt;

	ekf_.Q_ = G * Qv * G.transpose();
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (using_radar && (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)) 
  {
    // Radar updates
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    bool jacobian_empty = Hj_.isZero(0);
    if (jacobian_empty) { // Jacobian (and Hessian) holds no information - no update
      return;
    }

    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } 
  else if (using_laser && (measurement_pack.sensor_type_ == MeasurementPackage::LASER))
  {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output

  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
