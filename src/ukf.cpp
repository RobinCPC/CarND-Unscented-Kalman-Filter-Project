#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // not get first measuremnt
  is_initialized_ = false;

  // set state dimension
  n_x_ = 5;
  // set augmented dimension
  n_aug_ = 7;
  // define spreading parameter
  lambda_ = 3 - n_aug_;

  // initializes predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0);

  // initialize weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(1 / (2*(lambda_ + n_aug_)));
  weights_(0) = (lambda_ / (lambda_ + n_aug_));

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if(!is_initialized_){
    cout << "Start to Initialized!\n";
    if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      // initialize state vector
      x_ << meas_package.raw_measurements_[0],
      meas_package.raw_measurements_[1],
      0, 0, 0;
    }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      // covert radar from polar to cartesian coordinates
      float r = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float px = r * cos(phi);
      float py = -1 * r * sin(phi);
      x_ << px, py, 0, 0, 0;
    }

    // P covariance could initialize depend on sensor type
    // but now just use identity matrix
    P_ = MatrixXd::Identity(n_x_, n_x_);
    time_us_ = meas_package.timestamp_;
    cout << "x_ = \n" << x_ << endl;
    cout << "P_ = \n" << P_ << endl;
    // done initialize, no need to predict or update at first time
    is_initialized_ = true;
    cout << "Finish Init!\n";
    return;
  }

  double dt = (meas_package.timestamp_ -  time_us_) / 1000000.0; // dt - expressed in seconds
  time_us_ = meas_package.timestamp_;                            // update time stamp

  /*****************************************************************************
   *  prediction
   ****************************************************************************/
  cout << "prediction: dt=" << dt << endl;
  Prediction(dt);


  /*****************************************************************************
   *  update
   ****************************************************************************/
  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  cout << "update!\n";
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
    // Radar updates
    UpdateRadar(meas_package);
  }else{
    // Lidar updares
    UpdateLidar(meas_package);
  }

  return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
