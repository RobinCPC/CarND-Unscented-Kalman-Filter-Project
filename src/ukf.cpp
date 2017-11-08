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

  /*=== generate sigma points  ===*/
  /*
  lambda_ = 3 - n_x_;       // settingg spreading parameter for generating part.
  MatrixXd Xsig = MatrixXd(n_x_, 2*n_x_ + 1);   // create sigma point matrix.
  MatrixXd a = P_.llt().matrixL();  // calculate sqrt of p_

  // set first column of sigma point matrix
  Xsig.col(0) = x_;

  // calculate remaining of simga point (p_k|k part)
  for(int i=0; i<n_x_; ++i){
    Xsig.col(i+1)      = x_ + sqrt(lambda_ + n_x_) * a.col(i);
    Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_ + n_x_) * a.col(i);
  }
  */

  // augment sigma point with process noise
  lambda_ = 3 - n_aug_;                                 // setting spreading parameter for augmenting.
  VectorXd x_aug = VectorXd(n_aug_);                    // create augmented mean vector.
  MatrixXd p_aug = MatrixXd(n_aug_, n_aug_);            // create augmented covariance.
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); // create augmented sigma point matrix.

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  for (int i = n_x_; i < n_aug_; i++) {
    x_aug(i) = 0;
  }

  // create augmented covariance matrix
  p_aug.fill(0.0);
  p_aug.topLeftCorner(n_x_, n_x_) = P_;
  p_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) << std_a_*std_a_, 0,
    0, std_yawdd_*std_yawdd_;
  // create sqare root matrix
  MatrixXd l = p_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * l.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * l.col(i);
  }

  /*=== predict sigma points ===*/
  // initialize predict sigma points
  double dt_2 = delta_t * delta_t;
  Xsig_pred_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    // extract values for better readability
    double p_x      = Xsig_aug(0, i);
    double p_y      = Xsig_aug(1, i);
    double v        = Xsig_aug(2, i);
    double yaw      = Xsig_aug(3, i);
    double yawd     = Xsig_aug(4, i);
    double nu_a     = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if(fabs(yaw) > 1e-3){
      px_p = p_x + (v/yaw) * ( sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + (v/yaw) * (-cos(yaw + yawd*delta_t) + cos(yaw));
    }else{
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*dt_2 * cos(yaw);
    py_p = py_p + 0.5*nu_a*dt_2 * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*dt_2;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }


  /*=== Predict Mean and Covariance ===*/
  // weight setting on in constructor
  // predict state mean
  x_.fill(0.0);
  for(int i=0; i < n_x_; ++i){        // use row multiple to compute each x_(i)
    x_(i) = Xsig_pred_.row(i) * weights_;
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for(int i=0; i < 2*n_aug_+1; ++i){  // iterate over sigma points
    // get state difference
    MatrixXd del_x = Xsig_pred_.col(i) - x_;
    // angle normalization
    while(del_x(3) > M_PI){
      del_x(3) -= 2.*M_PI;
    }
    while(del_x(3) < -M_PI){
      del_x(3) += 2.*M_PI;
    }

    P_ += weights_(i) * del_x * del_x.transpose();
  }

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
