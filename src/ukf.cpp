#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 0.0, 0.0, 0.0, 0.0, 0.0;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ = MatrixXd::Identity(5,5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 15;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 15;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //Initializing state dimension
  n_x_ = 5;

  //Initializing augmented state dimension
  n_aug_ = 2 * n_x_ + 1;

  // Initializing sigma point spreading parameters
  lambda_ = 3 - n_x_;

  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  ///* time when the state is true, in us
  time_us_ = 0.0;

  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
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

	// Only perform this process is there is a known sensor type
	if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
		(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)) {

		// Initializing
		if (!is_initialized_) {
			if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
				// Assigning values to the state vector
				double px = meas_package.raw_measurements_(0);
				double py = meas_package.raw_measurements_(1);
				x_ << px, py, 0.0, 0.0, 0.0;
			}
			else { // Radar measurements
				double rho = meas_package.raw_measurements_(0);
				double phi = meas_package.raw_measurements_(1);
				double rho_dot = meas_package.raw_measurements_(2);

				double px = rho * cos(phi);
				double py = rho * sin(phi);
				double vx = rho_dot * cos(phi);
				double vy = rho_dot * sin(phi);
				double v = sqrt(vx*vx + vy * vy);
				x_ << px, py, v, 0.0, 0.0;
			}
			// Saving firsr time stamp in seconds
			time_us_ = meas_package.timestamp_;
			// done initializing, no need to predict or update
			is_initialized_ = true;
			return;
		}

		// Calculate delta t
		double dt = (meas_package.timestamp_ = time_us_) / 1000000.0;
		time_us_ = meas_package.timestamp_;
		// Prediction step
		Prediction(dt);
	}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.f
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	// Step 1. Generate sigma points
	// Create sigma point matrix
	VectorXd x_aug = VectorXd(n_aug_);
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented state variance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;
	
	// Creating sigma points

	





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

MatrixXd UKF::GenerateSigmaPoints(VectorXd x, MatrixXd P, double lambda, int n_sig) {
	int n = x.size();
	//Create sigma point matrix
	MatrixXd Xsig = MatrixXd(n, n_sig);

	// calculate square root of matrix P
	MatrixXd A = P.llt().matrixL();

	Xsig.col(0) = x;

	for (int i = 0; i < n; i++) {
		Xsig.col(i + 1) = x + sqrt(lambda + n)*A.col(i);
		Xsig.col(i + 1 + n) = x - sqrt(lambda + n)*A.col(i);
	}
	return Xsig;
}
