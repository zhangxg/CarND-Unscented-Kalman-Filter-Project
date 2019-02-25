#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#define epsilon 0.001

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;



/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  is_initialized_ = false;

  time_us_ = 0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  // parameters to tune
  // std_a_ = 0.2;
  // std_yawdd_ = 0.2;
  // std_laspx_ = 0.015;
  // std_laspy_ = 0.015;
  // std_radr_ = 0.1;
  // std_radphi_ = 0.0175;
  // std_radrd_ = 0.1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  // std_yawdd_ = 30;

  std_a_ = 0.2;
  std_yawdd_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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
  if (!is_initialized_) {
      /**
      * Initialize the state state vector x_ with the first measurement.
      * Create the covariance matrix.
      */

      // first measurement
      x_ = VectorXd(5);

      // initial covariance matrix
      // P_ = MatrixXd(5, 5);
      // P_ <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      //         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
      //          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      //         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      //         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
      P_ = MatrixXd::Identity(5, 5);

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
          /**
          Convert radar from polar to cartesian coordinates and initialize state.
          */
          float ro = meas_package.raw_measurements_[0];
          float theta = meas_package.raw_measurements_[1];
          x_ << ro * cos(theta), ro * sin(theta), 0, 0, 0;
      } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
          /**
          Initialize state.
          */
          x_ <<
              meas_package.raw_measurements_[0],
              meas_package.raw_measurements_[0], 0, 0, 0;
      }

      // done initializing, no need to predict or update
      time_us_ = meas_package.timestamp_;

      // create vector for weights, the weights only be calculated once
      weights_ = VectorXd(2*n_aug_+1);
      lambda_ = 3 - n_aug_;
      weights_(0) = lambda_/(lambda_+n_aug_);
      for (int i = 1; i < n_aug_*2+1; ++i)
      {
        // cout << i << endl;
        weights_(i) = 0.5 / (lambda_ + n_aug_);
      }

      is_initialized_ = true;
      return;
  }

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  /*****************************************************************************
  *  Update
  ****************************************************************************/
  /**
  * Use the sensor type to perform the update step.
  * Update the state and covariance matrices.
  */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
    UpdateRadar(meas_package);
  } else {
      // Laser updates
    UpdateLidar(meas_package);
  }
  // print the output
   // cout << "x_ = " << x_ << endl;
   // cout << "P_ = " << P_ << endl;

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
  delta_t_ = delta_t;
  PredictMeanAndCovariance(x_, P_);
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
  MatrixXd H_ = MatrixXd(2, 5);
  H_ <<
      1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;
  MatrixXd R_ = MatrixXd(2, 2);
  R_ <<
      std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  x_ += K * y;
  P_ = (MatrixXd::Identity(5, 5) - K * H_) * P_;
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
  UpdateState(x_, P_, meas_package.raw_measurements_);
}

void UKF::GenerateSigmaPoints(MatrixXd& Xsig) {
  Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  // cout << "A" << endl << A << endl;

  Xsig.col(0) = x_;

  lambda_ = 3 - n_x_;   // needs to calculate here.
  float lambda_sqrt = sqrt(lambda_ + n_x_);

  for (int i = 0; i < n_x_; ++i)
  {
    Xsig.col(i+1) = x_ + lambda_sqrt * A.col(i);
    Xsig.col(i+n_x_+1) = x_ - A.col(i) * lambda_sqrt;
  }
}

void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_out) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug[5] = 0;
  x_aug[6] = 0;
  // cout << "x_aug" << endl << x_aug << endl;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);  // fill zero
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  // cout << "p_aug" << endl << P_aug << endl;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  // cout << "p_aug_sqrt" << endl << A << endl;

  //create augmented sigma points
  Xsig_out = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  lambda_ = 3 - n_aug_;
  float lambda_sqrt = sqrt(lambda_ + n_aug_);
  Xsig_out.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_out.col(i+1) = x_aug + lambda_sqrt * A.col(i);
    Xsig_out.col(i+n_aug_+1) = x_aug - lambda_sqrt * A.col(i);
  }
}

void UKF::SigmaPointPrediction(MatrixXd& Xsig_pred) {
  MatrixXd Xsig_aug;
  AugmentedSigmaPoints(Xsig_aug);

  //create matrix with predicted sigma points as columns
  Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t_) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t_) );
    }
    else {
        px_p = p_x + v*delta_t_*cos(yaw);
        py_p = p_y + v*delta_t_*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t_;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t_*delta_t_ * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t_*delta_t_ * sin(yaw);
    v_p = v_p + nu_a*delta_t_;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_*delta_t_;
    yawd_p = yawd_p + nu_yawdd*delta_t_;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance(VectorXd& x_pred, MatrixXd& P_pred) {
  // get the prediction points
  MatrixXd Xsig_pred;
  SigmaPointPrediction(Xsig_pred);
  // cout << "x_prediction: " << endl << Xsig_pred << endl;

  // // create vector for weights
  // weights_ = VectorXd(2*n_aug_+1);
  // lambda_ = 3 - n_aug_;
  // weights_(0) = lambda_/(lambda_+n_aug_);
  // for (int i = 1; i < n_aug_*2+1; ++i)
  // {
  //   // cout << i << endl;
  //   weights_(i) = 0.5 / (lambda_ + n_aug_);
  // }
  // VectorXd weights_ = calculateWeights(n_aug_);
  // calculateWeights(n_aug_, weights_);
  // cout << weights_ << endl;

  // cout << "weights1:" << endl << weights_ << endl;
  //create vector for predicted state
  x_pred = VectorXd(n_x_);
  x_pred.fill(0); // initialize the elements to zeros
  for (int i = 0; i < n_aug_*2+1; ++i)
  {
    x_pred += weights_(i) * Xsig_pred.col(i);
  }

  //create covariance matrix for prediction
  P_pred = MatrixXd(n_x_, n_x_);
  P_pred.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_; //x is the predicted mean
    x_diff(3) = normalizeAngle(x_diff(3));
    P_pred = P_pred + weights_(i) * x_diff * x_diff.transpose() ;
  }
  Xsig_pred_ = Xsig_pred;  // cache the values
}

void UKF::PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out, MatrixXd& zsig_out) {

  int n_z = 3;

  //define spreading parameter
  // lambda_ = 3 - n_aug_;

  // //set vector for weights
  // weights_ = VectorXd(2*n_aug_+1);
  // weights_(0) = lambda_/(lambda_ + n_aug_);
  // for (int i=1; i<2*n_aug_+1; i++) {
  //   // double weight = 0.5/(n_aug+lambda);
  //   weights_(i) = 0.5/(n_aug_+lambda_);
  // }

  //create example matrix with predicted sigma points
  // MatrixXd Xsig_pred; // = MatrixXd(n_x, 2 * n_aug + 1);
  // SigmaPointPrediction(Xsig_pred);
  MatrixXd Xsig_pred = Xsig_pred_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    // while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    // while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff(1) = normalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;
  //write result
  z_out = z_pred;
  S_out = S;
  zsig_out = Zsig;
}

void UKF::UpdateState(VectorXd& x_out, MatrixXd& P_out, VectorXd& z) {
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  // double lambda = 3 - n_aug_;
  // MatrixXd Xsig_pred;
  // SigmaPointPrediction(Xsig_pred);
  MatrixXd Xsig_pred = Xsig_pred_;
  // cout << "Xsig_pred: " << endl << Xsig_pred << endl;

  MatrixXd Zsig;
  VectorXd z_pred;
  MatrixXd S;

  PredictRadarMeasurement(z_pred, S, Zsig);
  // cout << "Zsig: " << endl << Zsig << endl;
  // cout << "z_pred: " << endl << z_pred << endl;
  // cout << "S: " << endl << S << endl;

  // cout << "weights_ " << endl << weights_ << endl;

  //create example vector for incoming radar measurement
  // VectorXd z = VectorXd(n_z);
  // z <<
  //     5.9214,
  //     0.2187,
  //     2.0062;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    // while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    // while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    z_diff(1) = normalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    // while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    // while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    x_diff(3) = normalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  z_diff(1) = normalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //write result
  x_out = x_;
  P_out = P_;
}

double UKF::normalizeAngle(double angle) {
  while (angle> M_PI) angle-=2.*M_PI;
  while (angle<-M_PI) angle+=2.*M_PI;
  return angle;
}

void UKF::calculateWeights(int numSigmaPoints, VectorXd& weights) {
  //define spreading parameter
  int lambda = 3 - numSigmaPoints;
  //set vector for weights
  // VectorXd weights = VectorXd(2*numSigmaPoints+1);
  weights = VectorXd(2*numSigmaPoints+1);
  weights(0) = lambda/(lambda + numSigmaPoints);
  for (int i=1; i<2*numSigmaPoints+1; i++) {
    weights(i) = 0.5/(lambda + numSigmaPoints);
  }
  // return weights;
}
