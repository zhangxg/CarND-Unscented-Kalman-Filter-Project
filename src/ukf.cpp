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
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ <<  5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // std_yawdd_ = 30;
  std_yawdd_ = 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  // std_radphi_ = 0.03;
  std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  // std_radrd_ = 0.3;
  std_radrd_ = 0.1;
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* State dimension
  // int n_x_ = 5;
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  // lambda_ = 3 - n_x_;

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

void UKF::GenerateSigmaPoints(MatrixXd& Xsig) {
  // the n_x_ has to be initialized in the header file, initialization in
  // cpp file can not be referenced.  
  // stupid mistake, I copied the code with "int n_x_ = 5", the int declares another value
  // not the one from the header file. 

  // cout << *this->n_aug_ << *this->n_x_ << endl;
  // cout << *n_aug_ << *n_x_ << endl;
  // cout << n_aug_ << "----" << n_x_ << endl;
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

  double delta_t = 0.1; //time diff in sec

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
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

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
  cout << "x_prediction: " << endl << Xsig_pred << endl;

  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  lambda_ = 3 - n_aug_;
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i < n_aug_*2+1; ++i)
  {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
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
    //angle normalization   // how to understand this? 
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_pred = P_pred + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out, MatrixXd& zsig_out) {

  int n_z = 3;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  
    // double weight = 0.5/(n_aug+lambda);
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred; // = MatrixXd(n_x, 2 * n_aug + 1);
  SigmaPointPrediction(Xsig_pred);
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
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

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

void UKF::PredictRadarMeasurement2(VectorXd& z_out, MatrixXd& S_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  // Xsig_pred <<
  //        5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  //          1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  //         2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  //        0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  //         0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  Xsig_pred << 
          5.93553,6.0625,5.92217,5.9415,5.92361,5.93516,5.93705,5.93553,5.80833,5.94481,5.92935,5.94553,5.93589,5.93401,5.93553,
          1.48939,1.44673,1.66483,1.49719,1.508,1.49001,1.49022,1.48939,1.5308,1.31288,1.48182,1.46967,1.48876,1.48855,1.48939,
          2.2049,2.28414,2.24557,2.29582,2.2049,2.2049,2.23954,2.2049,2.12566,2.16423,2.11398,2.2049,2.2049,2.17026,2.2049,
          0.53678,0.473388,0.678099,0.554557,0.643644,0.543372,0.53678,0.538512,0.600172,0.395461,0.519003,0.429916,0.530188,0.53678,0.535048,
          0.3528,0.299973,0.462123,0.376339,0.48417,0.418721,0.3528,0.387441,0.405627,0.243477,0.329261,0.22143,0.286879,0.3528,0.318159;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

//transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

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
  // VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  // MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  // //print result
  // std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  // std::cout << "S: " << std::endl << S << std::endl;

  //write result
  z_out = z_pred;
  S_out = S;
}

void UKF::UpdateState(VectorXd& x_out, MatrixXd& P_out) {
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug_;
  MatrixXd Xsig_pred;
  SigmaPointPrediction(Xsig_pred);

  cout << "Xsig_pred: " << endl << Xsig_pred << endl;

  MatrixXd Zsig;
  VectorXd z_pred;
  MatrixXd S;

  PredictRadarMeasurement(z_pred, S, Zsig);
  cout << "Zsig: " << endl << Zsig << endl;
  cout << "z_pred: " << endl << z_pred << endl;
  cout << "S: " << endl << S << endl;

  // cout << "weights_ " << endl << weights_ << endl;  

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
      5.9214,
      0.2187,
      2.0062;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //write result
  x_out = x_;
  P_out = P_;
}

void UKF::UpdateState2(VectorXd& x_out, MatrixXd& P_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  // cout << "weights " << endl << weights << endl;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  // Xsig_pred <<
  //       5.93553,6.0625,5.92217,5.9415,5.92361,5.93516,5.93705,5.93553,5.80833,5.94481,5.92935,5.94553,5.93589,5.93401,5.93553,
  //       1.48939,1.44673,1.66483,1.49719,1.508,1.49001,1.49022,1.48939,1.5308,1.31288,1.48182,1.46967,1.48876,1.48855,1.48939,
  //       2.2049,2.28414,2.24557,2.29582,2.2049,2.2049,2.23954,2.2049,2.12566,2.16423,2.11398,2.2049,2.2049,2.17026,2.2049,
  //       0.53678,0.473388,0.678099,0.554557,0.643644,0.543372,0.53678,0.538512,0.600172,0.395461,0.519003,0.429916,0.530188,0.53678,0.535048,
  //       0.3528,0.299973,0.462123,0.376339,0.48417,0.418721,0.3528,0.387441,0.405627,0.243477,0.329261,0.22143,0.286879,0.3528,0.318159;
  //create example vector for predicted state mean
  VectorXd x = VectorXd(n_x);
  x <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;
  // x = x_;

  //create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x,n_x);
  P <<
  0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;
  // P = P_;

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig <<
      6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
     0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
      2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;
  // Zsig << 
  //       6.11954,6.23274,6.15173,6.12724,6.11254,6.11934,6.12122,6.11954,6.00666,6.08806,6.11171,6.12448,6.11974,6.11787,6.11954,
  //       0.245851,0.234255,0.274046,0.246849,0.249279,0.245965,0.245923,0.245851,0.257693,0.217355,0.244896,0.242332,0.245737,0.24578,0.245851,
  //       2.11225,2.21914,2.06474,2.18799,2.03565,2.1081,2.14548,2.11115,2.00221,2.13,2.03506,2.16622,2.1163,2.07902,2.11334;
  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred <<
      6.12155,
     0.245993,
      2.10313;
  // z_pred << 
  //       6.11934,
  //       0.245834,
  //       2.10274;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;
  // S << 
  //     0.0946302,-0.000145123,0.00408742,
  //     -0.000145123,0.000624209,-0.000781362,
  //     0.00408742,-0.000781362,0.0180473;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
      5.9214,
      0.2187,
      2.0062;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  // //print result
  // std::cout << "Updated state x: " << std::endl << x << std::endl;
  // std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  x_out = x;
  P_out = P;

  /* expected result x:
     x =
 5.92276
 1.41823
 2.15593
0.489274
0.321338
    */

  /* expected result P:
     P =
  0.00361579 -0.000357881   0.00208316 -0.000937196  -0.00071727
-0.000357881   0.00539867   0.00156846   0.00455342   0.00358885
  0.00208316   0.00156846   0.00410651   0.00160333   0.00171811
-0.000937196   0.00455342   0.00160333   0.00652634   0.00669436
 -0.00071719   0.00358884   0.00171811   0.00669426   0.00881797
    */

}

