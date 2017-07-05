
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::vector;




int main() {

	UKF ukf;

	cout << "hello, ukf" << endl;

  //  MatrixXd Xsig;
	// ukf.GenerateSigmaPoints(Xsig);
	// cout << endl << Xsig << endl;

	// MatrixXd x_aug;
	// ukf.AugmentedSigmaPoints(x_aug);
  // cout << endl << x_aug << endl;
	
 //  MatrixXd x_pred;
	// ukf.SigmaPointPrediction(x_pred);	
 //  cout << endl << x_pred << endl;

	// VectorXd x_p; // = VectorXd(5);
 //  MatrixXd P_p; // = MatrixXd(5, 5);
 //  ukf.PredictMeanAndCovariance(x_p, P_p);
 //  cout << "Predicted state" << endl;
 //  cout << x_p << endl;
 //  cout << "Predicted covariance matrix" << endl;
 //  cout << P_p << endl;

  // VectorXd z_out;
  // MatrixXd S_out;
  // MatrixXd zsig_out;
  // ukf.PredictRadarMeasurement(z_out, S_out, zsig_out);
  // //print result
  // cout << "z_pred1: " << endl << z_out << endl;
  // cout << "S1: " << endl << S_out << endl;
  // cout << "zsig_out: " << endl << zsig_out << endl;

  // cout << "=========\n\n";
  // VectorXd z_outt;
  // MatrixXd S_outt;
  // ukf.PredictRadarMeasurement2(z_outt, S_outt);
  // //print result
  // cout << "z_pred2: " << endl << z_outt << endl;
  // cout << "S2: " << endl << S_outt << endl;



  VectorXd x_out; //= VectorXd(5);
  MatrixXd P_out; // = MatrixXd(5, 5);
  ukf.UpdateState(x_out, P_out);
  cout << "x_out: " << endl << x_out << endl;
  cout << "P_out: " << endl << P_out << endl;

  VectorXd x_outt; // = VectorXd(5);
  MatrixXd P_outt; // = MatrixXd(5, 5);
  ukf.UpdateState2(x_outt, P_outt);
  cout << "x_out2: " << endl << x_outt << endl;
  cout << "P_out2: " << endl << P_outt << endl;

  // Vector3d v(1,2,3);
  // Vector3d w(0,1,2);
  // // cout << "Dot product: " << v.dot(w) << endl;
  // // double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  // // cout << "Dot product via a matrix product: " << dp << endl;
  // cout << "Cross product:\n" << v.cross(w) << endl;

	return 0;

}
