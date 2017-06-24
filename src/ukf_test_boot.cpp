
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

	MatrixXd Xsig = MatrixXd(11, 5);
	ukf.GenerateSigmaPoints(Xsig);
	cout << Xsig << endl;

	MatrixXd x_aug;
	ukf.AugmentedSigmaPoints(x_aug);

	MatrixXd x_pred;
	ukf.SigmaPointPrediction(x_aug);	

	VectorXd x_p = VectorXd(5);
  MatrixXd P_p = MatrixXd(5, 5);
  ukf.PredictMeanAndCovariance(x_p, P_p);

  // Vector3d v(1,2,3);
  // Vector3d w(0,1,2);
  // // cout << "Dot product: " << v.dot(w) << endl;
  // // double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  // // cout << "Dot product via a matrix product: " << dp << endl;
  // cout << "Cross product:\n" << v.cross(w) << endl;

	return 0;

}
