
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main() {

	UKF ukf;

	cout << "hello, ukf" << endl;

	MatrixXd Xsig = MatrixXd(11, 5);
	ukf.GenerateSigmaPoints(Xsig);
	cout << Xsig << endl;

	MatrixXd x_aug;
	ukf.AugmentedSigmaPoints(x_aug);

	return 0;

}
