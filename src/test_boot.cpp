//
// Created by zhangxg on 17/6/17.
//

#include "tools.h"
#include "Eigen/Dense"
#include "iostream"
#include "FileLoader.h"
#include "ukf.h"

using namespace std;

int main() {
    vector<MeasurementPackage> measurement_pack_list;

    FileLoader fileLoader = FileLoader("../data/obj_pose-laser-radar-synthetic-input.txt");
//    FileLoader fileLoader = FileLoader("../data/sample-laser-radar-measurement-data-1.txt");
//    FileLoader fileLoader = FileLoader("../data/sample-laser-radar-measurement-data-2.txt");
    measurement_pack_list = fileLoader.loadData();

    size_t N = measurement_pack_list.size();
    cout << N << endl;

    UKF ukf;
    Tools tools;

    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    for (size_t k = 0; k < N; ++k) {	//start filtering from the second frame (the speed is unknown in the first frame)
        cout << k << "====" << endl;
//        cout << measurement_pack_list[k].ground_truth_ << endl;
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        VectorXd estimate(4);

        double p_x = ukf.x_(0);
        double p_y = ukf.x_(1);
        double v1  = ukf.x_(2) * cos(ukf.x_(3));  
        double v2 = ukf.x_(3) * sin(ukf.x_(3));

        estimate(0) = p_x;
        estimate(1) = p_y;
        estimate(2) = v1;
        estimate(3) = v2;
        estimations.push_back(estimate);

        ground_truth.push_back(measurement_pack_list[k].ground_truth_);

        VectorXd rmse(4);
        rmse = tools.CalculateRMSE(estimations, ground_truth);
        cout << "rmse=\n" << rmse << endl;

    }
    return 0;
}
