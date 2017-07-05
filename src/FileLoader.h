//
// Created by zhangxg on 17/6/17.
//

#ifndef EXTENDEDKF_DATA_LOADING_H
#define EXTENDEDKF_DATA_LOADING_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class FileLoader {
public:
    /**
     * Constructors
     * */
//    FileLoader(string & filePath);
    FileLoader(const string & filePath);

    /**
     * Destructor
     * */
    virtual ~FileLoader();

    /**
     * the loaded data
     * */
    vector<MeasurementPackage> loadData();

private:
    string filePath_;
};

#endif //EXTENDEDKF_DATA_LOADING_H
