//
// Created by zhangxg on 17/6/17.
//
#include "FileLoader.h"

FileLoader::FileLoader(const string &filePath) {

/*
 * if i don't add `const` keyword in the parameter, the compiler complains
 * error: no matching conversion for functional-style cast from 'const char [49]' to 'FileLoader'

 * */
//FileLoader::FileLoader(string &filePath) {

    filePath_ = filePath;
    /*
     * if the parameter is added a `const`, then below statement is wrong,
     * is there a mechanism like java's `self`
     * */
//    filePath = filePath;
}

FileLoader::~FileLoader() {

}

void readGroundTruth(istringstream& iss, MeasurementPackage& meas_package){
    // read ground truth value
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    meas_package.ground_truth_ = VectorXd(4);
    meas_package.ground_truth_ << x_gt, y_gt, vx_gt, vy_gt;
}

vector<MeasurementPackage> FileLoader::loadData() {
    vector<MeasurementPackage> measurement_pack_list;

    // hardcoded input file with laser and radar measurements
    string in_file_name_ = filePath_;
    ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

    if (!in_file.is_open()) {
        cout << "Cannot open input file: " << in_file_name_ << endl;
    }

    string line;
    while(getline(in_file, line)){

        MeasurementPackage meas_package;

        istringstream iss(line);
        string sensor_type;
        iss >> sensor_type;	//reads first element from the current line
        long timestamp;
        if(sensor_type.compare("L") == 0){	//laser measurement
            //read measurements
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x,y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            readGroundTruth(iss, meas_package);

            measurement_pack_list.push_back(meas_package);

        }else if(sensor_type.compare("R") == 0){
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            readGroundTruth(iss, meas_package);
            measurement_pack_list.push_back(meas_package);
        }

        // WORKS.
//        // read ground truth value
//        float x_gt;
//        float y_gt;
//        float vx_gt;
//        float vy_gt;
//        iss >> x_gt;
//        iss >> y_gt;
//        iss >> vx_gt;
//        iss >> vy_gt;
//        meas_package.ground_truth_ << x_gt, y_gt, vx_gt, vy_gt;
//        measurement_pack_list.push_back(meas_package);
    }

    if(in_file.is_open()){
        in_file.close();
    }

    return measurement_pack_list;
}

//int main() {
//
//    vector<MeasurementPackage> measurement_pack_list;
////    FileLoader fileLoader("../data/obj_pose-laser-radar-synthetic-input.txt");
//    FileLoader fileLoader = FileLoader("../data/obj_pose-laser-radar-synthetic-input.txt");
//    measurement_pack_list = fileLoader.loadData();
//
//    size_t N = measurement_pack_list.size();
//    cout << N << endl;
//    for (size_t k = 0; k < N; ++k) {	//start filtering from the second frame (the speed is unknown in the first frame)
////        tracking.ProcessMeasurement(measurement_pack_list[k]);
////        cout << measurement_pack_list[k].timestamp_ << endl;
//    }
//    return 0;
//}
