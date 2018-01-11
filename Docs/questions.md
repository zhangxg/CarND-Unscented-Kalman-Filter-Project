
1. todo: redesign the kalman filter architecture, make the extended and unscent as plugins;
2. todo: organize the code of ukf, extract the common code, avoid redundant calculations;
3. todo: understand the math;
4. todo: 


the goal: 



### the initialization effect: the required value is: [.09, .10, .40, .30].
the bug:
```
} else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
          /**
          Initialize state.
          */
          x_ << 
              meas_package.raw_measurements_[0],
              meas_package.raw_measurements_[0], 0, 0, 0;
          // x_ << 
          //     meas_package.raw_measurements_[0],
          //     meas_package.raw_measurements_[1], 0, 0, 0;
      }
```

should be `meas_package.raw_measurements_[1]`, the result:

[0.096060 0.095921 0.510367 0.300825]   # bug
[0.202958 0.100559 0.640961 0.312729]   # fix

seems that the intialization state has huge impact to the unscented kalman filter. 

0.097826 0.0989891  0.509066  0.261663   # 0
0.0953907 0.0956115  0.504887  0.273861  # y/2 



### get to required RMSE
[the review](https://review.udacity.com/#!/reviews/613059)
"Only std_yawdd_ and std_a_ are allowed to be tuned. The rest should be left as default"
**update one**
```
// parameters to tune
// std_a_ = 0.2;
// std_yawdd_ = 0.2;
// std_laspx_ = 0.015;
// std_laspy_ = 0.015;
// std_radr_ = 0.1;
// std_radphi_ = 0.0175;
// std_radrd_ = 0.1;

=== changed to ===

// Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  // std_yawdd_ = 30;
  std_a_ = 0.2;
  std_yawdd_ = 0.2;

  std_laspx_ = 0.15;
  std_laspy_ = 0.15;
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;
```

**update two** change the p to identity matrix.
```
// initial covariance matrix
// P_ = MatrixXd(5, 5);
// P_ <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
//         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
//          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
//         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
//         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
P_ = MatrixXd::Identity(5, 5);
```

reached result: [0.0756751  0.101566  0.307558  0.244466] required is [.09, .10, .40, .30].
