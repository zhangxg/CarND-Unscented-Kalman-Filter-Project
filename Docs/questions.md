
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
