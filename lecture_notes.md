## unscented kalman filter. 
the cv model (constant velocity), this is for object tracking. 
ctrv model (constant turn rate and velocity magnitude)
ctra model (constant turn rate and acceleration)
csav model (constant steering angel and velocity)
cca model (constant curvature and acceleration)

in the ctrv (costant turning rate and velocity magnitude), what's the turning rate?  
understand the components: turning rate, velocity magnitude, acceleration, steering angel, curvature; 

#### unscented transformation
https://en.wikipedia.org/wiki/Unscented_transform

when doing the state transformation from time x to time x+1, by using the numerical analysis method, the result is not normal distributed; to continue using the normal distribution, we need to approximate the result with a normal distribution. this process is called the unscent transformation. 

the sigma points

the ukf augumentation, this is used to consider the noisy in the process
the ukf in human word:
basically the ukf is introduced to model the vehicle which makes turns, which is unlike the kalmand filter for vehicles driving straight. the ukf defines a model call ctrv, which assumes the turnning rate and velocity magnitude is constant, by introducing a 5 variable states vector. this vector is hard to solve mathimetically, so the sigma points were introduced to represent the model. the overall process of ukf is the same as ekf, which first make prediction, then update based on measurements. 

both the ekf and ukf are designed to tracking object, for sensor fusion. 



