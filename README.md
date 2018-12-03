# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model
The MPC model used is as described in the course. The state relies on x, y, psi and speed of the vehicle (v). The actuator inputs are the steering angle (delta) and the throttle or brake (a). 

The update equations are given below;

1. x1 = x0 + v * cos(psi) * delay;
1. y1 = y0 + v * sin(psi) * delay;
1. psi1 = psi0 - v * delta * delay / Lf;
1. v1 = v0 + a * delay;
1. cte1 = cte0 + (v * sin(epsi) * delay);
1. epsi1 = epsi0 + (v * epsi)* delay / Lf;

Lf is constant for a given vechicle and needs to be empirically determined. 

## Timestep Length and Elapsed Duration (N & dt)
N is set to 10 and dt to 0.1. Other values tried were 20/0.05 and 5/0.2. The results were not as stable as the one with 10 and 0.1

## Polynomial Fitting and MPC Preprocessing
The points provided are in the map coordinates. In main.cpp (lines 108-116), these coordinates are converted to the car coordinate system based on the equations provided earlier in the lesson. Around 15 points are plotted (lines 192-195) to show the waypoints from the car perspective based on the polynomial fitting. A third degree polinomial is used. This preprocessing results in the provided points being on the origin with respect to the car (px = py = 0) and 0 psi.

The cross track error (cte) and orientation error (epsi) are computed as described in the lesson. 

The processed points from MPC is pushed into the return value and used to display the green line. The angle is normalized to be between -1 and +1 by dividing the `steer_value` by `deg2rad(25)`


## Model Predictive Control with Latency
The approach used to handle latency was to predict where the vehicle will be with the given latency and optimize the actuator values based on the future. Lines 137-151 in main.cpp handle this transformation in accordance with the update equations shown above. It should be noted that the `delta` returned from the telementry event needs to be converted to angles for this processing to work. Theorectically the approach should work with varying actuation delays with errors increasing with the delay.
