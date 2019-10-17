# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Reflection
Steps involved in Path Planning
1) Since it is necessary to slow down / speed up the vehicle depending on traffic, the speed of the vehicle is considered as a variable (provided by "cur_vel" global variable in the code). It is initially assigned to 1 mph to avoid starting jumps in the velocity


2) The frontal lookahead distance is assigned to be 50 metres (provided by "horizon_front" global variable in the code). It is the frontal ahead distance for collision is predicted and for which trajectory is generated.


3) The next step is to predict collision for the ego vehicle with other vehicles based on its current position.
   It is done by the method "predictCollision". The basic logic involves looking out for the vehicles which are in the same lane as the ego vehicle and predict the s-distance for the future "num_timestamps_ahead". If the s-distance is greater than ego car's s-distance and within the "front lookahead" distance, then it is considered to be potential collision vehicle

4) If collision is expected, then reduce speed ("cur_vel") by a factor of 0.03 but not reducing below the minimum speed of 20 mph.
   If no chance of collision, then increase speed ("cur_vel") by a factor of 0.03 but not crossing the upper limit of 48 mph.
   
5) Trajectory Generation - It is done by the method "generateTrajectory".
   For trajectory generation, cubic spline interpolation is used given a set of points are known before hand. We take the previous points and add the future points (for a look ahead distance of 2 * horizon_front) which act as anchor points for spline interpolation. Before doing the spline interpolation, the anchor points are normalized by translation and rotation. Then we generate the trajectory points upto a look ahead distance of "horizon_front" but upto a maximum of 50 points including previous points. Note that the reversal of normalization is done for the generated points.
   
6) Lane Change State Machine:
	The finite state machine used here is a simple switch case depending on current lane. The current lane is decided by the variable "cur_lane".
    The lane change state machine will be executed only if there is a collision predicted and speed is lesser than 35 mph.
    The steps involved for lane change are as follows:
    a) For the given current lane, find out possible target lanes.
    b) For the first target lane, generate the new trajectory from current lane to target lane.
    c) Predict the collision at the mid point and end point of the new trajectory.
    d) If collision is predicted at either one point, current target lane is not suitable for lane change. Try the above steps for next target lane.
    e) If no collision is predicted at the target lane, do lane change.
    f) If no suitable target lane is identifed, target lane is same as current lane (do lane keeping)
    
7) Once target lane is identified, generate trajectory using cubic spline interpolation as described earlier and send to the simulator

###Area for Improvement:
Though the above logic works better with the car able to slow down, accelerate and change lanes, it is not able to predict the cutting in of other vehicles and does not react to it properly. I have noticed that cars from other lane cut in to ego vehicle lane suddenly though this behaviour is dangerous considering real world scenarios (I feel simulator should be upgraded to prevent such sudden cut-in incidents)

