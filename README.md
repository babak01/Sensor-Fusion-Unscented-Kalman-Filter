# Sensor Fusion
## Radar and Lidar - Unscented Kalman Filter
***

<br>
<br>
This code is part of one of the projects in [Udacity sensor fusion nanodegree program](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313).
<br>

![video_ukf_1_mute_ed_resized_gif](https://user-images.githubusercontent.com/54375769/125867324-72494423-d17c-49c6-991d-4c6d7d364136.gif)

<br>
The goal of this project is to track three traffic cars using CTRV motion model. Although the cars have variable acceleration and turning rates the UKF is still able to track objects quite well with CTRV model.<br>
The accuracy is calculated by the RMSEover eeach time step and for each car. All the cars are considered in the RMSE value so if one car is particularly inaccurate it would affect the overall RMSE in this case. In the animation above the 4 RMSE values are X, Y, Vx, Vy from top to bottom.
<br>
<br>


<br>
<br>

The green car is the main ego car and other cars are blue. The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so we are only tracking along the X/Y axis.


<br>
<br>

## Installation and Run
***
The installation and run processes are the same as one that explained in [Udacity github repository](https://github.com/rayryeng/SFND_Lidar_Obstacle_Detection).
