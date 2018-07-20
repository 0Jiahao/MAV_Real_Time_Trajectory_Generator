# MAV_Trajectory_Planner

A simulation setup for simple MAV trajectory planner.

## Basic Structure
Structure contains the position, the orientation and the speed on body frame.
```Matlab
obj.position = [x,y,z];  
obj.angle = [yaw,pitch,roll];
obj.speed = [vx,vy,vz]; % body frame
```
## Simple Dynamic Model
The discrete time dynamic model of MAV.
```Matlab
ts = 0.05; %  sampling time is 0.05 s
u = [throttle,yaw,pitch,roll]; % control input
obj = dynamic_mav(obj,u,ts);
```
## Minimum Snap Trajectory
Generate a trajectory by minimizing the snap over time.
```Matlab
> [waypoints,path_c] = path_planner(obj,tgt,time)  
```
## Find The Optimal Time Interval of The Trajectory
Finds the shortest trajectory with the constraint that the pitch angle should less than 20 degrees.
```Matlab
>[waypoints,path_c,opt_time] = time_optimal_path_planner(mav,tgt,ts);
```
![](images/find_best_time_interval.png)
