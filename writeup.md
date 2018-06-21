[//]: # (Image References)
[image1]: ./Screenshot_Best.PNG



## Path Planning Project
This is Project 1 of Udacity Self-Driving Car Nanodegree program Term3. 
In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data using C++.

The project was created with the Udacity [Starter Code](https://github.com/udacity/CarND-Path-Planning-Project).

## [Rubric](https://review.udacity.com/#!/rubrics/1020/view) Points

## Content of this repo
- `scr` a directory with the project code:
  - `main.cpp` - reads in data, define cost functions depending on the sensor fusion Data of surrounding traffic and creates a smooth trajectory
 - `spline.h` - a function for using splines to create a smooth trajectory
 - `json.hpp`
- `data`  a Directory with input files, provided by Udacity
- `CMakeLists.txt`
- `README.md` Description of the Project by Udacity
- `writeup.md` for submission of this Project
#####
## Result
#### Prediction
With the  solution from the Q&A-video provided by Udacity, the trajectory generation was done with creating smooth splines by help of spline.h. 
Controlling the veloctiy of the car, depending on existing vehicles in front of the car, was performed with several steps. First tthe future location of the detected vehicles on the same lane have been predicted by a linear motion model (main.cpp, line 348)

```
// predict the vehicle location
double pred_car_s_sf = car_s_sf;
pred_car_s_sf += ((double)prev_size * 0.02 * car_speed_sf);
```
#####
#### Speed Adjustment
Then the closest vehicle in front of the car has to be extracted out of the sensor fusion data. With this information, the speed adjustment was done depending on the predicted distance to the closest vehicle  (main.cpp, line 361).


```
// set the target_speed close to the velocity of vehicle in front
                  if ( pred_dist_delta <= range_avoid_collision)
                  {
                    target_speed =  car_speed_sf * 0.5 * ms_to_mph;
                    too_close = true;
                  }

                  // avoid collision, target speed below speed of vehicle in front
                  else if (pred_dist_delta <= range_adapt_speed )
                  {
                    target_speed =  car_speed_sf * ms_to_mph;
                    too_close = true;
                  }

                  // set target speed to speed limit
                  else
                  {
                    target_speed = speed_limit;
                    too_close = false;
                  }
```
#####
#### Cost Function
There are 3 cost functions for the situation following a vehicle which result in a sum cost for keeping the lane (main.cpp, line 382). The first cost function depends on the relation target speed and speed limit penalizing low speed.

```
Speed_cost_KL = (speed_limit - (target_speed))/(speed_limit);
```
The cost function to penalize collision situations is a binary reaction depending on the distance value

```
Collision_cost_KL = 0;
                    if (car_dist_sf < (3.0 * verhicle_radius))
                    {
                      Collision_cost_KL = 1;
                    }
                    else
                    {
                      Collision_cost_KL = 0;
                    }
```

The last cost function penalizes situations, where vehicles are in the near surrounding. This part helps finding trajectories on lanes with free space.

```
Buffer_cost_KL = 2.0 / (1 + exp(-(2.0 * verhicle_radius / car_dist_sf))) - 1.0;
```
#####
The cost function for  a path on the right or left lane are identical for speed and collision. The buffer cost takes a speed ratio into account. 
To avoid collisions during lane change,  the total cost  is increased to high values recognizing vehicles in specific situations. These situations are (main.cpp, line 481):

- the predicted s-distance or the total distance of the car and the nearest vehicle is smaller than the adjustable range value `range_avoid_collision_lc` to avoid collision
- or the  predicted s-distance of the nearest vehicle with higher speed as the car is less than a third of `range_avoid_collision_lc` and greater than twice `range_avoid_collision_lc`

With this cost increase, a  lane change is only performed, when there is no chance of collision.
#####
#### Lane Change
To avoid high frequency lane changes, a minium distance of 150m is necessary to get the permission for the next lane Change.
The decision of a lane change is done with the logic below by comparison of total cost for "Keep Lane",. "Lane Change Left" and "Lane Change Right" (main.cpp, line 605).

```
// Find trajectory with min cost for lane change
            if (too_close == true && lane_change_allowed == true)
            {
              if ((sum_cost_LCL < sum_cost_KL) && (sum_cost_LCL <= sum_cost_LCR))
              {
                lane -= 1;
                car_s_last_lc = car_s;
                target_speed = target_speed_LCL;
              }
              if ((sum_cost_LCR < sum_cost_KL) && (sum_cost_LCR < sum_cost_LCL))
              {
                lane += 1;
                car_s_last_lc = car_s;
                target_speed = target_speed_LCR;
              }
            }
```

#####
#### Summary
The car is able to drive at least 4.32 miles without incident
![alt text][image1] 




