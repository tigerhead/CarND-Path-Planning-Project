# CarND-Path-Planning-Project
Udacity Self-Driven Car Nanodegree program Term3 Project 1 

---

# Path Planing Implementation Reflection

## Introduction
Path planning is an interesting and challenging problem. The goal here is to find a safe, comfortable and efficient path through a set of dynamic maneuverable objects to reach our goal. Generally, path planning including path searching, prediction, behavior planning and trajectory generation. In this project, a path planning model on enclosed highway is implemented. So it simplify the path planning problem a lot. Since the highway map is already known, the path searching and prediction are easy. The car should just follow the highway based on the given way points. Trajectory generation and behavior planning will be main challenge in this project. 

## Trajectory generation

There are serval motion planning algorithms: hybrid A*, polynomial trajectory generation. Since this project is for highway path planning, uncertainty is low, so using polynomial trajectory generation will be a good choice. To make polynomial calculation easier, as suggest in the project page, a C++ tool spline is leveraged in this model. 3 future waypoints are predicted to provide a rough path. And each point co-ordinates are converted from global co-ordinates to car local co-ordinates so that the spline always starts at (0, 0). Following is code snippet for Tractory generation

```C++
vector<double> ptsx;

vector<double> ptsy;
//define rough path in Frenet coordinates, convert to XY



vector<double> next_wp0 = getXY(car_s+ distance_interval, 4* current_lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

vector<double> next_wp1 = getXY(car_s+ distance_interval * 2, 4* current_lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

vector<double> next_wp2 = getXY(car_s+ distance_interval * 3, 4* current_lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);


ptsx.push_back(next_wp0[0]);

ptsx.push_back(next_wp1[0]);

ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);

ptsy.push_back(next_wp1[1]);

ptsy.push_back(next_wp2[1]);

//convert global XY coordinates to car's reference frame to allow for polynomial smoothing

for (int i=0; i<ptsx.size(); i++)

{
double shift_x = ptsx[i]-ref_x;

double shift_y = ptsy[i]-ref_y;

ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));

ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
}



//create spline for smooth path

tk::spline sp;
sp.set_points(ptsx,ptsy);
```


Then based on this spline, more way points can be added to the path between start and target position. Before adding way points, local co-ordinates should be converted to global co-ordinates.

```C++
for(int i = 0; i <= 50-prev_points; i++)

{

double N = target_dist/(.02*ref_speed/2.24);

double x_point = x_add_on+target_x/N;

double y_point = sp(x_point);

x_add_on = x_point;

double x_ref = x_point;

double y_ref = y_point;

//convert points back to global coordinates

x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));

y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));



x_point += ref_x;

y_point += ref_y;


next_x_vals.push_back(x_point);

next_y_vals.push_back(y_point);

}

```


## Behavior planning

In high way driving, car can have following possible states: keep lane, left lane change, right lane change, prepare left lane change, prepare right lane change. I put an assumption in my model that left lane is faster lane, so left lane change is preferred whenever it is safe to make left lane change. In this case, I only use three states in my model: model keep lane left lane change, right lane change. 

I. Keep lane: 

Using fusion sensor data, other cars’s positions can be calculated and predicted.  If there is another car is ahead of target car, the distance between those two will be calculated. If the distance is less than safe driving distance, target car will reduce speed, otherwise car will accelerate if the speed is not over speed limit.  And there are 3 requirements in this project:
  *speed limit is 50 mph
  *total acceleration should be less than 10 m/s^2
  *jerk is less than 50 m/s^3.

I set starting speed as 0 and speed change is not greater than 1 m/s^2 (0.224) and set max speed to 49.5 mph to make sure it won’t violate speed limit, max speed acceleration and jerk rules. 

II. Lane change

When car is too close to the car ahead and has to reduce speed, it will try to change lane. Fusion sensor data are also used to predict if there are other cars in the lane that the target car tries to change to. The target car make lane change only if other cars are out of safe distance range. Following method is implemented to check if it is to make lane change.

```C++
    bool safe_to_change_lane(auto sensor_fusion, int target_lane, double car_s, int prev_points ){
    
    bool safe_to_change = true;
    
   for(int i=0; i<sensor_fusion.size(); i++)
  {

    //determine if car is directly ahead

   double check_car_d = sensor_fusion[i][6];

    // Car is in in target lane   
    if (check_car_d < (2 + 4 * target_lane + 2 ) && check_car_d > (2 + 4 * target_lane - 2)) 
    {

       double vx = sensor_fusion[i][3];

       double vy = sensor_fusion[i][4];

      double check_car_speed = sqrt(vx*vx + vy*vy);

      double check_car_s = sensor_fusion[i][5];

     check_car_s += (double)prev_points*.02*check_car_speed;

     double check_distance = check_car_s-car_s;
     
   if(check_distance > -15 && check_distance <30){
   
       safe_to_change = false;

       break;

    }

   }

   }

    return safe_to_change;
   }   
   
```
In my model, left lane change is preferred, so model check if left lance change is safe or not first. If it is possible and safe to make left lane change, the car will make left lane change. Otherwise, check if right lane change is safe. If it is possible and safe to make right lane change, the car will make right lane change.

##Emergency handling

In simulation test, I found sometimes, another car could cut in and collision would have happened in this case. To prevent collision happened in this emergency case, I added a full_break bool parameter, if other car ahead is really close less than 6m, it will be true. In this case, if the speed is greater than 10m/s(22.4 mph), it will reduced by 10m/s(22.4 mph) otherwise reduce speed to 0.1 m/s.

##Reflection

My model can drive the car though one loop without incident at most of time. However sometimes, if too many cars around, collision could happen. The behavior planning algorithm can be improved to deal with that issue. 5 state machine model can be used, and a better cost function can be implemented to decide which state the car should take.  

## Recorded driving video in simulator

Here is the link:
https://youtu.be/L8FJ1eNeK_w






