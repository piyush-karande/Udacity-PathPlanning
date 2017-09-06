# Path Planning #
The goal of this project was navigate a car on a three lane highway in a simulated environment. Specifically:

1. Car should follow a smooth trajectory without violating maximum velocity, acceleration, and jerk restrictions.
2. Car should not crash other cars on the 3 lane highway.
3. Car should stay inside its lane and change lanes to avoid getting stuck behind a slow car, while following all safety rules.
4. Car should use the localization, sensor fusion, and map data to perform the above mentioned tasks.

## Rubric Points ##

### Compilation ###

The code was modified from the starter provided by [Udacity on github](https://github.com/udacity/CarND-Path-Planning-Project). The code does not include any additional files and should compile correctly.

### Valid Trajectories ###

**1. The car should drive at least 4.32 miles without incident.**

The code was tested several times with the [term 3 simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) provided by Udacity. The tests were conducted by running the simulator continuously for 20-25 mins and also with 1-2 lap runs multiple times. The controlled vehicle was observed to drive the highway without any warnings and incidents except in rare cases. The only times an incident was observed were when there were "rogue" vehicles in the vicinity. This could be extreme braking or very quick changes in lanes by other vehicles.

**2. The car drives according to the speed limit.**

The maximum velocity of the car is set to 49.50mph. This helps the car drive below the speed limit at all times. The car is programmed to slow down to a velocity close to the vehicle it is following when the vehicle is within 30m. If the car is behind a slow moving vehicle, it is instructed to plan and change lanes while following all the rules. This helps to maintain the average velocity of the car as close to the speed limit as possible, while still following all the rules.

**3. Max Acceleration and Jerk are not Exceeded.**

In the speed control section of the code, the acceleration is set to either 5 or 7.5 m/sec<sup>2</sup>. This prevents the car from exceeding the acceleration and jerk. The only times the car might accelerate faster than the set value is while turning. However, the acceleration is set much below the max limit of 10m/sec<sup>2</sup> and thus never exceeds this limit.

**4. Car does not have collisions.**

The car is set to slow down when it sees a another vehicle in its lane. It is also programmed to execute lane changes while there are no vehicles too close to the car in the lane it is planning to move to. The implementation of these two behaviors ensures that the car does not collide with other vehicles.

**5. The car stays in its lane, except for the time between changing lanes.**

The trajectory is created with the intention of keeping the car in the lane while smoothly following the curves in the path. Lane changes are executed by using a spline fit and the car doesn't stay between lanes for very long.

**6. The car is able to change lanes.**

Lane changes are executed when desired by setting the d value of trajectory to the center of the desired lane. By using a combination of spline fit and residual points from previous path from the simulator, the car is able to change lanes smoothly. The controlled car makes a lane change plan when it behind a slower vehicle and its speed has reduced below 45mph. It stays behind the car until it can safely move to an adjacent lane or if the car speeds up and the controlled car is maintaining a speed above 49mph.

### Reflections ###

My goal behind implementing a solution for this project was to keep it as simple as possible. To meet the goals of the project three main things needed to be implemented; following a smooth trajectory, staying within the bounds of speed/acceleration/jerk and navigating through the traffic safely and efficiently. As the controlled vehicle is driving on a highway with a set speed limit and only three lanes, the car should be able to safely perform the task by following few simple behavior strategies (described in 3.)

**1. Smooth trajectories.**

The path planning uses the spline tool for generating smooth trajectories. It uses five "anchor points" for the spline fit. The first two points are either cars current and previous position (at the end of previous path) or if available the last two points of the previous trajectory. The last three points are 30m, 60m and, 90m ahead in frenet space. After fixing the anchor points, the program uses any previously available points and appends them with new points to produce a 50 point trajectory at every iteration. The new points are calculated at 30m horizon. A transformation to the car's local coordinate system is used to make this calculation simpler. This peace of code is almost identical to the one used in walkthrough video and is between lines 508 and 612 in main.cpp.

**2. Speed/Acceleration/Jerk control**

The car starts out with a reference velocity of zero and slowly (5m/sec<sup>2</sup>) ramps up. The piece of code that controls this is in main.cpp and shown below:

```cpp
// Setting min_vel and acceleration of car when another car is too close in same lane
double min_vel = front_car_vel - 2.0;
double acc = 0.224; // .224mph every 20msecs is approximately 5m/s^2

// Higher deccelaration and 0 min_vel when vehicle very close
if ((curr_front_car_dist < double(FRONT_DIST) / 2) || count > 1) {
  min_vel = 0.0;
  acc = 0.336;  // .336mph every 20msecs is approximately 7.5m/s^2
}

// Speed control if too close to a car
if (too_close && (ref_vel > min_vel)) {
  ref_vel -= acc; // Decrease ref_vel
  if (count > 1) {
    cout << "Num cars too close: " << count << endl;
  }

  // Start planning for lane change if speed below 45mph
  if (ref_vel < 45) { check_lanes = true; }
} else if (ref_vel < 49.50) {
  ref_vel += acc; // If no car is too close maintain speed right below 50mph
} else if (ref_vel > 49.0) { check_lanes = false; } // Dont check lanes if velocity is > 49mph


```

From the code above its seen that the car ramps up to 49.5mph and stays there until there is any car too close in front of it. The controlled car slows down and then starts a lane change attempt if its speed goes below 45mph. By doing so, it stays close to the 50mph speed as often as possible. The increase/decrease in speed of .224 mph at every update step is approximately 5m/sec<sup>2</sup>. This helps maintain the acceleration and jerk well below the maximum limits.

**3. Safe and efficient navigation**

As mentioned above, because the car is in a highway driving environment its behavior can be controlled by following simple strategies. In 1 and 2 it was described how the car follows a smooth trajectory and maintains the speed. The rest of the code in main() in main.cpp controls behavior of the car for safe and efficient navigation. The biggest part of navigation on the highway is changing lanes when required. The car attempts to do so only when it is stuck behind a slower vehicle and cannot efficiently drive on the highway at a speed close to speed limit (50mph).  The program first uses the sensor fusion data to detect any cars that are in the current lane the controlled car is driving in. If it finds a car in the current lane, it tries to determine if it is "too close" (50m) ahead. This is shown below:

```cpp
// Check for cars in lane
for (int i = 0; i < sensor_fusion.size(); i++) {
  float d = sensor_fusion[i][6];
  if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {

    // State of car found in lane
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double curr_car_speed = sqrt(vx * vx + vy * vy);
    double curr_car_s = sensor_fusion[i][5];

    // predict car's position in future. 0.02 is update step size in seconds
    curr_car_s += (double)prev_size * 0.02 * curr_car_speed;

    // If car is too close
    if ((curr_car_s > car_s) && (curr_car_s - car_s < FRONT_DIST)) {
      // Set too_close flag
      too_close = true;

      // Check if the current car is closer than previously found cars in lane
      // and update the distance and velocity of the car in front
      if (curr_front_car_dist > curr_car_s - car_s) {
        count += 1;
        curr_front_car_dist = curr_car_s - car_s;
        front_car_vel = curr_car_speed * 2.24;
        if (count > 1) {
          cout << "Updated front car, dist: " << curr_front_car_dist << ", vel: " << front_car_vel << "mph" << endl ;
        }
      }
    }
  }
}
```

As seen, the program sets a flag if there are cars too close and passes it on to the speed control as described in 2. The speed control section of the code also sets the check_lane flag for starting a lane change attempt when the speed goes below 45mph. Ones the check_lane flag is set, the program checks either left, right or both lanes to move into. The section of the code in lines 334-428 in main.cpp checks the left and right lanes. It is basically the same as above code with setting different lane number to check for vehicles. It also records s values of vehicles that are ahead in the checked lanes but, out of "vicinity". This is used for picking one of the two lanes when both are available to change to.

If the car is in the middle lane and it is safe to move to either of the adjacent lanes, a simple heuristic makes the decision which lane to pick. The program checks the closest vehicle in each lane and moves to the lane where the vehicle is further away. This helps in reducing the number of lane changes the car needs to make on an average. The code snippet performing the lane change is below and can be found in main.cpp

```cpp
// If both flags are set, pick a lane to move to
if (move_left && move_right) {

  cout << "Picking a lane.. " << endl;

  // Check the closest car in adjacent lanes
  double min_dist_left = 0;
  double min_dist_right = 0;

  // Check the average distance of cars in the adjacent lanes
  double avg_dist_left = 0;
  double avg_dist_right = 0;

  // If no car is present in adjancent lane, move to that lane with a preference to left lane
  if (car_dist_left.size() == 0 ) {
    min_dist_left = 1000;
    cout << "No cars detected on left side" << endl;
  } else if (car_dist_right.size() == 0 ) {
    min_dist_right = 1000;
    cout << "No cars detected on right side" << endl;
  } else {
    double avg_dist_left = accumulate(car_dist_left.begin(), car_dist_left.end(), 0.0) / car_dist_left.size();
    double avg_dist_right = accumulate(car_dist_right.begin(), car_dist_right.end(), 0.0) / car_dist_right.size();

    sort(car_dist_left.begin(), car_dist_left.end());
    sort(car_dist_right.begin(), car_dist_right.end());

    min_dist_left = car_dist_left[0];
    min_dist_right = car_dist_right[0];

    cout << "Left Avg dist: " << avg_dist_left << ", Closest car at: " << min_dist_left << endl;
    cout << "Right Avg dist: " << avg_dist_right << ", Closest car at: " << min_dist_right << endl;

  }

  // Move to the lane where the closest car is futher away
  if (min_dist_left > min_dist_right) {
    cout << "Moving one lane to the left!!" << endl << endl;
    if (lane == 1) { lane = 0; }
    else if (lane == 2) { lane = 1; }
    start = std::chrono::system_clock::now(); // restart timer
    check_lanes = false; // set check_lanes to false
  } else {
    cout << "Moving one lane to the right!!" << endl << endl;
    if (lane == 0) { lane = 1; }
    else if (lane == 1) { lane = 2; }
    start = std::chrono::system_clock::now();
    check_lanes = false;
  }
} else if (move_left) { // Move left
  cout << "Moving one lane to the left!!" << endl << endl;
  if (lane == 1) { lane = 0; }
  else if (lane == 2) { lane = 1; }
  start = std::chrono::system_clock::now();
  check_lanes = false;

} else if (move_right) { // Move right
  cout << "Moving one lane to the right!!" << endl << endl;
  if (lane == 0) { lane = 1; }
  else if (lane == 1) { lane = 2; }
  start = std::chrono::system_clock::now();
  check_lanes = false;

}
```

The timer being restarted at the end of lane change is used to avoid multiple lane changes. The program uses to this timer to force the car to wait at least 10secs between consecutive lane changes. The choice of using this timer was made to give the planner enough time to make a decision about multiple lane changes. It also prevents the controlled vehicle from becoming a "rogue driver" that keeps switching one lane to another eventually causing safety concerns to the rest of the traffic.

Thus, by following these simple behavioral strategies the path planner is safely and effectively able to navigate through the traffic in the highway setting of the simulator.
