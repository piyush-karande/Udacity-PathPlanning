# Path Planning #
The goal of this project was navigate a car on a three lane highway in a simulated enviorment. Specifically:

1. Car should follow a smooth trajectory without violating maximum velociy, acceleration, and jerk restrictions.
2. Car should not crash other cars on the 3 lane highway.
3. Car should stay inside its lane and change lanes to avoid getting stuck behind a slow car, while following all safety rules.
4. Car should use the localization, sensor fusion, and map data to perform the above mentioned tasks.

## Rubric Points ##

### Compilation ###

The code was modified from the starter provided by [Udacity on github](https://github.com/udacity/CarND-Path-Planning-Project). The code does not include any additional files and should compile correctly.

### Valid Trajectories ###

**1. The car should drive at least 4.32 miles without incident.**

The code was tested several times with the [term 3 simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) provided by Udacity. The tests were conducted by running the simulator continuosly for 20-25 mins and also with 1-2 lap runs multiple times. The controlled vehicle was observed to drive the highway without any warnings and incidents except in rare cases. The only times an incident was observed were when there were "rogue" vehicles in the vicinity. This could be extreme braking or very quick changes in lanes by other vehicles. 

**2. The car drives according to the speed limit.**

The maximum velocity of the car is set to 49.50mph. This helps the car drive below the speed limit at all times. The car is programmed to slow down to a velocity close to the vehicle it is following when the vehicle is within 30m. If the car is behind a slow moving vehicle, it is instructed to plan and change lanes while following all the rules. This helps to maintain the average velocity of the car as close to the speed limit as possible, while still following all the rules.

**3. Max Acceleration and Jerk are not Exceeded.**





