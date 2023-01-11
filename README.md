# Ultrasonic-SLAM
Basic SLAM with lower-cost Ultrasonic sensors

## How it works
### Mapping
First, we take the distance and theta measurements, and convert them to cartesian coordinates. Then we convert them to grid coordinates, and mark those coordinates as occupied on a grid.
### Localization
I use GraphSLAM to find localize the agent.

## Results
The mapping function returns a 2D array. Squares marked as 1 are occupied, and squares that are marked as 0 are not. The localization function returns an array that includes the x, y, and yaw of the robot(in that order). 
