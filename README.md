# Ultrasonic-SLAM
Basic SLAM with lower-cost Ultrasonic sensors

## How it works
### Mapping
First, we take the distance and theta measurements, and convert them to cartesian coordinates. Then we convert them to grid coordinates, and mark those coordinates as occupied on a grid.
### Localization
I use GraphSLAM to find localize the agent.

