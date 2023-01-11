from SLAM import mapping
from SLAM import localization
import numpy as np

#Test mapping

MAP_SIZE = (10, 10)
CELL_SIZE = 0.5

map = np.zeros(MAP_SIZE)
x = 0
y = 0
z = 3
theta = mapping.degrees_to_theta(90)

map = mapping.update_map(map, x, y, theta, z, CELL_SIZE)
print(map)
print("-----")

#Test localization

DT = 2.0#Updates per second

RFID = localization.maps_to_landmarks(map)

xDR = np.zeros((3, 1))
hxDR = []
hz = []

hz.append(z)
u = localization.calc_input(1.0, 0.1)#Warning: This is a simple control input calculator. I don't reccomend using this in real robots.
xDR = localization.motion_model(xDR, u, DT)
hxDR = xDR
x_opt = localization.graph_based_slam(hxDR, hz)
print(x_opt)


