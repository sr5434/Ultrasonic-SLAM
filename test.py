from SLAM import mapping
from SLAM import localization
import numpy as np

#Test mapping

MAP_SIZE = (10, 10)
CELL_SIZE = 0.5

map = np.zeros(MAP_SIZE)
x = 0
y = 0
theta = mapping.degrees_to_theta(90)

map = mapping.update_map(map, x, y, theta, 3, CELL_SIZE)
print(map)

#Test localization

DT = 2.0

RFID = localization.maps_to_landmarks(map)

xTrue = np.zeros((3, 1))
xDR = np.zeros((3, 1))
u = localization.calc_input(1.0, 0.1)#Warning: This is a simple control input calculator. I don't reccomend using this in real robots.
xTrue, z, xDR, ud = localization.observation(xTrue, xDR, u, RFID, DT)
print(xTrue)