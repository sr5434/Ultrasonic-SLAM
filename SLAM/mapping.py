import numpy as np
def degrees_to_theta(angle):
    # Convert the angle to the range [0, 2*pi]
    angle = np.radians(angle)
    theta = np.mod(angle, 2*np.pi)
    if theta < 0:
        theta += 2*np.pi
    return theta
def update_map(map, x, y, theta, z, cell_size):
    """
    Args:
        map: The current map
        x: The x of the agent(on the map)
        y: The y of the agent(on the map)
        theta: The sensor's heading
        z: The distance measurement from the ultrasonic sensor
        cell_size: the size of each cell
    Returns:
        The new map
    """
    # Convert the distance measurement to a Cartesian coordinates
    dx = z * np.cos(theta)
    dy = z * np.sin(theta)
    
    # Convert the Cartesian coordinate to grid coordinates
    gx = int((x + dx) / cell_size)
    gy = int((y + dy) / cell_size)
    
    # Update the map by setting the value of the cell at (gx, gy) to the distance measurement
    map[gx][gy] = 1
    return map