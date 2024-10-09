from scipy.interpolate import splprep, splev
import numpy as np


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def smooth_and_expand_points(points, offset, num_points, min_distance=5):
    x = []
    y = []

    for point in points:
        x.append(point[0])
        y.append(point[1])
    
    tck, u = splprep([x, y], s=1.0, per=1, k=3)
    
    u_new = np.linspace(0, 1, num_points)
    x_smooth, y_smooth = splev(u_new, tck)

    # Calculate the tangents (derivatives) for each point of the smooth line
    dx, dy = splev(u_new, tck, der=1)
    
    # Normalize the tangents and calculate the normal vectors
    norm = np.sqrt(dx**2 + dy**2)
    normal_x, normal_y = -dy / norm, dx / norm

    outer_cones = []
    inner_cones = []

    # Add the first outer and inner points
    for i in range(num_points):
        outer_point = (x_smooth[i] + offset * normal_x[i], 
                       y_smooth[i] + offset * normal_y[i])
        inner_point = (x_smooth[i] - offset * normal_x[i], 
                       y_smooth[i] - offset * normal_y[i])
        
        # If it's the first cone or the distance between the last point in the list and the one to be added is greater than or equal to the minimum distance, add the point
        if i == 0 or distance(outer_cones[-1], outer_point) >= min_distance:
            outer_cones.append(outer_point)
        if i == 0 or distance(inner_cones[-1], inner_point) >= min_distance:
            inner_cones.append(inner_point)

    return outer_cones, inner_cones
