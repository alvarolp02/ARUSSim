from scipy.interpolate import splprep, splev
import numpy as np


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def translation(cone_list):
    origin = cone_list[0]
    traslated_cones = []

    for cone in cone_list:
        translation_x = cone[0] - origin[0]
        translation_y = cone[1] - origin[1]
        traslated_cones.append((translation_x, translation_y))

    return traslated_cones

def rotation(cone_list):
    # Get the first two cones
    x1, y1 = cone_list[0]
    x2, y2 = cone_list[1]

    # Calculate the difference in x and y
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the rotation angle to align the first two cones with the X-axis
    rotation_angle = np.arctan2(dy, dx)

    # Rotation function: rotates a point (x, y) around the origin by the specified angle
    def rotate_point(x, y, angle):
        x_rotated = x * np.cos(angle) - y * np.sin(angle)
        y_rotated = x * np.sin(angle) + y * np.cos(angle)
        return x_rotated, y_rotated

    # Apply the rotation to all cones
    rotated_cones = [rotate_point(x, y, -rotation_angle) for x, y in cone_list]

    return rotated_cones

def smooth_and_expand_points(points, offset, num_points, min_distance=5):
    translated_points = translation(points)
    rotated_points = rotation(translated_points)

    x = []
    y = []

    for point in rotated_points:
        x.append(point[0])
        y.append(point[1])
    
    # Smoothed curve
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
        outer_point = (x_smooth[i] + offset/2 * normal_x[i], 
                       y_smooth[i] + offset/2 * normal_y[i])
        inner_point = (x_smooth[i] - offset/2 * normal_x[i], 
                       y_smooth[i] - offset/2 * normal_y[i])
        
        # If it is the first cone or the distance between the last point in the list and the one to be added is greater than or equal to the minimum distance, add the point
        if i == 0 or distance(outer_cones[-1], outer_point) >= min_distance:
            outer_cones.append(outer_point)
        if i == 0 or distance(inner_cones[-1], inner_point) >= min_distance:
            inner_cones.append(inner_point)

    return outer_cones, inner_cones