import numpy as np
import matplotlib.pyplot as plt
import math


def random_control(closest_x, closest_y, closest_theta, random_x, random_y, velocity, steering):
    wheelbase = 0.35
    theta_dot = velocity * np.tan(steering) / wheelbase
    dt = 0.03
    waypoints_x = []
    waypoints_y = []
    waypoints_theta = []
    waypoints_x.append(closest_x)
    waypoints_y.append(closest_y)
    waypoints_theta.append(closest_theta)
    reach_goal = False
    total_time = 0
    while not reach_goal:
        total_time += dt
        closest_theta += theta_dot * dt
        x_dot = velocity * np.cos(closest_theta)
        y_dot = velocity * np.sin(closest_theta)
        closest_x += x_dot * dt
        closest_y += y_dot * dt
        waypoints_x.append(closest_x)
        waypoints_y.append(closest_y)
        waypoints_theta.append(closest_theta)
        if ((closest_x - random_x) ** 2 + (closest_y - random_y) ** 2) ** 0.5 < 0.5:
            reach_goal = True
    return [waypoints_x, waypoints_y, waypoints_theta], total_time


random_x, random_y = 10, 20  # random point on map
closest_x, closest_y, yaw = 5, 7, np.pi * 1.2  # closest point in graph

Lf = ((random_x - closest_x) ** 2 + (random_y - closest_y) ** 2) ** 0.5
wheelbase = 0.35
alpha = math.atan2(random_y - closest_y, random_x - closest_x) - yaw

fig = plt.figure()
ax = fig.add_subplot()
velocity = 1
steering = 0.01
x_coords = []
y_coords = []
velocity = np.random.uniform(2.0)
steering = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1.0)

waypoints, total_time = random_control(closest_x, closest_y, yaw, random_x, random_y, velocity, steering)
ax.scatter(waypoints[0], waypoints[1])
ax.scatter([closest_x, random_x], [closest_y, random_y], c='red')
ax.set_aspect('equal', 'box')

plt.show()
