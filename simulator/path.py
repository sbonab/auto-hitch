import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Arc

def calculate_theta(radius, x_hitch, y_hitch, x_veh_r, y_veh_r):
    p_hitch = np.array([x_hitch, y_hitch])
    p_veh_r = np.array([x_veh_r, y_veh_r])
    p_O_hitch = p_hitch + np.array([0, radius]) * (1 if y_veh_r > y_hitch else -1)
    p_O_veh_r = p_veh_r + np.array([0, radius]) * (-1 if y_veh_r > y_hitch else 1)
    # Distance between two circle centers
    d_OO = np.linalg.norm(p_O_veh_r - p_O_hitch)
    # Angle of tangent line
    theta = np.arcsin((x_veh_r - x_hitch)/d_OO) - np.arccos(2*radius/d_OO)
    return theta

def generate_path(radius, x_hitch, y_hitch, x_veh_r, y_veh_r):
    coef = 1 if y_veh_r > y_hitch else -1
    p_hitch = np.array([x_hitch, y_hitch])
    p_veh_r = np.array([x_veh_r, y_veh_r])
    p_O_hitch = p_hitch + np.array([0, radius]) * coef
    p_O_veh_r = p_veh_r + np.array([0, radius]) * (-1 * coef)
    # Distance between two circle centers
    d_OO = np.linalg.norm(p_O_veh_r - p_O_hitch)
    # Distance between two tangent points
    d_GG = 2 * math.sqrt((d_OO/2)**2 - radius**2)
    # Angle of tangent line
    theta = np.arcsin((x_veh_r - x_hitch)/d_OO) - np.arccos(2*radius/d_OO)
    p_G_veh_r = p_veh_r + np.array([-radius * np.sin(theta), coef * (-radius + radius * np.cos(theta))])
    p_G_hitch = p_hitch + np.array([radius * np.sin(theta), coef * (radius - radius * np.cos(theta))])
    path = np.array([p_veh_r, p_G_veh_r, p_G_hitch, p_hitch])
    return path


# Function to draw the circles
def draw_path(ax, radius, x_hitch, y_hitch, x_veh_r, y_veh_r):
    ax.plot(x_hitch, y_hitch, 'b.')
    ax.plot(x_veh_r, y_veh_r, 'b.')
    coef = 1 if y_veh_r > y_hitch else -1
    x_O1 = x_hitch
    y_O1 = y_hitch + radius * coef
    x_O2 = x_veh_r
    y_O2 = y_veh_r - radius * coef
    circle1 = plt.Circle((x_O1, y_O1), radius, color='grey', fill=False, alpha=0.7)
    circle2 = plt.Circle((x_O2, y_O2), radius, color='grey', fill=False, alpha=0.7)
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    path = generate_path(radius, x_hitch, y_hitch, x_veh_r, y_veh_r)
    ax.plot(path[:,0], path[:,1], 'r.')
    ax.plot([path[1, 0], path[2, 0]], [path[1, 1], path[2, 1]], 'r-')
    
    # theta is the angle of the arc of the circle for co-tangent
    theta = calculate_theta(radius, x_hitch, y_hitch, x_veh_r, y_veh_r)
    # Plotting the arc
    arc1_theta1 = 180 + coef * 90
    arc1_theta2 = 180 + coef * 90 + coef * theta * 180 / np.pi
    arc2_theta1 = 180 - coef * 90
    arc2_theta2 = 180 - coef * 90 + coef * theta * 180 / np.pi
    arc1 = Arc((x_O1, y_O1), 2*radius, 2*radius, 0, min(arc1_theta1, arc1_theta2), max(arc1_theta1, arc1_theta2), color='red')
    arc2 = Arc((x_O2, y_O2), 2*radius, 2*radius, 0, min(arc2_theta1, arc2_theta2), max(arc2_theta1, arc2_theta2), color='red')
    ax.add_patch(arc1)
    ax.add_patch(arc2)

# b_veh = 3.6
# w_veh = 1.3

# # initial parameters
# radius = 6
# x_veh_r = 6
# y_veh_r = 1.4
# x_hitch = 0
# y_hitch = 0

# # Set up the figure and axis
# fig, ax = plt.subplots()

# ax.set_aspect('equal', 'box')
# plt.subplots_adjust(left=0.25, bottom=0.25)
# draw_path(ax, radius, x_hitch, y_hitch, x_veh_r, y_veh_r)
# ax.grid()
# ax.set_xlim(-1, 10)
# ax.set_ylim(-2, 3)
# plt.draw()
# plt.show()  # Add this line to display the plot window
