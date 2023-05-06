# import matplotlib.pyplot as plt
import numpy as np
#from scipy import interpolate
import bezier

# corners are a set of 4 points, corner point, entry, control, exit

# corner 1 is the corner closest to K lab
# corners are a set of 3 points, corner point, entry, control, exit, 

points = []

# Raceline points
corner1_r = np.array([np.array([6.87883,1.30847]), 
                    np.array([6.1366,0.0228705]), 
                    np.array([7.7, .5]), 
                    np.array([8.00807,3.29504])]) # wider gap = x,-y

corner2_r = np.array([np.array([6.43737,8.39063]), 
                    np.array([7.76042,6.79449]), 
                    np.array([7.25, 9.25]), 
                    np.array([5.35579,9.53412])]) # wider gap = x,y

corner3_r = np.array([np.array([-15.5477,6.88314]), 
                    np.array([-14.3637,7.96213]), 
                    np.array([-16.5, 7.7]), 
                    np.array([-16.2773,5.91601])]) # wider gap = -x,y 

corner4_r = np.array([np.array([-15.3005,0.0180681]), 
                    np.array([-16.4105,1.27514]), 
                    np.array([-16.35, -1.05]), 
                    np.array([-13.4578,-0.87522])]) # wider gap = -x,-y
points.append((corner1_r, corner2_r, corner3_r, corner4_r))

# Outside line points
corner1_o = np.array([np.array([6.87883,1.30847]), 
                    np.array([7.1,0.0228705]), 
                    np.array([8.6, .1]), 
                    np.array([8.00807,3.29504])]) # wider gap = x,-y

corner2_o = np.array([np.array([6.43737,8.39063]), 
                    np.array([7.6,8.5]), 
                    np.array([7.5, 9.63]), 
                    np.array([6.4,9.53412])]) # wider gap = x,y

corner3_o = np.array([np.array([-15.5477,6.88314]), 
                    np.array([-15.1, 8.0]), 
                    np.array([-16.5, 7.7]), 
                    np.array([-16.2773,6.1])]) # wider gap = -x,y 

corner4_o = np.array([np.array([-15.3005,0.0180681]), 
                    np.array([-16.4105,-.2]), 
                    np.array([-16.1, -1.15]), 
                    np.array([-15.,-1.])]) # wider gap = -x,-y
points.append((corner1_o, corner2_o, corner3_o, corner4_o))

# Inside line points
corner1_i = np.array([np.array([6.87883,1.30847]), 
                    np.array([6.87,0.75]), 
                    np.array([7.25, .97]), 
                    np.array([7.3,1.4])]) # wider gap = x,-y

corner2_i = np.array([np.array([6.43737,8.39063]), 
                    np.array([6.88,8.5]), 
                    np.array([6.62, 9.01]), 
                    np.array([6.0,8.9])]) # wider gap = x,y

corner3_i = np.array([np.array([-15.5477,6.88314]), 
                    np.array([-15.61,7.3]), 
                    np.array([-16, 7.1]), 
                    np.array([-16., 6.1])]) # wider gap = -x,y 

corner4_i = np.array([np.array([-15.3005,0.0180681]), 
                    np.array([-15.7,0.15]), 
                    np.array([-15.4, -.4]), 
                    np.array([-15.,-0.3])]) # wider gap = -x,-y
points.append((corner1_i, corner2_i, corner3_i, corner4_i))

paths = []
speeds = []

for corner1, corner2, corner3, corner4 in points:
    corners = np.stack([corner1, corner2, corner3, corner4])

    straight1 = np.linspace(corner1[-1], corner2[1], int(np.linalg.norm(corner1[-1] - corner2[1]) * 10), endpoint=False) #corner1 to corner2
    straight2 = np.linspace(corner2[-1], corner3[1], int(np.linalg.norm(corner2[-1] - corner3[1]) * 10), endpoint=False) #corner2 to corner3
    straight3 = np.linspace(corner3[-1], corner4[1], int(np.linalg.norm(corner3[-1] - corner4[1]) * 10), endpoint=False) #corner3 to corner4
    straight4 = np.linspace(corner4[-1], corner1[1], int(np.linalg.norm(corner4[-1] - corner1[1]) * 10), endpoint=True) #corner4 to corner1

    interp_corners = []
    for corner in corners:

        curve = bezier.Curve(corner[1:].T, degree=2)
        spline_x = np.linspace(0, 1, 30, endpoint=False)
        res = curve.evaluate_multi(spline_x)
        interp_corners.append(res.T)

        min_dist = np.linalg.norm(res.T - corner[0], axis=1)
        print(f"Min dist from corner: {min_dist.min()}")
        
        # figure, axes = plt.subplots()
        # circle = plt.Circle((corner[0, 0], corner[0, 1]), .5, fill=False)
        # axes.set_aspect(1)
        # axes.add_artist(circle)
        # plt.scatter(corner[1:, 0], corner[1:, 1])
        # plt.plot(res[0], res[1])
        # plt.scatter(corner[0, 0], corner[0, 1], c='red')
        # plt.show()

    full_path = np.concatenate([interp_corners[0], straight1, interp_corners[1], straight2, interp_corners[2], straight3, interp_corners[3], straight4])
    slowdown_entry = int(1.75 * 10.) # how far before entering the corner to start slowing down
    speedup_exit = int(.4 * 10.) # how far before exiting the corner to start speeding up
    straight_speed = 6.
    slowdown_speed = 3.
    speed_lookup = np.concatenate([np.ones(len(interp_corners[0])) * slowdown_speed, np.ones(speedup_exit) * slowdown_speed, 
        np.ones(len(straight1[speedup_exit:len(straight1)-slowdown_entry])) * straight_speed, np.ones(len(straight1[len(straight1)-slowdown_entry:])) * slowdown_speed,
        np.ones(len(interp_corners[1])) * slowdown_speed, np.ones(speedup_exit) * slowdown_speed,
        np.ones(len(straight2[speedup_exit:len(straight2)-slowdown_entry])) * straight_speed, np.ones(len(straight2[len(straight2)-slowdown_entry:])) * slowdown_speed,
        np.ones(len(interp_corners[2])) * slowdown_speed, np.ones(speedup_exit) * slowdown_speed,
        np.ones(len(straight3[speedup_exit:len(straight3)-slowdown_entry])) * straight_speed, np.ones(len(straight3[len(straight3)-slowdown_entry:])) * slowdown_speed,
        np.ones(len(interp_corners[3])) * slowdown_speed, np.ones(speedup_exit) * slowdown_speed,
        np.ones(len(straight4[speedup_exit:len(straight4)-slowdown_entry])) * straight_speed, np.ones(len(straight4[len(straight4)-slowdown_entry:])) * slowdown_speed])
    assert len(full_path) == len(speed_lookup)
    paths.append(full_path)
    speeds.append(speed_lookup)

def get_waypoints():
    return paths

def get_speed_lookup():
    return speeds