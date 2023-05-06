#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from generate_waypoints import get_waypoints, get_speed_lookup
from scipy.spatial.transform import Rotation
import sys

RACE_PATH = 0
OUTSIDE_PATH = 1
INSIDE_PATH = 2

class OverTaking(Node):
    def __init__(self, ego=True):
        super().__init__('overtaking_node' if ego else 'opp_node')
        # Launch file stuff to hopefully use in the future
        param_descriptor = ParameterDescriptor(description='ego or not')
        self.declare_parameter('ego', True, param_descriptor)
        self.ego = self.get_parameter('ego').get_parameter_value().bool_value
        self.ego = ego

        # TODO: create subscribers and publishers
        if self.ego:
            self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
            self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
            self.viz_pub = self.create_publisher(Marker, '/ego_racecar/pure_pursuit', 10)
            self.opp_sub = self.create_subscription(Odometry, '/ego_racecar/opp_odom', self.opp_callback, 10)
            self.laser_sub = self.create_subscription(LaserScan, '/scan',self.scan_callback, 10)
        else:
            self.odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)
            self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)
            self.viz_pub = self.create_publisher(Marker, '/opp_racecar/pure_pursuit', 10)
            self.opp_sub = self.create_subscription(Odometry, '/opp_racecar/opp_odom', self.opp_callback, 10)
            self.laser_sub = self.create_subscription(LaserScan, '/opp_scan',self.scan_callback, 10)

        # Lidar params
        # Right of the car is the end of the array
        self.angle_min = -2.3499999046325684
        self.angle_max = 2.3499999046325684
        self.angle_incr = 0.004351851996034384
        self.arr_len = 1080
        self.range_min = 0.
        self.range_max = 30.
        # For 90 deg
        self.min_idx = int((-np.pi/2. - self.angle_min)/self.angle_incr)
        self.max_idx = int((np.pi/2. - self.angle_min)/self.angle_incr)
        self.left = 0.
        self.right = 0.

        # Pure pursuit params
        self.L = 1.2
        self.K = .45
        self.curr_waypoint = 0 # waypoint index

        # Overtaking params
        self.curr_path = RACE_PATH
        self.overtake_dist = 1.2 # How close should the car infront be before we try overtaking
        self.cutoff_dist = .75 # How far ahead should we be before we cut back into the racing line
        self.overtake_width = .5 # How far to the side should the car infront be before we consider it safe to pass

        self.waypoints = get_waypoints() # array containing all waypoint arrays
        self.speeds = get_speed_lookup()

        self.opp_pose = None
        self.overtaking = False

    def viz_point(self, pose, id, thick=False):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'point'
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.
        if thick:
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        self.viz_pub.publish(marker)
    
    def viz_path(self, path, color, id):
        viz_waypoints = []
        for waypoint in path:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.
            viz_waypoints.append(p)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'path'
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = viz_waypoints
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.viz_pub.publish(marker)

    def scan_callback(self, msg):
        ranges = msg.ranges
        ranges = np.clip(ranges, self.range_min, self.range_max)
        # Take 20 readings ~5 deg
        self.left = np.mean(ranges[self.min_idx-10:self.min_idx+10])
        self.right = np.mean(ranges[self.max_idx-10:self.max_idx+10])

    def find_waypointV2(self, pose, H, path, set_var=True):
        # Requirements for waypoint:
        # Must be ahead of the car, unless there are none
        # Must be the closest to L meters away
        orig_pose = pose
        look_ahead = np.ceil((self.L + .5) * 10) # Waypoint density is roughly 10 per m, check an extra .5 meter
        pose = np.array([pose.x, pose.y]) # ignore the z
        curr_waypoints = self.waypoints[path]
        curr_speeds = self.speeds[path]

        dists = np.linalg.norm(curr_waypoints - pose, axis=1)
        # This tolerance param is quite important, dependent on waypoint density 
        canidates = np.argwhere(np.isclose(dists, self.L, atol=.3)) 

        if len(canidates) == 0:
            idx = np.argmin(dists)
            if set_var:
                self.curr_waypoint = idx
            return curr_waypoints[idx], curr_speeds[idx]
            
        # Check which points are ahead of the car
        points = curr_waypoints[canidates].reshape(len(canidates), 2)
        points = np.concatenate((points, np.array([[0,1]] * len(canidates))), axis=1) # Add a 0, 1 for the z
        points = (H @ points.T).T
        points = (points / points[:,-1][:, np.newaxis])[:, :-1] # normalize and remove last element
        canidates = canidates[points[:,0] > 0]

        if len(canidates) == 0:
            # Use old method to find point
            print("No canidates ahead of the car")
            return self.find_waypoint(orig_pose, set_var=set_var)
        
        # Get the point furtherest along the path as the canidate point
        idx = canidates.max()

        # # Do some checking to make sure that the idx is the first value that is greater than L for interp
        # if idx < 5:
        #     wrapped_dists = np.concatenate((dists[idx-5:], dists[:idx+5]))
        #     temp_idx = np.argmax(wrapped_dists > self.L)
        #     idx = (idx + (temp_idx - 5))
        #     if idx < 0:
        #         idx += len(dists)
        # elif idx > len(dists) - 5:
        #     wrapped_dists = np.concatenate((dists[idx-5:], dists[:5-(len(dists)-idx)]))
        #     temp_idx = np.argmax(wrapped_dists > self.L)
        #     idx = (idx + (temp_idx - 5))
        #     if idx > len(dists) - 1:
        #         idx -= len(dists)
        # else:
        #     idx = np.argmax(dists[idx-5:idx+5] > self.L)

        # Interpolation
        start = curr_waypoints[idx - 1]
        end = curr_waypoints[idx]
        interp = (self.L - dists[idx - 1]) / (dists[idx] - dists[idx - 1]) # interpolation factor
        
        interp_waypoint = start + interp * (end - start)
        if not np.isclose(np.linalg.norm(interp_waypoint - pose), self.L, rtol=1e-2):
            #print(f"Interpolation is not within .01 meters of L, {self.L}")
            #raise ValueError("Interpolation is not within .01 meters of L")
            duh = 1
        if set_var:
            self.curr_waypoint = idx
        return interp_waypoint, curr_speeds[idx]

    def find_waypoint(self, pose_in, set_var=True):
        """ Find the next desired waypoint
        Points are in 2D
        1. Only look 'forward' in the waypoint array from the current (loops back at the end)
        2. Find distance from current pose to each waypoint
        3. Find the waypoint that is closest to the lookahead distance
        4. Interpolate a waypoint if needed
        """

        # how many waypoints to look ahead, should be calculated based on L and how dense our waypoints are
        # Dont want to look 'backwards' into the array
        look_ahead = 60 # TODO change based on L and waypoint density
        pose = np.array([pose_in.x, pose_in.y]) # ignore the z
        curr_path = self.curr_path
        curr_waypoints = self.waypoints[curr_path]
        curr_speeds = self.speeds[curr_path]

        dists = np.linalg.norm(curr_waypoints - pose, axis=1)
        
        if self.curr_waypoint + look_ahead > len(curr_waypoints): # wrap around
            waypoints = np.concatenate([curr_waypoints[self.curr_waypoint:], curr_waypoints[:self.curr_waypoint + look_ahead - len(curr_waypoints)]])
            truncated_dists = np.concatenate([dists[self.curr_waypoint:], dists[:self.curr_waypoint + look_ahead - len(curr_waypoints)]])
        else:
            waypoints = curr_waypoints[self.curr_waypoint:self.curr_waypoint + look_ahead]
            truncated_dists = dists[self.curr_waypoint:self.curr_waypoint + look_ahead]
        
        # Find closests waypoint
        end = None
        for i in range(len(truncated_dists)):
            if truncated_dists[i] > self.L:
                end = waypoints[i]
                break
        if end is None or i == 0:
            #print(f"Error couldn't find valid waypoint!, {self.L}, {end}, {i}")
            # Here assume that we are "lost"
            # Find the nearest waypoint, set this as the current waypoint and recall this function
            i = np.argmin(dists)
            if set_var:
                self.curr_waypoint = i
            self.viz_point(curr_waypoints[i], 0, thick=True)
            return curr_waypoints[i], curr_speeds[dists.argmin()]

        # Interpolate
        start = waypoints[i - 1]
        interp = (self.L - truncated_dists[i - 1]) / (truncated_dists[i] - truncated_dists[i - 1]) # interpolation factor
        
        interp_waypoint = start + interp * (end - start)
        if not np.isclose(np.linalg.norm(interp_waypoint - pose), self.L, rtol=1e-2):
            #print(f"Interpolation is not within .01 meters of L, {self.L}")
            #raise ValueError("Interpolation is not within .01 meters of L")
            duh = 1
        if set_var:
            self.curr_waypoint = (self.curr_waypoint + i - 1) % len(curr_waypoints)

        return interp_waypoint, curr_speeds[dists.argmin()]
    
    def overtaking_controller(self, dist, opp_pose, pose, H):
        # Handle any overtaking logic of which path to be on and what not
        #print(opp_pose)
        if dist < self.overtake_dist and opp_pose[0] > 0: # want car to be infront in the pos x dir
            print('overtake!', opp_pose)
            threshold = self.overtake_width if not self.overtaking else self.overtake_width / 5
            if np.abs(opp_pose[1]) < threshold:
                # Gap is too small, switch to a diff path
                inside_point, inside_speed = self.find_waypointV2(pose, H, INSIDE_PATH, set_var=False)
                outside_point, outside_speed = self.find_waypointV2(pose, H, OUTSIDE_PATH, set_var=False)

                inside_point = H @ np.concatenate([inside_point, [0, 1]]) # Add a 0, 1 for the z, homography
                inside_point = (inside_point / inside_point[-1])[:-1] # normalize and remove last element

                outside_point = H @ np.concatenate([outside_point, [0, 1]]) # Add a 0, 1 for the z, homography
                outside_point = (outside_point / outside_point[-1])[:-1] # normalize and remove last element

                if np.abs(inside_point[1]) > np.abs(outside_point[1]):
                    print('switch to inside')
                    self.curr_path = INSIDE_PATH
                    self.overtaking = True
                else:
                    print('switch to outside')
                    self.curr_path = OUTSIDE_PATH
                    self.overtaking = True
        elif self.overtaking:
            if opp_pose[0] < -self.cutoff_dist:
                # If we are in overtaking mode and sufficently ahead of the other car, switch back to raceline
                self.curr_path = RACE_PATH
                self.overtaking = False
                print('done overtaking', opp_pose)
            elif dist > self.overtake_dist + .2 and opp_pose[0] > 0:
                self.curr_path = RACE_PATH
                self.overtaking = False
                print('overtake failed')

    def odom_callback(self, msg):
        # Find the current waypoint to track using methods mentioned in lecture
        pose = msg.pose.pose.position
        if self.ego:
            colors = [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]
            for i, (color, path) in enumerate(zip(colors, self.waypoints)):
                self.viz_path(path, color, i)
        
        # Find waypoint and convert to car frame
        # get a rotation matrix from the quaternion
        # rot_mat = quaternion_matrix([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w])
        rot_mat = Rotation.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_matrix()
        # homography matrix to transform waypoint to frame of car
        H = np.eye(4)
        H[:3, :3] = rot_mat[:3, :3]
        H[:3, 3] = [pose.x, pose.y, pose.z]
        H = np.linalg.inv(H)
        
        # Overtaking check
        if self.ego and self.opp_pose is not None:
            dist = np.linalg.norm(np.array([self.opp_pose.x, self.opp_pose.y]) - np.array([pose.x, pose.y]))
            opp_pose = H @ np.concatenate([np.array([self.opp_pose.x, self.opp_pose.y]), [0, 1]]) # Add a 0, 1 for the z, homography
            opp_pose = (opp_pose / opp_pose[-1])[:-1] # normalize and remove last element
            self.overtaking_controller(dist, opp_pose, pose, H)

        waypoint, speed = self.find_waypointV2(pose, H, self.curr_path, set_var=True)
        self.viz_point(waypoint, 0, thick=True)

        waypoint = H @ np.concatenate([waypoint, [0, 1]]) # Add a 0, 1 for the z, homography
        waypoint = (waypoint / waypoint[-1])[:-1] # normalize and remove last element

        # Calculate curvature/steering angle 2|y|/L^2
        curvature = 2 * np.abs(waypoint[1]) / (self.L ** 2)

        # Calculate and publish steering angle (clip to min and max steering angle)
        steering_angle = np.clip(curvature * self.K, -.36, .36) * np.sign(waypoint[1])
        speed = 1.5 if self.ego else 1.
        #speed = 0.1
        #print(f"Steering angle: {steering_angle}, curvature: {curvature}")
        self.drive_pub.publish(AckermannDriveStamped(drive=AckermannDrive(steering_angle=steering_angle, speed=speed)))

    def opp_callback(self, msg):
        # Opposition odom callback
        self.opp_pose = msg.pose.pose.position

def main(args=None,ego='two'):
    rclpy.init(args=args)
    print("Overtaking Initialized")

    if ego == 'two':
        print('two car mode')
        # Two car mode NOTE THIS DOES NOT WORK BC THE THREADS ARE BLOCKING
        
        overtaking_node = OverTaking(ego=True)
        opp_node = OverTaking(ego=False)
        
        executor = MultiThreadedExecutor()

        executor.add_node(overtaking_node)
        executor.add_node(opp_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            # Destroy the node explicitly
            # (optional - otherwise it will be done automatically
            # when the garbage collector destroys the node object)
            overtaking_node.destroy_node()
            opp_node.destroy_node()
            rclpy.shutdown()
    else:
        print(f'one car mode {ego}')
        ego = True if ego == 'ego' else False
        overtaking_node = OverTaking(ego=ego)
        rclpy.spin(overtaking_node)
        overtaking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    ego = sys.argv[1] if len(sys.argv) > 1 else 'two'
    main(ego=ego)
