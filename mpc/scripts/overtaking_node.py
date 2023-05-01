#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from generate_waypoints import get_waypoints, get_speed_lookup
from scipy.spatial.transform import Rotation

RACE_PATH = 0
OUTSIDE_PATH = 1
INSIDE_PATH = 2

class OverTaking(Node):
    def __init__(self, ego=True):
        super().__init__('overtaking_node' if ego else 'opp_node')
        self.ego = ego
        # TODO: create subscribers and publishers
        #self.laser_sub = self.create_subscription(LaserScan, '/scan',self.scan_callback, 10)
        if ego:
            self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
            self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
            self.viz_pub = self.create_publisher(Marker, '/ego_racecar/pure_pursuit', 10)
        else:
            self.odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_callback, 10)
            self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)
            self.viz_pub = self.create_publisher(Marker, '/opp_racecar/pure_pursuit', 10)

        # Pure pursuit params
        self.L = 1.2
        self.K = .45
        self.curr_waypoint = 0 # waypoint index

        # Overtaking params
        self.curr_path = RACE_PATH
        self.overtake_dist = 1.5 # How close should the car infront be before we try overtaking

        self.waypoints = get_waypoints() # array containing all waypoint arrays
        self.speeds = get_speed_lookup()

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

    def get_range(self, range_data, angle):
        # The line below is: angle_in_lidar = 135deg - angle
        #angle_in_lidar = -1*self.angle_min - angle
        angle_in_lidar = -1*self.angle_min + angle
        index = int(angle_in_lidar/self.angle_incr)
        return range_data[index]
    
    def overtaking_check(self, msg, location, path, waypoint):
        rangetocheck = 1
        if (self.get_dist(waypoint,location)< rangetocheck):
            pathgood = self.close_to_obstacle(waypoint,location,msg)
            if (pathgood == False):
                path = self.select_new_path(path)
    
    def get_dist(self, waypoint_location,car_location_in_world):
        return (((car_location_in_world[0]-waypoint_location[0])**2 + (car_location_in_world[1]-waypoint_location[1])**2 )**.5)

    def Select_new_path(path):
        if (path == 2):
            return 1
        else:
            return 2
        
    #helper function checks to see if a given waypoint is close to a lidar detected obstacle
    def close_to_obstacle(self,waypoint_location, self_location, msg):
        range_we_care_about=1
        max_acceptable_dist_from_obstacle= .2
        rangedata=msg.ranges
        angle1rad=(60*np.pi)/180
        #the heading angle of the car
        heading=0
        angles = np.linspace(-angle1rad, angle1rad, num=10)
        for angle in angles:
            theta= (3.14/2)-angle-heading
            dist= self.get_range(rangedata,angle)
            if (dist< range_we_care_about):
                obstacle_location_in_world_frame= (self_location[0]+np.cos(theta)*dist, self_location[1]+np.sin(theta)*dist)
                if (self.get_dist(waypoint_location, obstacle_location_in_world_frame)<max_acceptable_dist_from_obstacle):
                    return False
        return True

    def scan_callback(self, msg):
        pass

    def find_waypoint(self, pose_in):
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
            print(f"Error couldn't find valid waypoint!, {self.L}, {end}, {i}")
            # Here assume that we are "lost"
            # Find the nearest waypoint, set this as the current waypoint and recall this function
            i = np.argmin(dists)

            self.curr_waypoint = i
            self.viz_point(curr_waypoints[i], 0, thick=True)
            return curr_waypoints[i], curr_speeds[dists.argmin()]

        # Interpolate
        start = waypoints[i - 1]
        interp = (self.L - truncated_dists[i - 1]) / (truncated_dists[i] - truncated_dists[i - 1]) # interpolation factor
        
        interp_waypoint = start + interp * (end - start)
        if not np.isclose(np.linalg.norm(interp_waypoint - pose), self.L, rtol=1e-2):
            print(f"Interpolation is not within .01 meters of L, {self.L}")
            #raise ValueError("Interpolation is not within .01 meters of L")

        self.curr_waypoint = (self.curr_waypoint + i - 1) % len(curr_waypoints)

        self.viz_point(interp_waypoint, 0, thick=True)

        return interp_waypoint, curr_speeds[dists.argmin()]
    
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
        waypoint, speed = self.find_waypoint(pose)

        waypoint = H @ np.concatenate([waypoint, [0, 1]]) # Add a 0, 1 for the z, homography
        waypoint = (waypoint / waypoint[-1])[:-1] # normalize and remove last element

                # Calculate curvature/steering angle 2|y|/L^2
        curvature = 2 * np.abs(waypoint[1]) / (self.L ** 2)

        # Calculate and publish steering angle (clip to min and max steering angle)
        steering_angle = np.clip(curvature * self.K, -.36, .36) * np.sign(waypoint[1])
        speed = 3. if self.ego else 1.
        #print(f"Steering angle: {steering_angle}, curvature: {curvature}")
        self.drive_pub.publish(AckermannDriveStamped(drive=AckermannDrive(steering_angle=steering_angle, speed=speed)))

def main(args=None):
    rclpy.init(args=args)
    print("Overtaking Initialized")
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


if __name__ == '__main__':
    main()
