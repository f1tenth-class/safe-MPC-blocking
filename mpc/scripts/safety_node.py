#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.ranges=np.zeros(1080)
        self.time=0.
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(Odometry, '/ego_racecar/odom',self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, '/scan',self.scan_callback, 10)
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        speed=odom_msg.twist.twist.linear.x
        #self.get_logger().info("speed= "+ str(speed))

        self.speed = speed

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        #if(self.ranges[0]==0.0):
        #    self.ranges=scan_msg.ranges
        #    self.time=scan_msg.header.stamp.sec
        #    self.get_logger().info(str(scan_msg.header.stamp.sec))
        #    #self.get_logger().info(str(self.time) )
        #    self.get_logger().info("first") 
        #newranges=scan_msg.ranges
        #min=int(scan_msg.range_min)
        #max=int(scan_msg.range_max)
        #newranges=np.clip(newranges,min,max)
        minttc=1.4
        ranges=scan_msg.ranges
        length=len(ranges)
        angles=np.arange(start=scan_msg.angle_min, stop=scan_msg.angle_max, step=scan_msg.angle_increment)
        v=self.speed * np.cos(angles)
        v=np.clip(v,0,None)
        v=np.clip(v,.001,None)
        ttc=ranges/v
        if (any(ttc<minttc)):
        #OTHER METHOD
        #rprime=np.subtract(newranges,self.ranges)
        #newtime=scan_msg.header.stamp.sec
        #deltaT=newtime-self.time
        #self.time=newtime
        #if(deltaT>0):
        #    rprime=rprime/(deltaT)
        #timetocollision=np.ones(1080)*np.inf
        #rprime=-1*rprime
        #rprime=np.clip(rprime,0,None)
        #timetocollision= np.divide(newranges,(rprime))
        #self.get_logger().info(str(newranges))
        #if (any(timetocollision<.05)):
                msg = AckermannDriveStamped()
                msg.drive.speed=0.0
                self.publisher_.publish(msg)
                self.get_logger().info("braking")
        #if(timetocollision[540]<.09):
        #        msg = AckermannDriveStamped()
        #        msg.drive.speed=0.0
        #        self.publisher_.publish(msg)
        #        self.get_logger().info("secondary")
        #min=np.min(timetocollision)
        #self.ranges=newranges
        
        #self.get_logger().info("mintimetocollision= "+ str(min))
        #self.get_logger().info("angle= "+ str(angles[index]))
        #self.get_logger().info("dist.= "+ str(newranges[index]))
        
        
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
