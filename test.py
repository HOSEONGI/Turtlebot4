#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped,PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
import math
import qrcamera
# 추가
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class RobotExplorer(Node):
    def __init__(self):
        super().__init__('robot_explorer')

        self.state = "EXPLORER"
        self.twist = Twist()
        self.turn_count = 0
        self.turn_direction = 1
        self.initial_x = None
        self.initial_y = None
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            self.qos_policy
        )
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_cb,
            self.qos_policy
        )
       
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.publisher_path = self.create_publisher(
            Path,
            '/path',
            10
        )
        
        

        self.path = Path()
        self.path.header.frame_id = 'map'
        self.x = 0
        self.y = 0




    def pose_cb(self, msg):
        if self.initial_x is None and self.initial_y is None:
             self.initial_x = msg.pose.pose.position.x
             self.initial_y = msg.pose.pose.position.y
             print([self.initial_x, self.initial_y])
            

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        

    def listener_callback(self, msg):
        if self.state != "EXPLORER":
            return

        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        index_front = round((-1.5708 - angle_min) / angle_increment)

        index_start = max(0, index_front - 15)
        index_end = min(len(msg.ranges) - 1, index_front + 15)
        front_ranges = msg.ranges[index_start:index_end+1]
        # (기존_최솟값 계산)front_distance = min([x for x in front_ranges if x != float('inf')])

        # ///--------주행 추가 front(15도 시야), left(30도 시야), right(30도 시야)--------\\\
        try:
            front_distance = statistics.mean([x for x in front_ranges if x != float('inf')])
        except statistics.StatisticsError:
            front_distance = 0
        
        index_left = round((0 - angle_min) / angle_increment)
        index_left_start = max(0, index_left - 120)
        index_left_end = min(len(msg.ranges) - 1, index_left - 60)
        left_ranges = msg.ranges[index_left_start:index_left_end + 1]
        try:
            left_distance = statistics.mean([x for x in left_ranges if x != float('inf')])
        except statistics.StatisticsError:
            left_distance = 0

        index_right = round((-3.1416 - angle_min) / angle_increment)
        index_right_start = max(0, index_right + 60)
        index_right_end = min(len(msg.ranges) - 1, index_right + 120)
        right_ranges = msg.ranges[index_right_start:index_right_end + 1]
        try:
            right_distance = statistics.mean([x for x in right_ranges if x != float('inf')])
        except statistics.StatisticsError:
            right_distance = 0

        self.get_logger().info(f'Front distance: {front_distance}')
        self.get_logger().info(f'Left distance: {left_distance}')
        self.get_logger().info(f'Right distance: {right_distance}')

        if (left_distance < 0.3):
            self.twist.linear.x = 0.0
            self.turn_direction = -1
            self.twist.angular.z = math.pi / 6 * self.turn_direction  # 45 Grad in Rad
            if (front_distance < 0.4):
                self.twist.angular.z = math.pi / 4 * self.turn_direction
        elif (right_distance < 0.3):
            self.twist.linear.x = 0.0
            self.turn_direction = +1
            self.twist.angular.z = math.pi / 6 * self.turn_direction
            if (front_distance < 0.4):
                self.twist.angular.z = math.pi / 4 * self.turn_direction
        elif (front_distance < 0.5):
            self.twist.linear.x = 0.0
            self.twist.angular.z = math.pi
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
            
        self.publisher_cmd_vel.publish(self.twist)
        # \\\--------주행 추가 front(15도 시야), left(30도 시야), right(30도 시야)--------///
        
        # (기존 주행 알고리즘)
        # if front_distance < 0.7:
        #     self.twist.linear.x = 0.0
        #     self.twist.angular.z = math.pi / 4 * self.turn_direction
           
        #     self.turn_count += 1
           
        #     if self.turn_count >= 100:
        #         self.turn_direction *= -1
        #         self.turn_count = 0
        # else:
        #     self.twist.linear.x = 0.2
        #     self.twist.angular.z = 0.0

        
        # Update the path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x  # Set the actual x position of the robot
        pose.pose.position.y = self.y  # Set the actual y position of the robot

        self.path.poses.append(pose)
        self.publisher_path.publish(self.path)

        



    # 추가추가
    def finish2drive(self):
        if(qrcamera.flag is True):
            print("Return - Initial point")
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"   
            goal_pose.pose.position.x = self.initial_x
            goal_pose.pose.position.y = self.initial_y
            goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # default orientation
            self.send_navigation_goal(goal_pose)


    
    # 추가
    def send_navigation_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.nav_to_pose_client.wait_for_server()
        self.nav_to_pose_client.send_goal_async(goal_msg)



if __name__ == '__main__':
    rclpy.init()
    robotExplorer = RobotExplorer()
    #imageprocessor = ImageProcessor()
    #executor = rclpy.executors.MultiThreadedExecutor()

    # executor.add_node(robotExplorer)
    #executor.add_node(imageprocessor)
    rclpy.spin(robotExplorer)

    rclpy.shutdown()



