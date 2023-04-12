#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, DockSummary, RobotMode
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from action_msgs.msg import GoalStatus

class NavigatorBridge(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        # Declare parameter
        self.declare_parameter('robot_name', "")
        self.declare_parameter('x_pos', 0.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        x_init_pos = self.get_parameter('x_pos').get_parameter_value().double_value
        y_init_pos = self.get_parameter('y_pos').get_parameter_value().double_value
        yaw_init_pos = self.get_parameter('yaw').get_parameter_value().double_value
        self.offset = [0, 0]
        # Create publisher subscriber
        self.robot_status_pub = self.create_publisher(RobotState, "robot_state", 10)
        self.path_pub = self.create_subscription(PathRequest, 'robot_path_requests', self._path_request_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        # Status parameter
        self.robot_status = RobotState()
        self.robot_status.battery_percent = 1.0
        self.robot_status.name = self.robot_name
        self.robot_status.mode.mode = RobotMode.MODE_CHARGING
        # Set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = x_init_pos
        initial_pose.pose.position.y = y_init_pos
        q = self._get_quaternion_from_euler(0, 0, yaw_init_pos)
        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]
        self.setInitialPose(initial_pose)
        self.waitUntilNav2Active()

    def _get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def _get_euler_from_quaternion(self, qx, qy, qz, qw):
        t0 = +2.0 * (qw * qx + qy * qz)
        t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def _path_request_cb(self, msg: PathRequest):
        if msg.robot_name == self.robot_name:
            if abs(msg.path[0].x - msg.path[1].x)<0.001 and abs(msg.path[0].y - msg.path[1].y)<0.001 and abs(msg.path[0].yaw - msg.path[1].yaw)<0.001:
                self.robot_status.mode.mode = RobotMode.MODE_PAUSED
                self.robot_status.path = []
                # self.cancelTask()
            else:
                self.robot_status.mode.mode = RobotMode.MODE_MOVING
                self.robot_status.path = []
                self.robot_status.path.append(msg.path[0])
                self.robot_status.path.append(msg.path[1])
                self.robot_status.task_id = msg.task_id
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = msg.path[1].x
                goal_pose.pose.position.y = msg.path[1].y
                q = self._get_quaternion_from_euler(0, 0, msg.path[1].yaw)
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]
                self.goToPose(goal_pose)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_footprint to map: {ex}')
            return
        self.get_logger().info("loop timer")
        cur_loc = Location()
        cur_loc.x = t.transform.translation.x
        cur_loc.y = t.transform.translation.y
        (roll, pitch, yaw) = self._get_euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        cur_loc.yaw = yaw
        self.robot_status.location = cur_loc
        self.robot_status_pub.publish(self.robot_status)

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorBridge("navigatorAPI")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()