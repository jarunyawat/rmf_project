#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, DockSummary, RobotMode
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
import math
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

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
        self.initial_pose = None
        self.initial_pose_received = False
        self.offset = [0, 0]
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        # Create publisher subscriber
        self.robot_status_pub = self.create_publisher(RobotState, "robot_state", 10)
        self.path_pub = self.create_subscription(PathRequest, 'robot_path_requests', self._path_request_cb, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        # Action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
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
        cur_loc = Location()
        cur_loc.x = t.transform.translation.x
        cur_loc.y = t.transform.translation.y
        (roll, pitch, yaw) = self._get_euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        cur_loc.yaw = yaw
        self.robot_status.location = cur_loc
        self.robot_status_pub.publish(self.robot_status)

    def setInitialPose(self, initial_pose: PoseStamped):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.get_logger().info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                str(pose.pose.position.y) + '...')
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._goToPose_feedbackCallback)
        self._send_goal_future.add_done_callback(self.goToPose_response_callback)
        
    def goToPose_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goToPose rejected :(')
            return

        self.get_logger().info('goToPose accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goToPose_result_callback)

    def goToPose_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))
    
    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if localizer == 'amcl':
            self._waitForInitialPose()
        self._waitForNodeToActivate(navigator)
        self.get_logger().info('Nav2 is ready for use!')
        return
    
    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.get_logger().info('Setting initial pose')
            self._setInitialPose()
            self.get_logger().info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
    
    def _amclPoseCallback(self, msg):
        self.get_logger().info('Received amcl pose')
        self.initial_pose_received = True
        return
    
    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.get_logger().info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return
        
    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.get_logger().info(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.get_logger().info(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().info(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _goToPose_feedbackCallback(self, msg):
        # self.get_logger.info('Received action feedback message')
        self.feedback = msg.feedback
        return

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorBridge("navigatorAPI")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()