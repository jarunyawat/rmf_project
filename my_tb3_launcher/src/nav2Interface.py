#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel
import threading
import socket
import sys
import math
import numpy as np
import enum

class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2

class Request(BaseModel):
    map_name: Optional[str] = None
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str

app = FastAPI()

class Nav2Interface(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Get status of robot
        self.robot_name = "robot_1"
        self.level_name = "L1"
        self.battery_level = 1.0*100
        self.robot_state = RobotState.IDLE
        self.cur_position = [0.0, 0.0]
        self.destination = None
        self.last_completed_request = False
        self._get_result_future = None
        self.nominal_velocity = 0.22
        self.nominal_rotation_velocity = 2.84

        # Get status
        @app.get('/status/', response_model=Response)
        async def status():
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }
            response['data'] = self.get_robot_state()
            response['success'] = True
            # self.get_logger().info(f"robot data {response}")
            return response

        # Send navigation GOAL
        @app.post('/navigate/', response_model=Response)
        async def navigate(dest: Request):
            response = {'success': False, 'msg': ''}

            target_x = dest.destination['x']
            target_y = dest.destination['y']

            target_loc = PoseStamped()
            target_loc.header.frame_id = 'map'
            target_loc.header.stamp = self.get_clock().now().to_msg()
            target_loc.pose.position.x = target_x
            target_loc.pose.position.y = target_y
            angle = np.arctan2(target_y-self.cur_position[1], target_x-self.cur_position[0])
            q = self._get_quaternion_from_euler(0, 0, angle)
            target_loc.pose.orientation.x = q[0]
            target_loc.pose.orientation.y = q[1]
            target_loc.pose.orientation.z = q[2]
            target_loc.pose.orientation.w = q[3]
            self.last_completed_request = False
            self.goToPose(target_loc)
            self.destination = target_loc

            response['success'] = True
            return response

        # Send stop robot command
        @app.get('/stop_robot/', response_model=Response)
        async def stop():
            response = {'success': False, 'msg': ''}
            self.cancelTask()
            response['success'] = True
            return response
    
    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree
        self.destination = pose

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                str(pose.pose.position.y) + '...')
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._goToPose_feedbackCallback)
        self._send_goal_future.add_done_callback(self.goToPose_response_callback)
    
    def goToPose_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('goToPose rejected :(')
            self.destination = None
            self.robot_state = RobotState.IDLE
            return
        self.robot_state = RobotState.MOVING
        self.get_logger().info('goToPose accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goToPose_result_callback)

    def goToPose_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("reach GOAL")
        self.last_completed_request = True
        self.destination = None
        self.robot_state = RobotState.IDLE
        # self.get_logger().info('Result: {0}'.format(result.sequence))
    
    def _goToPose_feedbackCallback(self, msg):
        # self.get_logger.info('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def cancelTask(self):
        """Cancel pending task request of any type."""
        if self._get_result_future:
            self.get_logger().info('Canceling current task.')
            future = self.goal_handle.cancel_goal_async()
            self.last_completed_request = True
            self.destination = None
            self.robot_state = RobotState.IDLE
        return

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
    
    def get_robot_state(self):
        data = {}
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_footprint to map: {ex}')
            return data
        position = [t.transform.translation.x, t.transform.translation.y]
        self.cur_position = position
        (roll, pitch, yaw) = self._get_euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        angle = yaw
        data['robot_name'] = self.robot_name
        data['map_name'] = self.level_name
        data['position'] = {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = self.battery_level
        if (self.destination):
            destination = self.destination

            # calculate arrival estimate
            dist_to_target = self.disp(position, [destination.pose.position.x, destination.pose.position.y])
            (roll, pitch, des_yaw) = self._get_euler_from_quaternion(destination.pose.orientation.x, destination.pose.orientation.y, destination.pose.orientation.z, destination.pose.orientation.w)
            ori_delta = abs(abs(angle) - abs(des_yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (dist_to_target /
                        self.nominal_velocity +
                        ori_delta /
                        self.nominal_rotation_velocity)
            data['destination_arrival'] = {
                'duration': duration
            }
        else:
            data['destination_arrival'] = None
        data['last_completed_request'] = self.last_completed_request
        if (self.robot_state == RobotState.WAITING):
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data['replan'] = True
        else:
            data['replan'] = False
        return data
    
    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)


    interface = Nav2Interface("nav2_api_interface")

    spin_thread = threading.Thread(target=rclpy.spin, args=(interface,))
    spin_thread.start()

    hostname=socket.gethostname()   
    IPAddr=socket.gethostbyname(hostname)  
    uvicorn.run(app,
                host=IPAddr,
                port=5000,
                log_level='warning')


if __name__ == '__main__':
    main(sys.argv)