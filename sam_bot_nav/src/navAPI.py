import rclpy
from fastapi import FastAPI
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import 

def main():
    app = FastAPI()
    rclpy.init()
    robot1_navigator = BasicNavigator()

    @app.get('/')
    async def test():
        return {"msg": "hello world"}
    
    @app.get("/navigate")
    async def naviagate_to_pose_api():
        response = {'success': False, 'msg': ''}

        # target_x = dest.destination['x']
        # target_y = dest.destination['y']
        # target_yaw = dest.destination['yaw']
        # target_speed_limit = dest.speed_limit
        target_x = -4.0
        target_y = 0.0
        target_yaw = 1.0

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_x
        goal_pose.pose.position.y = target_y
        q = self._quaternion_from_euler(0, 0, target_yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.goToPose(goal_pose)

        # t = self.get_clock().now().to_msg()
        # disp = self.disp([target_x, target_y], [cur_x, cur_y])
        # duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
        #     int(abs(abs(cur_yaw) - abs(target_yaw)) /
        #         self.vehicle_traits.rotational.nominal_velocity)
        response['success'] = True
        return response

if __name__ == '__main__':
    main()
