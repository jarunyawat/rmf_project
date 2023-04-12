#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import PathRequest, Location

class RequestPathTest(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.path_pub = self.create_publisher(PathRequest, 'robot_path_requests', 10)
        self.request_period = self.create_timer(3.0, self.path_request_cb)
    
    def path_request_cb(self):
        path_request = PathRequest()
        path_request.robot_name = "robot_1"
        path_request.path = []
        # Appending the current location twice will effectively tell the
        # robot to stop
        start = Location()
        goal = Location()
        goal.x = -4.0
        goal.y = 0.0
        path_request.path.append(start)
        path_request.path.append(goal)
        self.path_pub.publish(path_request)

def main(args=None):
    rclpy.init(args=args)
    node = RequestPathTest("path_request_test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()