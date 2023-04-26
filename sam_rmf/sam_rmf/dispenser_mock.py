#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rmf_dispenser_msgs.msg import DispenserRequest, DispenserState, DispenserResult
import time

class DispenserNode(Node):
    def __init__(self):
        super().__init__(node_name="dispenser_node")
        self.create_subscription(DispenserRequest, "/dispenser_requests", self.request_cb, 10)
        self.state_pub = self.create_publisher(DispenserState, "/dispenser_states", 10)
        self.result_pub = self.create_publisher(DispenserResult, "/dispenser_results", 10)
        self.declare_parameter('handle_name', "dispenser_dropoff")
        self.handle_name = self.get_parameter('handle_name').get_parameter_value().string_value
        self.get_logger().info(f"{self.handle_name} station handle initialize")
        self.state = DispenserResult.ACKNOWLEDGED
        self.startTime = time.time()
    
    def request_cb(self, msg: DispenserRequest):
        if msg.target_guid == self.handle_name:
            self.get_logger().info(f"{self.handle_name} dispens tool")
            if self.state == DispenserResult.ACKNOWLEDGED:
                result_msg = DispenserResult()
                result_msg.time.sec = int(time.time() - self.startTime)
                result_msg.request_guid = msg.request_guid
                result_msg.source_guid = msg.target_guid
                result_msg.status = DispenserResult.ACKNOWLEDGED
                self.result_pub.publish(result_msg)
                result_msg.status = DispenserResult.SUCCESS
                self.result_pub.publish(result_msg)
                self.state = DispenserResult.SUCCESS
            elif self.state == DispenserResult.SUCCESS:
                result_msg = DispenserResult()
                result_msg.time.sec = int(time.time() - self.startTime)
                result_msg.status = DispenserResult.SUCCESS
                result_msg.request_guid = msg.request_guid
                result_msg.source_guid = msg.target_guid
                self.result_pub.publish(result_msg)
                self.state = DispenserResult.ACKNOWLEDGED
            
def main(args=None):
    rclpy.init(args=args)
    node = DispenserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
