#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorState, IngestorResult
import time

class DispenserNode(Node):
    def __init__(self):
        super().__init__(node_name="ingestor_node")
        self.create_subscription(IngestorRequest, "/ingestor_requests", self.request_cb, 10)
        self.state_pub = self.create_publisher(IngestorState, "/ingestor_states", 10)
        self.result_pub = self.create_publisher(IngestorResult, "/ingestor_results", 10)
        self.declare_parameter('handle_name', "tool_ingestor")
        self.handle_name = self.get_parameter('handle_name').get_parameter_value().string_value
        self.get_logger().info(f"{self.handle_name} station handle initialize")
        self.state = IngestorResult.ACKNOWLEDGED
        self.startTime = time.time()
    
    def request_cb(self, msg: IngestorRequest):
        if msg.target_guid == self.handle_name:
            self.get_logger().info(f"{self.handle_name} ingest tool")
            if self.state == IngestorResult.ACKNOWLEDGED:
                result_msg = IngestorResult()
                result_msg.time.sec = int(time.time() - self.startTime)
                result_msg.request_guid = msg.request_guid
                result_msg.source_guid = msg.target_guid
                result_msg.status = IngestorResult.ACKNOWLEDGED
                self.result_pub.publish(result_msg)
                result_msg.status = IngestorResult.SUCCESS
                self.result_pub.publish(result_msg)
                self.state = IngestorResult.SUCCESS
            elif self.state == IngestorResult.SUCCESS:
                result_msg = IngestorResult()
                result_msg.time.sec = int(time.time() - self.startTime)
                result_msg.status = IngestorResult.SUCCESS
                result_msg.request_guid = msg.request_guid
                result_msg.source_guid = msg.target_guid
                self.result_pub.publish(result_msg)
                self.state = IngestorResult.ACKNOWLEDGED
            
def main(args=None):
    rclpy.init(args=args)
    node = DispenserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
