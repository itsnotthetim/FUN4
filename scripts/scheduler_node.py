#!/usr/bin/python3

from fun4.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from controller_mode_interface.srv import ControllerMode

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        self.mode_pub_ = self.create_publisher()

        self.chmod_ = self.create_service(ControllerMode,'/change_mode',self.chmod_server_callback)

    def chmod_server_callback(self,request,respond):
        request

def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
