# Contains code for the connection layer between the ROS2 backend and the GUI frontend

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np

from fastapi import FastAPI

class FastAPINode(Node):
    """FastAPI class.
    Args:
        Node (Node): ROS node.
    """
        
    def __init__(self):
        super().__init__('fastapi_node')
        self.app = FastAPI()

        #@self.app.get("/")
        #async def read_root():
            #return {"Hello": "ROS 2 FastAPI Node"}

        #self.create_timer(1, self.publish_hello)

    #def publish_hello(self):
        #self.get_logger().info("Publishing 'Hello'")

        self.get_logger().info("FastAPI is running.")

def main():
    #rclpy.init()
    #node = FastAPINode()
    #try:
        #rclpy.spin(node)
    #except KeyboardInterrupt:
        #pass
    #finally:
        #node.destroy_node()
        #rclpy.shutdown()

    rclpy.init()
    fastapi_node = FastAPINode()
    executor = MultiThreadedExecutor()
    rclpy.spin(fastapi_node, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
