#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

class GripperCommand(BaseModel):
    gripper_state: float

class BackendNode(Node):
    def __init__(self):
        super().__init__('backend_node')
        self.app = FastAPI()

        # Enable CORS
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # Allow all origins, adjust as needed
            allow_credentials=True,
            allow_methods=["*"],  # Allow all HTTP methods
            allow_headers=["*"],  # Allow all headers
        )

        # Initialize gripper state
        self.gripper_state = "unknown"

        @self.app.get("/")
        async def read_gripper_state():
            return {"gripper_state": self.get_gripper_state()}

        @self.app.post("/set_gripper_state")
        async def set_gripper_state(gripper_command: GripperCommand):
            self.set_gripper_state(gripper_command.gripper_state)
            return {"gripper_state": self.get_gripper_state()}

    def get_gripper_state(self):
        return self.gripper_state

    def set_gripper_state(self, gripper_state):
        self.gripper_state = gripper_state
        self.get_logger().info(f"Gripper State updated to: {gripper_state}")


def main():
    rclpy.init()
    backend_node = BackendNode()
    rclpy.spin(backend_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
