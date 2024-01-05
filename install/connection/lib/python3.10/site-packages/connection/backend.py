#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from rclpy.executors import MultiThreadedExecutor
from pydantic import BaseModel
from fastapi.templating import Jinja2Templates
import subprocess
import os
import uvicorn
import threading

app = FastAPI()

class GripperCommand(BaseModel):
    gripper_state: float

class BackendNode(Node):
    def __init__(self):
        super().__init__('backend_node')
        #app = FastAPI()
        #self.app = FastAPI()

        # Enable CORS
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # Allow all origins, adjust as needed
            allow_credentials=True,
            allow_methods=["*"],  # Allow all HTTP methods
            allow_headers=["*"],  # Allow all headers
        )

        # Initialize gripper state
        self.gripper_state = "unknown"

        start_fastapi_server()

        @app.get("/")
        async def read_root():
            print("Root endpoint accessed!")
            return {"message": "Hello, this is the root endpoint!"}


        #@self.app.get("/")
        @app.get("/")
        async def read_gripper_state():
            return {"gripper_state": self.get_gripper_state()}

        #@self.app.post("/set_gripper_state")
        @app.post("/set_gripper_state")
        async def set_gripper_state(gripper_command: GripperCommand):
            self.set_gripper_state(gripper_command.gripper_state)
            return {"gripper_state": self.get_gripper_state()}

        # Start the FastAPI server using uvicorn
        #self.start_fastapi_server()

    def get_gripper_state(self):
        return self.gripper_state

    def set_gripper_state(self, gripper_state):
        self.gripper_state = gripper_state
        self.get_logger().info(f"Gripper State updated to: {gripper_state}")

def start_fastapi_server():
    # Use FastAPI static files middleware to serve files from the 'static' folder
    static_folder_path = os.path.join(os.getcwd(), 'index.html')
    #self.get_logger().info(f"{static_folder_path}")
    #app.mount("/static", StaticFiles(directory="/home/spider/gwpspider_ws/src/gwpspider/connection/connection/static"), name="static")
     # Use uvicorn.run to start the FastAPI server
    uvicorn.run("connection.backend:app", host="0.0.0.0", port=5000, reload=True)

    # Use uvicorn.run to start the FastAPI server
    #uvicorn.run("connection.backend:app", host="0.0.0.0", port=8000, reload=True)


def main():
    rclpy.init()
    backend = BackendNode()
    #rclpy.spin(backend)

    spin_thread = threading.Thread(target=rclpy.spin, args=(backend,))
    spin_thread.start()
    #start_fastapi_server(backend)
    #uvicorn.run("connection.backend:app", port=5000, log_level='warning', reload = True, host = "0.0.0.0")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
