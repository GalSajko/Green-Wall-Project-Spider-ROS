#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import subprocess
import os
import uvicorn
import threading

from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
from fastapi import Request

app = FastAPI()

class FastAPINode(Node):
    def __init__(self):
        super().__init__('fastapi_node')
        #self.app = FastAPI()

        """
        GripperCommand = gwp_msgs.GripperCommand
        GripperCommand.leg_id = 1

        @self.app.get("/")
        async def read_gripper_state():
            return {"gripper_state": self.get_gripper_state()}

        @self.app.post("/set_gripper_state")
        async def set_gripper_state(gripper_state: GripperCommand.command):
            self.set_gripper_state(gripper_state.gripper_state)
            return {"gripper_state": self.get_gripper_state()}
        """
        # Enable CORS
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # Allow all origins, adjust as needed
            allow_credentials=True,
            allow_methods=["*"],  # Allow all HTTP methods
            allow_headers=["*"],  # Allow all headers
        )


        # Start the FastAPI server using uvicorn
        self.start_fastapi_server()


    """
    def get_gripper_state(self):
        return self.gripper_state

    def set_gripper_state(self, gripper_state):
        self.gripper_state = gripper_state
        self.get_logger().info(f"Gripper State updated to: {gripper_state}")
    """

    def start_fastapi_server(self):
        # Get the current working directory
        current_directory = os.getcwd()
        # Serve static files (CSS and JavaScript)
        app.mount("/static", StaticFiles(directory="static"), name="static")

        # Jinja2 templates configuration
        templates = Jinja2Templates(directory="templates")

        """"
        # Specify the path to the FastAPI file (modify if needed)
        fastapi_file_path = os.path.join(current_directory, 'fastapi.py')

        # Command to start uvicorn with auto-reload
        command = f"uvicorn {fastapi_file_path}:app --reload"

        # Run the command in a subprocess
        subprocess.Popen(command, shell=True)
        """
        @app.get("/", response_class=HTMLResponse)
        def read_root(request: Request):
            return templates.TemplateResponse("index.html", {"request": request})

        uvicorn.run("connection.fastapi:app", host="0.0.0.0", port=5000, reload=True)

def main():
    rclpy.init()
    fastapi = FastAPINode()
    #executor = MultiThreadedExecutor()
    #rclpy.spin(fastapi_node, executor)
    spin_thread = threading.Thread(target=rclpy.spin, args=(fastapi,))
    spin_thread.start()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
