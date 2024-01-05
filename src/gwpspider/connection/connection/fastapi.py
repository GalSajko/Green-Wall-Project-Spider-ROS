#!/usr/bin/env python3

import threading
import json
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import uvicorn

from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
from fastapi import Request

app = FastAPI()

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins, adjust as needed
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)

# Serve the static files
app.mount("/static", StaticFiles(directory="/home/spider/gwpspider_ws/src/gwpspider/connection/connection/static"), name="static")
#app.mount("/static", StaticFiles(directory="connection.static"), name="static")

# Jinja2 templates configuration
templates = Jinja2Templates(directory="/home/spider/gwpspider_ws/src/gwpspider/connection/connection/templates")

# Variable to track work status
work_status = None
stop_status = None

# Initialize form data variables as None
x1 = None
y1 = None
z1 = None
coordSystem1 = None
offset1 = None

# Initialize variables for move body form data as None
x2 = None
y2 = None
z2 = None
coordSystem2 = None
offset2 = None

# Initialize gripper variables as None
gripperIdx = None
gripperAction = None

# Initialize leg variables as None
legIdx = None
x4 = None
y4 = None
z4 = None
coordSystem4 = None
offset4 = None

# Initialize refill variable as None
refill = None

# Initialize pump variables as None
pumpIdx = None
pumpAction = None

# Variables to track breaks status
breaksAct = None
breaksDeact = None

# JSON path
json_file_path = "/home/spider/gwpspider_ws/src/gwpspider/data/spider_state_dict"


# All the variables used in the GUI and all the endpoints have to be defined outside of main, otherwise, they aren't accessed!!!


@app.get("/", response_class=HTMLResponse)
def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})



#try:
with open(json_file_path, 'r') as json_file:
    # Load JSON data from the file
    json_data = json.load(json_file)
    #print(json_data)
    """
    except FileNotFoundError:
        self.get_logger().error(f'File not found: {file_path}')
    except json.JSONDecodeError:
        self.get_logger().error(f'Error decoding JSON file: {file_path}')
    except Exception as e:
        self.get_logger().error(f'Error reading JSON file: {str(e)}')
    """
"""
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        # Replace this with the actual logic to get and send your JSON data

        # Send JSON data to the connected WebSocket client
        await websocket.send_text(json.dumps(json_data))
"""

@app.post("/start_working")
def start_working():
    global work_status
    work_status = 1
    print(f"Work status: {work_status}")  # Print the work status to the terminal
    return {"message": "Started working"}

@app.post("/stop_working")
def stop_working():
    global stop_status
    stop_status = 1
    print(f"Stop status: {stop_status}")  # Print the stop status to the terminal
    return {"message": "Stopped working"}

@app.post("/process_walk_to_position")
def process_walk_to_position(data: dict):
    global x1, y1, z1, coordSystem, offset

    # Store form data in variables
    x1 = data.get("x1")
    y1 = data.get("y1")
    z1 = data.get("z1")
    coordSystem = data.get("coordSystem")
    offset = data.get("offset")

    print("Received form data:")
    print("x1:", x1)
    print("y1:", y1)
    print("z1:", z1)
    print("coordSystem:", coordSystem)
    print("offset:", offset)

    # You can perform further processing or store the values as needed

    return {"message": "Form data received successfully"}

@app.post("/process_move_body")
def process_move_body(data: dict):
    global x2, y2, z2, coordSystem2, offset2

    # Store form data in variables
    x2 = data.get("x2")
    y2 = data.get("y2")
    z2 = data.get("z2")
    coordSystem2 = data.get("coordSystem2")
    offset2 = data.get("offset2")

    print("Received form data:")
    print("x2:", x2)
    print("y2:", y2)
    print("z2:", z2)
    print("coordSystem2:", coordSystem2)
    print("offset2:", offset2)

    # You can perform further processing or store the values as needed

    return {"message": "Form data received successfully"}

@app.post("/process_gripper_action")
def process_gripper_action(data: dict):
    global gripperIdx, gripperAction

    # Store form data in variables
    gripperIdx = data.get("gripperIdx")
    gripperAction = data.get("gripperAction")

    print("Received gripper action data:")
    print("gripperIdx:", gripperIdx)
    print("gripperAction:", gripperAction)

    # You can perform further processing or store the values as needed

    return {"message": "Gripper action data received successfully"}

@app.post("/process_move_leg")
def process_move_leg(data: dict):
    global legIdx, x4, y4, z4, coordSystem4, offset4

    # Store form data in variables
    legIdx = data.get("legIdx")
    x4 = data.get("x4")
    y4 = data.get("y4")
    z4 = data.get("z4")
    coordSystem4 = data.get("coordSystem4")
    offset4 = data.get("offset4")

    print("Received move leg data:")
    print("legIdx:", legIdx)
    print("x4:", x4)
    print("y4:", y4)
    print("z4:", z4)
    print("coordSystem4:", coordSystem4)
    print("offset4:", offset4)

    # You can perform further processing or store the values as needed

    return {"message": "Move leg data received successfully"}

@app.post("/process_go_refill")
def process_go_refill(data: dict):
    global refill

    # Store form data in variables
    refill = data.get("refill")

    print("Received refill data:")
    print("refill:", refill)

    # You can perform further processing or store the value as needed

    return {"message": "Refill data received successfully"}

@app.post("/process_pump_action")
def process_pump_action(data: dict):
    global pumpIdx, pumpAction

    # Store form data in variables
    pumpIdx = data.get("pumpIdx")
    pumpAction = data.get("pumpAction")

    print("Received pump action data:")
    print("pumpIdx:", pumpIdx)
    print("pumpAction:", pumpAction)

    # You can perform further processing or store the values as needed

    return {"message": "Pump action data received successfully"}

@app.get("/activate_breaks")
def activate_breaks():
    global breaksAct
    breaksAct = 1
    print("Breaks activated:", breaksAct)
    return {"message": "Breaks activated successfully"}

@app.get("/deactivate_breaks")
def deactivate_breaks():
    global breaksDeact
    breaksDeact = 1
    print("Breaks deactivated:", breaksDeact)
    return {"message": "Breaks deactivated successfully"}

class FastAPINode(Node):
    def __init__(self):
        super().__init__('fastapi_node')

        # Start the FastAPI server using uvicorn
        start_fastapi_server()


def start_fastapi_server():
    uvicorn.run("connection.fastapi:app", host="0.0.0.0", port=8000, reload=True)



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
