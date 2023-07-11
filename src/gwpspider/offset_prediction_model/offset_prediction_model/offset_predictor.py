import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python import get_package_share_directory

import os
from gwpoffpm import offset_predictor
import numpy as np

from std_msgs.msg import Float32MultiArray

from gwpspider_interfaces.srv import GetCorrectionOffset
from gwpspider_interfaces import gwp_interfaces_data as gid

class OffsetPredictor(Node):
    def __init__(self):
        Node.__init__(self, 'offset_predictor')
        model_dir = os.path.join(get_package_share_directory('offset_prediction_model'), 'resource/model.pth')
        self.prediction_model = offset_predictor.OffsetPredictor(model_dir)

        self.callback_group = ReentrantCallbackGroup()
        self.create_service(GetCorrectionOffset, gid.GET_CORRECTION_OFFSET_SERVICE, self.get_correction_offset_callback, callback_group = self.callback_group)
    
    def get_correction_offset_callback(self, request, response):
        legs_positions = request.legs_positions.data
        rpy = request.rpy.data
        commanded_position = request.commanded_position.data
        one_hot_legs = request.one_hot_legs.data

        prediction_input_data = np.concatenate((legs_positions, rpy, commanded_position, one_hot_legs), axis = 0).astype(np.float64)
        offset = np.array(self.prediction_model(prediction_input_data), dtype = np.float32)
        print(offset)

        response.correction_offset = Float32MultiArray(data = offset)

        return response

def main():
    rclpy.init()
    app = OffsetPredictor()
    rclpy.spin(app)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
