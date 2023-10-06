import serial

class ArduinoComm():
    """Parent class for communication with Arduinos.
    """
    def __init__(self, device: str, read_data_length: int):
        """Initiate serial communication with Arduino.

        Args:
            device (str): Name of Arduino device.
            read_data_length (int): Length of data, that will be constantly comming from Arduino.
        """
        self.comm = serial.Serial(f'/dev/{device}', baudrate = 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.data_length = read_data_length

    def read(self) -> str:
        """Read data comming from Arduino.

        Returns:
            str: Data from Arduino.
        """
        data = self.comm.readline()
        while len(data) != self.data_length:
            data = self.comm.readline()
        data = data.decode('utf-8', errors = 'ignore').rstrip()

        return data

    def write(self, msg: str):
        """Write data to Arduino.

        Args:
            msg (str): Data to be written.
        """
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        self.comm.write(msg)