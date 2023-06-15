import serial

class ArduinoComm():
    def __init__(self, device, read_data_length):
        self.comm = serial.Serial(f'/dev/{device}', baudrate = 115200, timeout = 0)
        self.comm.reset_input_buffer()

        self.data_length = read_data_length

    def read(self):
        data = self.comm.readline()
        while len(data) != self.data_length:
            data = self.comm.readline()
        data = data.decode('utf-8', errors = 'ignore').rstrip()

        return data

    def write(self, msg):
        if msg[-1] != '\n':
            msg += '\n'
        msg = msg.encode("utf-8")
        self.comm.write(msg)