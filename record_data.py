import serial
import struct

class Teensy:
    
    def __init__(self, com_port, frame_delim = '$', debug = 0, msg_format = "f f f f f f f f f", msg_size = 36) -> None: #debug levels 0-3, higher means more printing
        self.serial = serial.Serial(port=com_port, baudrate=115200, timeout=1) #Serial Object of teensy
        self.frame_delim = frame_delim
        self.msg_size = msg_size #total message size, not including delimiter, in bytes
        self.msg_format = msg_format
        self.end_msg_buf = None #buffer for capturing partial messages
        self.partial_msg_counter = 0 #counter for debugging
        self.debug = debug
        
    def send_msg(self, data): #send bytes, same size as receive
        if self.debug >= 2:
            print(f"In function: send_msg(), data = {data}")
        new_data = data
        if len(data) != 9: #ensure data is 9 floats long
            if self.debug >= 3:
                print(f"Appending {9 - len(data)} floats to fit formatting")
            for i in range(9 - len(data)):
                new_data.append(0.0)

        snd_bytes = struct.pack(self.msg_format, new_data) #convert floats to bytes
        if self.debug >= 3:
            print(f"Float to bytes results in {snd_bytes.hex()}")
        self.serial.write(snd_bytes) #write to serial
        if self.debug:
            print("Data Sent")

    
    def read_input_buf(self):
        if self.debug >= 2:
            print(f"In function: read_input_buf()")
            print(f"There are {self.serial.in_waiting} in read buffer")
        all_data = self.serial.read(self.serial.in_waiting) #read all data in the buffer in bytes
        if self.debug >= 3:
            print(f"Input Buffer read: {all_data.hex()}")
        all_msgs = [] #array for all bytes messages
        cur_msg = self.end_msg_buf if self.end_msg_buf != None else [] #use the last read operation's leftover message bytes if exisiting
        if self.debug >= 3:
            print(f"Leftover bytes from last read: {cur_msg}")
        for byte in all_data:
            if str(byte) == self.frame_delim: 
                if len(cur_msg) == self.msg_size:
                    all_msgs.append(cur_msg) #if message is correct size once frame delimiter is reached, save message
                else:
                    cur_msg = [] #if not, throw away bytes
                    self.partial_msg_counter += 1
            else:
                cur_msg.append(int(byte)) #keep adding bytes until frame delimiter
        
        if len(cur_msg) == self.msg_size: #for last bytes, if no frame delimiter is reached but is right size, save message
            all_msgs.append(cur_msg)
            self.end_msg_buf = None #set end buffer to None, as message was saved
        elif len(cur_msg) == 0:
            self.end_msg_buf = None
        else:
            self.end_msg_buf = cur_msg #set buffer to current message, so next read operation can use it
        
        datapoints = []
        for msg in all_msgs:
            msg_bytes = bytes(msg) #transform int objects back to bytes
            datapoints.append(self.parse_datapoint(msg_bytes)) #parse the datapoint and add to all points
            
        return datapoints
            
            
    def parse_datapoint(self, msg):
        datapoint = []
        for i in range(0, self.msg_size, 4):
            datapoint.append(struct.unpack('<f', msg[i:i+4])) #parse each 4 bytes into a float
        return datapoint
            
    def flush_buffer(self):
        self.serial.read_all() #flush buffer by reading all bytes in it, but not using it
        
        

class CSVWriter:
    
    def __init__(self, path):
        self.path = path
        self.file = open(self.path, 'x')
        
        
    def close_file(self):
        self.file.close()
    
    def write_line(self, data):
        
        for item in data[:-1]:
            self.file.write(str(item) + ',')
            
        self.file.write(data[-1] + '\n')
        
import matplotlib.pyplot as plt
import pygame

if __name__ == "__main__":
    