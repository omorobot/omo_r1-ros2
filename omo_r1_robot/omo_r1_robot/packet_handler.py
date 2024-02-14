#!/usr/bin/env python3
# import io
import serial
from rclpy.logging import get_logger

class PacketHandler:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 0.1
        self.ser.open()
        if self.ser.is_open:
            self.print(f'serial port {self.ser.name} is opened')
        else:
            self.print('serial port open error')
            raise Exception('serial port open error')
        # self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1), newline = '\r', line_buffering = True)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.incoming_data = ['ODO', 'POSE', 'VW'] # ['GYRO', 'ODO', 'POSE', 'VW']
        # self._bat = [0.0, 0.0, 0.0]
        # self._gyro = [0.0, 0.0, 0.0]
        self._odo = [0.0, 0.0]
        self._pose = [0.0, 0.0, 0.0]
        self._vw = [0.0, 0.0]
        self.print('serial init complete')

    def print(self, str_info):
        get_logger('robot_driver').info(str_info)

    def write_data(self, tx_string):
        if self.ser.writable():
            self.ser.write((tx_string + '\r\n').encode())

    def close_port(self):
        self.write_data('$cPEEN,0')
        self.ser.close()
        self.print('serial port close')

    # def get_battery_state(self):
    #     self.write_data('$qBAT')

    # def odo_reset(self):
    #     self.write_data('$cODO,0')

    def vw_command(self, lin_vel, ang_vel):
        self.write_data('$cVW,{:.0f},{:.0f}'.format(lin_vel, ang_vel))

    def set_periodic_info(self, millisecond):
        for idx, item in enumerate(self.incoming_data):
            self.write_data('$cREGI,' + str(idx) + ',' + item)
            self.print(str(self.ser.readline()))
        self.write_data('$cPERI,' + str(millisecond))
        self.print(str(self.ser.readline()))
        self.write_data('$cPEEN,1')
        self.print(str(self.ser.readline())) # self.print("b'#PEEN,1\\r\\n'")
        self.print('set periodic info complete')

    def read_packet(self):
        if self.ser.readable():
            whole_packet = (self.ser.readline().split(b'\r')[0]).decode('utf-8').strip()
            # print(whole_packet)
            packet = whole_packet.split(',')
            try:
                header = packet[0].split('#')[1]
                if header.startswith('ODO'):                                                # omdometer
                    self._odo = [float(packet[1]), float(packet[2])]
                    self._odo_flag = True
                elif header.startswith('POSE'):                                             # roll, pitch, yaw
                    self._pose = [float(packet[1]), float(packet[2]), float(packet[3])]
                elif header.startswith('VW'):                                               # lin_vel, ang_vel
                    self._vw = [float(packet[1]), float(packet[2])]
                # elif header.startswith('BAT'):
                #     self._bat = [float(packet[1]), float(packet[2]), float(packet[3])]
                # elif header.startswith('GYRO'):                                             # 
                #     self._gyro = [float(packet[1]), float(packet[2]), float(packet[3])]
            except:
                self.print('serial error')
            whole_packet = None
