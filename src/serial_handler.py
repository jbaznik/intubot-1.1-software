import os
import threading
import sys
import glob
import csv
import time
import serial
import struct


# Com Constants
END_FLAG = b'UUU'
DATA_LENGTH = 38

# TODO: verify baud rate. Faster is better for the teensy, slower is better for signal in 3m cable.
BAUD_RATE = 115200


class SerialHandler:
    def __init__(self, start_time, recording=False, recording_path=None):
        self.connected = False
        self.writing = False
        self.recording = recording
        self.recording_init = False
        self.recording_path = recording_path
        self.frame_number = 0  # is modifed by video_handler
        self.start_time = start_time
        self.teensy = None
        self.thread = None
        self.running = True
        self.first_call = True

        self.button_auto = False
        self.joystick_left_button = False
        self.joystick_right_button = False
        self.power_switch = False
        self.motor = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0}
        self.hall_sensor = 0
        self.joystick_raw = {'left': {'x': 0, 'y': 0}, 'right': {'x': 0, 'y': 0}}
        self.joystick_raw_offset = {'left': {'x': -6, 'y': -21}, 'right': {'x': -1, 'y': 0}} # offset to bring value to 512 when in null position
        self.joystick = {'left': {'x': 0, 'y': 0}, 'right': {'x': 0, 'y': 0}}
        self.analog_rocker = 0

    def start(self):
        """
        fire up the thread
        """
        self.thread = threading.Thread(target=self.update, args=(), daemon=True)
        self.thread.start()

    def connect(self):
        """
        Connects to the port of our serial device
        """
        # TODO: Test this system out one more time
        '''
        port = self.get_serial_port(read_until=END_FLAG,
                                    data_length=DATA_LENGTH,
                                    baud_rate=BAUD_RATE)
         '''
        port = 'COM3'                           
        if port:
            try:
                self.teensy = serial.Serial(port, BAUD_RATE, timeout=0.5)
                self.connected = True
                print("connected to teensy")
            except: 
                print("not connected to Teensy")
                self.connected = False
        else:
            print('port not found')
            self.connected = False

    def update(self):
        """
        Main loop of the thread
        Parse Input from the Teensy
        Input bytes, big endian

        Bytes 0-3:   motor 1 pos
        Bytes 4-7:   motor 2 pos
        Bytes 8-11:  motor 3 pos
        Bytes 12-15: motor 4 pos
        Bytes 16-19: motor 5 pos
        Byte 20:     automated button
        Byte 21:     hall sensor
        Byte 22:     joystick left button
        Byte 23:     joystick right button
        Bytes 24-25: power switch
        Bytes 26-27: joystick left X
        Bytes 28-29: joystick left Y
        Bytes 30-31: joystick right X
        Bytes 32-33: joystick right Y
        Bytes 34-35: analog rocker
        Bytes 36-38: end flag (UUU)

        """

        # loop till connection gets closed
        while self.running:
            if not self.connected:
                time.sleep(.5)
                # attempt to reconnect if not connected
                self.connect()
                # print("calling connect")
                continue
            # read from serial device
            try:
                data = self.teensy.read_until(END_FLAG)
                self.teensy.reset_input_buffer()
                self.connected = True
                [self.motor['m1'], self.motor['m2'], self.motor['m3'],self.motor['m4'],self.motor['m5']] = struct.unpack('ffffl', data[0:20])

                self.button_auto = bool(data[20])
                self.hall_sensor = bool(data[21])

                self.joystick_left_button = bool(data[22])
                self.joystick_right_button = bool(data[23])
                self.power_switch = bool(data[24:26])
            
                [self.joystick_raw['left']['x'], self.joystick_raw['left']['y'], self.joystick_raw['right']['x'], self.joystick_raw['right']['y']] = struct.unpack('HHHH', data[26:34])
                [self.analog_rocker] = struct.unpack('H', data[34:36])
                #print("read all data")
                self.normalize_raw_analog_inputs()



                if self.recording:
                    self.save_data()
                del data
            
            except Exception as e:
                print(e)
                print("entered except")
                self.connected = False
                # attempt to reconnect if not connected
                if not self.teensy.isOpen():
                    print("reconnecting?")
                    self.connect()
                else:
                    pass
        
    def normalize_raw_analog_inputs(self):
        """
        convert raw joystick inputs from [0, 1023] to [-1, 1], 
        with null position equal to zero by rounding to 2 decimal
        """
        #print("normalizing")
        #print(self.joystick_raw['left']['x'], self.joystick_raw['left']['y'], self.joystick_raw['right']['x'], self.joystick_raw['right']['y'])

        self.joystick['left']['x'] = round(self.joystick_raw2norm(self.joystick_raw['left']['x'] + self.joystick_raw_offset['left']['x']), 2)
        self.joystick['left']['y'] = round(self.joystick_raw2norm(self.joystick_raw['left']['y'] + self.joystick_raw_offset['left']['y']), 2)
        self.joystick['right']['x'] = round(self.joystick_raw2norm(self.joystick_raw['right']['x'] + self.joystick_raw_offset['right']['x']), 2)
        self.joystick['right']['y'] = round(self.joystick_raw2norm(self.joystick_raw['right']['y'] + self.joystick_raw_offset['right']['y']), 2)
        self.analog_rocker = round(self.analog_rocker_raw2norm(self.analog_rocker - 8), 1)


    def joystick_raw2norm(self, raw):
        normal = ((1/1023) * raw) - 0.5

        if normal > 0.5:
            normal = 0.5
        elif normal < -0.5:
            normal = -0.5

        return normal     

    def analog_rocker_raw2norm(self, raw):
        normal = ((2/108) * raw) - 9.463

        if normal > 1:
            normal = 1.00
        elif normal < -1:
            normal = -1.00
    
        return normal

    def save_data(self):
        """
        save output in file
        """
        if not self.recording_path:
            print('path to recording folder no found -> cannot record session')
            return
        path = os.path.join(self.recording_path, 'recording.csv')
        with open(path, "a", newline='', encoding='utf-8') as file:
            writer = csv.writer(file, delimiter=",")
            if self.recording_init:  # at the first time write header
                writer.writerow(
                    ("time [s]", "frame", "button_automode", "button_a", "button_b", "power_switch", "motor_1_position",
                     "motor_2_position", "motor_3_position", "motor_4_position", "motor_4_position", "hall_sensor",
                     "joystick_left_x", "joystick_left_y", "joystick_right_x", "joystick_right_y", "analog_rocker"))
                self.recording_init = False
            writer.writerow(
                (time.time() - self.start_time, self.frame_number, self.button_auto, self.button_a, self.button_b, self.
                 power_switch,
                 self.motor['m1'],
                 self.motor['m2'],
                 self.motor['m3'],
                 self.motor['m4'],
                 self.motor['m5'],
                 self.hall_sensor, 
                 self.joystick['left']['x'],
                 self.joystick['left']['y'],
                 self.joystick['right']['x'],
                 self.joystick['right']['y'],
                 self.analog_rocker))

    def send(self, motors, led_auto=None, led_display=None, led_laryngoscope = None, stop=None):
        """
        Construct byte array to send to Teensy

        m1r: motor 1 (-255 .. +255)
        m2r: motor 2 (-255 .. +255)
        m3r: motor 3 (-255 .. +255)
        m4r: motor 4 (-255 .. +255)
        m5r: motor 5 (-255 .. +255)
        la: auto LED value (0 .. 255)
        ld: display LED value (0 .. 255)
        ll: laryngoscope LED value (0 .. 255)

        s: stop value (0, 1)
        z: end flag

        e.g.
        motor 1 drive -10
        motor 2 drive 112
        motor 3 drive 55
        display LED set to 10:
        b'm1r-10m2r112m3r55ld10z'
        """
        if not self.connected:
            return

        bytes_out = ''

        # Position of motors (-255 .. +255)
        if motors['m1'] is not None:
            bytes_out += f'm1r{motors["m1"]}'
        if motors['m2'] is not None:
            bytes_out += f'm2r{motors["m2"]}'
        if motors['m3'] is not None:
            bytes_out += f'm3r{motors["m3"]}'
        if motors['m4'] is not None:
            bytes_out += f'm4r{motors["m4"]}'
        if motors['m5'] is not None:
            bytes_out += f'm5r{motors["m5"]}'
        # LED value (0-255)
        if led_auto is not None:
            bytes_out += f'la{led_auto}'
        if led_display is not None:
            bytes_out += f'ld{led_display}'
        if led_laryngoscope is not None:
            bytes_out += f'ld{led_laryngoscope}'
        # Stop (0 or 1)
        if stop is not None:
            if stop:
                stopstring = '1'
            else:
                stopstring = '0'
            bytes_out += f's{stopstring}'
        # end flag
            bytes_out += 'z'

        self.writing = True
        try:
            self.teensy.flush()
            self.teensy.write(bytes_out.encode('utf-8'))
            while self.teensy.out_waiting:
                pass
        except:
            self.connected = False

        self.writing = False
        del bytes_out

    def close(self):
        """
        close serial connection
        """
        self.running = False
        if self.connected:
            self.teensy.close()
            time.sleep(0.1)
            if self.teensy.isOpen():
                print("still open")
            else:
                print("closed the teensy port")
            self.connected = False

    def get_serial_port(self, read_until=END_FLAG, data_length=DATA_LENGTH, baud_rate=BAUD_RATE):
        """ Iterates through all available ports until it finds the port of our device

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                Serial port name of the connected device or None
        """
        
        if sys.platform.startswith('win'):
            all_ports = [f'COM{i}' for i in range(1, 256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            all_ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            all_ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
 
        for port in all_ports:
            try:
                conn = serial.Serial(port, baud_rate)
                data = b''
                for _ in range(3):
                    data = conn.read_until(read_until)
                if len(data) == data_length:  # that is our device
                    conn.close()
                    return port
                conn.close()
            except (OSError, serial.SerialException):
                pass
        return None
