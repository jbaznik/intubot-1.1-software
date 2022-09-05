from inputs import get_gamepad
from inputs import devices
import time
import math
import threading

class xBoxHandler(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.xBox_connected = False
        self.xBox_lost = False
        self.xBox_inputs = {'js_l_x': 0, 'js_l_y': 0, 'js_r_x': 0, 'js_r_y': 0, 'trig_l':0, 'trig_r':0, 'button_A': False, 'button_B': False}
        self.thread = False
        self.running = True

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

    def start(self):
        self.thread = threading.Thread(target=self._monitor_controller, args=(), daemon = True)
        self.thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        self.xBox_inputs['js_l_x'] = self.LeftJoystickX * -1
        self.xBox_inputs['js_l_y'] = self.LeftJoystickY* -1
        self.xBox_inputs['js_r_x'] = self.RightJoystickX* -1
        self.xBox_inputs['js_r_y'] = self.RightJoystickY* -1
        self.xBox_inputs['trig_r'] = self.RightTrigger* -1
        self.xBox_inputs['trig_l'] = self.LeftTrigger* -1
        self.xBox_inputs['button_A'] = self.A
        self.xBox_inputs['button_B'] = self.B
        #print(self.xBox_inputs)
    
    def connect(self):
        for device in devices:
            device_name = str(device)
            if device_name == str("Microsoft X-Box 360 pad"):
                self.xBox_connected = True
            """
            if (device_found == True): # controller connected
                print("xBox controller connected")
                self.xBox_connected = True
                self.xBox_lost = False
            elif (device_found == False) and (self.xBox_connected == True): # controller was connected, but now lost lost
                print("xBox controller lost")
                self.xBox_connected = False
                self.xBox_lost = True    
            else:
                self.xBox_connected = False
        """ 

    def _monitor_controller(self):
        while self.running:
                if not self.xBox_connected:
                    time.sleep(0.1)
                    # attempt to reconnect if not connected
                    self.connect()
                    # print("calling connect")
                    continue       
                try:
                    events = get_gamepad()
                    for event in events:
                        if event.code == 'ABS_Y':
                            self.LeftJoystickY = event.state / xBoxHandler.MAX_JOY_VAL / 2 # normalize between -0.5 and 0.5
                        elif event.code == 'ABS_X':
                            self.LeftJoystickX = event.state / xBoxHandler.MAX_JOY_VAL / 2 # normalize between -0.5 and 0.5
                        elif event.code == 'ABS_RY':
                            self.RightJoystickY = event.state / xBoxHandler.MAX_JOY_VAL / 2  # normalize between -0.5 and 0.5
                        elif event.code == 'ABS_RX':
                            self.RightJoystickX = event.state / xBoxHandler.MAX_JOY_VAL / 2  # normalize between -0.5 and 0.5
                        elif event.code == 'ABS_Z':
                            self.LeftTrigger = (event.state / xBoxHandler.MAX_TRIG_VAL) # normalize between -1 and 0
                        elif event.code == 'ABS_RZ':
                            self.RightTrigger = -1 * (event.state / xBoxHandler.MAX_TRIG_VAL) # normalize between 0 and 1
                        elif event.code == 'BTN_SOUTH':
                            self.A = event.state
                        elif event.code == 'BTN_EAST':
                            self.B = event.state
                        """    
                        elif event.code == 'BTN_TL':
                            self.LeftBumper = event.state
                        elif event.code == 'BTN_TR':
                            self.RightBumper = event.state
                        elif event.code == 'BTN_SOUTH':
                            self.A = event.state
                        elif event.code == 'BTN_NORTH':
                            self.X = event.state
                        elif event.code == 'BTN_WEST':
                            self.Y = event.state
                        elif event.code == 'BTN_EAST':
                            self.B = event.state
                        elif event.code == 'BTN_THUMBL':
                            self.LeftThumb = event.state
                        elif event.code == 'BTN_THUMBR':
                            self.RightThumb = event.state
                        elif event.code == 'BTN_SELECT':
                            self.Back = event.state
                        elif event.code == 'BTN_START':
                            self.Start = event.state
                        elif event.code == 'BTN_TRIGGER_HAPPY1':
                            self.LeftDPad = event.state
                        elif event.code == 'BTN_TRIGGER_HAPPY2':
                            self.RightDPad = event.state
                        elif event.code == 'BTN_TRIGGER_HAPPY3':
                            self.UpDPad = event.state
                        elif event.code == 'BTN_TRIGGER_HAPPY4':
                            self.DownDPad = event.state
                        """

                        
                except:
                    print("lost xBox controller")
                    # attempt to reconnect if not connected
                    self.connect()

