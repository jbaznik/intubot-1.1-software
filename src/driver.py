from distutils.log import ERROR
from this import s
import time
import sys
import json
import numpy as np


# Driver Constants
FOURTEENBIT = 16384.0 # per revoution
QUADRATURE = 9000 # difference per step
GEAR_RATIO = 1
PULLEY_RADIUS = 0.0025  # 2.5mm
DRAW_2_BEND = 22.936 * (np.pi / 180.0)
PWM_STEP_INCREASE_LIMIT = 30

# TODO: tune control parameters. Relative velocities (0-1) --> PWM (0-255)?
CONTROL_CAP = 200
PROPORTIONAL_GAIN = 20
ERROR_CAP = CONTROL_CAP / PROPORTIONAL_GAIN
CO_BEND_GAIN = 50
CONTRA_BEND_GAIN = 50
OBJECT_PROPORTIONAL_GAIN = 500
OBJECT_CONTROL_CAP = 150
ANATOMY_PROPORTIONAL_GAIN = 200

# TODO: this might be easier to tune if we control relative to time? i.e. slow to 0 over 1.2 seconds?
RELEVANCE_VALUE_LIMIT = 25
RELEVANCE_VALUE_DECLINE = 0.9

NO_ANATOMY_COUNT_LIMIT = 25
NO_OBJECT_COUNT_LIMIT = 50

L1 = 0.0125  # 12.5 mm
L2 = 0.0125  # 12.5 mm
R = 0.180  # 180mm


class Driver:
    def __init__(self, path_to_control_values, frame_w, frame_h):
        self.stop = False
        self.invert_controls = False
        self.timestamp_lastupdate = None
        self.auto_mode_available = False
        self.co_manipulation_active = False
        self.count_no_object = 0
        self.object_target_previous = [0, 0]
        self.object_width_previous = 0
        self.target_detected_time = 0
        # TODO: Philippe, this is for the coffee cup forward and backward motion
        self.coffee_cup_width_offset = 0.35
        #
        
        self.motor_pwm_command = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0}
        self.velocity_vector = {'x_dot': 0, 'y_dot': 0, 'z_dot': 0}
        self.position_error = {'x': 0, 'y': 0, 'z': 0}
        self.omega = {'x_dot': 0, 'y_dot': 0}
        self.UI = {'js_l_x': 0, 'js_l_y': 0, 'js_r_x': 0, 'js_r_y': 0, 'sled':0}
        self.anatomy_detection_time =  {'uvula': 0, 'trachea': 0, 'epiglottis': 0, 'vocalFold': 0, 'cartilage':0}
        #
        self.motor_error =  {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0}
        #
        self.phi_1 = 0
        self.phi_2 = 0
        self.phi_3 = 0
        self.phi_4 = 0
        self.phi_5 = 0
        #
        self.phi_1_previous = 0
        self.phi_2_previous = 0
        self.phi_3_previous = 0
        self.phi_4_previous = 0
        self.phi_5_previous = 0
        #
        self.target_previous = [0, 0]
        #
        self.count_no_detection = 0
        self.count_no_anatomy = 0
        self.anatomy_relevance = {'trachea': {'value': 0, 'score': 0},
                                  'vocalFold': {'value': 0, 'score': 0},
                                  'epiglottis': {'value': 0, 'score': 0},
                                  'cartilage': {'value': 0, 'score': 0},
                                  'uvula': {'value': 0, 'score': 0}}

        self.scene_center_offset = [0.5, (frame_h/frame_w)/2]

        # Control Parameters
        try:
            with open(path_to_control_values, encoding='utf-8') as f:
                control_map = json.load(f)
        except FileNotFoundError:
            print(f'file {path_to_control_values} not found')
            sys.exit(0)

        try:
            self.uvula_offset = int(control_map["uvula_offset"])
            self.vocalfold_offset = int(control_map["vocalfold_offset"])
        except KeyError:
            print(f'missing keys in {path_to_control_values}')
            sys.exit(0)

    def change_inversion_status(self):
        if self.invert_controls:
            self.invert_controls = False
        else:
            self.invert_controls = True

    def choose_UI_inputs(self, joystick, analog_rocker, xBox_connected, xBox):
        """ Takes the greater of the issued commands between xBox Controller and intuBot, only if xBox controller connected """
        if self.invert_controls:
            xBox['js_r_y'] = -1 * xBox['js_r_y']   
        else:
            joystick['right']['y'] = -1 * joystick['right']['y'] 
            xBox['js_l_y'] = -1 * xBox['js_l_y']


        if xBox_connected:
            # left joystick, x
            if abs(joystick['left']['x']) > abs(xBox['js_l_x']): # take intuBot command
                self.UI['js_l_x'] = joystick['left']['x']
            else: # take xBox command
                self.UI['js_l_x'] = xBox['js_l_x']
            # left joystick, y
            if abs(joystick['left']['y']) > abs(xBox['js_l_y']): # take intuBot command
                self.UI['js_l_y'] = joystick['left']['y']
            else: # take xBox command
                self.UI['js_l_y'] = xBox['js_l_y']
            # right joystick, x
            if abs(joystick['right']['x']) > abs(xBox['js_r_x']): # take intuBot command
                self.UI['js_r_x'] = joystick['right']['x']
            else: # take xBox command
                self.UI['js_r_x'] = xBox['js_r_x']
            # right joystick, y
            if abs(joystick['right']['y']) > abs(xBox['js_r_y']): # take intuBot command
                self.UI['js_r_y'] = joystick['right']['y']
            else: # take xBox command
                self.UI['js_r_y'] = xBox['js_r_y']
            # analog rocker/triggers
            if abs(analog_rocker) > abs(xBox['trig_r'] + xBox['trig_l']): # take intuBot command
                self.UI['sled'] = -1 * analog_rocker
            else: # take xBox command
                self.UI['sled'] = xBox['trig_r'] + xBox['trig_l']
        
        else: # only take intuBot commands
            self.UI['js_l_x'] = joystick['left']['x']
            self.UI['js_l_y'] = joystick['left']['y']
            self.UI['js_r_x'] = joystick['right']['x']
            self.UI['js_r_y'] = joystick['right']['y']
            self.UI['sled'] = -1 * analog_rocker
        
        

    def manual_driving(self, joystick, xBox_connected, xBox, analog_rocker, motor):
        """
        Adjust the control variables based on joystick and analog rocker
        """
        #print(joystick['left']['x'], joystick['left']['y'], joystick['right']['x'], joystick['right']['y'])
        if self.timestamp_lastupdate:
            dt = time.time()-self.timestamp_lastupdate
        else:
            dt = 0  # at the very first frame we don't know dt

        # TODO: validate controls from UI. Ability to invert axis from keyboard?
        self.velocity_vector['z_dot'] = analog_rocker
        self.omega['x_dot'] = joystick['left']['y']
        self.omega['y_dot'] = joystick['left']['x']
        self.velocity_vector['x_dot'] = joystick['right']['x']
        self.velocity_vector['y_dot'] = joystick['right']['y']
        self.stop = False

        self.set_phis(motor['m1'], motor['m2'], motor['m3'], motor['m4'], motor['m5'])
        #print(self.phi_1, ", ", self.phi_2, ", ", self.phi_3, ", ", self.phi_4, ", ", self.phi_5 )
        if xBox['button_A']:
            self.converge_to_home_position()
        else:
            self.move_motors_raw_1to1(joystick,xBox_connected, xBox, analog_rocker)
        # print(self.motor_pwm_command)
        # self.move_motors_raw_co_contra(joystick,xBox_connected, xBox, analog_rocker)
        # print(joystick, analog_rocker)
        #self.control_loop_iteration(dt)

        self.timestamp_lastupdate = time.time()

    # TODO: GEAR_RATIO should be normalized from 14-bit sensor input for m1-m4, and quadrature input for m5.
    def set_phis(self, m1, m2, m3, m4, m5):
        """
        Converts motor sensors values to real space values for Jacobian controller

        :param m1:
        :param m2:
        :param m3:
        :param m4:
        :param m5:
        :return:
        """
        self.phi_1 = m5 
        self.phi_2 = m1 * ((360.0)/FOURTEENBIT) # * (np.pi/180)
        self.phi_3 = m2 * ((360.0)/FOURTEENBIT) # * (np.pi/180)
        self.phi_4 = m3 * ((360.0)/FOURTEENBIT) # * (np.pi/180)
        self.phi_5 = m4 * ((360.0)/FOURTEENBIT) # * (np.pi/180)

    def converge_to_home_position(self):
        m_list = ['m1', 'm2','m3','m4']
        # get error between current position and desired position
        self.motor_error['m1'] =  (-1 * self.phi_2)
        self.motor_error['m2'] =  (-1 * self.phi_3)
        self.motor_error['m3'] =  (-1 * self.phi_4)
        self.motor_error['m4'] =  (-1 * self.phi_5)
        '''
        self.motor_error['m1'] = self.motor_error['m1'] + (-1 * self.phi_2)
        self.motor_error['m2'] = self.motor_error['m2'] + (-1 * self.phi_3)
        self.motor_error['m3'] = self.motor_error['m3'] + (-1 * self.phi_4)
        self.motor_error['m4'] = self.motor_error['m4'] + (-1 * self.phi_5)
        '''
        for m in m_list:
            if abs(self.motor_error[m]) <= 15:
                self.motor_pwm_command[m] = 0
            else:    
                self.motor_pwm_command[m] = self.calculate_pwm_from_error( self.motor_error[m], m)
        
        if self.phi_1 != 0:
            self.motor_pwm_command['m5'] = -255
    
    def converge_to_target(self):
        m_list = ['m1', 'm2','m3','m4']
        # get error between current position and desired position
        self.motor_error['m1'] =  (-1 * self.phi_2)
        self.motor_error['m2'] =  (-1 * self.phi_3)
        self.motor_error['m3'] =  (-1 * self.phi_4)
        self.motor_error['m4'] =  (-1 * self.phi_5)
        '''
        self.motor_error['m1'] = self.motor_error['m1'] + (-1 * self.phi_2)
        self.motor_error['m2'] = self.motor_error['m2'] + (-1 * self.phi_3)
        self.motor_error['m3'] = self.motor_error['m3'] + (-1 * self.phi_4)
        self.motor_error['m4'] = self.motor_error['m4'] + (-1 * self.phi_5)
        '''
        for m in m_list:
            if abs(self.motor_error[m]) <= 15:
                self.motor_pwm_command[m] = 0
            else:    
                self.motor_pwm_command[m] = self.calculate_pwm_from_error( self.motor_error[m], m)
        
        if self.phi_1 != 0:
            self.motor_pwm_command['m5'] = -255


    def move_motors_raw_1to1(self, joystick, xBox_connected, xBox, analog_rocker):
        """ drive motors according to joystick inputs """
        self.choose_UI_inputs(joystick, analog_rocker, xBox_connected, xBox)
        pwm_limit = 400
        self.motor_pwm_command['m1'] = self.UI['js_l_x'] * pwm_limit
        self.motor_pwm_command['m2'] = self.UI['js_l_y'] * pwm_limit
        self.motor_pwm_command['m3'] = self.UI['js_r_x'] * pwm_limit
        self.motor_pwm_command['m4'] = self.UI['js_r_y'] * pwm_limit
        self.motor_pwm_command['m5'] = self.UI['sled'] * 255
    
    def move_motors_raw_co_contra(self, joystick, xBox_connected, xBox, analog_rocker):
        """ drive motors according to joystick inputs """
        self.choose_UI_inputs(joystick, analog_rocker, xBox_connected, xBox)
        pwm_limit = 400
        if xBox['js_r_y']: # co bend
            self.motor_pwm_command['m2'] = self.UI['js_r_y'] * pwm_limit
            self.motor_pwm_command['m4'] = self.UI['js_r_y'] * pwm_limit
        elif xBox['js_l_y']: # contra bend
            self.motor_pwm_command['m2'] = -1 * self.UI['js_l_y'] * pwm_limit
            self.motor_pwm_command['m4'] = self.UI['js_l_y'] * pwm_limit
        else: # deactivate motors
            self.motor_pwm_command['m2'] = 0
            self.motor_pwm_command['m4'] = 0

        if xBox['js_r_x']: # co bend
            self.motor_pwm_command['m1'] = self.UI['js_r_x'] * pwm_limit
            self.motor_pwm_command['m3'] = self.UI['js_r_x'] * pwm_limit
        elif xBox['js_l_x']: # contra bend
            self.motor_pwm_command['m1'] = self.UI['js_l_x'] * pwm_limit
            self.motor_pwm_command['m3'] = -1 * self.UI['js_l_x'] * pwm_limit
        else: # deactivate motors
            self.motor_pwm_command['m1'] = 0
            self.motor_pwm_command['m3'] = 0
    
        self.motor_pwm_command['m5'] = self.UI['sled'] * 255


    def sled_boundary_control(self, hall_sensor):
        """ If Hall Sensor low (i.e. zero) prevent translational sled from moving further""" 
        if hall_sensor == 0:
            if self.motor_pwm_command['m5'] < 0:
                self.motor_pwm_command['m5'] = 0
        
        if self.phi_1 > 22000:
            if self.motor_pwm_command['m5'] > 0:
                self.motor_pwm_command['m5'] = 0

    def visual_servoing_on_object(self, scene, joystick, analog_rocker, motor):
        '''
        if self.timestamp_lastupdate:
            dt = time.time()-self.timestamp_lastupdate
        else:
            dt = 0  # at the very first frame we don't know dt
        '''
        desired_target = None
        # Prioritize Anatomy
        if 'cup' in scene:
            # center on trachea
            desired_target = scene['cup']['center']
            target_width = scene['cup']['width']
            print("cup width: ", scene['cup']['width'])

            self.target_detected_time = time.time()
           

        if desired_target:
            # TODO: show target on GUI?
            # limit desired target to the interval x=[0,1], y=[0,1]
            desired_target[0] = min(max(desired_target[0], 0), 1)
            desired_target[1] = min(max(desired_target[1], 0), 1)
            # get velocity vector (may need scaling)
            self.get_position_error(desired_target, target_width)
            # save target location in case anatomy lost
            self.object_target_previous = desired_target
            self.object_width_previous = target_width
            # update omega['y_dot'] and omega['x_dot']

        elif (time.time() - self.target_detected_time < 0.1):
            self.get_position_error(self.object_target_previous, self.object_width_previous)

        else:
            self.object_target_previous = [0, 0]
            self.object_width_previous = self.coffee_cup_width_offset
            m_list = ['m1', 'm2','m3','m4', 'm5']
            for m in m_list:
                self.motor_pwm_command[m] = 0


        #print(self.position_error)

        self.motor_pwm_command['m1'] = self.position_error['x'] * OBJECT_PROPORTIONAL_GAIN
        self.motor_pwm_command['m2'] = self.position_error['y'] * OBJECT_PROPORTIONAL_GAIN * 2
        self.motor_pwm_command['m3'] = self.position_error['x'] * OBJECT_PROPORTIONAL_GAIN
        self.motor_pwm_command['m4'] = -1 * self.position_error['y'] * OBJECT_PROPORTIONAL_GAIN * 2
        self.motor_pwm_command['m5'] = -1 * self.position_error['z'] * OBJECT_PROPORTIONAL_GAIN * 3

        m_list = ['m1', 'm2','m3','m4']
        #m_list = ['m1', 'm3']
        for m in m_list:
            if self.motor_pwm_command[m] > OBJECT_CONTROL_CAP:
                self.motor_pwm_command[m] = OBJECT_CONTROL_CAP
            elif self.motor_pwm_command[m] < (-1 * OBJECT_CONTROL_CAP):
                self.motor_pwm_command[m] = -1 * OBJECT_CONTROL_CAP
        
        if self.motor_pwm_command['m5'] > 255:
                self.motor_pwm_command['m5'] = 255
        elif self.motor_pwm_command['m5'] < -255:
            self.motor_pwm_command['m5'] = -255
        
        

        #print(self.motor_pwm_command)

        '''
        # apply co-manipulation inputs
        if self.co_manipulation_active:
            self.integrate_co_manipulation(joystick, analog_rocker)


        # update motor angle positions
        self.set_phis(motor['m1'], motor['m2'], motor['m3'], motor['m4'], motor['m5'])

        # send desired velocity values to control loop
        self.control_loop_iteration(dt)

        # update time
        self.timestamp_lastupdate = time.time()
        '''
        
    # TODO: Philippe, this is the function that working inside of Hans
    def visual_servoing(self, scene, joystick, analog_rocker, motor):
        """
        Reviews the scene to check for the detection of the anatomy
        The code adjusts the control variables based on the anatomy in the scene

        Args:
            scene (dict): dict of found anatomies
            joystick
            analog_rocker

        # TODO: object_detection() spits out relative/normalized box data, between 0 and 1.0.
        Example scene:
        {
            'face': {
                'score': 0.74,
                'box': [129.22, 30.77, 944.61, 707.48],
                'center': [0.5 0.7],
                'width':fu 0.4
                'height': 0.5
            },
            'vocalFold': {
                'score': 0.82,
                'box': [176.26, 6.45, 970.18, 710.47],
                'center': [0.3 0.9],
                'width': 0.05
                'height': 0.03
            }
        }
        """

        if self.timestamp_lastupdate:
            dt = time.time()-self.timestamp_lastupdate
        else:
            dt = 0  # at the very first frame we don't know dt

        desired_target = None
        # Prioritize Anatomy
        if 'trachea' in scene:
            # center on trachea
            desired_target = scene['trachea']['center']
            self.anatomy_detection_time['trachea'] = time.time()
            self.velocity_vector['z_dot'] = 0.5

        elif 'vocalFold' in scene:
            # create a new sub-dictionary key to 'vocalFold'
            center = scene['vocalFold']['center']
            self.anatomy_detection_time['vocalFold'] = time.time()
            # center on vocalFold when far, move down when near
            desired_target = [
                center[0],
                center[1] - np.exp(.025 * (scene['vocalFold']['width'] - self.vocalfold_offset))]
            self.velocity_vector['z_dot'] = 0.5

        elif 'cartilage' in scene and 'epiglottis' in scene:
            # center on the middle point between cartilage and epiglottis
            self.anatomy_detection_time['cartilage'] = time.time()
            self.anatomy_detection_time['epiglottis'] = time.time()
            center_cartilage = scene['cartilage']['center']
            center_epiglottis = scene['epiglottis']['center']
            desired_target = [
                (center_cartilage[0] + center_epiglottis[0]) / 2, (center_cartilage[1] + center_epiglottis[1]) / 2]
            self.velocity_vector['z_dot'] = 0.5

        elif 'epiglottis' in scene:
            self.anatomy_detection_time['epiglottis'] = time.time()
            center = scene['epiglottis']['center']
            # move below epiglottis
            desired_target = [center[0], center[1] + scene['epiglottis']['height']/2]
            self.velocity_vector['z_dot'] = 0.5

        elif 'uvula' in scene: 
            #print("uvula width: ", scene['uvula']['width'])
            self.anatomy_detection_time['uvula'] = time.time()
            center = scene['uvula']['center']
            # determine largest side of bounding box
            # TODO: height actually shrinks as the uvula gets close and is at the bottom of the frame.
            #       width would be more reliable as a gauge on distance.
            max_dim = max(scene['uvula']['width'], scene['uvula']['height'])
            # move upwards from uvula
            if scene['uvula']['width'] > 0.35:
                desired_target = [center[0], center[1] + np.exp(.02*(max_dim-self.uvula_offset))]
            else:
                desired_target = [center[0], center[1]]
                self.velocity_vector['z_dot'] = 1
        
        #alternative for Hans?
        # elif 'uvula' in scene:
        #     self.anatomy_detection_time['uvula'] = time.time()
        #     while True:
        #         self.velocity_vector['z_dot'] = 1
        #         if 'uvula' in scene:
        #             break
        #         if 'cartilage' in scene:
        #             break
        #         if 'epiglottis' in scene:
        #             break
        #         if 'trachea' in scene:
        #             break
        #         if time.time > self.anatomy_detection_time['uvula']+ 3000:
        #             break


        elif 'cartilage' in scene:
            # create a new sub-dictionary key to 'cartilage'
            self.anatomy_detection_time['cartilage'] = time.time()
            center = scene['cartilage']['center']
            # Move above cartilage
            desired_target = [center[0], center[1] + 0.5]
            self.velocity_vector['z_dot'] = 0.5

        elif self.count_no_detection > 0 and self.count_no_anatomy == 0:
            # no anatomy detected, but something was recently, so we are still driving toward the last known target but at decreasing speed
            
            if (time.time() - self.anatomy_detection_time['trachea']) < 1:
                self.get_velocity_vector(self.target_previous)
                self.velocity_vector['z_dot'] = 0.5

            elif (time.time() - self.anatomy_detection_time['vocalFold']) < 1:
                self.get_velocity_vector(self.target_previous)
                self.velocity_vector['z_dot'] = 0.5

            elif ((time.time() - self.anatomy_detection_time['cartilage']) < 1) and ((time.time() - self.anatomy_detection_time['epiglottis']) < 1):
                self.get_velocity_vector(self.target_previous)
                self.velocity_vector['z_dot'] = 0.5

            elif (time.time() - self.anatomy_detection_time['epiglottis']) < 1:
                self.get_velocity_vector(self.target_previous)
                self.velocity_vector['z_dot'] = 0.5

            elif (time.time() - self.anatomy_detection_time['uvula']) < 1:
                self.get_velocity_vector(self.target_previous)
                self.velocity_vector['z_dot'] = 1
                self.target_previous[1] += 0.1

            elif (time.time() - self.anatomy_detection_time['cartilage']) < 1:
                self.get_velocity_vector(self.target_previous)   
                self.target_previous[1] += 0.1   
                self.velocity_vector['z_dot'] = 0.5

            else:
                self.velocity_vector['x_dot'] = 0
                self.velocity_vector['y_dot'] = 0
                self.velocity_vector['z_dot'] = 0  
            '''
            # TODO: all velocity vectors should decrease with count_no_detection.
            #       count_no_detection will always be increasing positive. This needs to be formatted relative to
            #       the direction of the velocities. e.g. if vel > 0: subtract velocity, else: increase velocity
            self.velocity_vector['z_dot'] = self.velocity_vector['z_dot'] * \
                (1 - (self.count_no_detection / RELEVANCE_VALUE_LIMIT))
                '''
        '''
        elif self.count_no_anatomy > 0:
            # TODO: verify "backing up" strategy. Set a marker for the GUI?
            # no anatomy available anymore, hold orientation and move backwards
            self.velocity_vector['x_dot'] = 0
            self.velocity_vector['y_dot'] = 0
            self.velocity_vector['z_dot'] = self.velocity_vector['z_dot'] * -(
            self.count_no_detection / RELEVANCE_VALUE_LIMIT)
        '''
        if desired_target:
            # TODO: show target on GUI?
            # limit desired target to the interval x=[0,1], y=[0,1]
            desired_target[0] = min(max(desired_target[0], 0), 1)
            desired_target[1] = min(max(desired_target[1], 0), 1)
            # get velocity vector (may need scaling)
            self.get_velocity_vector(desired_target) 
            # save target location in case anatomy lost
            self.target_previous = desired_target
            # update omega['y_dot'] and omega['x_dot']

        self.motor_pwm_command['m1'] = self.velocity_vector['x_dot'] * ANATOMY_PROPORTIONAL_GAIN
        self.motor_pwm_command['m2'] = -1 * self.velocity_vector['y_dot'] * ANATOMY_PROPORTIONAL_GAIN
        self.motor_pwm_command['m3'] = self.velocity_vector['x_dot'] * ANATOMY_PROPORTIONAL_GAIN
        self.motor_pwm_command['m4'] = self.velocity_vector['y_dot'] * ANATOMY_PROPORTIONAL_GAIN
        self.motor_pwm_command['m5'] = self.velocity_vector['z_dot'] * ANATOMY_PROPORTIONAL_GAIN 

        m_list = ['m1', 'm2','m3','m4']
        #m_list = ['m1', 'm3']
        for m in m_list:
            if self.motor_pwm_command[m] > CONTROL_CAP:
                self.motor_pwm_command[m] = CONTROL_CAP
            elif self.motor_pwm_command[m] < (-1 * CONTROL_CAP):
                self.motor_pwm_command[m] = -1 * CONTROL_CAP
        
        if self.motor_pwm_command['m5'] > 255:
                self.motor_pwm_command['m5'] = 255
        elif self.motor_pwm_command['m5'] < -255:
            self.motor_pwm_command['m5'] = -255
        
        

        #print(self.motor_pwm_command)

        # after calculating the desired velocities and for co-manipulation input, send info to control loop

        '''
        # apply co-manipulation inputs
        if self.co_manipulation_active:
            self.integrate_co_manipulation(joystick, analog_rocker)

        # update motor angle positions
        self.set_phis(motor['m1'], motor['m2'], motor['m3'], motor['m4'], motor['m5'])

        # send desired velocity values to control loop
        self.control_loop_iteration(dt)
        '''
        # update time
        self.timestamp_lastupdate = time.time()

    def get_velocity_vector(self, anatomy_target_center):
        xy_vector = np.subtract(self.scene_center_offset, anatomy_target_center)
        # distance_to_center = np.sqrt(xy_vector[0]**2 + xy_vector[1]**2)  # maximum value is sqrt(0.5)

        self.velocity_vector['x_dot'] = xy_vector[0]
        self.velocity_vector['y_dot'] = xy_vector[1]
        # the greater the misalignment, the smaller the forward velocity vector value is
        '''
        self.velocity_vector['z_dot'] = np.sqrt(0.5) - distance_to_center
        if self.velocity_vector['z_dot'] < 0:
            self.velocity_vector['z_dot'] = 0
        '''
    
    def get_position_error(self, object_target_center, object_target_width):
        xy_error = np.subtract(self.scene_center_offset, object_target_center)

        self.position_error['x'] = xy_error[0]
        self.position_error['y'] = xy_error[1]
        self.position_error['z'] = object_target_width - self.coffee_cup_width_offset
        

    def integrate_co_manipulation(self, joystick, analog_rocker):
        # consider averaging the inputs, rather than just summing, before going to more complex methods of calculation
        self.velocity_vector['z_dot'] = analog_rocker + self.velocity_vector['z_dot']
        self.omega['x_dot'] = joystick['left']['x']
        self.omega['y_dot'] = joystick['left']['y']
        self.velocity_vector['x_dot'] = joystick['right']['x'] + self.velocity_vector['x_dot']
        self.velocity_vector['y_dot'] = joystick['right']['y'] + self.velocity_vector['y_dot']

    def track_common_object(self, scene):
        """this is always called to track common objects when coco is running.
           this will be the function that decides if auto mode is or is not available.

        Args:
            scene (dict): dict of all detections
        """

        positive_detection = False
        common_object = 'cup'
        if common_object in scene:
            #print("Found ", common_object, " with score: ", scene[common_object]['score'], "location: ", scene[common_object]['center'])
            positive_detection = True
        else:
            positive_detection = False
        
                 # handle positive detection and no positive detection
        if positive_detection:
            self.count_no_object = 0
            # if anatomy has been detected, set auto mode as available
            self.auto_mode_available = True
        else:
            self.count_no_object += 1  # value tracked to know how to scale down forward motion
            if self.count_no_object < NO_OBJECT_COUNT_LIMIT:
                self.auto_mode_available = True
            else:
                self.auto_mode_available = False
        
        

    def track_anatomy_detection(self, scene):
        """this is always called to track anatomies.
           this will be the function that decides if auto mode is or is not available.

        Args:
            scene (dict): dict of all detections
        """
        positive_detection = False
        anatomies = ['trachea', 'vocalFold', 'epiglottis', 'uvula', 'cartilage']
        for anatomy in anatomies:
            if anatomy in scene:
                self.update_relevance(anatomy, scene[anatomy]['score'])
                positive_detection = True
            else:
                positive_detection = False
        
        # handle positive detection and no positive detection
        if positive_detection:
            self.count_no_detection = 0
            self.count_no_anatomy = 0
            # if anatomy has been detected, set auto mode as available
            self.auto_mode_available = True
        else:
            self.count_no_detection += 1  # value tracked to know how to scale down forward motion

            # check tracked anatomy for recent detection
            for anatomy in self.anatomy_relevance.values():
                # the anatomy relevance has an upper limit, this is the number of frames it will take to decrease all values until it does not return a value larger than 0
                if anatomy['value'] > 0:
                    self.auto_mode_available = True
                    break
            else:
                if self.count_no_anatomy < NO_ANATOMY_COUNT_LIMIT:
                    self.count_no_anatomy += 1  # value tracked to know when auto mode no longer available
                    self.auto_mode_available = True
                else:
                    self.auto_mode_available = False

    def update_relevance(self, anatomy_name, detection_score):
        if detection_score:
            self.anatomy_relevance[anatomy_name]['value'] += 1  # increase relevance
            if self.anatomy_relevance[anatomy_name]['value'] > RELEVANCE_VALUE_LIMIT:
                self.anatomy_relevance[anatomy_name]['value'] = RELEVANCE_VALUE_LIMIT

            # TODO: moving average with controllable list size will be easier to tune. i.e. average of 2 may still be noisy.
            self.anatomy_relevance[anatomy_name]['score'] = (
                self.anatomy_relevance[anatomy_name]['score'] + detection_score) / 2  # average scores of current and previous

        else:
            self.anatomy_relevance[anatomy_name]['value'] -= 1  # decrease relevance
            if self.anatomy_relevance[anatomy_name]['value'] < 0:
                self.anatomy_relevance[anatomy_name]['value'] = 0

            # reduce relevance
            self.anatomy_relevance[anatomy_name]['score'] = self.anatomy_relevance[anatomy_name]['score'] * RELEVANCE_VALUE_DECLINE

    def emergency_stop(self):
        """sets all motor PWM's to zero"""
        self.motor_pwm_command['m1'] = 0
        self.motor_pwm_command['m2'] = 0
        self.motor_pwm_command['m3'] = 0
        self.motor_pwm_command['m4'] = 0
        self.motor_pwm_command['m5'] = 0

        self.stop = True

        self.timestamp_lastupdate = time.time()

    def move_translational_sled(self, command):
        """move only translational sled, i.e. motor5"""
        self.motor_pwm_command['m1'] = 0
        self.motor_pwm_command['m2'] = 0
        self.motor_pwm_command['m3'] = 0
        self.motor_pwm_command['m4'] = 0
        if command == 'forward':
            self.motor_pwm_command['m5'] = CONTROL_CAP
        elif command == 'backward':
            self.motor_pwm_command['m5'] = CONTROL_CAP*(-1)
        elif command == 'stop':
            self.motor_pwm_command['m5'] = 0

        self.timestamp_lastupdate = time.time()

    """
    Below is the control structure for the control loop

    For understanding of system set-up, the following information is useful:

    Orientation of system
                              +(X)
                              /
                            /
    +(Z) _ _ _ _ _ _ _ _ _/
                          |O <- Bending Section Base
                          |
                          |
                          |
                          |
                         +(Y)

    The motion of the tip is considered to happen in the world frame represented above
    Assuming that the camera perceives the world as such:


    (0,0)_ _ _ _ _ _ _ _ _ _ _ _
        |                       |
        |                       |
        |                       |
        |           (0.5, 0.5)  |
        |           ^           |
        |                       |
        |                       |
        |_ _ _ _ _ _ _ _ _ _ _ _| (1,1)

        for motion in the end-effector the following sign pairs results in different motions:
        velocity vector = [#, #] where the values will be in the range of [-0.5, 0.5]
        velocity_vector = screen_center - anatomy_center

        [+#, +#] --> left & up motion
        [+#, -#] --> left & down motion
        [-#, +#] --> right & up motion
        [-#, -#] --> right & down motion

    """

    def calculate_inverse_kinematics_5dof(self, x_dot, y_dot, omega_z_dot):
        """Solve for the motor velocities that will achieve desired end-effector velocity

        Args:
            x_dot (float): desired velocity vector of the end-effector
            y_dot (float): desired velocity vector of the end-effector
            omega_z_dot (float): desired velocity vector of the end-effector

        Returns:
            list: set of motor velocities necessary to achieve end-effector velocity
        """

        input_state_array = np.array([[x_dot], [y_dot], [omega_z_dot]])

        # expressions and variables that make up the equations
        phi_13 = self.phi_1 + self.phi_3
        phi_135 = phi_13 + self.phi_5

        c_1 = np.cos(self.phi_1)
        s_1 = np.sin(self.phi_1)
        c_13 = np.cos(phi_13)
        s_13 = np.sin(phi_13)
        # c_135 = np.cos(phi_135)
        # s_135 = np.sin(phi_135)
        sigma_1 = np.sin(self.phi_1 - phi_13)
        sigma_2 = np.sin(self.phi_1 - phi_135)
        sigma_3 = np.sin(phi_13 - phi_135)
        sigma_4 = np.cos(self.phi_1 - phi_13)
        sigma_5 = np.cos(self.phi_1 - phi_135)
        exp_1 = L1 * sigma_1 + R * sigma_4
        exp_2 = (L1 * L2 + L1 * L1) * sigma_1 + (L1 * R + L2 * R) * sigma_4
        exp_3 = L1 * L2 * sigma_2 + L2 * R * sigma_5

        inv_jacobian = np.array([
            [(c_13 / exp_1),
             (s_13 / exp_1),
             (-L2 * sigma_3 / exp_1)],
            [((-L1 * (c_1 + c_13) - L2 * c_13 + R * s_1) / exp_2),
             ((-L1 * (s_1 + s_13) - L2 * s_13 - R * c_1) / exp_2),
             ((L2 * sigma_3 * (L1 + L2) + exp_3) / exp_2)],
            [((-L1 * c_1 + R * s_1) / exp_2),
             ((-L1 * s_1 - R * c_1) / exp_2),
             (1 + exp_3 / exp_2)]
        ])

        output_state_array = np.dot(inv_jacobian, input_state_array)
        output_state_list = output_state_array.T.tolist()[0]
        return output_state_list

    def calculate_co_contra_bend(self, z_dot, omega_y_dot):

        if abs(z_dot) > abs(omega_y_dot):  # co-bend
            #print("co bend")
            phi_2_dot_desired = z_dot * CO_BEND_GAIN
            phi_4_dot_desired = phi_2_dot_desired
        else:  # contra-bend
            #print("contra bend")
            phi_2_dot_desired = omega_y_dot * CONTRA_BEND_GAIN
            phi_4_dot_desired = -1 * phi_2_dot_desired

        return [phi_2_dot_desired, phi_4_dot_desired]

    def calculate_pwm_from_error(self, error, motor):
        """Generates a proportional response from the error between desired angular velocity and actual velocity

        Args:
            error (float): float with the error value

        Returns:
            int: an integer with a PWM value that correlates to duty cycle
        """
        # proportionally scale the error

        # TODO: How do we handle the motors differently? m5 will have a different gear ratio and sensor feedback?

        pwm = (error * PROPORTIONAL_GAIN)

        """
        # prevent a large (increasing) step in PWM values
        new_pwm_sign = np.sign(pwm)
        motor_pwm_sign = np.sign(self.motor_pwm_command[motor])
        # the motor remains moving in the same direction
        if new_pwm_sign == motor_pwm_sign:
            # limit increase of pwm to defined limit
            if (np.abs(self.motor_pwm_command[motor]) - np.abs(pwm)) > PWM_STEP_INCREASE_LIMIT: 
                pwm = self.motor_pwm_command[motor] + (motor_pwm_sign * PWM_STEP_INCREASE_LIMIT)
            # do not limit decrease in pwm
            else: 
                pass 
        # the motor is switching direction
        elif new_pwm_sign != motor_pwm_sign:
            # limit pwm in new direction from being larger than defined limit
            pwm = new_pwm_sign * PWM_STEP_INCREASE_LIMIT 
        """   

        # limit the PWM to fit within a signed 8 - bit integer
        if pwm > CONTROL_CAP:
            pwm = CONTROL_CAP
        elif pwm < -CONTROL_CAP:
            pwm = -CONTROL_CAP

        return pwm

    def control_loop_iteration(self, dt):
        """Calculate necessary motor PWMs to achieve desired tip velocities

        Args:
            dt (float): change in time since last function call
        """

        if dt == 0:
            return

        # transform velocities from video frame to robot frame
        robotframe_z_dot_desired = -1 * self.velocity_vector['x_dot']
        robotframe_y_dot_desired = self.velocity_vector['y_dot']
        robotframe_x_dot_desired = self.velocity_vector['z_dot']
        robotframe_omega_x_desired = -1 * self.omega['x_dot']
        robotframe_omega_y_desired = self.omega['y_dot']

        # Loop Step 1: Get motor angular velocity (phi_dot) desired
        # motor positions updated since last iteration. Encoders give position of the wheel

        # solve for desired motor velocities
        # TODO: Jac should output only sagittal motion? so no robot_frame_x???
        # TODO: Reactivate Jacobian control portion
        """
        [phi_1_dot_desired, phi_3_dot_desired, phi_5_dot_desired] = self.calculate_inverse_kinematics_5dof(
            0, robotframe_y_dot_desired, robotframe_omega_x_desired)
        """
        [phi_2_dot_desired,
         phi_4_dot_desired] = self.calculate_co_contra_bend(robotframe_z_dot_desired, robotframe_omega_y_desired)
        
        [phi_1_dot_desired, 
        phi_3_dot_desired, 
        phi_5_dot_desired] = [robotframe_y_dot_desired, robotframe_y_dot_desired, robotframe_x_dot_desired]
        # Loop Step 2: Calculate error between actual velocity and desired velocity
        # calculate actual velocity from encoder current and previous reading(s)
        phi_1_dot_actual = (self.phi_1 - self.phi_1_previous) / dt
        phi_2_dot_actual = (self.phi_2 - self.phi_2_previous) / dt
        phi_3_dot_actual = (self.phi_3 - self.phi_3_previous) / dt
        phi_4_dot_actual = (self.phi_4 - self.phi_4_previous) / dt
        phi_5_dot_actual = (self.phi_5 - self.phi_5_previous) / dt
        # update previous with current motor position
        self.phi_1_previous = self.phi_1
        self.phi_2_previous = self.phi_2
        self.phi_3_previous = self.phi_3
        self.phi_4_previous = self.phi_4
        self.phi_5_previous = self.phi_5
        # calculate error between these velocities
        phi_1_dot_error = phi_1_dot_desired - phi_1_dot_actual
        phi_2_dot_error = phi_2_dot_desired - phi_2_dot_actual
        phi_3_dot_error = phi_3_dot_desired - phi_3_dot_actual
        phi_4_dot_error = phi_4_dot_desired - phi_4_dot_actual
        phi_5_dot_error = phi_5_dot_desired - phi_5_dot_actual
        
        
        '''
        print("phi_1_dot_error: ", round(phi_1_dot_error), ", ",
              "phi_2_dot_error: ", round(phi_2_dot_error), ", ",
              "phi_3_dot_error: ", round(phi_3_dot_error), ", ",
              "phi_4_dot_error: ", round(phi_4_dot_error), ", ",
              "phi_5_dot_error: ", round(phi_5_dot_error),      
              )
        '''

        # Loop Step 3: Proportional controller to generate PWM signal
        # on the robot, it takes on an integer ranging from  [-180, 180]
        self.motor_pwm_command['m1'] = self.calculate_pwm_from_error(phi_2_dot_error, 'm1')
        self.motor_pwm_command['m2'] = self.calculate_pwm_from_error(phi_3_dot_error, 'm2')
        self.motor_pwm_command['m3'] = self.calculate_pwm_from_error(phi_4_dot_error, 'm3')
        self.motor_pwm_command['m4'] = self.calculate_pwm_from_error(phi_5_dot_error, 'm4')
        self.motor_pwm_command['m5'] = self.calculate_pwm_from_error(phi_1_dot_error, 'm5')

        # if trigger is hit, add velocity.
        self.motor_pwm_command['m5'] += robotframe_x_dot_desired * 160
        if self.motor_pwm_command['m5'] > CONTROL_CAP:
            self.motor_pwm_command['m5'] = CONTROL_CAP
        elif self.motor_pwm_command['m5'] < -CONTROL_CAP:
            self.motor_pwm_command['m5'] = -CONTROL_CAP
        
        m_list = ['m1', 'm2','m3','m4','m5']
        string = ""
        for m in m_list:

            if round(self.motor_pwm_command[m]) == 0:
               string += m + ": " + "0.00" 
            else:
                if np.sign(self.motor_pwm_command[m]) > 0:
                    string += m + ":  " + str(round(self.motor_pwm_command[m], 2))
                else:
                    string += m + ": " + str(round(self.motor_pwm_command[m], 2))

            if m != 'm5':
                string += ", "
        #print(string)
        '''
        print("m1: ", round(self.motor_pwm_command['m1']), ", ",
              "m2: ", round(self.motor_pwm_command['m2']), ", ",
              "m3: ", round(self.motor_pwm_command['m3']), ", ",
              "m4: ", round(self.motor_pwm_command['m4']), ", ",
              "m5: ", round(self.motor_pwm_command['m5']),      
              )
        '''
        # TODO: verify need for derivative control factor
