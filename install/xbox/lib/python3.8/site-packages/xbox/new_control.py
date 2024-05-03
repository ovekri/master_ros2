# for ROS2 
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.timer import Timer
from rclpy.duration import Duration

import time
import numpy as np
import heapq

# 
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

# for adafruit motor controller
from adafruit_servokit import ServoKit

kit=ServoKit(channels=16, frequency=100)

class Controller(Node):
    def __init__(self, kit):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            5)
        self.subscription 

        self.desicion_array_subscription = self.create_subscription(
            Float32MultiArray,
            'obstacle_distance',
            self.lidar_bank_callback,
            10)
        self.desicion_array_subscription

        self.desicion_array_subscription = self.create_subscription(
            Float64MultiArray,
            'plane_fitter_features_',
            self.realsense_bank_callback,
            10)
        self.desicion_array_subscription

        self.timer_period_lidar = 0.5
        self.timer_period_realsense = 0.5
        self.timer_lidar = self.create_timer(self.timer_period_lidar, self.lidar_desicion_callback)
        self.timer_realsense = self.create_timer(self.timer_period_realsense, self.realsense_desicion_callback)

        self.flag_lidar = False
        self.flag_realsense = False
        self.plane_fitter_features_mid = [0, 0, 0, 0]
        self.plane_fitter_features_left = [0, 0, 0, 0]
        self.plane_fitter_features_right = [0, 0, 0, 0]
        self.plane_fitter_features = 0
        self.temp_timer = None
        self.joy_state = None 
        self.lidar_decision_state = 0
        self.lidar_state = None
        self.realsense_decision_state = None
        self.realsense_state = None
        self.no_solution = False
        self.angle_motor = 106 # angels correspond to pulses 99≈1450, 106≈1500 and 112≈1550
        self.angle_servo = 90 # 55-90-125
        self.motor = kit.servo[0]
        self.motor.angle = self.angle_motor
        self.servo = kit.servo[1]
        self.servo.angle = self.angle_servo

    def joy_callback(self, msg):
        self.joy_state = msg # save the states from joy
        self.control()
    
    def lidar_bank_callback(self, msg):
        self.lidar_state = msg
    
    def realsense_bank_callback(self, msg):
        self.realsense_state = msg
        
    def lidar_desicion_callback(self):
        self.flag_lidar = True
        if self.lidar_state is None:
            self.get_logger().info('Lidar data not available yet.')
            return
        self.flag_lidar = True
        ## Find the gap ## 
        # gap array form [1,1,0,0,0,1,1,0,0,0,1]
        length_to_gap_array = np.array(self.lidar_state.data)
        #self.get_logger().info(f'length array: {length_to_gap_array}')

        gap_up = 1.75
        un = 0.75                                                       ####$$$$$$
        up = 1.25
        gap = np.zeros(19)

        for i in range(0, len(length_to_gap_array)):
            if (length_to_gap_array[i] > 2):
                if (i == 0):
                    if (length_to_gap_array[i] > 5 and (length_to_gap_array[i+1] > length_to_gap_array[i] or length_to_gap_array[i+1] > un*length_to_gap_array[i])):
                        gap[i] == 1
                if (gap[i-1] == 1 and i!=len(length_to_gap_array)-1):
                    if ((length_to_gap_array[i] > un*length_to_gap_array[i-1])):
                        gap[i] = 1
                elif (i == len(length_to_gap_array)-1):
                    if (un*length_to_gap_array[i-1] < length_to_gap_array[i]):
                        gap[i] = 1
                elif (((length_to_gap_array[i] > gap_up*length_to_gap_array[i-1]) or (length_to_gap_array[i] > 3)) and (length_to_gap_array[i+1] < up*length_to_gap_array[i] and length_to_gap_array[i+1] > un*length_to_gap_array[i])):
                    gap[i] = 1

        #self.get_logger().info(f'gap array: {gap}')

        # calculate the length of each segment [length_segment_1, ...]
        # not necessery i think
        """
        segment_length_array = np.zeros(19)
        for i in range(len(gap)):
            if (gap[i] > 0):
                segment_length_array[i] = np.sqrt(2*length_to_gap_array[i]^2 - 2*length_to_gap_array[i]^2*np.cos(0.06981))
            else: 
                segment_length_array[i] = 0
        """
        # rates the gap from its neighbours and itself
        gap_neighbours_rate = np.zeros(19)
        for i in range(len(gap)):
            flag = True
            if (gap[i] != 1):
                continue

            if (i != 0 and i != 18):
                gap_neighbours_rate[i] = 0.1
            elif (i == 0 or i == 18):
                gap_neighbours_rate[i] = 0.075

            if (i == 0):
                if (gap[i+1] == 1):
                    gap_neighbours_rate[i] = 0.05
            if (i == len(gap)-1):
                if (gap[i-1] == 1):
                    gap_neighbours_rate[i] = 0.05

            if (i <= 9 and i > 0):
                gap_neighbours_rate[i] += 0.02*i
                for j in range(i):
                    if (gap[i-j-1] == 1):
                        gap_neighbours_rate[i] += 0.05
                    else:
                        flag = False
                    if (gap[i+j+1] == 1):
                        gap_neighbours_rate[i] += 0.05
                    else:
                        flag = False
                    if (flag == False):
                        break

            if (i > 9 and i < len(gap)-1):
                gap_neighbours_rate[i] += 0.02*(len(gap)-i-1)
                for k in range(1, len(gap)-i):
                    if (gap[i-k] == 1):
                        gap_neighbours_rate[i] += 0.05
                    else:
                        flag = False
                    if (gap[i+k] == 1):
                        gap_neighbours_rate[i] += 0.05
                    else:
                        flag = False
                    if (flag == False):
                        break    

        #self.get_logger().info(f'gap neighbours array: {gap_neighbours_rate}')    

        # Filter length array
        # find fake 11m. if points are closer than 0.5m to the lidar 
        min_param = 1.5
        for i in range(len(length_to_gap_array)):
            if (length_to_gap_array[i] == 11):
                if (i == 0):
                    if (length_to_gap_array[i+1] < min_param and length_to_gap_array[i+2] < min_param):
                        length_to_gap_array[i] = -1
                if (i == len(length_to_gap_array)-1):
                    print(length_to_gap_array[i])
                    if (length_to_gap_array[i-1] < min_param and length_to_gap_array[i-2] < min_param):
                        length_to_gap_array[i] = -1
                if ((i > 1) and (i < len(length_to_gap_array)-1)):
                    if (((length_to_gap_array[i-1] < min_param and length_to_gap_array[i-2] < min_param) or (length_to_gap_array[i+1] < min_param and length_to_gap_array[i+2] < min_param)) or (length_to_gap_array[i+1] < min_param and length_to_gap_array[i-1] < min_param)): #((1,1,11or11,1,1)or(1,11,1))
                        length_to_gap_array[i] = -1
                    if ((length_to_gap_array[i-1] == None)):
                        length_to_gap_array[i] = -1
        #self.get_logger().info(f'Filtered length array: {length_to_gap_array}') 

        # make one array by multiply the lengths with the gap rate
        decision_array = gap_neighbours_rate*length_to_gap_array
        #self.get_logger().info(f'result array: {decision_array}') 

        # Check the lengths against the gaps
        count = 0
        for i in range(len(decision_array)):
            if (decision_array[i] > 0):
                count += 1
        if (count > 4):
            count = 4
        #self.get_logger().info(f'count: {count}')
        ## not used
        #indices_decision_array = np.argsort(decision_array)[-count:][::-1]
        #max_length_array = length_to_gap_array[decision_array]
        #self.get_logger().info(f'max length array: {max_length_array}, size: {len(max_length_array)}')
        ##
        #indices_max_gap = np.argsort(gap_neighbours_rate)[-count:][::-1]
        #max_gap_array = gap_neighbours_rate[indices_max_gap]

        #self.get_logger().info(f'max gap array: {max_gap_array}, size: {len(max_gap_array)}')
        #self.get_logger().info(f'indices: {indices_max_gap}, size: {len(indices_max_gap)}')
        

        # decision part, which GAP to choose
        # look true the gaps and choose the one with the longest range
        max_decision = 0.0
        max_length = 0.0
        gap_chosen = 0
        no_solu = False
        # check is values in decision array are <= 0
        check = np.any(decision_array > 0)
        #print("check ", check)
        if (check == True):
            for i in range(len(decision_array)):
                if (decision_array[i] > max_decision):
                    max_decision = decision_array[i]
                    gap_chosen = i
            max_length = length_to_gap_array[gap_chosen]
            #self.get_logger().info(f'Gap chosen: {gap_chosen}, size: {max_length}')
        else: 
            ## can reverse??? or choose the best value in the length array $$$$$$$$$$$$$$
            for i in range(len(length_to_gap_array)):
                if (length_to_gap_array[i] > max_length):
                    max_length = length_to_gap_array[i]
                    gap_chosen = i
            if (max_length < 1.5):
                max_length = None
                gap_chosen = None
                no_solu = True
                # reverse. look behind if clear, reverse
        self.no_solution = no_solu
        #if ((gap_chosen % 2) != 0 and gap_chosen != None):
        #    gap_chosen += 1
        if (gap_chosen != None):
            gap_angle = -36 + 4*(gap_chosen)
        else:
            gap_angle = 0
        self.lidar_decision_state = gap_angle
        self.control()

    def realsense_desicion_callback(self):
        if self.realsense_state is None:
            self.get_logger().info('Realsense data not available yet.')
            return
        self.flag_realsense = True
        self.plane_fitter_features = np.array(self.realsense_state.data)
        #self.plane_fitter_features_mid = plane_fitter_features[0:4]
        #self.plane_fitter_features_left = plane_fitter_features[4:8]
        #self.plane_fitter_features_right = plane_fitter_features[8:12]
        self.control()

    def set_state(self, angle, motor):
        self.angle_servo = 90+(int(angle*0.98))
        self.servo.angle = self.angle_servo
        #self.angle_motor = motor
        #self.motor.angle = self.angle_motor
        #time.sleep(0.15)  # can remove when its safe
        #self.angle_motor = 106
        #self.motor.angle = self.angle_motor
        #time.sleep(0.1)

    def control(self):
        if self.joy_state == None:
            if self.joy_state == None:
                self.get_logger().info('Waiting for joy')
                return
        #self.get_logger().info(f'm: {self.lidar_decision_state}')
        # Buttons
        joystick_value = self.joy_state.axes[0]*(-1) # left joystick
        forward = self.joy_state.buttons[0] # button A
        forward_fast = self.joy_state.buttons[4]
        reverse = self.joy_state.buttons[1] # button B
        autonom = self.joy_state.buttons[3] # button X
        
        if autonom == 0: # manually controlled
            if joystick_value != 0:
                self.angle_servo = joystick_value*35 + 90 # 55-90-125
                self.servo.angle = self.angle_servo
            if forward == 1:
                self.angle_motor = 114
                self.motor.angle = self.angle_motor
            elif forward_fast == 1:
                self.angle_motor = 118 
                self.motor.angle = self.angle_motor
            elif reverse == 1:
                self.angle_motor = 99
                self.motor.angle = self.angle_motor
            else:
                self.angle_motor = 106
                self.motor.angle = self.angle_motor
        
        # Self driving part
        if self.flag_lidar == True and self.flag_realsense == True:
            if autonom == 1 and forward == 0 and reverse == 0 and joystick_value == 0:
                #print("mid: ",self.plane_fitter_features_mid)
                #print("left: ",self.plane_fitter_features_left)
                #print("right: ",self.plane_fitter_features_right)
                self.get_logger().info(f'whole plane: {self.plane_fitter_features}')
                if (self.no_solution == True):
                    self.set_state(self.lidar_decision_state, 106)
                else:
                    self.set_state(self.lidar_decision_state, 114)
                self.get_logger().info(f'angle: {90+self.lidar_decision_state}')
                self.flag_lidar = False
                self.flag_realsense = False

def main(args=None):
    rclpy.init(args=args)
    controller = Controller(kit)
    controller.get_logger().info('controller is running.... ')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
