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
from std_msgs.msg import Int32

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
            1)
        self.subscription 

        self.desicion_array_subscription = self.create_subscription(
            Float32MultiArray,
            'obstacle_distance',
            self.lidar_desicion_callback,
            10)
        self.desicion_array_subscription

        self.desicion_array_subscription = self.create_subscription(
            Int32,
            'realsense_decision',
            self.realsense_desicion_callback,
            10)
        self.desicion_array_subscription

        self.temp_timer = None
        self.joy_state = None 
        self.lidar_decision_state = None
        self.realsense_decision_state = None
        self.angle_motor = 106 # angels correspond to pulses 99≈1450, 106≈1500 and 112≈1550
        self.angle_servo = 90 # 55-90-125
        self.motor = kit.servo[0]
        self.motor.angle = self.angle_motor
        self.servo = kit.servo[1]
        self.servo.angle = self.angle_servo

    def joy_callback(self, msg):
        self.joy_state = msg # save the states from joy
        self.control()
        
    def lidar_desicion_callback(self, msg):
        self.lidar_decision_state = msg 
        self.control()

    def realsense_desicion_callback(self, msg):
        self.realsense_decision_state = msg 
        self.control()

    def set_state(self, angle, motor):
        self.angle_servo = angle 
        self.servo.angle = self.angle_servo
        #time.sleep(0.1)
        self.angle_motor = motor
        self.motor.angle = self.angle_motor
        time.sleep(0.15)  # can remove when its safe
        self.angle_motor = 106
        self.motor.angle = self.angle_motor
        time.sleep(0.05)

    def control(self):
        self.get_logger().info('control is ON')
        ## Find the gap ## 
        # gap array form [1,1,0,0,0,1,1,0,0,0,1]
        data_from_lidar = self.lidar_decision_state.data
        length_to_gap_array = np.array(data_from_lidar)
        self.get_logger().info(f'length array: {length_to_gap_array}')

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

        self.get_logger().info(f'gap array: {gap}')

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
                gap_neighbours_rate[i] += 0.01*i
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
                gap_neighbours_rate[i] += 0.01*(len(gap)-i-1)
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

        self.get_logger().info(f'gap neighbours array: {gap_neighbours_rate}')    

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
        self.get_logger().info(f'Filtered length array: {length_to_gap_array}') 

        # make one array by multiply the lengths with the gap rate
        decision_array = gap_neighbours_rate*length_to_gap_array
        self.get_logger().info(f'result array: {decision_array}') 

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
        # check is values in decision array are <= 0
        check = np.any(decision_array > 0)
        print("check ", check)
        if (check == True):
            for i in range(len(decision_array)):
                if (decision_array[i] > max_decision):
                    max_decision = decision_array[i]
                    gap_chosen = i
            max_length = length_to_gap_array[gap_chosen]
            #for i in range(len(indices_decision_array)):
            #    if (length_to_gap_array[indices_decision_array[i]] > max_length):
             #       gap_chosen = indices_decision_array[i]
              #      max_length = length_to_gap_array[indices_decision_array[i]]
            self.get_logger().info(f'Gap chosen: {gap_chosen}, size: {max_length}')
        else: 
            ## can reverse??? or choose the best value in the length array $$$$$$$$$$$$$$
            for i in range(len(length_to_gap_array)):
                if (length_to_gap_array[i] > max_length):
                    max_length = length_to_gap_array[i]
                    gap_chosen = i
            if (max_length < 2):
                max_length = None
                gap_chosen = None
                # reverse. look behind if clear, reverse

def main(args=None):
    rclpy.init(args=args)
    controller = Controller(kit)
    controller.get_logger().info('controller is running.... ')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
