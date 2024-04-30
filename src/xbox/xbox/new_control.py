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
            1)
        self.desicion_array_subscription

        self.desicion_array_subscription = self.create_subscription(
            Int32,
            'realsense_decision',
            self.realsense_desicion_callback,
            1)
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
        
        if self.joy_state == None or self.lidar_decision_state == None or self.realsense_decision_state == None:
            #self.get_logger().info('Waiting for both joy and decision messages.')
            if self.joy_state == None:
                self.get_logger().info('Waiting for joy')
            if self.lidar_decision_state == None:
                self.get_logger().info('Waiting for lidar')
            if self.realsense_decision_state == None:
                self.get_logger().info('Waiting for realsense')
            return
        
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
        
        if autonom == 1 and forward == 0 and reverse == 0 and joystick_value == 0:
            ## Find the gap ## 
            # gap array form [1,1,0,0,0,1,1,0,0,0,1]
            length_to_gap_array = self.lidar_decision_state
            un = 0.9
            up = 1.1
            gap = np.zeros(19)
            for i in range(1, len(length_to_gap_array)-1):
                if (length_to_gap_array[i] < up*length_to_gap_array[i+1] and length_to_gap_array[i] > un*length_to_gap_array[i+1] and length_to_gap_array[i] < up*length_to_gap_array[i-1] and length_to_gap_array[i] > un*length_to_gap_array[i-1]):
                    gap[i] = 1
                else:
                    gap[i] = 0
            self.get_logger().info(f'gap array: [{gap}, size: {len(gap)}]')
            
            # calculate the length of each segment [length_segment_1, ...]
            # not necessery i think
            """
            segment_length_array = np.zeros(19)
            for i in range(len(gap)):
                if (gap[i] > 0):
                    segment_length_array[i] = np.sqrt(2*length_to_gap_array[i]^2 - 4*length_to_gap_array[i]*np.cos(0.066))
                else: 
                    segment_length_array[i] = 0
            """
            # rates the gap from its neighbours and itself
            gap_neighbours_rate = np.zeros(19)
            for i in range(len(gap)):
                flag = True
                if (gap[i] != 1):
                    continue
                if (i == 0):
                    if (gap[i+1] == 1):
                        gap_neighbours_rate[i] = 0.05
                    else: 
                        gap_neighbours_rate[i] = 0
                if (i == len(gap)-1):
                    if (gap[i-1] == 1):
                        gap_neighbours_rate[i] = 0.05
                    else: 
                        gap_neighbours_rate[i] = 0

                if (i <= 9 and i > 0):
                    for j in range(0, i):
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
                    for k in range(0, (i-len(gap))*(-1)):
                        if (gap[i-k-1] == 1):
                            gap_neighbours_rate[i] += 0.05
                        else:
                            flag = False
                        if (gap[i+k+1] == 1):
                            gap_neighbours_rate[i] += 0.05
                        else:
                            flag = False
                        if (flag == False):
                            break
                
            self.get_logger().info(f'gap neighbours array: [{gap_neighbours_rate}], size: {len(gap_neighbours_rate)}')    

            # Check the lengths against the gaps
            max_length_array = heapq.nlargest(3, length_to_gap_array)
            indices_max_length = [length_to_gap_array.index(value) for value in max_length_array]

            max_gap_array = heapq.nlargest(3, gap_neighbours_rate)
            indices_max_gap = [gap_neighbours_rate.index(value) for value in max_gap_array]

            if ((max_length_array[i] > 3 and max_gap_array[i] > 0.05) or (max_length_array[i] > 7 and max_gap_array[i] > 0.0)):
                


            

                    














            forward_auto = self.lidar_decision_state.data[0]
            backward_auto = None
            right_auto = self.lidar_decision_state.data[1] # if 1, left direction is free
            left_auto = self.lidar_decision_state.data[2]
            obsticle = self.realsense_decision_state.data
            #self.get_logger().info(f'obsticle: {obsticle}')


            if (forward_auto != 1):
                # maybe pulse backwards to stop forward moment?
                """if (self.angle_motor = 112):
                        self.angle_motor = 99
                        self.motor.angle = self.angle_motor"""
                if (left_auto == 1):
                    self.get_logger().info('Turning left')
                    self.set_state(55, 114)

                elif (right_auto == 1):
                    self.get_logger().info('Turning right')
                    self.set_state(125, 114)
                else: 
                    self.get_logger().info('Stop')
                    self.set_state(85, 106)

            if (forward_auto == 1):
                self.get_logger().info('Forward')
                self.set_state(85, 114)
            

def main(args=None):
    rclpy.init(args=args)
    controller = Controller(kit)
    controller.get_logger().info('controller is running.... ')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
