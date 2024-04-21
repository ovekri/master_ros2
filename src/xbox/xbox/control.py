# for ROS2 
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.timer import Timer
from rclpy.duration import Duration

import time

# 
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
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
            Int32MultiArray,
            'lidar_decision',
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
        self.angle_motor = motor
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
        reverse = self.joy_state.buttons[1] # button B
        autonom = self.joy_state.buttons[3] # button X
        
        
        if autonom == 0: # manually controled
            if joystick_value != 0:
                self.angle_servo = joystick_value*35 + 90 # 55-90-125
                self.servo.angle = self.angle_servo
            if forward == 1:
                self.angle_motor = 114
                self.motor.angle = self.angle_motor
            elif reverse == 1:
                self.angle_motor = 99
                self.motor.angle = self.angle_motor
            else:
                self.angle_motor = 106
                self.motor.angle = self.angle_motor
        
        if autonom == 1 and forward == 0 and reverse == 0 and joystick_value == 0:
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
                    self.set_state(55, 106)

            if (forward_auto == 1):
                self.get_logger().info('Forward')
                self.set_state(85, 114)
            else: 
                self.set_state(55, 106)
            

def main(args=None):
    rclpy.init(args=args)
    controller = Controller(kit)
    controller.get_logger().info('controller is running.... ')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()