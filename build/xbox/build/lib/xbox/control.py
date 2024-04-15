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
            self.desicion_callback,
            1)
        self.desicion_array_subscription

        self.temp_timer = None
        self.joy_state = None 
        self.decision_state = None
        self.angle_motor = 106 # temporary solution where i use the servo package. Little weird, have to use angels that correspond to pulses 99≈1450, 106≈1500 and 112≈1550
        self.angle_servo = 90 # 55-90-125
        self.motor = kit.servo[0]
        self.motor.angle = self.angle_motor
        self.servo = kit.servo[1]
        self.servo.angle = self.angle_servo
    

    def joy_callback(self, msg):
        self.joy_state = msg # save the states from joy
        self.control()
        
    def desicion_callback(self, msg):
        self.decision_state = msg 
        self.control()
    """
    def set_servo_angle_with_delay(self, angle, delay):
        self.servo.angle = angle
        self.get_logger().info(f"Servo angle set to {angle}, starting delay")
        if self.temp_timer:
            self.temp_timer.cancel()
        self.temp_timer = self.create_timer(delay, self.finish_servo_angle_setting)

    def finish_servo_angle_setting(self):
        self.get_logger().info("Delay complete")
        if self.temp_timer:
            self.temp_timer.cancel() 
    """
    def control(self):
        if self.joy_state == None or self.decision_state == None:
            #self.get_logger().info('Waiting for both joy and decision messages.')
            if self.joy_state == None:
                self.get_logger().info('Waiting for joy')
            if self.decision_state == None:
                self.get_logger().info('Waiting for decision')
            return
        
        joystick_value = self.joy_state.axes[0]*(-1) # left joystick
        forward = self.joy_state.buttons[0] # button A
        reverse = self.joy_state.buttons[1] # button B
        autonom = self.joy_state.buttons[3] # button X
        
        
        if autonom == 0:
            #self.get_logger().info('Joystick mode')
            if joystick_value != 0:
                self.angle_servo = joystick_value*35 + 90 # 55-90-125
                self.servo.angle = self.angle_servo
                self.get_logger().info(f'Servo angle: {self.angle_servo-90}')

            if forward == 1:
                if (self.angle_motor != 112):
                    self.get_logger().info('Forward')
                self.angle_motor = 112
                self.motor.angle = self.angle_motor
        
            elif reverse == 1:
                if (self.angle_motor != 99):
                    self.get_logger().info('Reverse')
                self.angle_motor = 99
                self.motor.angle = self.angle_motor

            else:
                self.angle_motor = 106
                self.motor.angle = self.angle_motor
        
        if autonom == 1 and forward == 0 and reverse == 0 and joystick_value == 0:
            #self.get_logger().info('Autonome mode')
            forward_auto = self.decision_state.data[0]
            backward_auto = None
            left_auto = self.decision_state.data[1] # if 1, left direction is free
            right_auto = self.decision_state.data[2]

            if (forward_auto != 1):
                
                # maybe pulse backwards to stop forward moment?
                """if (self.angle_motor = 112):
                        self.angle_motor = 99
                        self.motor.angle = self.angle_motor"""
                # look to the left
                if (left_auto == 1):
                    self.get_logger().info('Turning left')
                    #self.set_servo_angle_with_delay(125, 2)
                    self.angle_servo = 125 #between 55-90-125
                    self.servo.angle = self.angle_servo
                    time.sleep(0.1)
                    self.angle_motor = 112
                    self.motor.angle = self.angle_motor
                    time.sleep(0.15)
                    self.angle_motor = 106
                    self.motor.angle = self.angle_motor
                    time.sleep(0.1)

                elif (right_auto == 1):
                    self.get_logger().info('Turning right')
                    self.angle_servo = 55 #between 55-90-125
                    self.servo.angle = self.angle_servo
                    time.sleep(0.1)
                    self.angle_motor = 112
                    self.motor.angle = self.angle_motor
                    time.sleep(0.15)
                    self.angle_motor = 106
                    self.motor.angle = self.angle_motor
                    time.sleep(0.1)
                    #self.angle_motor = 112
                    #self.motor.angle = self.angle_motor
                else: 
                    self.get_logger().info('Stop')
                    self.angle_servo = 90 #between 55-90-125
                    self.servo.angle = self.angle_servo
                    self.angle_motor = 106
                    self.motor.angle = self.angle_motor


            if (forward_auto == 1):
                self.get_logger().info('Forward')
                self.angle_servo = 90 
                self.servo.angle = self.angle_servo
                self.angle_motor = 112
                self.motor.angle = self.angle_motor
                time.sleep(0.1)
                self.angle_motor = 106
                self.motor.angle = self.angle_motor
                time.sleep(0.1)
            else: 
                self.angle_servo = 90
                self.servo.angle = self.angle_servo
                self.angle_motor = 106
                self.motor.angle = self.angle_motor
            

            

def main(args=None):
    rclpy.init(args=args)
    controller = Controller(kit)
    controller.get_logger().info('controller is running.... ')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()