# for ROS2 
import rclpy
from rclpy.node import Node

# 
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

# for adafruit motor controller
from adafruit_servokit import ServoKit

kit=ServoKit(channels=16, frequency=100)

class Controller(Node):
    def __init__(self, kit):
        super().__init__('controller')
        self.joy_state = None 
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1)
        self.subscription 

        self.angle_motor = 106 # little weird, have to use angels that correspond to pulses 99≈1450, 106≈1500 and 112≈1550
        self.angle_servo = 90 # 55-90-125
        self.motor = kit.servo[0]
        self.motor.angle = self.angle_motor
        self.servo = kit.servo[1]
        self.servo.angle = self.angle_servo

    def joy_callback(self, msg):
        self.joy_state = msg
        if self.joy_state == None:
            #self.get_logger().info('Waiting for both joy and decision messages.')
            if self.joy_state == None:
                self.get_logger().info('Waiting for joy')
            return
        
        #self.pca.channels[0].duty_cycle = int(0x7FFF * 0.165) # Neutral
        joystick_value = msg.axes[0]*(-1) # left joystick
        forward = msg.buttons[0] # button A
        reverse = msg.buttons[1] # button B
        autonom = msg.buttons[3] # button X
        
        if autonom == 0:
            self.get_logger().info('Joystick mode')
            if joystick_value != 0:
                self.angle_servo = joystick_value*35 + 90 # 55-90-125
                self.servo.angle = self.angle_servo
                self.get_logger().info(f'Servo angle: {self.angle_servo-90}')

            if forward == 1:
                self.angle_motor = 112
                self.motor.angle = self.angle_motor
                self.get_logger().info('Forward')
        
            elif reverse == 1:
                self.angle_motor = 99
                self.motor.angle = self.angle_motor
                self.get_logger().info('Reverse')

            else:
                self.angle_motor = 106
                self.motor.angle = self.angle_motor
        
        if autonom == 1 and forward == 0 and reverse == 0 and joystick_value == 0:
            self.get_logger().info('Autonome mode')
            pass
        
        # self.get_logger().info(f'state: {self.angle_motor}')

def main(args=None):
    rclpy.init(args=args)
    controller = Controller(kit)
    controller.get_logger().info('controller is running.... ')
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()