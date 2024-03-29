import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32MultiArray

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1)
        self.joy_subscription 
        self.button = None

        self.desicion_array_subscription = self.create_subscription(
            Int32MultiArray,
            'lidar_decision',
            self.desicion_callback,
            1)
        self.desicion_array_subscription

         # Set the GPIO pin
        self.motor_pin = 32
        #self.servo_pin = 15
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.motor_pin, GPIO.OUT)
        #GPIO.setup(self.servo_pin, GPIO.OUT)

        # Initialize PWM
        self.pwm_frequency = 50
        self.pwm_motor1 = GPIO.PWM(self.motor_pin, self.pwm_frequency)
        #self.pwm_servo = GPIO.PWM(self.servo_pin, self.pwm_frequency)
        self.pwm_motor1.start(0)
        #self.pwm_servo.start(0)

    def joy_callback(self, msg):
        #if msg.buttons[2] == 1:
        #    pass
        self.button = msg.buttons[3] # buttom X or Y
        self.desicion_callback(msg)

    def desicion_callback(self, msg):
        if self.button == 1:
            self.get_logger().info('Autonome mode activated... ')
            if msg.data[0] == 1:
                if (pulse_width != 1550):
                    pass
                    #self.get_logger().info('Driveing.. ')
                pulse_width = 1550
            else:
                if (pulse_width != 1500):
                    pass
                    #self.get_logger().info('Brakeing.. ')
                pulse_width = 1500
            duty_cycle = self.microseconds_to_duty_cycle(pulse_width)
            self.pwm_motor1.ChangeDutyCycle(duty_cycle)

    def microseconds_to_duty_cycle(self, microseconds):
        # Convert microseconds to duty cycle percentage
        cycle_length = 1000000 / self.pwm_frequency 
        duty_cycle = (microseconds / cycle_length) * 100
        return duty_cycle
    
    def __del__(self):
        #self.pwm_servo.stop()
        self.pwm_motor1.stop()

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    node.get_logger().info('controller is running.... ')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
