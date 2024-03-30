import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32MultiArray

class xboxController(Node):
    def __init__(self):
        super().__init__('xbox')
        self.pulse_width = 1500
        self.joy_state = None
        self.decision_state = None
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

        # Set the GPIO pin
        self.motor_pin = 32
        self.servo_pin = 15
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.motor_pin, GPIO.OUT)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # Initialize PWM
        self.pwm_frequency = 50
        self.pwm_motor = GPIO.PWM(self.motor_pin, self.pwm_frequency)
        self.pwm_servo = GPIO.PWM(self.servo_pin, self.pwm_frequency)
        self.pwm_motor.start(0)
        self.pwm_servo.start(0)

        # GPIO pin for the servo
        self.angle_min = 55
        self.angle_neu = 85
        self.angle_max = 120

    def joy_callback(self, msg):
        joystick_value = msg.axes[0]*(-1) # -1 done to get the correct turning direction # upperleft joystick
        servo_angle = self.map_joystick_to_servo_angle(joystick_value, self.angle_min, self.angle_neu, self.angle_max) #55, 90, 125
        if servo_angle != 85:
            self.get_logger().info(f'servo_angle: {servo_angle-85}')
        self.set_servo_angle(servo_angle)

        if (msg.buttons[3] == 0):
            if msg.buttons[0] == 1: # button A on the xbox controller
            # forward motion
                self.pulse_width = 1550
            elif msg.buttons[1] == 1: # button B on the xbox controller
            # backward motion
                self.pulse_width = 1440
            else:
            # neutral position
                self.pulse_width = 1500

            duty_cycle = self.microseconds_to_duty_cycle(self.pulse_width)
            self.pwm_motor.ChangeDutyCycle(duty_cycle)

            # show information
            button_state_1 = 'Pressed' if msg.buttons[0] == 1 else 'Released'
            button_state_2 = 'Pressed' if msg.buttons[1] == 1 else 'Released'
            if button_state_1 == 'Pressed':
                self.get_logger().info(f'Button State Forward: {button_state_1}, Pulse Width: {self.pulse_width}, Duty Cycle: {duty_cycle:.2f}')
            if button_state_2 == 'Pressed':
                self.get_logger().info(f'Button State Backwards: {button_state_2}, Pulse Width: {self.pulse_width}, Duty Cycle: {duty_cycle:.2f}')


        self.joy_state = msg # save the states from joy
        self.control() # use them in control function

    def desicion_callback(self, msg):
            self.decision_state = msg
            self.control()

    def control(self):
        if self.joy_state == None or self.decision_state == None:
            self.get_logger().info('Waiting for both joy and decision messages.')
            return
        
        if (self.joy_state.buttons[3] == 1 and self.joy_state.buttons[1] == 0 and self.joy_state.buttons[0] == 0):
            self.get_logger().info('Autonome mode activated... ')
            #self.get_logger().info(f'data from decision state: {self.decision_state.data[0]}')
            if (self.decision_state.data[0] == 1):
                if (self.pulse_width != 1550):
                    pass
                    #self.get_logger().info('Driveing.. ')
                self.pulse_width = 1550
            else:
                if (self.pulse_width != 1500):
                    pass
                    #self.get_logger().info('Brakeing.. ')
                self.pulse_width = 1500
            
            duty_cycle = self.microseconds_to_duty_cycle(self.pulse_width)
            self.pwm_motor.ChangeDutyCycle(duty_cycle)

    def map_joystick_to_servo_angle(self, joystick_value, angle_min, angle_neutral, angle_max):
        if joystick_value > 0:
            angle = angle_neutral + (joystick_value * (angle_max - angle_neutral))
        else:
            angle = angle_neutral + (joystick_value * (angle_neutral - angle_min)) 
        return angle

    def set_servo_angle(self, angle):
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm_servo.ChangeDutyCycle(duty_cycle)

    def angle_to_duty_cycle(self, angle):
        min_pulse_width = 1100  # (changed from 1000)
        max_pulse_width = 1600  # (changed from 2000)
        pulse_range = max_pulse_width - min_pulse_width
        angle_range = self.angle_max - self.angle_min

        self.pulse_width = ((angle - self.angle_min) * pulse_range / angle_range) + min_pulse_width

        # 50 Hz
        duty_cycle = (self.pulse_width / 20000) * 100  # 20 ms period

        # Se values in terminal
        #self.get_logger().info(f'Angle: {angle}, pulse_width: {pulse_width}, duty_cycle: {duty_cycle}')
        
        return duty_cycle

    def microseconds_to_duty_cycle(self, microseconds):
        # Convert microseconds to duty cycle percentage
        cycle_length = 1000000 / self.pwm_frequency 
        duty_cycle = (microseconds / cycle_length) * 100
        return duty_cycle
    
    def __del__(self):
        self.pwm_servo.stop()
        self.pwm_motor.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    servo_controller = xboxController()
    servo_controller.get_logger().info('controller is running.... ')
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
