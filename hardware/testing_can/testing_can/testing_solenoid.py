import rclpy
from rclpy.node import Node
from custom_messages.msg import DigitalAndSolenoidCommand, DigitalAndAnalogFeedback
from dc_gamepad_msgs.msg import GamePad
import time

class Solenoid_control(Node):
    def init(self):
        super().init('test_solenoid_node')
        self.init_variables()
        self.init_subscription()

        
        self.solenoid_publisher = self.create_publisher(DigitalAndSolenoidCommand, '/publish_digital_solenoid', 10)

    def init_variables(self):
        self.sensor_value = 0.0
        self.gripper = False
        self.dribbling = False

        self.solenoid1_value = False 
        self.solenoid2_value = False

        self.start_detect = False

        self.counter_loop = 0

    def init_subscription(self):
        self.sensor_subscription = self.create_subscription(DigitalAndAnalogFeedback, '/digital_analog_feedback', self.DigitalAndAnalog_callback,10)
        self.gamepad_subscription = self.create_subscription(GamePad, '/pad', self.gamepad_callback, 10)

    def gamepad_callback(self,msg):

        self.button_lb = msg.button_lb
        self.previous_button_lb = msg.previous_button_lb

        self.button_a = msg.button_a
        self.previous_button_a = msg.previous_button_a

        if self.button_a and not self.previous_button_a:
            self.reset_all = True
            if self.solenoid_hand == False:
                self.solenoid_hand = True
            elif self.solenoid_hand == True:
                self.solenoid_hand = False

        if self.button_lb and not self.previous_button_lb:
            self.dribbling = True

    def DigitalAndAnalog_callback(self,msg):
        if msg.can_id == 500:
            self.sensor_value = msg.analog2_value

        if self.sensor_value >= 0.5 and self.start_detect:
            self.close_hand = True

        if self.reset_all == True:
            self.solenoid_hand = False
            self.counter_loop = 0
            self.reset_all = False

        if self.dribbling and not self.reset_all:
            self.counter_loop += 1

        self.publish_solenoid()

    def control_loop(self):
        if self.counter_loop <= 20:
            self.solenoid_push = True
            self.solenoid_hand = True
            print("1")
        elif self.counter_loop <= 25:
            self.solenoid_push = False
            self.solenoid_hand = True
            print("2")
        elif self.counter_loop <= 35:
            self.solenoid_push = False
            self.solenoid_hand = True
            self.start_detect = True
            print("3")
        elif self.close_hand == True:
            self.solenoid_hand = False
            self.solenoid_push = False
            self.dribbling = False
            self.counter_loop = 0
            self.start_detect = False
            print("4")

    def publish_solenoid(self):
        solenoid_msg = DigitalAndSolenoidCommand()
        solenoid_msg.can_id = 600
        solenoid_msg.solenoid1_value = self.solenoid_push
        solenoid_msg.solenoid2_value = self.solenoid_hand
        print(self.solenoid_hand)
        self.solenoid_publisher.publish(solenoid_msg)  


def main(args=None):
    rclpy.init(args=args)
    main_node = Solenoid_control()
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if name == 'main':
    main()