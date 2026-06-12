import rclpy
from rclpy.node import Node
from custom_messages.msg import DigitalAndSolenoidCommand
from dc_gamepad_msgs.msg import GamePad
import time

class relay_control(Node):
    def __init__(self):
        super().__init__('test_relay_node')
        self.init_variables()
        self.init_subscription()
        self.timer = self.create_timer(0.01, self.publish_solenoid)
    
    def init_variables(self):
        self.switching_relay = False

    def init_subscription(self):
       self.solenoid_publisher = self.create_publisher(DigitalAndSolenoidCommand, '/publish_digital_solenoid', 10)
       #self.gamepad_subscription = self.create_subscription(GamePad, '/pad', self.gamepad_callback, 10)

    # def gamepad_callback(self,msg):

    #     self.button_a = msg.button_a
    #     self.previous_button_a = msg.previous_button_a

    #     if self.button_a and not self.previous_button_a:
            
    #         if self.switching_relay == True :
    #             self.switching_relay = False

    #         elif self.switching_relay == False:
    #             self.switching_relay = True 

    #     self.previous_button_a = self.button_a
    #     self.publish_solenoid()

    def publish_solenoid(self):
        solenoid_msg = DigitalAndSolenoidCommand()
        solenoid_msg.can_id = 600
        solenoid_msg.solenoid1_value = True
        solenoid_msg.solenoid2_value = True
        solenoid_msg.solenoid3_value = False
        solenoid_msg.solenoid4_value = False
        self.solenoid_publisher.publish(solenoid_msg)  



def main(args=None):
    rclpy.init(args=args)
    main_node = relay_control()
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()