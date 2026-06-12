import rclpy
from rclpy.node import Node
from custom_messages.msg import DigitalAndAnalogFeedback, DigitalAndSolenoidCommand

class Dribble_arm (Node):
    def __init__(self):
        super().__init__('dribble_test')
        self.button_value = 0.0
        self.sensor_value = 0.0
        self.solenoid_push = False 
        self.close_hand = False
        self.solenoid_hand_value = False
        self.solenoid_push_value = False

        self.counter_hand = 0
        self.counter_push = 0

        self.solenoid_publisher = self.create_publisher(DigitalAndSolenoidCommand, '/publish_digital_solenoid', 10)
        self.Digital_and_Analog_subscription = self.create_subscription(DigitalAndAnalogFeedback, '/digital_analog_feedback', self.DigitalAndAnalog_callback,
        10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def DigitalAndAnalog_callback(self,msg):
        if msg.can_id == 500:
            self.button_value = msg.analog1_value
            self.sensor_value = msg.analog2_value

        if self.button_value >= 0.5:
            self.close_hand = False
            self.solenoid_push = True 

        if self.sensor_value >= 0.5 and not self.solenoid_push:
            self.close_hand = True
            print(self.sensor_value)
    
    def timer_callback(self):
        self.control_push()
        self.control_hand()
         
    def control_push(self):
        if self.solenoid_push:
            self.counter_push += 1
            if self.counter_push <= 50:
                self.solenoid_push_value = True
            else:
                self.solenoid_push_value = False
                self.solenoid_push = False
                self.counter_push = 0
    
    def control_hand(self):

        if not self.close_hand :
            self.solenoid_hand_value = False
        else: 
             self.solenoid_hand_value = True

        solenoid_msg = DigitalAndSolenoidCommand()
        solenoid_msg.can_id = 600
        solenoid_msg.solenoid1_value = self.solenoid_push_value
        solenoid_msg.solenoid2_value = self.solenoid_hand_value
        self.solenoid_publisher.publish(solenoid_msg)  

def main(args=None):
    rclpy.init(args=args)

    dribble_test = Dribble_arm()

    rclpy.spin(dribble_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dribble_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



