import rclpy
from rclpy.node import Node
from custom_messages.msg import EncoderFeedback, DigitalAndAnalogFeedback, MotorCommand, ServoCommand, PwmCommand, DigitalAndSolenoidCommand

class Encoder_subscriber (Node):
    def __init__(self):
        super().__init__('read_data')
        self.position = 0.0
        self.speed =0.0
        self.goal = 0.0
        self.digital1_value = False
        self.analog1_value = 0.0
        self.encoder_subscription = self.create_subscription(
            EncoderFeedback,
             '/encoder_feedback',
             self.encoder_callback,
            10)
        self.subscription

        self.Digital_and_Analog_subscription = self.create_subscription(
            DigitalAndAnalogFeedback,
            '/digital_analog_feedback',
            self.DigitalAndAnalog_callback,
        10)

    def encoder_callback(self,msg):
        if msg.can_id == 101:
            self.position = msg.position
            self.speed = msg.speed
        self.get_logger().info(f'Received encoder feedback: Position={self.position}, '
                               f'Velocity={self.speed}')

    def DigitalAndAnalog_callback(self,msg):
        if msg.can_id == 102:
            self.digital_value = msg.digital1_value
            self.analog_value = msg.analog1_value
        self.get_logger().info(f'Received DigitalAndAnalog feedback: digital_value={self.digital1_value}, '
                               f'analog_value={self.analog1_value}')

def main(args=None):
    rclpy.init(args=args)

    read_data = Encoder_subscriber()

    rclpy.spin(read_data)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



