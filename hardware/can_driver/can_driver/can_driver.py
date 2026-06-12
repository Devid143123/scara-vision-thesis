#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from custom_messages.msg import EncoderFeedback, MotorCommand, ServoCommand, PwmCommand, DigitalAndSolenoidCommand, DigitalAndAnalogFeedback
import struct
import subprocess
import array
import signal
import time

def run_subprocess(cmd):
    """Executes a command in the subprocess and returns the result."""
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

class CanDriver(Node, can.Listener):
    def __init__(self):
        Node.__init__(self, 'can_driver_node')
        self.error_timer = time.perf_counter()
        
        self.setup_can_interface()
        self.init_publisher()
        self.init_subscriber()
        self.shutdown_requested = False
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def setup_can_interface(self):
        self.canMsgData = [0, 0, 0, 0, 0, 0, 0, 0]

        try:
            """Checks and sets up the CAN interface."""
            result = run_subprocess(["ip", "link", "show", "can0"])
            if b"state UP" in result.stdout:
                self.get_logger().info("CAN interface is already up")
            else:
                result = run_subprocess(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"])
                if result.returncode == 0:
                    self.get_logger().info("CAN interface is up")
                else:
                    self.get_logger().info("CAN failed to setup")

            self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
            self.notifier = can.Notifier(self.bus, [self])
        except:
            result = run_subprocess(["sudo", "ip", "link", "set", "can0", "down"])
            
            if hasattr(self, 'bus'):
                try:
                    self.bus.shutdown()
                except Exception as e:
                    print(f"Error shutting down bus: {e}")
                finally:
                    delattr(self, 'bus')

            if hasattr(self, 'notifier'):
                try:
                    self.notifier.stop()
                except Exception as e:
                    print(f"Error stopping notifier: {e}")
                finally:
                    delattr(self, 'notifier')

            time.sleep(1)
            self.setup_can_interface()
        
    def init_subscriber(self):
        self.motor_command_subscriber = self.create_subscription(MotorCommand, '/publish_motor', self.motor_command_callback, 10)
        self.servo_command_subscriber = self.create_subscription(ServoCommand, '/publish_servo', self.servo_command_callback, 10)
        self.pwm_command_subscriber = self.create_subscription(PwmCommand, '/publish_pwm', self.pwm_command_callback, 10)
        self.digital_solenoid_subscriber = self.create_subscription(DigitalAndSolenoidCommand, '/publish_digital_solenoid', self.digital_and_solenoid_command_callback, 10)
        
    def init_publisher(self):
        self.encoder_publisher = self.create_publisher(EncoderFeedback, '/encoder_feedback', 10)
        self.digital_and_analog_input_publisher = self.create_publisher(DigitalAndAnalogFeedback, '/digital_analog_feedback', 10)

    def send_can_message(self, can_message):
        """Helper method to send CAN messages with error handling"""
        try:
            self.bus.send(can_message)
        except OSError as e:
            if "no buffer space available" in str(e).lower():
                self.get_logger().warn("Buffer space error, message dropped")
            else:
                # For other OSErrors, try to recover the CAN interface
                self.get_logger().error(f"CAN send error: {str(e)}")
                if time.perf_counter() - self.error_timer > 1.0:
                    self.setup_can_interface()
                    self.error_timer = time.perf_counter()
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending CAN message: {str(e)}")

    def motor_command_callback(self, msg):
        self.canMsgData[0] = (
            msg.positionmode +
            (msg.speedmode << 1) +
            (msg.voltagemode << 2) +
            (msg.stop << 3) +
            (msg.reset << 4)
        )
        
        goal_bytes = struct.pack("<f", msg.goal)
        self.canMsgData[2:6] = goal_bytes
        
        can_message = can.Message(arbitration_id=msg.can_id, data=self.canMsgData, is_extended_id=False)
        self.send_can_message(can_message)
            
    def servo_command_callback(self, msg):
        servo_count = 4
        servo_value = [msg.servo1_value, msg.servo2_value, msg.servo3_value, msg.servo4_value]

        for i in range(servo_count):
            if servo_value[i] > 1.0:
                servo_value[i] = 1.0
            elif servo_value[i] < 0.0:
                servo_value[i] = 0.0

            servo_value_int = int(servo_value[i] * 16383.0)

            self.canMsgData[2 * i] = servo_value_int >> 8
            self.canMsgData[2 * i + 1] = servo_value_int & 0xFF

            if i == 0:
                self.canMsgData[0] = self.canMsgData[0] | 0xC0

        can_message = can.Message(arbitration_id=msg.can_id, data=self.canMsgData, is_extended_id=False)
        self.send_can_message(can_message)

    def pwm_command_callback(self, msg):
        pwm_count = 4
        pwm_value = [msg.pwm1_value, msg.pwm2_value, msg.pwm3_value, msg.pwm4_value]

        for i in range(pwm_count):
            if pwm_value[i] > 1.0:
                pwm_value[i] = 1.0
            elif pwm_value[i] < 0.0:
                pwm_value[i] = 0.0

            pwm_value_int = int(pwm_value[i] * 16383.0)

            self.canMsgData[2 * i] = pwm_value_int >> 8
            self.canMsgData[2 * i + 1] = pwm_value_int & 0xFF

            if i == 0:
                self.canMsgData[0] = self.canMsgData[0] | 0x80

        can_message = can.Message(arbitration_id=msg.can_id, data=self.canMsgData, is_extended_id=False)
        self.send_can_message(can_message)

    def digital_and_solenoid_command_callback(self, msg):
        digital_count = 4
        solenoid_count = 6

        digital_value = [msg.digital1_value, msg.digital2_value, msg.digital3_value, msg.digital4_value]
        solenoid_value = [msg.solenoid1_value, msg.solenoid2_value, msg.solenoid3_value, 
                        msg.solenoid4_value, msg.solenoid5_value, msg.solenoid6_value]

        self.canMsgData[0] = 0x40
        self.canMsgData[1] = 0
        self.canMsgData[2] = 0

        for i in range(digital_count):
            self.canMsgData[1] |= (digital_value[i] << i)
        
        for i in range(solenoid_count):
            self.canMsgData[2] |= (solenoid_value[i] << i)

        can_message = can.Message(arbitration_id=msg.can_id, data=self.canMsgData, is_extended_id=False)
        self.send_can_message(can_message)
    def on_message_received(self, msg):
        if 100 <= msg.arbitration_id < 200:
            feedback_msg = EncoderFeedback()
            feedback_msg.can_id = msg.arbitration_id

            feedback_msg.position = struct.unpack_from("<f", msg.data, 0)[0]
            feedback_msg.speed = struct.unpack_from("<f", msg.data, 4)[0]
            self.encoder_publisher.publish(feedback_msg)

        if 500 <= msg.arbitration_id <= 510:
            feedback_msg = DigitalAndAnalogFeedback()
            
            feedback_msg.can_id = msg.arbitration_id

            feedback_msg.analog1_value = ((msg.data[0] << 4) + (msg.data[1] >> 4)) / 4095.0
            feedback_msg.analog2_value = (((msg.data[1] & 0x0F) << 8) + msg.data[2]) / 4095.0
            feedback_msg.analog3_value = ((msg.data[3] << 4) + (msg.data[4] >> 4)) / 4095.0
            feedback_msg.analog4_value = (((msg.data[4] & 0x0F) << 8) + msg.data[5]) / 4095.0
            feedback_msg.analog5_value = ((msg.data[6] << 4) + (msg.data[7] >> 4)) / 4095.0
            
            feedback_msg.digital1_value = (msg.data[7] & 1) == 1
            feedback_msg.digital2_value = ((msg.data[7] >> 1) & 1) == 1 
            feedback_msg.digital3_value = ((msg.data[7] >> 2) & 1) == 1
            feedback_msg.digital4_value = ((msg.data[7] >> 3) & 1) == 1

            self.digital_and_analog_input_publisher.publish(feedback_msg)

    def on_error(self,  exc):
        if time.perf_counter() - self.error_timer > 1.0:
            self.setup_can_interface()
            self.error_timer = time.perf_counter()

    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info(f"Received signal {signum}. Initiating shutdown...")
        self.shutdown_requested = True

    def shutdown(self):
        """Shuts down the CAN interface and other resources gracefully."""
        if self.shutdown_requested:
            return
        
        self.get_logger().info("Shutdown requested. Cleaning up...")
        self.shutdown_requested = True

        # Stop all ongoing operations
        self.notifier.stop()
        
        # Close the CAN bus
        self.bus.shutdown()
        
        # Shutdown the can0 interface
        result = run_subprocess(["sudo", "ip", "link", "set", "can0", "down"])
        if result.returncode == 0:
            self.get_logger().info("CAN interface successfully shut down.")
        else:
            self.get_logger().error("Failed to shut down CAN interface.")
        
        # Destroy the node.
        self.destroy_node()
        self.get_logger().info("Node destroyed. Shutdown complete.")

def main(args=None):
    rclpy.init(args=args)
    can_driver_node = CanDriver()
    try:
        while rclpy.ok() and not can_driver_node.shutdown_requested:
            rclpy.spin_once(can_driver_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        can_driver_node.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
