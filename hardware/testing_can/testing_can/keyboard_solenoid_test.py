#!/usr/bin/env python3
"""
keyboard_solenoid_test.py

Simple manual test for the pneumatic suction gripper on the SCARA robot.

    w  ->  suction ON   (energize solenoid)
    s  ->  suction OFF  (vent / release)
    q  ->  quit

It publishes a DigitalAndSolenoidCommand on /publish_digital_solenoid, which
can_driver.py packs into a CAN frame (byte0 = 0x40 header, byte2 = solenoid
bits) and sends to the Output2CAN board on the bus.

Run the CAN driver in one terminal first:
    ros2 run can_driver can_driver
then this node in another terminal:
    ros2 run testing_can keyboard_solenoid_test
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from custom_messages.msg import DigitalAndSolenoidCommand

# ---------------------------------------------------------------------------
# CONFIG  --  change these two lines if the test does not work
# ---------------------------------------------------------------------------
CAN_ID = 4            # board CAN id. If nothing clicks, try 600 (all the old
                      # scripts use 600, so that is the likely real value).
SUCTION_CHANNEL = 1   # which solenoid output the suction valve is wired to.
                      # 1 -> D1 (solenoid1_value), 2 -> D2, 3 -> D3, 4 -> D4
# ---------------------------------------------------------------------------


class KeyboardSolenoidTest(Node):
    def __init__(self):
        super().__init__('keyboard_solenoid_test')
        self.publisher = self.create_publisher(
            DigitalAndSolenoidCommand, '/publish_digital_solenoid', 10)

        # current state of the suction (False = off)
        self.suction_on = False

        # re-publish at 20 Hz so the board keeps getting a fresh command even
        # if a single frame is dropped on the bus
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info(
            f"Solenoid test ready. CAN_ID={CAN_ID}, channel D{SUCTION_CHANNEL}.\n"
            "  w = suction ON   |   s = suction OFF   |   q = quit")

    def set_suction(self, state: bool):
        """Update state and log it."""
        self.suction_on = state
        self.get_logger().info("Suction ON" if state else "Suction OFF")
        self.publish_state()

    def publish_state(self):
        """Build and send the command reflecting the current suction state."""
        msg = DigitalAndSolenoidCommand()
        msg.can_id = CAN_ID

        # all outputs default to False; set only the channel we use
        if SUCTION_CHANNEL == 1:
            msg.solenoid1_value = self.suction_on
        elif SUCTION_CHANNEL == 2:
            msg.solenoid2_value = self.suction_on
        elif SUCTION_CHANNEL == 3:
            msg.solenoid3_value = self.suction_on
        elif SUCTION_CHANNEL == 4:
            msg.solenoid4_value = self.suction_on

        self.publisher.publish(msg)


def get_key(timeout=0.1):
    """Non-blocking single-key read from the terminal."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        # wait up to `timeout` seconds for a key, so rclpy can keep spinning
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSolenoidTest()
    try:
        while rclpy.ok():
            # let timers / publishing run
            rclpy.spin_once(node, timeout_sec=0.0)

            key = get_key(timeout=0.1)
            if key == 'w':
                node.set_suction(True)
            elif key == 's':
                node.set_suction(False)
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        # always release the suction on exit so nothing stays stuck holding
        node.set_suction(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
