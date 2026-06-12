import rclpy
import time
import can
import subprocess
import struct
import threading


class ODriveCANNode:
    def __init__(self, can_id):
        self.can_id = can_id
        self.encoder_value = 0.0
        self.bus = None

    def send_can_message(self, arbitration_id, data):
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            print(f"  → Sent message ID {hex(arbitration_id)}: {data}")
        except can.CanError as e:
            print(f"  ✗ Failed to send message: {e}")

    def listen_can_messages(self):
        while True:
            msg = self.bus.recv(timeout=1.0)
            if msg is not None:
                expected_id = (self.can_id << 5) | 0x009
                if msg.arbitration_id == expected_id and len(msg.data) == 8:
                    pos, vel = struct.unpack('<ff', msg.data)
                    self.encoder_value = pos

    def full_calibration_sequences(self):
        arbitration_id = (self.can_id << 5) | 0x007
        data = [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.send_can_message(arbitration_id, data)

    def set_torque_control_mode(self):
        arbitration_id = (self.can_id << 5) | 0x00B
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.send_can_message(arbitration_id, data)

    def set_current_position_to_zero_odrive(self):
        arbitration_id = (self.can_id << 5) | 0x19
        data = struct.pack('<f', 0.0)
        self.send_can_message(arbitration_id, data)

    def set_closed_loop_control(self):
        arbitration_id = (self.can_id << 5) | 0x007
        data = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.send_can_message(arbitration_id, data)

    def set_position_control_mode(self):
        arbitration_id = (self.can_id << 5) | 0x00B
        data = [0x03, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00]
        self.send_can_message(arbitration_id, data)


def initialize_odrive(bus, can_id: int, calibration_wait: int = 20):
    print(f"\n{'='*45}")
    print(f"  Initializing ODrive — CAN ID {can_id}")
    print(f"{'='*45}")

    node = ODriveCANNode(can_id)
    node.bus = bus
    node.listener_thread = threading.Thread(target=node.listen_can_messages, daemon=True)
    node.listener_thread.start()

    print(f"\n[1/5] Calibrating motor (CAN ID {can_id})...")
    node.full_calibration_sequences()
    print(f"      Waiting {calibration_wait}s for calibration to complete...")
    time.sleep(calibration_wait)

    print(f"\n[2/5] Setting torque control mode (CAN ID {can_id})...")
    node.set_torque_control_mode()
    time.sleep(0.5)

    print(f"\n[3/5] Setting current position to zero (CAN ID {can_id})...")
    node.set_current_position_to_zero_odrive()
    time.sleep(0.5)

    print(f"\n[4/5] Setting closed-loop control (CAN ID {can_id})...")
    node.set_closed_loop_control()
    time.sleep(0.5)

    print(f"\n[5/5] Setting position control mode (CAN ID {can_id})...")
    node.set_position_control_mode()
    time.sleep(0.5)

    print(f"\n✓ CAN ID {can_id} initialization complete.")


def main():
    rclpy.init()

    print("Setting up CAN interface...")
    result = subprocess.run(["ip", "link", "show", "can0"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if b"state UP" not in result.stdout:
        subprocess.run(["sudo", "ip", "link", "set", "can0", "up",
                        "type", "can", "bitrate", "1000000"])
        print("CAN interface can0 brought up.")
    else:
        print("CAN interface can0 already up.")

    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

    try:
        for can_id in [1, 2]:
            initialize_odrive(bus, can_id, calibration_wait=20)

        print("\n✓ All axes initialized successfully.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        bus.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()