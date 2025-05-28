import time
import numpy as np
import serial

class ArduinoArmControllerSerial:
    def __init__(self, port='COM3', baud_rate=57600):
        """
        Initialize serial communication with Arduino for 4DOF robot arm
        Args:
            port: Serial port (e.g., 'COM3' for Windows, '/dev/ttyUSB0' for Linux)
            baud_rate: Serial communication baud rate
        """
        self.port = port
        self.baud_rate = baud_rate
        self.serial_connection = None

        try:
            self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2) # Wait for the serial connection to initialize
            print("✅ Serial connection to Arduino successful")

            # Set initial angles (matching the Arduino sketch's initial values)
            self.current_angles_deg = {
                'base': 80,  # Base rotation
                'shoulder': 60, # Shoulder joint
                'elbow': 130,  # Elbow joint
                'gripper': 0   # Gripper
            }
            # Send initial position command to Arduino to sync
            self.set_joint_angles_deg(self.current_angles_deg)
            time.sleep(2) # Give Arduino time to move

        except serial.SerialException as e:
            print(f"❌ Error opening serial port {port}: {e}")
            # Do not re-raise, allow fallback to simulation
            self.serial_connection = None # Ensure connection is None on failure
        except Exception as e:
            print(f"❌ An unexpected error occurred during serial connection: {e}")
            # Do not re-raise, allow fallback to simulation
            self.serial_connection = None # Ensure connection is None on failure

    def set_joint_angles_deg(self, angles_deg):
        """
        Send target joint angles (in degrees) to the Arduino over serial.
        Args:
            angles_deg: Dictionary of joint angles in degrees
                        {'base': angle, 'shoulder': angle, 'elbow': angle, 'gripper': angle}
        """
        if self.serial_connection and self.serial_connection.isOpen():
            try:
                # Format the command string: base,shoulder,elbow,gripper\n
                # Ensure all required angles are present, use current if missing
                base_angle = angles_deg.get('base', self.current_angles_deg.get('base', 90))
                shoulder_angle = angles_deg.get('shoulder', self.current_angles_deg.get('shoulder', 90))
                elbow_angle = angles_deg.get('elbow', self.current_angles_deg.get('elbow', 90))
                gripper_angle = angles_deg.get('gripper', self.current_angles_deg.get('gripper', 90))

                command = f"{int(base_angle)},{int(shoulder_angle)},{int(elbow_angle)},{int(gripper_angle)}\n"
                self.serial_connection.write(command.encode())
                # Update current angles (assuming command is successfully sent)
                self.current_angles_deg['base'] = base_angle
                self.current_angles_deg['shoulder'] = shoulder_angle
                self.current_angles_deg['elbow'] = elbow_angle
                self.current_angles_deg['gripper'] = gripper_angle

                # Small delay after sending command, adjust if needed
                # time.sleep(0.05)

            except Exception as e:
                print(f"❌ Error sending data over serial: {e}")
        # else:
            # print("Serial connection not open. Cannot send angles.") # Optional: log if connection is closed

    def close(self):
        """
        Close the serial connection.
        """
        if self.serial_connection and self.serial_connection.isOpen():
            self.serial_connection.close()
            print("Serial connection closed")

if __name__ == "__main__":
    # Test code
    try:
        arm = ArduinoArmControllerSerial()
        
        # Test each joint
        print("Testing base rotation...")
        arm.set_joint_angles_deg({'base': 0})
        time.sleep(1)
        arm.set_joint_angles_deg({'base': 180})
        time.sleep(1)
        arm.set_joint_angles_deg({'base': 90})
        
        print("Testing shoulder joint...")
        arm.set_joint_angles_deg({'shoulder': 0})
        time.sleep(1)
        arm.set_joint_angles_deg({'shoulder': 180})
        time.sleep(1)
        arm.set_joint_angles_deg({'shoulder': 90})
        
        print("Testing elbow joint...")
        arm.set_joint_angles_deg({'elbow': 0})
        time.sleep(1)
        arm.set_joint_angles_deg({'elbow': 180})
        time.sleep(1)
        arm.set_joint_angles_deg({'elbow': 90})
        
        print("Testing gripper...")
        arm.set_joint_angles_deg({'gripper': 0})
        time.sleep(1)
        arm.set_joint_angles_deg({'gripper': 180})
        time.sleep(1)
        arm.set_joint_angles_deg({'gripper': 90})
        
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        if 'arm' in locals():
            arm.close()

    # Example Usage (for testing this module directly if needed)
    # if __name__ == "__main__":
    #     try:
    #         arm_serial = ArduinoArmControllerSerial(port='COM3') # Adjust port
    #         time.sleep(3)
    #         print("Moving arm to angles: 45, 90, 45, 30")
    #         arm_serial.set_joint_angles_deg({'base': 45, 'shoulder': 90, 'elbow': 45, 'gripper': 30})
    #         time.sleep(3)
    #         print("Moving arm to angles: 135, 30, 150, 90")
    #         arm_serial.set_joint_angles_deg({'base': 135, 'shoulder': 30, 'elbow': 150, 'gripper': 90})
    #         time.sleep(3)
    #     except Exception as e:
    #         print(f"Error during serial test: {e}")
    #     finally:
    #         if 'arm_serial' in locals():
    #             arm_serial.close() 