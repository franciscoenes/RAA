import time
import numpy as np
import serial
from config import (ARDUINO_PORT, ARDUINO_BAUD_RATE, INITIAL_ARM_ANGLES_RAD)

class ArduinoArmControllerSerial:
    def __init__(self, port=ARDUINO_PORT, baud_rate=ARDUINO_BAUD_RATE):
        """
        Initialize serial communication with Arduino for base rotation control
        Args:
            port: Serial port (e.g., 'COM3' for Windows, '/dev/ttyUSB0' for Linux)
            baud_rate: Serial communication baud rate
        """
        self.port = port
        self.baud_rate = baud_rate
        self.serial_connection = None

        try:
            self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for the serial connection to initialize
            print("✅ Serial connection to Arduino successful")

            # Set initial base angle (using config values)
            self.current_angles_deg = {
                'base': np.degrees(INITIAL_ARM_ANGLES_RAD[0]),
                'shoulder': np.degrees(INITIAL_ARM_ANGLES_RAD[1]),
                'elbow': np.degrees(INITIAL_ARM_ANGLES_RAD[2]),
                'gripper': 0
            }
            
            # Send initial base position to Arduino
            self.set_joint_angles_deg(self.current_angles_deg)
            time.sleep(2)  # Give Arduino time to move

        except serial.SerialException as e:
            print(f"❌ Error opening serial port {port}: {e}")
            self.serial_connection = None
        except Exception as e:
            print(f"❌ An unexpected error occurred during serial connection: {e}")
            self.serial_connection = None

    def set_joint_angles_deg(self, angles_deg):
        """
        Send only base angle to the Arduino over serial.
        Other angles are stored but not sent.
        Args:
            angles_deg: Dictionary of joint angles in degrees
                        {'base': angle, 'shoulder': angle, 'elbow': angle, 'gripper': angle}
        """
        if self.serial_connection and self.serial_connection.isOpen():
            try:
                # Only get base angle, use current if missing
                base_angle = angles_deg.get('base', self.current_angles_deg.get('base', 0))
                
                # Store all angles (for simulation compatibility)
                self.current_angles_deg['base'] = base_angle
                self.current_angles_deg['shoulder'] = angles_deg.get('shoulder', self.current_angles_deg.get('shoulder'))
                self.current_angles_deg['elbow'] = angles_deg.get('elbow', self.current_angles_deg.get('elbow'))
                self.current_angles_deg['gripper'] = angles_deg.get('gripper', self.current_angles_deg.get('gripper'))

                # Na simulação: 
                # - ângulos positivos = direita
                # - ângulos negativos = esquerda
                # No servo:
                # - 0° = centro
                # - 90° = esquerda
                
                # Converte ângulo da simulação para ângulo do servo
                if base_angle < 0:  # Se negativo (esquerda na simulação)
                    servo_angle = abs(base_angle)  # Converte para positivo (esquerda no servo)
                    print(f"🔄 Convertendo ângulo negativo {base_angle}° para servo: {servo_angle}°")
                else:  # Se positivo (direita na simulação)
                    servo_angle = 0  # Por enquanto, mantém no centro para ângulos positivos
                    print(f"🔄 Ângulo positivo {base_angle}° -> mantendo no centro")
                
                # Limita entre 0-90
                servo_angle = max(0, min(90, servo_angle))
                
                # Send angle to Arduino
                command = f"{int(servo_angle)}\n"  # Convert to integer to avoid floating point issues
                print(f"📡 Enviando ângulo para Arduino: {servo_angle}° (ângulo original: {base_angle}°)")
                self.serial_connection.write(command.encode())
                self.serial_connection.flush()  # Ensure data is sent

            except Exception as e:
                print(f"❌ Error sending base angle over serial: {e}")

    def send_to_robot(self, joint_angles_rad):
        """Send angles to robot with Arduino support for base rotation"""
        try:
            # Convert base angle from radians to degrees
            base_angle_deg = np.degrees(joint_angles_rad[0])
            print(f"🔄 Ângulo base (rad): {joint_angles_rad[0]:.3f}, (graus): {base_angle_deg:.1f}°")
            self.set_joint_angles_deg({'base': base_angle_deg})
        except Exception as e:
            print(f"⚠️ Warning: Could not send angle to Arduino: {e}")

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
        
        # Test base rotation only
        print("Testing base rotation...")
        arm.set_joint_angles_deg({'base': 0})
        time.sleep(1)
        arm.set_joint_angles_deg({'base': 180})
        time.sleep(1)
        arm.set_joint_angles_deg({'base': 90})
        time.sleep(1)
        
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