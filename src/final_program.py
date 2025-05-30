"""Main program for robotic arm control"""
import numpy as np
import cv2
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
from vision_system import VisionSystem
import matplotlib.pyplot as plt
from arduino_control import ArduinoArmControllerSerial
from position_utils import calculate_safe_picking_position, get_relative_position
from config import (
    SIMULATION_MODE, CAMERA_INDEX, ARDUINO_PORT, ARDUINO_BAUD_RATE,
    INITIAL_ARM_ANGLES_RAD, HOME_POSITION
)

# Inicializações
arm = RobotArmRRR()  # Using default config values
planner = TrajectoryPlanner()  # Using default config values
simulator = DigitalTwinSimulator()
vision = VisionSystem()  # Using default config values

# Set up controllers
planner.set_arm_controller(arm)

# FIXED: Initialize vision system attributes to prevent AttributeError
vision.last_transforms = {}
vision.last_home_distance = 0.0
vision.last_is_at_home = False
vision.last_status = "Aguardando"
vision.last_color = (255, 255, 255)

# Global simulation variables
current_angles = INITIAL_ARM_ANGLES_RAD

def send_to_real_robot(joint_angles_rad):
    """Send angles to robot - now with Arduino support for base rotation"""
    if SIMULATION_MODE:
        simulator.update_arm(joint_angles_rad)
        
        # If Arduino is connected, send base angle
        if hasattr(send_to_real_robot, 'arduino_controller') and send_to_real_robot.arduino_controller:
            send_to_real_robot.arduino_controller.send_to_robot(joint_angles_rad)
    else:
        print(f"[REAL] Sending angles: {np.degrees(joint_angles_rad)}")

# Set the robot controller in the planner
planner.set_robot_controller(send_to_real_robot)

def main():
    global current_angles
    
    print("==============================")
    print(" SISTEMA DE PICK AND PLACE")
    print("==============================")
    print(f"Modo: Simulação")
    
    # Try to initialize Arduino connection
    try:
        arduino_controller = ArduinoArmControllerSerial(port=ARDUINO_PORT, baud_rate=ARDUINO_BAUD_RATE)
        if arduino_controller.serial_connection:
            print("✅ Arduino conectado com sucesso! Base física será controlada.")
            send_to_real_robot.arduino_controller = arduino_controller
        else:
            print("⚠️ Arduino não conectado. Continuando apenas com simulação.")
            send_to_real_robot.arduino_controller = None
    except Exception as e:
        print(f"⚠️ Erro ao conectar com Arduino: {e}")
        print("Continuando apenas com simulação.")
        send_to_real_robot.arduino_controller = None

    # Initialize simulation
    try:
        simulator.initialize_simulation()
        print("✅ Simulação inicializada com sucesso")
    except Exception as e:
        print(f"⚠️ Erro na inicialização da simulação: {e}")
    
    # Configure camera window
    cv2.namedWindow("Vision System", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Vision System", 800, 600)
    
    # Initialize camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Erro: Não foi possível abrir a câmera")
        return
    
    current_angles = INITIAL_ARM_ANGLES_RAD
    state = "WAITING_FOR_CAR"
    
    # Send initial position to simulation and Arduino
    send_to_real_robot(current_angles)
    
    print(f"\nPosicione o carro entre {vision.HOME_MIN_THRESHOLD*100:.1f}cm e {vision.HOME_MAX_THRESHOLD*100:.1f}cm do ponto Home")
    
    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("❌ Erro: Não foi possível ler o frame da câmera")
                break

            # Process vision - using original VisionSystem methods
            detected = vision.detect_markers(frame)
            transforms = vision.get_transforms(detected)
            vision.last_transforms = transforms
            vision.is_car_at_home(transforms)
            vision.draw_status_overlay(frame)
            
            # Update simulation markers
            simulator.update_from_vision(transforms)
            
            # State machine
            if state == "WAITING_FOR_CAR":
                if vision.is_car_at_home(transforms):
                    print("\n✅ Carro detectado na posição Home!")
                    state = "PICKING"
                    
                    # Get car position relative to home marker
                    car_pos_cm = get_relative_position(
                        vision.last_transforms['home'],
                        vision.last_transforms['car']
                    )
                    
                    # Calculate safe picking position
                    picking_pos = calculate_safe_picking_position(car_pos_cm)
                    print(f"\n🎯 Posição de picking calculada: {np.round(picking_pos, 1)}cm")
                    
                    # Move to picking position
                    success, current_angles = planner.move_through_safe_path(picking_pos, current_angles)
                    if not success:
                        print("❌ Falha ao mover para posição de picking")
                        state = "WAITING_FOR_CAR"
                    else:
                        print("✅ Braço na posição de picking")
                        state = "WAITING_FOR_DROPOFF"
            
            elif state == "WAITING_FOR_DROPOFF":
                if 'dropoff' in transforms:
                    print("\n✅ Marcador de dropoff detectado!")
                    
                    # Get dropoff position relative to home marker
                    dropoff_pos_cm = get_relative_position(
                        vision.last_transforms['home'],
                        vision.last_transforms['dropoff']
                    )
                    
                    # Calculate safe dropoff position
                    dropoff_pos = calculate_safe_picking_position(dropoff_pos_cm)
                    print(f"\n🎯 Posição de dropoff calculada: {np.round(dropoff_pos, 1)}cm")
                    
                    # Move to dropoff position
                    success, current_angles = planner.move_through_safe_path(dropoff_pos, current_angles)
                    if not success:
                        print("❌ Falha ao mover para posição de dropoff")
                    else:
                        print("✅ Movimento concluído")
                    
                    # Return to initial position
                    print("\n🔄 Retornando à posição inicial...")
                    success, current_angles = planner.move_through_safe_path(
                        HOME_POSITION, current_angles
                    )
                    if success:
                        print("✅ Retorno concluído")
                    else:
                        print("⚠️ Falha no retorno à posição inicial")
                    
                    state = "WAITING_FOR_CAR"
            
            # Show frame
            cv2.imshow("Vision System", frame)
            
            # Check for key press
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("\n👋 Encerrando sistema...")
                break
            elif key == ord('r'):
                print("\n🔄 Resetando sistema...")
                state = "WAITING_FOR_CAR"
                current_angles = INITIAL_ARM_ANGLES_RAD
                send_to_real_robot(current_angles)
    
    finally:
        print("🧹 Limpando recursos...")
        # Close Arduino connection if it exists
        if hasattr(send_to_real_robot, 'arduino_controller') and send_to_real_robot.arduino_controller:
            send_to_real_robot.arduino_controller.close()
        cap.release()
        cv2.destroyAllWindows()
        plt.close('all')

if __name__ == "__main__":
    main()
