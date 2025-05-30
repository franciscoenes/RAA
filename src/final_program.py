import numpy as np
import time
import cv2
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
from vision_system import VisionSystem
import matplotlib.pyplot as plt
from spatialmath import SE3
from arduino_control import ArduinoArmControllerSerial
from config import (
    SIMULATION_MODE, CAMERA_INDEX, MARKER_SIZE_CM, CAMERA_HEIGHT_CM,
    VISION_SCALE_FACTOR, ARDUINO_PORT, ARDUINO_BAUD_RATE,
    INITIAL_ARM_ANGLES_RAD, LINK_LENGTHS_METERS, WORKSPACE_LIMITS,
    HOME_POSITION, CAMERA_PARAMS_FILE, SIMULATION_PLOT_LIMITS,
    TRAJECTORY_DT
)

# Inicializações
arm = RobotArmRRR()  # Using default config values
planner = TrajectoryPlanner()  # Using default config values
simulator = DigitalTwinSimulator()
vision = VisionSystem()  # Using default config values

# FIXED: Initialize vision system attributes to prevent AttributeError
vision.last_transforms = {}
vision.last_home_distance = 0.0
vision.last_is_at_home = False
vision.last_status = "Aguardando"
vision.last_color = (255, 255, 255)

# Global simulation variables
simulation_env = None
current_angles = INITIAL_ARM_ANGLES_RAD
# ADD: Global marker variables
car_marker = None
home_marker = None
dropoff_marker = None

def initialize_simulation():
    """Initialize the simulation environment cleanly"""
    global simulation_env, car_marker, home_marker, dropoff_marker
    
    plt.ion()  # Enable interactive mode
    
    # Use Robotics Toolbox to create the environment
    simulation_env = simulator.robot.plot(
        INITIAL_ARM_ANGLES_RAD,
        backend='pyplot',
        block=False,
        jointaxes=True,
        eeframe=True,
        shadow=False
    )
    
    # Configure the plot
    ax = simulation_env.ax
    ax.set_xlim(-0.2, 0.5)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(0, 0.4)
    ax.set_title("Simulação do Braço Robótico")
    
    # ADD: Initialize marker plots with empty data
    car_marker, = ax.plot([], [], [], 'ro', label="Carro")
    home_marker, = ax.plot([], [], [], 'g^', label="Home")
    dropoff_marker, = ax.plot([], [], [], 'm^', label="Drop-off")
    ax.legend()
    
    plt.show(block=False)
    return simulation_env

def update_simulation_display(joint_angles_rad):
    """Update simulation display cleanly without blocking"""
    global simulation_env
    
    if simulation_env is not None:
        try:
            # First update the robot's joint configuration
            simulation_env.robots[0].robot.q = joint_angles_rad
            
            # Then call step to update the display
            simulation_env.step(dt=0.01)
            
            # ADD: Update markers if they exist
            if all(m is not None for m in [car_marker, home_marker, dropoff_marker]):
                update_simulation_markers(simulation_env.ax, simulator, car_marker, home_marker, dropoff_marker)
            
            # Brief pause for smooth animation
            plt.pause(0.001)
        except Exception as e:
            print(f"Warning: Simulation update error: {e}")

# ADD: Your working marker update function
def update_simulation_markers(ax, simulator, car_marker, home_marker, dropoff_marker):
    """Updates the positions of the marker plots in the simulation."""
    if simulator.car_pose is not None and simulator.home_pose is not None:
        # Calculate car position relative to home
        car_pos_relative = simulator.home_pose.inv() * simulator.car_pose
        car_marker.set_data([car_pos_relative.t[0]], [car_pos_relative.t[1]])
        car_marker.set_3d_properties([car_pos_relative.t[2]])
    else:
        car_marker.set_data([], [])
        car_marker.set_3d_properties([])

    # Home marker is always at (0,0,0) in this relative frame
    if simulator.home_pose is not None:
        home_marker.set_data([0], [0])
        home_marker.set_3d_properties([0])
    else:
        home_marker.set_data([], [])
        home_marker.set_3d_properties([])

    if simulator.dropoff_pose is not None and simulator.home_pose is not None:
        # Calculate dropoff position relative to home
        dropoff_pos_relative = simulator.home_pose.inv() * simulator.dropoff_pose
        dropoff_marker.set_data([dropoff_pos_relative.t[0]], [dropoff_pos_relative.t[1]])
        dropoff_marker.set_3d_properties([dropoff_pos_relative.t[2]])
    else:
        dropoff_marker.set_data([], [])
        dropoff_marker.set_3d_properties([])

def send_to_real_robot(joint_angles_rad):
    """Send angles to robot - now with Arduino support for base rotation"""
    if SIMULATION_MODE:
        simulator.update_arm(joint_angles_rad)
        update_simulation_display(joint_angles_rad)
        
        # If Arduino is connected, send base angle
        if hasattr(send_to_real_robot, 'arduino_controller') and send_to_real_robot.arduino_controller:
            try:
                # Convert base angle from radians to degrees
                base_angle_deg = np.degrees(joint_angles_rad[0])
                print(f"🔄 Ângulo base (rad): {joint_angles_rad[0]:.3f}, (graus): {base_angle_deg:.1f}°")
                send_to_real_robot.arduino_controller.set_joint_angles_deg({'base': base_angle_deg})
            except Exception as e:
                print(f"⚠️ Warning: Could not send angle to Arduino: {e}")
    else:
        print(f"[REAL] Sending angles: {np.degrees(joint_angles_rad)}")

def normalize_angle(angle):
    """Normalize angle to be between -180 and 180 degrees"""
    return ((angle + 180) % 360) - 180

def get_best_ik_solution(solutions, current_angles):
    """Choose the best IK solution based on joint limits"""
    if not solutions:
        return None
    
    current_angles = np.array(current_angles)
    joint_limits_deg = np.array([
        [-90, 90],   # Base limits
        [-90, 90],   # Shoulder limits
        [-90, 90]    # Elbow limits
    ])
    
    # Filter solutions within joint limits
    valid_solutions = []
    print("\n🔍 Soluções IK encontradas:")
    for i, sol in enumerate(solutions):
        angles_deg = np.degrees(sol)
        print(f"Solução {i+1}: Base={angles_deg[0]:.1f}°, Shoulder={angles_deg[1]:.1f}°, Elbow={angles_deg[2]:.1f}°")
        
        is_valid = True
        for j, (angle_deg, (min_deg, max_deg)) in enumerate(zip(angles_deg, joint_limits_deg)):
            norm_angle = normalize_angle(angle_deg)
            if norm_angle < min_deg or norm_angle > max_deg:
                is_valid = False
                break
        if is_valid:
            valid_solutions.append(sol)
    
    if valid_solutions:
        print(f"✅ {len(valid_solutions)} soluções válidas encontradas")
        # Choose solution with minimum angle change
        best_sol = None
        min_cost = float('inf')
        for sol in valid_solutions:
            angle_change_cost = np.sum(np.abs(sol - current_angles))
            if angle_change_cost < min_cost:
                min_cost = angle_change_cost
                best_sol = sol
        return best_sol
    
    print("❌ Nenhuma solução válida dentro dos limites das juntas")
    return None

def is_position_reachable(target_pos_cm):
    """Check if a position is reachable by the arm with IK validation"""
    xy_distance = np.sqrt(target_pos_cm[0]**2 + target_pos_cm[1]**2)
    
    # Check workspace limits first
    if xy_distance < WORKSPACE_LIMITS['xy_min']:
        print(f"❌ Posição muito próxima da base: {xy_distance:.1f}cm < {WORKSPACE_LIMITS['xy_min']:.1f}cm")
        return False
    
    if xy_distance > WORKSPACE_LIMITS['xy_max']:
        print(f"❌ Posição muito distante: {xy_distance:.1f}cm > {WORKSPACE_LIMITS['xy_max']:.1f}cm")
        return False
    
    if target_pos_cm[2] < WORKSPACE_LIMITS['z_min'] or target_pos_cm[2] > WORKSPACE_LIMITS['z_max']:
        print(f"❌ Altura fora dos limites: {target_pos_cm[2]:.1f}cm")
        return False
    
    # Test with inverse kinematics
    target_pos_m = np.array(target_pos_cm) / 100.0
    solutions = arm.InvKin(target_pos_m)
    
    if not solutions:
        print(f"❌ Posição geometricamente inatingível: {target_pos_cm}")
        return False
    
    # Check if any solution is within joint limits
    best_solution = get_best_ik_solution(solutions, current_angles)
    if best_solution is not None:
        return True
    
    print(f"❌ Posição fora dos limites das juntas: {target_pos_cm}")
    return False

def move_to_position(target_pos_cm, current_angles):
    """Move arm to position - simplified and clean"""
    try:
        # Convert to meters
        target_pos_m = np.array(target_pos_cm) / 100.0
        
        print(f"\n🎯 Movendo para posição: {np.round(target_pos_cm, 1)}cm")
        
        # Check reachability
        if not is_position_reachable(target_pos_cm):
            return False, current_angles
        
        # Get IK solutions
        solutions = arm.InvKin(target_pos_m)
        if not solutions:
            print("❌ Posição inatingível pelo braço.")
            return False, current_angles
        
        # Get best solution
        q_target_rad = get_best_ik_solution(solutions, current_angles)
        if q_target_rad is None:
            print("❌ Não foi possível encontrar uma solução válida.")
            return False, current_angles
        
        print(f"🎯 Ângulos alvo (graus): {np.round(np.degrees(q_target_rad), 1)}")
        
        # Generate trajectory
        q_trajectory_rad, dq, ddq, t = planner.generate_trajectory(
            current_angles, q_target_rad, v0=[0, 0, 0], vf=[0, 0, 0], T=2.0
        )
        
        # Execute trajectory cleanly
        print("🔄 Executando trajetória...")
        for i in range(len(t)):
            q_i_rad = q_trajectory_rad[i]
            send_to_real_robot(q_i_rad)
            time.sleep(0.02)  # Small delay for smooth movement
        
        print("✅ Movimento concluído")
        return True, q_target_rad
        
    except Exception as e:
        print(f"❌ Erro durante movimento: {str(e)}")
        return False, current_angles

def calculate_safe_picking_position(car_pos_cm):
    """Calculate a safe picking position"""
    car_pos = np.array(car_pos_cm)
    xy_distance = np.sqrt(car_pos[0]**2 + car_pos[1]**2)
    
    # Adjust if too close to base
    if xy_distance < WORKSPACE_LIMITS['xy_min']:
        scale = WORKSPACE_LIMITS['xy_min'] / xy_distance
        car_pos[0] *= scale
        car_pos[1] *= scale
    
    # Set safe height - reduced height for better reachability
    picking_height = max(WORKSPACE_LIMITS['z_min'], min(car_pos[2] + 10.0, 18.0))  # Maximum 18cm
    return np.array([car_pos[0], car_pos[1], picking_height])

def get_relative_position(home_transform, target_transform):
    """Calculate relative position between markers"""
    relative_transform = home_transform.inv() * target_transform
    pos_m = relative_transform.t
    pos_cm = pos_m * 100.0
    # Transform to arm coordinates (Z=0 as markers are on table)
    return np.array([pos_cm[0], pos_cm[1], 0.0])

def move_through_safe_path(target_pos_cm, current_angles):
    """Move arm through safe intermediate points with better error handling"""
    try:
        # Check if target position is reachable
        if not is_position_reachable(target_pos_cm):
            return False, current_angles
        
        # Get current position
        current_pos_m = arm.FwdKin(current_angles)
        current_pos_cm = current_pos_m * 100.0
        
        print("\n=== Planejamento de Movimento ===")
        print(f"Posição atual (cm): {np.round(current_pos_cm, 1)}")
        print(f"Posição alvo (cm): {np.round(target_pos_cm, 1)}")
        
        # Try direct movement first if target is reachable
        print("\n🔄 Tentando movimento direto...")
        success, new_angles = move_to_position(target_pos_cm, current_angles)
        if success:
            print("✅ Movimento direto bem-sucedido!")
            return True, new_angles
        
        # If direct movement fails, try with intermediate points
        print("\n🔄 Movimento direto falhou. Tentando caminho seguro...")
        
        # Calculate a safe intermediate position closer to current position
        current_xy_dist = np.sqrt(current_pos_cm[0]**2 + current_pos_cm[1]**2)
        target_xy_dist = np.sqrt(target_pos_cm[0]**2 + target_pos_cm[1]**2)
        
        # Choose intermediate position between current and target
        intermediate_distance = (current_xy_dist + target_xy_dist) / 2
        
        # Calculate intermediate XY position
        target_direction = np.array([target_pos_cm[0], target_pos_cm[1]])
        target_direction_norm = target_direction / np.linalg.norm(target_direction)
        
        intermediate_xy = target_direction_norm * intermediate_distance
        intermediate_height = max(current_pos_cm[2], target_pos_cm[2]) + 2.0  # Add 2cm clearance
        
        intermediate_pos = np.array([intermediate_xy[0], intermediate_xy[1], intermediate_height])
        
        print(f"Posição intermediária: {np.round(intermediate_pos, 1)}")
        
        # Move to intermediate position
        print("\n🔄 Movendo para posição intermediária...")
        success, current_angles = move_to_position(intermediate_pos, current_angles)
        if not success:
            print("❌ Falha ao mover para posição intermediária")
            return False, current_angles
        
        # Move to target
        print("\n🔄 Movendo para posição alvo...")
        success, current_angles = move_to_position(target_pos_cm, current_angles)
        if not success:
            print("❌ Falha ao mover para posição alvo")
            return False, current_angles
        
        return True, current_angles
        
    except Exception as e:
        print(f"❌ Erro durante movimento seguro: {str(e)}")
        return False, current_angles

def main():
    global current_angles, simulation_env
    
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
        simulation_env = initialize_simulation()
        print("✅ Simulação inicializada com sucesso")
    except Exception as e:
        print(f"⚠️ Erro na inicialização da simulação: {e}")
        simulation_env = None
    
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
            # Process camera frame (separate from trajectory execution)
            ret, frame = cap.read()
            if not ret:
                print("❌ Falha ao capturar frame da câmera")
                break
            
            # Vision processing
            detected = vision.detect_markers(frame)
            transforms = vision.get_transforms(detected)
            vision.last_transforms = transforms
            simulator.update_from_vision(transforms)
            
            # Check car position
            is_at_home = vision.is_car_at_home(transforms)
            vision.draw_status_overlay(frame)
            cv2.imshow("Vision System", frame)
            
            # State machine
            if state == "WAITING_FOR_CAR":
                if is_at_home and 'car' in transforms and 'home' in transforms:
                    print("\n✅ Carro detectado na posição correta!")
                    print(f"Distância até Home: {vision.last_home_distance:.1f}cm")
                    state = "PICKING"
                    
                    try:
                        home_transform = SE3(transforms['home'])
                        car_transform = SE3(transforms['car'])
                        car_pos_cm = get_relative_position(home_transform, car_transform)
                        pick_pos_cm = calculate_safe_picking_position(car_pos_cm)
                        
                        print(f"🎯 Posição de picking: {np.round(pick_pos_cm, 1)}cm")
                        success, new_angles = move_through_safe_path(pick_pos_cm, current_angles)
                        
                        if success:
                            current_angles = new_angles
                            state = "MOVING_TO_DROPOFF"
                            print("✅ Picking concluído!")
                        else:
                            print("❌ Falha no picking, retornando ao estado de espera")
                            state = "WAITING_FOR_CAR"
                            
                    except Exception as e:
                        print(f"❌ Erro durante picking: {str(e)}")
                        state = "WAITING_FOR_CAR"
                        
                elif 'car' in transforms and 'home' in transforms:
                    if hasattr(vision, 'last_status'):
                        print(f"\rAguardando... Status: {vision.last_status} ({vision.last_home_distance:.1f}cm)", end='')
            
            elif state == "MOVING_TO_DROPOFF":
                if 'dropoff' in transforms and 'home' in transforms:
                    try:
                        home_transform = SE3(transforms['home'])
                        dropoff_transform = SE3(transforms['dropoff'])
                        dropoff_pos_cm = get_relative_position(home_transform, dropoff_transform)
                        target_dropoff_pos_cm = dropoff_pos_cm.copy()
                        target_dropoff_pos_cm[2] += 8.0  # Add height clearance
                        
                        print(f"🎯 Posição de dropoff: {np.round(target_dropoff_pos_cm, 1)}cm")
                        success, new_angles = move_through_safe_path(target_dropoff_pos_cm, current_angles)
                        
                        if success:
                            current_angles = new_angles
                            print("✅ Dropoff concluído!")
                            # Return to home
                            print("\n🏠 Retornando à posição inicial...")
                            success_home, current_angles = move_to_position(HOME_POSITION, current_angles)
                            if success_home:
                                print("✅ Retornou à posição inicial!")
                            state = "WAITING_FOR_CAR"
                        else:
                            print("❌ Falha no dropoff, retornando ao estado de espera")
                            state = "WAITING_FOR_CAR"
                            
                    except Exception as e:
                        print(f"❌ Erro durante dropoff: {str(e)}")
                        state = "WAITING_FOR_CAR"
                else:
                    print("❌ Marcadores Home ou Dropoff não detectados.")
                    state = "WAITING_FOR_CAR"
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
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
        if simulation_env:
            plt.close('all')

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️ Interrompido pelo usuário")
    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")
    finally:
        # Ensure Arduino connection is closed if it exists
        if hasattr(send_to_real_robot, 'arduino_controller') and send_to_real_robot.arduino_controller:
            send_to_real_robot.arduino_controller.close()
        plt.close('all')
        cv2.destroyAllWindows()
