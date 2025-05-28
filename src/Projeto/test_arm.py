import numpy as np
import time
import cv2
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
from arduino_control import ArduinoArmControllerSerial
from vision_system import VisionSystem
import matplotlib.pyplot as plt
from spatialmath import SE3

# === Configurações ===
SIMULATION_MODE = False  # Modo de simulação (True) ou operação real (False)
ARDUINO_PORT = 'COM9'    # Ajuste para a porta correta do seu Arduino
ARDUINO_BAUD_RATE = 57600 # Match the baud rate in the Arduino sketch
CAMERA_INDEX = 0         # Índice da câmera a ser usada

# Initial angles from the Arduino sketch (in degrees)
INITIAL_ARM_ANGLES_DEG = {
    'base': 80,
    'shoulder': 60,
    'elbow': 130,
    'gripper': 0 # Gripper angle from sketch initial position
}

# Initial angles for kinematics and trajectory planning (in radians)
# We only need base, shoulder, elbow for RRR kinematics
INITIAL_ARM_ANGLES_RAD_KIN = [
    np.radians(INITIAL_ARM_ANGLES_DEG['base']),
    np.radians(INITIAL_ARM_ANGLES_DEG['shoulder']),
    np.radians(INITIAL_ARM_ANGLES_DEG['elbow'])
]

# Inicializações
arm = RobotArmRRR(link_lengths=[0.06, 0.10, 0.10])
planner = TrajectoryPlanner(dt=0.05)
simulator = DigitalTwinSimulator()
vision = VisionSystem("camera_params.npz", marker_length=0.097)

# Inicializar controlador Arduino se não estiver em modo simulação
arduino_controller = None
if not SIMULATION_MODE:
    try:
        arduino_controller = ArduinoArmControllerSerial(port=ARDUINO_PORT, baud_rate=ARDUINO_BAUD_RATE)
        # If serial connection failed, arduino_controller.serial_connection will be None
        if arduino_controller.serial_connection is None or not arduino_controller.serial_connection.isOpen():
             print("Serial connection not established. Mudando para modo simulação...")
             SIMULATION_MODE = True
        else:
            # Send the initial angles from Python to synchronize the real arm state
            print("Synchronizing real arm to initial pose...")
            arduino_controller.set_joint_angles_deg(INITIAL_ARM_ANGLES_DEG)
            time.sleep(3) # Give Arduino time to move to the initial pose

    except Exception as e:
        print(f"❌ Erro ao inicializar controlador Arduino: {e}")
        print("Mudando para modo simulação...")
        SIMULATION_MODE = True

def send_to_real_robot(joint_angles_rad):
    """Envia ângulos para o braço real via serial"""
    if arduino_controller and not SIMULATION_MODE:
        # Convert base, shoulder, elbow angles from radians to degrees
        base_angle_deg = np.degrees(joint_angles_rad[0])
        shoulder_angle_deg = np.degrees(joint_angles_rad[1])
        elbow_angle_deg = np.degrees(joint_angles_rad[2])

        # Get the current gripper angle (it's not part of the RRR trajectory)
        # We assume the gripper stays at its initial angle unless explicitly controlled
        gripper_angle_deg = arduino_controller.current_angles_deg.get('gripper', INITIAL_ARM_ANGLES_DEG['gripper'])

        angles_deg_to_send = {
            'base': base_angle_deg,
            'shoulder': shoulder_angle_deg,
            'elbow': elbow_angle_deg,
            'gripper': gripper_angle_deg
        }

        # Send angles in degrees over serial
        arduino_controller.set_joint_angles_deg(angles_deg_to_send)
    elif SIMULATION_MODE:
        # Simulation uses angles in radians
        simulator.update_arm(joint_angles_rad)

def move_to_position(target_pos, current_angles):
    """Move o braço para uma posição específica"""
    solutions = arm.InvKin(target_pos)
    if not solutions:
        print("❌ Posição inatingível pelo braço.")
        return False

    q_target_rad = solutions[0]
    q_trajectory_rad, dq, ddq, t = planner.generate_trajectory(
        current_angles, q_target_rad, [0,0,0], [0,0,0], T=4.0
    )

    for i in range(len(t)):
        q_i_rad = q_trajectory_rad[i]
        send_to_real_robot(q_i_rad)
        # In simulation mode, update the simulator's arm and simulation plot
        if SIMULATION_MODE:
            simulator.update_arm(q_i_rad)
            # Update robot pose in the existing plot environment
            env.step(0)
            # We need to update the marker positions as well during trajectory
            update_simulation_markers(ax, simulator, car_marker, object_marker, home_marker, dropoff_marker)
            fig.canvas.draw()
            fig.canvas.flush_events()
            
        time.sleep(planner.dt)

    return True

def update_simulation_markers(ax, simulator, car_marker, object_marker, home_marker, dropoff_marker):
    """Updates the positions of the marker plots in the simulation."""
    if simulator.car_pose is not None and simulator.home_pose is not None:
        # Calculate car position relative to home
        car_pos_relative = simulator.home_pose.inv() * simulator.car_pose
        car_marker.set_data([car_pos_relative.t[0]], [car_pos_relative.t[1]])
        car_marker.set_3d_properties([car_pos_relative.t[2]])
    else:
         car_marker.set_data([], [])
         car_marker.set_3d_properties([])

    # Object position is relative to the arm's base, which we align with home in simulation
    if simulator.has_object and simulator.object_pose is not None and simulator.home_pose is not None:
         # Calculate object position relative to home
        object_pos_relative = simulator.home_pose.inv() * simulator.object_pose
        object_marker.set_data([object_pos_relative.t[0]], [object_pos_relative.t[1]])
        object_marker.set_3d_properties([object_pos_relative.t[2]])
    else:
        object_marker.set_data([], [])
        object_marker.set_3d_properties([])

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

def main():
    global env, ax, fig, car_marker, object_marker, home_marker, dropoff_marker # Declare global variables for plot elements

    print("==============================")
    print(" SISTEMA DE PICK AND PLACE")
    print("==============================")
    print("Pressione 'q' para sair ou 'r' para reiniciar")
    print(f"Modo: {'Simulação' if SIMULATION_MODE else 'Real'}")

    # Configurar janelas
    cv2.namedWindow("Vision System", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Vision System", 800, 600)
    
    # Configurar figura do matplotlib para não bloquear
    plt.ion()
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # Inicializar plot do robô (cria a janela de simulação)
    env = None
    car_marker = None
    object_marker = None
    home_marker = None
    dropoff_marker = None

    if SIMULATION_MODE:
        env = simulator.robot.plot(INITIAL_ARM_ANGLES_RAD_KIN, backend="pyplot", block=False)
        ax = env.ax
        fig = ax.figure # Get the figure from the axes
        # Initialize marker plots with empty data
        car_marker, = ax.plot([], [], [], 'ro', label="Carro")
        object_marker, = ax.plot([], [], [], 'bo', label="Objeto")
        home_marker, = ax.plot([], [], [], 'g^', label="Home")
        dropoff_marker, = ax.plot([], [], [], 'm^', label="Drop-off")
        ax.legend()

        ax.set_xlim(-0.2, 0.5)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0, 0.4)
        ax.set_title("Simulação do Braço Robótico")
        plt.show(block=False)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    current_angles = INITIAL_ARM_ANGLES_RAD_KIN
    state = "WAITING_FOR_CAR"  # Estados: WAITING_FOR_CAR, PICKING, MOVING_TO_DROPOFF, DROPPING

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Processar frame da câmera
        transforms = vision.get_transforms(frame)
        simulator.update_from_vision(transforms)

        # Mostrar status na imagem
        cv2.putText(frame, f"Estado: {state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Modo: {'Simulação' if SIMULATION_MODE else 'Real'}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Mostrar marcadores detectados
        y_pos = 90
        for marker_name in ['home', 'car', 'dropoff']:
            status = "Detectado" if marker_name in transforms else "Não detectado"
            color = (0, 255, 0) if marker_name in transforms else (0, 0, 255)
            cv2.putText(frame, f"{marker_name}: {status}", (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            y_pos += 30

        cv2.imshow("Vision System", frame)

        # Atualizar simulação (apenas se em modo simulação)
        if SIMULATION_MODE:
            # Update marker positions
            update_simulation_markers(ax, simulator, car_marker, object_marker, home_marker, dropoff_marker)

            # Update the plot display
            fig.canvas.draw()
            fig.canvas.flush_events()

        # Máquina de estados
        if state == "WAITING_FOR_CAR":
            if vision.is_car_at_home(transforms):
                print("Carro detectado na posição inicial!")
                state = "PICKING"
                # Posição para pegar o objeto (10cm above the car relative to the home marker)
                # We need to get the latest car position relative to home from the vision system
                if 'car' in transforms and 'home' in transforms:
                    home_transform = transforms['home']
                    car_transform = transforms['car']
                    car_pos_relative_to_home = home_transform.inv() * car_transform
                    pick_pos_relative_to_home = car_pos_relative_to_home.t + np.array([0, 0, 0.1])

                    # move_to_position expects a target position in the robot's base frame (which is now aligned with home)
                    if move_to_position(pick_pos_relative_to_home, current_angles):
                        # current_angles is updated within move_to_position now for simulation mode
                        current_angles = arm.InvKin(pick_pos_relative_to_home)[0] # Still update current_angles here for the next move
                        state = "MOVING_TO_DROPOFF"
                else:
                    print("Home or Car marker not detected for picking.")
                    state = "WAITING_FOR_CAR" # Stay in waiting state if markers not detected

        elif state == "MOVING_TO_DROPOFF":
            # We need to get the latest dropoff position relative to home from the vision system
            if 'dropoff' in transforms and 'home' in transforms:
                home_transform = transforms['home']
                dropoff_transform = transforms['dropoff']
                dropoff_pos_relative_to_home = home_transform.inv() * dropoff_transform
                # Move to a position 10cm above the dropoff point relative to home
                target_dropoff_pos = dropoff_pos_relative_to_home.t + np.array([0, 0, 0.1])

                if move_to_position(target_dropoff_pos, current_angles):
                     # current_angles is updated within move_to_position now for simulation mode
                    current_angles = arm.InvKin(target_dropoff_pos)[0] # Still update current_angles here for the next move
                    state = "DROPPING"
                    time.sleep(1)  # Tempo para soltar o objeto
                    state = "WAITING_FOR_CAR"  # Volta ao estado inicial
            else:
                print("Home or Dropoff marker not detected.")
                state = "WAITING_FOR_CAR" # Return to waiting state if markers not detected

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            state = "WAITING_FOR_CAR"
            current_angles = INITIAL_ARM_ANGLES_RAD_KIN
            send_to_real_robot(current_angles)
            # In simulation mode, reset the simulator's arm directly
            if SIMULATION_MODE:
                simulator.update_arm(current_angles)
                # Update the robot pose in the plot environment after resetting
                env.step(0)
                # Update marker positions after reset
                update_simulation_markers(ax, simulator, car_marker, object_marker, home_marker, dropoff_marker)
                fig.canvas.draw()
                fig.canvas.flush_events()

    cap.release()
    cv2.destroyAllWindows()
    # Close the matplotlib window if it was opened
    if SIMULATION_MODE and plt.get_fignums():
         plt.close(fig)

if __name__ == "__main__":
    # Declare global variables for plot elements here as well for the finally block
    env = None
    ax = None
    car_marker = None
    object_marker = None
    home_marker = None
    dropoff_marker = None
    fig = None # Declare fig as global too
    try:
        main()
    finally:
        # Close the serial connection if it was opened
        if arduino_controller is not None and hasattr(arduino_controller, 'serial_connection'):
            arduino_controller.close()
        # Ensure all plot windows are closed on exit
        plt.close('all') # Use plt.close('all') to close any remaining figures
