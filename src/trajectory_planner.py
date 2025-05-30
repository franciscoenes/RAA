"""Trajectory planning module"""
import numpy as np
import matplotlib.pyplot as plt
import time
from config import TRAJECTORY_DT, INITIAL_ARM_ANGLES_RAD

class TrajectoryPlanner:
    def __init__(self, dt=TRAJECTORY_DT):
        """Inicializa com o periodo de amostragem dt (em segundos)"""
        self.dt = dt
        self.arm = None  # Will be set by set_arm_controller
        self.robot_controller = None  # Will be set by set_robot_controller

    def set_arm_controller(self, arm):
        """Set the arm kinematics controller"""
        self.arm = arm

    def set_robot_controller(self, controller):
        """Set the robot controller (real or simulation)"""
        self.robot_controller = controller

    def generate_trajectory(self, q0, qf, v0, vf, T=None, a_max=None):
        """
        Gera uma trajetoria suave (interpolacao polinomial de grau 3).
        
        Argumentos:
        - q0: posicao inicial (array-like)
        - qf: posicao final (array-like)
        - v0: velocidade inicial (array-like)
        - vf: velocidade final (array-like)
        - T: duracao total (segundos) [opcional]
        - a_max: aceleracao maxima admissivel por junta [opcional]

        Retorna:
        - q: lista de posicoes ao longo do tempo (NxD)
        - dq: lista de velocidades
        - ddq: lista de aceleracoes
        - t: vetor de tempo
        """
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)

        D = len(q0)  # Numero de juntas

        if T is None:
            if a_max is None:
                raise ValueError("T ou a_max devem ser especificados")
            # Estima T com base na aceleracao maxima permitida
            delta = np.abs(qf - q0)
            T = np.max(np.sqrt(6 * delta / a_max))

        t = np.arange(0, T + self.dt, self.dt)
        N = len(t)

        q = np.zeros((N, D))
        dq = np.zeros((N, D))
        ddq = np.zeros((N, D))

        for d in range(D):
            # Coeficientes do polinomio de grau 3 para a interpolacao
            a0 = q0[d]
            a1 = v0[d]
            a2 = (3 * (qf[d] - q0[d]) - (2 * v0[d] + vf[d]) * T) / (T ** 2)
            a3 = (-2 * (qf[d] - q0[d]) + (v0[d] + vf[d]) * T) / (T ** 3)

            # Calcula posicao, velocidade e aceleracao em cada instante de tempo
            q[:, d] = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3
            dq[:, d] = a1 + 2 * a2 * t + 3 * a3 * t ** 2
            ddq[:, d] = 2 * a2 + 6 * a3 * t

        return q, dq, ddq, t

    def move_to_position(self, target_pos_cm, current_angles):
        """Move arm to position - simplified and clean"""
        try:
            if self.arm is None or self.robot_controller is None:
                raise ValueError("Arm controller or robot controller not set")

            # Convert to meters
            target_pos_m = np.array(target_pos_cm) / 100.0
            
            print(f"\n🎯 Movendo para posição: {np.round(target_pos_cm, 1)}cm")
            
            # Check reachability
            if not self.arm.is_position_reachable(target_pos_cm, current_angles):
                return False, current_angles
            
            # Get IK solutions
            solutions = self.arm.InvKin(target_pos_m)
            if not solutions:
                print("❌ Posição inatingível pelo braço.")
                return False, current_angles
            
            # Get best solution
            q_target_rad = self.arm.get_best_ik_solution(solutions, current_angles)
            if q_target_rad is None:
                print("❌ Não foi possível encontrar uma solução válida.")
                return False, current_angles
            
            print(f"🎯 Ângulos alvo (graus): {np.round(np.degrees(q_target_rad), 1)}")
            
            # Generate trajectory
            q_trajectory_rad, dq, ddq, t = self.generate_trajectory(
                current_angles, q_target_rad, v0=[0, 0, 0], vf=[0, 0, 0], T=2.0
            )
            
            # Execute trajectory cleanly
            print("🔄 Executando trajetória...")
            for i in range(len(t)):
                q_i_rad = q_trajectory_rad[i]
                self.robot_controller(q_i_rad)
                time.sleep(0.02)  # Small delay for smooth movement
            
            print("✅ Movimento concluído")
            return True, q_target_rad
            
        except Exception as e:
            print(f"❌ Erro durante movimento: {str(e)}")
            return False, current_angles

    def move_through_safe_path(self, target_pos_cm, current_angles):
        """Move arm through safe intermediate points with better error handling"""
        try:
            if self.arm is None:
                raise ValueError("Arm controller not set")

            # Check if target position is reachable
            if not self.arm.is_position_reachable(target_pos_cm, current_angles):
                return False, current_angles
            
            # Get current position
            current_pos_m = self.arm.FwdKin(current_angles)
            current_pos_cm = current_pos_m * 100.0
            
            print("\n=== Planejamento de Movimento ===")
            print(f"Posição atual (cm): {np.round(current_pos_cm, 1)}")
            print(f"Posição alvo (cm): {np.round(target_pos_cm, 1)}")
            
            # Try direct movement first if target is reachable
            print("\n🔄 Tentando movimento direto...")
            success, new_angles = self.move_to_position(target_pos_cm, current_angles)
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
            success, current_angles = self.move_to_position(intermediate_pos, current_angles)
            if not success:
                print("❌ Falha ao mover para posição intermediária")
                return False, current_angles
            
            # Move to target
            print("\n🔄 Movendo para posição alvo...")
            success, current_angles = self.move_to_position(target_pos_cm, current_angles)
            if not success:
                print("❌ Falha ao mover para posição alvo")
                return False, current_angles
            
            return True, current_angles
            
        except Exception as e:
            print(f"❌ Erro durante movimento seguro: {str(e)}")
            return False, current_angles


if __name__ == "__main__":
    planner = TrajectoryPlanner()

    # Exemplo para 3 juntas
    q0 = INITIAL_ARM_ANGLES_RAD
    qf = [np.pi/4, np.pi/6, -np.pi/6]
    v0 = [0.0, 0.0, 0.0]
    vf = [0.0, 0.0, 0.0]
    T = 2.0  # duracao da trajetoria

    q, dq, ddq, t = planner.generate_trajectory(q0, qf, v0, vf, T=T)

    # Visualizar as trajetorias
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    joint_labels = ['Junta 1', 'Junta 2', 'Junta 3']

    for i in range(3):
        axs[0].plot(t, q[:, i], label=joint_labels[i])
        axs[1].plot(t, dq[:, i], label=joint_labels[i])
        axs[2].plot(t, ddq[:, i], label=joint_labels[i])

    axs[0].set_ylabel("Posicao (rad)")
    axs[1].set_ylabel("Velocidade (rad/s)")
    axs[2].set_ylabel("Aceleracao (rad/s2)")
    axs[2].set_xlabel("Tempo (s)")

    for ax in axs:
        ax.grid(True)
        ax.legend()

    plt.suptitle("Trajetoria de Juntas - Planeamento Polinomial de grau 3")
    plt.tight_layout()
    plt.show()
