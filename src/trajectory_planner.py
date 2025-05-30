"""Módulo de planeamento de trajetórias para o braço robótico"""

import numpy as np
import matplotlib.pyplot as plt
import time
from config import TRAJECTORY_DT, INITIAL_ARM_ANGLES_RAD


class TrajectoryPlanner:
    def __init__(self, dt=TRAJECTORY_DT):
        """
        Inicializa o planeador de trajetórias.

        Parâmetros:
            dt: Período de amostragem em segundos
        """
        self.dt = dt
        self.arm = (
            None  # Controlador cinemático do braço (definido por set_arm_controller)
        )
        self.robot_controller = (
            None  # Controlador do robô (definido por set_robot_controller)
        )

    def set_arm_controller(self, arm):
        """Define o controlador cinemático do braço"""
        self.arm = arm

    def set_robot_controller(self, controller):
        """Define o controlador do robô (real ou simulado)"""
        self.robot_controller = controller

    def generate_trajectory(self, q0, qf, v0, vf, T=None, a_max=None):
        """
        Gera uma trajetória suave utilizando interpolação polinomial de terceiro grau.

        Parâmetros:
            q0: Posição inicial das juntas (array)
            qf: Posição final das juntas (array)
            v0: Velocidade inicial das juntas (array)
            vf: Velocidade final das juntas (array)
            T: Duração total do movimento em segundos [opcional]
            a_max: Aceleração máxima permitida por junta [opcional]

        Retorna:
            q: Matriz de posições ao longo do tempo (NxD)
            dq: Matriz de velocidades ao longo do tempo (NxD)
            ddq: Matriz de acelerações ao longo do tempo (NxD)
            t: Vetor de instantes temporais
        """
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)

        D = len(q0)  # Número de juntas do robô

        if T is None:
            if a_max is None:
                raise ValueError("É necessário especificar T ou a_max")
            # Estimativa do tempo total com base na aceleração máxima
            delta = np.abs(qf - q0)
            T = np.max(np.sqrt(6 * delta / a_max))

        # Geração do vetor temporal
        t = np.arange(0, T + self.dt, self.dt)
        N = len(t)

        # Inicialização das matrizes de trajetória
        q = np.zeros((N, D))  # Posições
        dq = np.zeros((N, D))  # Velocidades
        ddq = np.zeros((N, D))  # Acelerações

        for d in range(D):
            # Cálculo dos coeficientes do polinómio de terceiro grau
            a0 = q0[d]  # Termo independente
            a1 = v0[d]  # Termo linear
            a2 = (3 * (qf[d] - q0[d]) - (2 * v0[d] + vf[d]) * T) / (
                T**2
            )  # Termo quadrático
            a3 = (-2 * (qf[d] - q0[d]) + (v0[d] + vf[d]) * T) / (T**3)  # Termo cúbico

            # Cálculo das posições, velocidades e acelerações
            q[:, d] = a0 + a1 * t + a2 * t**2 + a3 * t**3
            dq[:, d] = a1 + 2 * a2 * t + 3 * a3 * t**2
            ddq[:, d] = 2 * a2 + 6 * a3 * t

        return q, dq, ddq, t

    def move_to_position(self, target_pos_cm, current_angles):
        """
        Move o braço para uma posição específica.

        Parâmetros:
            target_pos_cm: Posição alvo em centímetros [x, y, z]
            current_angles: Ângulos atuais das juntas em radianos

        Retorna:
            (sucesso, novos_ângulos): Tuplo indicando o sucesso da operação e os ângulos finais
        """
        try:
            if self.arm is None or self.robot_controller is None:
                raise ValueError("Controladores do braço não definidos")

            # Conversão para metros
            target_pos_m = np.array(target_pos_cm) / 100.0

            print(f"\n🎯 A mover para a posição: {np.round(target_pos_cm, 1)}cm")

            # Verificação de alcançabilidade
            if not self.arm.is_position_reachable(target_pos_cm, current_angles):
                return False, current_angles

            # Cálculo da cinemática inversa
            solutions = self.arm.InvKin(target_pos_m)
            if not solutions:
                print("❌ Posição fora do espaço de trabalho do braço.")
                return False, current_angles

            # Seleção da melhor solução
            q_target_rad = self.arm.get_best_ik_solution(solutions, current_angles)
            if q_target_rad is None:
                print("❌ Não foi encontrada uma solução válida.")
                return False, current_angles

            print(
                f"🎯 Ângulos pretendidos (graus): {np.round(np.degrees(q_target_rad), 1)}"
            )

            # Geração da trajetória
            q_trajectory_rad, dq, ddq, t = self.generate_trajectory(
                current_angles, q_target_rad, v0=[0, 0, 0], vf=[0, 0, 0], T=2.0
            )

            # Execução da trajetória
            print("🔄 A executar trajetória...")
            for i in range(len(t)):
                q_i_rad = q_trajectory_rad[i]
                self.robot_controller(q_i_rad)
                time.sleep(0.02)  # Pequena pausa para movimento suave

            print("✅ Movimento concluído com sucesso")
            return True, q_target_rad

        except Exception as e:
            print(f"❌ Erro durante a execução do movimento: {str(e)}")
            return False, current_angles

    def move_through_safe_path(self, target_pos_cm, current_angles):
        """
        Move o braço através de um caminho seguro, utilizando pontos intermédios quando necessário.

        Parâmetros:
            target_pos_cm: Posição alvo em centímetros [x, y, z]
            current_angles: Ângulos atuais das juntas em radianos

        Retorna:
            (sucesso, novos_ângulos): Tuplo indicando o sucesso da operação e os ângulos finais
        """
        try:
            if self.arm is None:
                raise ValueError("Controlador do braço não definido")

            # Verificação de alcançabilidade
            if not self.arm.is_position_reachable(target_pos_cm, current_angles):
                return False, current_angles

            # Obtenção da posição atual
            current_pos_m = self.arm.FwdKin(current_angles)
            current_pos_cm = current_pos_m * 100.0

            print("\n=== Planeamento de Movimento ===")
            print(f"Posição atual (cm): {np.round(current_pos_cm, 1)}")
            print(f"Posição pretendida (cm): {np.round(target_pos_cm, 1)}")

            # Tentativa de movimento direto
            print("\n🔄 A tentar movimento direto...")
            success, new_angles = self.move_to_position(target_pos_cm, current_angles)
            if success:
                print("✅ Movimento direto concluído com sucesso!")
                return True, new_angles

            # Se o movimento direto falhar, tenta com pontos intermédios
            print("\n🔄 Movimento direto falhou. A tentar caminho alternativo...")

            # Cálculo de uma posição intermédia segura
            current_xy_dist = np.sqrt(current_pos_cm[0] ** 2 + current_pos_cm[1] ** 2)
            target_xy_dist = np.sqrt(target_pos_cm[0] ** 2 + target_pos_cm[1] ** 2)

            # Seleção do ponto intermédio entre posição atual e alvo
            intermediate_distance = (current_xy_dist + target_xy_dist) / 2

            # Cálculo da posição XY intermédia
            target_direction = np.array([target_pos_cm[0], target_pos_cm[1]])
            target_direction_norm = target_direction / np.linalg.norm(target_direction)

            intermediate_xy = target_direction_norm * intermediate_distance
            intermediate_height = (
                max(current_pos_cm[2], target_pos_cm[2]) + 2.0
            )  # Adiciona 2cm de margem

            intermediate_pos = np.array(
                [intermediate_xy[0], intermediate_xy[1], intermediate_height]
            )

            print(f"Posição intermédia calculada: {np.round(intermediate_pos, 1)}")

            # Movimento para a posição intermédia
            print("\n🔄 A mover para posição intermédia...")
            success, current_angles = self.move_to_position(
                intermediate_pos, current_angles
            )
            if not success:
                print("❌ Falha no movimento para a posição intermédia")
                return False, current_angles

            # Movimento para a posição final
            print("\n🔄 A mover para posição final...")
            success, current_angles = self.move_to_position(
                target_pos_cm, current_angles
            )
            if not success:
                print("❌ Falha no movimento para a posição final")
                return False, current_angles

            return True, current_angles

        except Exception as e:
            print(f"❌ Erro durante a execução do movimento seguro: {str(e)}")
            return False, current_angles


if __name__ == "__main__":
    # Código de teste do planeador de trajetórias
    planner = TrajectoryPlanner()

    # Exemplo de trajetória para um braço com 3 juntas
    q0 = INITIAL_ARM_ANGLES_RAD  # Posição inicial
    qf = [np.pi / 4, np.pi / 6, -np.pi / 6]  # Posição final
    v0 = [0.0, 0.0, 0.0]  # Velocidade inicial nula
    vf = [0.0, 0.0, 0.0]  # Velocidade final nula
    T = 2.0  # Duração da trajetória em segundos

    # Geração da trajetória
    q, dq, ddq, t = planner.generate_trajectory(q0, qf, v0, vf, T=T)

    # Visualização das trajetórias
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    joint_labels = ["Junta 1", "Junta 2", "Junta 3"]

    # Desenho dos gráficos
    for i in range(3):
        axs[0].plot(t, q[:, i], label=joint_labels[i])
        axs[1].plot(t, dq[:, i], label=joint_labels[i])
        axs[2].plot(t, ddq[:, i], label=joint_labels[i])

    # Configuração dos eixos
    axs[0].set_ylabel("Posição (rad)")
    axs[1].set_ylabel("Velocidade (rad/s)")
    axs[2].set_ylabel("Aceleração (rad/s²)")
    axs[2].set_xlabel("Tempo (s)")

    # Configuração da apresentação
    for ax in axs:
        ax.grid(True)
        ax.legend()

    plt.suptitle("Trajetórias das Juntas - Interpolação Polinomial de 3º Grau")
    plt.tight_layout()
    plt.show()
