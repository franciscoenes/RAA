import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH
import time
from config import LINK_LENGTHS_METERS, INITIAL_ARM_ANGLES_RAD, SIMULATION_PLOT_LIMITS


class DigitalTwinSimulator:
    def __init__(self):
        # Definição do manipulador RRR utilizando parâmetros de Denavit-Hartenberg
        l1, l2, l3 = LINK_LENGTHS_METERS
        self.robot = DHRobot(
            [
                RevoluteDH(a=0, d=l1, alpha=np.pi / 2),  # Junta da base - Rotação em Z
                RevoluteDH(a=l2, d=0, alpha=0),  # Junta do ombro - Rotação em Y
                RevoluteDH(a=l3, d=0, alpha=0),  # Junta do cotovelo - Rotação em Y
            ],
            name="Braço RRR",
        )

        # Inicialização das variáveis de estado do sistema
        self.q_current = INITIAL_ARM_ANGLES_RAD.copy()  # Configuração atual das juntas
        self.car_pose = None  # Pose do carro
        self.home_pose = None  # Pose da posição inicial
        self.dropoff_pose = None  # Pose da posição de descarga
        self.simulation_env = None  # Ambiente de simulação
        self.car_marker = None  # Marcador do carro
        self.home_marker = None  # Marcador da posição inicial
        self.dropoff_marker = None  # Marcador da posição de descarga

    def initialize_simulation(self):
        """
        Inicializa o ambiente de simulação 3D interativo.
        Configura a visualização do robô e dos marcadores no espaço de trabalho.
        """
        plt.ion()  # Ativa o modo interativo do matplotlib

        # Criação do ambiente de simulação utilizando o Robotics Toolbox
        self.simulation_env = self.robot.plot(
            self.q_current,
            backend="pyplot",
            block=False,
            jointaxes=True,  # Mostra os eixos das juntas
            eeframe=True,  # Mostra o referencial do efetuador final
            shadow=False,  # Desativa a projeção de sombra
        )

        # Configuração dos parâmetros de visualização
        ax = self.simulation_env.ax
        ax.set_xlim(SIMULATION_PLOT_LIMITS["x"])
        ax.set_ylim(SIMULATION_PLOT_LIMITS["y"])
        ax.set_zlim(SIMULATION_PLOT_LIMITS["z"])
        ax.set_title("Simulação do Braço Robótico")

        # Inicialização dos marcadores visuais no espaço de trabalho
        (self.car_marker,) = ax.plot([], [], [], "ro", label="Car", markersize=7)
        (self.home_marker,) = ax.plot([], [], [], "g^", label="Home", markersize=7)
        (self.dropoff_marker,) = ax.plot(
            [], [], [], "m^", label="Dropoff", markersize=7
        )
        ax.legend()

        plt.show(block=False)
        return self.simulation_env

    def update_simulation_display(self, joint_angles_rad):
        """
        Atualiza a visualização da simulação com os novos ângulos das juntas.
        Parâmetros:
            joint_angles_rad: Lista com os ângulos das juntas em radianos
        """
        if self.simulation_env is not None:
            try:
                # Atualiza a configuração das juntas do robô
                self.simulation_env.robots[0].robot.q = joint_angles_rad

                # Atualiza a visualização
                self.simulation_env.step(dt=0.01)

                # Atualiza a posição dos marcadores
                self.update_simulation_markers()

                # Pequena pausa para suavizar a animação
                plt.pause(0.001)
            except Exception as e:
                print(f"Aviso: Erro na atualização da simulação: {e}")

    def update_simulation_markers(self):
        """
        Atualiza a posição dos marcadores visuais na simulação.
        Inclui marcadores para o carro, posição inicial e posição de descarga.
        """
        if (
            self.car_marker is None
            or self.home_marker is None
            or self.dropoff_marker is None
        ):
            return

        # Atualização do marcador do carro
        if self.car_pose is not None and self.home_pose is not None:
            car_pos = self.car_pose.t
            self.car_marker.set_data([car_pos[0]], [car_pos[1]])
            self.car_marker.set_3d_properties([car_pos[2]])
        else:
            self.car_marker.set_data([], [])
            self.car_marker.set_3d_properties([])

        # Atualização do marcador da posição inicial
        if self.home_pose is not None:
            home_pos = self.home_pose.t
            self.home_marker.set_data([home_pos[0]], [home_pos[1]])
            self.home_marker.set_3d_properties([home_pos[2]])
        else:
            self.home_marker.set_data([], [])
            self.home_marker.set_3d_properties([])

        # Atualização do marcador da posição de descarga
        if self.dropoff_pose is not None:
            dropoff_pos = self.dropoff_pose.t
            self.dropoff_marker.set_data([dropoff_pos[0]], [dropoff_pos[1]])
            self.dropoff_marker.set_3d_properties([dropoff_pos[2]])
        else:
            self.dropoff_marker.set_data([], [])
            self.dropoff_marker.set_3d_properties([])

        # Força a atualização da visualização
        self.simulation_env.ax.figure.canvas.draw()
        self.simulation_env.ax.figure.canvas.flush_events()

    def update_from_vision(self, transforms):
        """
        Atualiza as poses dos objetos com base nas transformações obtidas do sistema de visão.
        Parâmetros:
            transforms: Dicionário com as matrizes de transformação homogénea dos marcadores
        """
        if "car" in transforms:
            car_pos = transforms["car"][:3, 3]
            self.car_pose = SE3(car_pos[0], car_pos[1], car_pos[2])

        if "home" in transforms:
            home_pos = transforms["home"][:3, 3]
            self.home_pose = SE3(home_pos[0], home_pos[1], home_pos[2])

        if "dropoff" in transforms:
            dropoff_pos = transforms["dropoff"][:3, 3]
            self.dropoff_pose = SE3(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2])

        # Atualiza imediatamente os marcadores após receber novas transformações
        if self.simulation_env is not None:
            self.update_simulation_markers()

    def update_arm(self, q):
        """
        Atualiza a configuração do braço robótico.
        Parâmetros:
            q: Lista com os novos ângulos das juntas em radianos
        """
        self.q_current = q
        print("🔧 Configuração do braço atualizada para:", np.round(q, 3))
        self.update_simulation_display(q)

    def show(self):
        """
        Apresenta a simulação do sistema completo em modo interativo.
        """
        if self.simulation_env is None:
            self.initialize_simulation()
        plt.show(block=True)
        time.sleep(0.1)


if __name__ == "__main__":
    # Código de teste para demonstração do gémeo digital
    sim = DigitalTwinSimulator()

    # Inicialização e configuração inicial do sistema
    sim.initialize_simulation()
    sim.update_arm(INITIAL_ARM_ANGLES_RAD)

    # Definição de poses de teste para os marcadores (em metros)
    # Posição inicial no centro do referencial
    home_transform = np.array(
        [[1, 0, 0, 0.0], [0, 1, 0, 0.0], [0, 0, 1, 0.0], [0, 0, 0, 1]]
    )

    # Carro a 22cm da posição inicial no eixo X
    car_transform = np.array(
        [[1, 0, 0, 0.22], [0, 1, 0, 0.0], [0, 0, 1, 0.0], [0, 0, 0, 1]]
    )

    # Posição de descarga a 20cm em X e -15cm em Y
    dropoff_transform = np.array(
        [[1, 0, 0, 0.20], [0, 1, 0, -0.15], [0, 0, 1, 0.0], [0, 0, 0, 1]]
    )

    # Atualização das poses no simulador
    sim.update_from_vision(
        {"home": home_transform, "car": car_transform, "dropoff": dropoff_transform}
    )

    # Apresentação da simulação
    sim.show()
