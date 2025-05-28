import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH
import time

class DigitalTwinSimulator:
    def __init__(self):
        # Define o braço RRR com parâmetros DH simples (pode ser ajustado)
        self.robot = DHRobot([
            RevoluteDH(a=0,     d=0.1, alpha=np.pi/2),  # junta base
            RevoluteDH(a=0.15,  d=0,   alpha=0),        # junta ombro
            RevoluteDH(a=0.15,  d=0,   alpha=0)         # junta cotovelo
        ], name='Braço RRR')

        self.q_current = [0, 0, 0]
        self.car_pose = None
        self.home_pose = None
        self.dropoff_pose = None
        self.object_pose = None
        self.has_object = False

    def update_from_vision(self, transforms):
        """Atualiza as poses baseado nos marcadores detectados"""
        if 'car' in transforms:
            car_pos = transforms['car'][:3, 3]
            self.car_pose = SE3(car_pos[0], car_pos[1], car_pos[2])
            
            # Se o carro está na posição inicial, o objeto está acima dele
            if self.home_pose is not None:
                home_pos = self.home_pose.t
                if np.linalg.norm(car_pos - home_pos) < 0.1:  # threshold
                    self.object_pose = SE3(car_pos[0], car_pos[1], car_pos[2] + 0.1)  # 10cm acima do carro
                    self.has_object = True

        if 'home' in transforms:
            home_pos = transforms['home'][:3, 3]
            self.home_pose = SE3(home_pos[0], home_pos[1], home_pos[2])

        if 'dropoff' in transforms:
            dropoff_pos = transforms['dropoff'][:3, 3]
            self.dropoff_pose = SE3(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2])

    def update_arm(self, q):
        """Atualiza a configuração do braço"""
        self.q_current = q
        print("🔧 Braço atualizado para:", np.round(q, 3))

    def show(self):
        """Mostra a simulação do sistema completo"""
        fig = plt.figure()
        self.robot.plot(self.q_current, backend="pyplot", block=False, fig=fig)
        ax = plt.gca()

        # Desenhar o carro
        if self.car_pose is not None:
            car_pos = self.car_pose.t
            ax.plot([car_pos[0]], [car_pos[1]], 'ro', label="Carro")
            ax.text(car_pos[0], car_pos[1], car_pos[2] + 0.02, 'Carro', color='red')

        # Desenhar o objeto se estiver presente
        if self.has_object and self.object_pose is not None:
            obj_pos = self.object_pose.t
            ax.plot([obj_pos[0]], [obj_pos[1]], 'bo', label="Objeto")
            ax.text(obj_pos[0], obj_pos[1], obj_pos[2] + 0.02, 'Objeto', color='blue')

        # Desenhar pontos de referência
        if self.home_pose is not None:
            home_pos = self.home_pose.t
            ax.plot([home_pos[0]], [home_pos[1]], 'g^', label="Home")
            ax.text(home_pos[0], home_pos[1], home_pos[2] + 0.02, 'Home', color='green')

        if self.dropoff_pose is not None:
            dropoff_pos = self.dropoff_pose.t
            ax.plot([dropoff_pos[0]], [dropoff_pos[1]], 'm^', label="Drop-off")
            ax.text(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2] + 0.02, 'Drop-off', color='magenta')

        ax.set_xlim(-0.2, 0.5)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0, 0.4)
        plt.legend()
        plt.title("Gémeo Digital: Braço Robótico + Sistema de Visão")
        plt.show(block=True)
        time.sleep(0.1)


if __name__ == "__main__":
    sim = DigitalTwinSimulator()

    # Atualizar pose do braço e do carro
    sim.update_arm([0, np.pi/6, -np.pi/6])
    sim.update_from_vision({'car': np.array([[1, 0, 0, 0.3], [0, 1, 0, 0.1], [0, 0, 1, 0], [0, 0, 0, 1]])})

    # Mostrar simulação
    sim.show()
