import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH
import time
from config import (LINK_LENGTHS_METERS, INITIAL_ARM_ANGLES_RAD, 
                   SIMULATION_PLOT_LIMITS)

class DigitalTwinSimulator:
    def __init__(self):
        # Define o braço RRR com parâmetros DH usando configurações globais
        l1, l2, l3 = LINK_LENGTHS_METERS
        self.robot = DHRobot([
            RevoluteDH(a=0,  d=l1,  alpha=np.pi/2),  # junta base
            RevoluteDH(a=l2, d=0,   alpha=0),        # junta ombro
            RevoluteDH(a=l3, d=0,   alpha=0)         # junta cotovelo
        ], name='Braço RRR')

        self.q_current = INITIAL_ARM_ANGLES_RAD.copy()
        self.car_pose = None
        self.home_pose = None
        self.dropoff_pose = None
        self.simulation_env = None
        self.car_marker = None
        self.home_marker = None
        self.dropoff_marker = None

    def initialize_simulation(self):
        """Initialize the simulation environment cleanly"""
        plt.ion()  # Enable interactive mode
        
        # Use Robotics Toolbox to create the environment
        self.simulation_env = self.robot.plot(
            self.q_current,
            backend='pyplot',
            block=False,
            jointaxes=True,
            eeframe=True,
            shadow=False
        )
        
        # Configure the plot
        ax = self.simulation_env.ax
        ax.set_xlim(SIMULATION_PLOT_LIMITS['x'])
        ax.set_ylim(SIMULATION_PLOT_LIMITS['y'])
        ax.set_zlim(SIMULATION_PLOT_LIMITS['z'])
        ax.set_title("Simulação do Braço Robótico")
        
        # Initialize marker plots with empty data
        self.car_marker, = ax.plot([], [], [], 'ro', label="Carro", markersize=7)
        self.home_marker, = ax.plot([], [], [], 'g^', label="Home", markersize=7)
        self.dropoff_marker, = ax.plot([], [], [], 'm^', label="Drop-off", markersize=7)
        ax.legend()
        
        plt.show(block=False)
        return self.simulation_env

    def update_simulation_display(self, joint_angles_rad):
        """Update simulation display cleanly without blocking"""
        if self.simulation_env is not None:
            try:
                # First update the robot's joint configuration
                self.simulation_env.robots[0].robot.q = joint_angles_rad
                
                # Then call step to update the display
                self.simulation_env.step(dt=0.01)
                
                # Always update markers
                self.update_simulation_markers()
                
                # Brief pause for smooth animation
                plt.pause(0.001)
            except Exception as e:
                print(f"Warning: Simulation update error: {e}")

    def update_simulation_markers(self):
        """Updates the positions of the marker plots in the simulation."""
        if self.car_marker is None or self.home_marker is None or self.dropoff_marker is None:
            return

        # Update car marker
        if self.car_pose is not None and self.home_pose is not None:
            car_pos = self.car_pose.t
            self.car_marker.set_data([car_pos[0]], [car_pos[1]])
            self.car_marker.set_3d_properties([car_pos[2]])
        else:
            self.car_marker.set_data([], [])
            self.car_marker.set_3d_properties([])

        # Update home marker
        if self.home_pose is not None:
            home_pos = self.home_pose.t
            self.home_marker.set_data([home_pos[0]], [home_pos[1]])
            self.home_marker.set_3d_properties([home_pos[2]])
        else:
            self.home_marker.set_data([], [])
            self.home_marker.set_3d_properties([])

        # Update dropoff marker
        if self.dropoff_pose is not None:
            dropoff_pos = self.dropoff_pose.t
            self.dropoff_marker.set_data([dropoff_pos[0]], [dropoff_pos[1]])
            self.dropoff_marker.set_3d_properties([dropoff_pos[2]])
        else:
            self.dropoff_marker.set_data([], [])
            self.dropoff_marker.set_3d_properties([])

        # Force update the display
        self.simulation_env.ax.figure.canvas.draw()
        self.simulation_env.ax.figure.canvas.flush_events()

    def update_from_vision(self, transforms):
        """Atualiza as poses baseado nos marcadores detectados"""
        if 'car' in transforms:
            car_pos = transforms['car'][:3, 3]
            self.car_pose = SE3(car_pos[0], car_pos[1], car_pos[2])

        if 'home' in transforms:
            home_pos = transforms['home'][:3, 3]
            self.home_pose = SE3(home_pos[0], home_pos[1], home_pos[2])

        if 'dropoff' in transforms:
            dropoff_pos = transforms['dropoff'][:3, 3]
            self.dropoff_pose = SE3(dropoff_pos[0], dropoff_pos[1], dropoff_pos[2])

        # Update markers immediately after receiving new transforms
        if self.simulation_env is not None:
            self.update_simulation_markers()

    def update_arm(self, q):
        """Atualiza a configuração do braço"""
        self.q_current = q
        print("🔧 Braço atualizado para:", np.round(q, 3))
        self.update_simulation_display(q)

    def show(self):
        """Mostra a simulação do sistema completo"""
        if self.simulation_env is None:
            self.initialize_simulation()
        plt.show(block=True)
        time.sleep(0.1)


if __name__ == "__main__":
    sim = DigitalTwinSimulator()

    # Atualizar pose do braço e do carro
    sim.initialize_simulation()
    sim.update_arm(INITIAL_ARM_ANGLES_RAD)
    
    # Valores de teste mais realistas para os marcadores (em metros)
    # Home no centro
    home_transform = np.array([
        [1, 0, 0, 0.0],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    
    # Carro a 22cm do home em x
    car_transform = np.array([
        [1, 0, 0, 0.22],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])
    
    # Dropoff a 20cm em x e -15cm em y
    dropoff_transform = np.array([
        [1, 0, 0, 0.20],
        [0, 1, 0, -0.15],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])

    sim.update_from_vision({
        'home': home_transform,
        'car': car_transform,
        'dropoff': dropoff_transform
    })

    # Mostrar simulação
    sim.show()
