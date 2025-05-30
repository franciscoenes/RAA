import numpy as np
import time
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
import matplotlib.pyplot as plt
from config import (
    LINK_LENGTHS_METERS, INITIAL_ARM_ANGLES_RAD,
    SIMULATION_PLOT_LIMITS, TRAJECTORY_DT
)

# === Configurações Iniciais ===
# Inicialização dos objetos principais
arm = RobotArmRRR()  # Using default config values
planner = TrajectoryPlanner()  # Using default config values
simulator = DigitalTwinSimulator()

# Variáveis globais para a simulação
simulation_env = None
current_angles = INITIAL_ARM_ANGLES_RAD.copy()

def initialize_simulation():
    """Inicializa o ambiente de simulação de forma limpa"""
    global simulation_env
    
    plt.ion()  # Ativa o modo interativo
    
    # Utiliza a Toolbox de Robótica para criar o ambiente
    simulation_env = simulator.robot.plot(
        current_angles,
        backend='pyplot',
        block=False,
        jointaxes=True,
        eeframe=True,
        shadow=False
    )
    
    # Configura o gráfico
    ax = simulation_env.ax
    ax.set_xlim(SIMULATION_PLOT_LIMITS['x'])
    ax.set_ylim(SIMULATION_PLOT_LIMITS['y'])
    ax.set_zlim(SIMULATION_PLOT_LIMITS['z'])
    ax.set_title("Simulação do Braço Robótico")
    
    plt.show(block=False)
    return simulation_env

def update_simulation_display(joint_angles_rad):
    """Atualiza o ecrã da simulação sem bloquear"""
    global simulation_env
    
    if simulation_env is not None:
        try:
            # Primeiro atualiza a configuração das juntas do robô
            simulation_env.robots[0].robot.q = joint_angles_rad
            
            # Depois chama o passo de atualização
            simulation_env.step(dt=0.01)
            
            # Pequena pausa para animação suave
            plt.pause(0.001)
        except Exception as e:
            print(f"Aviso: Erro na atualização da simulação: {e}")

# === Interface de Utilizador ===
print("==============================")
print(" SIMULAÇÃO DO BRAÇO RRR")
print("==============================")

try:
    # Inicializa a simulação
    simulation_env = initialize_simulation()
    print("✅ Simulação iniciada com sucesso")
    
    # Define o estado inicial
    q_init = current_angles
    simulator.update_arm(q_init)

    while True:
        # Solicita a posição final ao utilizador
        print("\nIntroduza a posição final pretendida para a garra (x y z), em metros (ex: 0.1 0.2 0.3) ou 'q' para sair:")
        pos_str = input("> ").strip().lower()
        
        if pos_str == 'q':
            print("\n👋 A encerrar a simulação...")
            break
            
        try:
            pos_str = pos_str.replace('(', '').replace(')', '')
            x, y, z = map(float, pos_str.split())
            pos_final = [x, y, z]

            # Resolve a cinemática inversa
            solutions = arm.InvKin(pos_final)
            if not solutions:
                print("❌ Posição inalcançável pelo braço.")
                continue

            print("\nForam encontradas", len(solutions), "soluções possíveis. A utilizar a primeira.")
            q_target = solutions[0]

            # Gera a trajetória
            q, dq, ddq, t = planner.generate_trajectory(q_init, q_target, [0,0,0], [0,0,0], T=2.0)

            print("\nA iniciar o movimento...")
            for i in range(len(t)):
                q_i = q[i]
                update_simulation_display(q_i)
                time.sleep(0.02)  # Pequeno atraso para movimento suave

            print("✅ Movimento concluído")
            q_init = q_target  # Atualiza a posição inicial para o próximo movimento
            current_angles = q_target

        except ValueError:
            print("❌ Erro: Formato inválido. Utilize três números separados por espaço.")
        except Exception as e:
            print("❌ Erro:", e)

except KeyboardInterrupt:
    print("\n⚠️ Interrompido pelo utilizador")
except Exception as e:
    print(f"❌ Erro fatal: {str(e)}")
finally:
    print("🧹 A limpar recursos...")
    if simulation_env:
        plt.close('all')
