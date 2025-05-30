import numpy as np
import time
import matplotlib.pyplot as plt
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
from config import INITIAL_ARM_ANGLES_RAD

# === Configurações Iniciais ===
# Inicialização dos objetos principais
arm = RobotArmRRR()  # Using default config values
planner = TrajectoryPlanner()  # Using default config values
simulator = DigitalTwinSimulator()

# Variáveis globais para a simulação
current_angles = INITIAL_ARM_ANGLES_RAD.copy()

# === Interface de Utilizador ===
print("==============================")
print(" SIMULAÇÃO DO BRAÇO RRR")
print("==============================")

try:
    # Inicializa a simulação
    simulator.initialize_simulation()
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
                simulator.update_simulation_display(q_i)
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
    plt.close('all')
