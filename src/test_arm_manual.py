"""
Programa de teste para controlo manual do braço robótico.
Permite ao utilizador especificar posições alvo e visualizar o movimento do braço em simulação.
"""

import time
import matplotlib.pyplot as plt
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
from config import INITIAL_ARM_ANGLES_RAD

# === Inicialização dos Componentes do Sistema ===
# Criação das instâncias principais utilizando os valores predefinidos
arm = RobotArmRRR()  # Modelo cinemático do braço
planner = TrajectoryPlanner()  # Planeador de trajetórias
simulator = DigitalTwinSimulator()  # Simulador 3D

# Configuração inicial dos ângulos das juntas
current_angles = INITIAL_ARM_ANGLES_RAD.copy()

# === Interface com o Utilizador ===
print("==============================")
print(" SIMULAÇÃO DO BRAÇO RRR")
print("==============================")

try:
    # Inicialização do ambiente de simulação
    simulator.initialize_simulation()
    print("✅ Ambiente de simulação iniciado com sucesso")

    # Configuração da posição inicial do braço
    q_init = current_angles
    simulator.update_arm(q_init)

    while True:
        # Interface para introdução das coordenadas do ponto alvo
        print(
            "\nIntroduza a posição final pretendida (x y z), em metros (exemplo: 0.1 0.2 0.3) ou 'q' para sair:"
        )
        pos_str = input("> ").strip().lower()

        if pos_str == "q":
            print("\n👋 A encerrar o programa de teste...")
            break

        try:
            # Processamento das coordenadas introduzidas
            pos_str = pos_str.replace("(", "").replace(")", "")
            x, y, z = map(float, pos_str.split())
            pos_final = [x, y, z]

            # Cálculo da cinemática inversa para a posição alvo
            solutions = arm.InvKin(pos_final)
            if not solutions:
                print("❌ Posição fora do espaço de trabalho do braço.")
                continue

            print(
                "\nForam encontradas",
                len(solutions),
                "soluções possíveis. A utilizar a primeira solução.",
            )
            q_target = solutions[0]

            # Geração da trajetória entre a posição atual e a posição alvo
            # Velocidades e acelerações iniciais e finais nulas, duração de 2 segundos
            q, dq, ddq, t = planner.generate_trajectory(
                q_init, q_target, [0, 0, 0], [0, 0, 0], T=2.0
            )

            print("\nA executar o movimento planeado...")
            # Execução da trajetória na simulação
            for i in range(len(t)):
                q_i = q[i]
                simulator.update_simulation_display(q_i)
                time.sleep(0.02)  # Atraso para visualização suave do movimento

            print("✅ Movimento concluído com sucesso")
            q_init = q_target  # Atualização da posição inicial para o próximo movimento
            current_angles = q_target

        except ValueError:
            print(
                "❌ Erro: Formato inválido. Introduza três números separados por espaços."
            )
        except Exception as e:
            print("❌ Erro durante a execução:", e)

except KeyboardInterrupt:
    print("\n⚠️ Programa interrompido pelo utilizador")
except Exception as e:
    print(f"❌ Erro crítico do sistema: {str(e)}")
finally:
    print("🧹 A libertar recursos do sistema...")
    plt.close("all")  # Encerramento de todas as janelas de visualização
