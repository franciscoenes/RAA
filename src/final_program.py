"""Programa principal para controlo do braço robótico"""

import numpy as np
import cv2
from arm_kinematics import RobotArmRRR
from trajectory_planner import TrajectoryPlanner
from digital_twin import DigitalTwinSimulator
from vision_system import VisionSystem
import matplotlib.pyplot as plt
from arduino_control import ArduinoArmControllerSerial
from position_utils import calculate_safe_picking_position, get_relative_position
from config import (
    SIMULATION_MODE,
    CAMERA_INDEX,
    ARDUINO_PORT,
    ARDUINO_BAUD_RATE,
    INITIAL_ARM_ANGLES_RAD,
    HOME_POSITION,
)

# Inicialização dos componentes principais do sistema
arm = RobotArmRRR()  # Inicializa o modelo cinemático do braço
planner = TrajectoryPlanner()  # Inicializa o planeador de trajetórias
simulator = DigitalTwinSimulator()  # Inicializa o simulador
vision = VisionSystem()  # Inicializa o sistema de visão

# Configuração do controlador do braço
planner.set_arm_controller(arm)

# Inicialização dos atributos do sistema de visão para evitar erros de atributos
vision.last_transforms = {}
vision.last_home_distance = 0.0
vision.last_is_at_home = False
vision.last_status = "Aguardando"
vision.last_color = (255, 255, 255)

# Variáveis globais para a simulação
current_angles = INITIAL_ARM_ANGLES_RAD


def send_to_real_robot(joint_angles_rad):
    """
    Envia os ângulos das juntas para o robô físico ou simulado.
    Inclui suporte para controlo da base através do Arduino.
    Parâmetros:
        joint_angles_rad: Lista com os ângulos das juntas em radianos
    """
    if SIMULATION_MODE:
        simulator.update_arm(joint_angles_rad)

        # Se o Arduino estiver ligado, envia o ângulo da base
        if (
            hasattr(send_to_real_robot, "arduino_controller")
            and send_to_real_robot.arduino_controller
        ):
            send_to_real_robot.arduino_controller.send_to_robot(joint_angles_rad)
    else:
        print(f"[REAL] A enviar ângulos: {np.degrees(joint_angles_rad)}")


# Define o controlador do robô no planeador
planner.set_robot_controller(send_to_real_robot)


def main():
    global current_angles

    print("==============================")
    print(" SISTEMA DE PICK AND PLACE")
    print("==============================")
    print(f"Modo: Simulação")

    # Tentativa de inicialização da ligação com o Arduino
    try:
        arduino_controller = ArduinoArmControllerSerial(
            port=ARDUINO_PORT, baud_rate=ARDUINO_BAUD_RATE
        )
        if arduino_controller.serial_connection:
            print("✅ Arduino conectado com sucesso! A base física será controlada.")
            send_to_real_robot.arduino_controller = arduino_controller
        else:
            print("⚠️ Arduino não conectado. A continuar apenas com simulação.")
            send_to_real_robot.arduino_controller = None
    except Exception as e:
        print(f"⚠️ Erro ao conectar com o Arduino: {e}")
        print("A continuar apenas com simulação.")
        send_to_real_robot.arduino_controller = None

    # Inicialização do ambiente de simulação
    try:
        simulator.initialize_simulation()
        print("✅ Simulação iniciada com sucesso")
    except Exception as e:
        print(f"⚠️ Erro na inicialização da simulação: {e}")

    # Configuração da janela de visualização do sistema de visão
    cv2.namedWindow("Sistema de Visão", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Sistema de Visão", 800, 600)

    # Inicialização da câmara
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Erro: Não foi possível iniciar a câmara")
        return

    current_angles = INITIAL_ARM_ANGLES_RAD
    state = "WAITING_FOR_CAR"  # Estado inicial: à espera do carro

    # Envio da posição inicial para a simulação e Arduino
    send_to_real_robot(current_angles)

    print(
        f"\nPosicione o carro entre {vision.HOME_MIN_THRESHOLD*100:.1f}cm e {vision.HOME_MAX_THRESHOLD*100:.1f}cm do ponto inicial"
    )

    try:
        while True:
            # Captura de uma frame da câmara
            ret, frame = cap.read()
            if not ret:
                print("❌ Erro: Não foi possível capturar imagem da câmara")
                break

            # Processamento do sistema de visão
            detected = vision.detect_markers(frame)  # Deteção dos marcadores ArUco
            transforms = vision.get_transforms(detected)  # Cálculo das transformações
            vision.last_transforms = transforms
            vision.is_car_at_home(transforms)  # Verificação da posição do carro
            vision.draw_status_overlay(frame)  # Desenho das informações no ecrã

            # Atualização dos marcadores na simulação
            simulator.update_from_vision(transforms)

            # Máquina de estados do sistema
            if state == "WAITING_FOR_CAR":
                if vision.is_car_at_home(transforms):
                    print("\n✅ Carro detetado na posição inicial!")
                    state = "PICKING"

                    # Cálculo da posição do carro relativamente ao marcador inicial
                    car_pos_cm = get_relative_position(
                        vision.last_transforms["home"], vision.last_transforms["car"]
                    )

                    # Cálculo da posição segura para recolha
                    picking_pos = calculate_safe_picking_position(car_pos_cm)
                    print(
                        f"\n🎯 Posição de recolha calculada: {np.round(picking_pos, 1)}cm"
                    )

                    # Movimento para a posição de recolha
                    success, current_angles = planner.move_through_safe_path(
                        picking_pos, current_angles
                    )
                    if not success:
                        print("❌ Falha no movimento para a posição de recolha")
                        state = "WAITING_FOR_CAR"
                    else:
                        print("✅ Braço na posição de recolha")
                        state = "WAITING_FOR_DROPOFF"

            elif state == "WAITING_FOR_DROPOFF":
                if "dropoff" in transforms:
                    print("\n✅ Marcador de descarga detetado!")

                    # Cálculo da posição de descarga relativamente ao marcador inicial
                    dropoff_pos_cm = get_relative_position(
                        vision.last_transforms["home"],
                        vision.last_transforms["dropoff"],
                    )

                    # Cálculo da posição correta para descarga
                    dropoff_pos = calculate_safe_picking_position(dropoff_pos_cm)
                    print(
                        f"\n🎯 Posição de descarga calculada: {np.round(dropoff_pos, 1)}cm"
                    )

                    # Movimento para a posição de descarga
                    success, current_angles = planner.move_through_safe_path(
                        dropoff_pos, current_angles
                    )
                    if not success:
                        print("❌ Falha no movimento para a posição de descarga")
                    else:
                        print("✅ Movimento concluído com sucesso")

                    # Retorno à posição inicial
                    print("\n🔄 A retornar à posição inicial...")
                    success, current_angles = planner.move_through_safe_path(
                        HOME_POSITION, current_angles
                    )
                    if success:
                        print("✅ Retorno concluído com sucesso")
                    else:
                        print("⚠️ Falha no retorno à posição inicial")

                    state = "WAITING_FOR_CAR"

            # Apresentação da imagem do sistema de visão
            cv2.imshow("Sistema de Visão", frame)

            # Verificação de teclas pressionadas
            key = cv2.waitKey(1)
            if key == ord("q"):
                print("\n👋 A encerrar o sistema...")
                break
            elif key == ord("r"):
                print("\n🔄 A reiniciar o sistema...")
                state = "WAITING_FOR_CAR"
                current_angles = INITIAL_ARM_ANGLES_RAD
                send_to_real_robot(current_angles)

    finally:
        print("🧹 A libertar recursos...")
        # Fecha a ligação com o Arduino, se existir
        if (
            hasattr(send_to_real_robot, "arduino_controller")
            and send_to_real_robot.arduino_controller
        ):
            send_to_real_robot.arduino_controller.close()
        cap.release()
        cv2.destroyAllWindows()  # Fecha todas as janelas
        plt.close("all")  # Fecha todos os gráficos


if __name__ == "__main__":
    main()
