import time
import numpy as np
import serial
from config import ARDUINO_PORT, ARDUINO_BAUD_RATE, INITIAL_ARM_ANGLES_RAD


class ArduinoArmControllerSerial:
    def __init__(self, port=ARDUINO_PORT, baud_rate=ARDUINO_BAUD_RATE):
        """
        Inicializa a comunicação série com o Arduino para controlo da rotação da base.
        Parâmetros:
            port: Porta série (exemplo: 'COM3' para Windows, '/dev/ttyUSB0' para Linux)
            baud_rate: Taxa de transmissão da comunicação série
        """
        self.port = port
        self.baud_rate = baud_rate
        self.serial_connection = None

        try:
            self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Aguarda a inicialização da ligação série
            print("✅ Ligação série ao Arduino estabelecida com sucesso")

            # Define os ângulos iniciais (utilizando valores da configuração)
            self.current_angles_deg = {
                "base": np.degrees(INITIAL_ARM_ANGLES_RAD[0]),
                "shoulder": np.degrees(INITIAL_ARM_ANGLES_RAD[1]),
                "elbow": np.degrees(INITIAL_ARM_ANGLES_RAD[2]),
                "gripper": 0,
            }

            # Envia a posição inicial da base para o Arduino
            self.set_joint_angles_deg(self.current_angles_deg)
            time.sleep(2)  # Permite tempo para o Arduino executar o movimento

        except serial.SerialException as e:
            print(f"❌ Erro ao abrir a porta série {port}: {e}")
            self.serial_connection = None
        except Exception as e:
            print(f"❌ Ocorreu um erro inesperado durante a ligação série: {e}")
            self.serial_connection = None

    def set_joint_angles_deg(self, angles_deg):
        """
        Envia apenas o ângulo da base para o Arduino através da ligação série.
        Os outros ângulos são armazenados mas não enviados.
        Parâmetros:
            angles_deg: Dicionário com os ângulos das juntas em graus
                       {'base': ângulo, 'shoulder': ângulo, 'elbow': ângulo, 'gripper': ângulo}
        """
        if self.serial_connection and self.serial_connection.isOpen():
            try:
                # Obtém apenas o ângulo da base, usa o valor atual se não especificado
                base_angle = angles_deg.get(
                    "base", self.current_angles_deg.get("base", 0)
                )

                # Armazena todos os ângulos (para compatibilidade com a simulação)
                self.current_angles_deg["base"] = base_angle
                self.current_angles_deg["shoulder"] = angles_deg.get(
                    "shoulder", self.current_angles_deg.get("shoulder")
                )
                self.current_angles_deg["elbow"] = angles_deg.get(
                    "elbow", self.current_angles_deg.get("elbow")
                )
                self.current_angles_deg["gripper"] = angles_deg.get(
                    "gripper", self.current_angles_deg.get("gripper")
                )

                # Mapeamento de ângulos entre simulação e realidade:
                # Na simulação:
                # - ângulos positivos = rotação para a direita
                # - ângulos negativos = rotação para a esquerda
                # No servomotor:
                # - 0° = posição central
                # - 90° = rotação máxima à esquerda

                # Converte o ângulo da simulação para o ângulo do servomotor
                if base_angle < 0:  # Se negativo (esquerda na simulação)
                    servo_angle = abs(
                        base_angle
                    )  # Converte para positivo (esquerda no servo)
                    print(
                        f"🔄 A converter ângulo negativo {base_angle}° para servo: {servo_angle}°"
                    )
                else:  # Se positivo (direita na simulação)
                    servo_angle = 0  # Mantém na posição central para ângulos positivos
                    print(
                        f"🔄 Ângulo positivo {base_angle}° -> mantendo na posição central"
                    )

                # Limita o ângulo entre 0-90 graus
                servo_angle = max(0, min(90, servo_angle))

                # Envia o ângulo para o Arduino
                command = f"{int(servo_angle)}\n"  # Converte para inteiro para evitar problemas com números decimais
                print(
                    f"📡 A enviar ângulo para o Arduino: {servo_angle}° (ângulo original: {base_angle}°)"
                )
                self.serial_connection.write(command.encode())
                self.serial_connection.flush()  # Garante que os dados são enviados

            except Exception as e:
                print(f"❌ Erro ao enviar o ângulo da base pela porta série: {e}")

    def send_to_robot(self, joint_angles_rad):
        """
        Envia os ângulos para o robô com suporte do Arduino para a rotação da base.
        Parâmetros:
            joint_angles_rad: Lista de ângulos das juntas em radianos
        """
        try:
            # Converte o ângulo da base de radianos para graus
            base_angle_deg = np.degrees(joint_angles_rad[0])
            print(
                f"🔄 Ângulo da base (rad): {joint_angles_rad[0]:.3f}, (graus): {base_angle_deg:.1f}°"
            )
            self.set_joint_angles_deg({"base": base_angle_deg})
        except Exception as e:
            print(f"⚠️ Aviso: Não foi possível enviar o ângulo para o Arduino: {e}")

    def close(self):
        """
        Fecha a ligação série com o Arduino.
        """
        if self.serial_connection and self.serial_connection.isOpen():
            self.serial_connection.close()
            print("Ligação série terminada")


if __name__ == "__main__":
    # Código de teste
    try:
        arm = ArduinoArmControllerSerial()

        # Testa apenas a rotação da base
        print("A testar a rotação da base...")
        arm.set_joint_angles_deg({"base": 0})
        time.sleep(1)
        arm.set_joint_angles_deg({"base": 180})
        time.sleep(1)
        arm.set_joint_angles_deg({"base": 90})
        time.sleep(1)

    except Exception as e:
        print(f"Erro durante o teste: {e}")
    finally:
        if "arm" in locals():
            arm.close()

    # Exemplo de Utilização (para testar este módulo diretamente, se necessário)
    # if __name__ == "__main__":
    #     try:
    #         arm_serial = ArduinoArmControllerSerial(port='COM3') # Ajustar porta
    #         time.sleep(3)
    #         print("A mover o braço para os ângulos: 45, 90, 45, 30")
    #         arm_serial.set_joint_angles_deg({'base': 45, 'shoulder': 90, 'elbow': 45, 'gripper': 30})
    #         time.sleep(3)
    #         print("A mover o braço para os ângulos: 135, 30, 150, 90")
    #         arm_serial.set_joint_angles_deg({'base': 135, 'shoulder': 30, 'elbow': 150, 'gripper': 90})
    #         time.sleep(3)
    #     except Exception as e:
    #         print(f"Erro durante o teste série: {e}")
    #     finally:
    #         if 'arm_serial' in locals():
    #             arm_serial.close()
