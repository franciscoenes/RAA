# Importação das bibliotecas necessárias
import cv2
import cv2.aruco as aruco
import numpy as np
from config import (
    MARKER_LENGTH_METERS,
    ARUCO_ID_HOME,
    ARUCO_ID_CAR,
    ARUCO_ID_DROPOFF,
    HOME_MIN_THRESHOLD,
    HOME_MAX_THRESHOLD,
    CAMERA_INDEX,
    CAMERA_PARAMS_FILE,
    STATUS_COLORS,
)


class VisionSystem:
    def __init__(
        self, camera_params_path=CAMERA_PARAMS_FILE, marker_length=MARKER_LENGTH_METERS
    ):
        self.marker_length = marker_length  # Comprimento do marcador em metros
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # Carregamento dos parâmetros de calibração da câmara
        params = np.load(camera_params_path)
        self.K = params["K"]  # Matriz de parâmetros intrínsecos
        self.D = params["D"]  # Coeficientes de distorção

        # Identificadores dos marcadores ArUco
        self.ID_HOME = ARUCO_ID_HOME
        self.ID_CAR = ARUCO_ID_CAR
        self.ID_DROPOFF = ARUCO_ID_DROPOFF

        self.HOME_MIN_THRESHOLD = HOME_MIN_THRESHOLD
        self.HOME_MAX_THRESHOLD = HOME_MAX_THRESHOLD

    def detect_markers(self, frame):
        # Deteção dos marcadores ArUco na imagem
        corners, ids, _ = aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.parameters
        )
        if ids is not None:
            # Estimativa da pose 3D dos marcadores
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.K, self.D
            )

            # Desenho dos marcadores e eixos de coordenadas
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                cv2.drawFrameAxes(
                    frame, self.K, self.D, rvecs[i], tvecs[i], self.marker_length / 2
                )

            return dict(zip(ids.flatten(), zip(rvecs, tvecs)))
        return {}

    def get_transforms(self, transforms):
        """Calcula as transformações relativas entre os marcadores"""
        transforms_out = {}

        # Verificação da presença do marcador de referência (home)
        if self.ID_HOME in transforms:
            rvec_H, tvec_H = transforms[self.ID_HOME]
            T_home = self._build_transform(rvec_H, tvec_H)
            T_home_inv = np.linalg.inv(T_home)  # Matriz de transformação inversa

            # Definição da posição de origem
            transforms_out["home"] = np.eye(4)

            # Cálculo da posição do carro relativamente à origem
            if self.ID_CAR in transforms:
                rvec_C, tvec_C = transforms[self.ID_CAR]
                T_car = self._build_transform(rvec_C, tvec_C)
                transforms_out["car"] = T_home_inv @ T_car

            # Cálculo da posição do ponto de entrega relativamente à origem
            if self.ID_DROPOFF in transforms:
                rvec_D, tvec_D = transforms[self.ID_DROPOFF]
                T_dropoff = self._build_transform(rvec_D, tvec_D)
                transforms_out["dropoff"] = T_home_inv @ T_dropoff

        return transforms_out

    def _build_transform(self, rvec, tvec):
        """Construção da matriz de transformação homogénea"""
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    def is_car_at_home(self, transforms):
        """Verifica se o carro está na posição correta relativamente à origem"""
        if "home" in transforms and "car" in transforms:
            home_pos = transforms["home"][:3, 3]
            car_pos = transforms["car"][:3, 3]
            distance = np.linalg.norm(home_pos - car_pos)

            # Conversão para centímetros
            distance_cm = distance * 100

            # Verificação da distância aceitável
            is_at_home = (
                distance >= self.HOME_MIN_THRESHOLD
                and distance <= self.HOME_MAX_THRESHOLD
            )

            # Determinação do estado e cor correspondente
            if distance < self.HOME_MIN_THRESHOLD:
                status = "Muito próximo"
                color = (0, 0, 255)  # Vermelho
            elif distance > self.HOME_MAX_THRESHOLD:
                status = "Muito distante"
                color = (0, 0, 255)  # Vermelho
            else:
                status = "Em posição"
                color = (0, 255, 0)  # Verde

            # Armazenamento para visualização
            self.last_home_distance = distance_cm
            self.last_is_at_home = is_at_home
            self.last_status = status
            self.last_color = color

            return is_at_home
        return False

    def draw_status_overlay(self, frame):
        """Desenha informações de estado sobre a imagem"""
        # Apresentação do estado de deteção e posições dos marcadores
        y_pos = frame.shape[0] - 10
        for marker_name in ["home", "car", "dropoff"]:
            if marker_name in self.last_transforms:
                # Obtenção da posição da matriz de transformação
                position = self.last_transforms[marker_name][:3, 3]
                status = f"Detetado - Pos [x,y,z]: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]m"
                color = STATUS_COLORS["success"]
            else:
                status = "Não detetado"
                color = STATUS_COLORS["error"]

            cv2.putText(
                frame,
                f"{marker_name}: {status}",
                (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
            y_pos -= 20

        # Apresentação da distância à origem
        if hasattr(self, "last_home_distance"):
            cv2.putText(
                frame,
                f"Distância até Home: {self.last_home_distance:.1f}cm ({self.last_status})",
                (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                self.last_color,
                1,
                cv2.LINE_AA,
            )
            y_pos -= 20

            # Apresentação do intervalo aceitável
            cv2.putText(
                frame,
                f"Intervalo aceitável: {self.HOME_MIN_THRESHOLD*100:.1f}cm - {self.HOME_MAX_THRESHOLD*100:.1f}cm",
                (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                STATUS_COLORS["neutral"],
                1,
                cv2.LINE_AA,
            )


# Execução principal do programa
if __name__ == "__main__":
    cap = cv2.VideoCapture(CAMERA_INDEX)
    vision = VisionSystem()  # Inicialização com valores predefinidos

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Deteção dos marcadores e cálculo das transformações
        detected = vision.detect_markers(frame)
        transforms = vision.get_transforms(detected)
        vision.last_transforms = transforms  # Armazenamento para visualização

        # Verificação da posição do carro e apresentação do estado
        vision.is_car_at_home(transforms)
        vision.draw_status_overlay(frame)

        cv2.imshow("Sistema de Visão", frame)
        if cv2.waitKey(1) == 27:  # Tecla ESC para sair
            break

    cap.release()
    cv2.destroyAllWindows()
