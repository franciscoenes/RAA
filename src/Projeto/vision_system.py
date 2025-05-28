import cv2
import cv2.aruco as aruco
import numpy as np

class VisionSystem:
    def __init__(self, camera_params_path, marker_length):
        self.marker_length = marker_length  # em metros
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # Carregar parametros da câmara
        params = np.load(camera_params_path)
        self.K = params['K']
        self.D = params['D']

        # IDs atribuídos (ajustar conforme preferível)
        self.ID_HOME = 0     # marcador que define o ponto de origem
        self.ID_CAR = 1      # marcador que define a posição do carro
        self.ID_DROPOFF = 2  # marcador que define o ponto de entrega

    def detect_markers(self, frame):
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.D)
            return dict(zip(ids.flatten(), zip(rvecs, tvecs)))
        return {}

    def get_transforms(self, frame):
        detected = self.detect_markers(frame)
        transforms = {}

        if self.ID_HOME in detected:
            rvec_H, tvec_H = detected[self.ID_HOME]
            transforms['home'] = self._build_transform(rvec_H, tvec_H)

        if self.ID_CAR in detected:
            rvec_C, tvec_C = detected[self.ID_CAR]
            transforms['car'] = self._build_transform(rvec_C, tvec_C)

        if self.ID_DROPOFF in detected:
            rvec_D, tvec_D = detected[self.ID_DROPOFF]
            transforms['dropoff'] = self._build_transform(rvec_D, tvec_D)

        return transforms

    def _build_transform(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    def is_car_at_home(self, transforms, threshold=0.1):
        """Check if car is close enough to home position"""
        if 'home' in transforms and 'car' in transforms:
            home_pos = transforms['home'][:3, 3]
            car_pos = transforms['car'][:3, 3]
            distance = np.linalg.norm(home_pos - car_pos)
            return distance < threshold
        return False

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # Ajustar índice conforme a câmara correta
    vision = VisionSystem("camera_params.npz", marker_length=0.097)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        transforms = vision.get_transforms(frame)

        # Mostrar todos os marcadores com eixos
        detected = vision.detect_markers(frame)
        for id, (rvec, tvec) in detected.items():
            cv2.drawFrameAxes(frame, vision.K, vision.D, rvec, tvec, 0.10)

        # Mostrar status dos marcadores
        y_pos = frame.shape[0] - 10
        for marker_name in ['home', 'car', 'dropoff']:
            status = "Detectado" if marker_name in transforms else "Não detectado"
            color = (0, 255, 0) if marker_name in transforms else (0, 0, 255)
            cv2.putText(frame, f"{marker_name}: {status}", (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            y_pos -= 20

        if vision.is_car_at_home(transforms):
            cv2.putText(frame, "Carro na posição!", (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("Vision System", frame)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
