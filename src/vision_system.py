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

        self.HOME_MIN_THRESHOLD = 0.21  # 21cm minimum (increased from 15cm)
        self.HOME_MAX_THRESHOLD = 0.22  # 22cm maximum (increased from 17cm)

    def detect_markers(self, frame):
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.D)
            
            # Draw markers and axes
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, self.K, self.D, rvecs[i], tvecs[i], self.marker_length/2)
            
            return dict(zip(ids.flatten(), zip(rvecs, tvecs)))
        return {}

    def get_transforms(self, transforms):
        """Atualiza as poses baseado nos marcadores detectados"""
        transforms_out = {}

        # Primeiro, verifica se temos o marcador home
        if self.ID_HOME in transforms:
            rvec_H, tvec_H = transforms[self.ID_HOME]
            T_home = self._build_transform(rvec_H, tvec_H)
            T_home_inv = np.linalg.inv(T_home)  # Inversa da transformação home
            
            # O home sempre estará na origem
            transforms_out['home'] = np.eye(4)

            # Calcula posição do carro relativa ao home
            if self.ID_CAR in transforms:
                rvec_C, tvec_C = transforms[self.ID_CAR]
                T_car = self._build_transform(rvec_C, tvec_C)
                transforms_out['car'] = T_home_inv @ T_car

            # Calcula posição do ponto de entrega relativa ao home
            if self.ID_DROPOFF in transforms:
                rvec_D, tvec_D = transforms[self.ID_DROPOFF]
                T_dropoff = self._build_transform(rvec_D, tvec_D)
                transforms_out['dropoff'] = T_home_inv @ T_dropoff

        return transforms_out

    def _build_transform(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T

    def is_car_at_home(self, transforms):
        """Check if car is within the acceptable distance range from home position"""
        if 'home' in transforms and 'car' in transforms:
            home_pos = transforms['home'][:3, 3]
            car_pos = transforms['car'][:3, 3]
            distance = np.linalg.norm(home_pos - car_pos)
            
            # Convert to cm for display
            distance_cm = distance * 100
            
            # Check if distance is within acceptable range
            is_at_home = (distance >= self.HOME_MIN_THRESHOLD and 
                         distance <= self.HOME_MAX_THRESHOLD)
            
            # Determine status message and color
            if distance < self.HOME_MIN_THRESHOLD:
                status = "Muito próximo"
                color = (0, 0, 255)  # Red
            elif distance > self.HOME_MAX_THRESHOLD:
                status = "Muito distante"
                color = (0, 0, 255)  # Red
            else:
                status = "Em posição"
                color = (0, 255, 0)  # Green
            
            # Store for display
            self.last_home_distance = distance_cm
            self.last_is_at_home = is_at_home
            self.last_status = status
            self.last_color = color
            
            return is_at_home
        return False

    def draw_status_overlay(self, frame):
        """Draw status information on frame"""
        # Draw marker detection status and positions
        y_pos = frame.shape[0] - 10
        for marker_name in ['home', 'car', 'dropoff']:
            if marker_name in self.last_transforms:
                # Get position from transformation matrix
                position = self.last_transforms[marker_name][:3, 3]
                status = f"Detectado - Pos [x,y,z]: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]m"
                color = (0, 255, 0)
            else:
                status = "Não detectado"
                color = (0, 0, 255)
            
            cv2.putText(frame, f"{marker_name}: {status}", (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            y_pos -= 20

        # Draw distance to home if available
        if hasattr(self, 'last_home_distance'):
            cv2.putText(frame, 
                       f"Distância até Home: {self.last_home_distance:.1f}cm ({self.last_status})",
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.last_color, 1, cv2.LINE_AA)
            y_pos -= 20
            
            # Add range information
            cv2.putText(frame,
                       f"Faixa aceitável: {self.HOME_MIN_THRESHOLD*100:.1f}cm - {self.HOME_MAX_THRESHOLD*100:.1f}cm",
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # Ajustar índice conforme a câmara correta
    vision = VisionSystem("camera_params.npz", marker_length=0.097)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect markers and get transforms
        detected = vision.detect_markers(frame)
        transforms = vision.get_transforms(detected)
        vision.last_transforms = transforms  # Store for status overlay

        # Check car position and draw status
        vision.is_car_at_home(transforms)
        vision.draw_status_overlay(frame)

        cv2.imshow("Vision System", frame)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
