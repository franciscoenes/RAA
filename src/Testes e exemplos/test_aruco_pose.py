import cv2
import cv2.aruco as aruco
import numpy as np

# Carregar os parâmetros da câmara calibrada
params = np.load("camera_params.npz")
camera_matrix = params["K"]
dist_coeffs = params["D"]

# Dicionário de marcadores e tamanho real (20 cm)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
marker_length = 0.097  # metros

# Selecionar a câmara correta
cap = cv2.VideoCapture(1)

font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detetar marcadores
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
    overlay_text = []  # Guardar linhas de texto para overlay

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        for i, id in enumerate(ids.flatten()):
            rvec, tvec = rvecs[i], tvecs[i]

            # Desenhar eixos 3D
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.10)

            # Guardar texto para overlay
            t = tvec.flatten()
            overlay_text.append(f"ID {id} → tvec: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]")

    # Escrever overlay no canto inferior esquerdo
    height = frame.shape[0]
    for i, line in enumerate(overlay_text):
        y = height - 10 - (20 * (len(overlay_text) - i - 1))
        cv2.putText(frame, line, (10, y), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Mostrar imagem
    cv2.imshow("ArUco Pose Test", frame)
    if cv2.waitKey(1) == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
