import cv2
import numpy as np
import glob
from config import CAMERA_PARAMS_FILE

# Definição das dimensões do padrão de calibração
# Número de intersecções internas do tabuleiro de xadrez (onde as linhas se cruzam)
chessboard_size = (9, 6)
square_size = 0.024  # Tamanho do quadrado em metros (24mm)

# Critérios de otimização para refinamento da deteção dos cantos
# Utiliza critérios de término baseados em precisão e número máximo de iterações
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Criação do padrão de pontos 3D no referencial do mundo
# Define uma grelha de pontos 3D correspondente às intersecções do tabuleiro
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : chessboard_size[0], 0 : chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # Aplica a escala real em metros

# Inicialização das listas para armazenar os pontos de calibração
objpoints = []  # Lista para armazenar os pontos 3D do mundo real
imgpoints = []  # Lista para armazenar os pontos 2D correspondentes na imagem

# Carrega todas as imagens PNG da pasta de calibração
images = glob.glob("calib_images/*.png")

for fname in images:
    # Carrega e converte cada imagem para escala de cinzentos
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Deteta os cantos do padrão de xadrez na imagem
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)  # Adiciona os pontos 3D do padrão

        # Refina a posição dos cantos com precisão subpixel
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(refined_corners)

        # Visualização dos cantos detetados para verificação
        cv2.drawChessboardCorners(img, chessboard_size, refined_corners, ret)
        cv2.imshow("Cantos Detetados", img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Realiza a calibração da câmara utilizando os pontos recolhidos
# Calcula a matriz intrínseca e os coeficientes de distorção
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Matriz intrínseca da câmara:\n", camera_matrix)
print("Coeficientes de distorção:\n", dist_coeffs)

# Guarda os parâmetros de calibração num ficheiro para utilização posterior
np.savez(CAMERA_PARAMS_FILE, K=camera_matrix, D=dist_coeffs)
