import cv2
import numpy as np
import glob

# Tamanho interior do tabuleiro de Xadrez (Numero de cantos internos onde as linhas do tabuleiro se cruzam)
chessboard_size = (9, 6)
square_size = 0.024  # 24mm

# Criterios de precisao para cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Pontos 3D no mundo real para o padrao do tabuleiro
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # Pontos 3D acumulados
imgpoints = []  # Pontos 2D (cantos) detectados nas imagens

# Lista todos os ficheiros PNG na pasta de imagens para calibracao
images = glob.glob('calib_images/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Tenta encontrar os cantos do tabuleiro de xadrez
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)  # Adiciona os pontos 3D correspondentes

        # Refina a localizacao dos cantos para maior precisao
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(refined_corners)
        
        # Desenha os cantos na imagem para visualizacao
        cv2.drawChessboardCorners(img, chessboard_size, refined_corners, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Executa a calibracao da camera com os pontos recolhidos
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Matriz da camera:\n", camera_matrix)
print("Coeficientes de distorcao:\n", dist_coeffs)

# Guarda os parametros calibrados para usar posteriormente
np.savez("camera_params.npz", K=camera_matrix, D=dist_coeffs)