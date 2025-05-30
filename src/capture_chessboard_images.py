import cv2
import os
from config import CAMERA_INDEX

# Inicialização da câmara utilizando o índice definido na configuração
cap = cv2.VideoCapture(CAMERA_INDEX)

# Definição do diretório para armazenamento das imagens de calibração
save_dir = "calib_images"
# Criação do diretório se não existir, evitando erros de escrita
os.makedirs(save_dir, exist_ok=True)

# Contador para numeração sequencial das imagens capturadas
i = 0
while True:
    # Captura de uma frame da câmara
    ret, frame = cap.read()
    if not ret:
        break

    # Apresentação da imagem em tempo real para auxiliar o posicionamento do padrão
    cv2.imshow("Calibração da Câmara", frame)

    # Verificação das teclas pressionadas
    key = cv2.waitKey(1)
    if key == ord("s"):
        # Guarda a frame atual quando a tecla 's' é pressionada
        filename = os.path.join(save_dir, f"frame_{i:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"Imagem {i} guardada com sucesso em {filename}")
        i += 1
    elif key == 27:  # Termina o programa quando a tecla 'ESC' é pressionada
        break

cap.release()
cv2.destroyAllWindows()
