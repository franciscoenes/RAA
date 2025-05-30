import cv2
import os
from config import CAMERA_INDEX

cap = cv2.VideoCapture(CAMERA_INDEX)  # Abre a camera 
save_dir = "calib_images"  # Pasta para guardar as imagens
os.makedirs(save_dir, exist_ok=True)  # Cria a pasta se nao existir

i = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Calibracao", frame)  # Mostra o video em tempo real
    key = cv2.waitKey(1)
    if key == ord('s'):
        # Guarda frame atual quando se pressiona 's'
        filename = os.path.join(save_dir, f"frame_{i:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"Imagem {i} guardada em {filename}")
        i += 1
    elif key == 27:  # Pressionar ESC para sair
        break

cap.release()  # Desliga/Fecha a camera
cv2.destroyAllWindows()  # Fecha todas as janelas
