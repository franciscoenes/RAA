import cv2
import os

cap = cv2.VideoCapture(1)
save_dir = "calib_images"
os.makedirs(save_dir, exist_ok=True)

i = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("Calibração", frame)
    key = cv2.waitKey(1)
    if key == ord('s'):
        filename = os.path.join(save_dir, f"frame_{i:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"Imagem {i} guardada em {filename}")
        i += 1
    elif key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()