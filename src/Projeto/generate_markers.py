import cv2
import cv2.aruco as aruco
import os

def generate_aruco_markers(ids, dictionary=aruco.DICT_4X4_50, marker_size=200, output_dir="markers"):
    os.makedirs(output_dir, exist_ok=True)
    aruco_dict = aruco.getPredefinedDictionary(dictionary)
    
    for id in ids:
        img = aruco.generateImageMarker(aruco_dict, id, marker_size)
        filename = f"{output_dir}/aruco_{id}.png"
        cv2.imwrite(filename, img)
        print(f"Saved marker ID {id} to {filename}")

if __name__ == "__main__":
    # Define os IDs que queres usar: por ex. 0 para o braço, 1 para o carro, 2 para a zona de descarga
    generate_aruco_markers(ids=[0, 1, 2])
