import cv2
import cv2.aruco as aruco
import os

def generate_aruco_markers(ids, dictionary=aruco.DICT_4X4_50, marker_size=200, output_dir="markers"):
    # Cria a pasta de saida se nao existir
    os.makedirs(output_dir, exist_ok=True)
    # Obtem o dicionario pre-definido de marcadores ArUco
    aruco_dict = aruco.getPredefinedDictionary(dictionary)
    
    for id in ids:
        # Gera a imagem do marcador ArUco para o ID especificado
        img = aruco.generateImageMarker(aruco_dict, id, marker_size)
        # Define o nome do ficheiro para guardar o marcador
        filename = f"{output_dir}/aruco_{id}.png"
        # Guarda a imagem do marcador no computador
        cv2.imwrite(filename, img)
        print(f"Guardado marcador ID {id} em {filename}")

if __name__ == "__main__":
    # Chama a função que posteriormente cria os aruco markers de acordo com os ids fornecidos
    generate_aruco_markers(ids=[0, 1, 2])
