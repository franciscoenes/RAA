import cv2
import cv2.aruco as aruco
import os
from config import ARUCO_ID_HOME, ARUCO_ID_CAR, ARUCO_ID_DROPOFF


def generate_aruco_markers(
    ids, dictionary=aruco.DICT_4X4_50, marker_size=200, output_dir="markers"
):
    """
    Gera marcadores ArUco para os IDs especificados.

    Parâmetros:
        ids: Lista de IDs para os marcadores a gerar
        dictionary: Dicionário ArUco a utilizar (predefinido: 4x4 com 50 marcadores)
        marker_size: Dimensão do marcador em píxeis
        output_dir: Diretório onde os marcadores serão guardados
    """
    # Criação do diretório de saída, se não existir
    os.makedirs(output_dir, exist_ok=True)

    # Carregamento do dicionário ArUco predefinido
    aruco_dict = aruco.getPredefinedDictionary(dictionary)

    for id in ids:
        # Geração da imagem do marcador ArUco para o ID especificado
        img = aruco.generateImageMarker(aruco_dict, id, marker_size)

        # Definição do caminho completo do ficheiro para guardar o marcador
        filename = f"{output_dir}/aruco_{id}.png"

        # Armazenamento da imagem do marcador no sistema
        cv2.imwrite(filename, img)
        print(f"Marcador com ID {id} guardado em {filename}")


if __name__ == "__main__":
    # Geração dos marcadores ArUco para as posições inicial, do automóvel e de descarga
    generate_aruco_markers(ids=[ARUCO_ID_HOME, ARUCO_ID_CAR, ARUCO_ID_DROPOFF])
