"""Utilitários para cálculo de posições no espaço de trabalho"""

import numpy as np
from config import WORKSPACE_LIMITS


def calculate_safe_picking_position(car_pos_cm):
    """
    Calcula uma posição segura para recolha do objeto.

    Parâmetros:
        car_pos_cm: Posição do automóvel em centímetros [x, y, z]

    Retorna:
        Posição ajustada para recolha segura, respeitando os limites do espaço de trabalho
    """
    car_pos = np.array(car_pos_cm)
    xy_distance = np.sqrt(
        car_pos[0] ** 2 + car_pos[1] ** 2
    )  # Distância no plano horizontal

    # Ajusta a posição se estiver demasiado próxima da base
    if xy_distance < WORKSPACE_LIMITS["xy_min"]:
        scale = WORKSPACE_LIMITS["xy_min"] / xy_distance
        car_pos[0] *= scale  # Ajuste da coordenada X
        car_pos[1] *= scale  # Ajuste da coordenada Y

    # Define uma altura segura para a recolha, limitada para melhor alcance
    picking_height = max(
        WORKSPACE_LIMITS["z_min"], min(car_pos[2] + 10.0, 18.0)
    )  # Máximo de 18cm
    return np.array([car_pos[0], car_pos[1], picking_height])


def get_relative_position(home_transform, target_transform):
    """
    Calcula a posição relativa entre marcadores no espaço.

    Parâmetros:
        home_transform: Matriz de transformação homogénea do marcador de referência (4x4)
        target_transform: Matriz de transformação homogénea do marcador alvo (4x4)

    Retorna:
        Posição relativa em centímetros, projetada no plano da mesa (Z=0)
    """
    # Cálculo da transformação relativa entre os marcadores
    relative_transform = np.linalg.inv(home_transform) @ target_transform
    pos_m = relative_transform[:3, 3]  # Extração do vetor de translação
    pos_cm = pos_m * 100.0  # Conversão de metros para centímetros

    # Projeção no plano da mesa (Z=0), assumindo que os marcadores estão sobre a mesa
    return np.array([pos_cm[0], pos_cm[1], 0.0])
