import numpy as np

# === Modo de Operação ===
SIMULATION_MODE = True

# === Configuração da Câmara ===
CAMERA_INDEX = 0  # Índice da câmara a utilizar (0 para câmara predefinida)
CAMERA_PARAMS_FILE = (
    "camera_params.npz"  # Ficheiro para armazenar parâmetros de calibração
)
CAMERA_HEIGHT_CM = 69.0  # Altura da câmara em centímetros

# === Configuração dos Marcadores ArUco ===
MARKER_SIZE_CM = 9.5  # Dimensão do marcador em centímetros
MARKER_LENGTH_METERS = 0.097  # Dimensão do marcador em metros (9.7cm)
VISION_SCALE_FACTOR = 0.01  # Fator de escala para conversão de unidades

# Identificadores dos marcadores ArUco
ARUCO_ID_HOME = 0  # ID do marcador da posição inicial
ARUCO_ID_CAR = 1  # ID do marcador do automóvel
ARUCO_ID_DROPOFF = 2  # ID do marcador da posição de descarga

# Limites de tolerância para a posição inicial (em metros)
HOME_MIN_THRESHOLD = 0.21  # Limite mínimo (21cm)
HOME_MAX_THRESHOLD = 0.22  # Limite máximo (22cm)

# === Configuração do Braço Robótico ===
# Ângulos iniciais das juntas (em radianos)
INITIAL_ARM_ANGLES_RAD = [0.0, np.radians(45), np.radians(-45)]

# Dimensões dos segmentos do braço
LINK_LENGTHS_METERS = [
    0.10,
    0.15,
    0.15,
]  # Comprimentos em metros [base, braço, antebraço]

# Limites angulares das juntas (em graus)
JOINT_LIMITS_DEG = {
    "base": [-90, 90],  # Rotação da base
    "shoulder": [-90, 90],  # Rotação do ombro
    "elbow": [-90, 90],  # Rotação do cotovelo
}

# === Limites do Espaço de Trabalho (em centímetros) ===
WORKSPACE_LIMITS = {
    "xy_min": 15.0,  # Distância mínima no plano horizontal
    "xy_max": 28.0,  # Distância máxima no plano horizontal
    "z_min": 5.0,  # Altura mínima
    "z_max": 25.0,  # Altura máxima
}

# === Posição Inicial (em centímetros) ===
HOME_POSITION = np.array([22.0, 0.0, 18.0])  # Coordenadas [x, y, z]

# === Configuração do Arduino ===
ARDUINO_PORT = "COM9"  # Porta de comunicação série
ARDUINO_BAUD_RATE = 57600  # Taxa de transmissão em bits por segundo

# === Planeamento de Trajetória ===
TRAJECTORY_DT = 0.05  # Intervalo de tempo entre pontos da trajetória (em segundos)

# === Configuração da Visualização da Simulação ===
SIMULATION_PLOT_LIMITS = {
    "x": (-0.2, 0.5),  # Limites do eixo X em metros
    "y": (-0.3, 0.3),  # Limites do eixo Y em metros
    "z": (0, 0.4),  # Limites do eixo Z em metros
}

# === Cores para Indicação de Estado (formato BGR) ===
STATUS_COLORS = {
    "success": (0, 255, 0),  # Verde - Operação bem-sucedida
    "error": (0, 0, 255),  # Vermelho - Erro ou falha
    "neutral": (255, 255, 255),  # Branco - Estado neutro ou em espera
}
