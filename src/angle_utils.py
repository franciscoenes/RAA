"""Funções utilitárias para operações com ângulos"""


def normalize_angle(angle):
    """Normaliza um ângulo para o intervalo entre -180 e 180 graus, garantindo uma representação consistente"""
    return ((angle + 180) % 360) - 180
