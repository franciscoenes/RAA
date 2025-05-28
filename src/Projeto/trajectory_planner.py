import numpy as np
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    def __init__(self, dt=0.01):
        """Inicializa com o período de amostragem dt (em segundos)"""
        self.dt = dt

    def generate_trajectory(self, q0, qf, v0, vf, T=None, a_max=None):
        """
        Gera uma trajetória suave (interpolação polinomial de 3ª ordem).
        
        Argumentos:
        - q0: posição inicial (array-like)
        - qf: posição final (array-like)
        - v0: velocidade inicial (array-like)
        - vf: velocidade final (array-like)
        - T: duração total (segundos) [opcional]
        - a_max: aceleração máxima admissível por junta [opcional]

        Retorna:
        - q: lista de posições ao longo do tempo (NxD)
        - dq: lista de velocidades
        - ddq: lista de acelerações
        - t: vetor de tempo
        """
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)

        D = len(q0)  # Número de juntas

        if T is None:
            if a_max is None:
                raise ValueError("T ou a_max devem ser especificados")
            # Estimar T com base em aceleração máxima
            delta = np.abs(qf - q0)
            T = np.max(np.sqrt(6 * delta / a_max))

        t = np.arange(0, T + self.dt, self.dt)
        N = len(t)

        q = np.zeros((N, D))
        dq = np.zeros((N, D))
        ddq = np.zeros((N, D))

        for d in range(D):
            # Coeficientes do polinômio de 3ª ordem
            a0 = q0[d]
            a1 = v0[d]
            a2 = (3 * (qf[d] - q0[d]) - (2 * v0[d] + vf[d]) * T) / (T ** 2)
            a3 = (-2 * (qf[d] - q0[d]) + (v0[d] + vf[d]) * T) / (T ** 3)

            q[:, d] = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3
            dq[:, d] = a1 + 2 * a2 * t + 3 * a3 * t ** 2
            ddq[:, d] = 2 * a2 + 6 * a3 * t

        return q, dq, ddq, t


if __name__ == "__main__":
    planner = TrajectoryPlanner(dt=0.01)

    # Exemplo para 3 juntas
    q0 = [0.0, 0.0, 0.0]
    qf = [np.pi/4, np.pi/6, -np.pi/6]
    v0 = [0.0, 0.0, 0.0]
    vf = [0.0, 0.0, 0.0]
    T = 2.0  # duração da trajetória

    q, dq, ddq, t = planner.generate_trajectory(q0, qf, v0, vf, T=T)

    # Visualizar as trajetórias
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    joint_labels = ['Junta 1', 'Junta 2', 'Junta 3']

    for i in range(3):
        axs[0].plot(t, q[:, i], label=joint_labels[i])
        axs[1].plot(t, dq[:, i], label=joint_labels[i])
        axs[2].plot(t, ddq[:, i], label=joint_labels[i])

    axs[0].set_ylabel("Posição (rad)")
    axs[1].set_ylabel("Velocidade (rad/s)")
    axs[2].set_ylabel("Aceleração (rad/s²)")
    axs[2].set_xlabel("Tempo (s)")

    for ax in axs:
        ax.grid(True)
        ax.legend()

    plt.suptitle("Trajetória de Juntas - Planeamento Polinomial de 3ª Ordem")
    plt.tight_layout()
    plt.show()
