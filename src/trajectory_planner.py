import numpy as np
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    def __init__(self, dt=0.01):
        """Inicializa com o periodo de amostragem dt (em segundos)"""
        self.dt = dt

    def generate_trajectory(self, q0, qf, v0, vf, T=None, a_max=None):
        """
        Gera uma trajetoria suave (interpolacao polinomial de grau 3).
        
        Argumentos:
        - q0: posicao inicial (array-like)
        - qf: posicao final (array-like)
        - v0: velocidade inicial (array-like)
        - vf: velocidade final (array-like)
        - T: duracao total (segundos) [opcional]
        - a_max: aceleracao maxima admissivel por junta [opcional]

        Retorna:
        - q: lista de posicoes ao longo do tempo (NxD)
        - dq: lista de velocidades
        - ddq: lista de aceleracoes
        - t: vetor de tempo
        """
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)

        D = len(q0)  # Numero de juntas

        if T is None:
            if a_max is None:
                raise ValueError("T ou a_max devem ser especificados")
            # Estima T com base na aceleracao maxima permitida
            delta = np.abs(qf - q0)
            T = np.max(np.sqrt(6 * delta / a_max))

        t = np.arange(0, T + self.dt, self.dt)
        N = len(t)

        q = np.zeros((N, D))
        dq = np.zeros((N, D))
        ddq = np.zeros((N, D))

        for d in range(D):
            # Coeficientes do polinomio de grau 3 para a interpolacao
            a0 = q0[d]
            a1 = v0[d]
            a2 = (3 * (qf[d] - q0[d]) - (2 * v0[d] + vf[d]) * T) / (T ** 2)
            a3 = (-2 * (qf[d] - q0[d]) + (v0[d] + vf[d]) * T) / (T ** 3)

            # Calcula posicao, velocidade e aceleracao em cada instante de tempo
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
    T = 2.0  # duracao da trajetoria

    q, dq, ddq, t = planner.generate_trajectory(q0, qf, v0, vf, T=T)

    # Visualizar as trajetorias
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    joint_labels = ['Junta 1', 'Junta 2', 'Junta 3']

    for i in range(3):
        axs[0].plot(t, q[:, i], label=joint_labels[i])
        axs[1].plot(t, dq[:, i], label=joint_labels[i])
        axs[2].plot(t, ddq[:, i], label=joint_labels[i])

    axs[0].set_ylabel("Posicao (rad)")
    axs[1].set_ylabel("Velocidade (rad/s)")
    axs[2].set_ylabel("Aceleracao (rad/s2)")
    axs[2].set_xlabel("Tempo (s)")

    for ax in axs:
        ax.grid(True)
        ax.legend()

    plt.suptitle("Trajetoria de Juntas - Planeamento Polinomial de grau 3")
    plt.tight_layout()
    plt.show()
