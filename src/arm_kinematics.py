import numpy as np

class RobotArmRRR:
    def __init__(self, link_lengths):
        # Comprimentos dos elos do braco robotico
        self.l1, self.l2, self.l3 = link_lengths  

    def FwdKin(self, theta):
        """Cinematica direta: retorna a posicao [x, y, z] da garra"""
        t1, t2, t3 = theta
        
        # Calcula a posicao da garra com base nos angulos das juntas
        x = (self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)) * np.cos(t1)
        y = (self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)) * np.sin(t1)
        z = self.l1 + self.l2 * np.sin(t2) + self.l3 * np.sin(t2 + t3)

        return np.array([x, y, z])

    def InvKin(self, pos):
        """Cinematica inversa: retorna lista de solucoes [theta1, theta2, theta3] para uma posicao dada"""
        x, y, z = pos

        # Calcula theta1 diretamente pela projecao no plano XY
        t1 = np.arctan2(y, x)

        # Calcula a distancia no plano horizontal e a altura relativa
        r = np.sqrt(x**2 + y**2)
        z_offset = z - self.l1

        # Usado na formula do cosseno da lei dos cossenos
        D = (r**2 + z_offset**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        
        # Se D estiver fora do intervalo [-1,1], nao ha solucao real
        if np.abs(D) > 1:
            return []  # Sem solucao real

        # Duas solucoes para theta3 (cotovelo para cima e para baixo)
        t3_1 = np.arctan2(np.sqrt(1 - D**2), D)
        t3_2 = np.arctan2(-np.sqrt(1 - D**2), D)

        solutions = []
        for t3 in [t3_1, t3_2]:
            k1 = self.l2 + self.l3 * np.cos(t3)
            k2 = self.l3 * np.sin(t3)
            # Calcula theta2 com base em triangulos auxiliares
            t2 = np.arctan2(z_offset, r) - np.arctan2(k2, k1)
            solutions.append([t1, t2, t3])

        return solutions

    def DiffKin(self, theta):
        """Jacobiano J(theta): mapeia velocidades das juntas para velocidades da garra"""
        t1, t2, t3 = theta

        l2c2 = self.l2 * np.cos(t2)
        l3c23 = self.l3 * np.cos(t2 + t3)
        l2s2 = self.l2 * np.sin(t2)
        l3s23 = self.l3 * np.sin(t2 + t3)

        # Derivadas parciais da posicao em relacao aos angulos das juntas
        dx_dt1 = - (l2c2 + l3c23) * np.sin(t1)
        dy_dt1 =   (l2c2 + l3c23) * np.cos(t1)
        dz_dt1 = 0

        dx_dt2 = - (l2s2 + l3s23) * np.cos(t1)
        dy_dt2 = - (l2s2 + l3s23) * np.sin(t1)
        dz_dt2 =   (l2c2 + l3c23)

        dx_dt3 = - l3s23 * np.cos(t1)
        dy_dt3 = - l3s23 * np.sin(t1)
        dz_dt3 =   l3c23

        # Matriz jacobiana 3x3
        J = np.array([
            [dx_dt1, dx_dt2, dx_dt3],
            [dy_dt1, dy_dt2, dy_dt3],
            [dz_dt1, dz_dt2, dz_dt3],
        ])
        return J

    def InvDiffKin(self, theta):
        """Jacobiano inverso: mapeia das velocidades da garra para as velocidades das juntas"""
        J = self.DiffKin(theta)
        # Verificar se o jacobiano se pode inverter
        if np.linalg.matrix_rank(J) < 3:
            raise ValueError("Jacobiano singular (nao inversivel)")
        return np.linalg.inv(J)


if __name__ == "__main__":
    # Exemplo com comprimentos dos elos em metros, l1 e a altura da base e l2 e l3 sao segmentos do braco
    arm = RobotArmRRR(link_lengths=[0.1, 0.15, 0.15])

    print("=== FwdKin ===")
    pos = arm.FwdKin([0, np.pi/4, -np.pi/4])
    print("Posicao final:", pos)

    print("\n=== InvKin ===")
    sol = arm.InvKin(pos)
    for i, s in enumerate(sol):
        print(f"Solução {i+1}: {np.round(s, 4)}")

    print("\n=== DiffKin ===")
    J = arm.DiffKin([0, np.pi/4, -np.pi/4])
    print("Jacobiano:\n", np.round(J, 4))

    print("\n=== InvDiffKin ===")
    Jinv = arm.InvDiffKin([0, np.pi/4, -np.pi/4])
    print("Jacobiano inverso:\n", np.round(Jinv, 4))
