import numpy as np

class RobotArmRRR:
    def __init__(self, link_lengths):
        self.l1, self.l2, self.l3 = link_lengths  # Comprimentos dos elos

    def FwdKin(self, theta):
        """Cinemática direta: retorna a posição [x, y, z] da garra"""
        t1, t2, t3 = theta
        
        # Cinemática direta para braço RRR (base rotacional + 2 planos)
        x = (self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)) * np.cos(t1)
        y = (self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)) * np.sin(t1)
        z = self.l1 + self.l2 * np.sin(t2) + self.l3 * np.sin(t2 + t3)

        return np.array([x, y, z])

    def InvKin(self, pos):
        """Cinemática inversa: retorna lista de soluções [theta1, theta2, theta3]"""
        x, y, z = pos

        # Calcular theta1 diretamente
        t1 = np.arctan2(y, x)

        # Projeção no plano base
        r = np.sqrt(x**2 + y**2)
        z_offset = z - self.l1

        D = (r**2 + z_offset**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        
        if np.abs(D) > 1:
            return []  # Sem solução real

        t3_1 = np.arctan2(np.sqrt(1 - D**2), D)
        t3_2 = np.arctan2(-np.sqrt(1 - D**2), D)

        solutions = []
        for t3 in [t3_1, t3_2]:
            k1 = self.l2 + self.l3 * np.cos(t3)
            k2 = self.l3 * np.sin(t3)
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

        dx_dt1 = - (l2c2 + l3c23) * np.sin(t1)
        dy_dt1 =   (l2c2 + l3c23) * np.cos(t1)
        dz_dt1 = 0

        dx_dt2 = - (l2s2 + l3s23) * np.cos(t1)
        dy_dt2 = - (l2s2 + l3s23) * np.sin(t1)
        dz_dt2 =   (l2c2 + l3c23)

        dx_dt3 = - l3s23 * np.cos(t1)
        dy_dt3 = - l3s23 * np.sin(t1)
        dz_dt3 =   l3c23

        J = np.array([
            [dx_dt1, dx_dt2, dx_dt3],
            [dy_dt1, dy_dt2, dy_dt3],
            [dz_dt1, dz_dt2, dz_dt3],
        ])
        return J

    def InvDiffKin(self, theta):
        """Jacobiano inverso: mapeia velocidades da garra para velocidades das juntas"""
        J = self.DiffKin(theta)
        if np.linalg.matrix_rank(J) < 3:
            raise ValueError("Jacobiano singular (não inversível)")
        return np.linalg.inv(J)


if __name__ == "__main__":
    # Exemplo com link lengths em metros: l1 = altura da base, l2, l3 = segmentos do braço
    arm = RobotArmRRR(link_lengths=[0.1, 0.15, 0.15])

    print("=== FwdKin ===")
    pos = arm.FwdKin([0, np.pi/4, -np.pi/4])
    print("Posição final:", pos)

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
