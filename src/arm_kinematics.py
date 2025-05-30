import numpy as np
from config import LINK_LENGTHS_METERS, INITIAL_ARM_ANGLES_RAD, WORKSPACE_LIMITS
from angle_utils import normalize_angle


class RobotArmRRR:
    def __init__(self, link_lengths=LINK_LENGTHS_METERS):
        # Define os comprimentos dos três segmentos do braço robótico em metros
        self.l1, self.l2, self.l3 = link_lengths

    def FwdKin(self, theta):
        """Implementa a cinemática direta do braço robótico.
        Calcula a posição [x, y, z] do efetuador final com base nos ângulos das juntas.
        """
        t1, t2, t3 = theta

        # Aplica as equações da cinemática direta para calcular a posição do efetuador
        x = (self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)) * np.cos(t1)
        y = (self.l2 * np.cos(t2) + self.l3 * np.cos(t2 + t3)) * np.sin(t1)
        z = self.l1 + self.l2 * np.sin(t2) + self.l3 * np.sin(t2 + t3)

        return np.array([x, y, z])

    def InvKin(self, pos):
        """Implementa a cinemática inversa do braço robótico.
        Calcula os ângulos das juntas [theta1, theta2, theta3] necessários para alcançar uma posição específica.
        """
        x, y, z = pos

        # Calcula o ângulo da base (theta1) através da projeção no plano XY
        t1 = np.arctan2(y, x)

        # Calcula a distância radial no plano XY e a altura relativa à base
        r = np.sqrt(x**2 + y**2)
        z_offset = z - self.l1

        # Aplica a lei dos cossenos para determinar theta3
        D = (r**2 + z_offset**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)

        # Verifica se existe uma solução física possível
        if np.abs(D) > 1:
            return []  # Retorna lista vazia se não houver solução possível

        # Calcula as duas possíveis soluções para theta3 (configuração cotovelo para cima e para baixo)
        t3_1 = np.arctan2(np.sqrt(1 - D**2), D)
        t3_2 = np.arctan2(-np.sqrt(1 - D**2), D)

        solutions = []
        for t3 in [t3_1, t3_2]:
            k1 = self.l2 + self.l3 * np.cos(t3)
            k2 = self.l3 * np.sin(t3)
            # Calcula theta2 utilizando relações trigonométricas
            t2 = np.arctan2(z_offset, r) - np.arctan2(k2, k1)
            solutions.append([t1, t2, t3])

        return solutions

    def DiffKin(self, theta):
        """Calcula a matriz Jacobiana do braço robótico.
        Relaciona as velocidades das juntas com as velocidades lineares do efetuador final.
        """
        t1, t2, t3 = theta

        # Cálculos intermediários para simplificar as expressões
        l2c2 = self.l2 * np.cos(t2)
        l3c23 = self.l3 * np.cos(t2 + t3)
        l2s2 = self.l2 * np.sin(t2)
        l3s23 = self.l3 * np.sin(t2 + t3)

        # Derivadas parciais da posição em relação a cada ângulo
        dx_dt1 = -(l2c2 + l3c23) * np.sin(t1)
        dy_dt1 = (l2c2 + l3c23) * np.cos(t1)
        dz_dt1 = 0

        dx_dt2 = -(l2s2 + l3s23) * np.cos(t1)
        dy_dt2 = -(l2s2 + l3s23) * np.sin(t1)
        dz_dt2 = l2c2 + l3c23

        dx_dt3 = -l3s23 * np.cos(t1)
        dy_dt3 = -l3s23 * np.sin(t1)
        dz_dt3 = l3c23

        # Construção da matriz Jacobiana 3x3
        J = np.array(
            [
                [dx_dt1, dx_dt2, dx_dt3],
                [dy_dt1, dy_dt2, dy_dt3],
                [dz_dt1, dz_dt2, dz_dt3],
            ]
        )
        return J

    def InvDiffKin(self, theta):
        """Calcula o Jacobiano inverso do braço robótico.
        Permite converter velocidades lineares do efetuador em velocidades angulares das juntas.
        """
        J = self.DiffKin(theta)
        # Verifica a possibilidade de inversão da matriz Jacobiana
        if np.linalg.matrix_rank(J) < 3:
            raise ValueError("Matriz Jacobiana singular - configuração não inversível")
        return np.linalg.inv(J)

    def get_best_ik_solution(self, solutions, current_angles):
        """Seleciona a melhor solução de cinemática inversa considerando os limites das juntas.
        Retorna a solução que minimiza o movimento total das juntas."""
        if not solutions:
            return None

        current_angles = np.array(current_angles)
        joint_limits_deg = np.array(
            [
                [-90, 90],  # Limites da base
                [-90, 90],  # Limites do ombro
                [-90, 90],  # Limites do cotovelo
            ]
        )

        # Filtra soluções dentro dos limites das juntas
        valid_solutions = []
        print("\n🔍 Soluções de cinemática inversa encontradas:")
        for i, sol in enumerate(solutions):
            angles_deg = np.degrees(sol)
            print(
                f"Solução {i+1}: Base={angles_deg[0]:.1f}°, Ombro={angles_deg[1]:.1f}°, Cotovelo={angles_deg[2]:.1f}°"
            )

            is_valid = True
            for j, (angle_deg, (min_deg, max_deg)) in enumerate(
                zip(angles_deg, joint_limits_deg)
            ):
                norm_angle = normalize_angle(angle_deg)
                if norm_angle < min_deg or norm_angle > max_deg:
                    is_valid = False
                    break
            if is_valid:
                valid_solutions.append(sol)

        if valid_solutions:
            print(f"✅ {len(valid_solutions)} soluções válidas encontradas")
            # Escolhe a solução que minimiza a variação total dos ângulos
            best_sol = None
            min_cost = float("inf")
            for sol in valid_solutions:
                angle_change_cost = np.sum(np.abs(sol - current_angles))
                if angle_change_cost < min_cost:
                    min_cost = angle_change_cost
                    best_sol = sol
            return best_sol

        print("❌ Nenhuma solução válida dentro dos limites das juntas")
        return None

    def is_position_reachable(self, target_pos_cm, current_angles):
        """Verifica se uma posição é alcançável pelo braço robótico.
        Realiza validações de espaço de trabalho e limites das juntas."""
        xy_distance = np.sqrt(target_pos_cm[0] ** 2 + target_pos_cm[1] ** 2)

        # Verifica os limites do espaço de trabalho
        if xy_distance < WORKSPACE_LIMITS["xy_min"]:
            print(
                f"❌ Posição demasiado próxima da base: {xy_distance:.1f}cm < {WORKSPACE_LIMITS['xy_min']:.1f}cm"
            )
            return False

        if xy_distance > WORKSPACE_LIMITS["xy_max"]:
            print(
                f"❌ Posição fora do alcance máximo: {xy_distance:.1f}cm > {WORKSPACE_LIMITS['xy_max']:.1f}cm"
            )
            return False

        if (
            target_pos_cm[2] < WORKSPACE_LIMITS["z_min"]
            or target_pos_cm[2] > WORKSPACE_LIMITS["z_max"]
        ):
            print(f"❌ Altura fora dos limites permitidos: {target_pos_cm[2]:.1f}cm")
            return False

        # Testa a cinemática inversa
        target_pos_m = np.array(target_pos_cm) / 100.0
        solutions = self.InvKin(target_pos_m)

        if not solutions:
            print(f"❌ Posição matematicamente inatingível: {target_pos_cm}")
            return False

        # Verifica se existe alguma solução dentro dos limites das juntas
        best_solution = self.get_best_ik_solution(solutions, current_angles)
        if best_solution is not None:
            return True

        print(f"❌ Posição fora dos limites articulares: {target_pos_cm}")
        return False


if __name__ == "__main__":
    # Código de teste para demonstrar as funcionalidades da classe
    arm = RobotArmRRR()

    print("=== Teste de Cinemática Direta ===")
    pos = arm.FwdKin(INITIAL_ARM_ANGLES_RAD)
    print("Posição do efetuador final:", pos)

    print("\n=== Teste de Cinemática Inversa ===")
    sol = arm.InvKin(pos)
    for i, s in enumerate(sol):
        print(f"Solução {i+1}: {np.round(s, 4)}")

    print("\n=== Teste do Jacobiano ===")
    J = arm.DiffKin(INITIAL_ARM_ANGLES_RAD)
    print("Matriz Jacobiana:\n", np.round(J, 4))

    print("\n=== Teste do Jacobiano Inverso ===")
    Jinv = arm.InvDiffKin(INITIAL_ARM_ANGLES_RAD)
    print("Matriz Jacobiana Inversa:\n", np.round(Jinv, 4))
