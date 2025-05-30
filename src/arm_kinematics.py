import numpy as np
from config import LINK_LENGTHS_METERS, INITIAL_ARM_ANGLES_RAD, WORKSPACE_LIMITS
from angle_utils import normalize_angle

class RobotArmRRR:
    def __init__(self, link_lengths=LINK_LENGTHS_METERS):
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

    def get_best_ik_solution(self, solutions, current_angles):
        """Choose the best IK solution based on joint limits"""
        if not solutions:
            return None
        
        current_angles = np.array(current_angles)
        joint_limits_deg = np.array([
            [-90, 90],   # Base limits
            [-90, 90],   # Shoulder limits
            [-90, 90]    # Elbow limits
        ])
        
        # Filter solutions within joint limits
        valid_solutions = []
        print("\n🔍 Soluções IK encontradas:")
        for i, sol in enumerate(solutions):
            angles_deg = np.degrees(sol)
            print(f"Solução {i+1}: Base={angles_deg[0]:.1f}°, Shoulder={angles_deg[1]:.1f}°, Elbow={angles_deg[2]:.1f}°")
            
            is_valid = True
            for j, (angle_deg, (min_deg, max_deg)) in enumerate(zip(angles_deg, joint_limits_deg)):
                norm_angle = normalize_angle(angle_deg)
                if norm_angle < min_deg or norm_angle > max_deg:
                    is_valid = False
                    break
            if is_valid:
                valid_solutions.append(sol)
        
        if valid_solutions:
            print(f"✅ {len(valid_solutions)} soluções válidas encontradas")
            # Choose solution with minimum angle change
            best_sol = None
            min_cost = float('inf')
            for sol in valid_solutions:
                angle_change_cost = np.sum(np.abs(sol - current_angles))
                if angle_change_cost < min_cost:
                    min_cost = angle_change_cost
                    best_sol = sol
            return best_sol
        
        print("❌ Nenhuma solução válida dentro dos limites das juntas")
        return None

    def is_position_reachable(self, target_pos_cm, current_angles):
        """Check if a position is reachable by the arm with IK validation"""
        xy_distance = np.sqrt(target_pos_cm[0]**2 + target_pos_cm[1]**2)
        
        # Check workspace limits first
        if xy_distance < WORKSPACE_LIMITS['xy_min']:
            print(f"❌ Posição muito próxima da base: {xy_distance:.1f}cm < {WORKSPACE_LIMITS['xy_min']:.1f}cm")
            return False
        
        if xy_distance > WORKSPACE_LIMITS['xy_max']:
            print(f"❌ Posição muito distante: {xy_distance:.1f}cm > {WORKSPACE_LIMITS['xy_max']:.1f}cm")
            return False
        
        if target_pos_cm[2] < WORKSPACE_LIMITS['z_min'] or target_pos_cm[2] > WORKSPACE_LIMITS['z_max']:
            print(f"❌ Altura fora dos limites: {target_pos_cm[2]:.1f}cm")
            return False
        
        # Test with inverse kinematics
        target_pos_m = np.array(target_pos_cm) / 100.0
        solutions = self.InvKin(target_pos_m)
        
        if not solutions:
            print(f"❌ Posição geometricamente inatingível: {target_pos_cm}")
            return False
        
        # Check if any solution is within joint limits
        best_solution = self.get_best_ik_solution(solutions, current_angles)
        if best_solution is not None:
            return True
        
        print(f"❌ Posição fora dos limites das juntas: {target_pos_cm}")
        return False

if __name__ == "__main__":
    # Exemplo usando configurações padrão
    arm = RobotArmRRR()
    
    print("=== FwdKin ===")
    pos = arm.FwdKin(INITIAL_ARM_ANGLES_RAD)
    print("Posicao final:", pos)

    print("\n=== InvKin ===")
    sol = arm.InvKin(pos)
    for i, s in enumerate(sol):
        print(f"Solução {i+1}: {np.round(s, 4)}")

    print("\n=== DiffKin ===")
    J = arm.DiffKin(INITIAL_ARM_ANGLES_RAD)
    print("Jacobiano:\n", np.round(J, 4))

    print("\n=== InvDiffKin ===")
    Jinv = arm.InvDiffKin(INITIAL_ARM_ANGLES_RAD)
    print("Jacobiano inverso:\n", np.round(Jinv, 4))
