# import cvxpy as cp
# import numpy as np
# from parameters import pms
# from zmp_controller import get_com_acc

# class MPCSolver:
#     def __init__(self, horizon=10, dt=0.005):
#         self.horizon = horizon
#         self.dt = dt
#         self.mass = pms.mass
        
#         # Весовые матрицы
#         self.Q = np.eye(2) * 10.0  # Вес ошибки положения
#         self.R = np.eye(2) * 0.1   # Вес управления
        
#         # Ограничения
#         self.max_accel = 5.0  # м/с²
#         self.prev_solution = np.zeros((horizon, 2))
        
#         # Кэширование матриц
#         self._setup_dynamics_matrices()
    
#     def _setup_dynamics_matrices(self):
#         """Предварительно вычисляем матрицы динамики"""
#         self.A = np.eye(4)
#         self.A[0, 2] = self.dt
#         self.A[1, 3] = self.dt
        
#         self.B = np.zeros((4, 2))
#         self.B[2, 0] = self.dt
#         self.B[3, 1] = self.dt
        
#     def solve(self, state, ref_traj):
#         """Решает задачу MPC с использованием выпуклой оптимизации"""
#         try:
#             N = min(self.horizon, len(ref_traj))
            
#             # Переменные оптимизации
#             u = cp.Variable((N, 2))
#             x = cp.Variable((N+1, 4))
            
#             # Начальное условие
#             constraints = [x[0] == state]
            
#             # Динамика системы
#             for k in range(N):
#                 constraints += [
#                     x[k+1] == self.A @ x[k] + self.B @ u[k],
#                     cp.norm(u[k], 'inf') <= self.max_accel
#                 ]
            
#             # Целевая функция
#             cost = 0
#             for k in range(N):
#                 cost += cp.quad_form(x[k, :2] - ref_traj[k], self.Q)
#                 cost += cp.quad_form(u[k], self.R)
            
#             # Решение задачи
#             problem = cp.Problem(cp.Minimize(cost), constraints)
#             problem.solve(solver=cp.OSQP, warm_start=True)
            
#             if problem.status == cp.OPTIMAL:
#                 self.prev_solution = u.value
#                 return self.mass * u.value[0]  # Возвращаем силу для первого шага
                
#         except Exception as e:
#             print(f"[MPC] Ошибка: {e}")
        
#         # Fallback: PD-регулятор или предыдущее решение
#         return self.mass * np.clip(self.prev_solution[0], -2.0, 2.0)
# 

import cvxpy as cp
import numpy as np
from parameters import pms
from zmp_controller import get_com_acc

class MPCSolver:
    def __init__(self, horizon=30, dt=0.01, max_accel=0.5):
        self.horizon = horizon
        self.dt = dt
        self.max_accel = max_accel
        self.mass = pms.mass
        
        # Весовые матрицы для состояния [x, y, vx, vy] и управления [ax, ay]
        self.Q = np.diag([50.0, 50.0, 10.0, 10.0])  # Штраф по положению и скорости
        self.R = np.diag([20.0, 20.0])  # Штраф по управлению
        
        self.prev_solution = np.zeros(2)
        self._setup_problem()

    def _setup_problem(self):
        self.u = cp.Variable((self.horizon, 2))  # Управление (ускорения)
        self.x = cp.Variable((self.horizon+1, 4))  # Состояние [x, y, vx, vy]
        self.ref_traj = cp.Parameter((self.horizon, 2))  # Опорная траектория
        self.init_state = cp.Parameter(4)  # Начальное состояние

        # Матрицы непрерывной системы LIPM
        h =abs(pms.lz0) # Защита от нулевой высоты
        g = pms.gravity
        omega = np.sqrt(g/h)
        
        A_cont = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [omega**2, 0, 0, 0],
            [0, omega**2, 0, 0]
        ])
        
        B_cont = np.array([
            [0, 0],
            [0, 0],
            [-omega**2, 0],
            [0, -omega**2]
        ])

        # Дискретизация (метод Эйлера)
        self.Ad = np.eye(4) + A_cont * self.dt
        self.Bd = B_cont * self.dt

        constraints = [self.x[0] == self.init_state]
        cost = 0
        
        for k in range(self.horizon):
            constraints += [
                self.x[k+1] == self.Ad @ self.x[k] + self.Bd @ self.u[k],
                cp.norm(self.u[k], 'inf') <= self.max_accel
            ]
            # Штраф по отклонению от траектории и скорости
            cost += cp.quad_form(self.x[k,:2] - self.ref_traj[k], self.Q[:2,:2])
            cost += cp.quad_form(self.x[k,2:], self.Q[2:,2:])
            # Штраф по управлению
            cost += cp.quad_form(self.u[k], self.R)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def solve(self, state, ref_traj):
        try:
            # В solve()
            if state[0] < -0.05:  # CoM слишком назад
                print("[MPC] CoM too far back, reducing forward acc.")
                self.ref_traj.value[:, 0] = np.clip(self.ref_traj.value[:, 0], -0.1, 0.1)

            # Проверка и коррекция входных данных
            if np.any(np.isnan(state)) or np.any(np.isinf(state)):
                print("[MPC] Invalid state: NaN or inf detected")
                return None
            
            if len(ref_traj.shape) == 1:
                ref_traj = ref_traj.reshape(1, -1)
            
            if len(ref_traj) < self.horizon:
                ref_traj = np.vstack([
                    ref_traj, 
                    np.tile(ref_traj[-1], (self.horizon - len(ref_traj), 1))
                ])
            
            # Установка параметров задачи
            self.init_state.value = state
            self.ref_traj.value = ref_traj[:self.horizon]
            
            # Решение с обработкой возможных ошибок
            try:
                self.problem.solve(solver=cp.OSQP, verbose=False, max_iter=10000)
            except Exception as e:
                print(f"[MPC] OSQP failed, trying ECOS: {str(e)[:200]}")
                self.problem.solve(solver=cp.ECOS, verbose=False)
            
            if self.problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                self.prev_solution = self.u.value[0].copy()
                return self.mass * self.prev_solution  # Возвращаем силу (F=ma)
            else:
                print(f"[MPC] Solver failed with status: {self.problem.status}")
                return None
            
        except Exception as e:
            print(f"[MPC] Error: {str(e)[:200]}")
            return None

mpc_solver = MPCSolver()

def solve_mpc(com, com_ref_traj, model, data):
    vel, _ = get_com_acc(model, data)
    state = np.array([com[0], com[1], vel[0], vel[1]])
    
    # Обеспечиваем правильную размерность ref_traj
    if len(com_ref_traj.shape) == 1:
        com_ref_traj = np.tile(com_ref_traj, (mpc_solver.horizon, 1))
    elif len(com_ref_traj) < mpc_solver.horizon:
        com_ref_traj = np.vstack([
            com_ref_traj, 
            np.tile(com_ref_traj[-1], (mpc_solver.horizon - len(com_ref_traj), 1))
        ])
    
    force_xy = mpc_solver.solve(state, com_ref_traj)
    
    # Безопасная обработка None-результата
    if force_xy is None:
        print("[MPC] Using fallback solution")
        return np.array([0.0, 0.0, 0.0])
    
    return np.array([force_xy[0], force_xy[1], 0.0])