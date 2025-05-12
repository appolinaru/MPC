import mujoco as mj # type: ignore
import mujoco.viewer as viewer # type: ignore
import numpy as np # type: ignore
import globals
from state_machine import state_machine
from cartesian_traj import cartesian_traj
from joint_trah import joint_traj
from joint_control import joint_control 
from high_level_control import high_level_control
from zmp_controller import zmp_controller,compute_com
import pandas as pd
import time

start_time = time.time()
flag_trajectory_generation = 0

model_path = r"C:\Users\Polina\Documents\ITMO\Graduation_Thesis\mujoco-3.3.0-windows-x86_64\model\unitree_a1\scene.xml"
model = mj.MjModel.from_xml_path(model_path)
data = mj.MjData(model)

globals.init()
hip = 0
pitch = 0.9
knee = -1.8

# sol = forward_kinematics_leg(np.array([hip,pitch,knee]),0)
# end_eff_pos = sol.end_eff_pos
# lz0=end_eff_pos[2]
#print(lz0)

pos = np.array([0,0,0.3])
quat = np.array([1,0,0,0])
qleg = np.array([hip,pitch,knee])

#setting initial position
data.qpos = np.concatenate((pos,quat,qleg,qleg,qleg,qleg))

com_history = []
zmp_history = []
time_history = []

# Увеличьте шаг симуляции и уменьшите частоту записи
model.opt.timestep = 0.002  # Увеличение с 0.001
record_interval = 0.007  # Записывать каждые 20 мс вместо каждого шага


dt = 0.001
N = int(10 / dt)  # если ты хочешь 10 секунд траектории
# длина траектории
start_pos = np.array([0.0, 0.0])  # текущее положение CoM
velocity = np.array([0.1, 0.0])   # пусть CoM движется по X со скоростью 0.1 м/с

globals.com_ref_traj = np.array([start_pos + i * dt * velocity for i in range(N)])

with viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as vis:
    next_record = 0
    while vis.is_running():
        state_machine()
        cartesian_traj()
        joint_traj()
        globals.time = data.time
        # Оптимизированный основной цикл
        if flag_trajectory_generation == 1:
            data.time += model.opt.timestep
            data.qpos = np.concatenate((pos, quat, globals.q_ref))
            mj.mj_forward(model, data)
        else:
            globals.q_act = data.qpos[7:].copy()
            globals.u_act = data.qvel[6:].copy()

            joint_control(model,data)
            high_level_control()
            data.ctrl = globals.trq.copy()
            # Вычисляем CoM и ZMP только при необходимости
            if data.time >= next_record:
                com = compute_com(model, data)
                zmp_x, zmp_y = zmp_controller(model, data)
                time_history.append(data.time)
                com_history.append(com)
                zmp_history.append((zmp_x, zmp_y))
                next_record += record_interval
            
            mj.mj_step(model, data)
        #Camera setup
        vis.cam.lookat[:] = data.qpos[:3] 
        vis.cam.distance = 2.0
        vis.cam.elevation = -10
        vis.cam.azimuth = 90

        # Синхронизация с пониженной частотой
        if data.time % 0.1 < model.opt.timestep:  # Синхронизировать каждые 0.1 сек
            vis.sync()
# com_ref = globals.com_ref_traj[:len(com_history)]  # обрезаем до длины com_history
# com_actual = np.array(com_history)
# rmse_com = np.sqrt(np.mean(np.linalg.norm(com_actual[:, :2] - com_ref, axis=1)**2))
# print(f"RMSE по CoM: {rmse_com:.4f} м")

# with viewer.launch_passive(model, data) as vis:
#     while vis.is_running():
        
#         model.opt.timestep = 0.001
#         state_machine()
#         cartesian_traj()
#         joint_traj()
#         globals.time = data.time

#         if(flag_trajectory_generation==1): #kinematic mode
#             data.time +=model.opt.timestep
#             data.qpos = np.concatenate((pos,quat,globals.q_ref))
#             mj.mj_forward(model,data)
#         else: #dynamic mode
#             globals.q_act = data.qpos[7:].copy()
#             globals.u_act = data.qvel[6:].copy()
#             joint_control(model,data)
#             high_level_control()
#             data.ctrl = globals.trq.copy()
#             mj.mj_step(model, data)

# #         # # Calculate CoM and ZMP
# #         # com_data = compute_com(model, data)
# #         # zmp_data = zmp_controller(model, data)

# #         # # Append data to history
# #         # time_history.append(data.time)
# #         # com_history.append(com_data)
# #         #zmp_history.append(zmp_data)

# #         #Camera setup
# #         vis.cam.lookat[:] = data.qpos[:3] 
# #         vis.cam.distance = 2.0
# #         vis.cam.elevation = -10
# #         vis.cam.azimuth = 90

# #         com = compute_com(model, data)
# #         zmp_x, zmp_y = zmp_controller(model, data)
        
# #         # Append data to history
# #         time_history.append(data.time)
# #         com_history.append(com)
# #         zmp_history.append((zmp_x, zmp_y))
#         vis.sync()

print(f"Скорость симуляции: {data.time / (time.time() - start_time):.2f}x real-time")
print(f"Симулированное время: {data.time:.2f} сек")
print(f"Реальное время выполнения: {time.time() - start_time:.2f} сек")

df = pd.DataFrame({
    'time': time_history,
    'com_x': [com[0] for com in com_history],
    'com_y': [com[1] for com in com_history],
    'com_z': [com[2] for com in com_history],
    'zmp_x': [zmp[0] for zmp in zmp_history],
    'zmp_y': [zmp[1] for zmp in zmp_history]
})
df.to_csv('motion_data.csv', index=False)