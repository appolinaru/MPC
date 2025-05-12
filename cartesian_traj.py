import globals
from quintic_poly import quintic_poly
from parameters import pms
import numpy as np

def cartesian_traj():
    #set lz_ref, lx_ref, ly_ref

    time = globals.time

    t_i = globals.t_i
    t_f = globals.t_f
    lz_i = globals.lz_i
    lz_f = globals.lz_f
    lx_i = globals.lx_i
    lx_f = globals.lx_f
    ly_i = globals.ly_i
    ly_f = globals.ly_f


    t_fsm = globals.t_fsm
    fsm = globals.fsm

    fsm_stance = pms.fsm_stance
    fsm_swing = pms.fsm_swing
    fsm_stand = pms.fsm_stand

    t_step = pms.t_step

    lx_ref, ly_ref, lz_ref, lxdot_ref, lydot_ref, lzdot_ref = (np.zeros(4) for _ in range(6))

    # === Центр масс (CoM) траектория ===
    # Простой вариант: CoM движется вперёд со скоростью xdot_ref
    globals.com_x_ref = globals.xdot_ref * time
    globals.com_xdot_ref = globals.xdot_ref

    # Пока не двигаем по Y
    globals.com_y_ref = 0.0
    globals.com_ydot_ref = 0.0
    
    for leg_no in range(4):

        if (fsm[leg_no]==fsm_stand or fsm[leg_no]==fsm_stance):
            #lz_ref from quintic_poly
            lz_ref[leg_no],lzdot_ref[leg_no],_ = quintic_poly(time-t_fsm[leg_no],t_i[leg_no],t_f[leg_no],lz_i[leg_no],lz_f[leg_no])


        if (fsm[leg_no]==fsm_swing):
            #lz_ref from quintic_pol
            lx_ref[leg_no],lxdot_ref[leg_no],_ = quintic_poly(time-t_fsm[leg_no],t_i[leg_no],t_f[leg_no],lx_i[leg_no],lx_f[leg_no])
            ly_ref[leg_no],lydot_ref[leg_no],_ = quintic_poly(time-t_fsm[leg_no],t_i[leg_no],t_f[leg_no],ly_i[leg_no],ly_f[leg_no])
            
            if (time-t_fsm[leg_no]<=0.5*t_step):
                lz_ref[leg_no],lzdot_ref[leg_no],_ = quintic_poly(time-t_fsm[leg_no],t_i[leg_no],t_f[leg_no]/2,lz_i[leg_no],lz_f[leg_no])
            else:
                lz_ref[leg_no],lzdot_ref[leg_no],_ = quintic_poly(time-t_fsm[leg_no],t_f[leg_no]/2,t_f[leg_no],lz_f[leg_no],lz_i[leg_no])

        globals.lz_ref[leg_no] = lz_ref[leg_no]
        globals.lzdot_ref[leg_no] = lzdot_ref[leg_no]

        globals.lx_ref[leg_no] = lx_ref[leg_no]
        globals.lxdot_ref[leg_no] = lxdot_ref[leg_no]

        globals.ly_ref[leg_no] = ly_ref[leg_no]
        globals.lydot_ref[leg_no] = lydot_ref[leg_no]
# import globals
# from quintic_poly import quintic_poly
# from parameters import pms
# import numpy as np

# def cartesian_traj():
#     time = globals.time
#     gait = pms.gaits[pms.current_gait]
    
#     for leg_no in range(4):
#         t_elapsed = time - globals.t_fsm[leg_no]
#         phase_progress = t_elapsed / globals.t_f[leg_no] if globals.t_f[leg_no] > 0 else 0

#         # Инициализация референсных значений
#         lx_ref = ly_ref = lz_ref = 0
#         lxdot_ref = lydot_ref = lzdot_ref = 0

#         if globals.fsm[leg_no] == pms.fsm_swing:
#             # 1. X-координата (движение вперед/назад)
#             lx_ref, lxdot_ref, _ = quintic_poly(
#                 t_elapsed, 
#                 globals.t_i[leg_no], 
#                 globals.t_f[leg_no],
#                 globals.lx_i[leg_no], 
#                 globals.lx_f[leg_no]
#             )

#             # 2. Y-координата (боковое движение)
#             ly_ref, lydot_ref, _ = quintic_poly(
#                 t_elapsed,
#                 globals.t_i[leg_no],
#                 globals.t_f[leg_no],
#                 globals.ly_i[leg_no],
#                 globals.ly_f[leg_no]
#             )

#             # 3. Z-координата (подъем/опускание)
#             if phase_progress <= 0.5:  # Фаза подъема
#                 lz_ref, lzdot_ref, _ = quintic_poly(
#                     t_elapsed,
#                     globals.t_i[leg_no],
#                     globals.t_f[leg_no]/2,
#                     globals.lz_i[leg_no],
#                     globals.lz_f[leg_no]
#                 )
#             else:  # Фаза опускания
#                 lz_ref, lzdot_ref, _ = quintic_poly(
#                     t_elapsed - globals.t_f[leg_no]/2,
#                     0,
#                     globals.t_f[leg_no]/2,
#                     globals.lz_f[leg_no],
#                     globals.lz_i[leg_no]
#                 )

#         elif globals.fsm[leg_no] in [pms.fsm_stand, pms.fsm_stance]:
#             # Для stance/stand - нога остается на месте
#             lx_ref, lxdot_ref, _ = quintic_poly(
#                 t_elapsed,
#                 globals.t_i[leg_no],
#                 globals.t_f[leg_no],
#                 globals.lx_i[leg_no],
#                 globals.lx_i[leg_no]  # Конечная = начальной
#             )
            
#             ly_ref, lydot_ref, _ = quintic_poly(
#                 t_elapsed,
#                 globals.t_i[leg_no],
#                 globals.t_f[leg_no],
#                 globals.ly_i[leg_no],
#                 globals.ly_i[leg_no]
#             )
            
#             lz_ref, lzdot_ref, _ = quintic_poly(
#                 t_elapsed,
#                 globals.t_i[leg_no],
#                 globals.t_f[leg_no],
#                 globals.lz_i[leg_no],
#                 globals.lz_i[leg_no]
#             )

#         # Записываем результаты в глобальные переменные
#         globals.lx_ref[leg_no] = lx_ref
#         globals.ly_ref[leg_no] = ly_ref
#         globals.lz_ref[leg_no] = lz_ref
#         globals.lxdot_ref[leg_no] = lxdot_ref
#         globals.lydot_ref[leg_no] = lydot_ref
#         globals.lzdot_ref[leg_no] = lzdot_ref