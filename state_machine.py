import globals
from parameters import pms



# Хорошая реализация trot(ориг)
# def state_machine():

#     time = globals.time
#     #print(globals.time)
#     #print(globals.fsm)
#     fsm_stand = pms.fsm_stand
#     fsm_stance = pms.fsm_stance
#     fsm_swing = pms.fsm_swing

#     t_stand = pms.t_stand
#     t_step = pms.t_step



#     for leg_num in range(4):
#         if(time >= globals.t_fsm[leg_num]+ t_stand and globals.fsm[leg_num]==fsm_stand):
#             if(leg_num == 0 or leg_num == 3):
#                 globals.fsm[leg_num] = fsm_swing
#                 globals.t_fsm[leg_num] = time

#                 globals.t_i[leg_num] = 0
#                 globals.t_f[leg_num] = t_step
#                 globals.lz_i[leg_num] = pms.lz0 
#                 globals.lz_f[leg_num] = pms.lz0 + pms.hcl

#             if(leg_num == 1 or leg_num == 2):
#                 globals.fsm[leg_num] = fsm_stance
#                 globals.t_fsm[leg_num] = time

#                 globals.t_i[leg_num] = 0
#                 globals.t_f[leg_num] = t_step
#                 globals.lz_i[leg_num] = pms.lz0
#                 globals.lz_f[leg_num] = pms.lz0

#         if(time >= globals.t_fsm[leg_num]+ t_step and globals.fsm[leg_num]==fsm_stance):
#             globals.fsm[leg_num] = fsm_swing
#             globals.t_fsm[leg_num] = time
#             globals.t_i[leg_num] = 0
#             globals.t_f[leg_num] = t_step
#             globals.lz_i[leg_num] = pms.lz0
#             globals.lz_f[leg_num] = pms.lz0+pms.hcl

#             globals.lx_i[leg_num] = -0.5*globals.xdot_ref * t_step
#             globals.lx_f[leg_num] = 0.5*globals.xdot_ref * t_step
#             globals.ly_i[leg_num] = -0.5*globals.ydot_ref * t_step
#             globals.ly_f[leg_num] = 0.5*globals.ydot_ref * t_step

#         if(time >= globals.t_fsm[leg_num]+ t_step and globals.fsm[leg_num]==fsm_swing):
#             if (leg_num == 0 or leg_num == 1):
#                 globals.step+=1
#             globals.fsm[leg_num] = fsm_stance
#             globals.t_fsm[leg_num] = time
#             globals.t_i[leg_num] = 0
#             globals.t_f[leg_num] = t_step
#             globals.lz_i[leg_num] = pms.lz0
#             globals.lz_f[leg_num] = pms.lz0

#             globals.lx_i[leg_num] = 0.5*globals.xdot_ref * t_step
#             globals.lx_f[leg_num] = -0.5*globals.xdot_ref * t_step
#             globals.ly_i[leg_num] = 0.5*globals.ydot_ref * t_step
#             globals.ly_f[leg_num] = -0.5*globals.ydot_ref * t_step

def _init_swing_leg(leg_num):
    """Инициализация параметров для ноги в swing-фазе"""
    globals.lz_i[leg_num] = pms.lz0
    globals.lz_f[leg_num] = pms.lz0 + pms.hcl
    globals.lx_i[leg_num] = -0.5 * globals.xdot_ref * pms.t_step
    globals.lx_f[leg_num] = 0.5 * globals.xdot_ref * pms.t_step
    globals.ly_i[leg_num] = 0
    globals.ly_f[leg_num] = 0

def _init_stance_leg(leg_num):
    """Инициализация параметров для ноги в stance-фазе"""
    globals.lz_i[leg_num] = pms.lz0
    globals.lz_f[leg_num] = pms.lz0
    globals.lx_i[leg_num] = 0
    globals.lx_f[leg_num] = 0
    globals.ly_i[leg_num] = 0
    globals.ly_f[leg_num] = 0

def _ensure_walk_stability(current_swing_leg):
    """Для walk гарантируем, что всегда 3 ноги на земле"""
    if pms.current_gait == "walk":
        for leg_num in range(4):
            if leg_num != current_swing_leg and globals.fsm[leg_num] != pms.fsm_stance:
                globals.fsm[leg_num] = pms.fsm_stance
                _init_stance_leg(leg_num)

def state_machine():
    time = globals.time
    fsm_stand = pms.fsm_stand
    fsm_stance = pms.fsm_stance
    fsm_swing = pms.fsm_swing
    t_step = pms.t_step
    gait = pms.gaits[pms.current_gait]

    # Инициализация при старте
    if all(state == fsm_stand for state in globals.fsm):
        for leg_num in range(4):
            if time >= pms.t_stand:
                phase = (time / t_step + gait["phase_offsets"][leg_num]) % 1.0
                globals.fsm[leg_num] = fsm_swing if phase >= gait["stance_phase"][leg_num] else fsm_stance
                globals.t_fsm[leg_num] = time
                globals.t_i[leg_num] = 0
                globals.t_f[leg_num] = t_step
                
                if globals.fsm[leg_num] == fsm_swing:
                    _init_swing_leg(leg_num)
                else:
                    _init_stance_leg(leg_num)

    # Основной цикл переключения состояний
    for leg_num in range(4):
        if globals.fsm[leg_num] in [fsm_stance, fsm_swing] and time >= globals.t_fsm[leg_num] + t_step:
            new_state = fsm_swing if globals.fsm[leg_num] == fsm_stance else fsm_stance
            globals.fsm[leg_num] = new_state
            globals.t_fsm[leg_num] = time
            
            if new_state == fsm_swing:
                _init_swing_leg(leg_num)
            else:
                _init_stance_leg(leg_num)

            if new_state == fsm_swing and leg_num in [0, 1]:
                globals.step += 1
