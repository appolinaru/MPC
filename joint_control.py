import globals
import numpy as np
from parameters import pms
from jac_end_effector_leg import jac_end_effector_leg
from zmp_controller import zmp_controller
from mpc_controller import solve_mpc

globals.last_F_com = np.zeros(3)  # Инициализация

def joint_control(model,data):

    fsm = globals.fsm
    fsm_stance = pms.fsm_stance
    fsm_swing = pms.fsm_swing
    fsm_stand = pms.fsm_stand

    q_act = globals.q_act
    u_act = globals.u_act
    q_ref = globals.q_ref
    u_ref = globals.u_ref

    # Получаем текущее положение CoM
    #com_x, com_y = zmp_controller(globals.q_act)
    com_x, com_y = zmp_controller(model,data)
    # Ошибка по положению CoM относительно желаемой траектории
    com =(com_x,com_y,0)
    if globals.step % 3 == 0:
        F_com = solve_mpc(com, globals.com_ref_traj, model, data)
        if F_com is not None:
            globals.last_F_com = F_com
    else:
        F_com = globals.last_F_com



    for leg_no in range(4):
        if (fsm[leg_no]==fsm_stand):
            #pd control trq = -kp*(q_act-q_ref)-kd*(u_act - u_ref)
            globals.trq[3*leg_no] = - 100*(q_act[3*leg_no]-q_ref[3*leg_no])-10*(u_act[3*leg_no] - u_ref[3*leg_no])
            globals.trq[3*leg_no + 1] = -100*(q_act[3*leg_no +1]-q_ref[3*leg_no + 1])-10*(u_act[3*leg_no + 1] - u_ref[3*leg_no + 1])
            globals.trq[3*leg_no+2] =  -100*(q_act[3*leg_no+2] -q_ref[3*leg_no+2] )-10*(u_act[3*leg_no+2] -u_ref[3*leg_no+2] )

        if (fsm[leg_no]==fsm_swing):
            #pd control trq = -kp*(q_act-q_ref)-kd*(u_act - u_ref)
            globals.trq[3*leg_no] = -100*(q_act[3*leg_no]-q_ref[3*leg_no])-10*(u_act[3*leg_no] - u_ref[3*leg_no])
            globals.trq[3*leg_no + 1] = -100*(q_act[3*leg_no +1]-q_ref[3*leg_no + 1])-10*(u_act[3*leg_no + 1] - u_ref[3*leg_no + 1])
            globals.trq[3*leg_no+2] =  -100*(q_act[3*leg_no+2] -q_ref[3*leg_no+2] )-10*(u_act[3*leg_no+2] -u_ref[3*leg_no+2] )

        if (fsm[leg_no]==fsm_stance):
            F = np.array([0,0,0.5*pms.mass*pms.gravity])
            q_leg = np.array([q_ref[3*leg_no],q_ref[3*leg_no+1],q_ref[3*leg_no+2]])
            J = jac_end_effector_leg(q_leg,leg_no)
            trq_grav = -J.T@F
            #trq_grav = np.zeros(3)
            
            F_total = F + F_com  # добавляем корректирующую силу к силе тяжести
            J = jac_end_effector_leg(q_leg,leg_no)
            trq_grav = -J.T @ F_total

            #pd control trq = -kp*(q_act-q_ref)-kd*(u_act - u_ref)
            globals.trq[3*leg_no] =trq_grav[0] -100*(q_act[3*leg_no]-q_ref[3*leg_no])-10*(u_act[3*leg_no] - u_ref[3*leg_no])
            globals.trq[3*leg_no + 1] =trq_grav[1] -100*(q_act[3*leg_no +1]-q_ref[3*leg_no + 1])-10*(u_act[3*leg_no + 1] - u_ref[3*leg_no + 1])
            globals.trq[3*leg_no + 2] =trq_grav[2]  -100*(q_act[3*leg_no+2] -q_ref[3*leg_no+2] )-10*(u_act[3*leg_no+2] -u_ref[3*leg_no+2] )       

