import globals
import numpy as np
from parameters import pms
from set_command_step import set_command_step
#from zmp_controller import zmp_controller

def high_level_control():
    if(globals.prev_step < globals.step):
        globals.prev_step = globals.step

        vx = globals.xdot_ref
        vx_ = 1 #0.1 desired
        globals.xdot_ref = set_command_step(vx_, vx, pms.vx_min, pms.vx_max,pms.dvx)

        # vy = globals.ydot_ref
        # vy_ = 0.4 #desired
        # globals.ydot_ref = set_command_step(vy_, vy, pms.vy_min, pms.vy_max,pms.dvy)

