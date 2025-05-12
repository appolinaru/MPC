class parameters:
    def __init__(self):

        self.fsm_stand = 1
        self.fsm_stance = 2
        self.fsm_swing = 3
        self.t_stand = 0.1
        self.t_step = 0.15

        self.lz0 = -0.24864398730826576
        self.hcl = 0.075

        self.mass = 12.453
        self.gravity = 9.81

        self.vx_min = -2.0
        self.vx_max = 2.0
        self.dvx = 0.1

        # self.vy_min = -1.0
        # self.vy_max = 1.0
        # self.dvy = 0.05
        # Параметры походок
        self.gaits = {
            "trot": {
                "phase_offsets": [0, 0.5, 0.5, 0],  # Диагональные пары
                "stance_phase": [0.5, 0.5, 0.5, 0.5],  # 50% цикла в stance
                "description": "Диагональная походка (LF+RH → RF+LH)"
            },
            "pace": {
                "phase_offsets": [0, 0.5, 0, 0.5],  # Боковые пары
                "stance_phase": [0.5, 0.5, 0.5, 0.5],
                "description": "Боковая походка (LF+RF → LH+RH)"
            },
            "walk": {
                "phase_offsets": [0, 0.75, 0.5, 0.25],  # Последовательная активация
                "stance_phase": [0.75, 0.75, 0.75, 0.75],  # 75% цикла в stance
                "description": "Поочередный шаг (LF → RH → RF → LH)"
            },
            "bound": {
                "phase_offsets": [0, 0, 0.5, 0.5],  # Передние/задние пары
                "stance_phase": [0.5, 0.5, 0.5, 0.5],
                "description": "Прыжковая походка (передние → задние)"
            }
        }
        self.current_gait = "trot"

pms = parameters()