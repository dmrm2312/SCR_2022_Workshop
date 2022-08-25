from sim.robot_def.webots import DifferentialDriveDef
# sensor names and definitoins
SENSOR = {
    "bumper_left": bool, "bumper_right": bool,
    "cliff_left": float, "cliff_front_left": float,
    "cliff_front_right": float, "cliff_right": float
}

# Driver names and definitions
# pos = inf means velocity control
DRIVE = {
    "left wheel motor": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
    "right wheel motor": dict(pos=float('inf'), vel=float, acc=float, on=bool, pid=list),
}


class CreateDef(DifferentialDriveDef):
    def __init__(self, *args, **kwargs):
         super().__init__(radius=0.031, length=0.135878*2, max_vel=16, *args, **kwargs)

    def define(self, *args, **kwargs):
        super().define(DRIVE, SENSOR)
        self.name = 'Create'

