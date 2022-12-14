import logging, os, ray
import time

from sim import Simulation
from sim.inputs import FileCsvInput
from sim.outputs import FileCsvOutput
from sim.robots.bind_robot import bind_robot
from sim.robots.scaler_leg.ScalerManipulatorDef import ScalerManipulator
from sim.robots.scaler_leg.ScalarHard import ScalerHard
from sim.device.kinematic_model.KinematicModel import KinematicModel
from sim.robots.scaler_leg.Pybullet import Pybullet

ray.init(local_mode=True)

pybullet_robots = bind_robot(ScalerManipulator, (ScalerHard, '/dev/ttyUSB0', 2))
#pybullet_robots = bind_robot(ScalerManipulator, Pybullet)
pybullet_robots.inpt.set(dict(x=0.0,y=0,z=-0.3))
pybullet_robots.run.DT = 0.01
pybullet_robots.init()
N = 10
for n in range(N):
    pybullet_robots.drive(pybullet_robots.inpt, 0)
    pybullet_robots.clock(0)
    s=pybullet_robots.sense()
    pybullet_robots.observe_state()
    print(s)
    time.sleep(1)
pybullet_robots.close()
pass
