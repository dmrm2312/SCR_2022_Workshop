import logging
from sim import Simulation
from sim.inputs import KeyboardInput, FileInput
from sim.outputs import FileOutput
from sim.utils import time_str
from sim.Config import SimConfig
from sim.robot_def.WoodbotDef import WoodbotDef
from sim.robots.differential_drive.WoodbotHard import WoodbotHard

import ray


logging.basicConfig(level=logging.INFO)

s = Simulation(debug_mode=False)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('out/some.csv', loop=False)
i = KeyboardInput()

s.set_input(i)  # specify inputs to run
# Create instance of robots and corresponding omutput methods.
# each robot can have multiple output system
# Robot simulation using kinematics model

out_dir = 'out/'
webots_csv = FileOutput(out_dir+'webots'+time_str()+'.csv')      # save to test.csv at the same dir as the

# add robots to simulation

s.add_robot(WoodbotDef, WoodbotHard)


s.run(SimConfig(max_duration=10, dt=0.01, realtime=True, start_time=0, run_speed=1))  # run 10sec, at the end of run, automatically do outputs.

