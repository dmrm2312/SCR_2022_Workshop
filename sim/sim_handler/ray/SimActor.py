import ray, time
import numpy as np
from sim.sim_handler.ray.autocounter import auto_count
from sim.robots import RobotBase
from sim.utils.tictoc import tictoc
import logging
ROUND = 2
DISPLAY_MAX_ARRAY_LENGTH = 5

@ray.remote
class SimActor:
    def __init__(self, robot, outputs):
        self.robot = robot
        self.outputs = outputs
        self.t = 0.0
        logging.getLogger().setLevel(logging.INFO)

    def step_forward(self, inpt, t_sys, DT):
        st = time.perf_counter()
        state = None
        observe = None
        info = None
        while np.round(self.t - t_sys, ROUND) < DT:
            self.robot.drive(inpt, self.t)
            observe = self.robot.sense()
            state = self.robot.observe_state()

            self.t = self.robot.clock(self.t)
            info = self.robot.info
        dt_actual = time.perf_counter() - st
        return observe, state, info, dt_actual, self.t

    def step(self, inpt, t_sys, DT):
        if np.round(self.t - t_sys, ROUND) >= DT:
            return
        #set input here
        inpt = self.robot.inpt.update(inpt)
        outpt, state, info, dt_actual, t = self.step_forward(inpt, t_sys, DT)

        for out in self.outputs:
            if out is None:
                continue
            out.process(state, inpt, outpt, t, info)
        disp = []
        for r in [inpt, state, outpt]:
            try:
                disp.append(round(r, ROUND))
            except TypeError:
                disp.append(r)

        if not self.robot.run.supress_info:
            logging.info("Name: {}, dt: {}, t: {}, inpt: {}, state: {}, output: {}, info: {}".format(
            self.robot.run.name,
            np.round(dt_actual, 5), np.round(t, ROUND), disp[0], disp[1], disp[2], info))

    def set_DT(self, DT):
        if self.robot.run.DT is None:
            self.robot.run.DT = DT

    def _call_func(self, name, *args, **kwargs):
        return getattr(self.robot, name)(*args, **kwargs)

    # for communication
    def _get_variable(self, name):
        return getattr(self.robot, name)


    def _set_variable(self, name, val):
        #eval("self.robot."+name+"= val")
        setattr(self.robot, name, val)
        #getattr(self.robot, name)

    def get_robot(self):
        return self.robot

    def make_outputs(self):
        # right now make outputs can be called only once
        for out in self.outputs:
            if out is None:
                continue
            out.make_output()


