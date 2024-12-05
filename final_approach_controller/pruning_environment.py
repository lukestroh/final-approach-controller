#!/usr/bin/activate
from final_approach_controller import CONFIG_PATH
from final_approach_controller.hybrid_controller import HybridController

from pybullet_tree_sim.pruning_environment import PruningEnv
from pybullet_tree_sim.utils.pyb_utils import PyBUtils
import pybullet_tree_sim.utils.yaml_utils as yutils

import numpy as np
from numpy.typing import ArrayLike
import os
import time

from zenlog import log


class FinalApproachControllerEnvironment(PruningEnv):

    _sensor_config_file = os.path.join(CONFIG_PATH, "sensors.yaml")

    def __init__(self, pbutils: PyBUtils, verbose: bool = True) -> None:
        super().__init__(pbutils=pbutils, verbose=True)
        return


def main():
    from pybullet_tree_sim.robot import Robot

    pbutils = PyBUtils(renders=False)
    fapc_env = FinalApproachControllerEnvironment(pbutils=pbutils, verbose=True)
    robot = Robot(pbclient=pbutils.pbclient, verbose=False)
    data = fapc_env.read_tof_sensors(robot=robot)

    return


if __name__ == "__main__":
    main()
