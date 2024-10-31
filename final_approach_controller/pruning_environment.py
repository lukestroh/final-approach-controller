#!/usr/bin/activate
from final_approach_controller import CONFIG_PATH
from final_approach_controller.hybrid_controller import HybridController

from pybullet_tree_sim.camera import Camera
from pybullet_tree_sim.pruning_environment import PruningEnv
from pybullet_tree_sim.utils.pyb_utils import PyBUtils
from pybullet_tree_sim.time_of_flight import TimeOfFlight
import pybullet_tree_sim.utils.yaml_utils as yutils

from numpy.typing import ArrayLike
import os

class FinalApproachControllerEnvironment(PruningEnv):
    
    _sensor_config_file = os.path.join(CONFIG_PATH, "sensors.yaml")

    
    def __init__(self, pbutils: PyBUtils, load_robot: bool, robot_pos: ArrayLike, verbose: bool = True) -> None:
        super().__init__(pbutils=pbutils, load_robot=True, robot_pos=[0, 1, 0], verbose=True)
        
        self.hybrid_controller = HybridController()
        
        self.sensor_config = yutils.load_yaml(self._sensor_config_file)
        self.cameras = {}
        self.tofs = {}
        

        
        # Camera, tof setup
        for sensor, metadata in self.sensor_config["cameras"].items():
            for i in range(metadata["quantity"]):
                if sensor == "rgbd":
                    self.cameras[f"{metadata['type']}_{i}"] = Camera(sensor_name=metadata["type"])
                if sensor == "tof":
                    self.tofs[f"{metadata['type']}_{i}"] = TimeOfFlight(sensor_name=metadata["type"])
        print(self.cameras)
        print(self.tofs)
        return
        
        
def main():
    pbutils = PyBUtils(renders=False, cam_width=8, cam_height=8, dfov=65)
    fapc_env = FinalApproachControllerEnvironment(pbutils=pbutils, load_robot=True, robot_pos=[0, 1, 0], verbose=True)
    
    return
    
    
if __name__ == "__main__":
    main()