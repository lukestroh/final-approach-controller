#!/usr/bin/env python3
from final_approach_controller.cut_point_rotate_axis_controller import CutPointRotateAxisController
from final_approach_controller.final_approach_controller import FinalApproachController

class HybridController:
    def __init__(self, links: dict, sensors: dict) -> None:

        self.speed_scale = 0.1
        # self.links = links
        self.cut_point_rotate_axis_controller = CutPointRotateAxisController(links=links, sensors=sensors)
        self.final_approach_controller = FinalApproachController(links=links, sensors=sensors)
        return
