#!/usr/bin/env python3
from final_approach_controller.cut_point_rotate_axis_controller import CutPointRotateAxisController


class HybridController:
    def __init__(self, links: dict, sensors: dict) -> None:

        self.speed_scale = 0.3
        # self.links = links
        self.cut_point_rotate_axis_controller = CutPointRotateAxisController(links=links, sensors=sensors)

        return
