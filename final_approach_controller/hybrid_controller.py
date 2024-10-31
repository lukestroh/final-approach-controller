#!/usr/bin/env python3
from final_approach_controller.cut_point_rotate_axis_controller import CutPointRotateAxisController


class HybridController:
    def __init__(self) -> None:
        
        self.cut_point_rotate_axis_controller = CutPointRotateAxisController()
        
        return