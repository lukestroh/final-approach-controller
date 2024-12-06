#!/usr/bin/env python3
# from pybullet_tree_sim.

import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation

from zenlog import log
import pprint as pp

class FinalApproachController:
    def __init__(self, links, sensors) -> None:
        self.max_linear_speed = 0.01
        
        self.tf_base_to_tof0 = np.identity(4)
        self.tf_base_to_tof0[:3, 3] = links[sensors['tof0'].tf_frame]['tf_from_parent']
        self.tf_base_to_tof1 = np.identity(4)
        self.tf_base_to_tof1[:3, 3] = links[sensors['tof1'].tf_frame]['tf_from_parent']
        # self.tf_tof0_to_tof1 = mr.TransInv(self.tf_base_to_tof0) @ self.tf_base_to_tof1
        self.tf_base_to_cut_point = np.identity(4)
        self.tf_base_to_cut_point[:3, 3] = links['mock_pruner__tool0']['tf_from_parent']
        self.tf_tof0_to_cut_point = mr.TransInv(self.tf_base_to_tof0) @ self.tf_base_to_cut_point
        
        self.tf_tof0_to_tof1 = mr.TransInv(self.tf_base_to_tof0) @ self.tf_base_to_tof1
        
        # log.warn(pp.pformat(links))
        self.tf_base_to_cut_point = np.identity(4)  # we can get this from the robot class
        self.tf_base_to_cut_point[:3, 3] = links['mock_pruner__tool0']['tf_from_parent']
        return

    def get_cut_point_distance(self, data: dict) -> tuple:
        if not np.all(np.isclose(self.tf_tof0_to_tof1[:3, :3], np.identity(3), atol=1e-3)):
            raise ValueError("The two ToF frames are not aligned with each other.")
        tof0_to_tof1_pos_vec = self.tf_tof0_to_tof1[:3, 3]
        tof_linear_distance = np.linalg.norm(tof0_to_tof1_pos_vec)
        d0 = np.linalg.norm(data["tof0"]["data"][0, 0:3])
        d1 = np.linalg.norm(data["tof1"]["data"][0, 0:3])
        dist = (d0 + d1) / 2
        d_diff = d0 - d1

        theta = np.arctan(d_diff / tof_linear_distance)  # should return angle (-pi/2, pi/2)

        return dist, theta
        
        
    def get_twist(self, orientation, dist) -> np.ndarray:
        """Need to get the view matrix of the cut point to the rotation axis, normalize to the max lin speed"""
        log.info(self.tf_tof0_to_cut_point)
        if dist - self.tf_tof0_to_cut_point[2,3] <= 0:
            return np.zeros((6, 1))
        Kp = 1 / (dist - self.tf_tof0_to_cut_point[2,3])
        
        orientation = Rotation.from_quat(orientation).as_matrix()
        velocity = Kp * self.max_linear_speed * orientation @ [0,0,1]
        # tf_eef_to_world = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['mock_pruner__tool0']['id'])).reshape([4, 4], order="F")
        twist = np.zeros(6)
        twist[0:3] = velocity / np.linalg.norm(velocity) * self.max_linear_speed
        return twist