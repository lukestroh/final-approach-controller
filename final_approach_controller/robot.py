#!/usr/bin/env python3
from final_approach_controller.hybrid_controller import HybridController

from pybullet_tree_sim.robot import Robot

import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation
import time

from zenlog import log


class PruningRobot(Robot):
    def __init__(self, pbclient, position, orientation, randomize_pose=False, verbose=True) -> None:
        super().__init__(
            pbclient=pbclient,
            position=position,
            orientation=orientation,
            randomize_pose=randomize_pose,
            verbose=verbose,
        )
        self.hybrid_controller = HybridController(links=self.links, sensors=self.sensors)

        self.debounce_time = time.time()

        return

    def get_key_controller_action(self, keys_pressed: list) -> np.ndarray:
        action = np.zeros(6)
        if keys_pressed:
            if ord("o") in keys_pressed:
                if time.time() - self.debounce_time > 0.1:
                    sensor_data = self.read_sensors()
                    theta = self.hybrid_controller.cut_point_rotate_axis_controller.get_angle_from_perpendicular(
                        data=sensor_data
                    )
                    if not np.isclose(theta, 0.0, atol=np.radians(2)):
                        rot_ax = self.hybrid_controller.cut_point_rotate_axis_controller.get_rotation_axis(data=sensor_data)
    
                        tf_cut_point_to_rot_axis = (
                            self.hybrid_controller.cut_point_rotate_axis_controller.get_cut_point_to_rot_axis_transform(
                                data=sensor_data, rot_ax=rot_ax
                            )
                        )
                        
                        # Rotate around camera y-axis
                        rot_ax_orientation = rot_ax[3:6, :].flatten()
                        log.info(f"Rotating around axis: {rot_ax_orientation}")
                        # R = np.identity(4)
                        # R[:3, :3] = Rotation.from_euler("xyz", (rot_ax_orientation * np.array([0, theta, 0])), degrees=False).as_matrix()
                        
                        # log.debug(R)
                        
                        twist_mp_tool0_frame = self.hybrid_controller.cut_point_rotate_axis_controller.get_twist(tf_cut_point_to_rot_axis, theta)
                        
                        log.warn(f'Twist: {twist_mp_tool0_frame}')
                        
                        tf_world_to_eef = np.asarray(self.get_eef_view_mat_at_curr_pose()).reshape([4, 4], order="F")
                        
                        log.err(mr.TransInv(tf_world_to_eef))
                        log.err(np.concatenate((twist_mp_tool0_frame[3:6, 0], [1])))
                        linear_v_world_frame = mr.TransInv(tf_world_to_eef) @ np.concatenate((twist_mp_tool0_frame[0:3, 0], [1]))
                        angular_v_world_frame = mr.TransInv(tf_world_to_eef) @ np.concatenate((twist_mp_tool0_frame[3:6, 0], [1]))
                        
                        
                        action = np.concatenate((linear_v_world_frame[0:3], angular_v_world_frame[0:3]))
                        log.debug(action)
                    # log.warn(f"Theta: {theta}")
                    else:
                        log.info(f"Perpendicularity reached. Theta: {theta * 180 / np.pi}")
                else:
                    log.debug(f"debounce time not reached")

        return action

    def read_sensors(self):
        sensor_data = {}
        for sensor_name, sensor in self.sensors.items():
            if sensor_name.startswith("tof"):
                view_matrix = self.get_view_mat_at_curr_pose(camera=sensor)
                rgb, depth = self.get_rgbd_at_cur_pose(camera=sensor, type="sensor", view_matrix=view_matrix)
                view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
                depth = depth.reshape((sensor.depth_width * sensor.depth_height, 1), order="F")

                camera_points = self.deproject_pixels_to_points(
                    sensor=sensor, data=depth, view_matrix=view_matrix, return_frame="sensor"
                )

                sensor_data.update(
                    {
                        sensor_name: {
                            "data": camera_points,
                            "tf_frame": sensor.tf_frame,
                            "view_matrix": view_matrix,
                            "sensor": sensor,  # doing: getting cut point to eef transform
                        }
                    }
                )
        # plot.debug_sensor_world_data(sensor_data)
        self.debounce_time = time.time()
        return sensor_data
