#!/usr/bin/env python3
from final_approach_controller.hybrid_controller import HybridController

from pybullet_tree_sim.robot import Robot

import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation
import time

from zenlog import log
import pprint as pp


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
            if ord("j") in keys_pressed:
                self.run_cut_point_rotate_axis_controller(action)
            if ord('k') in keys_pressed:
                self.run_final_approach_controller(action)
            if ord('l') in keys_pressed:
                ...
            if ord(';') in keys_pressed:
                ...
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
                
                # log.error(f"Reading sensor: {sensor_name}")
                # log.error(f'sensor frame: {sensor.tf_frame}')
                # log.error(mr.TransInv(view_matrix))
                # log.warn(f'tf id: {self.links[sensor.tf_frame]["id"]}')
                # log.warn(f'tf from parent: {self.links[sensor.tf_frame]["tf_from_parent"]}')
                
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

    def run_cut_point_rotate_axis_controller(self, action) -> np.ndarray:
        if time.time() - self.debounce_time > 0.1:
            sensor_data = self.read_sensors()
            theta = self.hybrid_controller.cut_point_rotate_axis_controller.get_angle_from_perpendicular(
                data=sensor_data
            ) * -1 # TODO: Remove the negative sign, tofs are flipped
            # log.debug(f"theta: {theta * 180 / np.pi}")
            # log.debug(pp.pformat(sensor_data))
            # log.err(self.pbclient.getLinkState(self.robot, self.links['mock_pruner__tool0']['id']))
            # tf_world_to_eef = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['mock_pruner__tool0']['id'])).reshape([4, 4], order="F")
            # tf_world_to_mock_pruner_base = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['mock_pruner__base']['id'])).reshape([4,4], order="F")
            # tf_world_to_ur5e_tool0 = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['ur5e__tool0']['id'])).reshape([4,4], order="F")
            # log.debug(f'tf world to eef:\n{mr.TransInv(tf_world_to_eef)}')
            # log.debug(f'to base:\n{tf_world_to_mock_pruner_base}')
            # log.debug(f'to ur5e tool0:\n{tf_world_to_ur5e_tool0}')
            if not np.isclose(theta, 0.0, atol=np.radians(1)):
                rot_ax = self.hybrid_controller.cut_point_rotate_axis_controller.get_rotation_axis(data=sensor_data)
                # log.debug(f"Rotation axis:\n{rot_ax}")
                tf_cut_point_to_rot_axis = (
                    self.hybrid_controller.cut_point_rotate_axis_controller.get_cut_point_to_rot_axis_transform(
                        data=sensor_data, rot_ax=rot_ax
                    )
                )
                
                # Rotate around camera y-axis
                rot_ax_orientation = rot_ax[3:6, :].flatten()
                # log.debug(f"Rotating around axis: {rot_ax_orientation}")
                # DON't DELETE, use for test
                # R = np.identity(4)
                # R[:3, :3] = Rotation.from_euler("xyz", (rot_ax_orientation * np.array([0, theta, 0])), degrees=False).as_matrix()
                
                # log.debug(R)
                
                twist_mp_tool0_frame = self.hybrid_controller.cut_point_rotate_axis_controller.get_twist(tf_cut_point_to_rot_axis, theta)
                
                # log.warn(f'Twist:\n{twist_mp_tool0_frame}')
                
                tf_eef_to_world = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['mock_pruner__tool0']['id'])).reshape([4, 4], order="F")
                # tf_mock_pruner_base_to_world = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['mock_pruner__base']['id'])).reshape([4,4], order="F")
                # tf_ur5e_tool0_to_world = np.asarray(self.get_view_mat_by_id_at_curr_pose(id=self.links['ur5e__tool0']['id'])).reshape([4,4], order="F")
                # log.debug(f'tf world to eef:\n{mr.TransInv(tf_eef_to_world)}')
                # log.debug(f'to base:\n{mr.TransInv(tf_mock_pruner_base_to_world)}')
                # log.debug(f'to ur5e tool0:\n{mr.TransInv(tf_ur5e_tool0_to_world)}')
                # Rotate our vectors to the world frame
                linear_v_world_frame = np.linalg.inv(tf_eef_to_world[:3, :3]) @ twist_mp_tool0_frame[0:3, 0]
                angular_v_world_frame = np.linalg.inv(tf_eef_to_world[:3, :3]) @ twist_mp_tool0_frame[3:6, 0]
                # log.warn(f'linear_v_camera_frame: {twist_mp_tool0_frame[0:3,0]}')
                # log.warn(f'angular_v_camera_frame: {twist_mp_tool0_frame[3:6,0]}')
                # log.debug(f'linear_v_world_frame: {linear_v_world_frame}')
                # log.debug(f'angular_v_world_frame: {angular_v_world_frame}')
                
                action += np.concatenate((linear_v_world_frame, angular_v_world_frame)) * self.hybrid_controller.speed_scale
                # log.debug(f'action\n{action}')
            # log.warn(f"Theta: {theta}")
            else:
                log.info(f"Perpendicularity reached. Theta: {theta * 180 / np.pi}")
        else:
            log.debug(f"debounce time not reached")
        
        return action
        
    def run_final_approach_controller(self, action) -> np.ndarray:
        if time.time() - self.debounce_time > 0.1:
            sensor_data = self.read_sensors()
            dist, theta = self.hybrid_controller.final_approach_controller.get_cut_point_distance(data=sensor_data)
            # log.debug(f"Distance: {dist}, Theta: {theta * 180 / np.pi}")
            if np.isclose(theta, 0.0, atol=np.radians(1)):
                pose, orientation = self.get_current_pose(index=self.links['mock_pruner__tool0']['id'])
                twist = self.hybrid_controller.final_approach_controller.get_twist(orientation, dist)
                action += twist
        return action