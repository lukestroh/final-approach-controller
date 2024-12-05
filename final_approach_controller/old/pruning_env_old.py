#!/usr/bin/activate
from final_approach_controller import CONFIG_PATH
from final_approach_controller.hybrid_controller import HybridController

from pybullet_tree_sim.camera import Camera
from pybullet_tree_sim.pruning_environment import PruningEnv
from pybullet_tree_sim.utils.pyb_utils import PyBUtils
from pybullet_tree_sim.time_of_flight import TimeOfFlight
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

        self.hybrid_controller = HybridController()

        # Load sensor config
        self.sensor_config = yutils.load_yaml(self._sensor_config_file)
        if self.sensor_config is None:
            raise ValueError(f"Failed to load sensor config from {self._sensor_config_file}")

        # Sensors
        self.cameras = {}
        self.tofs = {}

        # Sensor data
        self.sensor_frame_data = {
            "cameras": {},
            "tofs": {},
        }
        self.eef_frame_data = {
            "cameras": {},
            "tofs": {},
        }
        self.world_frame_data = {
            "cameras": {},
            "tofs": {},
        }

        # Camera, tof setup
        for camera_type, metadata in self.sensor_config["cameras"].items():
            for i, frame in enumerate(metadata["tf_frames"]):
                log.info(f"Setting up sensor {metadata['name']}_{i}")
                if camera_type == "rgbd":
                    self.cameras[f"{metadata['name']}_{i}"] = Camera(
                        pbutils=self.pbutils,
                        sensor_name=metadata["name"],
                    )
                    self.cameras[f"{metadata['name']}_{i}"].pan = metadata["pan"]
                    self.cameras[f"{metadata['name']}_{i}"].tilt = metadata["tilt"]
                    self.cameras[f"{metadata['name']}_{i}"].tf_frame = frame

                    ##############################################################################################
                    # TODOOOOOO
                    # #########################################################################
                    self.cameras[f"{metadata['name']}_{i}"].tf_frame_index = ...  # TODO: Claire's code has this
                    self.cameras[f"{metadata['name']}_{i}"].tf_frame_to_eef_mat = ...

                    self.sensor_frame_data["cameras"][f"{metadata['name']}_{i}"] = []

                elif camera_type == "tof":
                    self.tofs[f"{metadata['name']}_{i}"] = TimeOfFlight(
                        pbutils=self.pbutils,
                        sensor_name=metadata["name"],
                    )
                    self.tofs[f"{metadata['name']}_{i}"].pan = metadata["pan"]
                    self.tofs[f"{metadata['name']}_{i}"].tilt = metadata["tilt"]
                    self.tofs[f"{metadata['name']}_{i}"].tf_frame = frame

                    ##############################################################################################
                    # TODOOOOOO
                    # #########################################################################
                    self.tofs[f"{metadata['name']}_{i}"].tf_frame_index = ...  # TODO: Claire's code has this'
                    self.tofs[f"{metadata['name']}_{i}"].tf_frame_to_eef_mat = ...

                    self.sensor_frame_data["tofs"][f"{metadata['name']}_{i}"] = []

        self.last_button_push_time = 0
        return

    def get_tof_observation(self):
        """Read from the ToF sensors and return the observation in the camera frame"""
        for i, tof in enumerate(self.tofs.values()):

            # Get view and projection matrices
            sensor_view_matrix = self.robot.get_view_mat_at_curr_pose(camera=tof)
            # This gets in eef frame, convert!!!

            # log.warning(f"button p pressed")
            rgb, depth = self.pbutils.get_rgbd_at_cur_pose(camera=tof, type="robot", view_matrix=sensor_view_matrix)
            # log.debug(f'depth:\n{depth}')

            depth = -1 * depth.reshape((tof.depth_width * tof.depth_height, 1), order="F")
            points = self.deproject_pixels_to_points(
                camera=tof,
                data=depth,
                view_matrix=np.asarray(sensor_view_matrix).reshape([4, 4], order="F"),
                return_frame="camera",
            )

            ### TODO: Change each camera frame to EEF frame.

            self.sensor_frame_data["tofs"][f"{tof.name}_{i}"].append(points)  # TODO: Just save name with ID value

        return

    def debug_plots(self, camera, data, cam_coords, world_coords, view_matrix):
        import plotly.graph_objects as go
        import modern_robotics as mr

        hovertemplate = "id: %{id}<br>x: %{x}<br>y: %{y}<br>z: %{z}<extra></extra>"

        _data = data.reshape([camera.depth_width, camera.depth_height], order="F")
        _data = _data.reshape((camera.depth_width * camera.depth_height, 1), order="C")
        fig = go.Figure(
            data=[
                go.Scatter3d(
                    x=list(range(8)) * 8,
                    y=np.array([list(range(8))] * 8).T.flatten(order="C"),
                    z=_data.flatten(order="C"),
                    mode="markers",
                    ids=np.array([f"{i}" for i in range(camera.depth_width * camera.depth_height)])
                    .reshape((8, 8), order="C")
                    .flatten(order="F"),
                    hovertemplate=hovertemplate,
                )
            ]
        )
        fig.update_layout(
            title="Pixel Coordinates",
            scene=dict(
                aspectmode="cube",
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    center=dict(x=0, y=0, z=0),
                    eye=dict(x=-1.25, y=-1.25, z=1.25),
                ),
            ),
        )
        fig.show()
        fig = go.Figure(
            data=[
                go.Scatter3d(
                    x=camera.depth_film_coords[:, 0],
                    y=camera.depth_film_coords[:, 1],
                    z=data.flatten(order="F"),
                    mode="markers",
                    ids=[f"{i}" for i in range(camera.depth_width * camera.depth_height)],
                    hovertemplate=hovertemplate,
                )
            ]
        )
        fig.update_layout(
            title="Film Coordinates",
            scene=dict(
                aspectmode="cube",
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    center=dict(x=0, y=0, z=0),
                    eye=dict(x=-1.25, y=-1.25, z=1.25),
                ),
            ),
        )
        fig.show()
        fig = go.Figure(
            data=[
                go.Scatter3d(
                    x=cam_coords[:, 0],
                    y=cam_coords[:, 1],
                    z=cam_coords[:, 2],
                    mode="markers",
                    ids=[
                        f"{i}" for i in range(camera.depth_width * camera.depth_height)
                    ],  # TODO: change these to be counted as in image space
                    hovertemplate=hovertemplate,
                )
            ]
        )  # reverse sign of z to match world coords
        fig.update_layout(
            title="Camera Coordinates",
            scene=dict(
                aspectmode="cube",
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    center=dict(x=0, y=0, z=0),
                    eye=dict(x=-1.25, y=-1.25, z=1.25),
                ),
            ),
        )
        fig.show()
        fig = go.Figure(
            data=[
                go.Scatter3d(
                    x=world_coords[:, 0],
                    y=world_coords[:, 1],
                    z=world_coords[:, 2],
                    name="tof_data",
                    mode="markers",
                    marker=dict(size=2),
                    ids=np.array([f"{i}" for i in range(camera.depth_width * camera.depth_height)]),
                    hovertemplate=hovertemplate,
                )
            ]
        )
        inv_view_matrix = mr.TransInv(view_matrix)
        fig.add_trace(
            go.Scatter3d(
                x=[inv_view_matrix[0, 3]],
                y=[inv_view_matrix[1, 3]],
                z=[inv_view_matrix[2, 3]],
                mode="markers",
                name="camera_origin",
                marker=dict(size=5),
            )
        )
        fig.update_layout(
            title="World Coordinates",
            scene=dict(
                aspectmode="cube",
                xaxis=dict(range=[-1.0, 1.0]),
                yaxis=dict(range=[-1.0, 1.0]),
                zaxis=dict(range=[-0.0, 2.1]),
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    center=dict(x=0, y=0, z=0),
                    eye=dict(x=-1.5, y=-1.5, z=1.5),
                ),
            ),
        )
        fig.show()

        # log.warn(f"view_matrix: {view_matrix}")
        # log.warn(f"inv_view_matrix: {inv_view_matrix}")
        return


def main():
    pbutils = PyBUtils(renders=False)
    fapc_env = FinalApproachControllerEnvironment(pbutils=pbutils, load_robot=True, robot_pos=[0, 1, 0], verbose=True)
    fapc_env.get_tof_observation()
    print(fapc_env.sensor_data)
    # print(fapc_env.cameras)
    # print(fapc_env.tofs)
    return


if __name__ == "__main__":
    main()
