#!/usr/bin/env python3
import modern_robotics as mr
import numpy as np
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation
import time


class CutPointRotateAxisController():
    def __init__(self) -> None:
        self.num_sensors = 2 # TODO: Get from config.
        self.speed_scale = 1.0 # TODO: Get from config.
        return

    def get_angle_from_perpendicular(self, data: dict) -> float:
        # Make sure the two TOF frames are aligned with the end effector frame # TODO: This should be checked in init
        M = mr.TransInv(data['tf_tof0_to_eef']) @ data['tf_tof1_to_eef']
        # print(M)
        if not np.all(np.isclose(M[:3, :3], np.identity(3))):
            raise ValueError("The two TOF frames are not aligned with the end effector frame.")
    
        # Calculate the distance between the two TOF sensors # TODO: save this info in class attr
        tof_tof0_to_tof1_pos_vec = M[:3, 3]
        # print(tof_linear_pos_vec)
        tof_linear_distance = np.linalg.norm(tof_tof0_to_tof1_pos_vec)
        # print(x_diff)
        d0 = np.linalg.norm(data['reading0'] - data['tof0_pos'])
        d1 = np.linalg.norm(data['reading1'] - data['tof1_pos'])
        d_diff = d0 - d1
    
        theta = np.arctan(d_diff / tof_linear_distance) # should return angle [-pi/2, pi/2]
    
        return theta

    def get_rotation_axis(self, data: dict):
        """Get the rotation axis in the camera frame."""
        rotation_point = np.mean([data['reading0'], data['reading1']], axis=0)
        rotation_axis = np.zeros((6,1), dtype=float)
        rotation_axis[0:3, :] = rotation_point.reshape(3,1)
        rotation_axis[3:6, :] = np.cross(data['reading0'] - data['tof0_pos'], data['reading1'] - data['tof1_pos']).reshape(3,1)
        # If the cross product is zero then the two vectors are parallel, so we can just choose the  y-axis (camera frame)
        if np.linalg.norm(rotation_axis[3:6, :]) != 0:
            rotation_axis[3:6, :] = rotation_axis[3:6, :] / np.linalg.norm(rotation_axis[3:6, :])
        else:
            rotation_axis[3:6, :] = np.array([0, 1, 0]).reshape(3,1)
        return rotation_axis

    def get_cut_point_to_rot_axis_transform(self, data: dict, rot_ax: np.ndarray):
        """TODO: replace with actual transform from end effector to cut point."""
        tf_axis_to_eef = np.identity(4)
        tf_axis_to_eef[:3, 3] = rot_ax[:3, 0]
    
        tf_cut_point_to_rot_axis = data['tf_eef_to_cut_point'] @ tf_axis_to_eef
        return tf_cut_point_to_rot_axis

    def get_twist(self, tf_cut_point_to_rot_axis: np.ndarray, angle_from_perpendicular: float):
    
        # TODO: put this in class attr
        max_angular_speed = np.pi / 2
        K_p = 1 / max_angular_speed
        
        # We are in the camera frame, so the angular velocity is along the y-axis, which points down
        angular_velocity = [0, K_p * angle_from_perpendicular, 0]
        linear_velocity = np.cross(angular_velocity, tf_cut_point_to_rot_axis[:3, 3])
        # print(f"Cut point to rotation axis transform:\n{tf_cut_point_to_rot_axis}")
        # print(f"Linear velocity:\n{linear_velocity}")
        # print(f"Angular velocity:\n{angular_velocity}")
        twist = np.concatenate((linear_velocity, angular_velocity), axis=0).reshape(6,1)
        print(twist)
        return linear_velocity, angular_velocity

    def plot_3d_coordinate_frame(self, fig, position: np.ndarray, orientation: np.ndarray, name: str = "origin"):
        # if name in self.coordinate_frames:
        #     raise ValueError(f"Coordinate frame with name '{name}' already exists.")
        # else:
        #     self.coordinate_frames = [name]
        zoom_scale = 0.05
        cone_scale = 0.25
        axes = {
            "x-axis": {
                "x": np.array([position[0], position[0] + zoom_scale]),
                "y": np.array([position[1], position[1] + 0]),
                "z": np.array([position[2], position[2] + 0]),
                "u": np.array([zoom_scale * cone_scale]),
                "v": np.array([0]),
                "w": np.array([0]),
                "color": "red"
            },
            "y-axis": {
                "x": np.array([position[0], position[0] + 0]),
                "y": np.array([position[1], position[1] + zoom_scale]),
                "z": np.array([position[2], position[2] + 0]),
                "u": np.array([0]),
                "v": np.array([zoom_scale * cone_scale]),
                "w": np.array([0]),
                "color": "green"
            },
            "z-axis": {
                "x": np.array([position[0], position[0] + 0]),
                "y": np.array([position[1], position[1] + 0]),
                "z": np.array([position[2], position[2] + zoom_scale]),
                "u": np.array([0]),
                "v": np.array([0]),
                "w": np.array([zoom_scale * cone_scale]),
                "color": "blue"
            }
        }
    
        for axis, val in axes.items():
            fig.add_trace(
                go.Scatter3d(
                    x=axes[axis]["x"],
                    y=axes[axis]["y"],
                    z=axes[axis]["z"],
                    name=f"{name}_{axis}",
                    mode="lines",
                    line=dict(
                        width=3,
                        color=axes[axis]["color"]
                    ),
                    showlegend=True,
                    legendgroup=f"{name}_{axis}",
                    legendgrouptitle=dict(text=name),
                )
            )
    
            fig.add_trace(
                go.Cone(
                    x=[axes[axis]["x"][1]],
                    y=[axes[axis]["y"][1]],
                    z=[axes[axis]["z"][1]],
                    u=axes[axis]["u"],
                    v=axes[axis]["v"],
                    w=axes[axis]["w"],
                    name=axis,
                    showscale=False,
                    colorscale= [ [ 0, axes[axis]["color"] ], [ 1, axes[axis]["color"] ] ],
                    anchor="tail",
                    sizemode="scaled",
                    legendgroup=f"{name}_{axis}",
                    legendgrouptitle=dict(text=name),
                    showlegend=True
                    # hoverinfo="skip",
                    # hovertemplate=None
                )
            )
        return fig

    def plot(self, data: dict):
        fig = go.Figure()
        
        self.plot_3d_coordinate_frame(fig=fig, position=[0,0,0], orientation=[0,0,0,1], name="coord_eef")
        self.plot_3d_coordinate_frame(fig=fig, position=data['tf_eef_to_cut_point'][:3, 3].T, orientation=[0,0,0,1], name="coord_cut_point")
    
    
        for i in range(self.num_sensors):
            self.plot_3d_coordinate_frame(fig=fig, position=data[f'tof{i}_pos'], orientation=[0,0,0,1], name=f"coord_tof{i}")
            fig.add_trace(
                go.Scatter3d(
                    x=[data[f'reading{i}'][0]],
                    y=[data[f'reading{i}'][1]],
                    z=[data[f'reading{i}'][2]],
                    name=f"TOF_reading_{i}",
                    mode="markers",
                    marker=dict(
                        size=6,
                        color="red"
                    )
                )
            )
    
        _size = 1.0
        fig.update_layout(
            scene=dict(
                aspectmode="cube",
                xaxis=dict(range=[-_size, _size]),
                yaxis=dict(range=[-_size, _size]),
                zaxis=dict(range=[-_size, _size]),
            ),
            title="EEF coordinates",
            legend=dict(traceorder='grouped')
        )
    
    
        fig.show()
    
        return


def main():

    # tf_tof0_to_eef = _get_transform_matrix("tof0", "eef")
    # tf_tof1_to_eef = _get_transform_matrix("tof1", "eef")
    # tf_cut_point_to_eef = _get_transform_matrix("cut_point", "eef")
    # tf_eef_to_world = _get_transform_matrix("eef", "world")
    # 
    controller = CutPointRotateAxisController()

    x_offset = 0.0
    y_offset = 0.0
    z_offset = 0.0

    tof_x_dist_from_eef = 0.15

    test_data = dict(
        max_angular_velocity = 0.1,
        max_linear_velocity = 0.1,
        tof0_pos = np.array([tof_x_dist_from_eef+x_offset, 0, 0.1]),
        tof1_pos = np.array([-tof_x_dist_from_eef+x_offset, 0, 0.1]),

        # These should be in the camera frame, not the eef frame nor the tof frames.
        reading0 = np.array([tof_x_dist_from_eef+x_offset, 0, 0.61]),
        reading1 = np.array([-tof_x_dist_from_eef+x_offset, 0, 0.81]),

        tf_tof0_to_eef = np.array(
            [[1, 0, 0, 0.1],
             [0, 1, 0, 0],
             [0, 0, 1, 0.1],
             [0, 0, 0, 1]
            ]
        ),
        tf_tof1_to_eef = np.array(
            [[1, 0, 0, -0.1],
             [0, 1, 0, 0],
             [0, 0, 1, 0.1],
             [0, 0, 0, 1]
            ]
        ),
        tf_eef_to_cut_point = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0.4],
             [0, 0, 0, 1]
            ]
        ),
        tf_eef_to_world = np.array(
            [[1, 0, 0, -0.25],
             [0, 1, 0, 0.6],
             [0, 0, 1, 1.45],
             [0, 0, 0, 1]
            ]
        )
    )
    
    


    while True:
        try:
            angle_from_perpendicular = controller.get_angle_from_perpendicular(test_data)
            if not np.isclose(angle_from_perpendicular, 0, atol=np.radians(2)):
                rot_ax = controller.get_rotation_axis(test_data)
                # print(rot_ax)
                tf_cut_point_to_rot_axis = controller.get_cut_point_to_rot_axis_transform(test_data, rot_ax)

                # print(rot_ax[3:6, :] * np.array([[0, angle_from_perpendicular, 0]]).T)

                # Rotate around camera y-axis
                R = np.identity(4)
                R[:3, :3] = Rotation.from_euler('xyz', (rot_ax[3:6, :] * np.array([[0, angle_from_perpendicular, 0]]).T).T, degrees=False).as_matrix()

                twist = controller.get_twist(tf_cut_point_to_rot_axis, angle_from_perpendicular)
                print(R)

                time.sleep(1)
                break

            print('debug exit loop')
            break

        except KeyboardInterrupt:
            break

    controller.plot(test_data)


    return


if __name__ == "__main__":
    main()
