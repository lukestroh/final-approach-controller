#!/usr/bin/env python3

import modern_robotics as mr
import numpy as np

def get_angle_from_perpendicular(data: dict) -> float:
    # Make sure the two TOF frames are aligned with the end effector frame # TODO: This should be checked in init
    M = mr.TransInv(data['tf_tof0_to_eef']) @ data['tf_tof1_to_eef']
    # print(M)
    if not np.all(np.isclose(M[:3, :3], np.identity(3))):
        raise ValueError("The two TOF frames are not aligned with the end effector frame.")

    # Calculate the distance between the two TOF sensors # TODO: save this info in class attr
    tof_linear_pos_vec = abs(M[:3, 3])
    # print(tof_linear_pos_vec)
    tof_linear_pos_diff = np.linalg.norm(tof_linear_pos_vec)
    # print(x_diff)
    d0 = np.linalg.norm(data['reading0'] - data['tof0_pos'])
    d1 = np.linalg.norm(data['reading1'] - data['tof1_pos']) 
    d_diff = abs(d0 - d1)
    
    theta = np.arctan(d_diff / tof_linear_pos_diff)
    # print(theta * 180 / np.pi)
    return theta

def get_rotation_axis(data: dict):
    rotation_point = np.mean([data['reading0'], data['reading1']], axis=0)
    rotation_axis = np.zeros((6,1), dtype=float)
    rotation_axis[0:3, :] = rotation_point.reshape(3,1)
    rotation_axis[3:6, :] = np.cross(data['reading0'] - data['tof0_pos'], data['reading1'] - data['tof1_pos']).reshape(3,1)
    rotation_axis[3:6, :] = rotation_axis[3:6, :] / np.linalg.norm(rotation_axis[3:6, :])
    return rotation_axis

def get_axis_rotation_from_world_z(axis: np.ndarray) -> Rotation:
    # Get rotational vector, convert to quaternion
    z_axis = np.array([[0, 0, 1]])
    rot_axis = np.cross(z_axis, axis[3:6, :].T)
    rot_angle = np.arccos(np.dot(z_axis, axis[3:6, :]))[0,0] # both vectors are unit vectors
    rotation = mr.Rotation.from_rotvec(rotvec = (rot_angle * rot_axis))
    return rotation
    
def get_axis_rotation_matrix_from_world_z(axis: np.ndarray) -> np.ndarray:
    rotation = get_axis_rotation_from_world_z(axis)
    return rotation.as_matrix()

def get_axis_angle_from_world_z(axis: np.ndarray, as_quat: bool = False):
    # Get rotational vector, convert to quaternion
    z_axis = np.array([[0, 0, 1]])
    # print(axis[3:6, :])
    # print(z_axis)
    rot_axis = np.cross(z_axis, axis[3:6, :].T)
    rot_angle = np.arccos(np.dot(z_axis, axis[3:6, :]))[0,0] # both vectors are unit vectors
    
    # self.info_logger(f"Cross: {rot_axis}")
    if as_quat:
        rotation = mr.Rotation.from_rotvec(rotvec=rot_angle * rot_axis)
        quat = rotation.as_quat()
        return quat
    else:
        return rot_angle


def main():
    # tf_tof0_to_eef = _get_transform_matrix("tof0", "eef")
    # tf_tof1_to_eef = _get_transform_matrix("tof1", "eef")
    # tf_cut_point_to_eef = _get_transform_matrix("cut_point", "eef")
    # tf_eef_to_world = _get_transform_matrix("eef", "world")

    x_offset = 0.0
    y_offset = 0.0
    z_offset = 0.0

    test_data = dict(
        tof0_pos = np.array([0.1+x_offset, 0, 0]),
        tof1_pos = np.array([-0.1+x_offset, 0, 0]),
        reading0 = np.array([0.15+x_offset, 0.41, 0]),
        reading1 = np.array([-0.15+x_offset, 0.51, 0]),

        tf_tof0_to_eef = np.array(
            [[1, 0, 0, 0.1],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]
            ]
        ),
        tf_tof1_to_eef = np.array(
            [[1, 0, 0, -0.1],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]
            ]
        ),
        tf_cut_point_to_eef = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0.1],
             [0, 0, 0, 1]
            ]
        ),
        tf_eef_to_world = np.array(
            [[1, 0, 0, 0.25],
             [0, 1, 0, -0.6],
             [0, 0, 1, -1.45],
             [0, 0, 0, 1]
            ]
        )
    )


    while True:

        break


    get_rotation_axis(test_data)

    return


if __name__ == "__main__":
    main()
