import numpy as np
import trimesh.transformations as tfs

from urdfpy import (rpy_to_matrix, matrix_to_rpy, matrix_to_xyz_rpy,
                    xyz_rpy_to_matrix)


def test_rpy_to_matrix():
    xr45 = tfs.rotation_matrix(np.pi / 2, [1, 0, 0])[:3,:3]
    yr45 = tfs.rotation_matrix(np.pi / 2, [0, 1, 0])[:3,:3]
    zr45 = tfs.rotation_matrix(np.pi / 2, [0, 0, 1])[:3,:3]
    c = zr45.dot(yr45.dot(xr45))

    assert np.allclose(rpy_to_matrix([np.pi / 2, 0, 0]), xr45)
    assert np.allclose(rpy_to_matrix([0, np.pi / 2, 0]), yr45)
    assert np.allclose(rpy_to_matrix([0, 0, np.pi / 2]), zr45)
    assert np.allclose(rpy_to_matrix(np.pi / 2 * np.ones(3)), c)


def test_matrix_to_rpy():
    xr45 = tfs.rotation_matrix(np.pi / 2, [1, 0, 0])[:3,:3]
    yr45 = tfs.rotation_matrix(np.pi / 2, [0, 1, 0])[:3,:3]
    zr45 = tfs.rotation_matrix(np.pi / 2, [0, 0, 1])[:3,:3]

    assert np.allclose([np.pi / 2, 0, 0], matrix_to_rpy(xr45))
    assert np.allclose([0, np.pi / 2, 0], matrix_to_rpy(yr45))
    assert np.allclose([0, 0, np.pi / 2], matrix_to_rpy(zr45))


def test_matrix_to_xyz_rpy():
    xr45 = tfs.rotation_matrix(np.pi / 2, [1, 0, 0])
    yr45 = tfs.rotation_matrix(np.pi / 2, [0, 1, 0])
    zr45 = tfs.rotation_matrix(np.pi / 2, [0, 0, 1])

    xr45[:3,3] = np.array([1,2,3])
    yr45[:3,3] = np.array([2,3,1])
    zr45[:3,3] = np.array([3,1,2])

    assert np.allclose([1, 2, 3, np.pi / 2, 0, 0], matrix_to_xyz_rpy(xr45))
    assert np.allclose([2, 3, 1, 0, np.pi / 2, 0], matrix_to_xyz_rpy(yr45))
    assert np.allclose([3, 1, 2, 0, 0, np.pi / 2], matrix_to_xyz_rpy(zr45))


def test_xyz_rpy_to_matrix():
    xr45 = tfs.rotation_matrix(np.pi / 2, [1, 0, 0])
    yr45 = tfs.rotation_matrix(np.pi / 2, [0, 1, 0])
    zr45 = tfs.rotation_matrix(np.pi / 2, [0, 0, 1])

    xr45[:3,3] = np.array([1,2,3])
    yr45[:3,3] = np.array([2,3,1])
    zr45[:3,3] = np.array([3,1,2])
    assert np.allclose(xyz_rpy_to_matrix([1, 2, 3, np.pi / 2, 0, 0]), xr45)
    assert np.allclose(xyz_rpy_to_matrix([2, 3, 1, 0, np.pi / 2, 0]), yr45)
    assert np.allclose(xyz_rpy_to_matrix([3, 1, 2, 0, 0, np.pi / 2]), zr45)
