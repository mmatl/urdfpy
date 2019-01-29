"""Utilities for URDF parsing.
"""
import numpy as np

def rpy_to_mat(r, p, y):
    ""
    c1 = np.cos(y)
    c2 = np.cos(p)
    c3 = np.cos(r)
    s1 = np.sin(y)
    s2 = np.sin(p)
    s3 = np.sin(r)

    return np.array([
        [c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
        [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
        [-s2,   c2*s3,          c2*c3]
    ])

def mat_to_rpy(m, solution=1):
    r = 0.0
    p = 0.0
    y = 0.0

    if np.abs(m[2,0]) >= 1.0 - 1e-12:
        y = 0
        if m[2,0] < 0:
            p = np.pi / 2
            r = np.arctan2(m[0,1], m[0,2])
        else:
            p = -np.pi / 2
            r = np.arctan2(-m[0,1], -m[0,2])
    else:
        if solution == 1:
            p = -np.arcsin(m[2,0])
        else:
            p = np.pi + np.arcsin(m[2,0])
        r = np.arctan2(m[2,1] / np.cos(p), m[2,2] / np.cos(p))
        y = np.arctan2(m[2,0] / np.cos(p), m[0,0] / np.cos(p))

    return r, p, y
