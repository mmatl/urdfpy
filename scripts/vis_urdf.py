"""
Script for visualizing a robot from a URDF.
Author: Matthew Matl
"""
import argparse

import urdfpy

if __name__ == '__main__':

    # Parse Args
    parser = argparse.ArgumentParser(
        description='Visualize a robot from a URDF file'
    )
    parser.add_argument('urdf', type=str,
                        help='Path to URDF file that describes the robot')
    parser.add_argument('-a', action='store_true',
                        help='Visualize robot articulation')
    parser.add_argument('-c', action='store_true',
                        help='Use collision geometry')

    args = parser.parse_args()

    robot = urdfpy.URDF.load(args.urdf)

    if args.a:
        robot.animate(use_collision=args.c)
    else:
        robot.show(use_collision=args.c)
