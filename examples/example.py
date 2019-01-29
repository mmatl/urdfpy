import numpy as np

from urdfpy.urdf import URDF

if __name__ == '__main__':
    rob = URDF.from_xml_file('../data/ur5/ur5.urdf')

    # Visualize rob
    from ambidex.utils.visualization import Visualizer3D as vis3d
    vis3d.figure()
    joint_config = {
        'shoulder_pan_joint' : 0.0,
        'shoulder_lift_joint' : -np.pi/4.0,
        'elbow_joint' : np.pi/2.0,
        'wrist_1_joint' : -3.0*np.pi/4.0,
        'wrist_2_joint' : -np.pi/2.0,
        'wrist_3_joint' : 0.0,
    }
    link_to_pose = rob.forward_kinematics(joint_config)
    for l in link_to_pose:
        if len(l.visuals) > 0:
            m = l.visuals[0].geometry.mesh.mesh
            p = link_to_pose[l].dot(l.visuals[0].origin)
            vis3d.mesh(m, pose=p)
    vis3d.show()

    rob.to_xml_file('./output/ur5.urdf')
