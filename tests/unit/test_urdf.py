import os

import numpy as np

from urdfpy import URDF, Link, Joint, Transmission, Material

def test_urdfpy(tmpdir):
    outfn = tmpdir.mkdir('urdf').join('ur5.urdf').strpath

    # Load
    u = URDF.from_xml_file('data/ur5/ur5.urdf')

    assert isinstance(u, URDF)
    for j in u.joints:
        assert isinstance(j, Joint)
    for l in u.links:
        assert isinstance(l, Link)
    for t in u.transmissions:
        assert isinstance(t, Transmission)
    for m in u.materials:
        assert isinstance(u, Material)

    # Test fk
    fk = u.forward_kinematics()
    assert isinstance(fk, dict)
    for l in fk:
        assert isinstance(l, Link)
        assert isinstance(fk[l], np.ndarray)
        assert fk[l].shape == (4,4)

    # Test save
    u.to_xml_file(outfn)

    nu = URDF.from_xml_file('data/ur5/ur5.urdf')
    assert len(u.links) == len(nu.links)
    assert len(u.joints) == len(nu.joints)
