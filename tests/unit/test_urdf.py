import numpy as np

from urdfpy import URDF, Link, Joint, Transmission, Material


def test_urdfpy(tmpdir):
    outfn = tmpdir.mkdir('urdf').join('ur5.urdf').strpath

    # Load
    u = URDF.load('tests/data/ur5/ur5.urdf')

    assert isinstance(u, URDF)
    for j in u.joints:
        assert isinstance(j, Joint)
    for l in u.links:
        assert isinstance(l, Link)
    for t in u.transmissions:
        assert isinstance(t, Transmission)
    for m in u.materials:
        assert isinstance(m, Material)

    # Test fk
    fk = u.link_fk()
    assert isinstance(fk, dict)
    for l in fk:
        assert isinstance(l, Link)
        assert isinstance(fk[l], np.ndarray)
        assert fk[l].shape == (4,4)

    fk = u.link_fk({'shoulder_pan_joint': 2.0})
    assert isinstance(fk, dict)
    for l in fk:
        assert isinstance(l, Link)
        assert isinstance(fk[l], np.ndarray)
        assert fk[l].shape == (4,4)

    fk = u.link_fk(np.zeros(6))
    assert isinstance(fk, dict)
    for l in fk:
        assert isinstance(l, Link)
        assert isinstance(fk[l], np.ndarray)
        assert fk[l].shape == (4,4)

    fk = u.link_fk(np.zeros(6), link='upper_arm_link')
    assert isinstance(fk, np.ndarray)
    assert fk.shape == (4,4)

    fk = u.link_fk(links=['shoulder_link', 'upper_arm_link'])
    assert isinstance(fk, dict)
    assert len(fk) == 2
    for l in fk:
        assert isinstance(l, Link)
        assert isinstance(fk[l], np.ndarray)
        assert fk[l].shape == (4,4)

    fk = u.link_fk(links=list(u.links)[:2])
    assert isinstance(fk, dict)
    assert len(fk) == 2
    for l in fk:
        assert isinstance(l, Link)
        assert isinstance(fk[l], np.ndarray)
        assert fk[l].shape == (4,4)

    # Test save
    u.save(outfn)

    nu = URDF.load(outfn)
    assert len(u.links) == len(nu.links)
    assert len(u.joints) == len(nu.joints)
