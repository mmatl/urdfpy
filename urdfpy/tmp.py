import xml.etree.ElementTree as ET

import numpy as np
import six

def _rpy_to_mat(r, p, y):
    ci = np.cos(r)
    cj = np.cos(p)
    ch = np.cos(y)
    si = np.sin(r)
    sj = np.sin(p)
    sh = np.sin(y)
    cc = ci * ch
    cs = ci * sh
    sc = si * ch
    ss = si * sh

    return np.array([
        [cj*ch, sj*sc-cs, sj*cc+ss],
        [cj*sh, sj*ss+cc, sj*cs-sc],
        [-sh,   cj*si,    cj*ci]
    ])

def _mat_to_rpy(mat):
    r = 0.0;
    p = 0.0;
    y = 0.0;
    if np.abs(mat[2,0]) >= 1.0:
        if mat[2,0] < 0:
            delta = np.atan2(m[0,1], m[0,2])
            p = np.pi / 2.0
            r = delta
        else:
            delta = np.atan2(-m[0,1], -m[0,2])
            p = -np.pi / 2
            r = delta
    else:
        p = -np.arcsin(mat[2,0])
        r = np.atan2(mat[2,1] / np.cos(p), mat[2,2] / np.cos(p))
        y = np.atan2(mat[1,0] / np.cos(p), mat[0,0] / np.cos(p))

    return r, p, y

def _unpack_origin(self, node):
    origin = np.eye(4)
    origin_node = node.find('origin')
    if origin_node is not None:
        if 'xyz' in origin_node.attrib:
            origin[:3,3] = np.fromstring(origin_node.attrib['xyz'], sep=' ')
        if 'rpy' in origin_node.attrib:
            rpy = np.fromstring(origin_node.attrib['rpy'], sep=' ')
            origin[:3,:3] = _rpy_to_mat(*rpy)
    return origin

def _pack_origin(self, origin):
    node = ET.Element('origin')
    r, p, y = _mat_to_rpy(origin[:3,:3])
    x, y, z = origin[:3,3]
    node.attrib['xyz'] = '{} {} {}'.format(x, y, z)
    node.attrib['rpy'] = '{} {} {}'.format(r, p, y)
    return node

class URDFType(object):
    ATTRIBS = {} # Map from attrib name to (type, required)
    ELEMENTS = {} # Map from element name to (type, required, multiple)
    TAG = ''

    def __init__(self):
        pass

    @classmethod
    def _parse_attrib(cls, val_type, val):
        if val_type == np.ndarray:
            val = np.fromstring(val, sep=' ')
        else:
            val = val_type(val)
        return val

    @classmethod
    def _parse_simple_attribs(cls, node):
        kwargs = {}
        for a in cls.ATTRIBS:
            t, r = cls.ATTRIBS[a]
            if r:
                v = cls._parse_attrib(t, node.attrib[a])
            else:
                v = None
                if a in node.attribs:
                    v = cls._parse_attrib(t, node.attrib[a])
            kwargs[a] = v
        return kwargs

    @classmethod
    def _parse_simple_elements(cls, node, path):
        kwargs = {}
        for a in cls.ELEMENTS:
            t, r, m = cls.ELEMENTS[a]
            if not m:
                v = node.find(t.TAG)
                if r or v is not None:
                    v = t._from_xml(v, path)
            else:
                vs = node.findall(a)
                v = [t._from_xml(n, path) for n in vs]
            kwargs[a] = v
        return kwargs

    @classmethod
    def _parse(cls, node, path):
        kwargs = cls._parse_simple_attribs(node)
        kwargs.update(cls._parse_simple_elements(node, path))
        return kwargs

    @classmethod
    def _from_xml(cls, node, path):
        return cls(**cls._parse(node, path))

    def _unparse_attrib(self, val_type, val):
        if val_type == np.ndarray:
            val = np.array2string(val)[1:-1]
        else:
            val = str(val)
        return val

    def _unparse_simple_attribs(self, node):
        for a in self.ATTRIBS:
            t, r = self.ATTRIBS[a]
            v = getattr(self, a, None)
            if r or v is not None:
                node.attrib[a] = self._unparse_attrib(t, v)

    def _unparse_simple_elements(self, node, path):
        for a in cls.ELEMENTS:
            t, r, m = cls.ELEMENTS[a]
            v = getattr(self, a, None)
            if not m:
                if r or v is not None:
                    node.append(v._to_xml(path))
            else:
                vs = v
                for v in vs:
                    node.append(v._to_xml(path))

    def _unparse(self, path):
        node = ET.Element(self.TAG)
        self._unparse_simple_attribs(node)
        self._unparse_simple_elements(node, path)
        return node

    def _to_xml(self, path):
        return self._unparse(path)

class JointDynamics(URDFType):
    ATTRIBS = {
        'damping' : (float, False),
        'friction' : (float, False),
    }
    TAG = 'dynamics'

    def __init__(self, damping, friction):
        self.damping = damping
        self.friction = friction

class Box(URDFType):
    ATTRIBS = {
        'size' : (np.ndarray, True)
    }
    TAG = 'box'

    def __init__(self, size):
        self.size = size

class Cylinder(URDFType):
    ATTRIBS = {
        'radius' : (float, True),
        'length' : (float, True),
    }
    TAG = 'cylinder'

    def __init__(self, radius, length):
        self.radius = radius
        self.length = length

class Sphere(URDFType):
    ATTRIBS = {
        'radius' : (float, True),
    }
    TAG = 'sphere'

    def __init__(self, radius):
        self.radius = radius

class Mesh(URDFType):
    ATTRIBS = {
        'filename' : (str, True),
        'scale' : (float, False)
    }
    TAG = 'mesh'

    def __init__(self, filename, scale, mesh):
        self.filename = filename
        self.scale = scale
        self.mesh = mesh

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['mesh'] = trimesh.load(os.path.join(path, filename))
        return Mesh(**kwargs)

    def _to_xml(self, path):
        self.mesh.export(os.path.join(path, self.filename))
        return self._unparse(path)

class Geometry(URDFType):
    ELEMENTS = {
        'box' : (Box, False, False),
        'cylinder' : (Cylinder, False, False),
        'sphere' : (Sphere, False, False),
        'mesh' : (Mesh, False, False),
    }
    TAG = 'geometry'

    def __init__(self, box, cylinder, sphere, mesh):
        self.box = box
        self.cylinder = cylinder
        self.sphere = sphere
        self.mesh = mesh

class Collision(URDFType):
    ATTRIBS = {
        'name' : (str, False)
    }
    ELEMENTS = {
        'geometry' : (Geometry, True, False),
    }
    TAG = 'collision'

    def __init__(self, name, origin, geometry):
        self.name = name
        self.geometry = geometry
        self.origin = origin

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['origin'] = _unpack_origin(node)
        return Collision(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        node.append(_pack_origin(self.origin))
        return node

class Visual(URDFType):
    ATTRIBS = {
        'name' : (str, False)
    }
    ELEMENTS = {
        'geometry' : (Geometry, True, False),
        'material' : (LinkMaterial, False, False),
    }
    TAG = 'visual'

    def __init__(self, name, origin, geometry, material):
        self.name = name
        self.geometry = geometry
        self.origin = origin
        self.material = material

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['origin'] = _unpack_origin(node)
        return Visual(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        node.append(_pack_origin(self.origin))
        node.append(self.geometry._to_xml())
        if self.material is not None:
            node.append(self.material._to_xml())
        return node

class Texture(URDFType):
    ATTRIBS = {
        'filename' : (str, True)
    }
    TAG = 'texture'

    def __init__(self, filename, image):
        self.filename = filename
        self.image = image

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['image'] = ColorImage.open(os.path.join(path, filename))
        return Texture(**kwargs)

    def _to_xml(self, path):
        self.image.save(os.path.join(path, self.filename))
        return self._unparse(path)

class LinkMaterial(URDFType):
    ATTRIBS = {
        'name' : (str, True)
    }
    ELEMENTS = {
        'texture' : (Texture, False, False),
    }
    TAG = 'material'

    def __init__(self, name, color, texture):
        self.name = name
        self.color = color
        self.texture = texture

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        color = node.find('color')
        if color is not None:
            color = np.fromstring(color.attrib['rgba'], sep=' ')
        kwargs['color'] = color
        return LinkMaterial(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        if self.color is not None:
            color = ET.Element('color')
            color.attrib['rgba'] = np.array2string(self.color)[1:-1]
            node.append(color)
        return node

class Inertial(URDFType):

    def __init__(self, mass, inertia, origin):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin

    @classmethod
    def _from_xml(cls, node, path):
        origin = _unpack_origin(node)
        mass = float(node.find('mass').text)
        n = node.find('inertia')
        xx = float(n.attrib['ixx'])
        xy = float(n.attrib['ixy'])
        xz = float(n.attrib['ixz'])
        yy = float(n.attrib['iyy'])
        yz = float(n.attrib['iyz'])
        zz = float(n.attrib['izz'])
        inertia = np.array([
            [xx, xy, xz],
            [xy, yy, yz],
            [xz, yz, zz]
        ])
        return Inertial(mass=mass, inertia=inertia, origin=origin)

    def _to_xml(self, path):
        node = ET.Element('inertial')
        node.append(_pack_origin(self.origin))
        mass = ET.Element('mass')
        mass.text = str(self.mass)
        node.append(mass)
        inertia = ET.Element('inertia')
        inertia.attrib['ixx'] = str(self.inertia[0,0])
        inertia.attrib['ixy'] = str(self.inertia[0,1])
        inertia.attrib['ixz'] = str(self.inertia[0,2])
        inertia.attrib['iyy'] = str(self.inertia[1,1])
        inertia.attrib['iyz'] = str(self.inertia[1,2])
        inertia.attrib['izz'] = str(self.inertia[2,2])
        node.append(inertia)
        return node


class JointCalibration(URDFType):
    ATTRIBS = {
        'rising' : (float, False),
        'falling' : (float, False)
    }
    TAG = 'calibration'

    def __init__(self, rising, falling):
        self.rising = rising
        self.falling = falling

class JointLimit(URDFType):
    ATTRIBS = {
        'lower' : (float, False),
        'upper' : (float, False)
        'effort' : (float, True),
        'velocity' : (float, True)
    }
    TAG = 'limit'

    def __init__(self, effort, velocity, lower, upper):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

class JointMimic(URDFType):
    ATTRIBS = {
        'joint' : (str, True),
        'multiplier' : (float, False)
        'offset' : (float, True),
    }
    TAG = 'mimic'

    def __init__(self, joint, multiplier, offset):
        self.joint = joint_name
        self.multiplier = multiplier
        self.offset = offset

class SafetyController(URDFType):
    ATTRIBS = {
        'soft_lower_limit' : (float, False),
        'soft_upper_limit' : (float, False),
        'k_position' : (float, False),
        'k_velocity' : (float, True)
        'offset' : (float, True),
    }
    TAG = 'safety_controller'

    def __init__(self, soft_lower_limit, soft_upper_limit, k_position, k_velocity):
        self.k_velocity = k_velocity
        self.k_position = k_position
        self.soft_lower_limit = soft_lower_limit
        self.soft_upper_limit = soft_upper_limit

class Joint(URDFType):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    ATTRIBS = {
        'name' : (str, True),
        'type' : (str, True),
    }
    ELEMENTS = {
        'dynamics' : (JointDynamics, False, False),
        'limit' : (JointLimit, False, False),
        'mimic' : (JointMimic, False, False),
        'safety_controller' : (SafetyController, False, False),
        'calibration' : (JointCalibration, False, False),
    }
    TAG = 'joint'

    def __init__(self, name, parent, child, type, axis, origin,
                 limit, dynamics, safety_controller, calibration, mimic):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['parent'] = node.find('parent').attrib['link']
        kwargs['child'] = node.find('child').attrib['link']
        axis = node.find['axis']
        if axis is not None:
            axis = np.fromstring(axis.attrib['xyz'], sep=' ')
        else:
            if kwargs['type'] in set(['revolute', 'prismatic', 'planar']):
                axis = np.array([1.0, 0.0, 0.0])
        kwargs['axis'] = axis
        kwargs['origin'] = _pack_origin(node)
        return Joint(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        parent = ET.Element('parent')
        parent.attrib['link'] = self.parent
        node.append(parent)
        child = ET.Element('child')
        child.attrib['link'] = self.child
        node.append(child)
        if self.axis is not None:
            axis = ET.Element('axis')
            axis.attrib['xyz'] = np.array2string(self.axis)[1:-1]
            node.append(axis)
        node.append(_pack_origin(self.origin))
        return node

class Link(URDFType):
    ATTRIBS = {
        'name' : (str, True),
    }
    ELEMENTS = {
        'inertial' : (Inertial, False, False),
        'visuals' : (Visual, False, True),
        'collisions' : (Collision, False, True),
    }
    TAG = 'link'

    def __init__(self, name, visuals, inertial, collisions):
        self.name = name
        self.visuals = visuals
        self.inertial = inertial
        self.collisions = collisions

class Actuator(URDFType):
    ATTRIBS = {
        'name' : (str, True),
    }
    TAG = 'actuator'

    def __init__(self, name, mechanicalReduction, hardwareInterfaces):
        self.name = name
        self.mechanicalReduction = mechanicalReduction
        self.hardwareInterfaces = hardwareInterfaces

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        mr = node.find('mechanicalReduction')
        if mr is not None:
            mr = float(mr.text)
        kwargs['mechanicalReduction'] = mr
        hi = node.findall('hardwareInterface')
        if len(hi) > 0:
            hi = [h.text for h in hi]
        kwargs['hardwareInterfaces'] = hi
        return Actuator(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        if self.mechanicalReduction is not None:
            mr = ET.Element('mechanicalReduction')
            mr.text = str(self.mechanicalReduction)
            node.append(mr)
        if len(self.hardwareInterfaces) > 0:
            for hi in self.hardwareInterfaces:
                h = ET.Element('hardwareInterface')
                h.text = hi
                node.append(h)
        return node

class TransmissionJoint(URDFType):
    ATTRIBS = {
        'name' : (str, True),
    }
    TAG = 'joint'

    def __init__(self, name, hardwareInterfaces):
        self.name = name
        self.hardwareInterfaces = []

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        hi = node.findall('hardwareInterface')
        if len(hi) > 0:
            hi = [h.text for h in hi]
        kwargs['hardwareInterfaces'] = hi
        return TransmissionJoint(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        if len(self.hardwareInterfaces) > 0:
            for hi in self.hardwareInterfaces:
                h = ET.Element('hardwareInterface')
                h.text = hi
                node.append(h)
        return node

class Transmission(URDFType):
    ATTRIBS = {
        'name' : (str, True),
    }
    ELEMENTS = {
        'joints' : (TransmissionJoint, True, True),
        'actuators' : (Actuator, True, True),
    }
    TAG = 'transmission'

    def __init__(self, name, type, joints, actuators):
        self.name = name
        self.type = type
        self.joints = joints
        self.actuators = actuators

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['type'] = node.find('type').text
        return Transmission(**kwargs)

    def _to_xml(self, path):
        node = self._unparse(path)
        ttype = ET.Element('type')
        ttype.text = self.type
        node.append(ttype)
        return node

class Robot(URDFType):
    ATTRIBS = {
        'name' : (str, True),
    }
    ELEMENTS = {
        'joints' : (Joint, False, True),
        'links' : (Link, True, True),
        'transmissions' : (Transmission, False, True),
        'materials' : (LinkMaterial, False, True),
    }
    TAG = 'robot'

    def __init__(self, name=None, joints=None, links=None,
                 transmissions=None, materials=None):
        self.name = name
        self.joints = joints
        self.links = links
        self.transmissions = transmissions
        self.materials = materials

    @staticmethod
    def _from_xml_file(file_obj):
        if isinstance(file_obj, six.string_types):
            tree = ET.fromstring(file_obj)
            path, _ = os.path.split(file_obj)
        else:
            tree = ET.parse(file_obj)
            path, _ = os.path.split(file_obj.name)

        node = tree.getroot()
        return Robot._from_xml(node, path)

    @staticmethod
    def _to_xml_file(self, file_obj):
        if isinstance(file_obj, six.string_types):
            path, _ = os.path.split(file_obj)
        else:
            path, _ = os.path.split(file_obj.name)

        node = self._to_xml(path)
        tree = ET.ElementTree(node)
        tree.write(file_obj)

if __name__ == '__main__':
    import sys
    x = sys.argv[1]
    r = Robot._from_xml_file(x)
    import pdb
    pdb.set_trace()
