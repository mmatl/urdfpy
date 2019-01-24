import xml.etree.ElementTree as ET
import six

def _parse(node, name, val_type, many=False, required=False):
    val = node.findall(name)
    if required and len(val) == 0:
        pn = ''
        if 'name' in node.attrib:
            pn = node.attrib['name']
        raise ValueError('Missing required field {} in {} "{}"'.format(name, pn, node.tag))
    if many:
        return [val_type._from_xml(v) for v in val]
    else:
        if len(val) == 0:
            return []
        else:
            return val_type._from_xml(val[-1])

#class URDFType(object):
#
#    TAG = ''        # String tag of node
#    ATTRIBS = []    # Simple, list of str
#    ELEMENTS = {}   # Map from attrib name to tuple containing Type, Required, Many
#    SPECIALS = {}   # Map from special attribs to Function, Requred, ?
#
#    def __init__(self):
#        pass
#
#    @classmethod
#    def _parse(cls, node):
#        kwargs = {}
#
#        # Get attributes
#        for a in cls.ATTRIBS:
#            kwargs[a] = node.attrib[a]
#
#        # Set elements
#        for e in cls.ELEMENTS:
#            vtype, required, many = cls.ELEMENTS[e]
#            values = node.findall(e)
#            if required and len(values) == 0:
#                pn = ''
#                if 'name' in node.attrib:
#                    pn = node.attrib['name']
#                raise ValueError('Missing required field {} in {} "{}"'.format(name, pn, node.tag))
#            if many or len(values) == 0:
#                kwargs[e] = [vtype._parse(v) for v in values]
#            else:
#                return vtype._parse(values[-1])
#
#        # Set specials
#        for e in cls.SPECIALS:
#            kwargs[e] = cls.SPECIALS[e](
#
#        kwargs = cls._postparse(kwargs)
#
#        return cls(**kwargs)
#
#    @classmethod
#    def _postparse(cls, kwargs):
#        return kwargs
#
#    def _serialize(self, parent):
#        node = ET.Element(self.TAG)
#
#        # Set attributes
#        for a in self.ATTRIBS:
#            node.attrib[a] = getattr(self, a, Nonde)
#
#        # Get elements
#        for e in self.ELEMENTS:
#            pass







class JointDynamics(object):
    def __init__(self, damping=None, friction=None):
        self.damping = damping
        self.friction = friction

class Box(object):

    def __init__(self, size=None):
        self.size = size

    @staticmethod
    def _from_xml(self, node, path):
        size = np.fromstring(node.attrib['size'], sep=' ')
        return Box(size=size)

    def _to_xml(self, path):
        node = ET.Element('box')
        node.attrib['size'] = np.array2string(self.size)[1:-1]
        return node

class Cylinder(object):

    def __init__(self, radius, length):
        self.radius = radius
        self.length = length

    @staticmethod
    def _from_xml(self, node, path):
        radius = float(node.attrib['radius'])
        length = float(node.attrib['length'])
        return Cylinder(radius, length)

    def _to_xml(self, path):
        node = ET.Element('cylinder')
        node.attrib['radius'] = str(self.radius)
        node.attrib['length'] = str(self.length)
        return node

class Sphere(object):

    def __init__(self, radius):
        self.radius = radius

    @staticmethod
    def _from_xml(self, node, path):
        radius = float(node.attrib['radius'])
        return Sphere(radius)

    def _to_xml(self, path):
        node = ET.Element('cylinder')
        node.attrib['radius'] = str(self.radius)
        return node

class Mesh(object):

    def __init__(self, filename, scale, mesh):
        self.filename = filename
        self.scale = scale
        self.mesh = mesh

    @staticmethod
    def _from_xml(self, node, path):
        filename = node.attrib['filename']
        scale = 1.0
        if 'scale' in node.attrib:
            scale = float(node.attrib['scale'])
        mesh = trimesh.load(os.path.join(path, filename))
        return Mesh(filename=filename, scale=scale, mesh=mesh)

    def _to_xml(self, path):
        node = ET.Element('mesh')
        node.attrib['filename'] = str(self.filename)
        node.attrib['scale'] = str(self.scale)
        self.mesh.export(os.path.join(path, self.filename))
        return node

class Geometry(object):

    def __init__(self, box=None, cylinder=None, sphere=None, mesh=None):
        self.box = box
        self.cylinder = cylinder
        self.sphere = sphere
        self.mesh = mesh

    @staticmethod
    def _from_xml(self, node, path):
        box, cylinder, sphere, mesh = None, None, None, None
        children = []
        for child in node:
            children.append(child)
        child = children[-1]
        if child.tag == 'box':
            box = Box._from_xml(child, path)
        elif child.tag == 'cylinder':
            cylinder = Cylinder._from_xml(child, path)
        elif child.tag == 'sphere':
            sphere = Sphere._from_xml(child, path)
        elif child.tag == 'mesh':
            mesh = Mesh._from_xml(child, path)
        return Geometry(box, cylinder, sphere, mesh)

    @staticmethod
    def _to_xml(self, path):
        node = ET.Element('geometry')
        if self.box is not None:
            node.append(self.box._to_xml(path))
        elif self.cylinder is not None:
            node.append(self.cylinder._to_xml(path))
        elif self.sphere is not None:
            node.append(self.sphere._to_xml(path))
        elif self.mesh is not None:
            node.append(self.mesh._to_xml(path))
        return node

class Collision(object):

    def __init__(self, name=None, origin=None, geometry=None):
        self.name = name
        self.geometry = geometry
        self.origin = origin

    @staticmethod
    def _from_xml(self, node, path):
        name = None
        if 'name' in node.attrib:
            name = node.attrib['name']

        origin = np.eye(4)
        origin_node = node.find('origin')
        if origin_node is not None:
            if 'xyz' in origin_node.attrib:
                origin[:3,3] = np.fromstring(origin_node.attrib['xyz'], sep=' ')
            if 'rpy' in origin_node.attrib:
                rpy = np.fromstring(origin_node.attrib['rpy'], sep=' ')
        size = np.fromstring(node.attrib['size'], sep=' ')

        filename = node.attrib['filename']
        scale = 1.0
        if 'scale' in node.attrib:
            scale = float(node.attrib['scale'])
        mesh = trimesh.load(os.path.join(path, filename))
        return Mesh(filename=filename, scale=scale, mesh=mesh)

    def _to_xml(self, path):
        node = ET.Element('mesh')
        node.attrib['filename'] = str(self.filename)
        node.attrib['scale'] = str(self.scale)
        self.mesh.export(os.path.join(path, self.filename))
        return node


class Texture(object):
    def __init__(self, filename=None):
        self.filename = filename

class LinkMaterial(object):
    def __init__(self, name=None, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    def check_valid(self):
        if self.color is None and self.texture is None:
            xmlr.on_error("Material has neither a color nor texture.")

class Visual(object):
    def __init__(self, geometry=None, material=None, origin=None):
        self.geometry = geometry
        self.material = material
        self.origin = origin

class Inertia(object):
    KEYS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']

    def __init__(self, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0):
        self.ixx = ixx
        self.ixy = ixy
        self.ixz = ixz
        self.iyy = iyy
        self.iyz = iyz
        self.izz = izz

    def to_matrix(self):
        return [
            [self.ixx, self.ixy, self.ixz],
            [self.ixy, self.iyy, self.iyz],
            [self.ixz, self.iyz, self.izz]]


class Inertial(object):
    def __init__(self, mass=0.0, inertia=None, origin=None):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin

class JointCalibration(object):
    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling


class JointLimit(object):
    def __init__(self, effort=None, velocity=None, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

class JointMimic(object):
    def __init__(self, joint_name=None, multiplier=None, offset=None):
        self.joint = joint_name
        self.multiplier = multiplier
        self.offset = offset

class SafetyController(object):
    def __init__(self, velocity=None, position=None, lower=None, upper=None):
        self.k_velocity = velocity
        self.k_position = position
        self.soft_lower_limit = lower
        self.soft_upper_limit = upper

class Joint(object):
    TYPES = ['unknown', 'revolute', 'continuous', 'prismatic',
             'floating', 'planar', 'fixed']

    def __init__(self, name=None, parent=None, child=None, joint_type=None,
                 axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    def _from_xml(self, xml):
        pass

class Link(object):

    def __init__(self, name=None, visual=None, inertial=None, collision=None):
        self.name = name
        self.visual = visual
        self.inertial = inertial
        self.collision = collision

    def _link(self, links, joints, transmissions, materials):
        for child in [self.visual, self.inertial, self.collision]:
            child._link(links, joints, transmissions, materials)

    def _to_xml():
        node = ET.Element('link')
        node.attrib['name'] = name
        node.append(



    @staticmethod
    def _from_xml(self, node):
        name = node.attrib['name']
        inertial = _parse(node, 'inertial', Inertial)
        visuals = _parse(node, 'visual', Visual, many=True)
        visual = Visual.merge(visuals)
        collisions = _parse(node, 'collision' Collision, many=True)
        collision = Collision.merge(collisions)
        return Link(name=name, visual=visual, inertial=inertial, collision=collision)


class Actuator(object):
    def __init__(self, name=None, mechanicalReduction=1):
        self.name = name
        self.mechanicalReduction = None


class TransmissionJoint(object):
    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.hardwareInterfaces = []

class Transmission(object):
    """ New format: http://wiki.ros.org/urdf/XML/Transmission """

    def __init__(self, name=None):
        self.aggregate_init()
        self.name = name
        self.joints = []
        self.actuators = []


class Robot(object):

    def __init__(self, name=None, joints=None, links=None,
                 transmissions=None, materials=None):
        self.name = name
        self.joints = joints
        self.links = links
        self.transmissions = transmissions
        self.materials = materials

    @staticmethod
    def _from_xml_file(self, file_obj):
        if isinstance(file_obj, six.string_types):
            tree = ET.fromstring(file_obj)
        else:
            tree = ET.parse(file_obj)

        node = tree.getroot()
        name = node.attrib['name']

        joints = {}
        links = {}
        transmissions = {}
        materials = {}
        # Extract joints, links, transmissions, materials -- first pass
        for child in node:
            if child.tag == 'joint':
                joints[child.attrib['name']] = Joint._from_xml(child)
            elif child.tag == 'link':
                links[child.attrib['name']] = Link._from_xml(child)
            elif child.tag == 'material':
                materials[child.attrib['name']] = LinkMaterial._from_xml(child)
            elif child.tag == 'transmission':
                transmissions[child.attrib['name']] = Transmission._from_xml(child)
            else:
                pass

        # Now, take second pass, linking up all of the items
        for joint_name in joints:
            joint = joints[joint_name]
            joint._update(links, joints, transmissions, materials)

        for link_name in links:
            link = links[link_name]
            link._link(links, joints, transmissions, materials)

        for transmission_name in transmissions:
            transmission = transmissions[transmission_name]
            transmission._update(links, joints, transmissions, materials)

        return Robot(name=name, joints=[joints[k] for k in joints],
                                links=[links[k] for k in links],
                                transmissions=[transmissions[k] for k in transmissions],
                                materials=[materials[k] for k in materials])

    @staticmethod
    def _to_xml_file(self, file_obj):
        node = ET.Element('robot')
        node.attrib['name'] = self.name

        for child in self.joints + self.links + self.transmissions + self.materials:
            node.append(child._to_xml())

        tree = ET.ElementTree(node)
        tree.write(file_obj)
