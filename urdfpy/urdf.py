import os

from lxml import etree as ET
import networkx as nx
import numpy as np
import trimesh
import six

from .utils import rpy_to_mat, mat_to_rpy

def parse_origin(node):
    """Find the 'origin' subelement of a node
    and turn it into a 4x4 homogenous matrix.
    """
    origin = np.eye(4)
    origin_node = node.find('origin')
    if origin_node is not None:
        if 'xyz' in origin_node.attrib:
            origin[:3,3] = np.fromstring(origin_node.attrib['xyz'], sep=' ')
        if 'rpy' in origin_node.attrib:
            rpy = np.fromstring(origin_node.attrib['rpy'], sep=' ')
            origin[:3,:3] = rpy_to_mat(*rpy)
    return origin

def unparse_origin(origin):
    """Turn a 4x4 homogenous matrix into an 'origin' XML node.
    """
    node = ET.Element('origin')
    r, p, yaw = mat_to_rpy(origin[:3,:3])
    x, y, z = origin[:3,3]
    node.attrib['xyz'] = '{} {} {}'.format(x, y, z)
    node.attrib['rpy'] = '{} {} {}'.format(r, p, yaw)
    return node

class URDFType(object):
    """Abstract base class for all URDF types.
    """
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
                if a in node.attrib:
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
                vs = node.findall(t.TAG)
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
        for a in self.ELEMENTS:
            t, r, m = self.ELEMENTS[a]
            v = getattr(self, a, None)
            if not m:
                if r or v is not None:
                    node.append(v._to_xml(node, path))
            else:
                vs = v
                for v in vs:
                    node.append(v._to_xml(node, path))

    def _unparse(self, parent, path):
        node = ET.Element(self.TAG)
        self._unparse_simple_attribs(node)
        self._unparse_simple_elements(node, path)
        return node

    def _to_xml(self, parent, path):
        return self._unparse(parent, path)

################################################################################
# Link types
################################################################################
class Box(URDFType):
    ATTRIBS = {
        'size' : (np.ndarray, True)
    }
    TAG = 'box'

    def __init__(self, size):
        self.size = size
        self._mesh = None

    @property
    def mesh(self):
        if self._mesh is None:
            self._mesh = trimesh.creation.box(extents=self.size)
        return self._mesh

class Cylinder(URDFType):
    ATTRIBS = {
        'radius' : (float, True),
        'length' : (float, True),
    }
    TAG = 'cylinder'

    def __init__(self, radius, length):
        self.radius = radius
        self.length = length
        self._mesh = None

    @property
    def mesh(self):
        if self._mesh is None:
            self._mesh = trimesh.creation.cylinder(radius=self.radius, height=self.height)
        return self._mesh

class Sphere(URDFType):
    ATTRIBS = {
        'radius' : (float, True),
    }
    TAG = 'sphere'

    def __init__(self, radius):
        self.radius = radius
        self._mesh = None

    @property
    def mesh(self):
        if self._mesh is None:
            self._mesh = trimesh.creation.icosphere(radius=self.radius)
        return self._mesh

class Mesh(URDFType):
    ATTRIBS = {
        'filename' : (str, True),
    }
    TAG = 'mesh'

    def __init__(self, filename, scale, mesh):
        self.filename = filename
        self.scale = scale
        self.mesh = mesh

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        # Process collision geometry, but not visual geometry
        process = True
        if node.getparent().getparent().tag == Visual.TAG:
            process = False
        mesh = trimesh.load_mesh(os.path.join(path, kwargs['filename']), process=process)
        if process and (isinstance(mesh, list) or isinstance(mesh, tuple)):
            m = mesh[0]
            for n in mesh[1:]:
                m += n
            mesh = m
        kwargs['mesh'] = mesh

        scale = None
        if 'scale' in node.attrib:
            scale = np.fromstring(node.attrib['scale'], sep=' ')
        kwargs['scale'] = scale

        # Get scale
        return Mesh(**kwargs)

    def _to_xml(self, parent, path):
        filepath = os.path.join(path, self.filename)
        p, _ = os.path.split(filepath)
        if not os.path.exists(p):
            os.makedirs(p)
        trimesh.exchange.export.export_mesh(self.mesh, filepath)
        node = self._unparse(parent, path)
        if self.scale is not None:
            node.attrib['scale'] = np.array2string(self.scale)[1:-1]
        return node

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

    @property
    def geometry(self):
        if self.box is not None:
            return self.box
        if self.cylinder is not None:
            return self.cylinder
        if self.sphere is not None:
            return self.sphere
        if self.mesh is not None:
            return self.mesh

    @property
    def trimesh(self):
        return self.geometry.mesh

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
        filepath = os.path.join(path, self.filename)
        p, _ = os.path.split(filepath)
        if not os.path.exists(p):
            os.makedirs(p)
        kwargs['image'] = ColorImage.open(filepath)
        return Texture(**kwargs)

    def _to_xml(self, parent, path):
        filepath = os.path.join(path, self.filename)
        p, _ = os.path.split(filepath)
        if not os.path.exists(p):
            os.makedirs(p)
        self.image.save(filepath)
        return self._unparse(parent, path)

class Material(URDFType):
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
        return Material(**kwargs)

    def _to_xml(self, parent, path):
        if parent.tag != 'robot':
            node = ET.Element('material')
            node.attrib['name'] = self.name
            return node
        else:
            node = self._unparse(parent, path)
            if self.color is not None:
                color = ET.Element('color')
                color.attrib['rgba'] = np.array2string(self.color)[1:-1]
                node.append(color)
            return node

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
        kwargs['origin'] = parse_origin(node)
        return Collision(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
        node.append(unparse_origin(self.origin))
        return node

class Visual(URDFType):
    ATTRIBS = {
        'name' : (str, False)
    }
    ELEMENTS = {
        'geometry' : (Geometry, True, False),
        'material' : (Material, False, False),
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
        kwargs['origin'] = parse_origin(node)
        return Visual(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
        node.append(unparse_origin(self.origin))
        return node

class Inertial(URDFType):
    TAG = 'inertial'

    def __init__(self, mass, inertia, origin):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin

    @classmethod
    def _from_xml(cls, node, path):
        origin = parse_origin(node)
        mass = float(node.find('mass').attrib['value'])
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

    def _to_xml(self, parent, path):
        node = ET.Element('inertial')
        node.append(unparse_origin(self.origin))
        mass = ET.Element('mass')
        mass.attrib['value'] = str(self.mass)
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

################################################################################
# Joint types
################################################################################

class JointCalibration(URDFType):
    ATTRIBS = {
        'rising' : (float, False),
        'falling' : (float, False)
    }
    TAG = 'calibration'

    def __init__(self, rising, falling):
        self.rising = rising
        self.falling = falling

class JointDynamics(URDFType):
    ATTRIBS = {
        'damping' : (float, False),
        'friction' : (float, False),
    }
    TAG = 'dynamics'

    def __init__(self, damping, friction):
        self.damping = damping
        self.friction = friction

class JointLimit(URDFType):
    ATTRIBS = {
        'lower' : (float, False),
        'upper' : (float, False),
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
        'multiplier' : (float, False),
        'offset' : (float, False),
    }
    TAG = 'mimic'

    def __init__(self, joint, multiplier, offset):
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset

class SafetyController(URDFType):
    ATTRIBS = {
        'soft_lower_limit' : (float, False),
        'soft_upper_limit' : (float, False),
        'k_position' : (float, False),
        'k_velocity' : (float, True),
    }
    TAG = 'safety_controller'

    def __init__(self, soft_lower_limit, soft_upper_limit, k_position, k_velocity):
        self.k_velocity = k_velocity
        self.k_position = k_position
        self.soft_lower_limit = soft_lower_limit
        self.soft_upper_limit = soft_upper_limit

################################################################################
# Transmission types
################################################################################

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

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
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
        self.hardwareInterfaces = hardwareInterfaces

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        hi = node.findall('hardwareInterface')
        if len(hi) > 0:
            hi = [h.text for h in hi]
        kwargs['hardwareInterfaces'] = hi
        return TransmissionJoint(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
        if len(self.hardwareInterfaces) > 0:
            for hi in self.hardwareInterfaces:
                h = ET.Element('hardwareInterface')
                h.text = hi
                node.append(h)
        return node

################################################################################
# Top-level types
################################################################################

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

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
        ttype = ET.Element('type')
        ttype.text = self.type
        node.append(ttype)
        return node

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

    def get_child_pose(self, cfg=None):
        if cfg is None or self.type == 'fixed':
            pose = self.origin
        elif self.type == 'revolute' or self.type == 'continuous':
            rotation = trimesh.transformations.rotation_matrix(cfg, self.axis)
            pose = self.origin.dot(rotation)
        elif self.type == 'prismatic':
            translation = np.eye(4)
            translation[:3,3] = self.axis * cfg
            pose = self.origin.dot(translation)
        else:
            raise NotImplementedError('Unsupported joint type: {}'.format(self.type))
        return pose

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['parent'] = node.find('parent').attrib['link']
        kwargs['child'] = node.find('child').attrib['link']
        axis = node.find('axis')
        if axis is not None:
            axis = np.fromstring(axis.attrib['xyz'], sep=' ')
        else:
            if kwargs['type'] in set(['revolute', 'prismatic', 'planar']):
                axis = np.array([1.0, 0.0, 0.0])
        kwargs['axis'] = axis
        kwargs['origin'] = parse_origin(node)
        return Joint(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
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
        node.append(unparse_origin(self.origin))
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

        self._visual_meshes = None
        self._collision_mesh = None

    @property
    def visual_meshes(self):
        """list of :obj:`trimesh.Trimesh`: The visual meshes that make up
        this link. All meshes are specified in the link frame.
        """
        if self._visual_meshes is None:
            meshes = []
            for v in self.visuals:
                pose = v.origin
                m = v.geometry.trimesh.copy()
                m.apply_transform(pose)
                meshes.append(m)
            self._visual_meshes = meshes
        return self._visual_meshes

    @property
    def collision_mesh(self):
        """:obj:`trimesh.Trimesh`: The collision mesh that makes up this link.
        The mesh is specified in the link frame.
        """
        if len(self.collisions) == 0:
            return None
        if self._collision_mesh is None:
            m = self.collisions[0].geometry.trimesh.copy()
            pose = self.collisions[0].origin
            m.apply_transform(pose)
            for c in self.collisions[1:]:
                nm = c.geometry.trimesh.copy()
                nm.apply_transform(c.origin)
                m += nm
            self._collision_mesh = m
        return self._collision_mesh

class URDF(URDFType):
    ATTRIBS = {
        'name' : (str, True),
    }
    ELEMENTS = {
        'joints' : (Joint, False, True),
        'links' : (Link, True, True),
        'transmissions' : (Transmission, False, True),
        'materials' : (Material, False, True),
    }
    TAG = 'robot'

    def __init__(self, name, joints, links,
                 transmissions, materials, other_xml):
        self.name = name
        self.joints = joints
        self.links = links
        self.transmissions = transmissions
        self.materials = materials
        self.other_xml = other_xml

        # Create link map and joint map
        self._link_map = { l.name : l for l in self.links }
        self._joint_map = { j.name : j for j in self.joints }

        # Create graph
        G = nx.Graph()
        for l in self.links:
            G.add_node(l)

        base_links = set(self.links)
        end_links = set(self.links)
        for j in self.joints:
            parent = self._link_map[j.parent]
            child = self._link_map[j.child]
            G.add_edge(child, parent, object=j)
            if child in base_links:
                base_links.remove(child)
            if parent in end_links:
                end_links.remove(parent)

        # Check for single base link
        if len(base_links) != 1:
            msg = 'URDF has multiple base links: '
            for l in base_links:
                msg += '{} '.format(l.name)
            raise ValueError(msg)

        # Check for cycles
        # TODO SUPPORT PARALLEL-LINK ROBOTS/GRIPPERS
        cycle_bases = nx.cycle_basis(G)
        if len(cycle_bases) > 0:
            raise ValueError('URDF does not support cycles')

        self._graph = G
        self._base_link = list(base_links)[0]
        self._end_links = list(end_links)
        self._paths_to_base = nx.single_target_shortest_path(G, self._base_link)

    @property
    def base_link(self):
        return self._base_link

    @property
    def end_links(self):
        return self._end_links

    def forward_kinematics(self, joint_cfg=None, link_names=None):
        """From a dictionary mapping joint names to joint configurations
        (float or (2,) vector for planar joints), compute the pose of each link.
        """
        if joint_cfg is None:
            joint_cfg = {}

        # Iterate over the links and compute poses for each
        if link_names is not None:
            links = [l for l in self.links if l.name in link_names]
        else:
            links = self.links

        # Compute the pose of each link
        link_to_pose = { l : None for l in links }

        # Iterate over the links and compute poses for each
        for link in links:
            pose = np.eye(4)
            path = self._paths_to_base[link]
            for i in range(len(path)-1):
                child = path[i]
                parent = path[i+1]

                # Get joint
                joint = self._graph.get_edge_data(child, parent)['object']

                # Get joint cfg
                cfg = None
                if joint.name in joint_cfg:
                    cfg = joint_cfg[joint.name]
                elif joint.mimic is not None:
                    mimic_joint = self._joint_map[joint.mimic.joint]
                    if mimic_joint.name in joint_cfg:
                        cfg = joint_cfg[mimic_joint.name]
                        multiplier = 1.0
                        offset = 0.0
                        if joint.mimic.multiplier is not None:
                            multiplier = joint.mimic.multiplier
                        if joint.mimic.offset is not None:
                            offset = joint.mimic.offset
                        cfg = multiplier * cfg + offset
                child_pose = joint.get_child_pose(cfg)

                pose = child_pose.dot(pose)

                if parent in link_to_pose and link_to_pose[parent] is not None:
                    pose = link_to_pose[parent].dot(pose)
                    break

            link_to_pose[link] = pose

        return link_to_pose

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)

        extra_xml_node = ET.Element('extra')
        for child in node:
            if child.tag not in set(['joint', 'link', 'transmission', 'material']):
                extra_xml_node.append(child)

        data = ET.tostring(extra_xml_node)
        kwargs['other_xml'] = data
        return URDF(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(parent, path)
        extra_tree = ET.fromstring(self.other_xml)
        for child in extra_tree:
            node.append(child)
        return node

    @staticmethod
    def from_xml_file(file_obj):
        if isinstance(file_obj, six.string_types):
            if os.path.isfile(file_obj):
                parser = ET.XMLParser(remove_comments=True, remove_blank_text=True)
                tree = ET.parse(file_obj, parser=parser)
                path, _ = os.path.split(file_obj)
            else:
                raise ValueError('{} is not a file'.format(file_obj))
        else:
            parser = ET.XMLParser(remove_comments=True, remove_blank_text=True)
            tree = ET.parse(file_obj, parser=parser)
            path, _ = os.path.split(file_obj.name)

        node = tree.getroot()
        return URDF._from_xml(node, path)

    def to_xml_file(self, file_obj):
        if isinstance(file_obj, six.string_types):
            path, _ = os.path.split(file_obj)
        else:
            path, _ = os.path.split(file_obj.name)

        node = self._to_xml(None, path)
        tree = ET.ElementTree(node)
        tree.write(file_obj, pretty_print=True, xml_declaration=True, encoding='utf-8')
