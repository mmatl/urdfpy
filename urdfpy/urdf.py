import copy
import os
import time

from lxml import etree as ET
import networkx as nx
import numpy as np
import PIL
import trimesh
import six

from .utils import (parse_origin, unparse_origin, get_filename, load_meshes,
                    configure_origin)


class URDFType(object):
    """Abstract base class for all URDF types.

    This has useful class methods for automatic parsing/unparsing
    of XML trees.

    There are three overridable class variables:

    - ``_ATTRIBS`` - This is a dictionary mapping attribute names to a tuple,
      ``(type, required)`` where ``type`` is the Python type for the
      attribute and ``required`` is a boolean stating whether the attribute
      is required to be present.
    - ``_ELEMENTS`` - This is a dictionary mapping element names to a tuple,
      ``(type, required, multiple)`` where ``type`` is the Python type for the
      element, ``required`` is a boolean stating whether the element
      is required to be present, and ``multiple`` is a boolean indicating
      whether multiple elements of this type could be present.
      Elements are child nodes in the XML tree, and their type must be a
      subclass of :class:`.URDFType`.
    - ``_TAG`` - This is a string that represents the XML tag for the node
      containing this type of object.
    """
    _ATTRIBS = {}   # Map from attrib name to (type, required)
    _ELEMENTS = {}  # Map from element name to (type, required, multiple)
    _TAG = ''       # XML tag for this element

    def __init__(self):
        pass

    @classmethod
    def _parse_attrib(cls, val_type, val):
        """Parse an XML attribute into a python value.

        Parameters
        ----------
        val_type : :class:`type`
            The type of value to create.
        val : :class:`object`
            The value to parse.

        Returns
        -------
        val : :class:`object`
            The parsed attribute.
        """
        if val_type == np.ndarray:
            val = np.fromstring(val, sep=' ')
        else:
            val = val_type(val)
        return val

    @classmethod
    def _parse_simple_attribs(cls, node):
        """Parse all attributes in the _ATTRIBS array for this class.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse attributes for.

        Returns
        -------
        kwargs : dict
            Map from attribute name to value. If the attribute is not
            required and is not present, that attribute's name will map to
            ``None``.
        """
        kwargs = {}
        for a in cls._ATTRIBS:
            t, r = cls._ATTRIBS[a]  # t = type, r = required (bool)
            if r:
                try:
                    v = cls._parse_attrib(t, node.attrib[a])
                except Exception:
                    raise ValueError(
                        'Missing required attribute {} when parsing an object '
                        'of type {}'.format(a, cls.__name__)
                    )
            else:
                v = None
                if a in node.attrib:
                    v = cls._parse_attrib(t, node.attrib[a])
            kwargs[a] = v
        return kwargs

    @classmethod
    def _parse_simple_elements(cls, node, path):
        """Parse all elements in the _ELEMENTS array from the children of
        this node.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse children for.
        path : str
            The string path where the XML file is located (used for resolving
            the location of mesh or image files).

        Returns
        -------
        kwargs : dict
            Map from element names to the :class:`URDFType` subclass (or list,
            if ``multiple`` was set) created for that element.
        """
        kwargs = {}
        for a in cls._ELEMENTS:
            t, r, m = cls._ELEMENTS[a]
            if not m:
                v = node.find(t._TAG)
                if r or v is not None:
                    v = t._from_xml(v, path)
            else:
                vs = node.findall(t._TAG)
                if len(vs) == 0 and r:
                    raise ValueError(
                        'Missing required subelement(s) of type {} when '
                        'parsing an object of type {}'.format(
                            t.__name__, cls.__name__
                        )
                    )
                v = [t._from_xml(n, path) for n in vs]
            kwargs[a] = v
        return kwargs

    @classmethod
    def _parse(cls, node, path):
        """Parse all elements and attributes in the _ELEMENTS and _ATTRIBS
        arrays for a node.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse.
        path : str
            The string path where the XML file is located (used for resolving
            the location of mesh or image files).

        Returns
        -------
        kwargs : dict
            Map from names to Python classes created from the attributes
            and elements in the class arrays.
        """
        kwargs = cls._parse_simple_attribs(node)
        kwargs.update(cls._parse_simple_elements(node, path))
        return kwargs

    @classmethod
    def _from_xml(cls, node, path):
        """Create an instance of this class from an XML node.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse.
        path : str
            The string path where the XML file is located (used for resolving
            the location of mesh or image files).

        Returns
        -------
        obj : :class:`URDFType`
            An instance of this class parsed from the node.
        """
        return cls(**cls._parse(node, path))

    def _unparse_attrib(self, val_type, val):
        """Convert a Python value into a string for storage in an
        XML attribute.

        Parameters
        ----------
        val_type : :class:`type`
            The type of the Python object.
        val : :class:`object`
            The actual value.

        Returns
        -------
        s : str
            The attribute string.
        """
        if val_type == np.ndarray:
            val = np.array2string(val)[1:-1]
        else:
            val = str(val)
        return val

    def _unparse_simple_attribs(self, node):
        """Convert all Python types from the _ATTRIBS array back into attributes
        for an XML node.

        Parameters
        ----------
        node : :class:`object`
            The XML node to add the attributes to.
        """
        for a in self._ATTRIBS:
            t, r = self._ATTRIBS[a]
            v = getattr(self, a, None)
            if r or v is not None:
                node.attrib[a] = self._unparse_attrib(t, v)

    def _unparse_simple_elements(self, node, path):
        """Unparse all Python types from the _ELEMENTS array back into child
        nodes of an XML node.

        Parameters
        ----------
        node : :class:`object`
            The XML node for this object. Elements will be added as children
            of this node.
        path : str
            The string path where the XML file is being written to (used for
            writing out meshes and image files).
        """
        for a in self._ELEMENTS:
            t, r, m = self._ELEMENTS[a]
            v = getattr(self, a, None)
            if not m:
                if r or v is not None:
                    node.append(v._to_xml(node, path))
            else:
                vs = v
                for v in vs:
                    node.append(v._to_xml(node, path))

    def _unparse(self, path):
        """Create a node for this object and unparse all elements and
        attributes in the class arrays.

        Parameters
        ----------
        path : str
            The string path where the XML file is being written to (used for
            writing out meshes and image files).

        Returns
        -------
        node : :class:`lxml.etree.Element`
            The newly-created node.
        """
        node = ET.Element(self._TAG)
        self._unparse_simple_attribs(node)
        self._unparse_simple_elements(node, path)
        return node

    def _to_xml(self, parent, path):
        """Create and return an XML node for this object.

        Parameters
        ----------
        parent : :class:`lxml.etree.Element`
            The parent node that this element will eventually be added to.
            This base implementation doesn't use this information, but
            classes that override this function may use it.
        path : str
            The string path where the XML file is being written to (used for
            writing out meshes and image files).

        Returns
        -------
        node : :class:`lxml.etree.Element`
            The newly-created node.
        """
        return self._unparse(path)

###############################################################################
# Link types
###############################################################################


class Box(URDFType):
    """A rectangular prism whose center is at the local origin.

    Parameters
    ----------
    size : (3,) float
        The length, width, and height of the box in meters.
    """

    _ATTRIBS = {
        'size': (np.ndarray, True)
    }
    _TAG = 'box'

    def __init__(self, size):
        self.size = size
        self._meshes = []

    @property
    def size(self):
        """(3,) float : The length, width, and height of the box in meters.
        """
        return self._size

    @size.setter
    def size(self, value):
        self._size = np.asanyarray(value).astype(np.float)
        self._meshes = []

    @property
    def meshes(self):
        """list of :class:`~trimesh.base.Trimesh` : The triangular meshes
        that represent this object.
        """
        if len(self._meshes) == 0:
            self._meshes = [trimesh.creation.box(extents=self.size)]
        return self._meshes


class Cylinder(URDFType):
    """A cylinder whose center is at the local origin.

    Parameters
    ----------
    radius : float
        The radius of the cylinder in meters.
    length : float
        The length of the cylinder in meters.
    """

    _ATTRIBS = {
        'radius': (float, True),
        'length': (float, True),
    }
    _TAG = 'cylinder'

    def __init__(self, radius, length):
        self.radius = radius
        self.length = length
        self._meshes = None

    @property
    def radius(self):
        """float : The radius of the cylinder in meters.
        """
        return self._radius

    @radius.setter
    def radius(self, value):
        self._radius = float(value)
        self._meshes = None

    @property
    def length(self):
        """float : The length of the cylinder in meters.
        """
        return self._length

    @length.setter
    def length(self, value):
        self._length = float(value)
        self._meshes = None

    @property
    def meshes(self):
        """list of :class:`~trimesh.base.Trimesh` : The triangular meshes
        that represent this object.
        """
        if len(self._meshes) == 0:
            self._meshes = [trimesh.creation.cylinder(
                radius=self.radius, height=self.length
            )]
        return self._mesh


class Sphere(URDFType):
    """A sphere whose center is at the local origin.

    Parameters
    ----------
    radius : float
        The radius of the sphere in meters.
    """
    _ATTRIBS = {
        'radius': (float, True),
    }
    _TAG = 'sphere'

    def __init__(self, radius):
        self.radius = radius
        self._meshes = []

    @property
    def radius(self):
        """float : The radius of the sphere in meters.
        """
        return self._radius

    @radius.setter
    def radius(self, value):
        self._radius = float(value)
        self._meshes = []

    @property
    def meshes(self):
        """list of :class:`~trimesh.base.Trimesh` : The triangular meshes
        that represent this object.
        """
        if len(self._meshes) == 0:
            self._meshes = [trimesh.creation.icosphere(radius=self.radius)]
        return self._meshes


class Mesh(URDFType):
    """A triangular mesh object.

    Parameters
    ----------
    filename : str
        The path to the mesh that contains this object. This can be
        relative to the top-level URDF or an absolute path.
    scale : (3,) float, optional
        The scaling value for the mesh along the XYZ axes.
        If ``None``, assumes no scale is applied.
    meshes : list of :class:`~trimesh.base.Trimesh`
        A list of meshes that compose this mesh.
        The list of meshes is useful for visual geometries that
        might be composed of separate trimesh objects.
        If not specified, the mesh is loaded from the file using trimesh.
    """
    _ATTRIBS = {
        'filename': (str, True),
        'scale': (np.ndarray, False)
    }
    _TAG = 'mesh'

    def __init__(self, filename, scale=None, meshes=None):
        if meshes is None:
            meshes = load_meshes(filename)
        self.filename = filename
        self.scale = scale
        self.meshes = meshes

    @property
    def filename(self):
        """str : The path to the mesh file for this object.
        """
        return self._filename

    @filename.setter
    def filename(self, value):
        self._filename = value

    @property
    def scale(self):
        """(3,) float : A scaling for the mesh along its local XYZ axes.
        """
        return self._scale

    @scale.setter
    def scale(self, value):
        if value is not None:
            value = np.asanyarray(value).astype(np.float)
        self._scale = value

    @property
    def meshes(self):
        """list of :class:`~trimesh.base.Trimesh` : The triangular meshes
        that represent this object.
        """
        return self._meshes

    @meshes.setter
    def meshes(self, value):
        if isinstance(value, six.string_types):
            value = load_meshes(value)
        elif isinstance(value, (list, tuple, set)):
            value = list(value)
            if len(value) == 0:
                raise ValueError('Mesh must have at least one trimesh.Trimesh')
            for m in value:
                if not isinstance(m, trimesh.Trimesh):
                    raise TypeError('Mesh requires a trimesh.Trimesh or a '
                                    'list of them')
        elif isinstance(value, trimesh.Trimesh):
            value = [value]
        else:
            raise TypeError('Mesh requires a trimesh.Trimesh')
        self._meshes = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)

        # Load the mesh, combining collision geometry meshes but keeping
        # visual ones separate to preserve colors and textures
        fn = get_filename(path, kwargs['filename'])
        combine = node.getparent().getparent().tag == Collision._TAG
        meshes = load_meshes(fn)
        if combine:
            # Delete visuals for simplicity
            for m in meshes:
                m.visual = trimesh.visual.ColorVisuals(mesh=m)
            meshes = [meshes[0] + meshes[1:]]
        kwargs['meshes'] = meshes

        return Mesh(**kwargs)

    def _to_xml(self, parent, path):
        # Get the filename
        fn = get_filename(path, self.filename, makedirs=True)

        # Export the meshes as a single file
        meshes = self.meshes
        if len(meshes) == 1:
            meshes = meshes[0]
        trimesh.exchange.export.export_mesh(meshes, fn)

        # Unparse the node
        node = self._unparse(path)
        return node


class Geometry(URDFType):
    """A wrapper for all geometry types.

    Only one of the following values can be set, all others should be set
    to ``None``.

    Parameters
    ----------
    box : :class:`.Box`, optional
        Box geometry.
    cylinder : :class:`.Cylinder`
        Cylindrical geometry.
    sphere : :class:`.Sphere`
        Spherical geometry.
    mesh : :class:`.Mesh`
        Mesh geometry.
    """

    _ELEMENTS = {
        'box': (Box, False, False),
        'cylinder': (Cylinder, False, False),
        'sphere': (Sphere, False, False),
        'mesh': (Mesh, False, False),
    }
    _TAG = 'geometry'

    def __init__(self, box=None, cylinder=None, sphere=None, mesh=None):
        if (box is None and cylinder is None and
                sphere is None and mesh is None):
            raise ValueError('At least one geometry element must be set')
        self.box = box
        self.cylinder = cylinder
        self.sphere = sphere
        self.mesh = mesh

    @property
    def box(self):
        """:class:`.Box` : Box geometry.
        """
        return self._box

    @box.setter
    def box(self, value):
        if value is not None and not isinstance(value, Box):
            raise TypeError('Expected Box type')
        self._box = value

    @property
    def cylinder(self):
        """:class:`.Cylinder` : Cylinder geometry.
        """
        return self._cylinder

    @cylinder.setter
    def cylinder(self, value):
        if value is not None and not isinstance(value, Cylinder):
            raise TypeError('Expected Cylinder type')
        self._cylinder = value

    @property
    def sphere(self):
        """:class:`.Sphere` : Spherical geometry.
        """
        return self._sphere

    @sphere.setter
    def sphere(self, value):
        if value is not None and not isinstance(value, Sphere):
            raise TypeError('Expected Sphere type')
        self._sphere = value

    @property
    def mesh(self):
        """:class:`.Mesh` : Mesh geometry.
        """
        return self._mesh

    @mesh.setter
    def mesh(self, value):
        if value is not None and not isinstance(value, Mesh):
            raise TypeError('Expected Mesh type')
        self._mesh = value

    @property
    def geometry(self):
        """:class:`.Box`, :class:`.Cylinder`, :class:`.Sphere`, or
        :class:`.Mesh` : The valid geometry element.
        """
        if self.box is not None:
            return self.box
        if self.cylinder is not None:
            return self.cylinder
        if self.sphere is not None:
            return self.sphere
        if self.mesh is not None:
            return self.mesh
        return None

    @property
    def meshes(self):
        """list of :class:`~trimesh.base.Trimesh` : The geometry's triangular
        mesh representation(s).
        """
        return self.geometry.meshes


class Texture(URDFType):
    """An image-based texture.

    Parameters
    ----------
    filename : str
        The path to the image that contains this texture. This can be
        relative to the top-level URDF or an absolute path.
    image : :class:`PIL.Image.Image`, optional
        The image for the texture.
        If not specified, it is loaded automatically from the filename.
    """

    _ATTRIBS = {
        'filename': (str, True)
    }
    _TAG = 'texture'

    def __init__(self, filename, image=None):
        if image is None:
            image = PIL.image.open(filename)
        self.filename = filename
        self.image = image

    @property
    def filename(self):
        """str : Path to the image for this texture.
        """
        return self._filename

    @filename.setter
    def filename(self, value):
        self._filename = str(value)

    @property
    def image(self):
        """:class:`PIL.Image.Image` : The image for this texture.
        """
        return self._image

    @image.setter
    def image(self, value):
        if isinstance(value, str):
            value = PIL.Image.open(value)
        if isinstance(value, np.ndarray):
            value = PIL.Image.fromarray(value)
        elif not isinstance(value, PIL.Image.Image):
            raise ValueError('Texture only supports numpy arrays '
                             'or PIL images')
        self._image = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)

        # Load image
        fn = get_filename(path, kwargs['filename'])
        kwargs['image'] = PIL.Image.open(fn)

        return Texture(**kwargs)

    def _to_xml(self, parent, path):
        # Save the image
        filepath = get_filename(path, self.filename, makedirs=True)
        self.image.save(filepath)

        return self._unparse(path)


class Material(URDFType):
    """A material for some geometry.

    Parameters
    ----------
    name : str
        The name of the material.
    color : (4,) float, optional
        The RGBA color of the material in the range [0,1].
    texture : :class:`.Texture`, optional
        A texture for the material.
    """
    _ATTRIBS = {
        'name': (str, True)
    }
    _ELEMENTS = {
        'texture': (Texture, False, False),
    }
    _TAG = 'material'

    def __init__(self, name, color=None, texture=None):
        self.name = name
        self.color = color
        self.texture = texture

    @property
    def name(self):
        """str : The name of the material.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def color(self):
        """(4,) float : The RGBA color of the material, in the range [0,1].
        """
        return self._color

    @color.setter
    def color(self, value):
        if value is not None:
            value = np.asanyarray(value).astype(np.float)
            value = np.clip(value, 0.0, 1.0)
            if value.shape != (4,):
                raise ValueError('Color must be a (4,) float')
        self._color = value

    @property
    def texture(self):
        """:class:`.Texture` : The texture for the material.
        """
        return self._texture

    @texture.setter
    def texture(self, value):
        if value is not None:
            if isinstance(value, six.string_types):
                image = PIL.Image.open(value)
                value = Texture(filename=value, image=image)
            elif not isinstance(value, Texture):
                raise ValueError('Invalid type for texture -- expect path to '
                                 'image or Texture')
        self._texture = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)

        # Extract the color -- it's weirdly an attribute of a subelement
        color = node.find('color')
        if color is not None:
            color = np.fromstring(color.attrib['rgba'], sep=' ')
        kwargs['color'] = color

        return Material(**kwargs)

    def _to_xml(self, parent, path):
        # Simplify materials by collecting them at the top level.

        # For top-level elements, save the full material specification
        if parent.tag == 'robot':
            node = self._unparse(path)
            if self.color is not None:
                color = ET.Element('color')
                color.attrib['rgba'] = np.array2string(self.color)[1:-1]
                node.append(color)

        # For non-top-level elements just save the material with a name
        else:
            node = ET.Element('material')
            node.attrib['name'] = self.name
        return node


class Collision(URDFType):
    """Collision properties of a link.

    Parameters
    ----------
    geometry : :class:`.Geometry`
        The geometry of the element
    name : str, optional
        The name of the collision geometry.
    origin : (4,4) float, optional
        The pose of the collision element relative to the link frame.
        Defaults to identity.
    """

    _ATTRIBS = {
        'name': (str, False)
    }
    _ELEMENTS = {
        'geometry': (Geometry, True, False),
    }
    _TAG = 'collision'

    def __init__(self, name, origin, geometry):
        self.geometry = geometry
        self.name = name
        self.origin = origin

    @property
    def geometry(self):
        """:class:`.Geometry` : The geometry of this element.
        """
        return self._geometry

    @geometry.setter
    def geometry(self, value):
        if not isinstance(value, Geometry):
            raise TypeError('Must set geometry with Geometry object')
        self._geometry = value

    @property
    def name(self):
        """str : The name of this collision element.
        """
        return self._name

    @name.setter
    def name(self, value):
        if value is not None:
            value = str(value)
        self._name = value

    @property
    def origin(self):
        """(4,4) float : The pose of this element relative to the link frame.
        """
        return self._origin

    @origin.setter
    def origin(self, value):
        self._origin = configure_origin(value)

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['origin'] = parse_origin(node)
        return Collision(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        node.append(unparse_origin(self.origin))
        return node


class Visual(URDFType):
    """Visual properties of a link.

    Parameters
    ----------
    geometry : :class:`.Geometry`
        The geometry of the element
    name : str, optional
        The name of the visual geometry.
    origin : (4,4) float, optional
        The pose of the visual element relative to the link frame.
        Defaults to identity.
    material : :class:`.Material`, optional
        The material of the element.
    """
    _ATTRIBS = {
        'name': (str, False)
    }
    _ELEMENTS = {
        'geometry': (Geometry, True, False),
        'material': (Material, False, False),
    }
    _TAG = 'visual'

    def __init__(self, geometry, name=None, origin=None, material=None):
        self.geometry = geometry
        self.name = name
        self.origin = origin
        self.material = material

    @property
    def geometry(self):
        """:class:`.Geometry` : The geometry of this element.
        """
        return self._geometry

    @geometry.setter
    def geometry(self, value):
        if not isinstance(value, Geometry):
            raise TypeError('Must set geometry with Geometry object')
        self._geometry = value

    @property
    def name(self):
        """str : The name of this visual element.
        """
        return self._name

    @name.setter
    def name(self, value):
        if value is not None:
            value = str(value)
        self._name = value

    @property
    def origin(self):
        """(4,4) float : The pose of this element relative to the link frame.
        """
        return self._origin

    @origin.setter
    def origin(self, value):
        self._origin = configure_origin(value)

    @property
    def material(self):
        """:class:`.Material` : The material for this element.
        """
        return self._material

    @material.setter
    def material(self, value):
        if value is not None:
            if not isinstance(value, Material):
                raise TypeError('Must set material with Material object')
        self._material = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['origin'] = parse_origin(node)
        return Visual(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        node.append(unparse_origin(self.origin))
        return node


class Inertial(URDFType):
    """The inertial properties of a link.

    Parameters
    ----------
    mass : float
        The mass of the link in kilograms.
    inertia : (3,3) float
        The 3x3 symmetric rotational inertia matrix.
    origin : (4,4) float, optional
        The pose of the inertials relative to the link frame.
        Defaults to identity if not specified.
    """
    _TAG = 'inertial'

    def __init__(self, mass, inertia, origin=None):
        self.mass = mass
        self.inertia = inertia
        self.origin = origin

    @property
    def mass(self):
        """float : The mass of the link in kilograms.
        """
        return self._mass

    @mass.setter
    def mass(self, value):
        self._mass = float(value)

    @property
    def inertia(self):
        """(3,3) float : The 3x3 symmetric rotational inertia matrix.
        """
        return self._inertia

    @inertia.setter
    def inertia(self, value):
        value = np.asanyarray(value).astype(np.float)
        if not np.allclose(value, value.T):
            raise ValueError('Inertia must be a symmetric matrix')
        self._inertia = value

    @property
    def origin(self):
        """(4,4) float : The pose of the inertials relative to the link frame.
        """
        return self._origin

    @origin.setter
    def origin(self, value):
        self._origin = configure_origin(value)

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

###############################################################################
# Joint types
###############################################################################


class JointCalibration(URDFType):
    """The reference positions of the joint.

    Parameters
    ----------
    rising : float, optional
        When the joint moves in a positive direction, this position will
        trigger a rising edge.
    falling :
        When the joint moves in a positive direction, this position will
        trigger a falling edge.
    """
    _ATTRIBS = {
        'rising': (float, False),
        'falling': (float, False)
    }
    _TAG = 'calibration'

    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling

    @property
    def rising(self):
        """float : description.
        """
        return self._rising

    @rising.setter
    def rising(self, value):
        if value is not None:
            value = float(value)
        self._rising = value

    @property
    def falling(self):
        """float : description.
        """
        return self._falling

    @falling.setter
    def falling(self, value):
        if value is not None:
            value = float(value)
        self._falling = value


class JointDynamics(URDFType):
    """The dynamic properties of the joint.

    Parameters
    ----------
    damping : float
        The damping value of the joint (Ns/m for prismatic joints,
        Nms/rad for revolute).
    friction : float
        The static friction value of the joint (N for prismatic joints,
        Nm for revolute).
    """
    _ATTRIBS = {
        'damping': (float, False),
        'friction': (float, False),
    }
    _TAG = 'dynamics'

    def __init__(self, damping, friction):
        self.damping = damping
        self.friction = friction

    @property
    def damping(self):
        """float : The damping value of the joint.
        """
        return self._damping

    @damping.setter
    def damping(self, value):
        if value is not None:
            value = float(value)
        self._damping = value

    @property
    def friction(self):
        """float : The static friction value of the joint.
        """
        return self._friction

    @friction.setter
    def friction(self, value):
        if value is not None:
            value = float(value)
        self._friction = value


class JointLimit(URDFType):
    """The limits of the joint.

    Parameters
    ----------
    effort : float
        The maximum joint effort (N for prismatic joints, Nm for revolute).
    velocity : float
        The maximum joint velocity (m/s for prismatic joints, rad/s for
        revolute).
    lower : float, optional
        The lower joint limit (m for prismatic joints, rad for revolute).
    upper : float, optional
        The upper joint limit (m for prismatic joints, rad for revolute).
    """

    _ATTRIBS = {
        'effort': (float, True),
        'velocity': (float, True),
        'lower': (float, False),
        'upper': (float, False),
    }
    _TAG = 'limit'

    def __init__(self, effort, velocity, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

    @property
    def effort(self):
        """float : The maximum joint effort.
        """
        return self._effort

    @effort.setter
    def effort(self, value):
        self._effort = float(value)

    @property
    def velocity(self):
        """float : The maximum joint velocity.
        """
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        self._velocity = float(value)

    @property
    def lower(self):
        """float : The lower joint limit.
        """
        return self._lower

    @lower.setter
    def lower(self, value):
        if value is not None:
            value = float(value)
        self._lower = value

    @property
    def upper(self):
        """float : The upper joint limit.
        """
        return self._upper

    @upper.setter
    def upper(self, value):
        if value is not None:
            value = float(value)
        self._upper = value


class JointMimic(URDFType):
    """A mimicry tag for a joint, which forces its configuration to
    mimic another joint's.

    This joint's configuration value is set equal to
    ``multiplier * other_joint_cfg + offset``.

    Parameters
    ----------
    joint : str
        The name of the joint to mimic.
    multiplier : float
        The joint configuration multiplier. Defaults to 1.0.
    offset : float, optional
        The joint configuration offset. Defaults to 0.0.
    """
    _ATTRIBS = {
        'joint': (str, True),
        'multiplier': (float, False),
        'offset': (float, False),
    }
    _TAG = 'mimic'

    def __init__(self, joint, multiplier=None, offset=None):
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset

    @property
    def joint(self):
        """float : The name of the joint to mimic.
        """
        return self._joint

    @joint.setter
    def joint(self, value):
        self._joint = str(value)

    @property
    def multiplier(self):
        """float : The multiplier for the joint configuration.
        """
        return self._multiplier

    @multiplier.setter
    def multiplier(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 1.0
        self._multiplier = value

    @property
    def offset(self):
        """float : The offset for the joint configuration
        """
        return self._offset

    @offset.setter
    def offset(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._offset = value


class SafetyController(URDFType):
    """A controller for joint movement safety.

    Parameters
    ----------
    k_velocity : float
        An attribute specifying the relation between the effort and velocity
        limits.
    k_position : float, optional
        An attribute specifying the relation between the position and velocity
        limits. Defaults to 0.0.
    soft_lower_limit : float, optional
        The lower joint boundary where the safety controller kicks in.
        Defaults to 0.0.
    soft_upper_limit : float, optional
        The upper joint boundary where the safety controller kicks in.
        Defaults to 0.0.
    """
    _ATTRIBS = {
        'k_velocity': (float, True),
        'k_position': (float, False),
        'soft_lower_limit': (float, False),
        'soft_upper_limit': (float, False),
    }
    _TAG = 'safety_controller'

    def __init__(self, k_velocity, k_position=None, soft_lower_limit=None,
                 soft_upper_limit=None):
        self.k_velocity = k_velocity
        self.k_position = k_position
        self.soft_lower_limit = soft_lower_limit
        self.soft_upper_limit = soft_upper_limit

    @property
    def soft_lower_limit(self):
        """float : The soft lower limit where the safety controller kicks in.
        """
        return self._soft_lower_limit

    @soft_lower_limit.setter
    def soft_lower_limit(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._soft_lower_limit = value

    @property
    def soft_upper_limit(self):
        """float : The soft upper limit where the safety controller kicks in.
        """
        return self._soft_upper_limit

    @soft_upper_limit.setter
    def soft_upper_limit(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._soft_upper_limit = value

    @property
    def k_position(self):
        """float : A relation between the position and velocity limits.
        """
        return self._k_position

    @k_position.setter
    def k_position(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._k_position = value

    @property
    def k_velocity(self):
        """float : A relation between the effort and velocity limits.
        """
        return self._k_velocity

    @k_velocity.setter
    def k_velocity(self, value):
        self._k_velocity = float(value)

###############################################################################
# Transmission types
###############################################################################


class Actuator(URDFType):
    """An actuator.

    Parameters
    ----------
    name : str
        The name of this actuator.
    mechanicalReduction : str, optional
        A specifier for the mechanical reduction at the joint/actuator
        transmission.
    hardwareInterfaces : list of str, optional
        The supported hardware interfaces to the actuator.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _TAG = 'actuator'

    def __init__(self, name, mechanicalReduction=None,
                 hardwareInterfaces=None):
        self.name = name
        self.mechanicalReduction = mechanicalReduction
        self.hardwareInterfaces = hardwareInterfaces

    @property
    def name(self):
        """str : The name of this actuator.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def mechanicalReduction(self):
        """str : A specifier for the type of mechanical reduction.
        """
        return self._mechanicalReduction

    @mechanicalReduction.setter
    def mechanicalReduction(self, value):
        if value is not None:
            value = str(value)
        self._mechanicalReduction = value

    @property
    def hardwareInterfaces(self):
        """list of str : The supported hardware interfaces.
        """
        return self._hardwareInterfaces

    @hardwareInterfaces.setter
    def hardwareInterfaces(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for i, v in enumerate(value):
                value[i] = str(v)
        self._hardwareInterfaces = value

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
    """A transmission joint specification.

    Parameters
    ----------
    name : str
        The name of this actuator.
    hardwareInterfaces : list of str, optional
        The supported hardware interfaces to the actuator.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _TAG = 'joint'

    def __init__(self, name, hardwareInterfaces):
        self.name = name
        self.hardwareInterfaces = hardwareInterfaces

    @property
    def name(self):
        """str : The name of this transmission joint.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def hardwareInterfaces(self):
        """list of str : The supported hardware interfaces.
        """
        return self._hardwareInterfaces

    @hardwareInterfaces.setter
    def hardwareInterfaces(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for i, v in enumerate(value):
                value[i] = str(v)
        self._hardwareInterfaces = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        hi = node.findall('hardwareInterface')
        if len(hi) > 0:
            hi = [h.text for h in hi]
        kwargs['hardwareInterfaces'] = hi
        return TransmissionJoint(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        if len(self.hardwareInterfaces) > 0:
            for hi in self.hardwareInterfaces:
                h = ET.Element('hardwareInterface')
                h.text = hi
                node.append(h)
        return node

###############################################################################
# Top-level types
###############################################################################


class Transmission(URDFType):
    """An element that describes the relationship between an actuator and a
    joint.

    Parameters
    ----------
    name : str
        The name of this transmission.
    trans_type : str
        The type of this transmission.
    joints : list of :class:`.TransmissionJoint`
        The joints connected to this transmission.
    actuators : list of :class:`.Actuator`
        The actuators connected to this transmission.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'joints': (TransmissionJoint, True, True),
        'actuators': (Actuator, True, True),
    }
    _TAG = 'transmission'

    def __init__(self, name, trans_type, joints=None, actuators=None):
        self.name = name
        self.trans_type = trans_type
        self.joints = joints
        self.actuators = actuators

    @property
    def name(self):
        """str : The name of this transmission.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def trans_type(self):
        """str : The type of this transmission.
        """
        return self._trans_type

    @trans_type.setter
    def trans_type(self, value):
        self._trans_type = str(value)

    @property
    def joints(self):
        """:class:`.TransmissionJoint` : The joints the transmission is
        connected to.
        """
        return self._joints

    @joints.setter
    def joints(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, TransmissionJoint):
                    raise TypeError(
                        'Joints expects a list of TransmissionJoint'
                    )
        self._joints = value

    @property
    def actuators(self):
        """:class:`.Actuator` : The actuators the transmission is connected to.
        """
        return self._actuators

    @actuators.setter
    def actuators(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, Actuator):
                    raise TypeError(
                        'Actuators expects a list of Actuator'
                    )
        self._actuators = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['trans_type'] = node.find('type').text
        return Transmission(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        ttype = ET.Element('type')
        ttype.text = self.trans_type
        node.append(ttype)
        return node


class Joint(URDFType):
    """A connection between two links.

    There are several types of joints, including:

    - ``fixed`` - a joint that cannot move.
    - ``prismatic`` - a joint that slides along the joint axis.
    - ``revolute`` - a hinge joint that rotates about the axis with a limited
      range of motion.
    - ``continuous`` - a hinge joint that rotates about the axis with an
      unlimited range of motion.
    - ``planar`` - a joint that moves in the plane orthogonal to the axis.
    - ``floating`` - a joint that can move in 6DoF.

    Parameters
    ----------
    name : str
        The name of this joint.
    parent : str
        The name of the parent link of this joint.
    child : str
        The name of the child link of this joint.
    joint_type : str
        The type of the joint. Must be one of :obj:`.Joint.TYPES`.
    axis : (3,) float, optional
        The axis of the joint specified in joint frame. Defaults to
        ``[1,0,0]``.
    origin : (4,4) float, optional
        The pose of the child link with respect to the parent link's frame.
        The joint frame is defined to be coincident with the child link's
        frame, so this is also the pose of the joint frame with respect to
        the parent link's frame.
    limit : :class:`.JointLimit`, optional
        Limit for the joint. Only required for revolute and prismatic
        joints.
    dynamics : :class:`.JointDynamics`, optional
        Dynamics for the joint.
    safety_controller : :class`.SafetyController`, optional
        The safety controller for this joint.
    calibration : :class:`.JointCalibration`, optional
        Calibration information for the joint.
    mimic : :class:`JointMimic`, optional
        Joint mimicry information.
    """
    TYPES = ['fixed', 'prismatic', 'revolute',
             'continuous', 'floating', 'planar']
    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'dynamics': (JointDynamics, False, False),
        'limit': (JointLimit, False, False),
        'mimic': (JointMimic, False, False),
        'safety_controller': (SafetyController, False, False),
        'calibration': (JointCalibration, False, False),
    }
    _TAG = 'joint'

    def __init__(self, name, joint_type, parent, child, axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.joint_type = joint_type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    @property
    def name(self):
        """str : Name for this joint.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def joint_type(self):
        """str : The type of this joint.
        """
        return self._joint_type

    @joint_type.setter
    def joint_type(self, value):
        value = str(value)
        if value not in Joint.TYPES:
            raise ValueError('Unsupported joint type {}'.format(value))
        self._joint_type = value

    @property
    def parent(self):
        """str : The name of the parent link.
        """
        return self._parent

    @parent.setter
    def parent(self, value):
        self._parent = str(value)

    @property
    def child(self):
        """str : The name of the child link.
        """
        return self._child

    @child.setter
    def child(self, value):
        self._child = str(value)

    @property
    def axis(self):
        """(3,) float : The joint axis in the joint frame.
        """
        return self._axis

    @axis.setter
    def axis(self, value):
        if value is None:
            value = np.array([1.0, 0.0, 0.0])
        else:
            value = np.asanyarray(value).astype(np.float)
            if value.shape != (3,):
                raise ValueError('Invalid shape for axis, should be (3,)')
            value = value / np.linalg.norm(value)
        self._axis = value

    @property
    def origin(self):
        """(4,4) float : The pose of child and joint frames relative to the
        parent link's frame.
        """
        return self._origin

    @origin.setter
    def origin(self, value):
        self._origin = configure_origin(value)

    @property
    def limit(self):
        """:class:`.JointLimit` : The limits for this joint.
        """
        return self._limit

    @limit.setter
    def limit(self, value):
        if value is None:
            if self.joint_type in ['prismatic', 'revolute']:
                raise ValueError('Require joint limit for prismatic and '
                                 'revolute joints')
        elif not isinstance(value, JointLimit):
            raise TypeError('Expected JointLimit type')
        self._limit = value

    @property
    def dynamics(self):
        """:class:`.JointDynamics` : The dynamics for this joint.
        """
        return self._dynamics

    @dynamics.setter
    def dynamics(self, value):
        if value is not None:
            if not isinstance(value, JointDynamics):
                raise TypeError('Expected JointDynamics type')
        self._dynamics = value

    @property
    def safety_controller(self):
        """:class:`.SafetyController` : The safety controller for this joint.
        """
        return self._safety_controller

    @safety_controller.setter
    def safety_controller(self, value):
        if value is not None:
            if not isinstance(value, SafetyController):
                raise TypeError('Expected SafetyController type')
        self._safety_controller = value

    @property
    def calibration(self):
        """:class:`.JointCalibration` : The calibration for this joint.
        """
        return self._calibration

    @calibration.setter
    def calibration(self, value):
        if value is not None:
            if not isinstance(value, JointCalibration):
                raise TypeError('Expected JointCalibration type')
        self._calibration = value

    @property
    def mimic(self):
        """:class:`.JointMimic` : The mimic for this joint.
        """
        return self._mimic

    @mimic.setter
    def mimic(self, value):
        if value is not None:
            if not isinstance(value, JointMimic):
                raise TypeError('Expected JointMimic type')
        self._mimic = value

    def is_valid(self, cfg):
        """Check if the provided configuration value is valid for this joint.

        Parameters
        ----------
        cfg : float, (2,) float, (6,) float, or (4,4) float
            The configuration of the joint.

        Returns
        -------
        is_valid : bool
            True if the configuration is valid, and False otherwise.
        """
        if self.joint_type not in ['fixed', 'revolute']:
            return True
        if self.joint_limit is None:
            return True
        cfg = float(cfg)
        lower = -np.infty
        upper = np.infty
        if self.limit.lower is not None:
            lower = self.limit.lower
        if self.limit.upper is not None:
            upper = self.limit.upper
        return (cfg >= lower and cfg <= upper)

    def get_child_pose(self, cfg=None):
        """Computes the child pose relative to a parent pose for a given
        configuration value.

        Parameters
        ----------
        cfg : float, (2,) float, (6,) float, or (4,4) float
            The configuration values for this joint. They are interpreted
            based on the joint type as follows:

            - ``fixed`` - not used.
            - ``prismatic`` - a translation along the axis in meters.
            - ``revolute`` - a rotation about the axis in radians.
            - ``continuous`` - a rotation about the axis in radians.
            - ``planar`` - the x and y translation values in the plane.
            - ``floating`` - the xyz values followed by the rpy values,
              or a (4,4) matrix.

            If ``cfg`` is ``None``, then this just returns the joint pose.

        Returns
        -------
        pose : (4,4) float
            The pose of the child relative to the parent.
        """
        if cfg is None:
            return self.origin
        elif self.joint_type == 'fixed':
            return self.origin
        elif self.joint_type in ['revolute', 'continuous']:
            if cfg is None:
                cfg = 0.0
            else:
                cfg = float(cfg)
            R = trimesh.transformations.rotation_matrix(cfg, self.axis)
            return self.origin.dot(R)
        elif self.joint_type == 'prismatic':
            if cfg is None:
                cfg = 0.0
            else:
                cfg = float(cfg)
            translation = np.eye(4)
            translation[:3,3] = self.axis * cfg
            return self.origin.dot(translation)
        elif self.joint_type == 'planar':
            if cfg is None:
                cfg = np.zeros(2)
            else:
                cfg = np.asanyarray(cfg)
            if cfg.shape != (2,):
                raise ValueError(
                    '(2,) float configuration required for planar joints'
                )
            translation = np.eye(4)
            translation[:3,3] = self.origin[:3,:2].dot(cfg)
            return self.origin.dot(translation)
        elif self.joint_type == 'floating':
            if cfg is None:
                cfg = np.zeros(6)
            else:
                cfg = configure_origin(cfg)
            if cfg is None:
                raise ValueError('Invalid configuration for floating joint')
            return self.origin.dot(cfg)
        else:
            raise ValueError('Invalid configuration')

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['joint_type'] = str(node.attrib['type'])
        kwargs['parent'] = node.find('parent').attrib['link']
        kwargs['child'] = node.find('child').attrib['link']
        axis = node.find('axis')
        if axis is not None:
            axis = np.fromstring(axis.attrib['xyz'], sep=' ')
        kwargs['axis'] = axis
        kwargs['origin'] = parse_origin(node)
        return Joint(**kwargs)

    def _to_xml(self, parent, path):
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
        node.append(unparse_origin(self.origin))
        node.attrib['type'] = self.joint_type
        return node


class Link(URDFType):
    """A link of a rigid object.

    Parameters
    ----------
    name : str
        The name of the link.
    inertial : :class:`.Inertial`, optional
        The inertial properties of the link.
    visuals : list of :class:`.Visual`, optional
        The visual properties of the link.
    collsions : list of :class:`.Collision`, optional
        The collision properties of the link.
    """

    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'inertial': (Inertial, False, False),
        'visuals': (Visual, False, True),
        'collisions': (Collision, False, True),
    }
    _TAG = 'link'

    def __init__(self, name, inertial, visuals, collisions):
        self.name = name
        self.inertial = inertial
        self.visuals = visuals
        self.collisions = collisions

        self._visual_meshes = None
        self._collision_mesh = None

    @property
    def name(self):
        """str : The name of this link.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def inertial(self):
        """:class:`.Inertial` : Inertial properties of the link.
        """
        return self._inertial

    @inertial.setter
    def inertial(self, value):
        if value is not None and not isinstance(value, Inertial):
            raise TypeError('Expected Intertial object')
        self._inertial = value

    @property
    def visuals(self):
        """list of :class:`.Visual` : The visual properties of this link.
        """
        return self._visuals

    @visuals.setter
    def visuals(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, Visual):
                    raise ValueError('Expected list of Visual objects')
        self._visuals = value

    @property
    def collisions(self):
        """list of :class:`.Collision` : The collision properties of this link.
        """
        return self._collisions

    @collisions.setter
    def collisions(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, Collision):
                    raise ValueError('Expected list of Collision objects')
        self._collisions = value

    @property
    def collision_mesh(self):
        """:class:`~trimesh.base.Trimesh` : A single collision mesh for
        the link, specified in the link frame, or None if there isn't one.
        """
        if len(self.collisions) == 0:
            return None
        if self._collision_mesh is None:
            meshes = []
            for c in self.collisions:
                for m in c.geometry.meshes:
                    m = m.copy()
                    pose = c.origin
                    if c.geometry.mesh is not None:
                        if c.geometry.mesh.scale is not None:
                            S = np.eye(4)
                            S[:3,:3] = np.diag(c.geometry.mesh.scale)
                            pose = pose.dot(S)
                    m.apply_transform(pose)
                    meshes.append(m)
            if len(meshes) == 0:
                return None
            self._collision_mesh = (meshes[0] + meshes[1:])
        return self._collision_mesh


class URDF(URDFType):
    """The top-level URDF specification.

    The URDF encapsulates an articulated object, such as a robot or a gripper.
    It is made of links and joints that tie them together and define their
    relative motions.

    Parameters
    ----------
    name : str
        The name of the URDF.
    links : list of :class:`.Link`
        The links of the URDF.
    joints : list of :class:`.Joint`, optional
        The joints of the URDF.
    transmissions : list of :class:`.Transmission`, optional
        The transmissions of the URDF.
    materials : list of :class:`.Material`, optional
        The materials for the URDF.
    other_xml : str, optional
        A string containing any extra XML for extensions.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'links': (Link, True, True),
        'joints': (Joint, False, True),
        'transmissions': (Transmission, False, True),
        'materials': (Material, False, True),
    }
    _TAG = 'robot'

    def __init__(self, name, links, joints=None,
                 transmissions=None, materials=None, other_xml=None):
        if joints is None:
            joints = []
        if transmissions is None:
            transmissions = []
        if materials is None:
            materials = []

        self.name = name
        self.other_xml = other_xml

        # No setters for these
        self._links = list(links)
        self._joints = list(joints)
        self._transmissions = list(transmissions)
        self._materials = list(materials)

        # Set up private helper maps from name to value
        self._link_map = {}
        self._joint_map = {}
        self._transmission_map = {}
        self._material_map = {}

        for x in self._links:
            if x.name in self._link_map:
                raise ValueError('Two links with name {} found'.format(x.name))
            self._link_map[x.name] = x

        for x in self._joints:
            if x.name in self._joint_map:
                raise ValueError('Two joints with name {} '
                                 'found'.format(x.name))
            self._joint_map[x.name] = x

        for x in self._transmissions:
            if x.name in self._transmission_map:
                raise ValueError('Two transmissions with name {} '
                                 'found'.format(x.name))
            self._transmission_map[x.name] = x

        for x in self._materials:
            if x.name in self._material_map:
                raise ValueError('Two materials with name {} '
                                 'found'.format(x.name))
            self._material_map[x.name] = x

        # Synchronize materials between links and top-level set
        self._merge_materials()

        # Validate the joints and transmissions
        self._actuated_joints = self._validate_joints()
        self._validate_transmissions()

        # Create the link graph and base link/end link sets
        self._G = nx.DiGraph()

        # Add all links
        for link in self.links:
            self._G.add_node(link)

        # Add all edges from CHILDREN TO PARENTS, with joints as their object
        for joint in self.joints:
            parent = self._link_map[joint.parent]
            child = self._link_map[joint.child]
            self._G.add_edge(child, parent, joint=joint)

        # Validate the graph and get the base and end links
        self._base_link, self._end_links = self._validate_graph()

        # Cache the paths to the base link
        self._paths_to_base = nx.shortest_path(
            self._G, target=self._base_link
        )

        # Cache the reverse topological order (useful for speeding up FK,
        # as we want to start at the base and work outward to cache
        # computation.
        self._reverse_topo = list(reversed(list(nx.topological_sort(self._G))))

    @property
    def name(self):
        """str : The name of the URDF.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def links(self):
        """list of :class:`.Link` : The links of the URDF.

        This returns a copy of the links array which cannot be edited
        directly. If you want to add or remove links, use
        the appropriate functions.
        """
        return copy.copy(self._links)

    @property
    def link_map(self):
        """dict : Map from link names to the links themselves.

        This returns a copy of the link map which cannot be edited
        directly. If you want to add or remove links, use
        the appropriate functions.
        """
        return copy.copy(self._link_map)

    @property
    def joints(self):
        """list of :class:`.Joint` : The links of the URDF.

        This returns a copy of the joints array which cannot be edited
        directly. If you want to add or remove joints, use
        the appropriate functions.
        """
        return copy.copy(self._joints)

    @property
    def joint_map(self):
        """dict : Map from joint names to the joints themselves.

        This returns a copy of the joint map which cannot be edited
        directly. If you want to add or remove joints, use
        the appropriate functions.
        """
        return copy.copy(self._joint_map)

    @property
    def transmissions(self):
        """list of :class:`.Transmission` : The transmissions of the URDF.

        This returns a copy of the transmissions array which cannot be edited
        directly. If you want to add or remove transmissions, use
        the appropriate functions.
        """
        return copy.copy(self._transmissions)

    @property
    def transmission_map(self):
        """dict : Map from transmission names to the transmissions themselves.

        This returns a copy of the transmission map which cannot be edited
        directly. If you want to add or remove transmissions, use
        the appropriate functions.
        """
        return copy.copy(self._transmission_map)

    @property
    def materials(self):
        """list of :class:`.Material` : The materials of the URDF.

        This returns a copy of the materials array which cannot be edited
        directly. If you want to add or remove materials, use
        the appropriate functions.
        """
        return copy.copy(self._materials)

    @property
    def material_map(self):
        """dict : Map from material names to the materials themselves.

        This returns a copy of the material map which cannot be edited
        directly. If you want to add or remove materials, use
        the appropriate functions.
        """
        return copy.copy(self._material_map)

    @property
    def other_xml(self):
        """str : Any extra XML that belongs with the URDF.
        """
        return self._other_xml

    @other_xml.setter
    def other_xml(self, value):
        self._other_xml = value

    @property
    def actuated_joints(self):
        """list of :class:`.Joint` : The joints that are independently
        actuated.

        This excludes mimic joints and fixed joints.
        """
        return self._actuated_joints

    @property
    def base_link(self):
        """:class:`.Link`: The base link for the URDF.

        The base link is the single link that has no parent.
        """
        return self._base_link

    @property
    def end_links(self):
        """list of :class:`.Link`: The end links for the URDF.

        The end links are the links that have no children.
        """
        return self._end_links

    @property
    def joint_limit_cfgs(self):
        """tuple of dict : The lower-bound and upper-bound joint configuration
        maps.

        The first map is the lower-bound map, which maps limited joints to
        their lower joint limits.
        The second map is the upper-bound map, which maps limited joints to
        their upper joint limits.
        """
        lb = {}
        ub = {}
        for joint in self.actuated_joints:
            if joint.limit is not None:
                if joint.limit.lower is not None:
                    lb[joint] = joint.limit.lower
                if joint.limit.upper is not None:
                    ub[joint] = joint.limit.upper
        return (lb, ub)

    def link_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's links via forward kinematics.

        Parameters
        ----------
        cfg : dict
            A map from joints or joint names to configuration values for
            each joint. If not specified, all joints are assumed to be
            in their default configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only these links will be in the returned map. If not specified,
            all links are returned.

        Returns
        -------
        fk : dict
            A map from links to 4x4 homogenous transform matrices that
            position them relative to the base link's frame.
        """

        # Process config value
        joint_cfg = {}
        if cfg is None:
            cfg = {}
        else:
            for joint in cfg:
                if isinstance(joint, six.string_types):
                    joint_cfg[self._joint_map[joint]] = cfg[joint]
                elif isinstance(joint, Joint):
                    joint_cfg[joint] = cfg[joint]
                else:
                    raise TypeError('Got key of type {} in cfg map'
                                    .format(type(joint)))

        # Process link set
        link_set = set()
        if links is None:
            link_set = self.links
        else:
            for link in links:
                if isinstance(link, six.string_types):
                    link_set.add(self._link_map[link])
                elif isinstance(link, Link):
                    link_set.add(link)
                else:
                    raise TypeError('Got object of type {} in links list'
                                    .format(type(link)))

        # Compute forward kinematics in reverse topological order
        fk = {}
        for link in self._reverse_topo:
            if link not in link_set:
                continue
            pose = np.eye(4)
            path = self._paths_to_base[link]
            for i in range(len(path) - 1):
                child = path[i]
                parent = path[i + 1]
                joint = self._G.get_edge_data(child, parent)['joint']

                cfg = None
                if joint.mimic is not None:
                    mimic_joint = self._joint_map[joint.mimic.joint]
                    if mimic_joint in joint_cfg:
                        cfg = joint_cfg[mimic_joint]
                        cfg = joint.mimic.multiplier * cfg + joint.mimic.offset
                elif joint in joint_cfg:
                    cfg = joint_cfg[joint]
                pose = joint.get_child_pose(cfg).dot(pose)

                # Check existing FK to see if we can exit early
                if parent in fk:
                    pose = fk[parent].dot(pose)
                    break
            fk[link] = pose

        return fk

    def visual_geometry_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's visual geometries using fk.

        Parameters
        ----------
        cfg : dict
            A map from joints or joint names to configuration values for
            each joint. If not specified, all joints are assumed to be
            in their default configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only geometries from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`Geometry` objects that are part of the visual
            elements of the specified links to the 4x4 homogenous transform
            matrices that position them relative to the base link's frame.
        """
        lfk = self.link_fk(cfg=cfg, links=links)

        fk = {}
        for link in lfk:
            for visual in link.visuals:
                fk[visual.geometry] = lfk[link].dot(visual.origin)
        return fk

    def visual_trimesh_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's visual trimeshes using fk.

        Parameters
        ----------
        cfg : dict
            A map from joints or joint names to configuration values for
            each joint. If not specified, all joints are assumed to be
            in their default configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only trimeshes from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`~trimesh.base.Trimesh` objects that are
            part of the visual geometry of the specified links to the
            4x4 homogenous transform matrices that position them relative
            to the base link's frame.
        """
        lfk = self.link_fk(cfg=cfg, links=links)

        fk = {}
        for link in lfk:
            for visual in link.visuals:
                for mesh in visual.geometry.meshes:
                    pose = lfk[link].dot(visual.origin)
                    if visual.geometry.mesh is not None:
                        if visual.geometry.mesh.scale is not None:
                            S = np.eye(4)
                            S[:3,:3] = np.diag(visual.geometry.mesh.scale)
                            pose = pose.dot(S)
                    fk[mesh] = pose
        return fk

    def collision_geometry_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's collision geometries using fk.

        Parameters
        ----------
        cfg : dict
            A map from joints or joint names to configuration values for
            each joint. If not specified, all joints are assumed to be
            in their default configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only geometries from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`Geometry` objects that are part of the collision
            elements of the specified links to the 4x4 homogenous transform
            matrices that position them relative to the base link's frame.
        """
        lfk = self.link_fk(cfg=cfg, links=links)

        fk = {}
        for link in lfk:
            for collision in link.collisions:
                fk[collision] = lfk[link].dot(collision.origin)
        return fk

    def collision_trimesh_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's collision trimeshes using fk.

        Parameters
        ----------
        cfg : dict
            A map from joints or joint names to configuration values for
            each joint. If not specified, all joints are assumed to be
            in their default configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only trimeshes from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`~trimesh.base.Trimesh` objects that are
            part of the collision geometry of the specified links to the
            4x4 homogenous transform matrices that position them relative
            to the base link's frame.
        """
        lfk = self.link_fk(cfg=cfg, links=links)

        fk = {}
        for link in lfk:
            if link.collision_mesh is not None:
                fk[link.collision_mesh] = lfk[link]
        return fk

    def animate(self, cfg_trajectory=None, loop_time=3.0, use_collision=False):
        """Animate the URDF through a configuration trajectory.

        Parameters
        ----------
        cfg_trajectory : dict
            A map from joints or joint names to lists of configuration values
            for each joint along the trajectory. If not specified,
            all joints will articulate from limit to limit.
            The trajectory steps are assumed to be equally spaced out in time.
        loop_time : float
            The time to loop the animation for, in seconds. The trajectory
            will play fowards and backwards during this time, ending
            at the inital configuration.
        use_collision : bool
            If True, the collision geometry is visualized instead of
            the visual geometry.

        Examples
        --------

        You can run this without specifying a ``cfg_trajectory`` to view
        the full articulation of the URDF

        >>> robot = URDF.load('ur5.urdf')
        >>> robot.animate()

        .. image:: /_static/ur5.gif

        >>> ct = {'shoulder_pan_joint': [0.0, 2 * np.pi]}
        >>> robot.animate(cfg_trajectory=ct)

        .. image:: /_static/ur5_shoulder.gif

        >>> ct = {
        ...    'shoulder_pan_joint' : [-np.pi / 4, np.pi / 4],
        ...    'shoulder_lift_joint' : [0.0, -np.pi / 2.0],
        ...    'elbow_joint' : [0.0, np.pi / 2.0]
        ... }
        >>> robot.animate(cfg_trajectory=ct)

        .. image:: /_static/ur5_three_joints.gif

        """
        import pyrender  # Save pyrender import for here for CI

        ct = cfg_trajectory

        traj_len = None  # Length of the trajectory in steps
        ct_np = {}       # Numpyified trajectory

        # If trajectory not specified, articulate between the limits.
        if ct is None:
            lb, ub = self.joint_limit_cfgs
            if len(lb) > 0:
                traj_len = 2
                ct_np = {k: np.array([lb[k], ub[k]]) for k in lb}

        # If it is specified, parse it and extract the trajectory length.
        elif isinstance(ct, dict):
            if len(ct) > 0:
                for k in ct:
                    val = np.asanyarray(ct[k]).astype(np.float)
                    if traj_len is None:
                        traj_len = len(val)
                    elif traj_len != len(val):
                        raise ValueError('Trajectories must be same length')
                    ct_np[k] = val
        else:
            raise TypeError('Invalid type for cfg_trajectory: {}'
                            .format(type(cfg_trajectory)))

        # If there isn't a trajectory to render, just show the model and exit
        if len(ct_np) == 0 or traj_len < 2:
            self.show(use_collision=use_collision)
            return

        # Create an array of times that loops from 0 to 1 and back to 0
        fps = 30.0
        n_steps = int(loop_time * fps / 2.0)
        times = np.linspace(0.0, 1.0, n_steps)
        times = np.hstack((times, np.flip(times)))

        # Create bin edges in the range [0, 1] for each trajectory step
        bins = np.arange(traj_len) / (float(traj_len) - 1.0)

        # Compute alphas for each time
        right_inds = np.digitize(times, bins, right=True)
        right_inds[right_inds == 0] = 1
        alphas = ((bins[right_inds] - times) /
                  (bins[right_inds] - bins[right_inds - 1]))

        # Create the new interpolated trajectory
        new_ct = {}
        for k in ct_np:
            new_ct[k] = (alphas * ct_np[k][right_inds - 1] +
                         (1.0 - alphas) * ct_np[k][right_inds])

        # Create the scene
        if use_collision:
            fk = self.collision_trimesh_fk()
        else:
            fk = self.visual_trimesh_fk()

        node_map = {}
        scene = pyrender.Scene()
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            node = scene.add(mesh, pose=pose)
            node_map[tm] = node

        # Get base pose to focus on
        blp = self.link_fk(links=[self.base_link])[self.base_link]

        # Pop the visualizer asynchronously
        v = pyrender.Viewer(scene, run_in_thread=True,
                            use_raymond_lighting=True,
                            view_center=blp[:3,3])

        # Now, run our loop
        i = 0
        while v.is_active:
            cfg = {k: new_ct[k][i] for k in new_ct}
            i = (i + 1) % len(times)

            if use_collision:
                fk = self.collision_trimesh_fk(cfg=cfg)
            else:
                fk = self.visual_trimesh_fk(cfg=cfg)

            v.render_lock.acquire()
            for mesh in fk:
                pose = fk[mesh]
                node_map[mesh].matrix = pose
            v.render_lock.release()

            time.sleep(1.0 / fps)

    def show(self, cfg=None, use_collision=False):
        """Visualize the URDF in a given configuration.

        Parameters
        ----------
        cfg : dict
            A map from joints or joint names to configuration values for
            each joint. If not specified, all joints are assumed to be
            in their default configurations.
        use_collision : bool
            If True, the collision geometry is visualized instead of
            the visual geometry.
        """
        import pyrender  # Save pyrender import for here for CI

        if use_collision:
            fk = self.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.visual_trimesh_fk(cfg=cfg)

        scene = pyrender.Scene()
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            scene.add(mesh, pose=pose)
        pyrender.Viewer(scene, use_raymond_lighting=True)

    def save(self, file_obj):
        """Save this URDF to a file.

        Parameters
        ----------
        file_obj : str or file-like object
            The file to save the URDF to. Should be the path to the
            ``.urdf`` XML file. Any paths in the URDF should be specified
            as relative paths to the ``.urdf`` file instead of as ROS
            resources.

        Returns
        -------
        urdf : :class:`.URDF`
            The parsed URDF.
        """
        if isinstance(file_obj, six.string_types):
            path, _ = os.path.split(file_obj)
        else:
            path, _ = os.path.split(os.path.realpath(file_obj.name))

        node = self._to_xml(None, path)
        tree = ET.ElementTree(node)
        tree.write(file_obj, pretty_print=True,
                   xml_declaration=True, encoding='utf-8')

    def _merge_materials(self):
        """Merge the top-level material set with the link materials.
        """
        for link in self.links:
            for v in link.visuals:
                if v.material is None:
                    continue
                if v.material.name in self.material_map:
                    v.material = self._material_map[v.material.name]
                else:
                    self._materials.append(v.material)
                    self._material_map[v.material.name] = v.material

    @staticmethod
    def load(file_obj):
        """Load a URDF from a file.

        Parameters
        ----------
        file_obj : str or file-like object
            The file to load the URDF from. Should be the path to the
            ``.urdf`` XML file. Any paths in the URDF should be specified
            as relative paths to the ``.urdf`` file instead of as ROS
            resources.

        Returns
        -------
        urdf : :class:`.URDF`
            The parsed URDF.
        """
        if isinstance(file_obj, six.string_types):
            if os.path.isfile(file_obj):
                parser = ET.XMLParser(remove_comments=True,
                                      remove_blank_text=True)
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

    def _validate_joints(self):
        """Raise an exception of any joints are invalidly specified.

        Checks for the following:

        - Joint parents are valid link names.
        - Joint children are valid link names that aren't the same as parent.
        - Joint mimics have valid joint names that aren't the same joint.

        Returns
        -------
        actuated_joints : list of :class:`.Joint`
            The joints in the model that are independently controllable.
        """
        actuated_joints = []
        for joint in self.joints:
            if joint.parent not in self._link_map:
                raise ValueError('Joint {} has invalid parent link name {}'
                                 .format(joint.name, joint.parent))
            if joint.child not in self._link_map:
                raise ValueError('Joint {} has invalid child link name {}'
                                 .format(joint.name, joint.child))
            if joint.child == joint.parent:
                raise ValueError('Joint {} has matching parent and child'
                                 .format(joint.name))
            if joint.mimic is not None:
                if joint.mimic.joint not in self._joint_map:
                    raise ValueError(
                        'Joint {} has an invalid mimic joint name {}'
                        .format(joint.name, joint.mimic.joint)
                    )
                if joint.mimic.joint == joint.name:
                    raise ValueError(
                        'Joint {} set up to mimic itself'
                        .format(joint.mimic.joint)
                    )
            elif joint.joint_type != 'fixed':
                actuated_joints.append(joint)
        return actuated_joints

    def _validate_transmissions(self):
        """Raise an exception of any transmissions are invalidly specified.

        Checks for the following:

        - Transmission joints have valid joint names.
        """
        for t in self.transmissions:
            for joint in t.joints:
                if joint.name not in self._joint_map:
                    raise ValueError('Transmission {} has invalid joint name '
                                     '{}'.format(t.name, joint.name))

    def _validate_graph(self):
        """Raise an exception if the link-joint structure is invalid.

        Checks for the following:

        - The graph is connected in the undirected sense.
        - The graph is acyclic in the directed sense.
        - The graph has only one base link.

        Returns
        -------
        base_link : :class:`.Link`
            The base link of the URDF.
        end_links : list of :class:`.Link`
            The end links of the URDF.
        """

        # Check that the link graph is weakly connected
        if not nx.is_weakly_connected(self._G):
            link_clusters = []
            for cc in nx.weakly_connected_components(self._G):
                cluster = []
                for n in cc:
                    cluster.append(n.name)
                link_clusters.append(cluster)
            message = ('Links are not all connected. '
                       'Connected components are:')
            for lc in link_clusters:
                message += '\n\t'
                for n in lc:
                    message += ' {}'.format(n)
            raise ValueError(message)

        # Check that link graph is acyclic
        if not nx.is_directed_acyclic_graph(self._G):
            raise ValueError('There are cycles in the link graph')

        # Ensure that there is exactly one base link, which has no parent
        base_link = None
        end_links = []
        for n in self._G:
            if len(nx.descendants(self._G, n)) == 0:
                if base_link is None:
                    base_link = n
                else:
                    raise ValueError('Links {} and {} are both base links!'
                                     .format(n.name, base_link.name))
            if len(nx.ancestors(self._G, n)) == 0:
                end_links.append(n)
        return base_link, end_links

    @classmethod
    def _from_xml(cls, node, path):
        valid_tags = set(['joint', 'link', 'transmission', 'material'])
        kwargs = cls._parse(node, path)

        extra_xml_node = ET.Element('extra')
        for child in node:
            if child.tag not in valid_tags:
                extra_xml_node.append(child)

        data = ET.tostring(extra_xml_node)
        kwargs['other_xml'] = data
        return URDF(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        extra_tree = ET.fromstring(self.other_xml)
        for child in extra_tree:
            node.append(child)
        return node
