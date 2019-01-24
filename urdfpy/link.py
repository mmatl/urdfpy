

class Link(object):

    def __init__(self,
                 name,
                 center_mass=None,
                 mass=None,
                 inertia=None,
                 visual_geometry=None
                 collision_geometry=None):
        self.name = name
        self.center_mass = center_mass
        self.mass = mass
        self.inertia = inertia
        self.visual_geometry = visual_geometry
        self.collision_geometry = collision_geometry


