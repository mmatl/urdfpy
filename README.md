# Urchin

Urchin is a fork of the no-longer-updated `urdfpy`. The API is still mostly the
same. The main difference currently is that you can load urdfs without the
meshes (which is ~100x faster) by passing the flag `lazy_load=True` to the `URDF.load` function.

## Installation
You can install urchin directly from pip.
```bash
pip install urchin
```
## Documentation

See the urdfpy documentation for the current docs. `urchin` will have its own
docs when it diverges enough to deserve it.

The urdfpy docs can be found [here](https://urdfpy.readthedocs.io/en/latest/examples/index.html) for
more information.
