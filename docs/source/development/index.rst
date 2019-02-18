.. _development:

Development Guide
=================

Read this guide before doing development in ``urdfpy``.

Setting Up
----------

To set up the tools you'll need for developing, you'll need to install
``urdfpy`` in development mode. Start by installing the development
dependencies:

.. code-block:: bash

   git clone https://github.com/mmatl/urdfpy.git
   cd urdfpy
   pip install -e .[dev,docs]

Next, install the git pre-commit hooks with the following command:

.. code-block:: bash

   pre-commit install

This step is crucial. It will install a pre-commit git hook that runs
the ``flake8`` style checker on your code before allowing you to commit.
The style checker isn't just an annoyance -- it does a lot more than just
check for trailing whitespace! It can identify missing or undefined variable
names, unused imports, and a whole host of other potential problems in your
code.

Running Code Style Checks
-------------------------

To run the style checker on all files under version control,
simply run the following command:

.. code-block:: bash

   pre-commit run --all-files

If you'd like to run the style checker more selectively, you can run
the ``flake8`` command directly on a directory (recursively) or on an
individual file:

.. code-block:: bash

   flake8 --ignore=E231,W504 /path/to/files

For more information, please see `the flake8 documentation`_.

.. _the flake8 documentation: https://flake8.pycqa.org/en/latest/user/options.html

Running Tests
-------------

This project uses `pytest`_, the standard Python testing framework.
Their website has tons of useful details, but here are the basics.

.. _pytest: https://docs.pytest.org/en/latest/

To run the testing suite, simply navigate to the top-level folder
in ``urdfpy`` and run the following command:

.. code-block:: bash

   pytest -v tests

You should see the testing suite run. There are a few useful command line
options that are good to know:

- ``-s`` - Shows the output of ``stdout``. By default, this output is masked.
- ``--pdb`` - Instead of crashing, opens a debugger at the first fault.
- ``--lf`` - Instead of running all tests, just run the ones that failed last.
- ``--trace`` - Open a debugger at the start of each test.

You can see all of the other command-line options `here`_.

.. _here: https://docs.pytest.org/en/latest/usage.html

By default, ``pytest`` will look in the ``tests`` folder recursively.
It will run any function that starts with ``test_`` in any file that starts
with ``test_``. You can run ``pytest`` on a directory or on a particular file
by specifying the file path:

.. code-block:: bash

   pytest -v tests/unit/utils/test_transforms.py

You can also test a single function in a file:

.. code-block:: bash

   pytest -v tests/unit/utils/test_images.py::test_rgbd_image

Generating Code Coverage Reports
--------------------------------

To generate code coverage documentation, re-run the tests with the ``--cov``
option set:

.. code-block:: bash

   pytest --cov=urdfpy tests

This will dump a code coverage file. To view it in a nice HTML document,
run the following command:

.. code-block:: bash

   coverage html

The generated HTML homepage will be stored at ``htmlcov/index.html``.

Building Documentation
----------------------

To build ``urdfpy``'s documentation, go to the ``docs`` directory and run
``make`` with the appropriate target.
For example,

.. code-block:: bash

    cd docs/
    make html

will generate HTML-based docs, which are probably the easiest to read.
The resulting index page is at ``docs/build/html/index.html``.
If the docs get stale, just run ``make clean`` to remove all build files.
