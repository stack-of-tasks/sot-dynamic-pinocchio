# SoT Dynamic Pinocchio

Encapsulate Pinocchio in SoT

[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/sot-dynamic-pinocchio/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/sot-dynamic-pinocchio/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/sot-dynamic-pinocchio/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/stack-of-tasks/sot-dynamic-pinocchio/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/sot-dynamic-pinocchio/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/sot-dynamic-pinocchio)

This software provides robot dynamic computation for dynamic-graph
by using pinocchio.

Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The matrix abstract layer depends on several packages which
have to be available on your machine.

 - Libraries:
   - [dynamic-graph][dynamic-graph] (>= 3.0.0)
   - [sot-core][sot-core] (>= 3.0.0)
   - [pinocchio][pinocchio] (>= 1.1.2)
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


[dynamic-graph]: http://github.com/stack-of-tasks/dynamic-graph
[pinocchio]: http://github.com/stack-of-tasks/pinocchio
[sot-core]: http://github.com/stack-of-tasks/sot-core
