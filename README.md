sot-dynamic-pinocchio
Encapsulate Pinocchio in SoT
===========

[![Build Status](https://travis-ci.org/proyan/sot-dynamic.png?branch=master)](https://travis-ci.org/proyan/sot-dynamic)
[![Coverage Status](https://coveralls.io/repos/proyan/sot-dynamic/badge.png)](https://coveralls.io/r/proyan/sot-dynamic)

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


[dynamic-graph]: http://github.com/proyan/dynamic-graph
[pinocchio]: http://github.com/stack-of-tasks/pinocchio
[sot-core]: http://github.com/proyan/sot-core
>>>>>>> devel
