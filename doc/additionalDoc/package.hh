/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-dynamic.
 * sot-dynamic is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dynamic is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
 */

/** \mainpage
\section sot_dynamic_section_introduction Introduction

The sot-dynamic package is a bridge between the stack of tasks framework and the dynamicsJRLJapan library.
It provides an inverse dynamic model of the robot through dynamic-graph entities.
More precisely it wraps the newton euler algorithm implemented by the dynamicsJRLJapan library
to make it accessible for the stack of tasks controllers
(in the Stack of Tasks Framework as defined in \ref Mansard2007.)

This package depends on the following packages:
\li dynamicsJRLJapan
\li sot-core
\li dynamic-graph
\li dynamic-graph-python

See the JRL umi3218's page on github for instructions on how to download and
install these packages at https://github.com/jrl-umi3218.

\section python_bindings Python bindings

As most packages based on the dynamic-graph framework (see https://github.com/jrl-umi3218/dynamic-graph),
the functionnality is exposed through entities. Hence python sub-modules of dynamic_graph  are generated. See <a href="../sphinx-html/index.html">sphinx documentation</a> for more details.

The following entities are created by this package:\n
(all entites are placed in the namespace sot::)
\li sot::ZmprefFromCom
\li sot::ForceCompensation
\li sot::IntegratorForceExact
\li sot::MassApparent
\li sot::IntegratorForceRk4
\li sot::IntegratorForce
\li sot::AngleEstimator
\li sot::WaistAttitudeFromSensor
\li sot::Dynamic - provides the inverse dynamics computations for of a humanoid robot

See each entity's documentation page for more information (when available).

\section References
\anchor Mansard2007

<b>"Task sequencing for sensor-based control"</b>,
<em>N. Mansard, F. Chaumette,</em>
IEEE Trans. on Robotics, 23(1):60-72, February 2007

**/
