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
\section sec_intro Introduction

The sot-dynamic package is a bridge between the stack of tasks framework and the dynamicsJRLJapan library.
It provides an inverse dynamic model of the robot through dynamic-graph entities.

This class provides an inverse dynamic model of the robot. More precisely it wraps the newton euler algorithm implemented by the dynamicsJRLJapan library to make it accessible in the stack of tasks
controllers in the Stack of Tasks Framework as defined in \ref Mansard2007.
