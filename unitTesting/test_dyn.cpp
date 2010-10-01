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

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <sot-dynamic/dynamic.h>
#include <iostream>
#include <sstream>

using namespace std;
using namespace sot;

int main( void )
{


  Dynamic * dyn = new Dynamic("tot");
  try {
  dyn->setVrmlDirectory("~/src/OpenHRP/etc/HRP2JRL/");
  dyn->setXmlSpecificityFile("~/src/PatternGeneratorJRL/src/data/HRP2Specificities.xml");
  dyn->setXmlRankFile("~/src/PatternGeneratorJRL/src/data/HRP2LinkJointRank.xml");
  dyn->setVrmlMainFile("HRP2JRLmain.wrl");

  dyn->parseConfigFiles();
  } catch (sot::ExceptionDynamic& e)
  {
	  if ( !strcmp(e.what(), "Error while parsing." )) {
		  cout << "Could not locate the necessary files for this test" << endl;
		  return 77;
	  }
	  else
		  // rethrow
		  throw e;
  }
  
  istringstream iss;
  string help("help");
  // Commented out to make this sample a unit test
  // dyn->commandLine(help,iss,cout);

  delete dyn;
  return 0;
}
