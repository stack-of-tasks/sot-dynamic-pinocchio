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
using namespace dynamicgraph::sot;

int main(int argc, char * argv[])
{

  if (argc!=5)
    {
      cerr << "Usage:" << endl;
      cerr << "./" << argv[0] << " DIR_OF_VRML_MODEL VRML_MODEL_FILENAME PATH_TO_SPECIFICITIES_FILE PATH_TO_LINK2JOINT_FILE " << endl;
      cerr << " DIR_OF_VRML_MODEL : Directory where the robot VRML model lies. "<< endl;
      cerr << " VRML_MODEL_FILENAME : Filename of the VRML model describing the robot."  << endl;
      cerr << " PATH_TO_SPECIFICITIES_FILE : Path to the file describing the humanoid robot specificities." << endl;
      cerr << " PATH_TO_LINK2JOINT_FILE: Path to the file describing the relationship of the joint and the state vector."  << endl;
    }
  Dynamic * dyn = new Dynamic("tot");
  try 
    {
      dyn->setVrmlDirectory(argv[1]);
      dyn->setXmlSpecificityFile(argv[3]);
      dyn->setXmlRankFile(argv[4]);
      dyn->setVrmlMainFile(argv[2]);
      
      dyn->parseConfigFiles();
    } 
  catch (ExceptionDynamic& e)
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
