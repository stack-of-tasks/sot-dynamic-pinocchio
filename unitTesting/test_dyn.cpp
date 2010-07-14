/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_flags.cc
 * Project:   sot
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
  dyn->setVrmlDirectory("/home/nmansard/src/OpenHRP/etc/HRP2JRL/");
  dyn->setXmlSpecificityFile("/home/nmansard/src/PatternGeneratorJRL/src/data/HRP2Specificities.xml");
  dyn->setXmlRankFile("/home/nmansard/src/PatternGeneratorJRL/src/data/HRP2LinkJointRank.xml");
  dyn->setVrmlMainFile("HRP2JRLmain.wrl");

  dyn->parseConfigFiles();
  
  istringstream iss;
  string help("help");
  // Commented out to make this sample a unit test
  // dyn->commandLine(help,iss,cout);

  delete dyn;
  return 0;
}
