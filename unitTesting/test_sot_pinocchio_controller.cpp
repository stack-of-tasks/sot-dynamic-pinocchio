/*
 * Copyright 2019,
 * Olivier Stasse,
 *
 * CNRS
 * See LICENSE.txt
 * 
 */

#include <iostream>
#include <sot/core/debug.hh>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot-dynamic-pinocchio/sot-pinocchio-controller.hh>

#include <sstream>
#include <fstream>

int main(int argc, char *argv[])
{
  dgsot::SoTPinocchioController aSotPinocchioController("SimpleHumanoid");
  
  
}


