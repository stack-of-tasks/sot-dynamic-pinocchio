#include <sot-dynamic-pinocchio/dynamic.h>

//#include <boost/version.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/format.hpp>
#include <dynamic-graph/all-commands.h>



using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dynamicgraph::sot::Dynamic::CLASS_NAME = "DynamicLib";

Dynamic::Dynamic( const std::string & name, bool build ):Entity(name)
{

}


Dynamic::~Dynamic( void )
{
    return;
}

void Dynamic::setUrdfPath( const std::string& path )
{

}
