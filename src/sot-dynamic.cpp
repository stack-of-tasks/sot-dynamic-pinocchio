#include <sot-dynamic-pinocchio/dynamic.h>

//#include <boost/version.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/format.hpp>
#include <dynamic-graph/all-commands.h>



using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dynamicgraph::sot::Dynamic::CLASS_NAME = "DynamicLib";

Dynamic::Dynamic( const std::string & name, bool build ):Entity(name),m_data(NULL)
{

}


Dynamic::~Dynamic( void )
{
    if (this->m_data) delete this->m_data;
    return;
}

void Dynamic::setUrdfPath( const std::string& path )
{
    this->m_model = se3::urdf::buildModel(path);
    this->m_urdfPath = path;
    if (this->m_data) delete this->m_data;
    this->m_data = new se3::Data(m_model);


    /*
    self.modelFileName = filename
    self.model = se3.buildModelFromUrdf(filename,True)
    self.data = self.model.createData()
    self.v0 = utils.zero(self.nv)
    self.q0 = utils.zero(self.nq)*/

}
