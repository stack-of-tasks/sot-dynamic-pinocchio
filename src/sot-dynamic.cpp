#include <sot/core/debug.hh>
#include <sot-dynamic-pinocchio/dynamic.h>

//#include <boost/version.hpp>
//#include <boost/filesystem.hpp>
//#include <boost/format.hpp>

#include <jrl/mal/matrixabstractlayer.hh>

#include <dynamic-graph/all-commands.h>



using namespace dynamicgraph::sot;
using namespace dynamicgraph;

const std::string dynamicgraph::sot::Dynamic::CLASS_NAME = "DynamicLib";

static matrix4d maalToMatrix4d(const ml::Matrix& inMatrix)
{
  matrix4d homogeneous;
  for (unsigned int r=0; r<4; r++) {
    for (unsigned int c=0; c<4; c++) {
      homogeneous(r,c) = inMatrix(r,c);
    }
  }
  return homogeneous;
}

static vector3d maalToVector3d(const ml::Vector& inVector)
{
  vector3d vector;
  vector(0) = inVector(0);
  vector(1) = inVector(1);
  vector(2) = inVector(2);
  return vector;
}

static matrix3d maalToMatrix3d(const ml::Matrix& inMatrix)
{
  matrix3d matrix;
  for (unsigned int r=0; r<3; r++) {
    for (unsigned int c=0; c<3; c++) {
      matrix(r,c) = inMatrix(r,c);
    }
  }
  return matrix;
}

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
