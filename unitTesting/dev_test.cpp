#include <sot-dynamic-pinocchio/dynamic.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/parser/urdf.hpp>


#include <iostream>
#include <sstream>

using namespace std;
using namespace dynamicgraph::sot;

int main(int argc, char * argv[])
{
    cout<<"tests sot-dynamic-pinocchio"<<endl;
    if (argc!=2)
    {
        cerr << "Wrong argument number: expect 1 got " << argc-1 <<endl;
        cerr << "Usage:" << endl;
        cerr << "./" << argv[0] << " PATH_TO_URDF_FILE" << endl;
        cerr << "\tPATH_TO_URDF_FILE : Path to the URDF model file describing the robot. "<< endl;
        return 1;
    }
    cout<< "Test parsing " << argv[1] << " ..."<<endl;
    Dynamic * dyn = new Dynamic("tot");

    dyn->setUrdfPath( argv[1]);
    cout<<dyn->m_model;          //display the model
    cout<<dyn->m_data->oMi[0];   //display the first oMi (not relevent but for access test)
    cout<<dyn->m_urdfPath<<endl;
    cout<<"size of oMi : "<<dyn->m_data->oMi.size()<<endl;
    cout<< "Upper Limit size : "<< dyn->m_data->upperPositionLimit.size() << endl;
    cout<< "Upper Limit : "<< dyn->m_data->upperPositionLimit << endl;
    cout<< "Upper Limit size : "<< dyn->m_data->lowerPositionLimit.size() << endl;
    cout<< "Lower Limit : "<< dyn->m_data->lowerPositionLimit << endl;

    ml::Vector q(dyn->m_model.nq);
    ml::Vector v(dyn->m_model.nv);
    ml::Vector a(dyn->m_model.nv);


    delete dyn;
    return 0;
}
