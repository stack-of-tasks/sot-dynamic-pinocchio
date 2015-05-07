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

    /*  -----  get body -----   */

    string name_body("CHEST");

    if(dyn->m_model.existBodyName(name_body))
    {
        cout << "Test acces : " << dyn->m_model.getBodyId(name_body) << endl;
    }
    else{
        cout << "ERROR : Any body with this name!"<< endl;
    }


    delete dyn;
    return 0;
}
