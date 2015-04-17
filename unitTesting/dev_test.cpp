#include <sot-dynamic/dynamic.h>
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
    }
}
