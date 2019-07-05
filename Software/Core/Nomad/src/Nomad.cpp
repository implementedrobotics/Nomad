
#include <Nomad/Nomad.hpp>
#include <iostream>


RigidBlock1D::RigidBlock1D(const double& mass, const int& num_states, const double& T_s /* = 1e-1*/)  : LinearDynamicalSystem(num_states, T_s), mass(mass)
{
    std::cout << "Num States: " << num_states<< std::endl;
}




int main()
{
    RigidBlock1D block = RigidBlock1D(1.0, 2);
    std::cout << "Got Block: " << block.GetMass() << std::endl;

    //ControlsLibrary::ContinuousToDiscrete(A, B, 0.1, A_d, B_d);

}

