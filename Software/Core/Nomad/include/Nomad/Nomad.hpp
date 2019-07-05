
#include <OptimalControl/ControlsLibrary.hpp>
#include <Systems/DynamicalSystem.hpp>

class RigidBlock1D : public LinearDynamicalSystem
{
    public:

        // Constructor
        RigidBlock1D(const double& mass, const int& num_states, const double& T_s = 1e-1);

        // Step System
        void Step() {};

        // Update
        void Update() {};

        // Mass
        double GetMass() {return mass;}
    

    protected:
        double mass;
        double width;
        double height;
        double length;

};