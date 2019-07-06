
#include <OptimalControl/ControlsLibrary.hpp>
#include <Systems/DynamicalSystem.hpp>
#include <Eigen/Dense>

class RigidBlock1D : public LinearDynamicalSystem
{
    public:

        // Constructor
        RigidBlock1D(const double& mass, const Eigen::Vector3d& box_shape, const double& T_s = 1e-1);
 
        // Step System
        void Step(const Eigen::VectorXd& u);

        // Update
        void Update();

        // Mass
        double GetMass() {return mass;}
    

    protected:
        double mass;
        double width;
        double height;
        double length;

};