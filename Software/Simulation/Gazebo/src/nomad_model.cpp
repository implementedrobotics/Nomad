#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


// Make model plugin that takes a force from external process
namespace gazebo
{
  class NomadModel : public ModelPlugin
  {
    public: 
        // Called when plugin is loaded
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Pointer to model this plugin is attached to
            this->model = _parent;

            // Callback for update event.  Called every physics simulation.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&NomadModel::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
            this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(0, 0, 10.0));
        }

    private:
        // Pointer to model object
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
  };

  // Register Plugin
  GZ_REGISTER_MODEL_PLUGIN(NomadModel)
}