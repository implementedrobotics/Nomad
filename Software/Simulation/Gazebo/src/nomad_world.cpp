#include <gazebo/gazebo.hh>


// TODO: Add a "box model and config"
// Setup paths to model
// Auto Place in this plugin\

namespace gazebo
{
  class NomadWorldPlugin : public WorldPlugin
  {
    public: 
        NomadWorldPlugin() : WorldPlugin()
        {
            printf("Hello Nomad!\n");
        }

    public: 
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
        }
        
  };
  GZ_REGISTER_WORLD_PLUGIN(NomadWorldPlugin)
}