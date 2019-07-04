#include <gazebo/gazebo.hh>

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