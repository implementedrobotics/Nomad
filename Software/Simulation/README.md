# GAZEBO

For robot simulation we will be using the Gazebo physics environment.

# Installation

The following instructions are for setup of a development environment for building executable code for Ubuntu 18.04.

Install [Gazebo](http://gazebosim.org/):

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update
```

```
sudo apt-get install gazebo10
sudo apt-get install libgazebo10-dev

gazebo
```

Install Gazebo Web Client:

If you are having dependency issues purge old installs
```
sudo apt remove --purge nodejs npm
sudo apt clean
sudo apt autoclean
sudo apt install -f
sudo apt autoremove
```

Check latest version @:
https://github.com/nodesource/distributions#debinstall

Now install NPM:
```
sudo apt install curl
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs
curl -sL https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install yarn
```

Verify installation:
```
npm -version

nodejs -v
```

Install Gazebo Web:
```
sudo apt install libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential

cd ~; hg clone https://bitbucket.org/osrf/gzweb

cd ~/gzweb
hg up gzweb_1.4.0

source /usr/share/gazebo/setup.sh

#Download Models and Thumbnails:
npm run deploy --- -m -t
```
If using VMWARE you will need this to run the GUI:
```
export SVGA_VGPU10=0

echo "export SVGA_VGPU10=0" >> ~/.profile
```

# PLUGINS

Gazebo uses plugins that control world and model behavior.  We will use these plugins to help simulate our robot model for better algorithm testing off the real hardware.  This speeds up development time and finds bugs before they can cause damnage on expensive hardware.

There are 6 different types of gazebo plugins:
    - World
    - Model
    - Sensor
    - System
    - Visual
    - GUI

Primarily we we will be using the World, Model and Sensor plugins.

## World Plugins:

Let's start by creating a framework for a world plugin.  World plugins are attached to your physics world and helps manage things like the physics parameters(gravity, engine, friction, timesteps, etc) of the simulation.  Also they will be used for adding models to the world, external disturbance forces etc.  

Create the our base nomad_world source template:
```
mkdir ./Gazebo/src/
cd ./Gazebo/src/
vim nomad_world.cpp
```

```
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

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
        }
  };
  GZ_REGISTER_WORLD_PLUGIN(NomadWorldPlugin)
}
```

Create a CMakeLists.txt:

```
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(nomad_world SHARED nomad_world.cc)
target_link_libraries(nomad_world ${GAZEBO_LIBRARIES})

```

Make sure this directory is in the gazebo plugin path:

```
export GAZEBO_PLUGIN_PATH=<INSERT PLUGIN LOCATION DIR>:${GAZEBO_PLUGIN_PATH}
```

If this is persistent make sure to put this in your profile(using bash):

```
echo export GAZEBO_PLUGIN_PATH=<INSERT PLUGIN LOCATION DIR>:${GAZEBO_PLUGIN_PATH} >> ~/.bashrc
source ~/.bashrc
```

Now create a world file to load plugin.  This file can also load your default models and states you want at startup:

```
vim nomad.world
```

```
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <plugin name="nomad_world" filename="libnomad_world.so"/>
  </world>
</sdf>
```

Make sure this directory is in the gazebo plugin path:

```
export GAZEBO_RESOURCE_PATH=<INSERT WORLD LOCATION DIR>:${GAZEBO_RESOURCE_PATH}
```

If this is persistent make sure to put this in your profile(using bash):

```
echo export GAZEBO_RESOURCE_PATH=<INSERT WORLD LOCATION DIR>:${GAZEBO_RESOURCE_PATH} >> ~/.bashrc
source ~/.bashrc
```

Now start the gazebo server:

```
gzserver <INSERT PLUGIN LOCATION DIR>/nomad.world --verbose
```

Or if paths are set correctly:

```
gzserver nomad.world --verbose
```

You should see "Hello Nomad" printed in the terminal.


License
----

MIT


