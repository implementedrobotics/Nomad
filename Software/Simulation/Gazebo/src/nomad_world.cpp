
/*
 * nomad_world.cpp
 *
 *  Created on: July 3, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


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