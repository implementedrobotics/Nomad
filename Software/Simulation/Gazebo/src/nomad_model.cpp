
/*
 * nomad_model.cpp
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


#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <Communications/Messages/double_vec_t.hpp>


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
            this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(50, 0, 0));
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