
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
// C++ Includes
#include <string>
#include <math.h>
// Third Party Includes
#include <zcm/zcm-cpp.hpp>


// Make model plugin that takes a force from external process
namespace gazebo
{
  class NomadModel : public ModelPlugin
  {
    public: 
        // Called when plugin is loaded
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            current_force_ = 0;
            sequence_num_ = 0;
            printf("Hello Nomad Model!\n");
            // Pointer to model this plugin is attached to
            this->model = _parent;

            // Callback for update event.  Called every physics simulation.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&NomadModel::OnUpdate, this));

            // TODO: Pass IP Address as a parameter, in the .model file?
            context_ = std::make_unique<zcm::ZCM>("udpm://239.255.76.67:7667?ttl=0");

            printf("Hello Nomad Model Connecting!\n");
            auto subs = context_->subscribe("nomad.forces", &NomadModel::OnMsg, this);
            context_->start();

            printf("Started Nomad Model!\n");

            // TODO: Publish state back
            pub_context_ = std::make_unique<zcm::ZCM>("udpm://239.255.76.67:7667?ttl=0");

            this->model->GetJoint("j_hfe_FL")->SetPosition(0, -M_PI_2);
            this->model->GetJoint("j_hfe_FR")->SetPosition(0, M_PI_2);
            this->model->GetJoint("j_hfe_RL")->SetPosition(0, -M_PI_2);
            this->model->GetJoint("j_hfe_RR")->SetPosition(0, M_PI_2);

            this->model->GetJoint("j_kfe_FL")->SetLowerLimit(0, -2.2);
            this->model->GetJoint("j_kfe_FL")->SetUpperLimit(0, 0.0);
        
        
    //         robot_->getDof("j_kfe_FL")->setPositionLimits(-2.2, 0.0);
    // robot_->getDof("j_kfe_FR")->setPositionLimits(0.0, 2.2);
    // robot_->getDof("j_kfe_RL")->setPositionLimits(-2.2, 0.0);
    // robot_->getDof("j_kfe_RR")->setPositionLimits(0.0, 2.2);

        }

        // Called by the world update start event
        void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
            //this->model->GetLink("base_link")->SetForce(ignition::math::Vector3d(current_force_, 0, 0));
            //printf("Force: :%f\n", current_force_);

            double_vec_t tx_msg;
            tx_msg.length = 13;
            tx_msg.data.resize(13);

            // TODO: Publish State
            uint64_t time_now = sequence_num_++;

            // Move this back to the PORT portion
            tx_msg.timestamp = time_now;
            tx_msg.sequence_num = sequence_num_++;
            tx_msg.data[0] = this->model->GetLink("base_link")->WorldCoGPose().Pos()[0];
            tx_msg.data[3] = this->model->GetLink("base_link")->WorldCoGLinearVel()[0];

            //printf("PUBLISH: %f/%f\n",tx_msg.data[0],tx_msg.data[1]);

            // Publish
            int rc = pub_context_->publish("nomad.imu", &tx_msg);
        }

        void OnMsg(const zcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const double_vec_t *msg)
        {
            //printf("Received message on channel \"%s\":\n", chan.c_str());
            //printf("Num: %lu\tU: %f", msg->sequence_num, msg->data[0]);

            current_force_ = msg->data[0];
        }

    private:
        // Pointer to model object
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        std::unique_ptr<zcm::ZCM> context_;

        std::unique_ptr<zcm::ZCM> pub_context_;
    
        uint64_t sequence_num_;
        double current_force_;
  };

  // Register Plugin
  GZ_REGISTER_MODEL_PLUGIN(NomadModel)
}