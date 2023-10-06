// #include "exomy_rover_plugin.hh"

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"

#include <exomy_sim_msgs/JointCommand.h>
#include <exomy_sim_msgs/JointCommandArray.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>

#include <functional>

#include <list>
#include <mutex>
#include <regex>
#include <thread>

namespace gazebo 
{

struct 
PIDParameter 
{
  std::string identifier;
  ignition::math::Vector3d pid_values;
  std::string regex;
};

class 
  ExomyRoverPlugin:  
                    public gazebo::ModelPlugin 
  {
    public:

      ExomyRoverPlugin(){};
      ~ExomyRoverPlugin(){};
      

      std::unique_ptr<ros::NodeHandle> rosNode;
      // Pointer to the model
      physics::ModelPtr model;
      physics::JointControllerPtr joint_controller_;
      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      
      exomy_sim_msgs::JointCommandArray joint_cmd_array_msg;

      ros::Subscriber joint_cmd_array_sub_;
      ros::Subscriber joint_cmd_sub_;
      double update_period_;
      common::Time last_update_time_;

      std::mutex lock_;
      std::list<PIDParameter> position_pid_parameters;
      std::list<PIDParameter> velocity_pid_parameters;
      ros::CallbackQueue rosQueue;
      std::thread rosQueueThread;

      void 
      QueueThread() 
      {
        static const double timeout = 0.01;
        while (this->rosNode->ok()) 
        {
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
        ROS_WARN("queue thread");
      }

      bool 
      LoadPIDParametersFromSDF(const sdf::ElementPtr _sdf) {
        GZ_ASSERT(_sdf, "_sdf element is null");
        
        // Get the position PID parameters
        sdf::ElementPtr position_pid_group = _sdf->GetElement("position_pids");
        sdf::ElementPtr position_pid = position_pid_group->GetFirstElement();

        while (position_pid) {
          PIDParameter pid_parameter;
          pid_parameter.identifier = position_pid->GetName();
          pid_parameter.pid_values =
              position_pid->Get<ignition::math::Vector3d>();
          if (position_pid->HasAttribute("regex")) {
            pid_parameter.regex =
                position_pid->GetAttribute("regex")->GetAsString();
          };

          this->position_pid_parameters.push_back(pid_parameter);

          position_pid = position_pid->GetNextElement();
        }

        // Get the velocity PID parameters
        sdf::ElementPtr velocity_pid_group = _sdf->GetElement("velocity_pids");
        sdf::ElementPtr velocity_pid = velocity_pid_group->GetFirstElement();
        while (velocity_pid) {
          PIDParameter pid_parameter;
          pid_parameter.identifier = velocity_pid->GetName();
          pid_parameter.pid_values =
              velocity_pid->Get<ignition::math::Vector3d>();
          if (velocity_pid->HasAttribute("regex")) {
            pid_parameter.regex =
                velocity_pid->GetAttribute("regex")->GetAsString();
          }

          this->velocity_pid_parameters.push_back(pid_parameter);

          velocity_pid = velocity_pid->GetNextElement();
        };
        ROS_WARN("PID params loaded from SDF ");
        return true;
      };

      void 
      OnUpdate(const gazebo::common::UpdateInfo &_info) {

        
        std::lock_guard<std::mutex> lock(this->lock_);
        double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

        if (seconds_since_last_update < update_period_) 
        {
          return;
        }

        // Update all joint controllers
        this->joint_controller_->Update();
      };

      void 
      OnJointCmdArray(exomy_sim_msgs::JointCommandArray _msg) 
      {
        std::lock_guard<std::mutex> scoped_lock(this->lock_);
       
        // Iterate over received joint command array and set controller targets
        for (auto const cmd : _msg.joint_command_array) 
        {  
          if (cmd.mode == "POSITION") 
          {
            if (!joint_controller_->SetPositionTarget(this->model->GetJoint(cmd.name)->GetScopedName(), cmd.value)) 
            {
              ROS_WARN("Joint from received command was not found in model.");
            }
          } 
          else if (cmd.mode == "VELOCITY") 
          {
            
            // This prohibits the wheels from backspinning
            this->model->GetJoint(cmd.name)->SetParam("fmax", 0, 10.0);
            this->model->GetJoint(cmd.name)->SetParam("vel", 0, double(cmd.value));
          } 
          else 
          {
            ROS_WARN("undefined mode in joint cmd received");
          }
        }
      };
      void 
      Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        
        
        //make sure the ros node is ready 
        if(!ros::isInitialized()) 
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "exomy_rover_plugin",
                    ros::init_options::NoSigintHandler);
        }
        this->rosNode.reset(new ros::NodeHandle("exomy_rover_plugin"));
        
        //get the model
        this->model = _model;

        //get the joint controller
        joint_controller_ = this->model->GetJointController();

        //run the loader for the PID params 
        this->LoadPIDParametersFromSDF(_sdf);

        // Set the PIDs of the joints accordingly to the given parameters
        for (auto const &position_pid_parameter_element : this->position_pid_parameters) 
        {
          auto const pid_identifier = position_pid_parameter_element.identifier;
          auto const pid_values = position_pid_parameter_element.pid_values;

          std::regex pid_regex("(^|_|:)" + pid_identifier + "($|_)");
          
          if (!position_pid_parameter_element.regex.empty()) 
          {
            pid_regex = std::regex(position_pid_parameter_element.regex);
          }

          bool parameter_has_joint = false;


          for (auto const &joint_element : joint_controller_->GetJoints()) 
          {
            auto const joint_name = joint_element.first;
            auto const joint = joint_element.second;

            if (std::regex_search(joint_name, pid_regex)) {
              parameter_has_joint = true;
              this->joint_controller_->SetPositionPID(
                  joint_name,
                  gazebo::common::PID(pid_values.X(), pid_values.Y(),
                                      pid_values.Z()));
              this->joint_controller_->SetPositionTarget(joint_name, 0.0);
             
            }
          }
        }

       

        
        for (auto const &velocity_pid_parameter_element : this->velocity_pid_parameters) 
        {
            auto const pid_identifier =
            velocity_pid_parameter_element.identifier;
            auto const pid_values = velocity_pid_parameter_element.pid_values;

            auto regex = std::regex("(^|_|:)" + pid_identifier + "($|_)");
            
            if (!velocity_pid_parameter_element.regex.empty()) 
            {
              regex = std::regex(velocity_pid_parameter_element.regex);
            }

            bool parameter_has_joint = false;


            for (auto const &joint_element : this->joint_controller_->GetJoints()) 
            {
              auto const joint_name = joint_element.first;
              auto const joint = joint_element.second;
              
           
              if (std::regex_search(joint_name, regex)) 
              {
                parameter_has_joint = true;
                this->joint_controller_->SetVelocityPID(
                    joint_name,
                    gazebo::common::PID(pid_values.X(), pid_values.Y(),
                                        pid_values.Z()));
                this->joint_controller_->SetVelocityTarget(joint_name, 0.0);
                auto const tim = this->joint_controller_->GetLastUpdateTime();

                
              }
            }
            
          

            //update rate
            auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
            if (update_rate > 0.0) 
            {
              this->update_period_ = 1.0 / update_rate;
            } 
            else 
            {
              this->update_period_ = 0.0;
            }
            this->last_update_time_ = this->model->GetWorld()->SimTime();
            
            // Subscribe to joint command array topic
            // need to figure out the correct template of subscribe and pass the correct arguments - passing a member function requires a 'static' function since there is no instance of the class available (?) 
            this->joint_cmd_array_sub_ = this->rosNode->subscribe("/joint_cmds",100,&ExomyRoverPlugin::OnJointCmdArray, this);

            
            // Listen to the update event (broadcast every simulation iteration)
            this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ExomyRoverPlugin::OnUpdate,this,std::placeholders::_1));

        
          };
        };
        void Reset() 
        {
          this->last_update_time_ = this->model->GetWorld()->SimTime();
          this->joint_controller_->Reset();
        };
      };

      GZ_REGISTER_MODEL_PLUGIN(ExomyRoverPlugin);
  };
 



// } // namespace gazebo_plugins