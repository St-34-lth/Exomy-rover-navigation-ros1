#include <exomy_msgs/MotorCommands.h>
#include <exomy_sim_msgs/JointCommand.h>
#include <exomy_sim_msgs/JointCommandArray.h>
#include <exomy_msgs/RoverCommand.h>
#include <list>
#include <memory>
#include <ros/ros.h>
#include <string>


class JointCommandNode
{
  public:
    
    JointCommandNode() 
    {
      nh = ros::NodeHandle("jointcmdnode");

      motor_cmds_sub_ = nh.subscribe(
          "/motor_commands", 100, &JointCommandNode::motorCmdsCallback, this);

      joint_cmd_pub = nh.advertise<exomy_sim_msgs::JointCommandArray>("/joint_cmds",100);

    }
    
    
    ros::NodeHandle nh;
    ros::Publisher joint_cmd_pub;
    ros::Subscriber motor_cmds_sub_;


  private:
    const std::list<std::string> driveJointNames = {
        "DRV_LF_joint", "DRV_RF_joint", "DRV_LM_joint",
        "DRV_RM_joint", "DRV_LR_joint", "DRV_RR_joint"};
    const std::list<std::string> steerJointNames = {
        "STR_LF_joint", "STR_RF_joint", "STR_LM_joint",
        "STR_RM_joint", "STR_LR_joint", "STR_RR_joint"};

    
    void
    motorCmdsCallback(const exomy_msgs::MotorCommands msg) const 
    {
      exomy_sim_msgs::JointCommandArray jointCmdArrayMsg;

      
      
      int i =0;
      for (std::string name : driveJointNames) 
      {
          exomy_sim_msgs::JointCommand jointCmdMsg;

          jointCmdMsg.header.stamp = ros::Time::now();
          jointCmdMsg.name = name;
          jointCmdMsg.mode = "VELOCITY";
          jointCmdMsg.value = float(msg.motor_speeds[i]) / 20.0;
          jointCmdArrayMsg.joint_command_array.push_back(jointCmdMsg);
          i++;
        
      }
      int j = 0;
      for (std::string name : steerJointNames) 
      {
          exomy_sim_msgs::JointCommand jointCmdMsg;
          
          jointCmdMsg.header.stamp = ros::Time::now();
          jointCmdMsg.name = name;
          jointCmdMsg.mode = "POSITION";
          jointCmdMsg.value = -msg.motor_angles[j] / 180.0 * 3.14 ;
          jointCmdArrayMsg.joint_command_array.push_back(jointCmdMsg);
          j++;
      }
      jointCmdArrayMsg.header.stamp = ros::Time::now();
      joint_cmd_pub.publish(jointCmdArrayMsg);

    };
};

int main(int argc, char *argv[]) 
{
  ros::init(argc, argv,"joint_command_node");
  JointCommandNode node;
  ROS_INFO("joint_command_node has started");
  ros::spin();
  ros::shutdown();
  return 0;
}
