#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"

#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/tree.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

using std::placeholders::_1;
using namespace KDL;

class PredictedPositionNode : public rclcpp::Node
{
  public:
    PredictedPositionNode()
    : Node("predictedpositionnode"), count_(0)
      {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&PredictedPositionNode::topic_callback, this, _1));

      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("manipulator_tip_position_KDL", 10);
      urdf_setup("install/cpp_my_robot/share/cpp_my_robot/my_robot.urdf.xml");
     }

  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    size_t count_;

    KDL::Tree my_robot_tree;
    KDL::Chain my_robot_chain;
    urdf::Model my_robot;

    geometry_msgs::msg::PoseStamped Position;

    void urdf_setup(const char *urdf_path);
    void calculate_new_position(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_calculated_position();
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

void PredictedPositionNode::urdf_setup(const char *urdf_path) {
  if (!my_robot.initFile(urdf_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse urdf robot model");
  }
  if (!kdl_parser::treeFromUrdfModel(my_robot, my_robot_tree)){
      RCLCPP_ERROR(get_logger(), "Failed to construct kdl tree");
  }
  my_robot_tree.getChain((std::string) "odom", (std::string) "arm1", my_robot_chain);
}

void PredictedPositionNode::calculate_new_position(const sensor_msgs::msg::JointState::SharedPtr msg) {
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(my_robot_chain);
 
  //Create joint array
  unsigned int nj = my_robot_chain.getNrOfJoints();
  KDL::JntArray jointpositions = JntArray(nj);

  // Assign some values to the joint positions
  for(unsigned int i=0;i<nj;i++){
      jointpositions(i) = (double) msg->position[i];
  }

  // Create the frame that will contain the results
  KDL::Frame tipframe;    

  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(jointpositions,tipframe);
  if(kinematics_status>=0){

      Position.pose.position.set__x(tipframe.p.data[0]);
      Position.pose.position.set__y(tipframe.p.data[1]);
      Position.pose.position.set__z(tipframe.p.data[2]);

      double x,y,z,w;
      tipframe.M.GetQuaternion(x, y, z, w);

      Position.pose.orientation.set__x(x);
      Position.pose.orientation.set__y(y);
      Position.pose.orientation.set__z(z);
      Position.pose.orientation.set__w(w);

      std_msgs::msg::Header tipheader;
      //tipheader.set__stamp(this->get_clock()->now();
      tipheader.set__frame_id("odom");
      Position.set__header(tipheader);
      
  }else{
      RCLCPP_ERROR(get_logger(), "Error: could not calculate forward kinematics");
  }
}

void PredictedPositionNode::publish_calculated_position() {
  publisher_->publish(this->Position);
}

void PredictedPositionNode::topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  calculate_new_position(msg);
  publish_calculated_position();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictedPositionNode>());
  rclcpp::shutdown();
   return 0;
}