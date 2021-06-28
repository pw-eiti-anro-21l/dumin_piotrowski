#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cpp_my_robot/srv/jint_control.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace std::chrono_literals;

class InterpolationNode : public rclcpp::Node
{
  public:
    InterpolationNode()
    : Node("interpolation_node"), count_(0)
      {
      publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    };

    void interpolation_linear(vector<double> start_point, vector<double> end_point, double travel_time);

  private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    size_t count_;

    vector<vector<double>> interpolate_vector_linear(vector<double> start_point, vector<double> end_point, double travel_time);
    void publish_positions(vector<vector<double>> consecutive_positions);
};

vector<vector<double>> InterpolationNode::interpolate_vector_linear(vector<double> start_point, vector<double> end_point, double travel_time) {
  int number_of_frames = floor(travel_time / 0.01);
  cout << travel_time << endl;
  cout << number_of_frames << endl;
  vector<vector<double>> interpolated_frames;
  interpolated_frames.push_back(start_point);
  vector<double> frame_distance;
  for (int i = 0; i < 3; i++) {
      frame_distance.push_back(end_point[i]-start_point[i]);
  }
  for (int frame_i = 0; frame_i < number_of_frames; frame_i++) {
      vector<double> current_frame;
      for (int dimention_n = 0; dimention_n < 3; dimention_n++){
          current_frame.push_back(interpolated_frames[frame_i][dimention_n]+frame_distance[dimention_n]);
      }
      interpolated_frames.push_back(current_frame);
      cout << current_frame[0] << endl;
  }
  interpolated_frames.push_back(end_point);
  return interpolated_frames;
}

void InterpolationNode::interpolation_linear(vector<double> start_point, vector<double> end_point, double travel_time) {
  publish_positions(interpolate_vector_linear(start_point, end_point, travel_time));
}

void InterpolationNode::publish_positions(vector<vector<double>> consecutive_positions) {
  rclcpp::Rate r(10);
  for (uint i = 0; i < consecutive_positions.size(); i++) {
    sensor_msgs::msg::JointState msg;
    msg.name = {"base_to_arm3", "arm3_to_arm2", "arm2_to_arm1"};
    msg.position = consecutive_positions[i];
    publisher_->publish(msg);
    r.sleep();
    cout << i << endl;
  }
}

void interpolate_arm(const std::shared_ptr<cpp_my_robot::srv::JintControl::Request> request, 
    std::shared_ptr<cpp_my_robot::srv::JintControl::Response> response)
{
  try {

    double travel_time = request.get()->movement_time;
     cout << travel_time << endl;
    vector<double> start_point = request.get()->start_joint_position;
     cout << start_point[0] << endl;
     cout << start_point[1] << endl;
     cout << start_point[2] << endl;
    vector<double> end_point = request.get()->end_joint_position;
    cout << end_point[0] << endl;
    cout << end_point[1] << endl;
    cout << end_point[1] << endl;
    cout << travel_time << endl;

    InterpolationNode int_node;
    int_node.interpolation_linear(start_point, end_point, travel_time);
    response->valid_data = true;
    response->message = (string) "Interpolation complete";
  } catch (string err_message) {
    response->valid_data = false;
    response->message = err_message;
  }                          
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("jint_control_srv");  

  rclcpp::Service<cpp_my_robot::srv::JintControl>::SharedPtr service =                 
  node->create_service<cpp_my_robot::srv::JintControl>("interpolate_arm",  &interpolate_arm);     

  rclcpp::spin(node);
  rclcpp::shutdown();
}