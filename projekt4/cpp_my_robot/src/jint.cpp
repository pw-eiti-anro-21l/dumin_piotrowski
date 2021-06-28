#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "cpp_my_robot/srv/jint_control.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
//   if (argc != 8) {
//       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: interpolate movement betwen points: v1[] v2[] time interpolation_type");
//       return 1;
//   }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("jint");
  rclcpp::Client<cpp_my_robot::srv::JintControl>::SharedPtr client =          
    node->create_client<cpp_my_robot::srv::JintControl>("interpolate_arm");              

  auto request = std::make_shared<cpp_my_robot::srv::JintControl::Request>();
    vector<double> start_point;
    vector<double> end_point;
    start_point.push_back(atoll(argv[1]));
    start_point.push_back(atoll(argv[2]));
    start_point.push_back(atoll(argv[3]));
    end_point.push_back(atoll(argv[4]));
    end_point.push_back(atoll(argv[5]));
    end_point.push_back(atoll(argv[6]));

    request->start_joint_position = start_point;
    request->end_joint_position = end_point;
    request->movement_time = atoll(argv[7]);  
    request->interpolation_type = (string) argv[8];     

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: " + result.get()->message);
  } else {
    cout << "err" << endl;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service jint_control_srv");  
  }

  rclcpp::shutdown();
  return 0;
}