#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hk_camera_interfaces/srv/take_photo.hpp"  // path to the custom service definition

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = rclcpp::Node::make_shared("image_client");

  // Create the service client
  auto client = node->create_client<hk_camera_interfaces::srv::TakePhoto>("save_image");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  // Build the request
  auto request = std::make_shared<hk_camera_interfaces::srv::TakePhoto::Request>();
  request->save_path = "/home/zth/Pictures/image.jpg"; // where the image is written

  // Send the request and wait for the response
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "%s", response->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
