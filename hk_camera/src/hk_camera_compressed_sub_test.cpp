#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp" // Include the string message header
#include <cv_bridge/cv_bridge.hpp>
#include "opencv2/opencv.hpp"

// Callback that handles each received compressed image message
void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try
    {
        // Decode the compressed image message into a cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // The received image can be processed here
        cv::Mat image = cv_ptr->image;

        // Add your image processing code here

        // Current time
        auto current_time = rclcpp::Clock().now();

        // Compute the frame rate
        static int frame_count = 0;
        static auto last_time = current_time;
        auto elapsed_time = current_time - last_time;
        double frame_rate = 1.0 / elapsed_time.seconds();
        last_time = current_time;
        frame_count++;

        // Draw the frame rate in the top-right corner
        std::stringstream ss;
        ss << "Frame Rate: " << frame_rate << " fps";
        cv::putText(image, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // Show the image in an OpenCV window
        cv::imshow("Received Image", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("image_subscriber"), "cv_bridge exception: " << e.what());
        return;
    }
}

// Callback that handles each received string message
void stringCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // Print the contents of the received string message
    std::string message = msg->data;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("string_subscriber"), "Received string message: " << message);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("image_subscriber");

    // Subscribe to the image topic "/hk_camera/rgb/compressed"
    auto image_subscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/hk_camera/rgb/compressed",
        1,
        imageCallback // callback handling each received image message
    );

    // Subscribe to the string topic "/hk_camera/strings"
    auto string_subscriber = node->create_subscription<std_msgs::msg::String>(
        "/hk_camera/strings",
        1,
        stringCallback // callback handling each received string message
    );

    // Initialize the OpenCV window
    cv::namedWindow("Received Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Received Image", 640, 480);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
