#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp" // Include the string message header
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include "hk_camera.hpp"

// Crop away the field of view that the camera and the lidar do not share; dropping the
// extra pixels keeps rosbag files smaller
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
#define FIT_min_x 420
#define FIT_min_y 70
#define FIT_max_x 2450
#define FIT_max_y 2000
#endif

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //********** rosnode init **********/
    rclcpp::init(argc, argv);
    auto hk_camera = std::make_shared<rclcpp::Node>("hk_camera");
    camera::Camera MVS_cap(*hk_camera);
    //********** rosnode init **********/

    // Set QoS profile with reliability set to "reliable" for both publishers
    rclcpp::QoS qos_profile_realtime(30); // Higher rate, adjust this value based on your requirements
    qos_profile_realtime.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // Reliable delivery
    qos_profile_realtime.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE); // Reduce message backlog

    auto image_pub = hk_camera->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/hk_camera/rgb/compressed", qos_profile_realtime); // Use the custom QoS profile for reliable image publishing

    auto string_pub = hk_camera->create_publisher<std_msgs::msg::String>(
        "/hk_camera/strings", qos_profile_realtime); // Use the custom QoS profile for reliable string publishing

    sensor_msgs::msg::CompressedImage compressed_image_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // this is the RGB format

    //********** 10 Hz        **********/
    rclcpp::Rate loop_rate(30);

    rclcpp::Time last_time = hk_camera->now(); // time the loop iteration started

    int count = 0; // Counter variable

    while (rclcpp::ok())
    {
        loop_rate.sleep();
        rclcpp::spin_some(hk_camera);

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }

#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x, FIT_max_y - FIT_min_y); // crop region: top-left pixel x, y, then width and height
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif

        compressed_image_msg = *(cv_ptr->toCompressedImageMsg());          // toCompressedImageMsg() builds the compressed image message
        compressed_image_msg.header.stamp = hk_camera->get_clock()->now(); // this is the ROS publish time, not the shutter time
        compressed_image_msg.header.frame_id = "hk_camera";
        image_pub->publish(compressed_image_msg); // publish using the compressed image message type

        // New string message with counting variable
        std_msgs::msg::String string_msg;
        string_msg.data = "This is a string message! Count: " + std::to_string(count);
        string_pub->publish(string_msg);

        count++; // Increment the counter variable

        rclcpp::Time current_time = hk_camera->now();              // current time
        rclcpp::Duration loop_duration = current_time - last_time; // how long the iteration took
        // report the iteration time
        RCLCPP_INFO(hk_camera->get_logger(), "Loop duration: %f seconds", loop_duration.seconds());
        last_time = current_time;                                  // start of the next iteration
    }

    rclcpp::shutdown();

    return 0;
}
