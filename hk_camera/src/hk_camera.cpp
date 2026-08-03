#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
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
    // Automatically declare parameters coming from launch/yaml, otherwise get_parameter_or
    // inside Camera only ever sees the defaults
    auto hk_camera = std::make_shared<rclcpp::Node>(
        "hk_camera",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    camera::Camera MVS_cap(*hk_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hk_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hk_camera/rgb", 1);
    
    sensor_msgs::msg::Image image_msg;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = std::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // this is the RGB format

    //********** 10 Hz        **********/
    rclcpp::Rate loop_rate(30);

    rclcpp::Time last_time = hk_camera->now(); // time the loop iteration started

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

        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = hk_camera->get_clock()->now(); // this is the ROS publish time, not the shutter time
        image_msg.header.frame_id = "hk_camera";
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        image_pub.publish(image_msg, camera_info_msg);    

        rclcpp::Time current_time = hk_camera->now();              // current time
        rclcpp::Duration loop_duration = current_time - last_time; // how long the iteration took
        // report the iteration time
        RCLCPP_INFO(hk_camera->get_logger(), "Loop duration: %f seconds", loop_duration.seconds());
        last_time = current_time;                                  // start of the next iteration
    }
    rclcpp::shutdown();

    return 0;
}
