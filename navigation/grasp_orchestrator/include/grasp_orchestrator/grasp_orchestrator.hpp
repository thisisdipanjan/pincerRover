#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>


using NavigateToPose = nav2_msgs::action::NavigateToPose;

class GraspOrchestrator : public rclcpp::Node{
    public:
        GraspOrchestrator();
        void grasp_prepose();
        void send_nav2_goal(const geometry_msgs::msg::PoseStamped &nav2_goal);

    private:
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber;
        void poseCallback(  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::PoseStamped pregrasp_pose;

        struct tag_offset{
            double offset_x{0.5};
            double offset_y{0.0};
        };

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;   

};