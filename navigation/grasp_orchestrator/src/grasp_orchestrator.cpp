#include "grasp_orchestrator/grasp_orchestrator.hpp"

GraspOrchestrator::GraspOrchestrator(): Node("grasp_orchestrator"){
    RCLCPP_INFO(get_logger(), "April tag pose detection started");
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate to tag");

    apriltag_subscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "tag_detection_array", 10, std::bind(&GraspOrchestrator::poseCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}



void GraspOrchestrator::poseCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){
    if (msg->detections.empty()){
        RCLCPP_ERROR(get_logger(), "No Pose detected");
    }
    else{
        auto posemsg = msg->detections[0];
        geometry_msgs::msg::TransformStamped tf_;


        tf_ = tf_buffer_->lookupTransform(
            "map",
            "tag_36h11:0",
            tf2::TimePointZero, 
            tf2::durationFromSec(0.5)
        );

        pregrasp_pose.header.frame_id = "map";
        pregrasp_pose.header.stamp = this->get_clock()->now();
        pregrasp_pose.pose.position.x = tf_.transform.translation.x;
        pregrasp_pose.pose.position.y = tf_.transform.translation.y;
        pregrasp_pose.pose.position.z = tf_.transform.translation.z;
        pregrasp_pose.pose.orientation = tf_.transform.rotation;

        RCLCPP_INFO(get_logger(), "Pose generated");
    }    
}



void GraspOrchestrator::grasp_prepose(){
    tag_offset offset_coordinate;

    tf2::Quaternion tag_quat(
        pregrasp_pose.pose.orientation.x,
        pregrasp_pose.pose.orientation.y,
        pregrasp_pose.pose.orientation.z,
        pregrasp_pose.pose.orientation.w
    );

    double tag_roll, tag_pitch, tag_yaw;
    tf2::Matrix3x3(tag_quat).getRPY(tag_roll, tag_pitch, tag_yaw);

    geometry_msgs::msg::PoseStamped nav2_pose;
    nav2_pose.header.frame_id = "map";
    nav2_pose.header.stamp = this->get_clock()->now();

    nav2_pose.pose.position.x = pregrasp_pose.pose.position.x - offset_coordinate.offset_x * cos(tag_yaw);
    nav2_pose.pose.position.y = pregrasp_pose.pose.position.y - offset_coordinate.offset_y * sin(tag_yaw);
    nav2_pose.pose.position.z = 0.0;

    double dx = pregrasp_pose.pose.position.x - nav2_pose.pose.position.x;
    double dy = pregrasp_pose.pose.position.y - nav2_pose.pose.position.y;
    double yaw = atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    nav2_pose.pose.orientation = tf2::toMsg(q);

    send_nav2_goal(nav2_pose);
}


void GraspOrchestrator::send_nav2_goal(const geometry_msgs::msg::PoseStamped &nav2_goal){
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))){
        RCLCPP_ERROR(get_logger(), "nav2 action server busy");
    }
    else{
        NavigateToPose::Goal goal_pose;
        goal_pose.pose = nav2_goal;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result){
            if (result.code==rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(get_logger(), "Navigation to tag_pose success");
            }
            else{
                RCLCPP_INFO(get_logger(), "Navigation failed");
            }
        };
        nav_client_->async_send_goal(goal_pose, send_goal_options);
    }
}