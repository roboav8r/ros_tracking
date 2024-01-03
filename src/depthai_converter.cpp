#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tracking_msgs/msg/detections3_d.hpp"
#include "tracking_msgs/msg/detection3_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using std::placeholders::_1;

// TODO - label map to get name
// - frame conversion - sub to /tf

class DetConverter : public rclcpp::Node
{
  public:
    DetConverter()
    : Node("depthai_converter_node")
    {
      this->subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "depthai_detections", 10, std::bind(&DetConverter::topic_callback, this, _1));
      this->publisher_ = this->create_publisher<tracking_msgs::msg::Detections3D>("converted_detections", 10);

      this->declare_parameter("tracker_frame",rclcpp::ParameterType::PARAMETER_STRING);
      this->declare_parameter("labels",rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
      
      this->tracker_frame_ = this->get_parameter("tracker_frame").as_string();
      this->labels_ = this->get_parameter("labels").as_string_array();

      this->tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
      this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      this->det_msg_ = tracking_msgs::msg::Detection3D();
      this->dets_msg_ = tracking_msgs::msg::Detections3D();
      this->dets_msg_.detections.reserve(this->max_dets_);
    }

  private:
    void topic_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
     
      if (msg->detections.size() > 0) // If there are messages to be published
      {
        this->dets_msg_ = tracking_msgs::msg::Detections3D();
        this->dets_msg_.header.stamp = msg->header.stamp;     
        this->dets_msg_.header.frame_id = this->tracker_frame_;

        for (auto it = msg->detections.begin(); it != msg->detections.end(); it++)
        {
              int n = it - msg->detections.begin();
              this->det_msg_ = tracking_msgs::msg::Detection3D();

              // Convert spatial information
              this->obj_pose_det_frame_ = geometry_msgs::msg::PoseStamped();
              this->obj_pose_det_frame_.header = msg->header;
              this->obj_pose_det_frame_.pose = it->results[0].pose.pose;
              this->obj_pose_trk_frame_ = this->tf_buffer_->transform(this->obj_pose_det_frame_,this->tracker_frame_);
              this->det_msg_.pose = obj_pose_trk_frame_.pose;
              this->det_msg_.bbox.center = obj_pose_trk_frame_.pose;
              this->det_msg_.bbox.size = it->bbox.size;

              // Add semantic information
              this->det_msg_.class_string = this->labels_[std::stoi(it->results[0].hypothesis.class_id)];
              this->det_msg_.class_confidence = it->results[0].hypothesis.score;

              this->dets_msg_.detections.emplace_back(this->det_msg_);
        }

        this->publisher_->publish(this->dets_msg_);
        // this->scene_publisher_->publish(scene_msg_);

      } else { // Don't publish a message
        return;

      }
           
    }

    std::string tracker_frame_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::PoseStamped obj_pose_det_frame_;
    geometry_msgs::msg::PoseStamped obj_pose_trk_frame_;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
    rclcpp::Publisher<tracking_msgs::msg::Detections3D>::SharedPtr publisher_;
    tracking_msgs::msg::Detections3D dets_msg_;
    tracking_msgs::msg::Detection3D det_msg_;
    int max_dets_{250}; 

    std::vector<std::string> labels_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetConverter>());
  rclcpp::shutdown();
  return 0;
}