#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/scene_entity.hpp"
#include "foxglove_msgs/msg/cube_primitive.hpp"

using std::placeholders::_1;

// TODO - label map to get name
// - frame conversion - sub to /tf

class DetConverter : public rclcpp::Node
{
  public:
    DetConverter()
    : Node("detection_converter")
    {

      this->subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      "depthai_detections", 10, std::bind(&DetConverter::topic_callback, this, _1));

      this->scene_publisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("foxglove_detections", 10);
      // TODO - converted detections

      this->scene_msg_ = foxglove_msgs::msg::SceneUpdate();
      this->entity_msg_ = foxglove_msgs::msg::SceneEntity();
      this->cube_ = foxglove_msgs::msg::CubePrimitive();
      this->scene_msg_.entities.reserve(this->max_dets_);
      // TODO - add detection messages, reserve space
    }

  private:
    void topic_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
     
      if (msg->detections.size() > 0) // If there are messages to be published
      {
        // TODO - initialize dets_msg_
        this->scene_msg_ = foxglove_msgs::msg::SceneUpdate();

        for (auto it = msg->detections.begin(); it != msg->detections.end(); it++)
        {
              int n = it - msg->detections.begin();
              this->entity_msg_ = foxglove_msgs::msg::SceneEntity();

              // Add header data
              this->entity_msg_.timestamp = msg->header.stamp;
              this->entity_msg_.frame_id = msg->header.frame_id;
              this->entity_msg_.id = std::to_string(n);

              // Add bounding box data
              this->cube_ = foxglove_msgs::msg::CubePrimitive();
              cube_.pose = it->results[0].pose.pose;
              cube_.size = geometry_msgs::msg::Vector3();
              cube_.size.x = 0.5;
              cube_.size.y = 0.5;
              cube_.size.z = 0.5;

              // Add semantic information

              // Add to entity
              this->entity_msg_.cubes.emplace_back(cube_);

              // Add entity to scene update
              this->scene_msg_.entities.emplace_back(this->entity_msg_);

              //this->det_msg_ = tracking_msgs::msg::Detection3D();
              
              // // Convert entity to detection3d
              // this->det_msg_.pose = it->cubes[0].pose;
              // this->det_msg_.class_string = it->metadata[0].value;
              // this->det_msg_.class_confidence = std::stof(it->metadata[1].value);
              // this->det_msg_.attribute = it->metadata[2].value;
              // this->det_msg_.bbox.center = it->cubes[0].pose;
              // this->det_msg_.bbox.size = it->cubes[0].size;

              // // Add detection to Detections3d
              // this->dets_msg_.detections.emplace_back(det_msg_);
        }

        // this->publisher_->publish(dets_msg_);
        this->scene_publisher_->publish(scene_msg_);

      } else { // Don't publish a message
        return;

      }
           
    }
    
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr scene_publisher_;
    foxglove_msgs::msg::SceneUpdate scene_msg_;
    foxglove_msgs::msg::SceneEntity entity_msg_;
    foxglove_msgs::msg::CubePrimitive cube_;
    int max_dets_{250}; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetConverter>());
  rclcpp::shutdown();
  return 0;
}