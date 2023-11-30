#include <memory>

#include "rclcpp/rclcpp.hpp"

// #include "gtsam/"
// #include "gtsam/nonlinear/NonlinearFactorGraph.h"

#include "tracking_msgs/msg/detections3_d.hpp"
#include "tracking_msgs/msg/detection3_d.hpp"
#include "tracking_msgs/msg/tracks3_d.hpp"
#include "tracking_msgs/msg/track3_d.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/scene_entity.hpp"

#include "ros_tracking/tracking_datatypes.hpp"
#include "ros_tracking/track_management.hpp"

using std::placeholders::_1;

class GraphTracker : public rclcpp::Node
{
  public:
    GraphTracker()
    : Node("graph_tracker")
    {
      // Initialize generic algorithm members
      this->graph_det_ = TrackingDatatypes::GraphDetection();
      this->graph_dets_.reserve(this->max_dets_);
      this->graph_trks_.reserve(this->max_trks_);     

      // Initialize ROS members
      subscription_ = this->create_subscription<tracking_msgs::msg::Detections3D>(
      "detections", 10, std::bind(&GraphTracker::detection_callback, this, _1));

      trk_publisher_ = this->create_publisher<tracking_msgs::msg::Tracks3D>("tracks", 10);
      trk_scene_publisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("track_scene", 10);

      dets_msg_ = tracking_msgs::msg::Detections3D();
      trk_msg_ = tracking_msgs::msg::Track3D();
      trks_msg_ = tracking_msgs::msg::Tracks3D();
      dets_msg_.detections.reserve(this->max_dets_);
      trks_msg_.tracks.reserve(this->max_trks_);

    }

  private:
    void detection_callback(const tracking_msgs::msg::Detections3D::SharedPtr msg)
    {
      //std::cout << "callback" << std::endl;
      // RCLCPP_INFO(this->get_logger(),"Callback");

      // Initialize detection storage
      this->graph_dets_.clear();
      this->graph_dets_.reserve(this->max_dets_);
     
      if (msg->detections.size() > 0) // If there are detections
      {
        this->trks_msg_ = tracking_msgs::msg::Tracks3D();
        this->trks_msg_.header = msg->header;

        // Populate graph_dets_ vector/convert ROS msg to GTSAM datatype
        for (auto it = msg->detections.begin(); it != msg->detections.end(); it++)
        {
            // Add detection message to graph_dets_ vector
            this->graph_det_ = TrackingDatatypes::GraphDetection(*it);
            this->graph_dets_.emplace_back(this->graph_det_);
            
        }
        RCLCPP_INFO(this->get_logger(),"Populated %li detections", this->graph_dets_.size());

        // Propagate existing tracks (if any)

        // Compute similarity with existing tracks (if there are any tracks)

        // Solve assignment

        // Update tracks with assigned detections

        // Handle unmatched tracks (deletion)

        // Handle unmatched detections (creation)

        this->trk_publisher_->publish(trks_msg_);


      } else { // Don't publish a message
        return;
      }
           
    }

    // Generic algorithm members
    int max_dets_{250}; 
    int max_trks_{250};
    TrackingDatatypes::GraphDetection graph_det_;
    std::vector<TrackingDatatypes::GraphDetection> graph_dets_;
    std::vector<TrackingDatatypes::GraphTrack> graph_trks_;

    // ROS members
    rclcpp::Subscription<tracking_msgs::msg::Detections3D>::SharedPtr subscription_;
    rclcpp::Publisher<tracking_msgs::msg::Tracks3D>::SharedPtr trk_publisher_;
    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr trk_scene_publisher_;
    tracking_msgs::msg::Detections3D dets_msg_;
    tracking_msgs::msg::Tracks3D trks_msg_;
    tracking_msgs::msg::Track3D trk_msg_;

    // GTSAM members
    // gtsam::NonlinearFactorGraph::shared_ptr graph_;
    // gtsam::Values::shared_ptr values_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphTracker>());
  rclcpp::shutdown();
  return 0;
}