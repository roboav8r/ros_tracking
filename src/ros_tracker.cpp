#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "tracking_msgs/msg/detections3_d.hpp"
#include "tracking_msgs/msg/detection3_d.hpp"
#include "tracking_msgs/msg/tracks3_d.hpp"
#include "tracking_msgs/msg/track3_d.hpp"
#include "foxglove_msgs/msg/scene_update.hpp"
#include "foxglove_msgs/msg/scene_entity.hpp"

#include "ros_tracking/tracking_datatypes.hpp"
#include "ros_tracking/track_management.hpp"
#include "ros_tracking/hungarian.hpp"
#include "ros_tracking/assignment.hpp"
#include "ros_tracking/sensors.hpp"

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

      // Initialize assignment vector and cost matrix
      this->ha_ = HungarianAlgorithm();
      this->assignment_vector_.reserve(std::max(this->max_dets_,this->max_trks_));
      for (size_t ii=0 ; ii < this->max_trks_; ii++) // iterate through rows
      {
          for (size_t jj=0 ; jj<this->max_dets_; jj++) // iterate through column values
          {
              this->cost_matrix_[ii][jj] = 0.;
          }
      }

      // Initialize ROS members
      sensor_model_ = Sensors::LinearGaussianSensor(gtsam::Vector3(.25,.25,.25));
      subscription_ = this->create_subscription<tracking_msgs::msg::Detections3D>(
      "detections", 10, std::bind(&GraphTracker::detection_callback, this, _1));

      trk_publisher_ = this->create_publisher<tracking_msgs::msg::Tracks3D>("tracks", 10);
      trk_scene_publisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("track_scene", 10);

      trk_idx_ = 0;
      dets_msg_ = tracking_msgs::msg::Detections3D();
      trk_msg_ = tracking_msgs::msg::Track3D();
      trks_msg_ = tracking_msgs::msg::Tracks3D();
      dets_msg_.detections.reserve(this->max_dets_);
      trks_msg_.tracks.reserve(this->max_trks_);
    }

  private:
    void detection_callback(const tracking_msgs::msg::Detections3D::SharedPtr msg)
    {

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
            this->graph_det_ = TrackingDatatypes::GraphDetection(*it, *msg, this->sensor_model_.posVar);
            this->graph_dets_.emplace_back(this->graph_det_);
            
        }
        RCLCPP_INFO(this->get_logger(),"Populated %li detections", this->graph_dets_.size());
      }

      // Propagate existing tracks (if any)
      // TODO - parallelize this step
      for (TrackingDatatypes::GraphTrack trk : this->graph_trks_)
      {
        trk.Predict(msg->header.stamp);
      }
      RCLCPP_INFO(this->get_logger(),"PREDICT %li tracks", this->graph_trks_.size());

      // Compute similarity/cost matrix 
      this->cost_matrix_.clear();
      Assignment::ComputeCostMatrix(this->graph_dets_, this->graph_trks_, this->cost_matrix_); 

      // Solve assignment
      this->cost_ = ha_.Solve(this->cost_matrix_, this->assignment_vector_);
      std::cout << this->assignment_vector_.size() << std::endl;
      //std::cout << this->assignment_vector_ << std::endl;

      // Update tracks with assigned detections

      // Handle unmatched tracks (deletion)
      this->graph_trks_ = ManageTracks::Delete(this->graph_trks_);
      RCLCPP_INFO(this->get_logger(),"DELETE %li tracks", this->graph_trks_.size());

      // Handle unmatched detections (creation)
      ManageTracks::Create(this->graph_dets_, this->graph_trks_, this->sensor_model_, this->trk_idx_);
      RCLCPP_INFO(this->get_logger(),"CREATE %li dets, %li tracks", this->graph_dets_.size(), this->graph_trks_.size());

      // Output 
      this->trk_publisher_->publish(trks_msg_);
           
    }

    // Generic algorithm members
    uint trk_idx_;
    size_t max_dets_{250}; 
    size_t max_trks_{250};
    TrackingDatatypes::GraphDetection graph_det_;
    std::vector<TrackingDatatypes::GraphDetection> graph_dets_;
    std::vector<TrackingDatatypes::GraphTrack> graph_trks_;

    // Assignment
    HungarianAlgorithm ha_;
    std::vector<std::vector<double>> cost_matrix_;
    std::vector<int> assignment_vector_;
    double cost_;

    // Sensor model
    Sensors::LinearGaussianSensor sensor_model_;

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