#ifndef TRACKING_DATATYPES_H
#define TRACKING_DATATYPES_H

#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose3.h"

#include "tracking_msgs/msg/detection3_d.hpp"

namespace TrackingDatatypes {

// A detection received from ROS message, converted to work in the Graph tracking framework
class GraphDetection
{
    public:
    
    // Default constructor
    GraphDetection(){}

    // Construct from ROS message
    GraphDetection(const tracking_msgs::msg::Detection3D& msg):
        Pose(gtsam::Rot3(gtsam::Quaternion(msg.pose.orientation.w,msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)),
        gtsam::Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)),
        classConfidence(msg.class_confidence)
    {}

    // Members
    gtsam::Pose3 Pose;
    float classConfidence;

    private:

};

// A track designed to work in the Graph tracking framework
class GraphTrack
{
    public:
    // Construct track from detection
    GraphTrack(const GraphDetection& Det) {}

    private:

};

} // Namespace

#endif