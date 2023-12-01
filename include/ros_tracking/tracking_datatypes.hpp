#ifndef TRACKING_DATATYPES_H
#define TRACKING_DATATYPES_H

#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/base/Vector.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/linear/KalmanFilter.h"

#include "tracking_msgs/msg/detection3_d.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "ros_tracking/sensors.hpp"

namespace TrackingDatatypes {

// A detection received from ROS message, converted to work in the Graph tracking framework
class GraphDetection
{
    public:
    
    // Default constructor
    GraphDetection(){}

    // Construct from ROS message
    GraphDetection(const tracking_msgs::msg::Detection3D& msg, const tracking_msgs::msg::Detections3D& dets_msg, const gtsam::Vector3& pos_var):
        pose(gtsam::Rot3(gtsam::Quaternion(msg.pose.orientation.w,msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)),
        gtsam::Point3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)),
        var(gtsam::noiseModel::Diagonal::Sigmas(pos_var)),
        classConfidence(msg.class_confidence),
        timeStamp(dets_msg.header.stamp)
    {}

    // Members
    builtin_interfaces::msg::Time timeStamp;
    gtsam::Pose3 pose;
    gtsam::SharedDiagonal var;
    float classConfidence;

    private:

};

// A track designed to work in the Graph tracking framework
class GraphTrack
{
    public:
    // Construct track from detection
    GraphTrack(const GraphDetection& Det, const Sensors::LinearGaussianSensor& sensor_model, const uint& idx):
    pose(Det.pose),
    vel(gtsam::Vector3()),
    trk_id(idx),
    timeStamp(Det.timeStamp)
    {
        this->spatialState = this->kf.init(gtsam::Vector6(this->pose.translation()[0],this->pose.translation()[1],this->pose.translation()[2],vel[0],vel[1],vel[2]),
                                           gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(sensor_model.posVar[0], sensor_model.posVar[1],sensor_model.posVar[2],sensor_model.posVar[0], sensor_model.posVar[1],sensor_model.posVar[2])));
    }

    // Predict
    void Predict(builtin_interfaces::msg::Time t)
    {
        this->dt = (t - this->timeStamp).to_sec();
    }

    // Update

    // Member variables - Admin
    uint trk_id;
    builtin_interfaces::msg::Time timeStamp;
    float dt;

    // Member variables - Spatial
    gtsam::KalmanFilter::State spatialState;
    gtsam::Pose3 pose;
    gtsam::Vector3 vel;

    // Member variables - Semantic TODO

    // GTSAM graph
    gtsam::KalmanFilter kf{6};

    private:

};

} // Namespace

#endif