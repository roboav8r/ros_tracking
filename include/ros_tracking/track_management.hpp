#ifndef TRACK_MANAGEMENT_H
#define TRACK_MANAGEMENT_H


#include "ros_tracking/tracking_datatypes.hpp"

namespace ManageTracks
{

// Determine whether or not to create track from detection
// TODO - do this probabilistically
// TODO - OR above specific confidence threshold
bool CreateTrack(const TrackingDatatypes::GraphDetection& Det)
{
    return true; 
}

// Create tracks based on some criteria
void Create(const std::vector<TrackingDatatypes::GraphDetection>& Dets, std::vector<TrackingDatatypes::GraphTrack>& Trks, Sensors::LinearGaussianSensor& sensor_model, uint& idx)
{
    for (TrackingDatatypes::GraphDetection det : Dets)
    {
        if (CreateTrack(det))
        {
            Trks.emplace_back(TrackingDatatypes::GraphTrack(det, sensor_model, idx));
            idx++;
        }
    }

}


}

#endif