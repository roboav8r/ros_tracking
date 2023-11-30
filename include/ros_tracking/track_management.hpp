#ifndef TRACK_MANAGEMENT_H
#define TRACK_MANAGEMENT_H


#include "ros_tracking/tracking_datatypes.hpp"

namespace ManageTracks
{

// Determine whether or not to create track from detection
// TODO - do this probabilistically
bool CreateTrack(const TrackingDatatypes::GraphDetection& Det)
{
    return Det.classConfidence > 0.3; // TODO make this a param
}

// Create tracks based on some criteria
void Create(const std::vector<TrackingDatatypes::GraphDetection>& Dets, std::vector<TrackingDatatypes::GraphTrack>& Trks)
{

    for (TrackingDatatypes::GraphDetection det : Dets)
    {
        if (CreateTrack(det)) {Trks.emplace_back(TrackingDatatypes::GraphTrack(det));}
    }

}


}

#endif