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
void Create(const std::vector<TrackingDatatypes::GraphDetection>& dets, std::vector<TrackingDatatypes::GraphTrack>& tracks, Sensors::LinearGaussianSensor& sensor_model, uint& idx)
{
    for (TrackingDatatypes::GraphDetection det : dets)
    {
        if (CreateTrack(det))
        {
            tracks.emplace_back(TrackingDatatypes::GraphTrack(det, sensor_model, idx));
            idx++;
        }
    }

}

// Determine whether or not to delete track
bool DeleteTrack(const TrackingDatatypes::GraphTrack& trk)
{ 
    return (trk.trackConf < trk.trackConfThresh 
            || trk.missedDets > trk.missedDetThresh 
            || (trk.timeStamp - trk.timeUpdated).seconds() > trk.timeout);
}

// Delete tracks based on some criteria
// TODO - for whatever reason, GraphTrack is not movable, so erase(remove_if ...) doesn't work
std::vector<TrackingDatatypes::GraphTrack> Delete(std::vector<TrackingDatatypes::GraphTrack> tracks)
{
    std::vector<TrackingDatatypes::GraphTrack> new_tracks;
    new_tracks.reserve(tracks.size());

    for (std::vector<TrackingDatatypes::GraphTrack>::iterator it = tracks.begin(); it != tracks.end(); it++)
    {
        if (!DeleteTrack(*it)) { new_tracks.push_back(*it); } 
    }
    return new_tracks;
}

} // namespace

#endif