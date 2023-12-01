#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

namespace Sensors
{

class LinearGaussianSensor
{
    public:
    LinearGaussianSensor(){}

    LinearGaussianSensor(const gtsam::Vector3 pos_var):
    posVar(pos_var)
    {}

    gtsam::Vector3 posVar;

};





} // Sensors
#endif