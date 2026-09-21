#include "hand_teleop/glove_driver.hpp"


GloveDriver::GloveDriver()
{
}


bool GloveDriver::initialize()
{
    // Initialise the physical glove here.

    return true;
}


bool GloveDriver::readSensors(
    std::vector<double> &sensor_values)
{
    // Read the physical glove sensors here.

    sensor_values.clear();

    return true;
}


void GloveDriver::shutdown()
{
    // Shut down the physical glove here.
}