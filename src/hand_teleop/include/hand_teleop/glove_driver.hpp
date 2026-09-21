#ifndef HAND_TELEOP_GLOVE_DRIVER_HPP
#define HAND_TELEOP_GLOVE_DRIVER_HPP

#include <vector>


class GloveDriver
{
public:

    GloveDriver();

    bool initialize();

    bool readSensors(
        std::vector<double> &sensor_values
    );

    void shutdown();
};


#endif