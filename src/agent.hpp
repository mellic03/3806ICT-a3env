#pragma once

#include <glm/glm.hpp>


struct Agent
{
    glm::vec2 position;
    float     bearing;

    float     linear;
    float     angular;

    float     sonar_dist;
    int       sonar_block;
};




