#pragma once

#include <glm/glm.hpp>
#include <SDL2/SDL.h>

#include "agent.hpp"


struct View
{
    glm::vec2  position;
    glm::ivec2 resolution;
    int        scale;
};


void renderRect( SDL_Renderer*, const View&, glm::vec2, glm::vec2, const glm::ivec3 &color );



void renderAgent( SDL_Renderer*, const View&, Agent& );



