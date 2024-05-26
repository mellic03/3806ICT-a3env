#pragma once

#include <glm/glm.hpp>
#include <SDL2/SDL.h>

#include <vector>

#include "agent.hpp"



constexpr glm::ivec3 BlockColors[9] = {
    // glm::ivec3(0, 155, 200),
    glm::ivec3(0),
    glm::ivec3(100, 155, 85),
    // glm::ivec3(177, 127, 88),
    glm::ivec3(0),
    glm::ivec3(170, 178, 181),
    glm::ivec3(170, 178, 181),
    glm::ivec3(220, 165, 18),
    glm::ivec3(220, 165, 18),
    glm::ivec3(220, 165, 18),
    glm::ivec3(220, 165, 18)
};


struct View
{
    glm::vec2  position;
    glm::ivec2 resolution;
    int        scale;
};


void renderRect  ( SDL_Renderer*, const View&, glm::vec2, glm::vec2, const glm::ivec3 &color );
void renderGrid  ( SDL_Renderer*, const View&, const std::vector<std::vector<uint8_t>>& );
void renderAgent ( SDL_Renderer*, const View&, Agent& );
void renderLine  ( SDL_Renderer*, const View&, const glm::vec2&, const glm::vec2& );



