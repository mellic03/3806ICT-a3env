#pragma once

#include <glm/glm.hpp>
#include <SDL2/SDL.h>

#include <vector>

#include "entities.hpp"




struct View
{
    glm::vec2   position;
    glm::ivec2  resolution;

    glm::ivec2  mouse_screen;
    glm::vec2   mouse_world;
    bool        mouse_down;
    bool        mouse_clicked;

    float       scale;
};


void renderRect   ( SDL_Renderer*, const View&, glm::vec2, glm::vec2, const glm::ivec4& );
void renderRect   ( SDL_Renderer*, const View&, glm::vec2, glm::vec2, const glm::ivec3& );
void renderGrid   ( SDL_Renderer*, const View&, const std::vector<std::vector<uint8_t>>& );
void renderEntity ( SDL_Renderer*, const View&, Entity* );
// void renderAgent  ( SDL_Renderer*, const View&, Agent& );
void renderLine   ( SDL_Renderer*, const View&, const glm::vec2&, const glm::vec2& );



