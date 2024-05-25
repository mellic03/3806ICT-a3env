#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "render.hpp"


enum BlockType
{
    BLOCK_NONE,
    BLOCK_GRASS,
    BLOCK_DIRT,
    BLOCK_STONE,
    BLOCK_SILVER, 
    BLOCK_GOLD,
    BLOCK_REE0,
    BLOCK_REE1,
    BLOCK_REE2
};

constexpr glm::ivec3 BlockColors[9] = {
    glm::ivec3(0, 155, 200),
    glm::ivec3(100, 155, 85),
    glm::ivec3(177, 127, 88),
    glm::ivec3(170, 178, 181),
    glm::ivec3(170, 178, 181),
    glm::ivec3(220, 165, 18),
    glm::ivec3(220, 165, 18),
    glm::ivec3(220, 165, 18),
    glm::ivec3(220, 165, 18)
};


class Environment
{
private:
    std::vector<std::vector<BlockType>> m_data;

public:
             Environment( int width );
    void     loadFile( const std::string& );
    std::vector<BlockType> &operator [] ( int row );

    /** Raycast against the grid using the DDA algorithm. */
    bool raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist, int &block );
    void render( SDL_Renderer*, const View& );

    void updateAgents( std::vector<Agent> &agents );

};





