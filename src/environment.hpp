#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "render.hpp"


enum BlockType
{
    BLOCK_NONE  = 0,
    BLOCK_GRASS = 1,
    BLOCK_DIRT,
    BLOCK_STONE,
    BLOCK_SILVER, 
    BLOCK_GOLD,
    BLOCK_REE0,
    BLOCK_REE1,
    BLOCK_REE2
};



class Environment
{
private:

public:
    std::vector<std::vector<uint8_t>> m_data;

             Environment( int width );
    void     loadFile( const std::string& );
    std::vector<uint8_t> &operator [] ( int row );

    bool raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist, int &block );
    void render( SDL_Renderer*, const View& );

    void updateAgents( std::vector<Agent> &agents );

};





