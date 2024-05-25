#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <glm/glm.hpp>
#include <SDL2/SDL.h>


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


struct View
{
    glm::vec2  position;
    glm::ivec2 resolution;
    int        scale;
};


struct Chunk
{
    std::vector<std::vector<BlockType>> data;

    Chunk(): data(128, std::vector<BlockType>(128, BLOCK_NONE)) {  };

    Chunk( int width ): data(width, std::vector<BlockType>(width, BLOCK_NONE))
    {
        
    };

    std::vector<BlockType> &operator [] (int i)
    {
        return data[i];
    };
};


class Environment
{
private:
    struct Accessor;

    using key_type = std::pair<int, int>;
    std::map<key_type, Chunk> m_chunks;

    const int m_chunk_w;


public:
             Environment( int chunk_width );
    void     loadFile( const std::string& );
    Accessor operator [] ( int row );

    /** Raycast against the grid using the DDA algorithm. */
    bool raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist, int &block );
    void render( SDL_Renderer*, const View& );

};


struct Environment::Accessor
{
    Environment &m_env;
    int          m_row;

    BlockType &operator [] ( int col );
};


