#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>

#include "entities.hpp"


enum BlockType
{
    BLOCK_UNKNOWN  = 0,
    BLOCK_AIR      = 1,
    BLOCK_WALL     = 2,
    BLOCK_SURVIVOR = 3,
    BLOCK_HOSTILE  = 4
};



class Environment
{
private:
    std::map<std::pair<int, int>, std::set<Hostile *>> m_hostile_positions;


public:
    std::vector<std::vector<uint8_t>> m_data;

             Environment( int width );
    void     loadFile( const std::string& );
    std::vector<uint8_t> &operator [] ( int row );

    void raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist,
                  uint8_t &block, uint32_t &data );



    void updateEntities( std::vector<Entity *> &entities );
    void updateAgents( std::vector<Agent *> &agents );
    void updateHostiles( std::vector<Hostile *> &hostiles );

};





