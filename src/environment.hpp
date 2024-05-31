#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>


#include "common.hpp"
#include "entities.hpp"



class Environment
{
private:
    std::map<std::pair<int, int>, std::set<Hostile *>>  m_hostile_positions;
    std::map<std::pair<int, int>, std::set<Survivor *>> m_survivor_positions;


public:
    std::vector<std::vector<uint8_t>> m_data;

             Environment( int width );
    void     loadFile( const std::string& );
    std::vector<uint8_t> &operator [] ( int row );

    void raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist,
                  glm::vec2 &hit, uint8_t &block, uint32_t &data );

    void sonar( const glm::vec2 &origin, std::vector<uint8_t>&, std::vector<uint32_t>& );


    void updateEntities( std::vector<Entity *> &entities );
    void updateAgents( std::vector<Agent *> &agents );
    void updateHostiles( std::vector<Hostile *> &hostiles );
    void updateSurvivors( std::vector<Agent*>&, std::vector<Hostile*>&, std::vector<Survivor*>& );

    int randomFreeCell();

};





