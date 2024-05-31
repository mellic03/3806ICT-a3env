#pragma once

#include <glm/glm.hpp>


enum EntityType
{
    ENTITY_AGENT,
    ENTITY_HOSTILE,
    ENTITY_SURVIVOR
};



struct Entity
{
    int        id;
    EntityType type;
    bool       active = true;

    Entity( int entity_id, EntityType t )
    :   id(entity_id),
        type(t)
    {
        
    };

    virtual ~Entity() = default;


    glm::vec2  position;
    float      bearing;

    float      linear;
    float      angular;
};


struct Agent: public Entity
{
    Agent( int entity_id ): Entity(entity_id, ENTITY_AGENT) {  };

    float      sonar_dist;
    float      sonar_bearing;
    glm::vec2  sonar_hit;
    uint8_t    sonar_block;
    uint32_t   sonar_data;
};


struct Hostile: public Entity
{
    Hostile( int entity_id ): Entity(entity_id, ENTITY_HOSTILE) {  };

};


struct Survivor: public Entity
{
    Survivor( int entity_id ): Entity(entity_id, ENTITY_SURVIVOR) {  };

};


