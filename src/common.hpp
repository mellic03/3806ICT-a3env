#pragma once



namespace a3env
{
    enum BlockType: uint8_t
    {
        BLOCK_UNKNOWN  = 0,
        BLOCK_AIR      = 1,
        BLOCK_WALL     = 2,
        BLOCK_SURVIVOR = 3,
        BLOCK_HOSTILE  = 4
    };

    static constexpr size_t NUM_AGENTS    = 2;
    static constexpr size_t NUM_HOSTILES  = 2;
    static constexpr size_t NUM_SURVIVORS = 3;
    static constexpr size_t MAP_WIDTH     = 12;

    static constexpr float ENTITY_BODY_W = 0.25f;
    static constexpr float ENTITY_HEAD_W = 0.75f*ENTITY_BODY_W;

}


