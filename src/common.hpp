#pragma once



namespace a3env
{
    enum BlockType
    {
        BLOCK_UNKNOWN  = 0,
        BLOCK_AIR      = 1,
        BLOCK_WALL     = 2,
        BLOCK_SURVIVOR = 3,
        BLOCK_HOSTILE  = 4
    };

    static constexpr size_t NUM_AGENTS    = 6;
    static constexpr size_t NUM_HOSTILES  = 6;
    static constexpr size_t NUM_SURVIVORS = 0;
    static constexpr size_t MAP_WIDTH     = 12;

}


