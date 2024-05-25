#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <set>
#include <map>
#include <stack>
#include <thread>
#include <functional>
#include <algorithm>
#include <cmath>


// Maze::print will use emoji characters to visualize
// the maze, which is easier to look at.
//
// #define PRINT_UNICODE



struct Cell
{
    int r, c;

    Cell() {  };
    Cell( int _r, int _c): r(_r), c(_c) {  };

    bool operator < ( const Cell &other ) const
    {
        return (r < other.r) || (r == other.r && c < other.c);
    }
};



inline static void dowith_probability( float p, std::function<void()> callback)
{
    if ((rand() % 1000) / 1000.0f < p)
    {
        callback();
    }
}



class Maze
{
public:
    enum CellType: int
    {
        CELL_EMPTY,
        CELL_WALL,
        CELL_START,
        CELL_GOAL,
        CELL_PATH,
        CELL_VISITED,
        CELL_TEMP
    };

    enum Direction: int
    {
        DIR_NORTH,
        DIR_EAST,
        DIR_SOUTH,
        DIR_WEST
    };

private:
    friend class MazeGenerator;
    friend class MazeSolver;

    const Cell                      m_size;
    std::vector<std::vector<int>>   m_data;
    std::map<Cell, Maze::Direction> m_directions;

    bool in_bounds( const Cell& );
    bool are_neighbours( const Cell&, const Cell& );


public:

    Maze( const Cell &size )
    :   m_size(size), m_data(size.r, std::vector<int>(size.c, 1))
    {
        m_directions = {
            {Cell(-1, 0), (Maze::Direction)0}, // North
            {Cell(0, +1), (Maze::Direction)1}, // East
            {Cell(+1, 0), (Maze::Direction)2}, // South
            {Cell(0, -1), (Maze::Direction)3}  // West
        }; 
    };

    void print();

    void is_valid_move( int r, int c, int dr, int dc );

    const Cell &size() const { return m_size; };
    int &operator[] ( const Cell& );
    std::vector<int> &operator[] ( int );
};



class MazeGenerator
{
private:

public:
    Maze generate ( Cell size, Cell start, Cell goal );
};



inline static bool operator == ( const Cell &a, const Cell &b )
{
    return a.r == b.r  &&  a.c == b.c;
}

inline static bool operator != ( const Cell &a, const Cell &b )
{
    return a.r != b.r  ||  a.c != b.c;
}


inline static Cell operator * ( int i, const Cell &v )
{
    return {i*v.r, i*v.c};
}


inline static Cell operator / ( const Cell &v, int i )
{
    return {v.r/i, v.c/i};
}


inline static Cell operator + ( const Cell &v, int i )
{
    return {v.r+i, v.c+i};
}


inline static Cell operator - ( const Cell &v, int i )
{
    return {v.r-i, v.c-i};
}


inline static Cell operator + ( const Cell &a, const Cell &b )
{
    return {a.r+b.r, a.c+b.c};
}

inline static Cell operator - ( const Cell &a, const Cell &b )
{
    return {a.r-b.r, a.c-b.c};
}