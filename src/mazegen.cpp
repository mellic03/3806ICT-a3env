#include "./mazegen.hpp"


void
Maze::print()
{
    for (int i=0; i<m_size.r; i++)
    {
        for (int j=0; j<m_size.c; j++)
        {
            int n = m_data[i][j];

            #ifdef PRINT_UNICODE
                switch (m_data[i][j])
                {
                    case CELL_EMPTY:   std::cout << "  "; break;
                    case CELL_WALL:    std::cout << "⬛"; break;
                    case CELL_START:   std::cout << "🟦"; break;
                    case CELL_GOAL:    std::cout << "❎"; break;
                    case CELL_PATH:    std::cout << "🟦"; break;
                    case CELL_VISITED: std::cout << "🟧"; break;
                    case CELL_TEMP:    std::cout << "🤖"; break;
                }
            #else
                switch (m_data[i][j])
                {
                    case CELL_EMPTY:   std::cout << "  "; break;
                    case CELL_WALL:    std::cout << "# "; break;
                    case CELL_START:   std::cout << "S "; break;
                    case CELL_GOAL:    std::cout << "G "; break;
                    case CELL_PATH:    std::cout << "O "; break;
                    case CELL_VISITED: std::cout << "\' "; break;
                    case CELL_TEMP:    std::cout << "O "; break;
                }
            #endif
        }
        std::cout << "\n";
    }
    std::cout << "\n\n";
}



bool
Maze::in_bounds( const Cell &cell )
{
    bool row = cell.r >= 0 && cell.r < m_size.r;
    bool col = cell.c >= 0 && cell.c < m_size.c;
    return row && col;
}


bool
Maze::are_neighbours( const Cell &a, const Cell &b )
{
    int dr = abs(a.r-b.r);
    int dc = abs(a.c-b.c);
    return dr <= 1 && dc <= 1;
}


int &
Maze::operator [] ( const Cell &cell )
{
    return m_data[cell.r][cell.c];
};


std::vector<int> &
Maze::operator [] ( int i )
{
    return m_data[i];
};



Maze
MazeGenerator::generate( Cell size, Cell start, Cell goal )
{
    Maze maze(size);

    std::stack<Cell>     S;
    std::set<Cell>       V;
    std::queue<Cell>     Q;
    std::map<Cell, Cell> P;

    S.push(start);
    P[start] = start;

    static const Cell offsets[4] = {
        {-1, 0}, {+1, 0}, {0, -1}, {0, +1}
    };


    maze[start] = Maze::CELL_START;
    maze[goal]  = Maze::CELL_GOAL;
    Cell current, parent;

    while (S.empty() == false || Q.empty() == false)
    {
        if (Q.empty() == false)
        {
            current = Q.front();
                      Q.pop();
        }

        else if (S.empty() == false)
        {
            current = S.top();
                      S.pop();
        }

        maze[current] = Maze::CELL_EMPTY;

        // if (delay >= 0)
        // {
        //     maze.print();
        //     std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        // }


        int num_neighbours = 0;
        int idx = rand() % 4;

        for (int i=0; i<4; i++)
        {
            Cell offset = offsets[idx];
            Cell mid    = current + offset;
            Cell cell   = current + 2*offset;

            if (maze.in_bounds(cell) && maze[cell] != Maze::CELL_EMPTY)
            {
                maze[mid] = Maze::CELL_EMPTY;
                maze[cell] = Maze::CELL_EMPTY;

                if (rand() % 100 < 25)
                {
                    Q.push(cell);
                }

                else
                {
                    S.push(cell);
                }

                P[cell] = current;
                num_neighbours += 1;
            }

            else if (maze.in_bounds(mid) && maze.are_neighbours(mid, goal))
            {
                maze[mid] = Maze::CELL_EMPTY;
                P[mid] = current;
            }

            idx = (idx + 1) % 4;
        }


        Cell parent = P[current];
        Cell mid    = current + ((current-parent) / 2);

        if (num_neighbours == 0 && maze.in_bounds(mid))
        {
            dowith_probability(0.1f, [&maze, &mid]()
            {
                maze[mid] = Maze::CELL_EMPTY;
            });
        }
    }


    maze[start] = Maze::CELL_START;
    maze[goal]  = Maze::CELL_GOAL;

    return maze;
};

