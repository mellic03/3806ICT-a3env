#include "environment.hpp"

#include <fstream>
#include <vector>


Environment::Environment( int width )
:   m_data(width, std::vector<BlockType>(width, BLOCK_NONE))
{

}


void
Environment::loadFile( const std::string &filepath )
{
    std::ifstream stream(filepath);
    std::string line;


    std::vector<int> tokens(4);

    std::cout << "Loading file\n";


    int idx = 0;


    while (std::getline(stream, line))
    {
        // std::cout << line << "\n";
        if (line.find("WALL:") != std::string::npos)
        {
            int row = idx / 25;
            int col = idx % 25;

            m_data[row][col] = BLOCK_DIRT;
        }

        idx += 1;
    }
    std::cout << "Loaded file\n";


    stream.close();
}



bool
Environment::raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist, int &block )
{
    float x = 16.0f * origin.x;
    float y = 16.0f * origin.y;

    // float dx = dir.x;
    // float dy = dir.y;

    // float dydx = dy/dx;
    // float dxdy = dx/dy;

    // float xstep = 0.0f;
    // float ystep = 0.0f;

    // float dxDist = 0.0f;
    // float dyDist = 0.0f;

    // for (int i=0; i<64; i++)
    // {
    //     if (dxDist < dyDist)
    //     {
    //         xstep = 1.0f;
    //         ystep = dydx;
    //         dxDist += 1.0f;
    //     }

    //     else
    //     {
    //         xstep = dxdy;
    //         ystep = 1.0f;
    //         dyDist += 1.0f;
    //     }

    //     x += xstep;
    //     y += ystep;
    // }

    for (int i=0; i<64; i++)
    {
        glm::vec2 pos = origin + (float(i)/64.0f)*dir;

        int x = int(pos.x);
        int y = int(pos.y);
    
        if (x < 0 || x >= m_data.size() || y < 0 || y >= m_data.size())
        {
            return false;
        }

        if (m_data[y][x] != BLOCK_NONE)
        {
            dist  = glm::distance(origin, pos);
            block = int(m_data[y][x]);
            return true;
        }
    }
}


void
Environment::render( SDL_Renderer *ren, const View &view )
{
    const glm::vec2 P = view.position;
    const int       W = m_data.size();

    int xmin = 0 - int(P.x) + (view.resolution.x/2);
    int ymin = 0 - int(P.y) + (view.resolution.y/2);

    for (int i=0; i<W; i++)
    {
        for (int j=0; j<W; j++)
        {
            SDL_Rect rect = {
                .x = xmin + 32*j,
                .y = ymin + 32*i,
                .w = 32,
                .h = 32
            };

            glm::ivec3 color = BlockColors[m_data[i][j]];
            SDL_SetRenderDrawColor(ren, color.r, color.g, color.b, 255);
            SDL_RenderFillRect(ren, &rect);
        }
    }
}


std::vector<BlockType> &
Environment::operator [] ( int row )
{
    return m_data[row];
}


