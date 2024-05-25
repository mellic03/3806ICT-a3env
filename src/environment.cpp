#include "environment.hpp"

#include <fstream>
#include <vector>


Environment::Environment( int chunk_width ): m_chunk_w(chunk_width)
{

}


void
Environment::loadFile( const std::string &filepath )
{
    std::ifstream stream(filepath);
    std::string line;


    int idx = 0;

    std::vector<int> tokens(4);

    while (std::getline(stream, line))
    {
        tokens[idx] = std::stoi(line);
        idx += 1;


        if (idx == 3)
        {
            BlockType block = BlockType(tokens[0]);
            float     x     = float(tokens[1]) * 2;
            float     y     = float(tokens[2]) * 2;
            float     span  = float(tokens[3]);

            (*this)[int(x)][int(y)] = block;
    
            idx = 0;
        }
    }


    stream.close();
}



bool
Environment::raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist, int &block )
{
    float x = origin.x;
    float y = origin.y;

    float dx = dir.x;
    float dy = dir.y;

    float dydx = dy/dx;
    float dxdy = dx/dy;

    float xstep = 0.0f;
    float ystep = 0.0f;

    float dxDist = 0.0f;
    float dyDist = 0.0f;

    for (int i=0; i<64; i++)
    {
        if (dxDist < dyDist)
        {
            xstep = 1.0f;
            ystep = dydx;
            dxDist += 1.0f;
        }

        else
        {
            xstep = dxdy;
            ystep = 1.0f;
            dyDist += 1.0f;
        }

        x += xstep;
        y += ystep;
    }


    for (int i=0; i<64; i++)
    {
        glm::vec2 pos = origin + (float(i)/64.0f)*dir;

        int x = int(pos.x);
        int y = int(pos.y);
    
        if ((*this)[y][x] != BLOCK_NONE)
        {
            dist  = glm::distance(origin, pos);
            block = int((*this)[y][x]);
            return true;
        }
    }
}


#ifdef ENVIRONMENT_VISUALISATION
void
Environment::render( SDL_Renderer *ren, const View &view )
{
    const glm::vec2 P = view.position;
    const glm::vec2 R = glm::vec2(view.resolution);
    const float     S = view.scale;
    const int       W = m_chunk_w;

    for (auto &[key, chunk]: m_chunks)
    {
        int width  = R.x / S;
        int height = R.y / S;
        int xmin   = W*key.second - int(S*P.x) + width/2;
        int ymin   = W*key.first  - int(S*P.y) + height/2;

        for (int i=0; i<W; i++)
        {
            for (int j=0; j<W; j++)
            {
                SDL_Rect rect = {
                    .x = xmin + j,
                    .y = ymin + i,
                    .w = 1,
                    .h = 1
                };

                glm::ivec3 color = BlockColors[int(chunk[i][j])];
                SDL_SetRenderDrawColor(ren, color.r, color.g, color.b, 255);
                SDL_RenderFillRect(ren, &rect);
            }
        }
    }
}
#endif



BlockType &
Environment::Accessor::operator [] ( int col )
{
    const int W = m_env.m_chunk_w;

    int grid_row = (int)floor(m_row / float(W));
    int grid_col = (int)floor(col / float(W));

    int rel_row = m_row - W*grid_row;
    int rel_col = col - W*grid_col;

    auto key = std::make_pair(grid_row, grid_col);

    if (m_env.m_chunks.find(key) == m_env.m_chunks.end())
    {
        m_env.m_chunks[key] = Chunk(W);
    }

    return m_env.m_chunks[key][rel_row][rel_col];
}


Environment::Accessor
Environment::operator [] ( int row )
{
    return {*this, row};
};

