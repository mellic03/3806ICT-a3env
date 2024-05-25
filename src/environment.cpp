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
    float x = origin.x;
    float y = origin.y;

    float dx = dir.x;
    float dy = dir.y;

    for (int i=0; i<64; i++)
    {
        x += dx;
        y += dy;

        if (m_data[int(y)][int(x)] != BLOCK_NONE)
        {
            break;
        }
    }

    float direction = -1.0f;
    float step = 0.5f;
    BlockType current = m_data[int(y)][int(x)];

    for (int i=0; i<64; i++)
    {
        x += direction*step*dx;
        y += direction*step*dy;

        if (m_data[int(y)][int(x)] != current)
        {
            current = m_data[int(y)][int(x)];
            direction *= -1.0f;
            step *= 0.5f;
        }

        if (step <= 0.01f)
        {
            break;
        }
    }

    dist = glm::distance(origin, glm::vec2(x, y));
    block = m_data[int(y)][int(x)];

    return true;

    return false;

}


void
Environment::render( SDL_Renderer *ren, const View &view )
{
    const glm::vec2 P = view.position;
    const int       W = m_data.size();

    for (int i=0; i<W; i++)
    {
        for (int j=0; j<W; j++)
        {
            renderRect(
                ren, view,
                glm::vec2(32*j, 32*i),
                glm::vec2(32.0f),
                BlockColors[m_data[i][j]]
            );

        }
    }
}


void
Environment::updateAgents( std::vector<Agent> &agents )
{
    for (Agent &agent: agents)
    {
        agent.bearing += agent.angular;

        glm::vec2 dir  = agent.linear * glm::vec2(cos(agent.bearing), sin(agent.bearing));
        glm::vec2 next = agent.position + dir;

        int x  = int(agent.position.x);
        int y  = int(agent.position.y);
        int nx = int(next.x);
        int ny = int(next.y);

        if (nx < 0 || nx >= m_data.size() || ny < 0 || ny >= m_data.size())
        {
            continue;
        }

        agent.position += dir*agent.linear;

        if (m_data[ny][nx] != BLOCK_NONE)
        {
            float dx = next.x - agent.position.x;
            float dy = next.y - agent.position.y;

            if (fabs(dx) > fabs(dy))
            {
                agent.position.x -= dx;
            }

            else
            {
                agent.position.y -= dy;
            }
        }
    }

    for (Agent &agent1: agents)
    {
        for (Agent &agent2: agents)
        {
            if (glm::distance(agent1.position, agent2.position) < 0.25f)
            {
                glm::vec2 dir = agent1.position - agent2.position;
                agent1.position += 0.125f * dir;
                agent2.position -= 0.125f * dir;
            }
        }
    }

}



std::vector<BlockType> &
Environment::operator [] ( int row )
{
    return m_data[row];
}


