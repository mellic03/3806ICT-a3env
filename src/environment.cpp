#include "environment.hpp"

#include <fstream>
#include <vector>


Environment::Environment( int width )
:   m_data(width, std::vector<uint8_t>(width, BLOCK_NONE))
{

}


void
Environment::loadFile( const std::string &filepath )
{
    std::ifstream stream(filepath);
    std::string line;

    int W = m_data.size();
    int idx = 0;

    while (std::getline(stream, line))
    {
        if (line.find("WALL:") != std::string::npos)
        {
            int row = idx / 25;
            int col = idx % 25;

            if (!(row >= W || col >= W))
            {
                m_data[row][col] = BLOCK_DIRT;
            }
        }

        idx += 1;
    }


    for (int i=0; i<W; i++)
    {
        m_data[i][0]   = BLOCK_DIRT;
        m_data[i][W-1] = BLOCK_DIRT;
        m_data[0][i]   = BLOCK_DIRT;
        m_data[W-1][i] = BLOCK_DIRT;
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
    int   current = m_data[int(y)][int(x)];

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
    m_data[int(y)][int(x)] = BLOCK_STONE;

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
    // Update velocity
    for (int i=0; i<agents.size(); i++)
    {
        Agent &agent = agents[i];

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

        // Agent-agent collisions
        bool good = true;
        for (int j=0; j<agents.size(); j++)
        {
            if (i != j && glm::distance(next, agents[j].position) < 0.25f)
            {
                good = false;
                break;
            }
        }

        if (!good)
        {
            continue;
        }

        if (m_data[ny][nx] == BLOCK_NONE)
        {
            agent.position += dir*agent.linear;
        }

    }

    // Update sonar readings
    for (Agent &agent: agents)
    {
        glm::vec2 dir = glm::vec2(cos(agent.bearing), sin(agent.bearing));
        raycast(agent.position, dir, agent.sonar_dist, agent.sonar_block);
    }

}



std::vector<uint8_t> &
Environment::operator [] ( int row )
{
    return m_data[row];
}


