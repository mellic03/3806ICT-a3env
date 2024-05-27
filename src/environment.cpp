#include "environment.hpp"

#include <fstream>
#include <vector>


Environment::Environment( int width )
:   m_data(width, std::vector<uint8_t>(width, BLOCK_AIR))
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
                m_data[row][col] = BLOCK_WALL;
            }
        }

        idx += 1;
    }


    for (int i=0; i<W; i++)
    {
        m_data[i][0]   = BLOCK_WALL;
        m_data[i][W-1] = BLOCK_WALL;
        m_data[0][i]   = BLOCK_WALL;
        m_data[W-1][i] = BLOCK_WALL;
    }

    stream.close();
}

void
Environment::raycast( const glm::vec2 &origin, const glm::vec2 &dir, float &dist,
                      uint8_t &block, uint32_t &data )
{
    glm::vec2 vRayUnitStepSize = {
        sqrt(1 + (dir.y / dir.x) * (dir.y / dir.x)),
        sqrt(1 + (dir.x / dir.y) * (dir.x / dir.y))
    };

    int row = int(origin.y);
    int col = int(origin.x);

    float dx = dir.x;
    float dy = dir.y;

    float dxdist = 0.0f;
    float dydist = 0.0f;

    glm::vec2 vRayLength1D;
    glm::ivec2 vStep;

    if (dx < 0)
    {
        vStep.x = -1;
        dxdist = (origin.x - col) * vRayUnitStepSize.x;
    }

    else
    {
        vStep.x = 1;
        dxdist = (col+1 - origin.x) * vRayUnitStepSize.x;
    }

    if (dy < 0)
    {
        vStep.y = -1;
        dydist = (origin.y - row) * vRayUnitStepSize.y;
    }

    else
    {
        vStep.y = 1;
        dydist = (row+1 - origin.y) * vRayUnitStepSize.y;
    }


    dist = 0.0f;

    while (dist < 2*m_data.size())
    {
        if (dxdist < dydist)
        {
            col += vStep.x;
            dist = dxdist;
            dxdist += vRayUnitStepSize.x;
        }
        else
        {
            row += vStep.y;
            dist = dydist;
            dydist += vRayUnitStepSize.y;
        }

        if (row < 0 || row >= m_data.size() || col < 0 || col>= m_data.size())
        {
            break;
        }


        // Hostile detection
        // ---------------------------------------------------
        auto key = std::make_pair(row, col);

        if (m_hostile_positions[key].empty() == false)
        {
            uint32_t bitmask = 0;

            for (Hostile *h: m_hostile_positions[key])
            {
                bitmask |= (1 << h->id);
            }

            block = BLOCK_HOSTILE;
            data  = bitmask;
            break;
        }
        // ---------------------------------------------------


        if (m_data[row][col] != BLOCK_AIR)
        {
            break;
        }
    }

}


void
Environment::updateEntities( std::vector<Entity *> &entities )
{
    // Update velocity
    for (int i=0; i<entities.size(); i++)
    {
        Entity *e = entities[i];

        e->bearing += e->angular;

        glm::vec2 dir  = e->linear * glm::vec2(cos(e->bearing), sin(e->bearing));
        glm::vec2 next = e->position + dir;

        int x  = int(e->position.x);
        int y  = int(e->position.y);
        int nx = int(next.x);
        int ny = int(next.y);

        if (nx < 0 || nx >= m_data.size() || ny < 0 || ny >= m_data.size())
        {
            continue;
        }

        // Entity-entity collisions
        bool good = true;
        for (int j=0; j<entities.size(); j++)
        {
            if (i != j && glm::distance(next, entities[j]->position) < 0.25f)
            {
                good = false;
                break;
            }
        }

        if (!good)
        {
            continue;
        }

        if (m_data[ny][nx] == BLOCK_AIR)
        {
            e->position += dir*e->linear;
        }
    }
}


void
Environment::updateAgents( std::vector<Agent *> &agents )
{
    // Update sonar readings
    for (Agent *agent: agents)
    {
        glm::vec2 dir = glm::vec2(cos(agent->bearing), sin(agent->bearing));

        raycast(
            agent->position, dir,
            agent->sonar_dist,
            agent->sonar_block,
            agent->sonar_data
        );
    }
}


void
Environment::updateHostiles( std::vector<Hostile *> &hostiles )
{
    // Create map of (row, col) to list of hostiles at (row, col)

    m_hostile_positions.clear();

    for (Hostile *h: hostiles)
    {
        h->bearing += 0.25f * ((rand() % 100) / 100.0f - 0.5f);
        // h->linear = 0.11f;


        glm::vec2 pos = h->position;

        int row  = int(pos.y);
        int col  = int(pos.x);
        auto key = std::make_pair(row, col);

        m_hostile_positions[key].insert(h);
    }
}




std::vector<uint8_t> &
Environment::operator [] ( int row )
{
    return m_data[row];
}


