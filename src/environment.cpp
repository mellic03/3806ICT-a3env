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



//  march(map) {

//     this.dir.normalise();
//     this.dir.scale(this.fov);

//     for (let x=0; x<scr_wdth; x+=this.res) {
//       let camx = (2*x)/(scr_wdth)-1;

//       this.march_dir.x = this.dir.x + this.plane.x*camx;
//       this.march_dir.y = this.dir.y + this.plane.y*camx;

//       let angle = vector2_angle(this.dir, this.march_dir);

//       let dx = sqrt(1 + (this.march_dir.y**2 / this.march_dir.x**2));
//       let dy = sqrt(1 + (this.march_dir.x**2 / this.march_dir.y**2));

//       let step_x, step_y;

//       let mapX = Math.floor(this.pos.x);
//       let mapY = Math.floor(this.pos.y);

//       let sideDistX, sideDistY;

//       if (this.march_dir.x < 0) {
//         step_x = -1;
//         sideDistX = (this.pos.x - mapX) * dx;
//       }
//       else {
//         step_x = 1;
//         sideDistX = (mapX + 1.0 - this.pos.x) * dx;
//       }

//       if (this.march_dir.y < 0) {
//         step_y = -1;
//         sideDistY = (this.pos.y - mapY) * dy;
//       }
//       else {
//         step_y = 1;
//         sideDistY = (mapY + 1.0 - this.pos.y) * dy;
//       }

//       let hit = 0;
//       let side;

//       let steps = 0;

//       while (hit == 0 && steps < 1000) {
//         steps++;
//         if (sideDistX < sideDistY) {
//           sideDistX += dx;
//           mapX += step_x;
//           side = 0;
//         }
//         else {
//           sideDistY += dy;
//           mapY += step_y;
//           side = 1;
//         }

//         if (point_in_cell(mapX, mapY, map)) {
//           hit = 1;
//         }
//       }

//       if (side == 0) {
//         for (let i=x; i<x+this.res; i++) {
//           if (i >= scr_wdth)
//             break;
//           this.depth_buffer[i].dist = angle * (sideDistX - dx);
//           this.depth_buffer[i].side = side;
//           this.depth_buffer[i].index = 25*Math.floor(mapY/25) + Math.floor(mapX/25);
//         }
//       }
//       else {
//         for (let i=x; i<x+this.res; i++) {
//           if (i >= scr_wdth)
//             break;
//           this.depth_buffer[i].dist = angle * (sideDistY - dy);
//           this.depth_buffer[i].side = side;
//           this.depth_buffer[i].index = 25*Math.floor(mapY/25) + Math.floor(mapX/25);
//         }
//       }
//     }

//     this.dir.normalise();
//   }

// bool
// Environment::raycast( SDL_Renderer *ren, View &view, const glm::vec2 &origin, const glm::vec2 &dir, float &dist, int &block )
// {
//     float x = origin.x;
//     float y = origin.y;

//     float dx = dir.x;
//     float dy = dir.y;

//     // Form ray cast from player into scene
//     glm::vec2 vRayStart = origin;
//     glm::vec2 vRayDir   = dir;

//     // Lodev.org also explains this additional optimistaion (but it's beyond scope of video)
//     glm::vec2 vRayUnitStepSize = { abs(1.0f / vRayDir.x), abs(1.0f / vRayDir.y) };

//     // glm::vec2 vRayUnitStepSize = {
//     //     sqrt(1.0f + (vRayDir.y / vRayDir.x) * (vRayDir.y / vRayDir.x)),
//     //     sqrt(1.0f + (vRayDir.x / vRayDir.y) * (vRayDir.x / vRayDir.y))
//     // };

//     glm::vec2 vMapCheck = vRayStart;
//     glm::vec2 vRayLength1D;
//     glm::vec2 vStep;

//     // Establish Starting Conditions
//     if (vRayDir.x < 0.0f)
//     {
//         vStep.x = -1.0f;
//         vRayLength1D.x = (vRayStart.x - float(vMapCheck.x)) * vRayUnitStepSize.x;
//     }
//     else
//     {
//         vStep.x = 1.0f;
//         vRayLength1D.x = (float(vMapCheck.x + 1.0f) - vRayStart.x) * vRayUnitStepSize.x;
//     }

//     if (vRayDir.y < 0.0f)
//     {
//         vStep.y = -1.0f;
//         vRayLength1D.y = (vRayStart.y - float(vMapCheck.y)) * vRayUnitStepSize.y;
//     }
//     else
//     {
//         vStep.y = 1.0f;
//         vRayLength1D.y = (float(vMapCheck.y + 1.0f) - vRayStart.y) * vRayUnitStepSize.y;
//     }

//     // Perform "Walk" until collision or range check
//     bool  bTileFound = false;
//     float fMaxDistance = 100.0f;
//     float fDistance = 0.0f;

//     while (!bTileFound && fDistance < fMaxDistance)
//     {
//         // Walk along shortest path
//         if (vRayLength1D.x < vRayLength1D.y)
//         {
//             vMapCheck.x += vStep.x;
//             fDistance = vRayLength1D.x;
//             vRayLength1D.x += vRayUnitStepSize.x;
//         }
//         else
//         {
//             vMapCheck.y += vStep.y;
//             fDistance = vRayLength1D.y;
//             vRayLength1D.y += vRayUnitStepSize.y;
//         }

//         x = vMapCheck.x;
//         y = vMapCheck.y;


//         if (x < 0 || x >= m_data.size() || y < 0 || y >= m_data.size())
//         {
//             return false;
//         }


//         if (m_data[int(y)][int(x)] != BLOCK_NONE)
//         {
//             float ax = vRayLength1D.x;
//             float ay = vRayLength1D.y;

//             dist = fDistance; // sqrt(ax*ax + ay*ay);
//             block = m_data[int(y)][int(x)];

//             std::cout << x << ", " << y << "\n";

//             return true;
//         }

//     }

//     return false;
// }


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


