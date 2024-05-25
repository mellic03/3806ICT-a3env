#include <ros/ros.h>
#include "a3env/sonars.h"

#include "environment.hpp"
#include "agent.hpp"
#include "mazegen.hpp"



void initWindow( SDL_Window*&, SDL_Renderer*&, View& );
void renderLoop( SDL_Renderer*&, View& );
void keyInput( View &view );

bool sonar_callback( a3env::sonars::Request &req, a3env::sonars::Response &res );


static constexpr size_t NUM_AGENTS = 16;
static std::vector<Agent> agents(NUM_AGENTS);
static Environment environment(128);




int main( int argc, char **argv )
{
    // Generate (or load) environment
    // --------------------------------------------------------------------------
    MazeGenerator MG;
    auto maze = MG.generate({128, 128}, {1, 1}, {128-1, 128-1});

    for (int i=0; i<128; i++)
    {
        for (int j=0; j<128; j++)
        {
            environment[i][j] = (maze[i][j]) ? BLOCK_DIRT : BLOCK_NONE;
        }
    }
    // --------------------------------------------------------------------------


    // ROS
    // --------------------------------------------------------------------------
    ros::init(argc, argv, "a3env");
    ros::NodeHandle n;

    for (int i=0; i<NUM_AGENTS; i++)
    {
        agents[i].position = glm::vec2(0.0f, float(i));
    }


    // ros::ServiceServer service = n.advertiseService("sonars", sonar_callback);
    // --------------------------------------------------------------------------



    // Rendering
    // --------------------------------------------------------------------------
    SDL_Window   *window;
    SDL_Renderer *ren;

    View view = {
        .position   = glm::vec2(0.0f),
        .resolution = glm::ivec2(1024),
        .scale      = 4
    };

    initWindow(window, ren, view);
    // --------------------------------------------------------------------------


    while (ros::ok())
    {
        renderLoop(ren, view);
    }


    return 0;
}



bool sonar_callback( a3env::sonars::Request &req, a3env::sonars::Response &res )
{
    glm::vec2 pos = agents[req.agentid].position;
    float bearing = agents[req.agentid].bearing;
    glm::vec2 dir = glm::vec2(cos(bearing), sin(bearing));

    float dist;
    int   block;

    if (environment.raycast(pos, dir, dist, block))
    {
        res.distance  = dist;
        res.blocktype = block;
        return true;
    }

    return false;
}



void initWindow( SDL_Window *&win, SDL_Renderer *&ren, View &view )
{
    SDL_Init(SDL_INIT_EVERYTHING);

    win = SDL_CreateWindow(
        "A3 Environment",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        view.resolution.x,
        view.resolution.y,
        0
    );

    ren = SDL_CreateRenderer(win, -1, 0);
    // SDL_RenderSetIntegerScale(ren, SDL_TRUE);
    SDL_RenderSetScale(ren, view.scale, view.scale);
}



void renderLoop( SDL_Renderer *&ren, View &view )
{
    static uint32_t a = SDL_GetTicks();
    static uint32_t b = SDL_GetTicks();


    // while (true)
    {
        a = SDL_GetTicks();
        uint32_t delta = a - b;

        if (delta < 1000.0/60.0)
        {
            return;
        }
        b = SDL_GetTicks();


        SDL_Event e;

        while (SDL_PollEvent(&e))
        {
            if ((e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE) ||
                (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) 
            {
                exit(0);
            }
        }
        SDL_PumpEvents();

        keyInput(view);

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        environment.render(ren, view);

        SDL_RenderPresent(ren);
    }
}



void keyInput( View &view )
{
    const glm::vec2 speed = glm::vec2(2.5f) / float(view.scale);

    const uint8_t *state = SDL_GetKeyboardState(NULL);

    if (state[SDL_SCANCODE_A])  view.position.x -= speed.x;
    if (state[SDL_SCANCODE_D])  view.position.x += speed.x;
    if (state[SDL_SCANCODE_W])  view.position.y -= speed.y;
    if (state[SDL_SCANCODE_S])  view.position.y += speed.y;
}
