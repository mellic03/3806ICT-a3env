#include <ros/ros.h>
#include "a3env/sonars.h"
#include "a3env/motors.h"

#include "environment.hpp"
#include "agent.hpp"
#include "mazegen.hpp"



void initWindow( SDL_Window*&, SDL_Renderer*&, View& );
void renderLoop( SDL_Renderer*&, View& );
void keyInput( View &view );

bool sonar_callback( a3env::sonars::Request &req, a3env::sonars::Response &res );
bool motor_callback( a3env::motors::Request &req, a3env::motors::Response &res );


static constexpr size_t NUM_AGENTS = 5;
static constexpr size_t MAP_WIDTH  = 30;

static std::vector<Agent> agents(NUM_AGENTS);
static Environment environment(MAP_WIDTH);

// ros::ServiceClient sonarServiceClient;



int main( int argc, char **argv )
{
    // Load environmentfrom file
    // --------------------------------------------------------------------------
    environment.loadFile("./data/m2.txt");
    // --------------------------------------------------------------------------


    // ROS
    // --------------------------------------------------------------------------
    ros::init(argc, argv, "a3env");
    ros::NodeHandle n;

    for (int i=0; i<NUM_AGENTS; i++)
    {
        int r = i / 3;
        int c = i % 3;

        agents[i].position = glm::vec2(1.2*float(1 + r/2.0f), 1.2*float(1 + c/2.0f));
        agents[i].linear   = 0.0f;
        agents[i].angular  = 0.0f;
        agents[i].bearing  = 0.0f;
    }

    ros::ServiceServer service1 = n.advertiseService("a3env/sonars", sonar_callback);
    ros::ServiceServer service2 = n.advertiseService("a3env/motors", motor_callback);
    // --------------------------------------------------------------------------



    // Rendering
    // --------------------------------------------------------------------------
    SDL_Window   *window;
    SDL_Renderer *ren;

    View view = {
        .position   = glm::vec2(0.0f),
        .resolution = glm::ivec2(1024),
        .scale      = 1
    };

    initWindow(window, ren, view);
    // --------------------------------------------------------------------------


    while (ros::ok())
    {

        renderLoop(ren, view);
        ros::spinOnce();
    }


    return 0;
}



bool sonar_callback( a3env::sonars::Request &req, a3env::sonars::Response &res )
{
    if (req.agentid < 0 || req.agentid >= NUM_AGENTS)
    {
        return false;
    }

    glm::vec2 pos = agents[req.agentid].position;
    float bearing = agents[req.agentid].bearing;
    glm::vec2 dir = glm::normalize(glm::vec2(cos(bearing), sin(bearing)));

    if (environment.raycast(pos, dir, res.distance, res.blocktype))
    {
        return true;
    }

    return false;
}


bool motor_callback( a3env::motors::Request &req, a3env::motors::Response &res )
{
    if (req.agentid < 0 || req.agentid >= NUM_AGENTS)
    {
        return false;
    }

    agents[req.agentid].angular = req.angular;
    agents[req.agentid].linear  = req.linear;

    return true;
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
        environment.updateAgents(agents);
    
        for (Agent &agent: agents)
        {
            renderAgent(ren, view, agent);
        }

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
