#include <ros/ros.h>
#include <ros/package.h>

#include "a3env/sonars.h"
#include "a3env/motors.h"
#include "a3env/odom.h"

#include "environment.hpp"
#include "entities.hpp"
#include "render.hpp"
#include "mazegen.hpp"


void initWindow( SDL_Window*&, SDL_Renderer*&, View& );
void renderLoop( SDL_Renderer*&, View& );
void keyInput( View &view );

void updateEnvironment();
bool motors_callback( a3env::motors::Request &req, a3env::motors::Response &res );


static constexpr size_t NUM_AGENTS   = 6;
static constexpr size_t NUM_HOSTILES = 6;
static constexpr size_t NUM_ENTITIES = NUM_AGENTS + NUM_HOSTILES;
static constexpr size_t MAP_WIDTH    = 12;


std::vector<Entity *>   entities;
std::vector<Agent *>    agents;
std::vector<Hostile *>  hostiles;


static Environment environment(MAP_WIDTH);

static std::vector<ros::Publisher> sonars_pub(NUM_AGENTS);
static std::vector<ros::Publisher> odom_pub(NUM_AGENTS);



int main( int argc, char **argv )
{
    // Load environment from file
    // --------------------------------------------------------------------------
    std::string path = ros::package::getPath("a3env");
    environment.loadFile(path + "/data/m2.txt");
    // --------------------------------------------------------------------------


    // Initialize entities
    // --------------------------------------------------------------------------
    for (int i=0; i<NUM_AGENTS; i++)
    {
        entities.push_back(new Agent(i));
        agents.push_back(dynamic_cast<Agent *>(entities.back()));
    }

    for (int i=0; i<NUM_HOSTILES; i++)
    {
        entities.push_back(new Hostile(i));
        hostiles.push_back(dynamic_cast<Hostile *>(entities.back()));
    }

    for (int i=0; i<NUM_AGENTS; i++)
    {
        int r = i / 3;
        int c = i % 3;

        agents[i]->position = glm::vec2(1.2*float(1 + r/2.0f), 1.2*float(1 + c/2.0f));
        agents[i]->linear   = 0.0f;
        agents[i]->angular  = 0.0f;
        agents[i]->bearing  = 0.0f;
    }

    for (int i=0; i<NUM_HOSTILES; i++)
    {
        int r = i / 3;
        int c = i % 3;

        hostiles[i]->position = glm::vec2(1.2*float(7 + r/2.0f), 1.2*float(7 + c/2.0f));
        hostiles[i]->linear   = 0.0f;
        hostiles[i]->angular  = 0.0f;
        hostiles[i]->bearing  = 0.0f;
    }
    // --------------------------------------------------------------------------


    // ROS
    // --------------------------------------------------------------------------
    ros::init(argc, argv, "a3env");
    ros::NodeHandle n;

    for (int i=0; i<NUM_AGENTS; i++)
    {
        std::string label1 = "a3env/sonars" + std::to_string(i);
        std::string label2 = "a3env/odom" + std::to_string(i);

        sonars_pub[i] = n.advertise<a3env::sonars>(label1, 16);
        odom_pub[i]   = n.advertise<a3env::odom>(label2, 16);
    }
    ros::ServiceServer service = n.advertiseService("a3env/motors", motors_callback);
    // --------------------------------------------------------------------------



    // Rendering
    // --------------------------------------------------------------------------
    SDL_Window   *window;
    SDL_Renderer *ren;

    View view = {
        .position   = glm::vec2(0.0f),
        .resolution = glm::ivec2(1024),
        .scale      = 64
    };

    initWindow(window, ren, view);
    // --------------------------------------------------------------------------

    ros::Rate rate(1);

    while (ros::ok())
    {
        renderLoop(ren, view);
        updateEnvironment();
        ros::spinOnce();
    }

    return 0;
}



void updateEnvironment()
{
    {
        static uint32_t a = SDL_GetTicks();
        static uint32_t b = SDL_GetTicks();

        a = SDL_GetTicks();
        uint32_t delta = a - b;

        if (delta < 1000.0/60.0)
        {
            return;
        }
        b = SDL_GetTicks();
    }

    environment.updateEntities(entities);
    environment.updateAgents(agents);
    environment.updateHostiles(hostiles);

    for (int i=0; i<NUM_AGENTS; i++)
    {
        a3env::sonars S;
        S.distance  = agents[i]->sonar_dist;
        S.blocktype = agents[i]->sonar_block;
        S.data      = agents[i]->sonar_data;

        a3env::odom O;
        O.xpos    = agents[i]->position.x;
        O.ypos    = agents[i]->position.y;
        O.bearing = agents[i]->bearing;

        sonars_pub[i].publish(S);
        odom_pub[i].publish(O);
    }
}



bool motors_callback( a3env::motors::Request &req, a3env::motors::Response &res )
{
    if (req.agentid < 0 || req.agentid >= NUM_AGENTS)
    {
        return false;
    }

    agents[req.agentid]->angular = req.angular;
    agents[req.agentid]->linear  = req.linear;

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
    SDL_RenderSetScale(ren, 1, 1);
}


void renderLoop( SDL_Renderer *&ren, View &view )
{
    {
        static uint32_t a = SDL_GetTicks();
        static uint32_t b = SDL_GetTicks();

        a = SDL_GetTicks();
        uint32_t delta = a - b;

        if (delta < 1000.0/60.0)
        {
            return;
        }
        b = SDL_GetTicks();
    }


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

    renderGrid(ren, view, environment.m_data);


    for (Entity *e: entities)
    {
        renderEntity(ren, view, e); 
    }
    // for (Agent *agent: agents)
    // {
    //     renderEntity(ren, view, dynamic_cast<Entity *>(agent)); 
    // }

    SDL_RenderPresent(ren);
}



void keyInput( View &view )
{
    const glm::vec2 speed = glm::vec2(2.5f);

    const uint8_t *state = SDL_GetKeyboardState(NULL);

    if (state[SDL_SCANCODE_A])  view.position.x -= speed.x;
    if (state[SDL_SCANCODE_D])  view.position.x += speed.x;
    if (state[SDL_SCANCODE_W])  view.position.y -= speed.y;
    if (state[SDL_SCANCODE_S])  view.position.y += speed.y;

    if (state[SDL_SCANCODE_UP])    view.scale += 1;
    if (state[SDL_SCANCODE_DOWN])  view.scale -= 1;

    view.scale = glm::clamp(view.scale, 8, 64);

}
