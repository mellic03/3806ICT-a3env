#include <ros/ros.h>
#include <ros/package.h>

#include "a3env/sonars.h"
#include "a3env/motors.h"
#include "a3env/odom.h"
#include "a3env/deactivate.h"

#include "environment.hpp"
#include "entities.hpp"
#include "render.hpp"
#include "mazegen.hpp"

#include "common.hpp"
using namespace a3env;



void initWindow( SDL_Window*&, SDL_Renderer*&, View& );
void renderLoop( SDL_Renderer*&, View& );
void renderUI( SDL_Renderer*, View& );

void keyInput( View& );
void mouseInput( View& );

void updateEnvironment();
void motors_callback( const a3env::motors &msg );
void deactivate_callback( const a3env::deactivate &msg );


static std::vector<Entity *>        entities;
static std::vector<Agent *>         agents;
static std::vector<Hostile *>       hostiles;
static std::vector<Survivor *>      survivors;

static Environment                  environment (MAP_WIDTH);
static std::vector<ros::Publisher>  sonars_pub  (NUM_AGENTS);
static std::vector<ros::Publisher>  odom_pub    (NUM_AGENTS);
static std::vector<ros::Subscriber> motors_sub  (NUM_AGENTS);



int main( int argc, char **argv )
{
    // Load environment from file
    // --------------------------------------------------------------------------
    std::string path = ros::package::getPath("a3env");
    environment.loadFile(path + "/data/m0.txt");
    // --------------------------------------------------------------------------

    srand(clock());

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

    for (int i=0; i<NUM_SURVIVORS; i++)
    {
        entities.push_back(new Survivor(i));
        survivors.push_back(dynamic_cast<Survivor *>(entities.back()));
    }

    for (int i=0; i<NUM_AGENTS; i++)
    {
        int idx = environment.randomFreeCell();
        int r = idx / MAP_WIDTH;
        int c = idx % MAP_WIDTH;

        agents[i]->position = glm::vec2(c, r) + 0.5f;
        agents[i]->linear   = 0.0f;
        agents[i]->angular  = 0.0f;
        agents[i]->bearing  = 0.0f;
        agents[i]->sonar_bearing  = 0.0f;
    }

    for (int i=0; i<NUM_HOSTILES; i++)
    {
        int idx = environment.randomFreeCell();
        int r = idx / MAP_WIDTH;
        int c = idx % MAP_WIDTH;

        hostiles[i]->position = glm::vec2(c, r) + 0.5f;
        hostiles[i]->linear   = 0.05f;
        hostiles[i]->angular  = 0.0f;
        hostiles[i]->bearing  = 0.0f;
    }

    for (int i=0; i<NUM_SURVIVORS; i++)
    {
        int idx = environment.randomFreeCell();
        int r = idx / MAP_WIDTH;
        int c = idx % MAP_WIDTH;

        survivors[i]->position = glm::vec2(c, r) + 0.5f;
        survivors[i]->linear   = 0.0f;
        survivors[i]->angular  = 0.0f;
        survivors[i]->bearing  = 0.0f;
    }
    // --------------------------------------------------------------------------


    // ROS
    // --------------------------------------------------------------------------
    ros::init(argc, argv, "a3env");
    ros::NodeHandle n;

    ros::Subscriber deactivate_sub = n.subscribe("a3env/deactivate", 8, deactivate_callback);

    for (int i=0; i<NUM_AGENTS; i++)
    {
        std::string label1 = "a3env/sonars" + std::to_string(i);
        std::string label2 = "a3env/odom" + std::to_string(i);
        std::string label3 = "/a3env/motors" + std::to_string(i);

        sonars_pub[i] = n.advertise<a3env::sonars>(label1, 64);
        odom_pub[i]   = n.advertise<a3env::odom>(label2, 64);
        motors_sub[i] = n.subscribe(label3, 64, motors_callback);
    }
    // ros::ServiceServer service = n.advertiseService("a3env/motors", motors_callback);
    // --------------------------------------------------------------------------



    // Rendering
    // --------------------------------------------------------------------------
    SDL_Window   *window;
    SDL_Renderer *ren;

    View view = {
        .position   = glm::vec2(a3env::MAP_WIDTH/2.0f),
        .resolution = glm::ivec2(1024),
        .scale      = 64.0f
    };

    initWindow(window, ren, view);
    // --------------------------------------------------------------------------

    // ros::Rate rate(4);

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
    environment.updateSurvivors(agents, hostiles, survivors);

    for (int i=0; i<NUM_AGENTS; i++)
    {
        a3env::sonars S;
        S.distance  = agents[i]->sonar_dist;
        S.dx        = cos(agents[i]->sonar_bearing);
        S.dy        = sin(agents[i]->sonar_bearing);
        S.xhit      = agents[i]->sonar_hit.x;
        S.yhit      = agents[i]->sonar_hit.y;
        S.blocktype = agents[i]->sonar_block;
        S.data      = agents[i]->sonar_data;

        a3env::odom O;
        O.xpos      = agents[i]->position.x;
        O.ypos      = agents[i]->position.y;
        // O.bearing   = agents[i]->bearing;

        sonars_pub[i].publish(S);
        odom_pub[i].publish(O);
    }
}



void motors_callback( const a3env::motors &msg )
{
    if (msg.agentid < 0 || msg.agentid >= NUM_AGENTS)
    {
        ROS_ERROR("REE");
        return;
    }

    agents[msg.agentid]->bearing = msg.bearing;
    agents[msg.agentid]->linear  = msg.linear;

}

void deactivate_callback( const a3env::deactivate &msg )
{
    ROS_INFO("Agent %d deactivated.", msg.agentid);
    agents[msg.agentid]->active = false;
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
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);
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



    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderClear(ren);

    keyInput(view);
    mouseInput(view);
    renderGrid(ren, view, environment.m_data);
    renderUI(ren, view);


    for (Entity *e: entities)
    {
        renderEntity(ren, view, e); 
    }

    SDL_RenderPresent(ren);
}


void renderUI( SDL_Renderer *ren, View &view )
{
    glm::vec2 mouse = view.mouse_world;

    renderRect(
        ren, view,
        glm::floor(view.mouse_world),
        glm::vec2(1.0f),
        glm::ivec4(255, 255, 255, 50)
    );

    // for (Agent *agent: agents)
    // {
    //     if (glm::distance(mouse, agent->position) < 0.25f)
    //     {
    //         renderRect(
    //             ren, view,
    //             agent->position - glm::vec2(0.25f),
    //             glm::vec2(0.5f),
    //             glm::ivec4(255, 255, 255, 100)
    //         );
    //     }
    // }
}



void keyInput( View &view )
{
    const uint8_t *state = SDL_GetKeyboardState(NULL);
    glm::vec2 speed = glm::vec2(5.0f) / view.scale;

    if (state[SDL_SCANCODE_LSHIFT]) speed *= 2.0f;
    if (state[SDL_SCANCODE_A])      view.position.x -= speed.x;
    if (state[SDL_SCANCODE_D])      view.position.x += speed.x;
    if (state[SDL_SCANCODE_W])      view.position.y -= speed.y;
    if (state[SDL_SCANCODE_S])      view.position.y += speed.y;
    if (state[SDL_SCANCODE_UP])     view.scale += 1.0f;
    if (state[SDL_SCANCODE_DOWN])   view.scale -= 1.0f;

    view.scale = glm::clamp(view.scale, 32.0f, 128.0f);

}


void mouseInput( View &view )
{
    bool prev_down = view.mouse_down;
    view.mouse_clicked = false;

    if (SDL_GetMouseState(&view.mouse_screen.x, &view.mouse_screen.y) && SDL_BUTTON_LMASK)
    {
        view.mouse_down = true;
    }

    else
    {
        view.mouse_down = false;
    }

    if (prev_down == true && view.mouse_down == false)
    {
        view.mouse_clicked = true;
    }


    const float S = view.scale;

    glm::vec2 res    = glm::vec2(view.resolution);
    glm::vec2 screen = glm::vec2(view.mouse_screen);

    view.mouse_world = (1.0f/S) * (screen - res/2.0f) + view.position;
}


