#include "render.hpp"



void renderRect( SDL_Renderer *ren, const View &view, glm::vec2 pos, glm::vec2 extents, const glm::ivec3 &color )
{
    int x = pos.x - view.position.x + (view.resolution.x / 2.0f);
    int y = pos.y - view.position.y + (view.resolution.y / 2.0f);

    SDL_Rect rect = {
        .x = x,
        .y = y,
        .w = int(extents.x),
        .h = int(extents.y)
    };

    SDL_SetRenderDrawColor(ren, color.r, color.g, color.b, 255);
    SDL_RenderFillRect(ren, &rect);
}


void renderAgent( SDL_Renderer *ren, const View &view, Agent &agent )
{
    glm::vec2 pos = 32.0f * agent.position;

    glm::vec2 dir = glm::vec2(cos(agent.bearing), sin(agent.bearing));
    glm::vec2 head = pos + 4.0f*dir;

    renderRect(ren, view, pos-glm::vec2(5.0f), glm::vec2(10.0f), glm::ivec3(255, 0, 0));
    renderRect(ren, view, head-glm::vec2(2.5f), glm::vec2(5.0f), glm::ivec3(0, 255, 0));

}


