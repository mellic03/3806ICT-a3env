#include "render.hpp"
#include "common.hpp"

constexpr glm::ivec3 BlockColors[9] = {
    glm::ivec3(20),
    glm::ivec3(0),
    glm::ivec3(170, 178, 181),
    glm::ivec3(0, 255, 0),
    glm::ivec3(255, 0, 0)
};


constexpr glm::ivec3 EntityBodyColors[3] = {
    // glm::ivec3(255, 0, 0),
    // glm::ivec3(0, 255, 0),
    glm::ivec3(100, 100, 0),
    glm::ivec3(100, 100, 0),
    glm::ivec3(0, 0, 255)
};


constexpr glm::ivec3 EntityHeadColors[3] = {
    glm::ivec3(0, 255, 0),
    glm::ivec3(255, 0, 0),
    glm::ivec3(0, 0, 255)
};




void renderRect( SDL_Renderer *ren, const View &view, glm::vec2 pos, glm::vec2 extents,
                 const glm::ivec4 &color )
{
    const float S = view.scale;

    int x = S*(pos.x - view.position.x) + (view.resolution.x / 2.0f);
    int y = S*(pos.y - view.position.y) + (view.resolution.y / 2.0f);

    int w = S*extents.x;
    int h = S*extents.y;

    SDL_Rect rect = {
        .x = x,
        .y = y,
        .w = w,
        .h = h
    };

    SDL_SetRenderDrawColor(ren, color.r, color.g, color.b, color.a);
    SDL_RenderFillRect(ren, &rect);
}

void renderRect( SDL_Renderer *ren, const View &view, glm::vec2 pos, glm::vec2 extents,
                 const glm::ivec3 &color )
{
    renderRect(ren, view, pos, extents, glm::ivec4(color, 255));
}



void renderGrid( SDL_Renderer *ren, const View &view, const std::vector<std::vector<uint8_t>> &grid )
{
    const glm::vec2 P = view.position;
    const int       W = grid.size();
    const float     S = view.scale;

    for (int i=0; i<W; i++)
    {
        for (int j=0; j<W; j++)
        {
            renderRect(
                ren, view,
                glm::vec2(j, i),
                glm::vec2(1.0f),
                BlockColors[grid[i][j]]
            );

        }
    }
}



void renderEntity( SDL_Renderer *ren, const View &view, Entity *e )
{
    constexpr float body_w = a3env::ENTITY_BODY_W;
    constexpr float head_w = a3env::ENTITY_HEAD_W;

    const glm::vec2 pos = e->position;

    glm::vec2 dir = glm::vec2(cos(e->bearing), sin(e->bearing));
    glm::vec2 head = pos + (head_w/2.0f)*dir;

    renderRect(
        ren, view,
        pos - glm::vec2(body_w/2.0f),
        glm::vec2(body_w),
        EntityBodyColors[int(e->type)]
    );

    renderRect(
        ren, view,
        head - glm::vec2(head_w/2.0f),
        glm::vec2(head_w),
        EntityHeadColors[int(e->type)]
    );


    if (e->type == ENTITY_AGENT)
    {
        Agent *a = (Agent *)e;
        glm::vec2 sdir = glm::vec2(cos(a->sonar_bearing), sin(a->sonar_bearing));

        glm::vec2 hit = pos + (dynamic_cast<Agent *>(e))->sonar_dist * sdir;
        renderLine(ren, view, pos, hit);
    }

}




void renderLine( SDL_Renderer *ren, const View &view,
                 const glm::vec2 &A, const glm::vec2 &B )
{
    const float S = view.scale;

    int x0 = S*(A.x - view.position.x) + (view.resolution.x / 2.0f);
    int y0 = S*(A.y - view.position.y) + (view.resolution.y / 2.0f);
    int x1 = S*(B.x - view.position.x) + (view.resolution.x / 2.0f);
    int y1 = S*(B.y - view.position.y) + (view.resolution.y / 2.0f);

    SDL_RenderDrawLine(ren, x0, y0, x1, y1);
}

