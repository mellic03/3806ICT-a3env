#include "render.hpp"


constexpr glm::ivec3 BlockColors[9] = {
    glm::ivec3(50),
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
                glm::vec2(S*j, S*i),
                glm::vec2(S),
                BlockColors[grid[i][j]]
            );

        }
    }
}



void renderEntity( SDL_Renderer *ren, const View &view, Entity *e )
{
    const float S = view.scale;
    const glm::vec2 pos = S * e->position;

    glm::vec2 dir = glm::vec2(cos(e->bearing), sin(e->bearing));
    glm::vec2 head = pos + 4.0f*dir;

    renderRect(
        ren, view,
        pos-glm::vec2(5.0f),
        glm::vec2(10.0f),
        EntityBodyColors[int(e->type)]
    );

    renderRect(
        ren, view,
        head-glm::vec2(2.5f),
        glm::vec2(5.0f),
        EntityHeadColors[int(e->type)]
    );


    if (e->type == ENTITY_AGENT)
    {
        glm::vec2 hit = pos + S * (dynamic_cast<Agent *>(e))->sonar_dist * dir;
        renderLine(ren, view, pos, hit);
    }

}




void renderLine( SDL_Renderer *ren, const View &view,
                 const glm::vec2 &A, const glm::vec2 &B )
{
    int x0 = A.x - view.position.x + (view.resolution.x / 2.0f);
    int y0 = A.y - view.position.y + (view.resolution.y / 2.0f);
    int x1 = B.x - view.position.x + (view.resolution.x / 2.0f);
    int y1 = B.y - view.position.y + (view.resolution.y / 2.0f);

    SDL_RenderDrawLine(ren, x0, y0, x1, y1);

}

