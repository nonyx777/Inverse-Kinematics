#include "../../include/Game/Scene.hpp"

Scene *Scene::instance = nullptr;

Scene::Scene()
{
    if (GLOBAL::display_grid)
    {
        configureGrid(GLOBAL::cell_size, &this->grid);
    }

    // setting up joints
    Circle joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f - 100.f, GLOBAL::window_height / 2.f));
    this->joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f, GLOBAL::window_height / 2.f + 100.f));
    this->joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 100.f, GLOBAL::window_height / 2.f));
    this->joints.push_back(joint);

    // setting up the links
    for (int i = 0; i < this->joints.size() - 1; i++)
    {
        Line line = Line(this->joints[i].property.getPosition(), this->joints[i + 1].property.getPosition());
        this->links.push_back(line);
    }
}

Scene::~Scene()
{
    delete instance;
}

Scene *Scene::getInstance()
{
    if (!instance)
        instance = new Scene();

    return instance;
}

void Scene::update(float dt)
{
}

void Scene::render(sf::RenderTarget *target)
{
    for (Circle &circle : this->joints)
        circle.render(target);
    for (Line &line : this->links)
        line.render(target);

    if (this->grid.size() > 0)
    {
        for (uint i = 0; i < grid.size(); i++)
        {
            for (uint j = 0; j < grid[i].size(); j++)
            {
                target->draw(grid[i][j].property);
            }
        }
    }
}