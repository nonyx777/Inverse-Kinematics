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

    // configuring orientations for joints
    // 1st joint
    sf::Vector2f o_joint1 = joints[1].property.getPosition() - joints[0].property.getPosition();
    float angle = Math::_atan2(o_joint1.y, o_joint1.x);
    joints[0].property.setRotation(angle);
    O.x = angle;
    
    // 2nd joint
    sf::Vector2f o_joint2 = joints[2].property.getPosition() - joints[1].property.getPosition();
    angle = Math::_atan2(o_joint2.y, o_joint2.x);
    joints[1].property.setRotation(angle);
    O.y = angle;

    // setting up the links
    for (int i = 0; i < this->joints.size() - 1; i++)
    {
        Line line = Line(this->joints[i].property.getPosition(), this->joints[i + 1].property.getPosition());
        this->links.push_back(line);
    }

    // target...
    target = Circle(10.f, joints[2].property.getPosition() + sf::Vector2f(0.f, 50.f));
    target.property.setFillColor(sf::Color::Red);
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
    this->alignLink();

    joints[0].property.rotate(1.f);
    float angle = joints[0].property.getRotation();
    sf::Vector2f joint_2 = sf::Vector2f(Math::_cos(angle) * this->length, Math::_sin(angle) * this->length);
    joints[1].property.setPosition(joints[0].property.getPosition() + joint_2);

    joints[1].property.rotate(0.5f);
    angle = joints[1].property.getRotation();
    sf::Vector2f joint_3 = sf::Vector2f(Math::_cos(angle) * this->length, Math::_sin(angle) * this->length);
    joints[2].property.setPosition(joints[1].property.getPosition() + joint_3);
}

void Scene::render(sf::RenderTarget *target)
{
    for (Circle &circle : this->joints)
        circle.render(target);
    for (Line &line : this->links)
        line.render(target);

    this->target.render(target);

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

void Scene::alignLink()
{
    this->links.clear();

    for (int i = 0; i < this->joints.size() - 1; i++)
    {
        Line line = Line(this->joints[i].property.getPosition(), this->joints[i + 1].property.getPosition());
        this->links.push_back(line);
    }
}

void Scene::solveIK(float dt)
{
    float distance = Math::_length(joints[2].property.getPosition() - target.property.getPosition());
    while (distance > epsilon)
    {
        dO = deltaOrientation();
        O += dO * dt;
    }
}

glm::vec2 Scene::deltaOrientation()
{
    jT = transposeJacobian();

    sf::Vector2f V_tempo = Math::_displacement(target.property.getPosition(), this->joints[2].property.getPosition());
    V = glm::vec2(V_tempo.x, V_tempo.y);

    dO = jT * V;
}

glm::mat2 Scene::transposeJacobian()
{
    glm::mat2 jacobian = glm::mat2(1.f);
    jT = glm::transpose(jacobian);

    return jT;
}