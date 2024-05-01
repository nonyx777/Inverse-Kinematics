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

    // target...
    target = Circle(10.f, joints[2].property.getPosition() - sf::Vector2f(50.f, 100.f));
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
    // ik
    solveIK(dt);
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

void Scene::getMousePos(sf::Vector2f mouse_position)
{
    target.property.setPosition(mouse_position);
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

void Scene::alignJoint()
{
    float angle = joints[0].property.getRotation();
    sf::Vector2f joint_2 = sf::Vector2f(Math::_cos(angle) * this->length, Math::_sin(angle) * this->length);
    joints[1].property.setPosition(joints[0].property.getPosition() + joint_2);

    angle = joints[1].property.getRotation();
    sf::Vector2f joint_3 = sf::Vector2f(Math::_cos(angle) * this->length, Math::_sin(angle) * this->length);
    joints[2].property.setPosition(joints[1].property.getPosition() + joint_3);
}

void Scene::solveIK(float dt)
{
    float distance = Math::_length(joints[2].property.getPosition() - target.property.getPosition());
    if (distance < epsilon)
        return;

    dO = deltaOrientation();
    joints[0].property.rotate(dO.x * timestep);
    joints[1].property.rotate(dO.y * timestep);
    alignJoint();
}

glm::vec2 Scene::deltaOrientation()
{
    jT = transposeJacobian();

    sf::Vector2f V_tempo = target.property.getPosition() - this->joints[2].property.getPosition();
    V = glm::vec2(V_tempo.x, V_tempo.y);

    return jT * V;
}

glm::mat2 Scene::transposeJacobian()
{
    glm::mat2x2 jacobian;

    // 1st column (1st joint)
    sf::Vector2f differ = this->joints[2].property.getPosition() - this->joints[0].property.getPosition();
    float dx_theta = -Math::_sin(this->joints[0].property.getRotation()) * differ.x;
    float dy_theta = -Math::_cos(this->joints[0].property.getRotation()) * differ.y;

    jacobian[0][0] = dx_theta;
    jacobian[1][0] = dy_theta;

    // 2nd column (2nd joint)
    differ = this->joints[2].property.getPosition() - this->joints[1].property.getPosition();
    dx_theta = -Math::_sin(this->joints[1].property.getRotation()) * differ.x;
    dy_theta = -Math::_cos(this->joints[1].property.getRotation()) * differ.y;

    jacobian[0][1] = dx_theta;
    jacobian[1][1] = dy_theta;

    // damping least square
    // float damping_factor = 0.1f;
    // glm::mat2x2 jacobian_t_damped = glm::transpose(jacobian) * glm::inverse(jacobian * glm::transpose(jacobian) + damping_factor * damping_factor * glm::mat2x2(1.f));

    // pseudoinverse
    // glm::mat2x2 jacobian_pseudoinv = glm::transpose(jacobian) * glm::inverse(jacobian * glm::transpose(jacobian));

    return glm::transpose(jacobian);
}