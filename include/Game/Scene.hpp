#pragma once
#include <glm/glm.hpp>

#include "../GameObject.hpp"
#include "../Entities/Entities.hpp"
#include "../Util/Gizmo.hpp"
#include "../Util/Grid.hpp"
#include "../Globals.hpp"
#include "../Util/Collision.hpp"

class Scene : public GameObject
{
private:
    static Scene *instance;
    std::vector<std::vector<Box>> grid;

    std::vector<Circle> joints;
    std::vector<Line> links;

    Circle target;
    float length = 100.f;

    //IK related
    glm::vec2 O, dO, V;
    glm::mat2x2 jT = glm::mat2(1.f);
    float epsilon = 0.1f;
    float timestep = 0.001f;

private:
    Scene();
    ~Scene();

public:
    // Delete copy constructor and assignment operator to prevent cloning
    Scene(const Scene &) = delete;
    Scene &operator=(const Scene &) = delete;

    static Scene *getInstance();

    void update(float dt) override;
    void update(sf::Vector2f &vec, float dt);
    void render(sf::RenderTarget *target) override;

    //get mouse position
    void getMousePos(sf::Vector2f mouse_position);

    //joint and link
    void alignLink();
    void alignJoint();

    //IK related
    void solveIK(float dt);
    glm::vec2 deltaOrientation();
    glm::mat2 transposeJacobian();
};