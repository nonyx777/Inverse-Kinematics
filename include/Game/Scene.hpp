#pragma once
#include <glm/glm.hpp>

#include "../GameObject.hpp"
#include "../Entities/Entities.hpp"
#include "../Util/Gizmo.hpp"
#include "../Util/Grid.hpp"
#include "../Globals.hpp"
#include "../Util/Collision.hpp"
#include "../Util/Loader.hpp"

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
    glm::vec2 dO, V;
    glm::mat2x2 jT;
    float epsilon = 0.1f;
    float timestep = 0.001f;

    //body parts
    sf::Texture upperarm_texture, lowerarm_texture, wrist_texture;
    Box upperarm, lowerarm, wrist;

private:
    Scene();
    ~Scene();

public:
    // Delete copy constructor and assignment operator to prevent cloning
    Scene(const Scene &) = delete;
    Scene &operator=(const Scene &) = delete;

    static Scene *getInstance();

    void update(float dt) override;
    void render(sf::RenderTarget *target) override;

    //get mouse position
    void getMousePos(sf::Vector2f mouse_position);

    //joint and link
    void alignLink();
    void alignJoint();
    void alignBody();

    //IK related
    void solveIK(float dt);
    glm::vec2 deltaOrientation();
    glm::mat2 transposeJacobian();
};