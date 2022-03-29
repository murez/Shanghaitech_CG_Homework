#ifndef _COLLISION_
#define _COLLISION_

#include <defines.h>
#include <rigid_body.h>

// The structure to store the collision information.
struct CollisionInfo
{
    bool is_collided;           // Collided or not.
    glm::vec3 collision_point;  // The collision point.
    float collision_degree;     // Collision degree.

    CollisionInfo() : is_collided(false)
    {
    }
};

// Collision handler
class CollisionHandler
{
public:
    float normal_epsilon;   // The collision decay factor along the normal direction.
    float tangent_epsilon;  // The collision decay factor along the tangent direction.
    float dt;               // The simulation time step.

public:
    CollisionHandler() = default;
    ~CollisionHandler() = default;

    // Check collision and obtain the collision information if necessary.
    bool Check(Wall &wall, Sphere &sphere, CollisionInfo &collision_info);
    bool Check(Sphere &sphere0, Sphere &sphere1, CollisionInfo &collision_info);
    // Handle the collision if necessary.
    void Handle(Wall &wall, Sphere &sphere);
    void Handle(Sphere &sphere0, Sphere &sphere1);
protected:
    // Helper function to check the collision between a parallelogram and a sphere.
    static bool CheckParlgrmAndSphere(
        glm::vec3 parlgrm_x,
        glm::vec3 parlgrm_s1,
        glm::vec3 parlgrm_s2,
        glm::vec3 sphere_x,
        float sphere_r,
        CollisionInfo &collision_info);
    static bool CheckSphereAndSphere(
            vec3 sphere0_x,
            vec3 sphere1_x,
            float sphere0_r,
            float sphere1_r,
            CollisionInfo& collision_info);
};

#endif // _COLLISION_