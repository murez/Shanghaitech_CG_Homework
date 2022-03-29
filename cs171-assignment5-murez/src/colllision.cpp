#include <collision.h>
#include <utils.h>

/**
 * @brief This function checks whether the sphere collides the wall or not, and stores the collision information if necessary.
 * 
 * @param[in] wall 
 * @param[in] sphere 
 * @param[out] collision_info 
 * @return true 
 * @return false 
 */
bool CollisionHandler::Check(Wall &wall, Sphere &sphere, CollisionInfo &collision_info) {
    return CheckParlgrmAndSphere(
            wall.GetX(), wall.GetS1(), wall.GetS2(),
            sphere.GetX(), sphere.GetRadius(), collision_info);
}

bool CollisionHandler::Check(Sphere &sphere0, Sphere &sphere1, CollisionInfo &collision_info) {
    return CheckSphereAndSphere(
            sphere0.GetX(), sphere1.GetX(), sphere0.GetRadius(), sphere1.GetRadius(), collision_info);
}

/**
 * @brief This function handles the collision between the sphere and the wall.
 * 
 * @param[in] wall
 * @param[in] sphere 
 */
void CollisionHandler::Handle(Wall &wall, Sphere &sphere) {
    /// DONE: Handle the colliding contact between the sphere and the wall.
    CollisionInfo collision_info;
    if (Check(wall, sphere, collision_info)) {
        vec3 n = glm::normalize(glm::cross(wall.GetS1(), wall.GetS2()));
        vec3 c_v = sphere.GetV();
        vec3 n_v = glm::dot(c_v, n) * n;
        vec3 next_v = (tangent_epsilon - 1) * c_v - (normal_epsilon + tangent_epsilon) * n_v;
        vec3 F = sphere.GetM() * next_v / dt;
        for (auto &constant_force: sphere.constant_forces) {
            F -= constant_force.F;
        }
        Force force(collision_info.collision_point - sphere.GetX(), F);
        sphere.transient_forces.push_back(force);
    }
}

/**
 * @brief This function handles the collision between the sphere and the wall.
 *
 * @param[in] wall
 * @param[in] sphere
 */
void CollisionHandler::Handle(Sphere &sphere0, Sphere &sphere1) {
    /// DONE: Handle the colliding contact between the sphere and the sphere.
    CollisionInfo collision_info;
    if (Check(sphere0, sphere1, collision_info)) {
        vec3 v0 = sphere0.GetV();
        vec3 v1 = sphere1.GetV();
        float m0 = sphere0.GetM();
        float m1 = sphere1.GetM();
        vec3 next_v = ((m0 - m1) * v0 + (1 + normal_epsilon) * m1 * v1) / (m0 + m1);
        vec3 F = m0 * (next_v - v0) / dt;
        for (auto &constant_force: sphere0.constant_forces) {
            F -= constant_force.F;
        }
        Force force(vec3(0.0f, 0.0f, 0.0f), F);
        sphere0.transient_forces.push_back(force);
    }
}

bool in_parlgrm(glm::vec3 x, glm::vec3 s1, glm::vec3 s2, glm::vec3 p) {
    auto v0 = x + s1 / 2.0f + s2 / 2.0f;
    auto v2 = x - s1 / 2.0f - s2 / 2.0f;
    return glm::dot(glm::cross(-s2, p - v0), glm::cross(p - v0, -s1)) >= 0 &&
           glm::dot(glm::cross(s1, p - v2), glm::cross(p - v2, s2)) >= 0;
}

/**
 * @brief This function checks whether the parallelogram collides with the sphere.
 *        The collision information will be store if there exists the collision.
 * @param[in] parlgrm_x 
 * @param[in] parlgrm_s1 
 * @param[in] parlgrm_s2 
 * @param[in] sphere_x 
 * @param[in] sphere_r 
 * @param[out] collision_info 
 * @return true 
 * @return false 
 */
bool CollisionHandler::CheckParlgrmAndSphere(
        glm::vec3 parlgrm_x,
        glm::vec3 parlgrm_s1,
        glm::vec3 parlgrm_s2,
        glm::vec3 sphere_x,
        float sphere_r,
        CollisionInfo &collision_info) {
    /// DONE: Check the collision between the parallelogram and the sphere.
    vec3 n = glm::normalize(glm::cross(parlgrm_s1, parlgrm_s2));
    vec3 p = glm::dot(n, sphere_x - parlgrm_x) * n;
    vec3 c = sphere_x - p;
    float d = glm::length(p);
    if (d < sphere_r && in_parlgrm(parlgrm_x, parlgrm_s1, parlgrm_s2, c)) {
        collision_info.is_collided = true;
        collision_info.collision_point = c;
        collision_info.collision_degree = sphere_r - d;
        return true;
    }
    return false;
}


/**
 * @brief This function checks whether the sphere collides with the sphere.
 *        The collision information will be store if there exists the collision.
 * @param[in] sphere0_x
 * @param[in] sphere1_x
 * @param[in] sphere0_r
 * @param[in] sphere1_r
 * @param[out] collision_info
 * @return true
 * @return false
 */
bool CollisionHandler::CheckSphereAndSphere(
        vec3 sphere0_x,
        vec3 sphere1_x,
        float sphere0_r,
        float sphere1_r,
        CollisionInfo &collision_info) {
    float d = glm::length(sphere0_x - sphere1_x);
    if (d < sphere0_r + sphere1_r) {
        collision_info.is_collided = true;
        collision_info.collision_point = sphere0_x + (sphere1_x - sphere0_x) * (sphere0_x / d);
        collision_info.collision_degree = sphere0_r + sphere1_r - d;
        return true;
    }
    return false;
}



