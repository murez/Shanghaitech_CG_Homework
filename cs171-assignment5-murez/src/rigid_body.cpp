#include <rigid_body.h>

///////////////////// RigidBody /////////////////////
glm::vec3 RigidBody::GetX() {
    return this->x;
}

glm::vec3 RigidBody::GetV() {
    return this->v;
}

float RigidBody::GetM() {
    return this->M;
}

void RigidBody::Draw(Shader &shader, ShaderParam &shader_param) {
    this->opengl_object.Draw(shader, shader_param);
}

void RigidBody::ClearTransientForces() {
    this->transient_forces.clear();
}

///////////////////// Sphere /////////////////////
Sphere::Sphere(
        glm::vec3 color,
        glm::vec3 x,
        glm::vec3 v,
        glm::vec3 w,
        float M,
        float radius) {
    this->x = x;
    this->v = v;
    this->w = w;
    this->M = M;

    this->radius = radius;

    InitOpenGLObject(color);
    InitInertiaTensor();

    Force G(glm::vec3(0.0f, 0.0f, 0.0f), this->M * g);
    this->constant_forces.push_back(G);

    this->P = this->M * this->v;
    this->L = this->I * this->w;
}

/**
 * @brief Get the radius of the sphere.
 * 
 * @return float 
 */
float Sphere::GetRadius() {
    return this->radius;
}

/**
 * @brief This function will update the sphere's state forward for the time step dt.
 * 
 * @param[in] dt 
 */
void Sphere::Forward(float dt) {
    // DONE: Your code here.
    for (auto &force: this->transient_forces) {
        this->v += force.F / this->M * dt;
    }
    for (auto &force: this->constant_forces) {
        this->v += force.F / this->M * dt;
    }
    x += v * dt;
    UpdateOpenGLObject();
}

/**
 * @brief This function will update the sphere's state backward for the time step dt.
 *        You may need this function to handle the collision.
 * 
 * @param[in] dt 
 */
void Sphere::Backward(float dt) {
    /// DONE: Your code here.
    x -= v * dt;
    for (auto &force: this->transient_forces) {
        this->v -= force.F / this->M * dt;
    }
    for (auto &force: this->constant_forces) {
        this->v -= force.F / this->M * dt;
    }
    UpdateOpenGLObject();
}

/**
 * @brief If you consider the rotation as well, you need to handle the intertia tensor.
 * 
 */
void Sphere::InitInertiaTensor() {
    this->I = glm::mat3(
            this->M * (this->radius * this->radius) / 2.5f, 0.0f, 0.0f,
            0.0f, this->M * (this->radius * this->radius) / 2.5f, 0.0f,
            0.0f, 0.0f, this->M * (this->radius * this->radius) / 2.5f);
    this->I_inv = glm::inverse(this->I);
}

/**
 * @brief This function initializes the opengl object for rendering.
 *        Specifically, the vertices, indices are generated.
 * 
 * @param[in] color 
 */
void Sphere::InitOpenGLObject(glm::vec3 color) {
    opengl_object.color = color;

    float delta = 0.01;
    int size = 2 * PI / delta + 1;

    opengl_object.vertices.resize(size * size);
    for (int t = 0; t < size; t++) {
        float theta = (t == size - 1) ? (2 * PI) : t * delta;
        for (int p = 0; p < size; p++) {
            float phi = (p == size - 1) ? (2 * PI) : p * delta;

            float x = radius * cosf(phi) * sinf(theta);
            float y = radius * sinf(phi) * sinf(theta);
            float z = radius * cosf(theta);

            opengl_object.vertices[t * size + p].position = glm::vec3(x, y, z);
            opengl_object.vertices[t * size + p].normal = normalize(glm::vec3(x, y, z));
        }
    }

    opengl_object.indices.reserve(3 * 2 * (size - 1) * (size - 1));
    for (int t = 0; t < size - 1; t++) {
        for (int p = 0; p < size - 1; p++) {
            opengl_object.indices.push_back(t * size + p);
            opengl_object.indices.push_back(t * size + p + 1);
            opengl_object.indices.push_back((t + 1) * size + p);

            opengl_object.indices.push_back(t * size + p + 1);
            opengl_object.indices.push_back((t + 1) * size + p);
            opengl_object.indices.push_back((t + 1) * size + p + 1);
        }
    }

    opengl_object.M = glm::translate(glm::mat4(1.0f), this->x);

    opengl_object.InitData();
}

/**
 * @brief This functioin updates the opengl object for rendering.
 *        We do not consider the rotation so we only need to handle the translation.
 *        
 */
void Sphere::UpdateOpenGLObject() {
    opengl_object.M = glm::translate(glm::mat4(1.0f), this->x);
}

///////////////////// Wall /////////////////////
Wall::Wall(
        glm::vec3 color,
        glm::vec3 x,
        glm::vec3 s1,
        glm::vec3 s2) {
    this->x = x;
    this->s1 = s1;
    this->s2 = s2;

    this->v = glm::vec3(0.0f);
    this->w = glm::vec3(0.0f);
    this->M = INFINITY;

    InitInertiaTensor();
    InitOpenGLObject(color);

    this->P = glm::vec3(0.0f);
    this->L = glm::vec3(0.0f);
}

glm::vec3 Wall::GetS1() {
    return this->s1;
}

glm::vec3 Wall::GetS2() {
    return this->s2;
}

void Wall::Forward(float dt) {
    return;
}

void Wall::Backward(float dt) {
    return;
}

void Wall::InitInertiaTensor() {
    return;
}

void Wall::InitOpenGLObject(glm::vec3 color) {
    opengl_object.color = color;

    opengl_object.vertices.resize(4);
    opengl_object.vertices[0].position = this->x - s1 / 2.0f + s2 / 2.0f; // top-left
    opengl_object.vertices[1].position = this->x + s1 / 2.0f + s2 / 2.0f; // top-right
    opengl_object.vertices[2].position = this->x - s1 / 2.0f - s2 / 2.0f; // bottom-left
    opengl_object.vertices[3].position = this->x + s1 / 2.0f - s2 / 2.0f; // bottom-right
    opengl_object.vertices[0].normal = normalize(cross(s1, s2));
    opengl_object.vertices[1].normal = opengl_object.vertices[0].normal;
    opengl_object.vertices[2].normal = opengl_object.vertices[0].normal;
    opengl_object.vertices[3].normal = opengl_object.vertices[0].normal;

    opengl_object.indices = {0, 1, 2, 1, 2, 3};

    opengl_object.M = glm::mat4(1.0f);

    opengl_object.InitData();

    return;
}

void Wall::UpdateOpenGLObject() {
    return;
}

Cube::Cube(glm::vec3 color, glm::vec3 x, glm::vec3 v, glm::vec3 w, float M, float e) {
    this->x = x;
    this->v = v;
    this->w = w;
    this->M = M;
    this->e = e;

    this->P = glm::vec3(0.0f);
    this->L = glm::vec3(0.0f);

    InitInertiaTensor();
    InitOpenGLObject(color);

    Force G(glm::vec3(0.0f, 0.0f, 0.0f), this->M * g);
    this->constant_forces.push_back(G);

    this->P = this->M * this->v;
    this->L = this->I * this->w;
}

float Cube::GetEdgeLength() {
    return e;
}

void Cube::Forward(float dt) {
    for (auto &force: this->transient_forces) {
        this->v += force.F / this->M * dt;
    }
    for (auto &force: this->constant_forces) {
        this->v += force.F / this->M * dt;
    }
    x += v * dt;
    UpdateOpenGLObject();
}

void Cube::Backward(float dt) {
    x -= v * dt;
    for (auto &force: this->transient_forces) {
        this->v -= force.F / this->M * dt;
    }
    for (auto &force: this->constant_forces) {
        this->v -= force.F / this->M * dt;
    }
    UpdateOpenGLObject();
}

void Cube::InitInertiaTensor() {
    this->I = this->M * (this->e * this->e) * glm::mat3(
            2.0 / 3.0, -1.0 / 4.0, -1.0 / 4.0,
            -1.0 / 4.0, 2.0 / 3.0, -1.0 / 4.0,
            -1.0 / 4.0, -1.0 / 4.0, 2.0 / 3.0);
    this->I_inv = glm::inverse(this->I);
}

void Cube::InitOpenGLObject(glm::vec3 color) {
    opengl_object.color = color;
    opengl_object.vertices.resize(24);
    /*
     *
     *    p0.+------+p3
     *    .' |    .'|
     * p1+---+--+'p2|
     *   |   |  |   |
     *   |p4,+--+---+p7
     *   |.'    | .'
     * p5+------+'p6
     *
     *
     * */
    // top
    opengl_object.vertices[0].position = this->x + glm::vec3(-e / 2.0f, e / 2.0f, -e / 2.0f); // p0
    opengl_object.vertices[1].position = this->x + glm::vec3(e / 2.0f, e / 2.0f, -e / 2.0f);  // p1
    opengl_object.vertices[2].position = this->x + glm::vec3(e / 2.0f, e / 2.0f, e / 2.0f);   // p2
    opengl_object.vertices[3].position = this->x + glm::vec3(-e / 2.0f, e / 2.0f, e / 2.0f);  // p3
    opengl_object.vertices[0].normal = normalize(vec3(0, 1, 0));
    opengl_object.vertices[1].normal = opengl_object.vertices[0].normal;
    opengl_object.vertices[2].normal = opengl_object.vertices[0].normal;
    opengl_object.vertices[3].normal = opengl_object.vertices[0].normal;
    // bottom
    opengl_object.vertices[4].position = this->x + glm::vec3(-e / 2.0f, -e / 2.0f, -e / 2.0f); // p4
    opengl_object.vertices[5].position = this->x + glm::vec3(e / 2.0f, -e / 2.0f, -e / 2.0f);  // p5
    opengl_object.vertices[6].position = this->x + glm::vec3(e / 2.0f, -e / 2.0f, e / 2.0f);   // p6
    opengl_object.vertices[7].position = this->x + glm::vec3(-e / 2.0f, -e / 2.0f, e / 2.0f);  // p7
    opengl_object.vertices[4].normal = normalize(vec3(0, -1, 0));
    opengl_object.vertices[5].normal = opengl_object.vertices[4].normal;
    opengl_object.vertices[6].normal = opengl_object.vertices[4].normal;
    opengl_object.vertices[7].normal = opengl_object.vertices[4].normal;
    // front
    opengl_object.vertices[8].position = this->x + glm::vec3(e / 2.0f, e / 2.0f, -e / 2.0f);   // p1
    opengl_object.vertices[9].position = this->x + glm::vec3(e / 2.0f, e / 2.0f, e / 2.0f);    // p2
    opengl_object.vertices[10].position = this->x + glm::vec3(e / 2.0f, -e / 2.0f, e / 2.0f);  // p6
    opengl_object.vertices[11].position = this->x + glm::vec3(e / 2.0f, -e / 2.0f, -e / 2.0f); // p5
    opengl_object.vertices[8].normal = normalize(vec3(1, 0, 0));
    opengl_object.vertices[9].normal = opengl_object.vertices[8].normal;
    opengl_object.vertices[10].normal = opengl_object.vertices[8].normal;
    opengl_object.vertices[11].normal = opengl_object.vertices[8].normal;
    // back
    opengl_object.vertices[12].position = this->x + glm::vec3(-e / 2.0f, e / 2.0f, -e / 2.0f);  // p0
    opengl_object.vertices[13].position = this->x + glm::vec3(-e / 2.0f, e / 2.0f, e / 2.0f);   // p3
    opengl_object.vertices[14].position = this->x + glm::vec3(-e / 2.0f, -e / 2.0f, e / 2.0f);  // p7
    opengl_object.vertices[15].position = this->x + glm::vec3(-e / 2.0f, -e / 2.0f, -e / 2.0f); // p4
    opengl_object.vertices[12].normal = normalize(vec3(-1, 0, 0));
    opengl_object.vertices[13].normal = opengl_object.vertices[12].normal;
    opengl_object.vertices[14].normal = opengl_object.vertices[12].normal;
    opengl_object.vertices[15].normal = opengl_object.vertices[12].normal;
    // left
    opengl_object.vertices[16].position = this->x + glm::vec3(-e / 2.0f, e / 2.0f, -e / 2.0f);  // p0
    opengl_object.vertices[17].position = this->x + glm::vec3(e / 2.0f, e / 2.0f, -e / 2.0f);   // p1
    opengl_object.vertices[18].position = this->x + glm::vec3(e / 2.0f, -e / 2.0f, -e / 2.0f);  // p5
    opengl_object.vertices[19].position = this->x + glm::vec3(-e / 2.0f, -e / 2.0f, -e / 2.0f); // p4
    opengl_object.vertices[16].normal = normalize(vec3(0, 0, -1));
    opengl_object.vertices[17].normal = opengl_object.vertices[16].normal;
    opengl_object.vertices[18].normal = opengl_object.vertices[16].normal;
    opengl_object.vertices[19].normal = opengl_object.vertices[16].normal;
    // right
    opengl_object.vertices[20].position = this->x + glm::vec3(e / 2.0f, e / 2.0f, e / 2.0f);    // p2
    opengl_object.vertices[21].position = this->x + glm::vec3(-e / 2.0f, e / 2.0f, e / 2.0f);   // p3
    opengl_object.vertices[22].position = this->x + glm::vec3(-e / 2.0f, -e / 2.0f, e / 2.0f);  // p7
    opengl_object.vertices[23].position = this->x + glm::vec3(e / 2.0f, -e / 2.0f, e / 2.0f);   // p6
    opengl_object.vertices[20].normal = normalize(vec3(0, 0, -1));
    opengl_object.vertices[21].normal = opengl_object.vertices[20].normal;
    opengl_object.vertices[22].normal = opengl_object.vertices[20].normal;
    opengl_object.vertices[23].normal = opengl_object.vertices[20].normal;
    // indices
    opengl_object.indices = {0, 1, 2, 1, 2, 3,
                             4, 5, 6, 5, 6, 7,
                             8, 9, 10, 9, 10, 11,
                             12, 13, 14, 13, 14, 15,
                             16, 17, 18, 17, 18, 19,
                             20, 21, 22, 21, 22, 23};
    opengl_object.M = glm::mat4(1.0f);
    opengl_object.InitData();
    return;
}

void Cube::UpdateOpenGLObject() {

}



