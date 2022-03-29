#include <object.h>
#include <shader.h>
#include <utils.h>
#include <fstream>
#include <vector>

Object::Object() {}

Object::~Object() {}

/**
 * TODO: initialize VAO, VBO, VEO and set the related varialbes
 */
void Object::init() {
    // generate
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // bind
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices.front(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices.front(), GL_STATIC_DRAW);
    // vertex
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(0);
    // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (sizeof(vec3)));
    glEnableVertexAttribArray(1);
    // reset
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

/**
 * TODO: draw object with VAO and VBO
 * You can choose to implement either one or both of the following functions.
 */

/* Implement this one if you do not use a shader */
void Object::drawArrays() const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
//    glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    glBindVertexArray(0);
}

/* Implement this one if you do use a shader */
void Object::drawArrays(const Shader &shader) const {
    shader.use();
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
//    glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    glBindVertexArray(0);
}

/**
 * TODO: draw object with VAO, VBO, and VEO
 * You can choose to implement either one or both of the following functions.
 */

/* Implement this one if you do not use a shader */
void Object::drawElements() const {
    // !DELETE THIS WHEN YOU FINISH THIS FUNCTION
    UNIMPLEMENTED;
}

/* Implement this one if you do use a shader */
void Object::drawElements(const Shader &shader) const {
    // !DELETE THIS WHEN YOU FINISH THIS FUNCTION
    UNIMPLEMENTED;
}