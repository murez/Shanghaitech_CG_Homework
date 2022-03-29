#ifndef _object_H_
#define _object_H_

#include "defines.h"
#include <shader.h>
#include <vector>

struct Vertex {
    vec3 position;
    vec3 normal;
};

struct VertexCurvature {
    vec3 position;
    vec3 normal;
    double curvature;
};


struct Point2D {
    float x=0;
    float y=0;
};

struct DrawMode {
    GLenum primitive_mode;
};

class Object {
private:
    GLuint VAO;
    GLuint VBO;
    GLuint EBO;

public:
    std::vector<Vertex> vertices;
    std::vector<GLuint> indices;

    DrawMode draw_mode;

    Object();

    ~Object();

    void init();

    void drawArrays() const;

    void drawArrays(const Shader &shader) const;

    void drawElements() const;

    void drawElements(const Shader &shader) const;
};

#endif