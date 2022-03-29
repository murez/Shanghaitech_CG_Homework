#ifndef _BEZIER_H_
#define _BEZIER_H_

#include "defines.h"
#include <object.h>
#include <tuple>
#include <vector>

const float v_eps = 1e-2;


class BezierCurve {
public:
    std::vector<vec3> control_points_;

    BezierCurve(int m);

    BezierCurve(std::vector<vec3> &control_points);

    void setControlPoint(int i, vec3 point);

    Vertex evaluate(std::vector<vec3> &control_points, float t);

    vec3 at(std::vector<vec3> control_points, float t, int derivative_order);

    Vertex evaluate(float t);

    double computeLength(std::vector<vec3> control_points);

    Object generateObject();
};

class BezierSurface {
public:
    std::vector<std::vector<vec3>> control_points_m_;
    std::vector<std::vector<vec3>> control_points_n_;

    BezierSurface(int m, int n);

    void setControlPoint(int i, int j, vec3 point);

    static Vertex evaluate(std::vector<std::vector<vec3>> &control_points, float u, float v);

    static VertexCurvature evaluateCurvature(std::vector<std::vector<vec3>> &control_points, float u, float v);

    double curveLength(std::vector<std::vector<vec3>> &control_points, float v);

    Object generateObject();

    std::pair<std::vector<Vertex>, std::vector<Point2D>>
    adaptiveSub(std::vector<std::vector<vec3>> &control_points_mm,
                std::vector<std::vector<vec3>> &control_points_nn,
                float u, float v_low, float v_up, vec3 point_low, vec3 point_up);

    Object generateObjectAdaptive();
};

BezierSurface readBezierSurfaceFromFile(const std::string &path);

std::vector<BezierSurface> readBPT(const std::string &path);

Object generateObject(std::vector<BezierSurface> bs_vector);

Object generateObjectAdaptive(std::vector<BezierSurface> bs_vector);

class SplineCurve {
public:
    unsigned int degree;
    std::vector<vec3> control_points_;
    std::vector<float> knot_vector_;


    SplineCurve(int m);

    SplineCurve(std::vector<vec3> &control_points);

    void setControlPoint(int i, vec3 point);

    int findSpan(float t);

    std::vector<float> basisFunctions(int i, float t);

    vec3 evaluatePosition(std::vector<vec3> &control_points, float t);

//    vec3 at(std::vector<vec3> control_points, float t, int derivative_order);

    Vertex evaluate(float t);

    Object generateObject();
};

class SplineSurface {
public:
    std::vector<std::vector<vec3>> control_points_m_;
    std::vector<std::vector<vec3>> control_points_n_;
    std::vector<float> knot_vector_m_;
    std::vector<float> knot_vector_n_;
    int degree_m_;
    int degree_n_;
    int m_;
    int n_;


    SplineSurface(int d_m, int d_n, int m, int n);

    void setControlPoint(int i, int j, vec3 point);

    static Vertex evaluate(std::vector<std::vector<vec3>> &control_points,
                           std::vector<float> knot_vector_u,
                           std::vector<float> knot_vector_v,
                           int degree_u, int degree_v,
                           float u, float v);

    Object generateObject();

};

SplineSurface readSplineSurfaceFromFile(const std::string &path);

#endif