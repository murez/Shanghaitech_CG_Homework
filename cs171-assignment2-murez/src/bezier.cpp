#include <bezier.h>
#include <utils.h>
#include <utility>
#include <vector>
#include <fstream>
#include <cmath>
#include <bpa.h>
#include "delaunator.cpp"

const unsigned int MAXSIZE = 10000;
GLuint MAXPOINTS = 16;
const float epsilon = 1e-2;
int f[MAXSIZE][MAXSIZE] = {0};

int C(int n, int i) {
    if (f[n][i] != 0) {
        return f[n][i];
    }
    if (i <= 0 || i == n) {
        f[n][i] = 1;
    } else {
        f[n][i] = C(n - 1, i - 1) + C(n - 1, i);
    }
    return f[n][i];
}

float binpow(float a, int b) {
    float res = 1;
    while (b > 0) {
        if (b & 1) res = res * a;
        a = a * a;
        b >>= 1;
    }
    return res;
}


template<typename F>
static double
simpson_3_8(F &&derivative_func, const double &L, const double &R) {
    double mid_L = (2 * L + R) / 3.0, mid_R = (L + 2 * R) / 3.0;
    return (derivative_func(L) +
            3.0 * derivative_func(mid_L) +
            3.0 * derivative_func(mid_R) +
            derivative_func(R)) * (R - L) / 8.0;
}

template<typename F>
static double
adaptive_simpson_3_8(F &&derivative_func,
                     const double &L, const double &R, const double &eps = 0.0001) {
    const double mid = (L + R) / 2.0;
    double ST = simpson_3_8(derivative_func, L, R),
            SL = simpson_3_8(derivative_func, L, mid),
            SR = simpson_3_8(derivative_func, mid, R);
    double ans = SL + SR - ST;
    if (fabs(ans) <= 15.0 * eps) return SL + SR + ans / 15.0;
    return adaptive_simpson_3_8(derivative_func, L, mid, eps / 2.0) +
           adaptive_simpson_3_8(derivative_func, mid, R, eps / 2.0);
}

BezierCurve::BezierCurve(int m) { control_points_.resize(m); }

BezierCurve::BezierCurve(std::vector<vec3> &control_points) {
    control_points_ = control_points;
}

void BezierCurve::setControlPoint(int i, vec3 point) {
    control_points_[i] = point;
}

/**
 * TODO: evaluate the point at t with the given control_points
 */
Vertex BezierCurve::evaluate(std::vector<vec3> &control_points, float t) {

    if (control_points.size() == 2) {
        Vertex x{};
        if (t == 0.0) {
            x.position = control_points[0];
        } else if (t == 1.0) {
            x.position = control_points[1];
        } else {
            x.position = (1 - t) * control_points[0] + t * control_points[1];
        }
        x.normal = control_points[1] - control_points[0];
        return x;
    } else {
        std::vector<vec3> next_control_points;
        vec3 first_vertex = control_points[0];// first vertex
        for (int i = 1; i < control_points.size(); i++) {
            vec3 second_vertex = control_points[i];
            next_control_points.push_back(vec3((1 - t) * first_vertex + t * second_vertex));
            first_vertex = second_vertex;
        }
        return this->evaluate(next_control_points, t);
    }
}

Vertex BezierCurve::evaluate(float t) { return evaluate(control_points_, t); }

/**
 * TODO: generate an Object of the current Bezier curve
 */
Object BezierCurve::generateObject() {
    return Object();
}

vec3 BezierCurve::at(std::vector<vec3> control_points, float t, int derivative_order) {
    auto temp = std::move(control_points);
    int n = temp.size();
    int prefix = 1;
    for (int i = 0; i < derivative_order; ++i)
        prefix *= n - i;
    for (int k = 0; k < derivative_order; ++k) {
        int I_range = n - derivative_order;
        for (int i = 0; i <= I_range; ++i) {
            temp[i] = temp[i + 1] - temp[i];
        }
    }
    int I_range = n - derivative_order;
    for (int i = 0; i < I_range; ++i) {
        int J_range = I_range - i;
        for (int j = 0; j < J_range; ++j)
            temp[j] = float(1.0 - t) * temp[j] + t * temp[j + 1];
    }
    return float(prefix) * temp[0];
}

double BezierCurve::computeLength(std::vector<vec3> control_points) {
    auto df = [&](float t) -> float {
        return glm::length(at(control_points, t, 1));
    };
    auto length = adaptive_simpson_3_8(df, 0, 1);
    return length;
}

BezierSurface::BezierSurface(int m, int n) {
    control_points_m_.resize(m);
    for (auto &sub_vec : control_points_m_) {
        sub_vec.resize(n);
    }
    control_points_n_.resize(n);
    for (auto &sub_vec : control_points_n_) {
        sub_vec.resize(m);
    }
}

/**
 * @param[in] i: index (i < m)
 * @param[in] j: index (j < n)
 * @param[in] point: the control point with index i, j
 */
void BezierSurface::setControlPoint(int i, int j, vec3 point) {
    control_points_m_[i][j] = point;
    control_points_n_[j][i] = point;
}

/**
 * TODO: evaluate the point at (u, v) with the given control points
 */
Vertex BezierSurface::evaluate(std::vector<std::vector<vec3>> &control_points,
                               float u, float v) {
    std::vector<vec3> control_point_q;
    BezierCurve p(0);
    // calculate rows
    for (const auto &control_point_list: control_points) {
        p.control_points_ = control_point_list;
        control_point_q.push_back(p.evaluate(v).position);
    }
    // calculate vertex on column direction
    p.control_points_ = control_point_q;
    return p.evaluate(u);
}

VertexCurvature BezierSurface::evaluateCurvature(std::vector<std::vector<vec3>> &control_points, float u, float v) {
    std::vector<vec3> control_point_q;
    BezierCurve p(0);
    for (const auto &control_point_list: control_points) {
        p.control_points_ = control_point_list;
        control_point_q.push_back(p.evaluate(v).position);
    }
    p.control_points_ = control_point_q;
    auto vertex = p.evaluate(u);
    auto vertex_d1 = p.at(control_point_q, u, 1);
    auto vertex_d2 = p.at(control_point_q, u, 2);

    float curvature = float(glm::length(glm::cross(vertex_d1, vertex_d2)) / pow(glm::length(vertex_d1), 3));

    VertexCurvature vertex_curvature{vertex.position, vertex.normal, curvature};
    return vertex_curvature;
}

double BezierSurface::curveLength(std::vector<std::vector<vec3>> &control_points, float v) {
    std::vector<vec3> control_point_q;
    BezierCurve p(0);
    for (const auto &control_point_list: control_points) {
        p.control_points_ = control_point_list;
        control_point_q.push_back(p.evaluate(v).position);
    }
    return p.computeLength(control_point_q);
}

/**
 * TODO: generate an Object of the current Bezier surface
 */
Object BezierSurface::generateObject() {
    MAXPOINTS = 10;
    Object new_object;
    float step = 1.0 / MAXPOINTS;
    for (float u = 0.0; u <= 1.0; u += step) {
        for (float v = 0.0; v <= 1.0; v += step) {
            auto vertex_1 = evaluate(control_points_m_, u, v);// vertex on direction y
            auto vertex_2 = evaluate(control_points_n_, v, u);// vertex on direction x
            auto position = vertex_1.position;
            auto normal = glm::normalize(glm::cross(vertex_1.normal, vertex_2.normal));
            Vertex vertex = {position, normal};
            new_object.vertices.push_back(vertex);
        }
    }
//    auto mesh = reconstruct(new_object.vertices, 0.5);

    for (unsigned int x = 0; x < MAXPOINTS - 1; x++) {
        for (unsigned int y = 0; y < MAXPOINTS - 1; y++) {
            // up triangle
            new_object.indices.push_back(x * MAXPOINTS + y);
            new_object.indices.push_back(x * MAXPOINTS + y + 1);
            new_object.indices.push_back((x + 1) * MAXPOINTS + y);
            // down triangle
            new_object.indices.push_back((x + 1) * MAXPOINTS + y);
            new_object.indices.push_back((x + 1) * MAXPOINTS + y + 1);
            new_object.indices.push_back(x * MAXPOINTS + y + 1);
        }
    }
    return new_object;
}


Object BezierSurface::generateObjectAdaptive() {
    Object new_object;
    float step_std = 1.0 / MAXPOINTS;
    const unsigned int max_iter = 10;
    std::vector<Point2D> mesh_2d;

    for (float u = 0.0; u <= 1.0; u += step_std) {
        std::vector<Vertex> vertices_tmp;

        auto vertex_0_1 = evaluate(control_points_m_, u, 0.0);// vertex on direction y
        auto vertex_0_2 = evaluate(control_points_n_, 0.0, u);// vertex on direction x
        auto position_0 = vertex_0_1.position;
        auto normal_0 = glm::normalize(glm::cross(vertex_0_1.normal, vertex_0_2.normal));
        Vertex vertex_0 = {position_0, normal_0};

        auto vertex_1_1 = evaluate(control_points_m_, u, 1.0);// vertex on direction y
        auto vertex_1_2 = evaluate(control_points_n_, 1.0, u);// vertex on direction x
        auto position_1 = vertex_1_1.position;
        auto normal_1 = glm::normalize(glm::cross(vertex_1_1.normal, vertex_1_2.normal));
        Vertex vertex_1 = {position_1, normal_1};

        auto result = adaptiveSub(control_points_m_, control_points_n_,
                                  u, 0, 1, position_0, position_1);

        new_object.vertices.push_back(vertex_0);
        new_object.vertices.push_back(vertex_1);

        mesh_2d.push_back(Point2D{u, 0.0});
        mesh_2d.push_back(Point2D{u, 1.0});

        new_object.vertices.insert(new_object.vertices.end(), result.first.begin(), result.first.end());
        mesh_2d.insert(mesh_2d.end(), result.second.begin(), result.second.end());
    }
    std::vector<double> coords;
    for (auto p: mesh_2d) {
        coords.push_back(p.x);
        coords.push_back(p.y);
    }
    delaunator::Delaunator d(coords);

    std::cout << d.triangles.size() << std::endl;
    new_object.indices = d.triangles;
    return new_object;
}

float DistancePoint2Line(vec3 A, vec3 B, vec3 P) {
    return glm::dot((B - A), (P - A)) / glm::length(B - A);
}

std::pair<std::vector<Vertex>, std::vector<Point2D>>
BezierSurface::adaptiveSub(std::vector<std::vector<vec3>> &control_points_mm,
                           std::vector<std::vector<vec3>> &control_points_nn,
                           float u,
                           float v_low, float v_up,
                           vec3 point_low, vec3 point_up) {
    std::vector<Vertex> vertex_results;
    std::vector<Point2D> mesh_results;
    auto v_half = (v_low + v_up) / 2;

    auto vertex_half = evaluateCurvature(control_points_mm, u, v_half);
    auto vertex_half_2 = evaluateCurvature(control_points_nn, v_half, u);

    if ((vertex_half.curvature > 1e-4 || vertex_half_2.curvature > 1e-4) && glm::length(point_low - point_up) >= 1e-2 ||
        glm::length(point_low - point_up) >= 5e-2 && (
                (v_up - v_low) >= v_eps ||
                DistancePoint2Line(point_low, point_up, vertex_half.position) > (glm::length(point_low - point_up)))) {
        auto normal = glm::normalize(glm::cross(vertex_half.normal, vertex_half_2.normal));
        Vertex vertex = {vertex_half.position, normal};
        vertex_results.push_back(vertex);
        mesh_results.push_back(Point2D{u, v_half});
        auto lower = adaptiveSub(control_points_mm, control_points_nn,
                                 u, v_low, v_half, point_low, vertex_half.position);
        auto upper = adaptiveSub(control_points_mm, control_points_nn,
                                 u, v_half, v_up, vertex_half.position, point_up);
        vertex_results.insert(vertex_results.end(), lower.first.begin(), lower.first.end());
        vertex_results.insert(vertex_results.end(), upper.first.begin(), upper.first.end());
        mesh_results.insert(mesh_results.end(), lower.second.begin(), lower.second.end());
        mesh_results.insert(mesh_results.end(), upper.second.begin(), upper.second.end());
    }
    return std::make_pair(vertex_results, mesh_results);
}


BezierSurface readBezierSurfaceFromFile(const std::string &path) {
    std::ifstream infile(path);
    int m, n;
    infile >> m >> n;
    BezierSurface bs(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++) {
            vec3 temp_vertex;
            infile >> temp_vertex[0] >> temp_vertex[1] >> temp_vertex[2];
            bs.setControlPoint(i, j, temp_vertex);
        }
    infile.close();
    return bs;
}

std::vector<BezierSurface> readBPT(const std::string &path) {
    std::ifstream infile(path);
    int t, m, n;
    std::vector<BezierSurface> BS_vector;
    infile >> t;
    for (int _ = 0; _ < t; _++) {
        infile >> m >> n;
        BezierSurface bs(m, n);
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++) {
                vec3 temp_vertex;
                infile >> temp_vertex[0] >> temp_vertex[1] >> temp_vertex[2];
                bs.setControlPoint(i, j, temp_vertex);
            }
        BS_vector.push_back(bs);
    }
    infile.close();
    return BS_vector;
}


Object generateObject(std::vector<BezierSurface> bs_vector) {
    Object new_object;
    float step = 1.0 / MAXPOINTS;
    for (auto bs: bs_vector) {
        int base_index = new_object.vertices.size();
        int mx = 0, my = 0;
        for (float u = 0.0; u <= 1.0; u += step) {
            mx += 1;
            for (float v = 0.0; v <= 1.0; v += step) {
                my += 1;
                auto vertex_1 = bs.evaluate(bs.control_points_m_, u, v);// vertex on direction x
                auto vertex_2 = bs.evaluate(bs.control_points_n_, v, u);// vertex on direction y
                auto position = vec3(float(0.5) * vertex_1.position + float(0.5) * vertex_2.position);
                auto normal = glm::normalize(glm::cross(vertex_1.normal, vertex_2.normal));
                Vertex vertex = {position, normal};
                new_object.vertices.push_back(vertex);
            }
        }
        my = my / mx;
        for (unsigned int x = 0; x < mx - 1; x++) {
            for (unsigned int y = 0; y < my - 1; y++) {
                // up triangle
                new_object.indices.push_back(base_index + x * my + y);
                new_object.indices.push_back(base_index + x * my + y + 1);
                new_object.indices.push_back(base_index + (x + 1) * my + y);
                // down triangle
                new_object.indices.push_back(base_index + (x + 1) * my + y);
                new_object.indices.push_back(base_index + (x + 1) * my + y + 1);
                new_object.indices.push_back(base_index + x * my + y + 1);
            }
        }
    }
    return new_object;
}

Object generateObjectAdaptive(std::vector<BezierSurface> bs_vector){
    Object new_object;
    for (auto bs: bs_vector) {
        auto obj_adaptive = bs.generateObjectAdaptive();
        auto base_vertex_num = new_object.vertices.size();
        new_object.vertices.insert(new_object.vertices.end(),
                                   obj_adaptive.vertices.begin(),
                                   obj_adaptive.vertices.end());
        for (auto i = 0;i<obj_adaptive.indices.size();i++){
            obj_adaptive.indices[i] += base_vertex_num;
        }
        new_object.indices.insert(new_object.indices.end(),
                                   obj_adaptive.indices.begin(),
                                   obj_adaptive.indices.end());
    }
    return new_object;
}


SplineCurve::SplineCurve(int m) {
    degree = m;
}

SplineCurve::SplineCurve(std::vector<vec3> &control_points) {
    control_points_ = control_points;
}

void SplineCurve::setControlPoint(int i, vec3 point) {
    control_points_[i] = point;
}

int SplineCurve::findSpan(float t) {
    int mid;
    if (t == knot_vector_[control_points_.size()])
        return control_points_.size() - 1;
    else if (t == knot_vector_[0])
        return degree;
    for (mid = degree; mid < control_points_.size(); mid++) {
        if (t <= knot_vector_[mid + 1] && t > knot_vector_[mid])
            break;
    }
    return mid;
}


std::vector<float> SplineCurve::basisFunctions(int i, float t) {
    std::vector<float> coeff, left, right;
    coeff.resize(degree + 1);
    left.resize(degree + 1);
    right.resize(degree + 1);
    coeff[0] = 1.0;

    for (auto j = 1; j <= degree; ++j) {
        left[j] = t - knot_vector_[i + 1 - j];
        right[j] = knot_vector_[i + j] - t;
        float saved = 0.0;
        for (auto r = 0; r < j; r++) {
            float tmp = coeff[r] / (right[r + 1] + left[j - r]);
            coeff[r] = saved + tmp * right[r + 1];
            saved = tmp * left[j - r];
        }
        coeff[j] = saved;
    }
    return coeff;
}

vec3 SplineCurve::evaluatePosition(std::vector<vec3> &control_points, float t) {
    auto span = findSpan(t);
    auto coeff = basisFunctions(span, t);
    vec3 result = vec3(0.0, 0.0, 0.0);
    for (auto i = 0; i <= degree; i++) {
        result += control_points[span - degree + i] * coeff[i];
    }
    return result;
}

Vertex SplineCurve::evaluate(float t) {
    auto position = evaluatePosition(control_points_, t);
    vec3 position_next, tangent;
    if (t + 1e-3 <= knot_vector_.back()) {
        position_next = evaluatePosition(control_points_, t + 1e-3);
        tangent = glm::normalize(position_next - position);
    } else {
        position_next = evaluatePosition(control_points_, t - 1e-3);
        tangent = glm::normalize(position - position_next);
    }
    return Vertex{position, tangent};
}

Object SplineCurve::generateObject() {
    return Object();
}


SplineSurface::SplineSurface(int d_m, int d_n, int m, int n) {
    degree_m_ = d_m;
    degree_n_ = d_n;
    m_ = m;
    n_ = n;
    control_points_m_.resize(m);
    for (auto &sub_vec : control_points_m_) {
        sub_vec.resize(n);
    }
    control_points_n_.resize(n);
    for (auto &sub_vec : control_points_n_) {
        sub_vec.resize(m);
    }
}

void SplineSurface::setControlPoint(int i, int j, vec3 point) {
    control_points_m_[i][j] = point;
    control_points_n_[j][i] = point;
}

Vertex SplineSurface::evaluate(std::vector<std::vector<vec3>> &control_points,
                               std::vector<float> knot_vector_u,
                               std::vector<float> knot_vector_v,
                               int degree_u, int degree_v,
                               float u, float v) {
    std::vector<vec3> control_point_q;
    SplineCurve p(degree_v);
    p.degree = degree_v;
    for (const auto &control_point_list : control_points) {
        p.control_points_ = control_point_list;
        p.knot_vector_ = knot_vector_v;
        auto vertex = p.evaluate(v);
        control_point_q.push_back(vertex.position);
    }
    p.degree = degree_u;
    p.control_points_ = control_point_q;
    p.knot_vector_ = knot_vector_u;
    return p.evaluate(u);
}

float triangle(vec3 v1, vec3 v2, vec3 v3) {
    glm::mat3x3 tri(v1, v2, v3);
    return fabs(glm::determinant(tri));
}

Object SplineSurface::generateObject() {
    Object new_object;

    auto u_max = knot_vector_m_.back();
    auto v_max = knot_vector_n_.back();


    float u_step = u_max / MAXPOINTS;
    float v_step = v_max / MAXPOINTS;

    int num_x = 0;
    int num_y = 0;
    for (float u = 0.0; u <= u_max + u_step / 2; u += u_step) {
        num_x += 1;
        for (float v = 0.0; v <= v_max + v_step / 2; v += v_step) {
            num_y += 1;
            auto vertex_1 = evaluate(control_points_m_,
                                     knot_vector_m_,
                                     knot_vector_n_,
                                     degree_m_, degree_n_,
                                     u, v);
            auto vertex_2 = evaluate(control_points_n_,
                                     knot_vector_n_,
                                     knot_vector_m_,
                                     degree_n_, degree_m_,
                                     v, u);
            auto position = vec3(float(0.5) * vertex_1.position + float(0.5) * vertex_2.position);

            auto normal = glm::normalize(glm::cross(vertex_1.normal, vertex_2.normal));
            Vertex vertex = {position, normal};
            new_object.vertices.push_back(vertex);
        }
    }
    num_y = num_y / num_x;
    for (auto x = 0; x <= num_x - 1; x++) {
        for (auto y = 0; y < num_y - 1; y++) {


            auto area = triangle(new_object.vertices[x * num_y + y].position,
                                 new_object.vertices[x * num_y + y + 1].position,
                                 new_object.vertices[(x + 1) * num_y + y].position);
            if (area >= 0.0001 && area <= 1) {
                new_object.indices.push_back(x * num_y + y);
                new_object.indices.push_back(x * num_y + y + 1);
                new_object.indices.push_back((x + 1) * num_y + y);
            }
            area = triangle(new_object.vertices[(x + 1) * num_y + y].position,
                            new_object.vertices[(x + 1) * num_y + y + 1].position,
                            new_object.vertices[x * num_y + y + 1].position);
            if (area >= 0.0001 && area <= 1) {
                new_object.indices.push_back((x + 1) * num_y + y);
                new_object.indices.push_back((x + 1) * num_y + y + 1);
                new_object.indices.push_back(x * num_y + y + 1);
            }
        }
    }
    return new_object;
}

SplineSurface readSplineSurfaceFromFile(const std::string &path) {
    std::ifstream infile(path);
    int degree_m, degree_n;
    int m, n;
    infile >> degree_m >> degree_n;
    infile >> m >> n;
    SplineSurface ss(degree_m, degree_n, m, n);

    int knot_m_length = degree_m + m + 1;
    int knot_n_length = degree_n + n + 1;

    for (auto i = 0; i < knot_m_length; i++) {
        float knot;
        infile >> knot;
        ss.knot_vector_m_.push_back(knot);
    }

    for (auto i = 0; i < knot_n_length; i++) {
        float knot;
        infile >> knot;
        ss.knot_vector_n_.push_back(knot);
    }

    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++) {
            vec3 temp_vertex;
            infile >> temp_vertex[0] >> temp_vertex[1] >> temp_vertex[2];
            ss.setControlPoint(i, j, temp_vertex);
        }
    infile.close();
    return ss;
}