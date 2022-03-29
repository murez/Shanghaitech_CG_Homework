#include <accel.h>
#include <ray.h>
#include <geometry.h>
#include <limits>

#define DELTA 0.0001f
#define EPSILON 0.00001f

/**
 * construct kd tree with given triangles
 * @param triangles a array of triangles
 */
KdTreeNode::KdTreeNode(std::vector<std::shared_ptr<Triangle>> triangles,
                       AABB box, int depth) {
    // the space of the current node
    this->box = box;
    // stop dividing when the number of triangles is small enough
    // or stop when depth is too large
    // 8 and 40 are just examples, it may not be good enough

    // || depth > maxDepth
    if (triangles.size() < 5 || depth > maxDepth) {
        // DONE: store triangles into the leaf node
        this->triangles = triangles;
        this->box = box;
        this->leftChild = nullptr;
        this->rightChild = nullptr;
        return;
    }
    // if root KdTreeNode set the space by iter all the triangles.
    /*
    if (box.lb == vec3::Zero() && box.ub == vec3::Zero()) {
        for (auto triangle: triangles) {
            auto v0 = triangle->getVertex(0);
            auto v1 = triangle->getVertex(1);
            auto v2 = triangle->getVertex(2);
            auto triangle_aabb = AABB(v0, v1, v2);
            box = AABB(box, triangle_aabb);
        }
    }
    */
    // DONE: divide the space into two parts according to one specific dimension
    AABB leftSpace(box.ub, box.lb);
    AABB rightSpace(box.ub, box.lb);

    // DONE: put the corresponding overlapping triangles
    std::vector<std::shared_ptr<Triangle>> leftTriangles;
    std::vector<std::shared_ptr<Triangle>> rightTriangles;
    // get the index of the max length of the box
    Eigen::Vector3f::Index index;
    index = (box.ub - box.lb).maxCoeff();
    // get threshold of the mid of the max length
    // auto threshold = (box.ub[index] + box.lb[index]) / 2;
    // put triangle into child nodes

    float threshold = 0.0f;

    float leftest_point = std::numeric_limits<float>::infinity();
    std::shared_ptr<Triangle> leftest_triangle;
    float rightest_point = -std::numeric_limits<float>::infinity();
    std::shared_ptr<Triangle> rightest_triangle;

    std::vector<float> triangle_centers;
    for (auto triangle: triangles) {
        auto v0 = triangle->getVertex(0);
        auto v1 = triangle->getVertex(1);
        auto v2 = triangle->getVertex(2);
        auto triangle_center = (v0[index] + v1[index] + v2[index]) * 0.333;
        triangle_centers.push_back(triangle_center);
    }

    sort(triangle_centers.begin(), triangle_centers.end());
    threshold = triangle_centers[triangle_centers.size() / 2];

    for (auto triangle: triangles) {
        // get triangle vertices
        auto v0 = triangle->getVertex(0);
        auto v1 = triangle->getVertex(1);
        auto v2 = triangle->getVertex(2);

        auto triangle_center = (v0[index] + v1[index] + v2[index]) * 0.333;

        auto triangle_aabb = AABB(v0, v1, v2);

        if (triangle_center < threshold) {
            leftTriangles.push_back(triangle);
            leftSpace = AABB(leftSpace, triangle_aabb);
        } else {
            rightTriangles.push_back(triangle);
            rightSpace = AABB(rightSpace, triangle_aabb);
        }
    }

    // recursively build left and right
    leftChild = new KdTreeNode(leftTriangles, leftSpace, depth + 1);
    rightChild = new KdTreeNode(rightTriangles, rightSpace, depth + 1);
}

bool KdTreeNode::intersect(Interaction &interaction, const Ray &ray) const {
    // DONE: first check whether ray hit the bounding box
    float tin, tout;
    if (!this->box.rayIntersection(ray, tin, tout))
        return false;

    if (isLeaf()) {
        // all triangles are stored in leaf nodes
        // DONE: do intersect with triangles
        bool hit = false;
        for (auto triangle: this->triangles) {
            Interaction cur_interaction;
            if (triangle->intersect(cur_interaction, ray)) {
                if (interaction.entryDist == -1 || cur_interaction.entryDist < interaction.entryDist)
                    interaction = cur_interaction;
                hit = true;
            }
        }
        return hit;
    }
    // DONE: recursively test intersection with left and right
    auto left_interaction = interaction;
    auto right_interaction = interaction;
    bool left_intersect = this->leftChild->intersect(left_interaction, ray);
    bool right_intersect = this->rightChild->intersect(right_interaction, ray);
    if (left_intersect && right_intersect) {
        if (left_interaction.entryDist < right_interaction.entryDist)
            interaction = left_interaction;
        else
            interaction = right_interaction;
        return true;
    }
    if (left_intersect) {
        interaction = left_interaction;
        return true;
    }
    if (right_intersect) {
        interaction = right_interaction;
        return true;
    }
    // !!!DELETE THIS WHEN FINISHED
    // UNIMPLEMENTED;
    return false;
}

KdTreeNode::~KdTreeNode() {
    if (leftChild) {
        delete leftChild;
        leftChild = nullptr;
    }
    if (rightChild) {
        delete rightChild;
        rightChild = nullptr;
    }
}

KdTreeAccel::KdTreeAccel(
        const std::vector<std::shared_ptr<Triangle>> &triangles) {
    AABB box;
    for (auto &tri: triangles)
        box = AABB(box,
                   AABB(tri->getVertex(0), tri->getVertex(1), tri->getVertex(2)));
    root = std::move(std::make_unique<KdTreeNode>(triangles, box, 0));
}

bool KdTreeAccel::intersect(Interaction &interaction, const Ray &ray) const {
    if (root) return root->intersect(interaction, ray);
    return false;
}

AABB::AABB(Float lbX, Float lbY, Float lbZ, Float ubX, Float ubY, Float ubZ) {
    lb = vec3(lbX, lbY, lbZ);
    ub = vec3(ubX, ubY, ubZ);
}

AABB::AABB(const vec3 &lb, const vec3 &ub) : lb(lb), ub(ub) {}

AABB::AABB(const vec3 &v1, const vec3 &v2, const vec3 &v3) {
    lb = v1.cwiseMin(v2).cwiseMin(v3);
    ub = v1.cwiseMax(v2).cwiseMax(v3);
}

AABB::AABB(const AABB &a, const AABB &b) {
    lb = vec3(a.lb.cwiseMin(b.lb));
    ub = vec3(a.ub.cwiseMax(b.ub));
}

vec3 AABB::getCenter() const { return (lb + ub) / 2; }

Float AABB::getDist(int c) const { return ub[c] - lb[c]; }

Float AABB::getVolume() const { return getDist(2) * getDist(1) * getDist(0); }

bool AABB::isOverlap(const AABB &a) const {
    return ((a.lb[0] >= this->lb[0] && a.lb[0] <= this->ub[0]) ||
            (this->lb[0] >= a.lb[0] && this->lb[0] <= a.ub[0])) &&
           ((a.lb[1] >= this->lb[1] && a.lb[1] <= this->ub[1]) ||
            (this->lb[1] >= a.lb[1] && this->lb[1] <= a.ub[1])) &&
           ((a.lb[2] >= this->lb[2] && a.lb[2] <= this->ub[2]) ||
            (this->lb[2] >= a.lb[2] && this->lb[2] <= a.ub[2]));
}

Float AABB::diagonalLength() const { return (ub - lb).norm(); }

bool AABB::rayIntersection(const Ray &ray, Float &tIn, Float &tOut) const {
    Float dirX = (ray.direction[0] == 0) ? INF : Float(1) / ray.direction[0];
    Float dirY = (ray.direction[1] == 0) ? INF : Float(1) / ray.direction[1];
    Float dirZ = (ray.direction[2] == 0) ? INF : Float(1) / ray.direction[2];

    Float tx1 = (lb[0] - ray.origin[0]) * dirX;
    Float tx2 = (ub[0] - ray.origin[0]) * dirX;
    Float ty1 = (lb[1] - ray.origin[1]) * dirY;
    Float ty2 = (ub[1] - ray.origin[1]) * dirY;
    Float tz1 = (lb[2] - ray.origin[2]) * dirZ;
    Float tz2 = (ub[2] - ray.origin[2]) * dirZ;

    tIn = std::max(std::max(std::min(tx1, tx2), std::min(ty1, ty2)),
                   std::min(tz1, tz2));
    tOut = std::min(std::min(std::max(tx1, tx2), std::max(ty1, ty2)),
                    std::max(tz1, tz2));

    /* When tOut < 0 and the ray is intersecting with AABB, the whole AABB is
     * behind us */
    if (tOut < 0) {
        return false;
    }

    return tOut >= tIn;
}

UniformGrid::UniformGrid(std::vector<std::shared_ptr<Triangle>> triangles, float lambda) {
    float inf = std::numeric_limits<float>::infinity();

    Eigen::Vector3f lb(inf, inf, inf);
    Eigen::Vector3f ub(-inf, -inf, -inf);


    num_triangles = triangles.size();

    for (auto &tri: triangles) {
        lb = lb.cwiseMin(tri->getVertex(0));
        lb = lb.cwiseMin(tri->getVertex(1));
        lb = lb.cwiseMin(tri->getVertex(2));

        ub = ub.cwiseMax(tri->getVertex(0));
        ub = ub.cwiseMax(tri->getVertex(1));
        ub = ub.cwiseMax(tri->getVertex(2));
    }

    bounding_box = AABB(lb, ub);
    Eigen::Vector3f grid_size = ub - lb;
    float V = grid_size.x() * grid_size.y() * grid_size.z();
    float c = std::pow(lambda * num_triangles / V, 1.0 / 3.0);

    x_size = ceil(grid_size.x() * c);
    y_size = ceil(grid_size.y() * c);
    z_size = ceil(grid_size.z() * c);

    dx = 1.0f / c;
    dy = 1.0f / c;
    dz = 1.0f / c;

    grid_triangles.resize(x_size, std::vector<std::vector<
            std::vector<std::shared_ptr<Triangle>>>>(y_size,std::vector<
            std::vector<std::shared_ptr<Triangle>>>(z_size)));
//    for (auto x_grid: grid_triangles) {
//        x_grid.resize(y_size);
//        for (auto y_grid: x_grid) {
//            y_grid.resize(z_size);
//        }
//    }



    for (int i = 0; i < triangles.size(); i++) {
        auto triangle = triangles[i];

        vec3f v0 = triangle->getVertex(0);
        vec3f v1 = triangle->getVertex(1);
        vec3f v2 = triangle->getVertex(2);
        vec3f l = v0.cwiseMin(v1).cwiseMin(v2);
        vec3f u = v0.cwiseMax(v1).cwiseMax(v2);


        int xmin = (l.x() - lb.x()) / dx;
        int xmax = (u.x() - lb.x()) / dx;
        int ymin = (l.y() - lb.y()) / dy;
        int ymax = (u.y() - lb.y()) / dy;
        int zmin = (l.z() - lb.z()) / dz;
        int zmax = (u.z() - lb.z()) / dz;

        for (int x = xmin; x <= xmax; x++) {
            for (int y = ymin; y <= ymax; y++) {
                for (int z = zmin; z <= zmax; z++) {
                    grid_triangles[x][y][z].push_back(triangle);
                }
            }
        }
    }

}

bool UniformGrid::intersect(Interaction &interaction, const Ray &ray) const {
    Interaction final_interaction;
    float t_in;
    float t_out;
    float inf = std::numeric_limits<float>::infinity();
    if (bounding_box.rayIntersection(ray, t_in, t_out)) {
        vec3 dir = ray.direction;
        vec3 p = ray.getPoint(t_in) - bounding_box.lb;

        Eigen::Vector3i idx;
        idx[0] = p.x() / dx;
        idx[1] = p.y() / dy;
        idx[2] = p.z() / dz;

        float dir_frac_x = (dir.x() <= EPSILON) ? inf : 1.0f / dir.x();
        float dir_frac_y = (dir.y() <= EPSILON) ? inf : 1.0f / dir.y();
        float dir_frac_z = (dir.z() <= EPSILON) ? inf : 1.0f / dir.z();

        Eigen::Vector3i step;
        step[0] = (dir.x() > 0) ? 1 : -1;
        step[1] = (dir.y() > 0) ? 1 : -1;
        step[2] = (dir.z() > 0) ? 1 : -1;
        step[0] = (dir_frac_x == inf) ? 0 : step[0];
        step[1] = (dir_frac_y == inf) ? 0 : step[1];
        step[2] = (dir_frac_z == inf) ? 0 : step[2];

        vec3 t_walk;
        t_walk[0] = (step.x() < 0 ? (idx.x() * dx - p.x()) : ((idx.x() + step.x()) * dx - p.x())) * dir_frac_x;
        t_walk[1] = (step.y() < 0 ? (idx.y() * dy - p.y()) : ((idx.y() + step.y()) * dy - p.y())) * dir_frac_y;
        t_walk[2] = (step.z() < 0 ? (idx.z() * dz - p.z()) : ((idx.z() + step.z()) * dz - p.z())) * dir_frac_z;

        vec3 t_delta;
        t_delta[0] = dx * abs(dir_frac_x);
        t_delta[1] = dy * abs(dir_frac_y);
        t_delta[2] = dz * abs(dir_frac_z);
        while (true) {
            if (checkIndex(idx)){
                for (auto triangle: grid_triangles[idx[0]][idx[1]][idx[2]]) {
                    Interaction cur_it;
                    if (triangle->intersect(cur_it, ray)) {
                        if (final_interaction.entryDist == -1 || cur_it.entryDist < final_interaction.entryDist) {
                            final_interaction = cur_it;
                        }
                    }
                }
                if (final_interaction.entryDist!= -1) {
                    interaction = final_interaction;
                    return true;
                }
                else{
                    Eigen::Vector3f::Index d;
                    t_walk.maxCoeff(&d);
                    idx(d) = idx(d) + step(d);
                    t_walk(d) = t_walk(d) + t_delta(d);
                }
            }
            else{
                return false;
            }
        }
    }


    return false;
}

AABB UniformGrid::getBoundingBox() {
    return bounding_box;
}

vec3 UniformGrid::getVoxelSize() {
    return vec3(dx, dy, dz);
}

bool UniformGrid::checkIndex(Eigen::Vector3i index) const {
    if (index.x() < 0 || index.y() < 0 || index.z() < 0)
        return false;
    if (index.x() >= x_size || index.y() >= y_size || index.z() >= z_size)
        return false;
    return true;
}
