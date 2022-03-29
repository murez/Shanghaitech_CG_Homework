#include <geometry.h>
#include <ray.h>
#include <interaction.h>
const int EPSILON = 1e-4;
/**
 * ray-triangle intersect
 * @param[out] interaction output intersect infos
 * @param[in] ray the given ray
 * @return whether ray hit the triangle
 */
bool Triangle::intersect(Interaction &interaction, const Ray &ray) const {
  const vec3 &v0 = mesh->p[v[0]];
  const vec3 &v1 = mesh->p[v[1]];
  const vec3 &v2 = mesh->p[v[2]];
  // TODO: ray-triangle intersection

  vec3 edge1, edge2, h, s, q;
  float a, f, uu, vv;
  edge1 = v1 - v0;
  edge2 = v2 - v0;
  h = ray.direction.cross(edge2);
  a = edge1.dot(h);
  if (a > -EPSILON && a < EPSILON)
    return false;
  f = 1.0 / a;
  s = ray.origin - v0;
  uu = f * s.dot(h);
  if (uu < 0.0 || uu > 1.0)
    return false;
  q = s.cross(edge1);
  vv = f * ray.direction.dot(q);
  if (vv < 0.0 || uu + vv > 1.0)
    return false;
  float t = f * edge2.dot(q);
  if (t > EPSILON) {
    interaction.entryDist = t;
    interaction.normal = (1-uu-vv)*mesh->n[v[0]] + uu*mesh->n[v[1]] + vv*mesh->n[v[2]];// TODO: normal may be computed by triangle
    interaction.entryPoint = ray.getPoint(t);
    interaction.uv = vec2(uu, vv);

    if (material != nullptr) {
      interaction.lightingModel = material->evaluate(interaction);
    }
    interaction.type = Interaction::Type::GEOMETRY;

    return true;
  }
  return false;
  // interaction.entryDist = t;
  // update entryPoint and normal
  // !!!DELETE THIS WHEN FINISHED
  //  UNIMPLEMENTED;

  //  if (material != nullptr) {
  //    interaction.lightingModel = material->evaluate(interaction);
  //  }
  //  interaction.type = Interaction::Type::GEOMETRY;
  //
  //  return true;
}

/**
 * Geometry class
 */
void Geometry::setMaterial(std::shared_ptr<Material> newMat) {
  material = newMat;
}

Triangle::Triangle(std::shared_ptr<TriangleMesh> mesh, const int *v,
                   std::shared_ptr<Material> mat)
    : mesh(mesh), v(v) {
  setMaterial(mat);
}

TriangleMesh::TriangleMesh(const std::vector<int> &indices, int nVertices,
                           const std::vector<vec3> &p,
                           const std::vector<vec3> &n,
                           const std::vector<vec2> &uv)
    : nTriangles(static_cast<int>(indices.size() / 3)), nVertices(nVertices) {
  this->indices = indices;
  this->p.resize(nVertices);
  for (int i = 0; i < nVertices; ++i)
    this->p[i] = p[i];  // objectToWorld(pos[i]);
  if (!n.empty()) {
    this->n.resize(nVertices);
    for (int i = 0; i < nVertices; ++i)
      this->n[i] = n[i];  // objectToWorld(normal[i]);
  }
  if (!uv.empty()) {
    this->uv.assign(uv.begin(), uv.begin() + nVertices);
  }
}

std::vector<std::shared_ptr<Geometry>> makeTriangleMesh(
    const std::vector<int> &indices, int nVertices, const std::vector<vec3> &p,
    const std::vector<vec3> &n, const std::vector<vec2> &uv,
    const std::shared_ptr<Material> &mat) {
  std::vector<std::shared_ptr<Geometry>> ret;
  auto mesh = std::make_shared<TriangleMesh>(indices, nVertices, p, n, uv);
  ret.reserve(mesh->nTriangles);
  for (int i = 0; i < mesh->nTriangles; ++i) {
    ret.push_back(
        std::make_shared<Triangle>(mesh, mesh->indices.data() + 3 * i, mat));
  }
  return ret;
}

std::vector<std::shared_ptr<Geometry>> makeParallelogram(
    const vec3 &p0, const vec3 &s0, const vec3 &s1, const vec3 &normal,
    std::shared_ptr<Material> mat) {
  std::vector<int> index{0, 1, 2, 0, 2, 3};
  std::vector<vec3> position{p0, p0 + s0, p0 + s0 + s1, p0 + s1};
  std::vector<vec3> normals{normal, normal, normal, normal};
  std::vector<vec2> uv{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  return makeTriangleMesh(index, 4, position, normals, uv, mat);
}