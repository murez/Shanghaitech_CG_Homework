#include <light.h>
#include <utils.h>

/**
 * Light class
 */
Light::Light(const vec3 &position, const vec3 &radiance)
        : position(position), radiance(radiance) {}

vec3 Light::getPosition() const { return position; }

vec3 Light::getRadiance() const { return radiance; }

/**
 * AreaLight class
 */
AreaLight::AreaLight(const vec3 &position, const vec3 &color, const vec2 &size)
        : Light(position, color),
          areaSize(size),
          normal(vec3(0.0, -1.0, 0.0)),
          p0(position - vec3(size[0], 0, size[1]) / 2),
          geoms(makeParallelogram(position - vec3(size[0], 0, size[1]) / 2,
                                  vec3(size[0], 0, 0), vec3(0, 0, size[1]),
                                  vec3(0, -1, 0), nullptr)) {
    s0_len = s0.norm();
    s1_len = s1.norm();
    this->s0 = s0.normalized();
    this->s1 = s1.normalized();
}

vec3 AreaLight::emission(vec3 pos, vec3 dir) {
    Eigen::Vector3f res = std::max(dir.dot(normal), 0.0f) * getRadiance();
    return res;
}

Float AreaLight::pdf(const Interaction &ref_it, vec3 pos) {
    float cos_var = abs(ref_it.wi.dot(normal));
    float distance = (pos - ref_it.entryPoint).norm();
    return cos_var / distance / distance;
}

vec3 AreaLight::sample(Interaction &refIt, Float *pdf) {
    float e1 = unif(0, 1, 1)[0];
    float e2 = unif(0, 1, 1)[0];

    vec3f sample_pos
            = position + (e1 * areaSize[0] * s0) + (e2 * areaSize[1] * s1);
    refIt.wi = (sample_pos - refIt.entryPoint).normalized();
    if (areaSize[0] * areaSize[1] != 0)
        *pdf = 1 / (areaSize[0] * areaSize[1]);
    return sample_pos;
}

bool AreaLight::intersect(Interaction &interaction, const Ray &ray) const {
    bool intersection = false;
    for (auto &i: geoms)
        intersection = intersection || i->intersect(interaction, ray);
    interaction.type = Interaction::Type::LIGHT;
    return intersection;
}

std::shared_ptr<Light> makeAreaLight(const vec3 &position, const vec3 &color,
                                     const vec2 &size) {
    return std::make_shared<AreaLight>(position, color, size);
}