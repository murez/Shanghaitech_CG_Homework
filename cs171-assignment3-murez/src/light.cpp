#include <light.h>

/**
 * Light class
 */
Light::Light(const vec3 &position, const vec3 &color)
    : position(position), color(color) {}

vec3 Light::getPosition() const { return position; }

vec3 Light::getColor() const { return color; }

/**
 * AreaLight class
 */
AreaLight::AreaLight(const vec3 &position, const vec3 &color, const vec2 &size)
    : Light(position, color),
      areaSize(size),
      geoms(makeParallelogram(position - vec3(size[0], 0, size[1]) / 2,
                              vec3(size[0], 0, 0), vec3(0, 0, size[1]),
                              vec3(0, -1, 0), nullptr)) {}

std::vector<LightSamplePair> AreaLight::samples() const {
  std::vector<LightSamplePair> light_samples;
  int sample_size = 10;
  light_samples.resize(sample_size * sample_size);
  vec3 p0 = position - vec3(areaSize[0], 0, areaSize[1]) / 2;
  for (auto i = 0; i < sample_size; i++) {
    for (auto j = 0; j < sample_size; j++) {
      auto index = j * sample_size + i;
      light_samples[index].first =
          p0 + vec3(areaSize[0], 0, 0) * (i + 0.5) / sample_size +
          vec3(0, 0, areaSize[1]) * (j + 0.5) / sample_size;
      light_samples[index].second = 1.0 / (sample_size * sample_size) * color;
    }
  }
  // TODO: generate rectangle light samples
  // !!!DELETE THIS WHEN FINISHED
  // UNIMPLEMENTED;
  // std::cout<<light_samples[0].first<<std::endl;
  return light_samples;
}

bool AreaLight::intersect(Interaction &interaction, const Ray &ray) const {
  bool intersection = false;
  for (auto &i : geoms)
    intersection = intersection || i->intersect(interaction, ray);
  interaction.type = Interaction::Type::LIGHT;
  return intersection;
}

std::shared_ptr<Light> makeAreaLight(const vec3 &position, const vec3 &color,
                                     const vec2 &size) {
  return std::make_shared<AreaLight>(position, color, size);
}