#include <material.h>
#include <interaction.h>

/**
 * ColorMaterial class
 */

ConstantMaterial::ConstantMaterial()
    : diffuse(vec3(1.0, 1.0, 1.0)), specular(vec3(0, 0, 0)), shininess(0.0) {}

ConstantMaterial::ConstantMaterial(vec3 diff, vec3 spec, vec3 amb, Float sh)
    : diffuse(diff), specular(spec), ambient(amb), shininess(sh) {}

ConstantMaterial::ConstantMaterial(vec3 rgb, Float sh = 16.0f)
    : diffuse(rgb), specular(rgb), ambient(rgb / 10), shininess(sh) {}

InteractionPhongModel ConstantMaterial::evaluate(const Interaction &in) const {
  InteractionPhongModel m;
  m.diffusion = diffuse;
  m.specular = specular;
  m.ambient = ambient;
  m.shininess = shininess;

  return m;
}

std::shared_ptr<ConstantMaterial> makeConstantMaterial(const vec3 &rgb,
                                                       Float shininess) {
  return std::make_shared<ConstantMaterial>(rgb, shininess);
}
