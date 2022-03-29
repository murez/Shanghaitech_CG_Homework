#ifndef CS171_HW3_INCLUDE_MATERIAL_H_
#define CS171_HW3_INCLUDE_MATERIAL_H_
#include <interaction.h>

/**
 * Base class of materials
 */
class Material {
 public:
  Material() = default;
  virtual ~Material() = default;
  /**
   * evaluate phong model
   */
  [[nodiscard]] virtual InteractionPhongModel evaluate(
      const Interaction &in) const = 0;
};

/**
 * Constant color-based materials
 */
class ConstantMaterial : public Material {
 protected:
  vec3 diffuse;
  vec3 specular;
  vec3 ambient;
  Float shininess;

 public:
  ConstantMaterial();
  ConstantMaterial(vec3 diff, vec3 spec, vec3 amb, Float sh = 16.0f);
  ConstantMaterial(vec3 rgb, Float shininess);
  [[nodiscard]] InteractionPhongModel evaluate(
      const Interaction &in) const override;
};

// optional
class TextureMaterial : public Material {
  // optional TODO: finish this
};

std::shared_ptr<ConstantMaterial> makeConstantMaterial(const vec3 &rgb,
                                                       Float shininess);
#endif  // CS171_HW3_INCLUDE_MATERIAL_H_
