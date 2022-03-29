#include <integrator.h>
#include <material.h>
#include <light.h>

/**
 * Integrator class
 */
Integrator::Integrator(std::shared_ptr<Camera> camera) : camera(camera) {}

/**
 * PhongLightingIntegrator class
 */
PhongLightingIntegrator::PhongLightingIntegrator(std::shared_ptr<Camera> camera)
    : Integrator(camera) {}

/**
 * render a scene
 * @param[in] the given scene
 */
void PhongLightingIntegrator::render(Scene &scene) {
  int now = 0;

  int num_sample = 2;

  vec2f film_size = camera->getFilmSize();
  vec2f pixel_size(film_size.x() / camera->getFilm().resolution.x(),
                   film_size.y() / camera->getFilm().resolution.y());
#pragma omp parallel for schedule(guided, 2) default(none) shared(now, pixel_size, num_sample, scene, film_size)
  for (int dx = 0; dx < camera->getFilm().resolution.x(); ++dx) {
#pragma omp atomic
    ++now;
    printf("\r%.02f%%", now * 100.0 / camera->getFilm().resolution.x());
    for (int dy = 0; dy < camera->getFilm().resolution.y(); ++dy) {
      vec3 L(0, 0, 0);

      vec2 pixel_pos(float(-film_size.x()) / 2 + dx * pixel_size.x(),
                     float(-film_size.y()) / 2 + dy * pixel_size.y());
      vec2 center_pos(pixel_pos.x() + pixel_size.x() / 2,
                      pixel_pos.y() + pixel_size.y() / 2);

      vec3f pixel_radiance(0, 0, 0);

      vec2f rotate_col1 = vec2f(2, -1).normalized();
      vec2f rotate_col2 = vec2f(1, 2).normalized();

      Eigen::Matrix<float, 2, 2> rotate_matrix;
      rotate_matrix.col(0) << rotate_col1.x(), rotate_col1.y();
      rotate_matrix.col(1) << rotate_col2.x(), rotate_col2.y();

      for (int i = 0; i < num_sample; i++) {
        for (int j = 0; j < num_sample; j++) {
          vec2f beam_origin(
              pixel_pos.x() + pixel_size.x() * (i + 0.5f) / num_sample,
              pixel_pos.y() + pixel_size.y() * (j + 0.5f) / num_sample);

           vec2f offset = beam_origin - center_pos;
           beam_origin = rotate_matrix * offset + center_pos;

          Ray beam = camera->generateRay(beam_origin.x(), beam_origin.y());
          Interaction interaction;
          scene.intersect(beam, interaction);
          auto radian = radiance(scene, interaction, beam);
          pixel_radiance += radian;
        }
      }
      L = pixel_radiance / (num_sample * num_sample);

      // TODO: generate camera ray & calculate radiance

      camera->setPixel(dx, dy, L);
    }
  }
}

/**
 * according to phong lighting model
 * calculate the radiance with given scene, ray and interaction
 * @param[in] scene the given scene
 * @param[in] interaction the given interaction
 * @param[in] the given ray
 */
vec3 PhongLightingIntegrator::radiance(Scene &scene,
                                       const Interaction &interaction,
                                       const Ray &ray) const {
  if (interaction.type == Interaction::Type::NONE) {
    return scene.getAmbientLight();
  } else if (interaction.type == Interaction::Type::LIGHT) {
    return scene.getLight()->getColor();
  }

  vec3 amb_comp = (scene.getAmbientLight().array() * interaction.lightingModel.diffusion.array()).matrix();
  Eigen::Array3f diff(0,0,0);
  Eigen::Array3f spec(0,0,0);
  std::vector<LightSamplePair> light_sample_pairs = scene.getLight()->samples();
  for (auto ls : light_sample_pairs){
    float diff_factor = 0;
    float spec_factor = 0;
    Eigen::Vector3f light_dir = (ls.first - interaction.entryPoint).normalized();
    Ray sur_to_light(interaction.entryPoint + 1e-3*(ls.first - interaction.entryPoint), ls.first - interaction.entryPoint, 0.01f, light_dir.norm()-0.0001f);
    // if (!scene.isShadowed(sur_to_light)){
    Interaction light_interaction;
    scene.intersect(sur_to_light, light_interaction);
    if(light_interaction.type == Interaction::Type::LIGHT){ 
      diff_factor = std::max(0.0f,interaction.normal.dot(light_dir));
      vec3f reflect = (2 * interaction.normal.dot(light_dir) * interaction.normal - light_dir).normalized();
      spec_factor = powf(std::max(reflect.dot(-ray.direction), 0.0f), interaction.lightingModel.shininess);
      diff += diff_factor * ls.second.array() * interaction.lightingModel.diffusion.array();
      spec += spec_factor * ls.second.array() * interaction.lightingModel.diffusion.array();
    }
  }
  return diff.matrix() + spec.matrix() + amb_comp;
  // !!!DELETE THIS WHEN FINISHED
  //  UNIMPLEMENTED;
  //  return vec3::Zero();
}

std::shared_ptr<Integrator> makePhongLightingIntegrator(
    std::shared_ptr<Camera> camera) {
  return std::make_shared<PhongLightingIntegrator>(camera);
}
